/**
 * @file video.cc
 * @brief 视频处理模块实现 —— 四线程视频流水线 + 独立录像线程
 *
 * 线程职责与性能指标（1920x1080，2026-09-23 实测）：
 * ┌───────────┬──────────────────────────────────────────┬──────────┐
 * │ 线程       │ 工作内容                                  │ 目标帧率 │
 * ├───────────┼──────────────────────────────────────────┼──────────┤
 * │ Thread1   │ VI → memcpy(NV12→DMA)                    │ 30fps    │
 * │ captureLoop│                                         │          │
 * ├───────────┼──────────────────────────────────────────┼──────────┤
 * │ Thread2   │ RGA NV12→RGB(约4–6ms) + 三缓冲轮换        │ 30fps    │
 * │ bgrConvert│ [替代 CPU cvtColor ~32ms]                 │          │
 * ├───────────┼──────────────────────────────────────────┼──────────┤
 * │ Thread3   │ letterbox+拷贝(约5–12ms) + RKNN(约117ms) │ ≈7.5fps  │
 * │ inference │ 每(INFERENCE_FRAME_SKIP+1)帧推理1次       │          │
 * ├───────────┼──────────────────────────────────────────┼──────────┤
 * │ Thread4   │ OpenCV绘制 + VENC H.264                  │ 26–30fps │
 * │ encodeLoop│ → RTSP + Recording队列                   │          │
 * ├───────────┼──────────────────────────────────────────┼──────────┤
 * │ Recording │ H.264→分片MP4→SD卡，同步/清理/断电恢复   │ 异步     │
 * └───────────┴──────────────────────────────────────────┴──────────┘
 *
 * RGA 使用约束（RV1106 RGA2）：
 *  - 必须使用 DMA CMA 物理连续内存（/dev/rk_dma_heap/rk-dma-heap-cma）
 *  - 必须使用 wrapbuffer_fd(fd, ...) 封装缓冲，不支持 wrapbuffer_virtualaddr
 *  - 所有 YUV/RGB/letterbox 缓冲均在构造函数 / initRgaBuffers() 中预分配
 *
 * 图像格式约定（贯穿整个流水线）：
 *  - captureLoop 输出：NV12（YCbCr 4:2:0 Semi-Planar）
 *  - bgrConvertLoop 输出：RGB888（R-G-B 字节序，RK_FORMAT_RGB_888）
 *  - VENC 输入格式：RK_FMT_RGB888 ✅（与 RGA 输出一致）
 *  - RKNN 推理输入：RGB NHWC UINT8 ✅（与 RGA 输出一致）
 *  - OpenCV Scalar：以 RGB 通道顺序解读（Scalar(R,G,B)）
 */
#include "video.h"
#include "control.h"

// 系统库
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/stat.h>
#include <sstream>
#include <iomanip>

namespace {
int align16(int value) {
    return (value + 15) & ~15;
}
}

// 网络库
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>

// ==================== 构造 / 析构 ====================

/**
 * @brief 构造函数：初始化所有成员并预分配 DMA 缓冲区
 *
 * DMA 分配说明：
 *  所有 RGA 操作的内存必须是物理连续的 DMA CMA 内存。
 *  优先使用 RV1106_CMA_HEAP_PATH（/dev/rk_dma_heap/rk-dma-heap-cma），
 *  失败时回退到 DMA_HEAP_PATH（/dev/dma_heap/system）。
 *
 *  分配的缓冲区：
 *   1. capture_buffer_.dma  ← NV12，width×height×3/2 字节
 *      captureLoop 通过 va 写入（memcpy），bgrConvertLoop 通过 fd 交给 RGA 读取
 *   2. bgr_buffer_.dma_write/ready/read ← RGB888，width×height×3 字节 ×3
 *      bgrConvertLoop 通过 fd 交给 RGA 写入，encodeLoop/inferenceLoop 通过 va 读取
 *   3. letterbox_dma_ ← 在 initRgaBuffers() 中分配（init() 阶段）
 *
 *  cv::Mat 封装：
 *   bgr_buffer_ 的三个 Mat 头 data 指针直接指向对应 DMA 的 va，
 *   Mat 析构时不会 free data（data 生命周期由 dma_buf_free 管理）。
 */
Video::Video(int width, int height, int model_width, int model_height)
    : width_(width), height_(height),
      vir_width_(align16(width)), vir_height_(align16(height)),
      model_width_(model_width), model_height_(model_height),
      H264_TimeRef_(0), 
      g_rtsplive_(NULL), g_rtsp_session_(NULL), 
      recording_(width, height),
      running_(false), resources_released_(false),
      ai_enable_(false), area_enable_(false), obj_enable_(false), 
      control_(nullptr), 
      last_send_time_(0), send_interval_(DEFAULT_SEND_INTERVAL),
#if defined(ENABLE_FPS_CONSOLE) || defined(ENABLE_FPS_DISPLAY)
      frame_count_(0), current_fps_(0.0f),
#endif
      thread_capture_(0), thread_bgr_convert_(0), 
      thread_inference_(0), thread_encode_(0),
      inference_frame_skip_(INFERENCE_FRAME_SKIP),
      has_detection_result_(false),
      letterbox_dma_() {
    
    // 初始化上下文和时间戳
    memset(&rknn_app_ctx_, 0, sizeof(rknn_app_context_t));
    memset(&stFrame_, 0, sizeof(stFrame_));
    memset(&stViFrame_, 0, sizeof(stViFrame_));
    memset(&h264_frame_, 0, sizeof(h264_frame_));
    src_Pool_ = MB_INVALID_POOLID;
    src_Blk_ = MB_INVALID_HANDLE;
    data_ = nullptr;
#if defined(ENABLE_FPS_CONSOLE) || defined(ENABLE_FPS_DISPLAY)
    memset(&fps_start_time_, 0, sizeof(struct timeval));
#endif

    // ===== DMA 内存分配（RGA 硬件加速必须使用 DMA buf）=====
    // 分配 YUV 缓冲区（采集→BGR转换，NV12格式）
    const size_t yuv_size = static_cast<size_t>(vir_width_) * vir_height_ * 3 / 2;
    capture_buffer_.dma.size = yuv_size;
    if (dma_buf_alloc(RV1106_CMA_HEAP_PATH, yuv_size,
                      &capture_buffer_.dma.fd,
                      &capture_buffer_.dma.va) < 0) {
        // RV1106 CMA路径失败时回退到通用 system heap
        dma_buf_alloc(DMA_HEAP_PATH, yuv_size,
                      &capture_buffer_.dma.fd,
                      &capture_buffer_.dma.va);
    }
    capture_buffer_.ready = false;
    pthread_mutex_init(&capture_buffer_.mutex, NULL);
    pthread_cond_init(&capture_buffer_.cond, NULL);

    // 分配 BGR 三缓冲区（BGR转换→编码，DMA内存，RGA直接写入）
    const size_t bgr_size = static_cast<size_t>(vir_width_) * vir_height_ * 3;
    auto alloc_bgr_dma = [&](DmaBuf& dma) {
        dma.size = bgr_size;
        if (dma_buf_alloc(RV1106_CMA_HEAP_PATH, bgr_size, &dma.fd, &dma.va) < 0)
            dma_buf_alloc(DMA_HEAP_PATH, bgr_size, &dma.fd, &dma.va);
    };
    alloc_bgr_dma(bgr_buffer_.dma_write);
    alloc_bgr_dma(bgr_buffer_.dma_ready);
    alloc_bgr_dma(bgr_buffer_.dma_read);
    // 封装成 cv::Mat（data 指向 DMA va，CPU可直接读写）
    const size_t rgb_step = static_cast<size_t>(vir_width_) * 3;
    bgr_buffer_.bgr_write = new cv::Mat(height_, width_, CV_8UC3,
                                        bgr_buffer_.dma_write.va, rgb_step);
    bgr_buffer_.bgr_ready = new cv::Mat(height_, width_, CV_8UC3,
                                        bgr_buffer_.dma_ready.va, rgb_step);
    bgr_buffer_.bgr_read  = new cv::Mat(height_, width_, CV_8UC3,
                                        bgr_buffer_.dma_read.va, rgb_step);
    bgr_buffer_.ready = false;
    bgr_buffer_.frame_index = 0;
    bgr_buffer_.timestamp = 0;
    pthread_mutex_init(&bgr_buffer_.mutex, NULL);
    pthread_cond_init(&bgr_buffer_.cond, NULL);
    
    // 推理专用BGR三缓冲区：BGR转换→推理（浅拷贝 Mat 头，共享 bgr_buffer_ DMA数据）
    inference_bgr_buffer_.bgr_write = new cv::Mat();
    inference_bgr_buffer_.bgr_ready = new cv::Mat();
    inference_bgr_buffer_.bgr_read  = new cv::Mat();
    inference_bgr_buffer_.ready = false; 
    inference_bgr_buffer_.frame_index = 0;
    pthread_mutex_init(&inference_bgr_buffer_.mutex, NULL);
    
    // 检测结果互斥锁
    pthread_mutex_init(&detection_mutex_, NULL);

    performance_ai_on_file_ = nullptr;
    performance_ai_off_file_ = nullptr;
    pthread_mutex_init(&performance_log_mutex_, nullptr);
    const char* configured_log_dir = getenv("IRC_PERFORMANCE_LOG_DIR");
    if (configured_log_dir && configured_log_dir[0] == '/') {
        performance_log_directory_ = configured_log_dir;
    } else {
        const char* record_root = getenv("IRC_RECORD_ROOT");
        if (!record_root) record_root = getenv("IRC_RECORD_MOUNT");
        performance_log_directory_ = std::string(record_root ? record_root : "/root/record") +
                                     "/logs";
    }
    const time_t session_seconds = time(nullptr) + 8 * 60 * 60;
    struct tm session_time;
    char session_name[32] = "unknown_time";
    if (gmtime_r(&session_seconds, &session_time))
        strftime(session_name, sizeof(session_name), "%Y-%m-%d-%H-%M-%S", &session_time);
    performance_session_name_ = session_name;
    recording_.setPerformanceLogCallback(recordingPerformanceLog, this);
}

void Video::recordingPerformanceLog(bool ai_enabled, const char* message,
                                    void* user_data) {
    Video* video = static_cast<Video*>(user_data);
    if (video) video->performanceLog(ai_enabled, "%s", message ? message : "");
}

FILE* Video::openPerformanceLog(bool ai_enabled) {
#ifdef ENABLE_PERFORMANCE_FILE
    FILE*& file = ai_enabled ? performance_ai_on_file_ : performance_ai_off_file_;
    if (file) return file;
    const size_t slash = performance_log_directory_.find_last_of('/');
    if (slash != std::string::npos && slash > 0) {
        const std::string parent = performance_log_directory_.substr(0, slash);
        if (mkdir(parent.c_str(), 0755) != 0 && errno != EEXIST) return nullptr;
    }
    if (mkdir(performance_log_directory_.c_str(), 0755) != 0 && errno != EEXIST)
        return nullptr;
    const std::string path = performance_log_directory_ + "/performance_" +
                             performance_session_name_ +
                             (ai_enabled ? "_AI_ON.txt" : "_AI_OFF.txt");
    file = fopen(path.c_str(), "a");
    if (file) {
        fprintf(file, "# session=%s mode=%s resolution=%dx%d pid=%d\n",
                performance_session_name_.c_str(), ai_enabled ? "AI_ON" : "AI_OFF",
                width_, height_, static_cast<int>(getpid()));
        fflush(file);
        printf("[性能日志] %s\n", path.c_str());
    } else {
        fprintf(stderr, "[性能日志] 无法打开 %s: %s\n", path.c_str(), strerror(errno));
    }
    return file;
#else
    (void)ai_enabled;
    return nullptr;
#endif
}

void Video::performanceLog(bool ai_enabled, const char* format, ...) {
#ifdef ENABLE_PERFORMANCE_TIMING
    char message[512];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    pthread_mutex_lock(&performance_log_mutex_);
#ifdef ENABLE_PERFORMANCE_CONSOLE
    printf("%s\n", message);
    fflush(stdout);
#endif
#ifdef ENABLE_PERFORMANCE_FILE
    FILE* file = openPerformanceLog(ai_enabled);
    if (file) {
        struct timeval now;
        gettimeofday(&now, nullptr);
        const time_t beijing_seconds = now.tv_sec + 8 * 60 * 60;
        struct tm beijing_time;
        char timestamp[40] = "unknown_time";
        if (gmtime_r(&beijing_seconds, &beijing_time)) {
            const size_t length = strftime(timestamp, sizeof(timestamp),
                                           "%Y-%m-%d %H:%M:%S", &beijing_time);
            if (length && length + 5 < sizeof(timestamp))
                snprintf(timestamp + length, sizeof(timestamp) - length,
                         ".%03ld", static_cast<long>(now.tv_usec / 1000));
        }
        fprintf(file, "[%s][%s] %s\n", timestamp,
                ai_enabled ? "AI_ON" : "AI_OFF", message);
        fflush(file);
    }
#endif
    pthread_mutex_unlock(&performance_log_mutex_);
#else
    (void)ai_enabled;
    (void)format;
#endif
}

void Video::closePerformanceLogs() {
    pthread_mutex_lock(&performance_log_mutex_);
    if (performance_ai_on_file_) {
        fclose(performance_ai_on_file_);
        performance_ai_on_file_ = nullptr;
    }
    if (performance_ai_off_file_) {
        fclose(performance_ai_off_file_);
        performance_ai_off_file_ = nullptr;
    }
    pthread_mutex_unlock(&performance_log_mutex_);
}

/**
 * @brief 析构函数：stop() 后按顺序释放所有资源
 *
 * 释放顺序：
 *  1. stop()：等待线程退出，释放硬件资源
 *  2. pthread_mutex_destroy：销毁各缓冲区互斥锁
 *  3. delete cv::Mat*：仅释放 Mat 头，不触碰 data（data 由 DMA 管理）
 *  4. dma_buf_free：释放 YUV、RGB 三缓冲、letterbox 共 5 块 DMA 内存
 *
 * ⚠️ 注意：bgr_buffer_ 三缓冲 dma 的顺序（write/ready/read）在运行时会随
 *          指针轮换而改变，析构时直接按当前字段名释放即可，无需关心顺序。
 */
Video::~Video() {
    stop();
    closePerformanceLogs();
    pthread_mutex_destroy(&performance_log_mutex_);
    
    // 销毁互斥锁
    pthread_mutex_destroy(&capture_buffer_.mutex);
    pthread_cond_destroy(&capture_buffer_.cond);
    pthread_mutex_destroy(&inference_bgr_buffer_.mutex);
    pthread_mutex_destroy(&detection_mutex_);
    pthread_mutex_destroy(&bgr_buffer_.mutex);
    pthread_cond_destroy(&bgr_buffer_.cond);
    
    // 释放 BGR Mat 头（data 由 DMA 管理，不能 delete data）
    delete bgr_buffer_.bgr_write;
    delete bgr_buffer_.bgr_ready;
    delete bgr_buffer_.bgr_read;
    delete inference_bgr_buffer_.bgr_write;
    delete inference_bgr_buffer_.bgr_ready;
    delete inference_bgr_buffer_.bgr_read;

    // 释放 DMA 缓冲区（YUV + BGR三缓冲 + letterbox）
    if (capture_buffer_.dma.fd >= 0)
        dma_buf_free(capture_buffer_.dma.size, &capture_buffer_.dma.fd, capture_buffer_.dma.va);
    if (bgr_buffer_.dma_write.fd >= 0)
        dma_buf_free(bgr_buffer_.dma_write.size, &bgr_buffer_.dma_write.fd, bgr_buffer_.dma_write.va);
    if (bgr_buffer_.dma_ready.fd >= 0)
        dma_buf_free(bgr_buffer_.dma_ready.size, &bgr_buffer_.dma_ready.fd, bgr_buffer_.dma_ready.va);
    if (bgr_buffer_.dma_read.fd >= 0)
        dma_buf_free(bgr_buffer_.dma_read.size, &bgr_buffer_.dma_read.fd, bgr_buffer_.dma_read.va);
    if (letterbox_dma_.fd >= 0)
        dma_buf_free(letterbox_dma_.size, &letterbox_dma_.fd, letterbox_dma_.va);
}

// ==================== 生命周期管理 ====================

/**
 * @brief 初始化所有硬件资源
 * @return true=成功, false=失败
 * 
 * 初始化顺序：
 * 1. RKNN模型加载
 * 2. 编码帧结构和内存池
 * 3. ISP（图像信号处理器）
 * 4. 系统初始化
 * 5. RTSP服务器
 * 6. VI（视频输入）
 * 7. VENC（视频编码）
 */
bool Video::init() {
    // 1. 停止可能占用资源的进程
    system("RkLunch-stop.sh");
    
    if (width_ <= 0 || height_ <= 0 || (width_ & 1) || (height_ & 1)) {
        fprintf(stderr, "[Video] 无效分辨率 %dx%d：NV12宽高必须为正偶数\n",
                width_, height_);
        return false;
    }
    if (capture_buffer_.dma.fd < 0 || !capture_buffer_.dma.va ||
        bgr_buffer_.dma_write.fd < 0 || !bgr_buffer_.dma_write.va ||
        bgr_buffer_.dma_ready.fd < 0 || !bgr_buffer_.dma_ready.va ||
        bgr_buffer_.dma_read.fd < 0 || !bgr_buffer_.dma_read.va) {
        fprintf(stderr, "[Video] DMA缓冲分配失败，分辨率 %dx%d，VI/RGA步长 %dx%d\n",
                width_, height_, vir_width_, vir_height_);
        return false;
    }
    printf("[Video] 分辨率: 编码 %dx%d, VI/RGA内部步长 %dx%d\n",
           width_, height_, vir_width_, vir_height_);

    // 2. 加载YOLO模型
    const char *model_path = "./model/yolov5.rknn";
    if (init_yolov5_model(model_path, &rknn_app_ctx_) != 0) {
        printf("[Video] ❌ RKNN模型加载失败\n");
        return false;
    }
    printf("[Video] ✅ RKNN模型加载成功\n");
    init_post_process();

    // 3. 创建编码帧结构体和内存池
    stFrame_.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S));
    
    MB_POOL_CONFIG_S PoolCfg;
    memset(&PoolCfg, 0, sizeof(MB_POOL_CONFIG_S));
    // VENC 的 RGB888 输入按有效尺寸紧凑存放。RV1106 的 RK_MPI VENC
    // 不接受将 1920x1080 的 u32VirHeight 手动改成 1088。
    const size_t venc_frame_size = static_cast<size_t>(width_) * height_ * 3;
    PoolCfg.u64MBSize = venc_frame_size;
    PoolCfg.u32MBCnt = 1;
    PoolCfg.enAllocType = MB_ALLOC_TYPE_DMA;
    
    src_Pool_ = RK_MPI_MB_CreatePool(&PoolCfg);
    if (src_Pool_ == MB_INVALID_POOLID) {
        fprintf(stderr, "[Video] VENC输入内存池创建失败\n");
        return false;
    }
    src_Blk_ = RK_MPI_MB_GetMB(src_Pool_, venc_frame_size, RK_TRUE);
    if (src_Blk_ == MB_INVALID_HANDLE) {
        fprintf(stderr, "[Video] VENC输入内存分配失败: %zu bytes\n", venc_frame_size);
        return false;
    }
    
    h264_frame_.stVFrame.u32Width = width_;
    h264_frame_.stVFrame.u32Height = height_;
    h264_frame_.stVFrame.u32VirWidth = width_;
    h264_frame_.stVFrame.u32VirHeight = height_;
    h264_frame_.stVFrame.enPixelFormat = RK_FMT_RGB888;
    h264_frame_.stVFrame.u32FrameFlag = 160;
    h264_frame_.stVFrame.pMbBlk = src_Blk_;
    
    data_ = (unsigned char *)RK_MPI_MB_Handle2VirAddr(src_Blk_);
    if (!data_) {
        fprintf(stderr, "[Video] VENC输入内存映射失败\n");
        return false;
    }
    memset(data_, 0, venc_frame_size);
    frame_ = cv::Mat(height_, width_, CV_8UC3, data_);
    printf("[Video] ✅ 编码帧结构初始化成功\n");

    // 4. ISP初始化
    RK_BOOL multi_sensor = RK_FALSE;
    const char *iq_dir = "/etc/iqfiles";
    rk_aiq_working_mode_t hdr_mode = RK_AIQ_WORKING_MODE_NORMAL;
    SAMPLE_COMM_ISP_Init(0, hdr_mode, multi_sensor, iq_dir);
    SAMPLE_COMM_ISP_Run(0);
    printf("[Video] ✅ ISP初始化成功\n");

    // 5. 系统初始化
    if (RK_MPI_SYS_Init() != RK_SUCCESS) {
        printf("[Video] ❌ MPI系统初始化失败\n");
        return false;
    }
    printf("[Video] ✅ MPI系统初始化成功\n");
    
    // 6. RTSP服务器初始化
    g_rtsplive_ = create_rtsp_demo(554);
    if (!g_rtsplive_) {
        fprintf(stderr, "[Video] RTSP 服务创建失败（端口 554）\n");
        return false;
    }
    g_rtsp_session_ = rtsp_new_session(g_rtsplive_, "/live/0");
    if (!g_rtsp_session_) {
        fprintf(stderr, "[Video] RTSP /live/0 会话创建失败\n");
        return false;
    }
    rtsp_set_video(g_rtsp_session_, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
    rtsp_sync_video_ts(g_rtsp_session_, rtsp_get_reltime(), rtsp_get_ntptime());
    printf("[Video] ✅ RTSP服务器初始化成功（端口554）\n");
    
    // 7. VI（视频输入）初始化
    if (vi_dev_init() != RK_SUCCESS) {
        fprintf(stderr, "[Video] ❌ VI设备初始化失败\n");
        return false;
    }
    if (vi_chn_init(0, width_, height_) != RK_SUCCESS) {
        fprintf(stderr, "[Video] ❌ VI视频输入初始化失败\n");
        return false;
    }
    printf("[Video] ✅ VI视频输入初始化成功\n");
    
    // 8. VENC（视频编码）初始化
    RK_CODEC_ID_E enCodecType = RK_VIDEO_ID_AVC;
    if (venc_init(0, width_, height_, enCodecType) != RK_SUCCESS) {
        fprintf(stderr, "[Video] ❌ VENC视频编码初始化失败\n");
        return false;
    }
    printf("[Video] ✅ VENC视频编码初始化成功\n");
    
    // 9. RGA缓冲区注册（硬件加速YUV→BGR使用）
    if (!initRgaBuffers()) {
        printf("[Video] ❌ RGA缓冲区初始化失败\n");
        return false;
    }
    printf("[Video] ✅ RGA缓冲区初始化成功\n");
    
    return true;
}

/**
 * @brief 启动四线程
 * @return true=成功, false=失败
 */
bool Video::start() {
    recording_.start();  // SD 未挂载时只关闭录像，不影响直播
    running_ = true;
    
    // 创建采集线程
    if (pthread_create(&thread_capture_, NULL, captureThreadFunc, this) != 0) {
        printf("[Video] ❌ 采集线程创建失败\n");
        return false;
    }
    
    // 创建BGR转换线程
    if (pthread_create(&thread_bgr_convert_, NULL, bgrConvertThreadFunc, this) != 0) {
        printf("[Video] ❌ BGR转换线程创建失败\n");
        return false;
    }
    
    // 创建推理线程
    if (pthread_create(&thread_inference_, NULL, inferenceThreadFunc, this) != 0) {
        printf("[Video] ❌ 推理线程创建失败\n");
        return false;
    }
    
    // 创建编码线程
    if (pthread_create(&thread_encode_, NULL, encodeThreadFunc, this) != 0) {
        printf("[Video] ❌ 编码线程创建失败\n");
        return false;
    }
    
    printf("[Video] ✅ 四线程启动成功（采集/BGR转换/推理/编码）\n");
    return true;
}

/**
 * @brief 停止所有线程并释放硬件资源
 */
void Video::stop() {
    if (resources_released_) return;
    resources_released_ = true;
    running_ = false;

    // 唤醒可能阻塞在条件变量上的线程，确保可及时退出
    pthread_mutex_lock(&capture_buffer_.mutex);
    pthread_cond_broadcast(&capture_buffer_.cond);
    pthread_mutex_unlock(&capture_buffer_.mutex);

    pthread_mutex_lock(&bgr_buffer_.mutex);
    pthread_cond_broadcast(&bgr_buffer_.cond);
    pthread_mutex_unlock(&bgr_buffer_.mutex);
    
    // 等待所有线程退出
    if (thread_capture_) {
        pthread_join(thread_capture_, NULL);
        thread_capture_ = 0;
        printf("[Video] 采集线程已停止\n");
    }
    if (thread_bgr_convert_) {
        pthread_join(thread_bgr_convert_, NULL);
        thread_bgr_convert_ = 0;
        printf("[Video] BGR转换线程已停止\n");
    }
    if (thread_inference_) {
        pthread_join(thread_inference_, NULL);
        thread_inference_ = 0;
        printf("[Video] 推理线程已停止\n");
    }
    if (thread_encode_) {
        pthread_join(thread_encode_, NULL);
        thread_encode_ = 0;
        printf("[Video] 编码线程已停止\n");
    }
    recording_.stop();
    
    // 释放硬件资源
    if (src_Blk_ != MB_INVALID_HANDLE) {
        RK_MPI_MB_ReleaseMB(src_Blk_);
        src_Blk_ = MB_INVALID_HANDLE;
    }
    if (src_Pool_ != MB_INVALID_POOLID) {
        RK_MPI_MB_DestroyPool(src_Pool_);
        src_Pool_ = MB_INVALID_POOLID;
    }
    RK_MPI_VI_DisableChn(0, 0);
    RK_MPI_VI_DisableDev(0);
    SAMPLE_COMM_ISP_Stop(0);
    RK_MPI_VENC_StopRecvFrame(0);
    RK_MPI_VENC_DestroyChn(0);
    
    if (stFrame_.pstPack) {
        free(stFrame_.pstPack);
        stFrame_.pstPack = nullptr;
    }
    if (g_rtsplive_) {
        rtsp_del_demo(g_rtsplive_);
        g_rtsplive_ = nullptr;
        g_rtsp_session_ = nullptr;
    }
    
    // 释放RGA缓冲句柄
    releaseRgaBuffers();
    
    RK_MPI_SYS_Exit();
    release_yolov5_model(&rknn_app_ctx_);
    deinit_post_process();
    
    printf("[Video] ✅ 所有资源已释放\n");
}

// ==================== 视频流控制 ====================

/**
 * @brief 暂停所有线程（节省CPU资源）
 * 
 * 应用场景：不需要视频流时暂停，CPU占用从70%降至5%
 * 注意：硬件资源（ISP/VI/VENC）保持激活状态
 */
void Video::pauseAllThreads() {
    if (!running_) {
        printf("[Video] ⚠️  视频系统已处于停止状态\n");
        return;
    }
    
    printf("[Video] ⏸️  暂停所有线程...\n");
    running_ = false;

    // 唤醒可能阻塞在条件变量上的线程，确保可及时退出
    pthread_mutex_lock(&capture_buffer_.mutex);
    pthread_cond_broadcast(&capture_buffer_.cond);
    pthread_mutex_unlock(&capture_buffer_.mutex);

    pthread_mutex_lock(&bgr_buffer_.mutex);
    pthread_cond_broadcast(&bgr_buffer_.cond);
    pthread_mutex_unlock(&bgr_buffer_.mutex);
    
    // 等待所有线程退出
    if (thread_capture_) {
        pthread_join(thread_capture_, NULL);
        thread_capture_ = 0;
        printf("[Video]    ✅ 采集线程已暂停\n");
    }
    if (thread_bgr_convert_) {
        pthread_join(thread_bgr_convert_, NULL);
        thread_bgr_convert_ = 0;
        printf("[Video]    ✅ BGR转换线程已暂停\n");
    }
    if (thread_inference_) {
        pthread_join(thread_inference_, NULL);
        thread_inference_ = 0;
        printf("[Video]    ✅ 推理线程已暂停\n");
    }
    if (thread_encode_) {
        pthread_join(thread_encode_, NULL);
        thread_encode_ = 0;
        printf("[Video]    ✅ 编码推流线程已暂停\n");
    }
    recording_.stop();  // 完成当前录像片段，暂停期间也可以查看已保存文件
    
    printf("[Video] ✅ 所有线程已暂停（硬件资源保持激活）\n");
}

/**
 * @brief 恢复所有线程
 */
void Video::resumeAllThreads() {
    if (running_) {
        printf("[Video] ⚠️  视频系统已在运行中\n");
        return;
    }
    
    printf("[Video] ▶️  恢复所有线程...\n");
    recording_.start();
    running_ = true;
    
    // 重启4个线程
    if (pthread_create(&thread_capture_, NULL, captureThreadFunc, this) != 0) {
        printf("[Video] ❌ 采集线程创建失败\n");
        running_ = false;
        return;
    }
    printf("[Video]    ✅ 采集线程已启动\n");
    
    if (pthread_create(&thread_bgr_convert_, NULL, bgrConvertThreadFunc, this) != 0) {
        printf("[Video] ❌ BGR转换线程创建失败\n");
        running_ = false;
        return;
    }
    printf("[Video]    ✅ BGR转换线程已启动\n");
    
    if (pthread_create(&thread_inference_, NULL, inferenceThreadFunc, this) != 0) {
        printf("[Video] ❌ 推理线程创建失败\n");
        running_ = false;
        return;
    }
    printf("[Video]    ✅ 推理线程已启动\n");
    
    if (pthread_create(&thread_encode_, NULL, encodeThreadFunc, this) != 0) {
        printf("[Video] ❌ 编码推流线程创建失败\n");
        running_ = false;
        return;
    }
    printf("[Video]    ✅ 编码推流线程已启动\n");
    
    printf("[Video] ✅ 所有线程已恢复运行\n");
}

// ==================== 线程入口函数 ====================
void* Video::captureThreadFunc(void* arg) {
    Video* self = static_cast<Video*>(arg);
    self->captureLoop();
    return nullptr;
}

void* Video::bgrConvertThreadFunc(void* arg) {
    Video* self = static_cast<Video*>(arg);
    self->bgrConvertLoop();
    return nullptr;
}

void* Video::inferenceThreadFunc(void* arg) {
    Video* self = static_cast<Video*>(arg);
    self->inferenceLoop();
    return nullptr;
}

void* Video::encodeThreadFunc(void* arg) {
    Video* self = static_cast<Video*>(arg);
    self->encodeLoop();
    return nullptr;
}

// ============ Thread1：采集循环 ============
// 职责：以 30fps 从 VI 硬件获取 NV12 YUV 帧，memcpy 到 capture_buffer_.dma.va
//
// 关键设计：
//  - 不检查 ready 标志，直接覆盖写入，保证采集实时性
//  - BGR 转换线程消费速度较慢时会丢帧，属于设计预期行为
//
// 1080P 稳定段实测（2026-09-23）：
//  - AI关：VI等待平均27.04ms，分行拷贝平均6.90ms，总计平均33.94ms
//  - AI开：VI等待平均21.08ms，分行拷贝平均14.19ms，总计平均35.27ms
//  - 耗时包含线程调度和内存带宽竞争，不可直接视为纯硬件处理时间
//  - 每30帧记录一次实际耗时，不再使用固定经验值判断性能
void Video::captureLoop() {
    RK_S32 s32Ret;
    int frame_counter = 0;
    
#ifdef ENABLE_PERFORMANCE_TIMING
    struct timeval t_start, t_end;
    long vi_time_us, copy_time_us, total_time_us;
#endif
    
    printf("[采集线程] 启动 - YUV快速拷贝模式（30fps）\n");
    
    while(running_) {
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_start, NULL);
#endif
        
        s32Ret = RK_MPI_VI_GetChnFrame(0, 0, &stViFrame_, -1);
        if(s32Ret != RK_SUCCESS) {
            continue; // 立即重试
        }
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_end, NULL);
        vi_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
        gettimeofday(&t_start, NULL);
#endif
        
        void *vi_data = RK_MPI_MB_Handle2VirAddr(stViFrame_.stVFrame.pMbBlk);
        if (!vi_data) {
            fprintf(stderr, "[采集线程] VI帧地址映射失败\n");
            RK_MPI_VI_ReleaseChnFrame(0, 0, &stViFrame_);
            continue;
        }
        uint64_t timestamp = TEST_COMM_GetNowUs();

        const int src_stride = stViFrame_.stVFrame.u32VirWidth >= static_cast<RK_U32>(width_)
            ? static_cast<int>(stViFrame_.stVFrame.u32VirWidth) : width_;
        const int src_vir_height = stViFrame_.stVFrame.u32VirHeight >= static_cast<RK_U32>(height_)
            ? static_cast<int>(stViFrame_.stVFrame.u32VirHeight) : height_;
        
        // 🔒 分行拷贝YUV到BGR转换缓冲区（1080P实测约7–14ms，加锁保护）
        // 注意：不检查ready标志，直接覆盖，BGR转换线程会丢帧（保证采集30fps实时性）
        pthread_mutex_lock(&capture_buffer_.mutex);
        unsigned char* dst_y = static_cast<unsigned char*>(capture_buffer_.dma.va);
        const unsigned char* src_y = static_cast<const unsigned char*>(vi_data);
        unsigned char* dst_uv = dst_y + static_cast<size_t>(vir_width_) * vir_height_;
        const unsigned char* src_uv = src_y + static_cast<size_t>(src_stride) * src_vir_height;
        for (int row = 0; row < height_; ++row)
            memcpy(dst_y + static_cast<size_t>(row) * vir_width_,
                   src_y + static_cast<size_t>(row) * src_stride, width_);
        for (int row = 0; row < height_ / 2; ++row)
            memcpy(dst_uv + static_cast<size_t>(row) * vir_width_,
                   src_uv + static_cast<size_t>(row) * src_stride, width_);
        capture_buffer_.timestamp = timestamp;
        capture_buffer_.ready = true;
    pthread_cond_signal(&capture_buffer_.cond);  // 通知BGR转换线程有新帧
        pthread_mutex_unlock(&capture_buffer_.mutex);
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_end, NULL);
        copy_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
        total_time_us = vi_time_us + copy_time_us;
        
        // 每30帧打印一次
        if (frame_counter % 30 == 0) {
            performanceLog(ai_enable_,
                           "[采集#%d] VI获取: %ld us, YUV分行拷贝: %ld us, 总计: %ld us",
                           frame_counter, vi_time_us, copy_time_us, total_time_us);
        }
#endif
        
        RK_MPI_VI_ReleaseChnFrame(0, 0, &stViFrame_);
        frame_counter++;
    }
    
    printf("[采集线程] 退出\n");
}

// ============ Thread2：RGB 转换循环 ============
// 职责：将 NV12 YUV 帧转换为 RGB888，输出到 bgr_buffer_ 三缓冲，
//        并按跳帧规则向 inference_bgr_buffer_ 传递推理帧
//
// RGA 加速路径（主路径）：
//  wrapbuffer_fd(capture_buffer_.dma.fd, NV12) →
//  imcvtcolor(BT.601 Full Range) →
//  wrapbuffer_fd(bgr_buffer_.dma_write.fd, RGB888)
//  1080P实测：AI关平均4.07ms；AI开平均4.84ms、P95 6.01ms
//
// CPU 回退路径（RGA 失败时）：
//  cv::cvtColor(capture_buffer_.dma.va, COLOR_YUV420sp2RGB)
//  保证不丢帧，但会增加 CPU 负载
//
// 三缓冲轮换规则（⚠️ DmaBuf 与 cv::Mat* 必须同步交换）：
//  bgrConvertLoop 写完 dma_write → 加锁 → swap(dma_write ↔ dma_ready, bgr_write ↔ bgr_ready)
//  encodeLoop 消费时 → 加锁 → swap(dma_ready ↔ dma_read, bgr_ready ↔ bgr_read)
//  违反同步规则会导致 RGA fd 与 Mat data 指向不同物理内存，引发频闪
void Video::bgrConvertLoop() {
    // RGA版本：直接从capture_buffer_.yuv_data（已注册RGA句柄）转换到bgr_write
    // 使用预分配DMA缓冲，不在正常RGA路径中临时申请图像内存
    int frame_counter = 0;
    
#ifdef ENABLE_PERFORMANCE_TIMING
    struct timeval t_start, t_end;
    long cvt_time_us, swap_time_us, inference_copy_time_us = 0;
#endif
    
    printf("[BGR转换线程] 启动 - RGA硬件加速模式（NV12→RGB）\n");
    
    while(running_) {
        // 🔒 等待YUV数据就绪（条件变量阻塞等待，避免轮询）
        pthread_mutex_lock(&capture_buffer_.mutex);
        while (running_ && !capture_buffer_.ready) {
            pthread_cond_wait(&capture_buffer_.cond, &capture_buffer_.mutex);
        }
        if (!running_) {
            pthread_mutex_unlock(&capture_buffer_.mutex);
            break;
        }
        capture_buffer_.ready = false;  // 标记已消费
        pthread_mutex_unlock(&capture_buffer_.mutex);
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_start, NULL);
#endif
        
        // ============ RGA硬件加速 YUV(NV12) → RGB ============
        // 源：capture_buffer_.dma.fd（DMA buf，wrapbuffer_fd零拷贝封装）
        // 目：bgr_buffer_.dma_write.fd（DMA buf，同上）
        // 输出格式：RK_FORMAT_RGB_888（R-G-B字节序）
        //   → VENC enPixelFormat = RK_FMT_RGB888 ✅
        //   → 推理模型 RKNN NHWC UINT8 默认期望 RGB ✅
        //   → OpenCV Mat CV_8UC3 存 R-G-B（后续 rectangle 颜色参数对应调整）
        // RV1106 RGA2 只支持 DMA fd，不支持虚拟地址，必须用 wrapbuffer_fd
        rga_buffer_t rga_src = wrapbuffer_fd(capture_buffer_.dma.fd,
                                              width_, height_,
                                              RK_FORMAT_YCbCr_420_SP,
                                              vir_width_, vir_height_);
        rga_buffer_t rga_dst = wrapbuffer_fd(bgr_buffer_.dma_write.fd,
                                              width_, height_,
                                              RK_FORMAT_RGB_888,
                                              vir_width_, vir_height_);
        IM_STATUS ret = imcvtcolor(rga_src, rga_dst,
                                   RK_FORMAT_YCbCr_420_SP, RK_FORMAT_RGB_888,
                                   IM_YUV_TO_RGB_BT601_FULL);
        if (ret != IM_STATUS_SUCCESS) {
            // RGA失败时回退到CPU软件转换，保证不丢帧
            printf("[BGR转换] ⚠️ RGA转换失败(%d)，回退CPU cvtColor\n", ret);
            std::vector<unsigned char> compact(static_cast<size_t>(width_) * height_ * 3 / 2);
            const unsigned char* padded = static_cast<const unsigned char*>(capture_buffer_.dma.va);
            const unsigned char* padded_uv = padded + static_cast<size_t>(vir_width_) * vir_height_;
            for (int row = 0; row < height_; ++row)
                memcpy(compact.data() + static_cast<size_t>(row) * width_,
                       padded + static_cast<size_t>(row) * vir_width_, width_);
            unsigned char* compact_uv = compact.data() + static_cast<size_t>(width_) * height_;
            for (int row = 0; row < height_ / 2; ++row)
                memcpy(compact_uv + static_cast<size_t>(row) * width_,
                       padded_uv + static_cast<size_t>(row) * vir_width_, width_);
            cv::Mat yuv_mat(height_ + height_ / 2, width_, CV_8UC1, compact.data());
            // 输出 RGB 保持与 RGA 路径一致
            cv::cvtColor(yuv_mat, *bgr_buffer_.bgr_write, cv::COLOR_YUV420sp2RGB);
        }
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_end, NULL);
        cvt_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
        gettimeofday(&t_start, NULL);
#endif
        
        // 🔒 三缓冲轮换：write→ready（加锁，仅轮换Mat指针 <1us）
        pthread_mutex_lock(&bgr_buffer_.mutex);
        // 轮换 DmaBuf（fd/va 跟随 Mat 指针一起换，保证 RGA fd 与 Mat data 一致）
        DmaBuf   tmp_dma         = bgr_buffer_.dma_ready;
        cv::Mat* tmp_mat         = bgr_buffer_.bgr_ready;
        bgr_buffer_.dma_ready    = bgr_buffer_.dma_write;   // write晋升为ready
        bgr_buffer_.bgr_ready    = bgr_buffer_.bgr_write;
        bgr_buffer_.dma_write    = tmp_dma;                  // 旧ready降为write
        bgr_buffer_.bgr_write    = tmp_mat;
        bgr_buffer_.timestamp    = TEST_COMM_GetNowUs();
        bgr_buffer_.frame_index  = frame_counter;
        bgr_buffer_.ready        = true;
    pthread_cond_signal(&bgr_buffer_.cond);  // 通知编码线程有新帧
        pthread_mutex_unlock(&bgr_buffer_.mutex);
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_end, NULL);
        swap_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
#endif
        
        // 🔒 每N帧传递BGR数据给推理线程（三缓冲指针轮换，零拷贝优化）
        if (ai_enable_ && (frame_counter % (inference_frame_skip_ + 1)) == 0) {
#ifdef ENABLE_PERFORMANCE_TIMING
            gettimeofday(&t_start, NULL);
#endif
            // 浅拷贝：仅复制Mat头（48字节），共享data指针（无锁，安全）
            // 同步记录对应的 DMA fd（bgr_ready 此时对应 dma_ready）
            *inference_bgr_buffer_.bgr_write = *bgr_buffer_.bgr_ready;
            inference_bgr_buffer_.dma_fd_write = bgr_buffer_.dma_ready.fd;
            
            // 三缓冲指针轮换（锁内<1us）
            pthread_mutex_lock(&inference_bgr_buffer_.mutex);
            if (!inference_bgr_buffer_.ready) {  // 避免覆盖未处理的帧
                cv::Mat* tmp      = inference_bgr_buffer_.bgr_ready;
                int      tmp_fd   = inference_bgr_buffer_.dma_fd_ready;
                inference_bgr_buffer_.bgr_ready    = inference_bgr_buffer_.bgr_write;
                inference_bgr_buffer_.dma_fd_ready = inference_bgr_buffer_.dma_fd_write;
                inference_bgr_buffer_.bgr_write    = tmp;
                inference_bgr_buffer_.dma_fd_write = tmp_fd;
                inference_bgr_buffer_.frame_index  = frame_counter;
                inference_bgr_buffer_.ready        = true;
            }
            pthread_mutex_unlock(&inference_bgr_buffer_.mutex);
            
#ifdef ENABLE_PERFORMANCE_TIMING
            gettimeofday(&t_end, NULL);
            inference_copy_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
        } else {
            inference_copy_time_us = 0;
#endif
        }
        
#ifdef ENABLE_PERFORMANCE_TIMING
        if (frame_counter % 30 == 0) {
            long total_us = cvt_time_us + swap_time_us + inference_copy_time_us;
            performanceLog(ai_enable_,
                           "[RGB转换#%d] RGA NV12→RGB: %ld us, 指针轮换: %ld us, "
                           "推理帧提交: %ld us, 总计: %ld us",
                           frame_counter, cvt_time_us, swap_time_us,
                           inference_copy_time_us, total_us);
        }
#endif
        
        frame_counter++;
    }
    
    printf("[BGR转换线程] 退出\n");
}

// ============ Thread3：推理循环 ============
// 职责：对 RGB 帧做 letterbox 缩放后送入 RKNN YOLOv5，将结果写入 shared_detections_
//
// RGA letterbox 路径（主路径）：
//  wrapbuffer_fd(bgr_dma_fd, RGB, 当前采集尺寸) →
//  improcess(缩放到 scaledW×scaledH，写入 letterbox_dma_ 的 (leftPadding_,topPadding_) 偏移) →
//  memcpy(letterbox_dma_.va → rknn_input_mems[0]->virt_addr)
//
//  letterbox 黑边只需在 initRgaBuffers() 时 memset(0) 一次，
//  此后 RGA 每帧只写图像区域，黑边保持不变。
//  1080P实测：letterbox+输入拷贝平均7.88ms，RKNN平均117.33ms，总计平均125.21ms
//
// CPU 回退路径（RGA letterbox 失败时）：
//  letterbox(bgr_mat)：CPU resize(INTER_NEAREST) + memset 黑边 + copyTo
//
// 结果处理：
//  - ROI 过滤（area_enable_）：仅保留中心落在 video_rectInfo 内的框
//  - 类别白名单过滤（obj_enable_）：仅保留 video_objList 内的类别
//  - 通过 detection_mutex_ 将结果写入 shared_detections_（encodeLoop 读取）
//  - 达到 send_interval_ 间隔时，通过 control_->onDetectionSummary() 上报
void Video::inferenceLoop() {
    cv::Mat bgr_mat;
    int sX, sY, eX, eY;
    
#ifdef ENABLE_PERFORMANCE_TIMING
    struct timeval t_start, t_end;
    long swap_time_us, letterbox_time_us, inference_time_us, total_time_us;
    int inference_counter = 0;
#endif
    
    printf("[推理线程] 启动 - 三缓冲指针交换模式（零拷贝优化）\n");
    
    while(running_) {
        if (!ai_enable_) {
            usleep(100000);  // AI关闭时休眠100ms
            continue;
        }
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_start, NULL);
#endif
        
        // 🔒 三缓冲指针交换获取待推理BGR帧（锁内<1us，零拷贝）
        bool has_frame = false;
        int  bgr_dma_fd = -1;
        pthread_mutex_lock(&inference_bgr_buffer_.mutex);
        if (inference_bgr_buffer_.ready) {
            cv::Mat* temp    = inference_bgr_buffer_.bgr_read;
            int      tmp_fd  = inference_bgr_buffer_.dma_fd_read;
            inference_bgr_buffer_.bgr_read    = inference_bgr_buffer_.bgr_ready;
            inference_bgr_buffer_.dma_fd_read = inference_bgr_buffer_.dma_fd_ready;
            inference_bgr_buffer_.bgr_ready    = temp;
            inference_bgr_buffer_.dma_fd_ready = tmp_fd;
            inference_bgr_buffer_.ready = false;
            has_frame = true;
        }
        bgr_dma_fd = inference_bgr_buffer_.dma_fd_read;
        pthread_mutex_unlock(&inference_bgr_buffer_.mutex);
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_end, NULL);
        swap_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
#endif
        
        if (!has_frame) {
            usleep(10000); // 10ms
            continue;
        }
        
        // 使用read缓冲（无锁，推理线程独占）
        bgr_mat = *inference_bgr_buffer_.bgr_read;
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_start, NULL);
#endif
        
        // ============ RGA硬件加速 letterbox（缩放+填充） ============
        // 计算letterbox缩放参数（与原letterbox函数等价）
        float scaleX = (float)model_width_  / (float)width_;
        float scaleY = (float)model_height_ / (float)height_;
        scale_       = scaleX < scaleY ? scaleX : scaleY;
        int scaledW  = (int)((float)width_  * scale_);
        int scaledH  = (int)((float)height_ * scale_);
        leftPadding_ = (model_width_  - scaledW) / 2;
        topPadding_  = (model_height_ - scaledH) / 2;
        
        // RGA缩放：RGB(width_×height_) → RGB(scaledW×scaledH)，写入letterbox_dma_的padding偏移处
        // letterbox_dma_.va已被memset(0)，黑边已就绪，无需每帧重刷
        // RV1106 RGA2 只支持 DMA fd，必须用 wrapbuffer_fd
        // 格式统一为 RK_FORMAT_RGB_888（与 imcvtcolor 输出一致，推理模型期望 RGB NHWC）
        rga_buffer_t rga_lb_src = wrapbuffer_fd(bgr_dma_fd,
                                                  width_, height_,
                                                  RK_FORMAT_RGB_888,
                                                  vir_width_, vir_height_);
        // dst: stride设为model_width_，使RGA直接以完整行宽写入，自然形成padding
        rga_buffer_t rga_lb_dst = wrapbuffer_fd(letterbox_dma_.fd,
                                                  model_width_, model_height_,
                                                  RK_FORMAT_RGB_888);
        // im_rect指定dst写入起始偏移 (leftPadding_, topPadding_)
        im_rect lb_src_rect  = {0, 0, width_, height_};
        im_rect lb_dst_rect  = {leftPadding_, topPadding_, scaledW, scaledH};
        rga_buffer_t rga_pat = {};
        im_rect      rect_empty = {};
        IM_STATUS lb_ret = improcess(rga_lb_src, rga_lb_dst, rga_pat,
                                     lb_src_rect, lb_dst_rect, rect_empty,
                                     IM_SYNC);
        if (lb_ret != IM_STATUS_SUCCESS) {
            // RGA失败时回退CPU letterbox（输出 RGB，与 RGA 路径一致）
            printf("[推理] ⚠️ RGA letterbox失败(%d)，回退CPU\n", lb_ret);
            // bgr_mat 实际存储的是 RGB（RGA imcvtcolor 输出 RK_FORMAT_RGB_888）
            // CPU letterbox 直接 resize + 填充，格式不变
            cv::Mat fallback = letterbox(bgr_mat);
            memcpy(rknn_app_ctx_.input_mems[0]->virt_addr, fallback.data,
                   model_width_ * model_height_ * 3);
        } else {
            // 直接从预分配letterbox_dma_拷贝到RKNN输入（640×640×3连续内存）
            memcpy(rknn_app_ctx_.input_mems[0]->virt_addr, letterbox_dma_.va,
                   model_width_ * model_height_ * 3);
        }
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_end, NULL);
        letterbox_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
        gettimeofday(&t_start, NULL);
#endif
        
        inference_yolov5_model(&rknn_app_ctx_, &od_results_);
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_end, NULL);
        inference_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
        total_time_us = swap_time_us + letterbox_time_us + inference_time_us;
        
        inference_counter++;
        if (inference_counter % 10 == 0) {
            performanceLog(true,
                           "[推理#%d] 指针交换: %ld us, letterbox及输入拷贝: %ld us, "
                           "RKNN推理: %ld us, 总计: %ld us",
                           inference_counter, swap_time_us, letterbox_time_us,
                           inference_time_us, total_time_us);
        }
#endif
        
        // 处理检测结果
        std::vector<DetectionInfo> temp_detections;
        
        for(int i = 0; i < od_results_.count; i++) {
            object_detect_result *det_result = &(od_results_.results[i]);
            sX = (int)(det_result->box.left);
            sY = (int)(det_result->box.top);
            eX = (int)(det_result->box.right);
            eY = (int)(det_result->box.bottom);
            mapCoordinates(&sX, &sY);
            mapCoordinates(&eX, &eY);
            
            bool drawBox = true;
            
            // 对象过滤
            if (obj_enable_ && !video_objList.empty()) {
                bool found = false;
                for (size_t j = 0; j < video_objList.size(); ++j) {
                    if (det_result->cls_id == video_objList[j]) {
                        found = true;
                        break;
                    }
                }
                if (!found) drawBox = false;
            }
            
            // 区域过滤
            if (area_enable_ && drawBox) {
                float rx = video_rectInfo.x;
                float ry = video_rectInfo.y;
                float rw = video_rectInfo.w;
                float rh = video_rectInfo.h;
                
                float left_norm = (float)sX / (float)width_;
                float top_norm = (float)sY / (float)height_;
                float right_norm = (float)eX / (float)width_;
                float bottom_norm = (float)eY / (float)height_;
                
                if (!(left_norm >= rx && right_norm <= rx+rw && 
                      top_norm >= ry && bottom_norm <= ry+rh)) {
                    drawBox = false;
                }
            }
            
            if (drawBox) {
                // printf("%s @ (%d %d %d %d) %.3f\n", 
                //        coco_cls_to_name(det_result->cls_id), sX, sY, eX, eY, det_result->prop);
                
                DetectionInfo detection;
                detection.cls_id = det_result->cls_id;
                detection.cls_name = coco_cls_to_name(det_result->cls_id);
                detection.x = sX;
                detection.y = sY;
                detection.w = eX - sX;
                detection.h = eY - sY;
                detection.confidence = det_result->prop;
                temp_detections.push_back(detection);
            }
        }
        
        // 🔒 更新共享检测结果（加锁保护）
        pthread_mutex_lock(&detection_mutex_);
        shared_detections_.clear();
        shared_detections_.assign(temp_detections.begin(), temp_detections.end());
        pthread_mutex_unlock(&detection_mutex_);
        
        // 发送检测结果
        if (!temp_detections.empty() && control_) {
            static time_t last_time = 0;
            time_t now = time(NULL);
            if (last_time == 0 || (now - last_time) >= send_interval_) {
                current_detections_ = temp_detections;
                std::string summary = buildDetectionSummary();
                control_->onDetectionSummary(summary);
                // printf("发送检测结果汇总 (包含%zu个物体)\n", temp_detections.size());
                last_time = now;
            }
        }
    }
    
    printf("[推理线程] 退出\n");
}

// ============ Thread4：编码推流循环 ============
// 职责：获取最新RGB帧并叠加检测信息，H.264编码后同时提供给RTSP和录像线程
//
// 帧获取：
//  加锁 → swap(dma_ready↔dma_read, bgr_ready↔bgr_read) → 解锁
//  DmaBuf 与 cv::Mat* 同步交换，保证 fd↔va 对应同一块物理内存
//
// 绘制（frame_ 为 RGB 字节序，Scalar 以 R-G-B 解读）：
//  - 检测框：绿色 Scalar(0,255,0)，标签文字同色
//  - ROI 区域框：蓝色 Scalar(0,0,255)
//  - 北京时间：白色 Scalar(255,255,255)，右上角
//  - FPS 数值：绿色，时间下方
//
// 编码路径：
//  memcpy(frame_.data → VENC DMA MB data_) →
//  RK_MPI_VENC_SendFrame → RK_MPI_VENC_GetStream →
//  ├→ rtsp_tx_video → rtsp_do_event
//  └→ Recording::enqueue → 有界内存队列 → 独立线程封装分片MP4
//
// 1080P 稳定段实测（2026-09-23）：
//  - AI关：绘制平均0.98ms，H.264平均20.68ms，总计平均33.86ms（约30fps）
//  - AI开：绘制平均4.03ms，H.264平均27.63ms，总计平均36.87ms（约26.6fps）
//  - AI开时的增幅主要来自 CPU调度和内存带宽竞争
//  - RTSP 推流和录像入队分别计时，录像磁盘写入不阻塞本线程
//  - Recording线程另行记录MP4封装、fflush和fdatasync耗时
void Video::encodeLoop() {
    RK_S32 s32Ret;
    int last_processed_index = -1;  // 上次处理的帧序号（避免重复）
    
    // FPS统计初始化
#if defined(ENABLE_FPS_CONSOLE) || defined(ENABLE_FPS_DISPLAY)
    frame_count_ = 0;
    gettimeofday(&fps_start_time_, NULL);
    current_fps_ = 0.0f;
#endif
    
#ifdef ENABLE_PERFORMANCE_TIMING
    struct timeval t_start, t_end;
    long swap_time_us, draw_time_us, encode_time_us, rtsp_time_us;
    long record_queue_time_us, total_time_us;
    int encode_counter = 0;
#endif
    
    printf("[编码线程] 启动 - 三缓冲模式，从BGR转换线程获取\n");
    
    while(running_) {
        // 🔒 从BGR缓冲区交换ready→read指针（加锁保护，允许跳帧）
        // 三缓冲优势：编码线程使用read缓冲期间，BGR转换可同时写write缓冲
        uint64_t timestamp;
        int current_frame_index;
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_start, NULL);
#endif
        
        pthread_mutex_lock(&bgr_buffer_.mutex);
        while (running_ && !bgr_buffer_.ready) {
            pthread_cond_wait(&bgr_buffer_.cond, &bgr_buffer_.mutex);
        }
        if (!running_) {
            pthread_mutex_unlock(&bgr_buffer_.mutex);
            break;
        }

        // 同时交换 DmaBuf 和 Mat 指针，保证两者始终对应同一块 DMA 内存
        DmaBuf   tmp_dma      = bgr_buffer_.dma_read;
        cv::Mat* tmp_mat      = bgr_buffer_.bgr_read;
        bgr_buffer_.dma_read  = bgr_buffer_.dma_ready;
        bgr_buffer_.bgr_read  = bgr_buffer_.bgr_ready;
        bgr_buffer_.dma_ready = tmp_dma;
        bgr_buffer_.bgr_ready = tmp_mat;

        timestamp = bgr_buffer_.timestamp;
        current_frame_index = bgr_buffer_.frame_index;
        bgr_buffer_.ready = false;  // 标记已消费
        last_processed_index = current_frame_index;  // 更新已处理帧序号
        pthread_mutex_unlock(&bgr_buffer_.mutex);
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_end, NULL);
        swap_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
        gettimeofday(&t_start, NULL);
#endif
        
        // 使用read缓冲区的BGR数据（浅拷贝，共享数据指针，无性能损失）
        frame_ = *bgr_buffer_.bgr_read;
        const bool ai_enabled_for_frame = ai_enable_;
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_end, NULL);
        // swap_time已包含在上面，这里不额外计时
        gettimeofday(&t_start, NULL);
#endif
        
        // 🔒 获取最新检测结果（加锁保护，仅读取）
        pthread_mutex_lock(&detection_mutex_);
        if (!shared_detections_.empty()) {
            // 有新推理结果，更新缓存
            cached_detections_.clear();
            cached_detections_.assign(shared_detections_.begin(), shared_detections_.end());
            has_detection_result_ = true;
        }
        pthread_mutex_unlock(&detection_mutex_);
        
        // 绘制检测框（使用缓存结果，每帧都绘制）
        // ⚠️ frame_ 存储 RGB 字节序（RGA 输出 RK_FORMAT_RGB_888），Scalar(R,G,B)
        if (ai_enabled_for_frame && has_detection_result_ && !cached_detections_.empty()) {
            for (const auto& det : cached_detections_) {
                cv::rectangle(frame_, 
                            cv::Point(det.x, det.y), 
                            cv::Point(det.x + det.w, det.y + det.h), 
                            cv::Scalar(0, 255, 0), 2);  // 绿色 (R=0,G=255,B=0)
                
                char text[64];
                sprintf(text, "%s %.0f%%", det.cls_name.c_str(), det.confidence * 100);
                cv::putText(frame_, text, cv::Point(det.x, det.y - 8), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 1);
            }
        }
        
        // 绘制区域框
        if (area_enable_) {
            int area_x = (int)(video_rectInfo.x * width_);
            int area_y = (int)(video_rectInfo.y * height_);
            int area_w = (int)(video_rectInfo.w * width_);
            int area_h = (int)(video_rectInfo.h * height_);
            
            // RGB Mat 中 Scalar(0,0,255) = 蓝色
            cv::rectangle(frame_, cv::Point(area_x, area_y), 
                        cv::Point(area_x + area_w, area_y + area_h), 
                        cv::Scalar(0, 0, 255), 1);
            
            cv::putText(frame_, "Area", cv::Point(area_x, area_y - 8),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        }
        
#if defined(ENABLE_FPS_CONSOLE) || defined(ENABLE_FPS_DISPLAY)
        // ✅ 精确FPS计算（使用微秒级时间）
        frame_count_++;
        struct timeval current_time;
        gettimeofday(&current_time, NULL);
        
        // 计算距离上次FPS更新的时间差（秒）
        double time_diff = (current_time.tv_sec - fps_start_time_.tv_sec) + 
                          (current_time.tv_usec - fps_start_time_.tv_usec) / 1000000.0;
        
        if (time_diff >= 1.0) {
            current_fps_ = (float)frame_count_ / time_diff;  // 精确FPS
#ifdef ENABLE_FPS_CONSOLE
            printf("[FPS统计] 实际处理帧数: %d, 时间差: %.3f秒, FPS: %.1f\n", 
                   frame_count_, time_diff, current_fps_);
#endif
            frame_count_ = 0;
            fps_start_time_ = current_time;
        }

#ifdef ENABLE_FPS_DISPLAY
        // 固定采用 UTC+8，避免开发板的 TZ 设置影响北京时间显示。
        const time_t beijing_seconds = current_time.tv_sec + 8 * 60 * 60;
        const double overlay_font_scale = 0.6;
        const int overlay_thickness = 2;
        const cv::Scalar overlay_color(0, 255, 0); // RGB Mat 中为绿色
        const int overlay_right_margin = 16;
        struct tm beijing_time;
        if (gmtime_r(&beijing_seconds, &beijing_time)) {
            char time_text[32];
            if (strftime(time_text, sizeof(time_text), "%Y-%m-%d %H:%M:%S", &beijing_time)) {
                // 时间格式固定为 19 个字符，按当前字体估算宽度以靠右显示。
                const int time_width = 220;
                const int time_x = width_ > time_width + overlay_right_margin
                    ? width_ - time_width - overlay_right_margin : 10;
                cv::putText(frame_, time_text, cv::Point(time_x, 30),
                            cv::FONT_HERSHEY_SIMPLEX, overlay_font_scale,
                            overlay_color, overlay_thickness);
            }
        }

        char fps_text[32];
        snprintf(fps_text, sizeof(fps_text), "FPS:%.1f", current_fps_);
        const int fps_width = 90;
        const int fps_x = width_ > fps_width + overlay_right_margin
            ? width_ - fps_width - overlay_right_margin : 10;
        cv::putText(frame_, fps_text, cv::Point(fps_x, 58),
                    cv::FONT_HERSHEY_SIMPLEX, overlay_font_scale,
                    overlay_color, overlay_thickness);
#endif
#endif
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_end, NULL);
        draw_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
        gettimeofday(&t_start, NULL);
#endif
        
        // 编码
        h264_frame_.stVFrame.u32TimeRef = H264_TimeRef_++;
        h264_frame_.stVFrame.u64PTS = timestamp;
        const size_t visible_row_bytes = static_cast<size_t>(width_) * 3;
        const size_t venc_row_bytes = visible_row_bytes;
        for (int row = 0; row < height_; ++row) {
            memcpy(data_ + static_cast<size_t>(row) * venc_row_bytes,
                   frame_.ptr(row), visible_row_bytes);
        }
        
        RK_S32 sendRet = RK_MPI_VENC_SendFrame(0, &h264_frame_, -1);
        if (sendRet != RK_SUCCESS) {
            fprintf(stderr, "[Video] VENC SendFrame 失败: %d\n", sendRet);
            continue;
        }
        s32Ret = RK_MPI_VENC_GetStream(0, &stFrame_, -1);
        
#ifdef ENABLE_PERFORMANCE_TIMING
        gettimeofday(&t_end, NULL);
        encode_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 + (t_end.tv_usec - t_start.tv_usec);
        rtsp_time_us = 0;
        record_queue_time_us = 0;
#endif
        
        // 推流
        if (s32Ret == RK_SUCCESS) {
            void *pData = RK_MPI_MB_Handle2VirAddr(stFrame_.pstPack->pMbBlk);
            if (pData && stFrame_.pstPack->u32Len) {
#ifdef ENABLE_PERFORMANCE_TIMING
                gettimeofday(&t_start, NULL);
#endif
                if (g_rtsplive_ && g_rtsp_session_) {
                    rtsp_tx_video(g_rtsp_session_, (uint8_t *)pData,
                                  stFrame_.pstPack->u32Len, stFrame_.pstPack->u64PTS);
                    rtsp_do_event(g_rtsplive_);
                }
#ifdef ENABLE_PERFORMANCE_TIMING
                gettimeofday(&t_end, NULL);
                rtsp_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 +
                               (t_end.tv_usec - t_start.tv_usec);
                gettimeofday(&t_start, NULL);
#endif
                recording_.enqueue(pData, stFrame_.pstPack->u32Len,
                                   stFrame_.pstPack->u64PTS, ai_enabled_for_frame);
#ifdef ENABLE_PERFORMANCE_TIMING
                gettimeofday(&t_end, NULL);
                record_queue_time_us = (t_end.tv_sec - t_start.tv_sec) * 1000000 +
                                       (t_end.tv_usec - t_start.tv_usec);
#endif
            }
        } else {
            fprintf(stderr, "[Video] VENC GetStream 失败: %d\n", s32Ret);
        }
        
#ifdef ENABLE_PERFORMANCE_TIMING
        total_time_us = swap_time_us + draw_time_us + encode_time_us +
                        rtsp_time_us + record_queue_time_us;
        
        encode_counter++;
        if (encode_counter % 30 == 0) {
            performanceLog(ai_enabled_for_frame,
                           "[编码#%d] 指针交换: %ld us, 画面叠加: %ld us, "
                           "H.264编码: %ld us, RTSP: %ld us, 录像入队: %ld us, 总计: %ld us",
                           encode_counter, swap_time_us, draw_time_us, encode_time_us,
                           rtsp_time_us, record_queue_time_us, total_time_us);
        }
#endif
        
        if (s32Ret == RK_SUCCESS) RK_MPI_VENC_ReleaseStream(0, &stFrame_);
    }
    
    printf("[编码线程] 退出\n");
}

/**
 * @brief CPU 版 letterbox 缩放（RGA 路径失败时的回退实现）
 * @param input 输入图像（RGB 或 BGR，当前采集尺寸，CV_8UC3）
 * @return 640×640 letterbox 图像（通道顺序与输入相同，黑边填 0）
 *
 * 实现步骤：
 *  1. 计算等比缩放系数 scale_ = min(W_scale, H_scale)
 *  2. INTER_NEAREST resize（比 INTER_LINEAR 快 2~5 倍，精度损失可接受）
 *  3. memset 黑色背景（640×640×3）
 *  4. copyTo 到 (leftPadding_, topPadding_) 偏移处
 *
 * 同步副作用：更新 scale_、leftPadding_、topPadding_ 成员变量，
 * 与 RGA 路径保持相同的坐标映射参数。
 */
cv::Mat Video::letterbox(cv::Mat input) {
    float scaleX = (float)model_width_ / (float)width_;
    float scaleY = (float)model_height_ / (float)height_;
    scale_ = scaleX < scaleY ? scaleX : scaleY;
    int inputWidth = (int)((float)width_ * scale_);
    int inputHeight = (int)((float)height_ * scale_);
    leftPadding_ = (model_width_ - inputWidth) / 2;
    topPadding_ = (model_height_ - inputHeight) / 2;
    cv::Mat inputScale;
    cv::resize(input, inputScale, cv::Size(inputWidth,inputHeight), 0, 0, cv::INTER_NEAREST);
    cv::Mat letterboxImage(model_height_, model_width_, CV_8UC3);
    memset(letterboxImage.data, 0, model_width_ * model_height_ * 3);
    cv::Rect roi(leftPadding_, topPadding_, inputWidth, inputHeight);
    inputScale.copyTo(letterboxImage(roi));
    return letterboxImage;
}

// ==================== RGA 资源管理 ====================

/**
 * @brief 分配 letterbox 专用 DMA 缓冲并预填黑色
 * @return true=成功，false=CMA 和 system heap 均分配失败
 *
 * 分配策略：
 *  优先 RV1106_CMA_HEAP_PATH(/dev/rk_dma_heap/rk-dma-heap-cma)，
 *  失败时回退 DMA_HEAP_PATH(/dev/dma_heap/system)。
 *
 * 预填黑色（memset 0）的意义：
 *  letterbox 的黑边区域只需初始化一次。
 *  后续 RGA improcess 每帧只写入图像区域（lb_dst_rect 指定偏移），
 *  黑边区域保持不变，无需每帧重新 memset，节省 ~0.5ms/帧。
 *
 * 在 init() 的最后一步调用，确保 model_width_/model_height_ 已设置。
 */
bool Video::initRgaBuffers() {
    int lb_size = model_width_ * model_height_ * 3;  // 640×640×3 = 1.2MB

    // 分配 letterbox 专用 DMA buf（CMA 物理连续，RGA 可直接写入）
    if (dma_buf_alloc(RV1106_CMA_HEAP_PATH, lb_size,
                      &letterbox_dma_.fd, &letterbox_dma_.va) < 0) {
        // 回退到通用 system heap
        if (dma_buf_alloc(DMA_HEAP_PATH, lb_size,
                          &letterbox_dma_.fd, &letterbox_dma_.va) < 0) {
            printf("[RGA] ❌ letterbox DMA缓冲分配失败\n");
            return false;
        }
    }
    letterbox_dma_.size = lb_size;
    memset(letterbox_dma_.va, 0, lb_size);  // 预填黑色（letterbox黑边只填一次）

    printf("[RGA] ✅ letterbox DMA缓冲初始化成功 (%d KB, fd=%d)\n",
           lb_size / 1024, letterbox_dma_.fd);
    return true;
}

/**
 * @brief 释放 letterbox DMA 缓冲（在 stop() 内调用）
 *
 * dma_buf_free 解除 mmap 并关闭 fd，
 * 随后将 letterbox_dma_ 重置为默认值（fd=-1）防止二次释放。
 * YUV 和 BGR 三缓冲的释放在析构函数中直接进行。
 */
void Video::releaseRgaBuffers() {
    if (letterbox_dma_.fd >= 0) {
        dma_buf_free(letterbox_dma_.size, &letterbox_dma_.fd, letterbox_dma_.va);
        letterbox_dma_ = DmaBuf{};  // 重置为默认值（fd=-1, va=nullptr, size=0）
    }
    printf("[RGA] ✅ RGA缓冲已释放\n");
}

/**
 * @brief 将模型输出坐标映射回原始图像坐标
 * @param x 输入模型坐标 x → 输出原图坐标 x
 * @param y 输入模型坐标 y → 输出原图坐标 y
 *
 * letterbox 正变换：原图 → 等比缩放 → 居中偏移(leftPadding_, topPadding_)
 * 逆变换：模型坐标 - padding → 除以 scale_
 * 依赖 inferenceLoop 在每帧推理前更新的 scale_ / leftPadding_ / topPadding_。
 */
void Video::mapCoordinates(int *x, int *y) {
    int mx = *x - leftPadding_;
    int my = *y - topPadding_;
    *x = (int)((float)mx / scale_);
    *y = (int)((float)my / scale_);
}


void Video::getRectInfo(const RectInfo& info) {
    //拷贝一份数据到 video_rectInfo
    video_rectInfo.x = info.x;
    video_rectInfo.y = info.y;
    video_rectInfo.w = info.w;
    video_rectInfo.h = info.h;
}

void Video::getObjectList(const std::vector<int>& objList) {
    // 处理对象列表逻辑，拷贝数据到 video_objList
    video_objList.clear();
    for (const auto& obj : objList) {
        video_objList.push_back(obj);
    }
}

void Video::startAI() {
    ai_enable_ = true;
    performanceLog(true, "[状态] AI推理已开启");
}
void Video::stopAI() {
    ai_enable_ = false;
    performanceLog(false, "[状态] AI推理已关闭");
}
void Video::startAreaDetect() {
    area_enable_ = true;
}
void Video::stopAreaDetect() {
    area_enable_ = false;
}
void Video::startObjectDetect() {
    obj_enable_ = true;
}
void Video::stopObjectDetect() {
    obj_enable_ = false;
}

void Video::setControl(Control* control) {
    control_ = control;
}

// 设置检测结果发送间隔（秒）
void Video::setSendInterval(int interval) {
    if (interval > 0) {
        send_interval_ = interval;
        printf("检测结果发送间隔已设置为: %d秒\n", send_interval_);
    } else {
        printf("无效的发送间隔，必须大于0\n");
    }
}

/**
 * @brief 将 current_detections_ 序列化为固定格式字符串，通过 TCP 上报
 * @return 格式："DETECTIONS:N|cls_id:name:x:y:w:h:conf|..."
 *         无检测时返回："DETECTIONS:NONE"
 */
std::string Video::buildDetectionSummary() {
    if (current_detections_.empty()) {
        return "DETECTIONS:NONE";
    }
    
    std::ostringstream oss;
    oss << "DETECTIONS:" << current_detections_.size();
    
    for (size_t i = 0; i < current_detections_.size(); ++i) {
        const DetectionInfo& detection = current_detections_[i];
        oss << "|" << detection.cls_id << ":" << detection.cls_name 
            << ":" << detection.x << ":" << detection.y 
            << ":" << detection.w << ":" << detection.h 
            << ":" << std::fixed << std::setprecision(3) << detection.confidence;
    }
    
    return oss.str();
}
