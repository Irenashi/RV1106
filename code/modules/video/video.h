/**
 * @file video.h
 * @brief 视频处理模块头文件 —— 四线程异步流水线架构
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │                     视频处理流水线                               │
 * │                                                                 │
 * │  Thread1: captureLoop   (30fps)                                 │
 * │    VI硬件 → NV12(DMA) ──memcpy──→ capture_buffer(DMA)          │
 * │                                         │                       │
 * │  Thread2: bgrConvertLoop (≈27fps)       │                       │
 * │    ←── capture_buffer(DMA fd)           │                       │
 * │    RGA imcvtcolor: NV12→RGB (<5ms)      │                       │
 * │    → bgr_buffer 三缓冲(DMA) ──┬─────────┘                       │
 * │                               │                                 │
 * │  Thread3: inferenceLoop (≈6fps)│                                │
 * │    ←── inference_bgr_buffer ──┘                                 │
 * │    RGA improcess: letterbox 缩放+填充 (<3ms)                     │
 * │    → RKNN YOLOv5 推理 (~96ms) → shared_detections_             │
 * │                                                                 │
 * │  Thread4: encodeLoop (≈27fps)                                   │
 * │    ←── bgr_buffer(DMA fd)                                       │
 * │    OpenCV 绘制检测框/FPS                                         │
 * │    → VENC H.264 硬件编码 → RTSP 推流                            │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * 关键设计：
 *  - 所有 RGA 操作的缓冲均使用 DMA CMA 物理连续内存（wrapbuffer_fd），
 *    满足 RV1106 RGA2 硬件约束（不支持虚拟地址）
 *  - BGR 三缓冲指针轮换：DmaBuf 与 cv::Mat* 同步交换，保证 fd↔va 绑定不变
 *  - 推理跳帧：每 (INFERENCE_FRAME_SKIP+1) 帧推理一次，降低 NPU 负载
 *  - letterbox 黑边预填充：DMA buf 初始化时 memset(0)，此后 RGA 只写图像区域
 */
#ifndef VIDEO_H
#define VIDEO_H

// ==================== 性能调试开关 ====================
#define ENABLE_PERFORMANCE_TIMING   // 开启各线程逐步耗时打印（30帧/次）
// #define ENABLE_FPS_CONSOLE       // 开启终端 FPS 输出
#define ENABLE_FPS_DISPLAY          // 开启画面右上角北京时间 + FPS 叠加显示

// ==================== 系统配置参数 ====================
#define IMAGE_WIDTH           1280   // 采集/编码分辨率：宽（像素）
#define IMAGE_HEIGHT          720   // 采集/编码分辨率：高（像素）
#define MODEL_WIDTH           640   // YOLOv5 模型输入宽度
#define MODEL_HEIGHT          640   // YOLOv5 模型输入高度
#define INFERENCE_FRAME_SKIP  3     // 推理跳帧数：每 (N+1) 帧推理 1 次，3 → 每4帧推1次

// ==================== 系统头文件 ====================
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include <vector>

// ==================== 第三方库 ====================
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "im2d.h"       // RGA 硬件加速 API（imcvtcolor / improcess）
#include "RgaApi.h"     // RGA 底层封装（wrapbuffer_fd 等）
#include "dma_alloc.h"  // DMA CMA 内存分配（dma_buf_alloc / dma_buf_free）

// ==================== 项目头文件 ====================
#include "rtsp_demo.h"
#include "luckfox_mpi.h"
#include "yolov5.h"
#include "common.h"
#include "recording.h"

// ==================== 前向声明 ====================
class Control;

/**
 * @class Video
 * @brief 视频处理核心类，封装完整的采集→转换→推理→编码流水线
 *
 * 硬件加速层次：
 *  - ISP：图像信号处理（曝光/白平衡/降噪）
 *  - RGA：YUV→RGB 色彩转换 + letterbox 缩放（替代 CPU cv::cvtColor）
 *  - NPU：YOLOv5 目标检测推理（RKNN）
 *  - VENC：H.264 硬件编码（MPP）
 *
 * 线程安全说明：
 *  - capture_buffer_：captureLoop 写，bgrConvertLoop 读，mutex 保护 ready 标志
 *  - bgr_buffer_：bgrConvertLoop 写，encodeLoop/inferenceLoop 读，mutex 保护指针轮换
 *  - inference_bgr_buffer_：bgrConvertLoop 写，inferenceLoop 读，mutex 保护指针轮换
 *  - shared_detections_：inferenceLoop 写，encodeLoop 读，detection_mutex_ 保护
 */
class Video {
public:
    // ==================== 生命周期 ====================

    /**
     * @brief 构造函数，分配所有 DMA 缓冲并初始化互斥锁
     * @param width       采集/编码视频宽度
     * @param height      采集/编码视频高度
     * @param model_width  RKNN 模型输入宽度
     * @param model_height RKNN 模型输入高度
     *
     * DMA 分配顺序：
     *  1. capture_buffer_.dma  — NV12 YUV，width×height×3/2 字节
     *  2. bgr_buffer_.dma_write/ready/read — RGB，width×height×3 字节 ×3
     *  3. letterbox_dma_ — 在 initRgaBuffers() 中分配，model_width×model_height×3 字节
     */
    Video(int width, int height, int model_width, int model_height);

    /**
     * @brief 析构函数，调用 stop() 后释放所有 Mat 头和 DMA 缓冲
     *
     * 释放顺序（反向）：
     *  1. delete cv::Mat 头（data 由 DMA 管理，Mat 析构不会 free data）
     *  2. dma_buf_free：YUV buf → BGR 三缓冲 → letterbox buf
     */
    ~Video();

    /**
     * @brief 初始化所有硬件资源
     * @return true=全部成功，false=任意步骤失败
     *
     * 初始化顺序：
     *  1. RKNN 模型加载（yolov5.rknn）
     *  2. VENC 内存池 + 帧结构（RK_FMT_RGB888）
     *  3. ISP 初始化（/etc/iqfiles）
     *  4. MPI 系统初始化
     *  5. RTSP 服务（端口554，/live/0）
     *  6. VI 采集通道（width×height）
     *  7. VENC 编码通道（H.264 CBR 8Mbps）
     *  8. RGA letterbox DMA 缓冲分配 + 预填黑色
     */
    bool init();

    /**
     * @brief 启动四个处理线程
     * @return true=全部启动成功，false=任意线程创建失败
     */
    bool start();

    /**
     * @brief 停止所有线程，join 后释放硬件资源（VI/VENC/ISP/RTSP/RKNN）
     */
    void stop();

    // ==================== 视频流控制 ====================
    void pauseAllThreads();                      // 暂停四线程（running_=false + join），硬件保持激活
    void resumeAllThreads();                     // 重新启动四线程（running_=true + create）
    bool isRunning() const { return running_; }  // 返回当前运行状态

    // ==================== AI 功能控制 ====================
    void startAI();           // 启用 YOLOv5 推理（inferenceLoop 开始处理帧）
    void stopAI();            // 禁用推理（inferenceLoop 休眠 100ms/次）
    void startAreaDetect();   // 启用 ROI 区域过滤（仅检测区域内目标）
    void stopAreaDetect();    // 禁用区域过滤
    void startObjectDetect(); // 启用类别白名单过滤
    void stopObjectDetect();  // 禁用类别过滤

    /** 设置检测 ROI 区域（归一化坐标，0.0~1.0） */
    void getRectInfo(const RectInfo& info);

    /** 设置类别白名单（COCO cls_id 列表，仅白名单内类别上报） */
    void getObjectList(const std::vector<int>& list);

    /** 设置检测结果上报间隔（秒，默认1s，通过 Control 回调发送） */
    void setSendInterval(int interval);

    // ==================== 模块关联 ====================
    /** 注入 Control 模块指针，用于检测结果回调 onDetectionSummary() */
    void setControl(Control* control);

private:
    // ==================== 内部数据结构 ====================

    /**
     * @brief 单个目标检测结果
     */
    struct DetectionInfo {
        int cls_id;           // COCO 类别 ID（0~79）
        std::string cls_name; // 类别名称字符串（"person" / "car" 等）
        int x, y, w, h;       // 边界框：左上角(x,y)，宽 w，高 h（原图像素坐标）
        float confidence;     // 置信度（0.0~1.0）
    };

    /**
     * @brief DMA 连续物理内存描述符
     *
     * RV1106 RGA2 仅支持物理连续内存（通过 fd 访问）。
     * 使用 dma_buf_alloc(/dev/rk_dma_heap/rk-dma-heap-cma) 分配，
     * 返回 fd（硬件访问句柄）和 va（CPU mmap 虚拟地址）。
     *
     * 使用约定：
     *  - RGA 操作：wrapbuffer_fd(fd, w, h, fmt)
     *  - CPU 读写：直接访问 va（如 memcpy、cv::Mat 封装）
     *  - 释放：dma_buf_free(size, &fd, va)
     */
    struct DmaBuf {
        int    fd   = -1;      // DMA buf 文件描述符（-1 表示未分配）
        void*  va   = nullptr; // mmap 虚拟地址（CPU 访问用）
        size_t size = 0;       // 缓冲区字节数
    };

    /**
     * @brief YUV 帧单缓冲区（采集线程写 → BGR 转换线程读）
     *
     * 不使用双缓冲：captureLoop 直接覆盖写入（保证30fps实时性），
     * bgrConvertLoop 通过 ready 标志判断是否有新帧，允许丢帧。
     */
    struct FrameBuffer {
        DmaBuf          dma;       // NV12 YUV DMA 内存（width×height×3/2 字节）
        uint64_t        timestamp; // 帧时间戳（微秒，来自 TEST_COMM_GetNowUs）
        bool            ready;     // true = 有新帧待处理
        pthread_mutex_t mutex;     // 保护 ready 标志的读写
        pthread_cond_t  cond;      // 条件变量：采集线程生产后唤醒 BGR 转换线程
    };

    /**
     * @brief RGB 帧三缓冲区（BGR 转换线程写 → 编码线程读）
     *
     * 三缓冲设计允许编码线程持有 read 缓冲独占读取期间，
     * BGR 转换线程同时向 write 缓冲写入下一帧，ready 缓冲暂存中间帧。
     *
     * ⚠️ 轮换规则：DmaBuf 与 cv::Mat* 必须同步交换，保证：
     *      bgr_write->data == dma_write.va（同一块 DMA 物理内存）
     *      bgr_ready->data == dma_ready.va
     *      bgr_read->data  == dma_read.va
     *
     * 违反此约定会导致 RGA 写入的物理地址与 CPU 读取的虚拟地址不对应，
     * 引发画面撕裂/频闪。
     */
    struct BGRFrameBuffer {
        DmaBuf   dma_write;    // 当前 RGA 写入目标（RGB，width×height×3 字节）
        DmaBuf   dma_ready;    // 已完成转换、等待消费的帧
        DmaBuf   dma_read;     // 编码/推理线程当前独占读取的帧
        cv::Mat* bgr_write;    // 封装 dma_write.va 的 Mat 头（CV_8UC3，不拥有 data）
        cv::Mat* bgr_ready;    // 封装 dma_ready.va 的 Mat 头
        cv::Mat* bgr_read;     // 封装 dma_read.va 的 Mat 头
        uint64_t timestamp;    // 最新 ready 帧的时间戳
        int      frame_index;  // 最新 ready 帧的帧序号（用于推理跳帧判断）
        bool     ready;        // true = dma_ready 有新帧待编码
        pthread_mutex_t mutex; // 保护 dma/Mat 指针轮换和 ready 标志
        pthread_cond_t  cond;  // 条件变量：BGR 转换线程生产后唤醒编码线程
    };

    /**
     * @brief 推理专用 RGB 三缓冲区（BGR 转换线程写 → 推理线程读）
     *
     * Mat 为浅拷贝头（共享 bgr_buffer_ DMA 内存的 va），
     * 不额外分配 DMA 内存，但需同步记录对应的 DMA fd（供 RGA letterbox 使用）。
     *
     * ⚠️ dma_fd_write/ready/read 必须与 bgr_write/ready/read 同步轮换，
     *    确保推理线程拿到的 fd 与 Mat->data 指向同一块物理内存。
     */
    struct InferenceBGRBuffer {
        cv::Mat* bgr_write;        // 写缓冲 Mat 头（浅拷贝自 bgr_buffer_.bgr_ready）
        cv::Mat* bgr_ready;        // 就绪缓冲
        cv::Mat* bgr_read;         // 推理线程独占读取缓冲
        int dma_fd_write  = -1;    // bgr_write 对应的 DMA fd（供 RGA wrapbuffer_fd）
        int dma_fd_ready  = -1;    // bgr_ready 对应的 DMA fd
        int dma_fd_read   = -1;    // bgr_read  对应的 DMA fd
        int  frame_index;          // 帧序号
        bool ready;                // true = bgr_ready 有新帧待推理
        pthread_mutex_t mutex;     // 保护指针轮换和 ready 标志
    };

    // ==================== 线程入口（static，转发到成员函数）====================
    static void* captureThreadFunc(void* arg);    // → captureLoop()
    static void* bgrConvertThreadFunc(void* arg); // → bgrConvertLoop()
    static void* inferenceThreadFunc(void* arg);  // → inferenceLoop()
    static void* encodeThreadFunc(void* arg);     // → encodeLoop()

    // ==================== 线程主循环 ====================

    /**
     * @brief 采集循环（Thread1，~30fps）
     *
     * RK_MPI_VI_GetChnFrame → memcpy(vi_data → capture_buffer_.dma.va) → Release
     * 不检查 ready 状态，直接覆盖，保证采集实时性（下游允许丢帧）。
     * 耗时：VI 获取 ~30ms（硬件同步），YUV memcpy ~2ms（900KB）
     */
    void captureLoop();

    /**
     * @brief RGB 转换循环（Thread2，~27fps，系统吞吐瓶颈）
     *
     * 等待 capture_buffer_.ready → RGA imcvtcolor(NV12→RGB) → 三缓冲轮换
     * RGA 格式：RK_FORMAT_YCbCr_420_SP → RK_FORMAT_RGB_888（BT.601 Full Range）
     * 每 (INFERENCE_FRAME_SKIP+1) 帧额外向 inference_bgr_buffer_ 传递一帧。
     * 耗时：RGA 转换 <5ms（对比 CPU cvtColor ~32ms）
     */
    void bgrConvertLoop();

    /**
     * @brief 推理循环（Thread3，~6fps，受 NPU 速度限制）
     *
     * 等待 inference_bgr_buffer_.ready →
     *   RGA improcess letterbox(RGB 1280×720 → RGB 640×640，带黑边填充) →
     *   memcpy(letterbox_dma_.va → rknn_input_mems) →
     *   inference_yolov5_model → 更新 shared_detections_
     * letterbox 黑边在 initRgaBuffers() 时预填充，此后 RGA 仅写图像区域，无需每帧 memset。
     * 耗时：RGA letterbox <3ms + RKNN 推理 ~96ms
     */
    void inferenceLoop();

    /**
     * @brief 编码推流循环（Thread4，~27fps）
     *
     * 等待 bgr_buffer_.ready → OpenCV 绘制检测框/北京时间/FPS →
     *   memcpy(frame_.data → VENC DMA MB) →
     *   RK_MPI_VENC_SendFrame → RK_MPI_VENC_GetStream → RTSP 推流
     * frame_ 为 RGB 字节序（CV_8UC3），Scalar(R,G,B) 通道顺序与 BGR Mat 相反。
     * 耗时：绘制 ~2ms + H.264 编码 ~12ms + RTSP ~1ms
     */
    void encodeLoop();

    // ==================== 图像处理辅助函数 ====================

    /**
     * @brief CPU 版 letterbox（RGA 失败时的回退路径）
     * @param input BGR/RGB Mat（1280×720）
     * @return 640×640 letterbox Mat（相同通道顺序，黑边填充）
     * 使用 INTER_NEAREST 插值（比 INTER_LINEAR 快 2~5 倍）。
     */
    cv::Mat letterbox(cv::Mat input);

    /**
     * @brief 分配 letterbox DMA 缓冲并预填黑色
     * @return true=成功，false=CMA 和 system heap 均分配失败
     * 优先使用 RV1106_CMA_HEAP_PATH，失败时回退 DMA_HEAP_PATH。
     */
    bool initRgaBuffers();

    /**
     * @brief 释放 letterbox DMA 缓冲（dma_buf_free + 重置 DmaBuf 字段）
     */
    void releaseRgaBuffers();

    /**
     * @brief 将模型坐标系中的点映射回原始图像坐标系
     * @param x  输入：模型坐标 x；输出：原图坐标 x
     * @param y  输入：模型坐标 y；输出：原图坐标 y
     * 逆 letterbox：先减去 padding，再除以 scale_。
     */
    void mapCoordinates(int* x, int* y);

    /**
     * @brief 将 current_detections_ 序列化为上报字符串
     * @return "DETECTIONS:N|cls_id:name:x:y:w:h:conf|..." 格式
     */
    std::string buildDetectionSummary();

    // ==================== 成员变量 ====================

    // --- 视频分辨率参数 ---
    int   width_;        // 采集/编码宽度（像素）
    int   height_;       // 采集/编码高度（像素）
    int   model_width_;  // RKNN 模型输入宽度
    int   model_height_; // RKNN 模型输入高度
    float scale_;        // letterbox 等比缩放系数（min(W_scale, H_scale)）
    int   leftPadding_;  // letterbox 水平黑边宽度（像素）
    int   topPadding_;   // letterbox 垂直黑边高度（像素）

    // --- RKNN 推理上下文 ---
    rknn_app_context_t        rknn_app_ctx_;  // RKNN 运行时上下文（模型/IO 内存等）
    object_detect_result_list od_results_;   // 单次推理输出的检测结果列表
    char                      text_[16];     // 临时字符串缓冲

    // --- RK MPI 视频采集/编码 ---
    VENC_STREAM_S      stFrame_;      // VENC 输出流包（含 H.264 NAL 数据）
    RK_U32             H264_TimeRef_; // H.264 时间戳递增计数器
    VIDEO_FRAME_INFO_S stViFrame_;    // VI GetChnFrame 返回的帧信息
    MB_POOL            src_Pool_;     // VENC 输入内存池（DMA 类型）
    MB_BLK             src_Blk_;      // VENC 输入内存块句柄
    VIDEO_FRAME_INFO_S h264_frame_;   // 送入 VENC 的帧描述（RK_FMT_RGB888）
    cv::Mat            frame_;        // 封装 data_ 的 Mat 头（编码线程绘制用）
    unsigned char*     data_;         // VENC 输入内存块的虚拟地址

    // --- RGA 硬件加速 ---
    /**
     * letterbox 输出专用 DMA 缓冲（model_width×model_height×3，RGB）
     * 在 initRgaBuffers() 分配并预填黑色（letterbox 黑边仅填一次）。
     * RGA improcess 将缩放后的图像写入 (leftPadding_, topPadding_) 偏移处，
     * 随后 memcpy(va → rknn_input_mems) 送入 NPU。
     */
    DmaBuf letterbox_dma_;

    // --- RTSP 推流 ---
    rtsp_demo_handle    g_rtsplive_;      // RTSP 服务器实例句柄
    rtsp_session_handle g_rtsp_session_;  // RTSP 会话句柄（/live/0）
    Recording           recording_;       // SD 卡录像线程，复用 VENC 的 H.264 输出

    // --- 线程管理 ---
    pthread_t thread_capture_;     // 采集线程 tid
    pthread_t thread_bgr_convert_; // RGB 转换线程 tid
    pthread_t thread_inference_;   // 推理线程 tid
    pthread_t thread_encode_;      // 编码推流线程 tid
    bool      running_;            // 全局运行标志（false → 各线程退出循环）

    // --- AI 功能开关 ---
    bool ai_enable_;   // true = inferenceLoop 处理帧，false = 休眠
    bool area_enable_; // true = 仅上报落在 video_rectInfo 区域内的目标
    bool obj_enable_;  // true = 仅上报 video_objList 白名单内的类别

    // --- 线程间通信缓冲区 ---
    FrameBuffer         capture_buffer_;          // Thread1→Thread2：NV12 单缓冲
    BGRFrameBuffer      bgr_buffer_;              // Thread2→Thread4：RGB 三缓冲
    InferenceBGRBuffer  inference_bgr_buffer_;    // Thread2→Thread3：RGB 三缓冲（浅拷贝）
    std::vector<DetectionInfo> shared_detections_; // Thread3→Thread4：最新检测结果
    std::vector<DetectionInfo> cached_detections_; // Thread4 本地缓存（避免长时间持锁）
    pthread_mutex_t    detection_mutex_;           // 保护 shared_detections_ 读写
    bool               has_detection_result_;      // Thread4 是否已有至少一次推理结果

    // --- AI 配置 ---
    int              inference_frame_skip_; // 推理跳帧数（运行时可调）
    RectInfo         video_rectInfo;        // ROI 区域（归一化坐标）
    std::vector<int> video_objList;         // 类别白名单（COCO cls_id）

    // --- 结果上报 ---
    Control*                   control_;             // Control 模块指针（结果回调）
    time_t                     last_send_time_;      // 上次上报时间（秒级时间戳）
    int                        send_interval_;       // 上报间隔（秒）
    static const int           DEFAULT_SEND_INTERVAL = 1;
    std::vector<DetectionInfo> current_detections_;  // 本次待上报的检测列表

    // --- FPS 统计（条件编译）---
#if defined(ENABLE_FPS_CONSOLE) || defined(ENABLE_FPS_DISPLAY)
    int            frame_count_;      // 当前统计周期内的编码帧数
    struct timeval fps_start_time_;   // 当前统计周期的起始时刻
    float          current_fps_;      // 最近一次计算得到的 FPS
#endif
};

#endif // VIDEO_H
