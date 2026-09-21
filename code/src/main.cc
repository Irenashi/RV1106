// main.cc 详细中文注释版
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "control.h"
#include "servo.h"
#include "video.h"
#include "tcp.h"
#include "udp_discovery.h"

volatile sig_atomic_t quit = 0;

static void sigterm_handler(int sig) {
    (void)sig;
    quit = 1;
}

// 获取本机IP地址（优先获取eth0，其次wlan0）
std::string getLocalIP() {
    struct ifaddrs * ifAddrStruct = NULL;
    struct ifaddrs * ifa = NULL;
    void * tmpAddrPtr = NULL;
    std::string ip = "127.0.0.1";

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            
            // 跳过回环地址
            if (strcmp(ifa->ifa_name, "lo") != 0) {
                ip = addressBuffer;
                // 优先返回eth0或wlan0
                if (strcmp(ifa->ifa_name, "eth0") == 0 || strcmp(ifa->ifa_name, "wlan0") == 0) {
                    break;
                }
            }
        }
    }
    if (ifAddrStruct != NULL) freeifaddrs(ifAddrStruct);
    return ip;
}

// 主程序入口
int main(int argc, char *argv[]) 
{
    // 使用video.h中定义的宏来设置分辨率参数
    int width = IMAGE_WIDTH;
    int height = IMAGE_HEIGHT;
    int model_width = MODEL_WIDTH;
    int model_height = MODEL_HEIGHT;

    // Ctrl+C / SIGTERM 都走线程收尾流程，让正在写入的 .part 完成并改名。
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = sigterm_handler;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);

    // 初始化云台
    printf("开始云台初始化！\n");
    Servo* g_servo = new Servo();
    printf("云台初始化成功\n");

    // 初始化并启动视频处理模块（不依赖TCP，优先启动）
    printf("开始视频初始化！\n");
    Video* g_video = new Video(width, height, model_width, model_height);
    bool videoReady = false;
    if (g_video->init()) {
        printf("视频模块初始化成功！\n");
        videoReady = g_video->start();
        printf(videoReady ? "视频线程已自动启动\n" : "视频线程启动失败！\n");
    } else {
        printf("视频模块初始化失败！\n");
    }

    // ✅ 初始化TCP客户端（初始IP为空，等待UDP发现后动态设置）
    printf("开始TCP初始化（等待UDP发现上位机）...\n");
    TcpClient* g_tcp = new TcpClient("", 8890);  // 初始IP为空，等待UDP发现
    g_tcp->start();  // 启动TCP线程（等待有效IP后才会连接）
    printf("TCP线程已启动（等待UDP发现上位机IP）\n");

    // ✅ 启动UDP广播发现服务（端口8888）
    printf("开始启动UDP广播发现服务...\n");
    // 生成设备ID（可根据实际需求调整，如使用MAC地址）
    std::string deviceId = "CAM_LUCKFOX_001";
    std::string deviceName = "智能识别摄像头";
    
    // 自动获取本机IP并构建RTSP URL
    std::string localIp = getLocalIP();
    std::string rtspUrl = "rtsp://" + localIp + "/live/0";
    printf("本机IP: %s, RTSP URL: %s\n", localIp.c_str(), rtspUrl.c_str());
    
    UdpDiscovery* g_udp = new UdpDiscovery(deviceId, deviceName, rtspUrl);
    g_udp->setIpAddress(localIp);  // 设置设备IP，以便在发现响应中返回
    g_udp->setManufacturer("Luckfox");
    g_udp->setModel("RV1106-Smart");
    g_udp->setFirmwareVersion("1.0.0");
    
    // 设置连接请求回调，只有收到connection_request时才建立TCP连接
    // discovery_request只会响应设备信息，不会触发此回调
    g_udp->setHostDiscoveredCallback([](const char* hostIp, int hostPort, void* userData) {
        TcpClient* tcp = static_cast<TcpClient*>(userData);
        if (tcp) {
            printf("🎯 收到连接请求，建立TCP连接: %s:%d\n", hostIp, hostPort);
            tcp->updateServerAddress(hostIp, hostPort);  // 使用解析的IP和端口
        }
    }, g_tcp);  // 将g_tcp作为userData传入
    
    if (g_udp->start()) {
        printf("✅ UDP广播发现服务启动成功！\n");
        printf("   - 监听端口: 8888\n");
        printf("   - 设备ID: %s\n", deviceId.c_str());
        printf("   - 工作模式: 两阶段连接\n");
        printf("     1️⃣  响应discovery_request（设备发现）\n");
        printf("     2️⃣  收到connection_request后建立TCP连接\n");
    } else {
        printf("❌ UDP广播发现服务启动失败\n");
    }

    // 实例化Control对象，传入所有需要的指针
    Control* g_control = new Control(g_servo, g_video, g_tcp);
    printf("Control模块已初始化\n");

    // 设置TCP客户端的Control指针，使其能够处理接收到的数据
    g_tcp->setControl(g_control);
    printf("TCP-Control关联已建立\n");

    // 设置Video的Control指针，使其能够通知Control检测结果
    g_video->setControl(g_control);
    printf("Video-Control关联已建立\n");

    char cmd[128];
    printf("\n===========================================\n");
    printf("系统就绪！支持本地命令输入（格式与TCP相同）\n");
    printf("命令示例：\n");
    printf("  DEVICE_1:OP_1:VALUE_90    - 云台控制\n");
    printf("  DEVICE_2:OP_6:VALUE_1     - 启动AI识别\n");
    printf("  DEVICE_2:OP_9:VALUE_0     - 暂停视频流\n");
    printf("  RECT:100,100,200,150      - 设置检测区域\n");
    printf("  LIST:0,1,2                - 设置检测对象\n");
    printf("  quit                      - 退出程序\n");
    printf("===========================================\n");
    printf("服务状态：\n");
    if (videoReady) printf("  ✅ RTSP 推流      - rtsp://%s/live/0\n", localIp.c_str());
    else printf("  ❌ RTSP 推流未启动\n");
    printf("  ✅ UDP 发现服务   - 端口 8888\n");
    printf("  ✅ TCP 通信       - 等待上位机连接\n");
    printf("===========================================\n");
    
    while (!quit)
    {
        printf("> ");
        fflush(stdout);
        if (fgets(cmd, sizeof(cmd), stdin) == NULL) {
            // Buildroot 后台服务通常没有标准输入；EOF 不代表要停止监控。
            if (!quit && feof(stdin)) {
                printf("标准输入已关闭，继续后台运行；发送 SIGTERM 可正常退出。\n");
                while (!quit) sleep(1);
            }
            break;
        }
        
        // 去除换行符
        cmd[strcspn(cmd, "\r\n")] = 0;
        
        if (strcmp(cmd, "quit") == 0) {
            quit = true;
        } else if (strlen(cmd) > 0) {
            // 处理本地命令（格式与TCP相同）
            printf("本地命令: %s\n", cmd);
            if (strncmp(cmd, "RECT:", 5) == 0) {
                g_control->parseRectInfo(std::string(cmd));
            } else if (strncmp(cmd, "LIST:", 5) == 0) {
                g_control->parseObjList(std::string(cmd));
            } else {
                g_control->parseAndDispatch(std::string(cmd));
            }
        }
    }
	// 等待视频线程和TCP线程结束
    printf("正在停止视频线程...\n");
    g_video->stop();
    printf("视频线程已停止\n");

    printf("正在停止TCP线程...\n");
    g_tcp->stop();
    printf("TCP线程已停止\n");

    printf("正在停止UDP发现服务...\n");
    g_udp->stop();
    printf("UDP发现服务已停止\n");

    // 清理Control对象
    if (g_control) {
        delete g_control;
        g_control = nullptr;
        printf("Control模块已清理\n");
    }

    // 清理其他动态分配的对象
    if (g_tcp) {
        delete g_tcp;
        g_tcp = nullptr;
        printf("TCP模块已清理\n");
    }

    if (g_udp) {
        delete g_udp;
        g_udp = nullptr;
        printf("UDP发现模块已清理\n");
    }

    if (g_video) {
        delete g_video;
        g_video = nullptr;
        printf("Video模块已清理\n");
    }

    if (g_servo) {
        delete g_servo;
        g_servo = nullptr;
        printf("Servo模块已清理\n");
    }

    printf("程序已安全退出\n");
    return 0;
}
