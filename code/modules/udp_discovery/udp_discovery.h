// udp_discovery.h - UDP广播设备发现模块头文件
// 用于响应上位机的设备发现请求和发送心跳
#ifndef UDP_DISCOVERY_H
#define UDP_DISCOVERY_H

#include <string>
#include <pthread.h>

// 默认UDP发现端口
#define UDP_DISCOVERY_PORT 8888
// 心跳发送间隔（秒）
#define HEARTBEAT_INTERVAL 15
// 接收缓冲区大小
#define UDP_BUFFER_SIZE 1024

// 设备状态枚举
enum DeviceStatus {
    STATUS_ONLINE = 0,
    STATUS_BUSY,
    STATUS_ERROR
};

// 上位机发现回调函数类型
// @param hostIp       上位机IP地址
// @param hostPort     上位机端口
// @param userData     用户自定义数据指针
typedef void (*HostDiscoveredCallback)(const char* hostIp, int hostPort, void* userData);

// UdpDiscovery类：封装UDP广播发现功能
class UdpDiscovery {
public:
    // 构造函数
    // @param deviceId     设备唯一标识符
    // @param deviceName   设备显示名称
    // @param rtspUrl      RTSP流地址
    // @param port         UDP发现端口（默认8888）
    UdpDiscovery(const std::string& deviceId, 
                 const std::string& deviceName,
                 const std::string& rtspUrl,
                 int port = UDP_DISCOVERY_PORT);
    
    // 析构函数
    ~UdpDiscovery();
    
    // 初始化UDP socket
    bool init();
    
    // 启动服务（接收线程 + 心跳线程）
    bool start();
    
    // 停止服务
    void stop();
    
    // 查询服务运行状态
    bool isRunning() const { return running_; }
    
    // 设置设备状态
    void setStatus(DeviceStatus status);
    
    // 动态更新RTSP URL（如运行时端口变化）
    void setRtspUrl(const std::string& rtspUrl);
    
    // 设置设备IP地址（可选，不设置则由接收方从UDP包获取）
    void setIpAddress(const std::string& ip);
    
    // 设置扩展字段
    void setManufacturer(const std::string& manufacturer);
    void setModel(const std::string& model);
    void setFirmwareVersion(const std::string& version);
    void setRtspPort(int port);
    void setRecordingHttpPort(int port);
    
    // 设置上位机发现回调函数
    // 当收到上位机的discovery_request时会调用此回调
    void setHostDiscoveredCallback(HostDiscoveredCallback callback, void* userData = nullptr);
    
    // 获取最近发现的上位机IP地址
    // @return 上位机IP，如果未发现则返回空字符串
    std::string getDiscoveredHostIp() const;
    
    // 检查是否已发现上位机
    bool hasDiscoveredHost() const;

private:
    // 接收线程入口函数
    static void* recvThreadFunc(void* arg);
    // 心跳线程入口函数
    static void* heartbeatThreadFunc(void* arg);
    
    // 接收处理主循环
    void recvLoop();
    // 心跳发送主循环
    void heartbeatLoop();
    
    // 构建发现响应JSON
    std::string buildDiscoveryResponse();
    // 构建心跳消息JSON
    std::string buildHeartbeat();
    
    // 发送UDP广播消息
    bool sendBroadcast(const std::string& message);
    
    // 获取设备状态字符串
    const char* getStatusString() const;
    
    // 设备配置
    std::string deviceId_;
    std::string deviceName_;
    std::string rtspUrl_;
    std::string ipAddress_;
    std::string manufacturer_;
    std::string model_;
    std::string firmwareVersion_;
    int rtspPort_;
    int recordingHttpPort_;
    DeviceStatus status_;
    
    // 网络配置
    int port_;
    int sockfd_;
    
    // 线程管理
    pthread_t recvThread_;
    pthread_t heartbeatThread_;
    bool running_;
    
    // 线程同步
    pthread_mutex_t mutex_;
    
    // 上位机发现回调
    HostDiscoveredCallback hostCallback_;
    void* callbackUserData_;
    
    // 最近发现的上位机IP
    std::string discoveredHostIp_;
    bool hostDiscovered_;
};

#endif // UDP_DISCOVERY_H
