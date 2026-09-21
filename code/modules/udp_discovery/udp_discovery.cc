// udp_discovery.cc - UDP广播设备发现模块实现
// 实现设备发现响应和心跳发送功能，对接QT服务端
#include "udp_discovery.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <errno.h>

// 构造函数
UdpDiscovery::UdpDiscovery(const std::string& deviceId, 
                           const std::string& deviceName,
                           const std::string& rtspUrl,
                           int port)
    : deviceId_(deviceId)
    , deviceName_(deviceName)
    , rtspUrl_(rtspUrl)
    , ipAddress_("")
    , manufacturer_("LuckfoxCamera")
    , model_("RV1106")
    , firmwareVersion_("1.0.0")
    , rtspPort_(554)
    , recordingHttpPort_(0)
    , status_(STATUS_ONLINE)
    , port_(port)
    , sockfd_(-1)
    , recvThread_(0)
    , heartbeatThread_(0)
    , running_(false)
    , hostCallback_(nullptr)
    , callbackUserData_(nullptr)
    , discoveredHostIp_("")
    , hostDiscovered_(false)
{
    pthread_mutex_init(&mutex_, NULL);
}

// 析构函数
UdpDiscovery::~UdpDiscovery() {
    stop();
    pthread_mutex_destroy(&mutex_);
}

// 初始化UDP socket
bool UdpDiscovery::init() {
    // 创建UDP套接字
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        perror("UDP socket创建失败");
        return false;
    }
    
    // 允许广播
    int broadcast_enable = 1;
    if (setsockopt(sockfd_, SOL_SOCKET, SO_BROADCAST, 
                   &broadcast_enable, sizeof(broadcast_enable)) < 0) {
        perror("设置SO_BROADCAST失败");
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }
    
    // 允许端口复用
    int reuse = 1;
    if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        perror("设置SO_REUSEADDR失败");
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }
    
    // 绑定端口
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port_);
    
    if (bind(sockfd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("UDP端口绑定失败");
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }
    
    printf("UDP发现服务初始化成功，端口: %d\n", port_);
    return true;
}

// 启动服务
bool UdpDiscovery::start() {
    if (sockfd_ < 0) {
        if (!init()) {
            return false;
        }
    }
    
    running_ = true;
    
    // 启动接收线程
    if (pthread_create(&recvThread_, NULL, recvThreadFunc, this) != 0) {
        perror("创建接收线程失败");
        running_ = false;
        return false;
    }
    
    // 启动心跳线程
    if (pthread_create(&heartbeatThread_, NULL, heartbeatThreadFunc, this) != 0) {
        perror("创建心跳线程失败");
        running_ = false;
        pthread_join(recvThread_, NULL);
        return false;
    }
    
    printf("UDP发现服务已启动\n");
    printf("  - 设备ID: %s\n", deviceId_.c_str());
    printf("  - 设备名称: %s\n", deviceName_.c_str());
    printf("  - RTSP URL: %s\n", rtspUrl_.c_str());
    return true;
}

// 停止服务
void UdpDiscovery::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    // 等待线程结束
    if (recvThread_) {
        pthread_join(recvThread_, NULL);
        recvThread_ = 0;
    }
    
    if (heartbeatThread_) {
        pthread_join(heartbeatThread_, NULL);
        heartbeatThread_ = 0;
    }
    
    // 关闭socket
    if (sockfd_ >= 0) {
        close(sockfd_);
        sockfd_ = -1;
    }
    
    printf("UDP发现服务已停止\n");
}

// 设置设备状态
void UdpDiscovery::setStatus(DeviceStatus status) {
    pthread_mutex_lock(&mutex_);
    status_ = status;
    pthread_mutex_unlock(&mutex_);
}

// 设置RTSP URL
void UdpDiscovery::setRtspUrl(const std::string& rtspUrl) {
    pthread_mutex_lock(&mutex_);
    rtspUrl_ = rtspUrl;
    pthread_mutex_unlock(&mutex_);
}

// 设置IP地址
void UdpDiscovery::setIpAddress(const std::string& ip) {
    pthread_mutex_lock(&mutex_);
    ipAddress_ = ip;
    pthread_mutex_unlock(&mutex_);
}

// 设置制造商
void UdpDiscovery::setManufacturer(const std::string& manufacturer) {
    pthread_mutex_lock(&mutex_);
    manufacturer_ = manufacturer;
    pthread_mutex_unlock(&mutex_);
}

// 设置型号
void UdpDiscovery::setModel(const std::string& model) {
    pthread_mutex_lock(&mutex_);
    model_ = model;
    pthread_mutex_unlock(&mutex_);
}

// 设置固件版本
void UdpDiscovery::setFirmwareVersion(const std::string& version) {
    pthread_mutex_lock(&mutex_);
    firmwareVersion_ = version;
    pthread_mutex_unlock(&mutex_);
}

// 设置RTSP端口
void UdpDiscovery::setRtspPort(int port) {
    pthread_mutex_lock(&mutex_);
    rtspPort_ = port;
    pthread_mutex_unlock(&mutex_);
}

void UdpDiscovery::setRecordingHttpPort(int port) {
    pthread_mutex_lock(&mutex_);
    recordingHttpPort_ = port;
    pthread_mutex_unlock(&mutex_);
}

// 设置上位机发现回调函数
void UdpDiscovery::setHostDiscoveredCallback(HostDiscoveredCallback callback, void* userData) {
    pthread_mutex_lock(&mutex_);
    hostCallback_ = callback;
    callbackUserData_ = userData;
    pthread_mutex_unlock(&mutex_);
}

// 获取最近发现的上位机IP地址
std::string UdpDiscovery::getDiscoveredHostIp() const {
    // 注意: const方法不能加锁，但读取字符串是原子的
    return discoveredHostIp_;
}

// 检查是否已发现上位机
bool UdpDiscovery::hasDiscoveredHost() const {
    return hostDiscovered_;
}

// 接收线程入口
void* UdpDiscovery::recvThreadFunc(void* arg) {
    UdpDiscovery* self = static_cast<UdpDiscovery*>(arg);
    self->recvLoop();
    return nullptr;
}

// 心跳线程入口
void* UdpDiscovery::heartbeatThreadFunc(void* arg) {
    UdpDiscovery* self = static_cast<UdpDiscovery*>(arg);
    self->heartbeatLoop();
    return nullptr;
}

// 接收处理主循环
void UdpDiscovery::recvLoop() {
    char buffer[UDP_BUFFER_SIZE];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    printf("UDP接收线程已启动，等待发现请求...\n");
    
    while (running_) {
        // 使用非阻塞接收，设置超时
        fd_set readfds;
        struct timeval timeout;
        FD_ZERO(&readfds);
        FD_SET(sockfd_, &readfds);
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        
        int ret = select(sockfd_ + 1, &readfds, NULL, NULL, &timeout);
        if (ret < 0) {
            if (errno != EINTR) {
                perror("select错误");
            }
            continue;
        } else if (ret == 0) {
            // 超时，继续循环
            continue;
        }
        
        // 接收数据
        ssize_t recv_len = recvfrom(sockfd_, buffer, UDP_BUFFER_SIZE - 1, 0,
                                     (struct sockaddr*)&client_addr, &addr_len);
        
        if (recv_len > 0) {
            buffer[recv_len] = '\0';
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
            
            printf("UDP收到 [%s:%d]: %s\n", 
                   client_ip, ntohs(client_addr.sin_port), buffer);
            
            // 检查是否为发现请求
            // 简单字符串匹配，避免引入JSON库
            if (strstr(buffer, "\"type\"") != NULL && 
                strstr(buffer, "\"discovery_request\"") != NULL) {
                printf("收到设备发现请求，发送响应...\n");
                printf("📍 上位机IP: %s\n", client_ip);
                
                // 保存上位机IP地址（但不触发连接）
                pthread_mutex_lock(&mutex_);
                discoveredHostIp_ = client_ip;
                hostDiscovered_ = true;
                pthread_mutex_unlock(&mutex_);
                
                // 构建并发送响应
                std::string response = buildDiscoveryResponse();
                if (sendBroadcast(response)) {
                    printf("✅ 已发送设备发现响应\n");
                } else {
                    printf("❌ 发送设备发现响应失败\n");
                }
            }
            // 检查是否为连接请求 🆕
            else if (strstr(buffer, "\"type\"") != NULL && 
                     strstr(buffer, "\"connection_request\"") != NULL) {
                printf("🔗 收到连接请求，准备建立TCP连接...\n");
                printf("📍 上位机IP: %s\n", client_ip);
                printf("📦 完整JSON数据: %s\n", buffer);  // 调试输出
                
                // 解析host_ip和tcp_port
                char* host_ip_ptr = strstr(buffer, "\"host_ip\"");
                char* tcp_port_ptr = strstr(buffer, "\"tcp_port\"");
                
                std::string hostIp = client_ip;  // 默认为UDP来源IP
                int tcpPort = 8890;              // 默认端口
                bool ip_parsed = false;
                
                // 1. 尝试解析JSON中的host_ip（优先级最高）
                if (host_ip_ptr) {
                    // 跳过 "host_ip"
                    char* value_start = host_ip_ptr + strlen("\"host_ip\"");
                    // 跳过空格和冒号
                    while (*value_start && (*value_start == ' ' || *value_start == ':')) value_start++;
                    // 跳过空格
                    while (*value_start && *value_start == ' ') value_start++;
                    
                    // 期待引号开始
                    if (*value_start == '"') {
                        value_start++;  // 跳过开始引号
                        char* value_end = strchr(value_start, '"');
                        if (value_end) {
                            int len = value_end - value_start;
                            if (len > 0 && len < 32) {
                                char ip_buf[32];
                                strncpy(ip_buf, value_start, len);
                                ip_buf[len] = '\0';
                                
                                // 简单验证是否为有效IP字符串（至少包含3个点）
                                int dots = 0;
                                for(int i=0; i<len; i++) {
                                    if(ip_buf[i] == '.') dots++;
                                }
                                
                                if (dots == 3) {
                                    hostIp = ip_buf;
                                    ip_parsed = true;
                                    printf("   ✅ 解析到JSON host_ip: %s\n", hostIp.c_str());
                                }
                            }
                        }
                    }
                }
                
                if (!ip_parsed) {
                    printf("   ℹ️  使用UDP来源IP作为备选: %s\n", hostIp.c_str());
                }
                
                // 尝试解析tcp_port
                // 格式: "tcp_port": 8080
                if (tcp_port_ptr) {
                    // 跳过 "tcp_port"
                    char* value_start = tcp_port_ptr + strlen("\"tcp_port\"");
                    // 跳过空格和冒号
                    while (*value_start && (*value_start == ' ' || *value_start == ':')) {
                        value_start++;
                    }
                    // 跳过空格
                    while (*value_start && *value_start == ' ') {
                        value_start++;
                    }
                    // 解析数字
                    tcpPort = atoi(value_start);
                    if (tcpPort > 0 && tcpPort < 65536) {
                        printf("   ✅ 解析到tcp_port: %d\n", tcpPort);
                    } else {
                        tcpPort = 8890;  // 无效端口，使用默认值
                    }
                }
                
                // 保存上位机信息并触发回调
                pthread_mutex_lock(&mutex_);
                discoveredHostIp_ = hostIp;
                hostDiscovered_ = true;
                HostDiscoveredCallback callback = hostCallback_;
                void* userData = callbackUserData_;
                pthread_mutex_unlock(&mutex_);
                
                // 触发回调，通知主程序建立TCP连接
                if (callback) {
                    printf("🎯 触发连接回调: %s:%d\n", hostIp.c_str(), tcpPort);
                    callback(hostIp.c_str(), tcpPort, userData);
                } else {
                    printf("⚠️  未设置连接回调函数\n");
                }
            }
        } else if (recv_len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("UDP接收错误");
        }
    }
    
    printf("UDP接收线程已退出\n");
}

// 心跳发送主循环
void UdpDiscovery::heartbeatLoop() {
    printf("UDP心跳线程已启动，间隔: %d秒\n", HEARTBEAT_INTERVAL);
    
    int counter = 0;
    while (running_) {
        sleep(1);
        counter++;
        
        if (counter >= HEARTBEAT_INTERVAL) {
            counter = 0;
            
            std::string heartbeat = buildHeartbeat();
            if (sendBroadcast(heartbeat)) {
                printf("❤ 心跳已发送\n");
            } else {
                printf("❌ 心跳发送失败\n");
            }
        }
    }
    
    printf("UDP心跳线程已退出\n");
}

// 构建发现响应JSON（手动拼接，避免引入JSON库）
std::string UdpDiscovery::buildDiscoveryResponse() {
    pthread_mutex_lock(&mutex_);
    
    char buffer[UDP_BUFFER_SIZE];
    
    // 基础字段
    int len = snprintf(buffer, sizeof(buffer),
        "{"
        "\"type\":\"discovery_response\","
        "\"device_id\":\"%s\","
        "\"device_name\":\"%s\","
        "\"rtsp_url\":\"%s\","
        "\"rtsp_port\":%d",
        deviceId_.c_str(),
        deviceName_.c_str(),
        rtspUrl_.c_str(),
        rtspPort_);
    
    // 可选IP地址字段
    if (!ipAddress_.empty()) {
        len += snprintf(buffer + len, sizeof(buffer) - len,
            ",\"ip_address\":\"%s\"", ipAddress_.c_str());
    }
    
    // 扩展字段
    len += snprintf(buffer + len, sizeof(buffer) - len,
        ",\"manufacturer\":\"%s\""
        ",\"model\":\"%s\""
        ",\"firmware_version\":\"%s\""
        ",\"recording_playback\":%s"
        ",\"recording_http_port\":%d"
        ",\"recording_api\":\"http://%s:%d/api/v1\""
        "}",
        manufacturer_.c_str(),
        model_.c_str(),
        firmwareVersion_.c_str(),
        recordingHttpPort_ > 0 ? "true" : "false",
        recordingHttpPort_, ipAddress_.c_str(), recordingHttpPort_);
    
    pthread_mutex_unlock(&mutex_);
    
    return std::string(buffer);
}

// 构建心跳消息JSON
std::string UdpDiscovery::buildHeartbeat() {
    pthread_mutex_lock(&mutex_);
    
    char buffer[256];
    snprintf(buffer, sizeof(buffer),
        "{"
        "\"type\":\"heartbeat\","
        "\"device_id\":\"%s\","
        "\"status\":\"%s\""
        "}",
        deviceId_.c_str(),
        getStatusString());
    
    pthread_mutex_unlock(&mutex_);
    
    return std::string(buffer);
}

// 发送UDP广播消息
bool UdpDiscovery::sendBroadcast(const std::string& message) {
    if (sockfd_ < 0) {
        return false;
    }
    
    struct sockaddr_in broadcast_addr;
    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_addr.s_addr = inet_addr("255.255.255.255");
    broadcast_addr.sin_port = htons(port_);
    
    ssize_t sent = sendto(sockfd_, message.c_str(), message.length(), 0,
                          (struct sockaddr*)&broadcast_addr, sizeof(broadcast_addr));
    
    if (sent < 0) {
        perror("UDP广播发送失败");
        return false;
    }
    
    return (size_t)sent == message.length();
}

// 获取设备状态字符串
const char* UdpDiscovery::getStatusString() const {
    switch (status_) {
        case STATUS_ONLINE:
            return "online";
        case STATUS_BUSY:
            return "busy";
        case STATUS_ERROR:
            return "error";
        default:
            return "unknown";
    }
}
