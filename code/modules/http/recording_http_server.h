#ifndef RECORDING_HTTP_SERVER_H
#define RECORDING_HTTP_SERVER_H

#include <atomic>
#include <pthread.h>
#include <stdint.h>
#include <string>

// 局域网录像回放 HTTP 服务。提供录像列表、HEAD 和单 Range 文件读取。
// 每个连接由短生命周期的独立线程处理，避免文件发送阻塞监听线程。
class RecordingHttpServer {
public:
    RecordingHttpServer();
    ~RecordingHttpServer();

    bool start();
    void stop();
    int port() const { return port_; }
    bool isRunning() const { return running_.load(); }

private:
    static void* acceptThreadEntry(void* arg);
    void acceptLoop();

    std::string root_path_;
    std::string directory_;
    int port_;
    int max_clients_;
    int listen_fd_;
    pthread_t thread_;
    std::atomic<bool> running_;
};

#endif
