#ifndef RECORDING_H
#define RECORDING_H

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <deque>
#include <string>
#include <vector>

struct MP4E_mux_tag;

// 将 VENC 已编码的 H.264 Annex B 数据封装为 MP4，不额外占用编码通道。
// 所有磁盘操作只在录像线程进行；编码线程只复制数据到有上限的队列。
class Recording {
public:
    Recording(int width, int height);
    ~Recording();

    bool start();
    void stop();
    void enqueue(const void* data, size_t size, uint64_t pts);

private:
    struct Packet {
        std::vector<unsigned char> bytes;
        uint64_t pts;
        uint64_t wall_ms;
        bool idr;
        bool discontinuity;
    };

    static void* threadEntry(void* arg);
    void run();
    bool prepareDirectory();
    bool maintainStorage(uint64_t now_ms, size_t incoming);
    bool recoverSegment(const std::string& name);
    bool openSegment(const Packet& packet);
    void closeSegment(bool keep);
    bool writePacket(const Packet& packet);

    std::string root_path_;
    std::string directory_;
    uint64_t max_bytes_;
    uint64_t min_free_bytes_;
    uint64_t max_age_seconds_;
    uint64_t segment_seconds_;
    uint64_t sync_seconds_;
    size_t max_queue_bytes_;

    pthread_t thread_;
    pthread_mutex_t mutex_;
    pthread_cond_t cond_;
    bool running_;
    bool dropping_;
    bool discontinuity_pending_;
    size_t queued_bytes_;
    std::deque<Packet> queue_;

    FILE* file_;
    MP4E_mux_tag* mux_;
    int track_id_;
    int width_;
    int height_;
    uint64_t last_pts_;
    uint32_t last_duration_;
    uint64_t samples_;
    std::string temporary_path_;
    uint64_t segment_start_ms_;
    uint64_t segment_last_ms_;
    uint64_t segment_bytes_;
    uint64_t last_maintenance_ms_;
    uint64_t last_sync_ms_;
    uint64_t interval_mux_us_;
    uint64_t interval_mux_samples_;
    std::vector<unsigned char> sps_;
    std::vector<unsigned char> pps_;
};

#endif
