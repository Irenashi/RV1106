#include "recording.h"
#include "../../../3rdparty/minimp4/minimp4.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <limits>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/time.h>
#include <unistd.h>

namespace {
const uint64_t MiB = 1024ULL * 1024ULL;

uint64_t setting(const char* name, uint64_t fallback) {
    const char* value = getenv(name);
    if (!value || !*value || *value == '-') return fallback;
    char* end = nullptr;
    errno = 0;
    unsigned long long parsed = strtoull(value, &end, 10);
    return errno == 0 && end != value && *end == '\0' ? parsed : fallback;
}

uint64_t nowMs() {
    timeval tv;
    gettimeofday(&tv, nullptr);
    return uint64_t(tv.tv_sec) * 1000 + tv.tv_usec / 1000;
}

uint64_t monotonicUs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
}

std::string timeName(uint64_t milliseconds) {
    time_t seconds = milliseconds / 1000;
    tm utc;
    gmtime_r(&seconds, &utc);
    char buffer[40];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", &utc);
    char suffix[8];
    snprintf(suffix, sizeof(suffix), "-%03u", unsigned(milliseconds % 1000));
    return std::string(buffer) + suffix;
}

bool endsWith(const std::string& value, const char* suffix) {
    const size_t length = strlen(suffix);
    return value.size() >= length && value.compare(value.size() - length, length, suffix) == 0;
}

size_t startCode(const unsigned char* data, size_t size, size_t pos, size_t* code_size);

int mp4Write(int64_t offset, const void* data, size_t size, void* token) {
    FILE* file = static_cast<FILE*>(token);
    if (offset < 0 || fseeko(file, static_cast<off_t>(offset), SEEK_SET) != 0) return -1;
    return fwrite(data, 1, size, file) == size ? 0 : -1;
}

// MP4 中的 AVC 样本使用 4 字节 NAL 长度前缀，参数集单独存入 avcC。
bool makeAvcSample(const unsigned char* data, size_t size,
                   std::vector<unsigned char>& sample) {
    bool has_slice = false;
    size_t code_size = 0;
    for (size_t pos = startCode(data, size, 0, &code_size); pos < size;) {
        const size_t nal = pos + code_size;
        size_t next_code_size = 0;
        const size_t next = startCode(data, size, nal, &next_code_size);
        if (nal < next) {
            const unsigned type = data[nal] & 0x1f;
            if (type == 1 || type == 5) has_slice = true;
            if (type != 7 && type != 8 && type != 9 && type != 12) {
                const size_t length = next - nal;
                if (length > UINT32_MAX) return false;
                const uint32_t n = static_cast<uint32_t>(length);
                sample.push_back(static_cast<unsigned char>(n >> 24));
                sample.push_back(static_cast<unsigned char>(n >> 16));
                sample.push_back(static_cast<unsigned char>(n >> 8));
                sample.push_back(static_cast<unsigned char>(n));
                sample.insert(sample.end(), data + nal, data + next);
            }
        }
        pos = next;
        code_size = next_code_size;
    }
    return has_slice && !sample.empty();
}

bool parameterNal(const std::vector<unsigned char>& bytes, const unsigned char** nal, int* length) {
    size_t code_size = 0;
    const size_t pos = startCode(bytes.data(), bytes.size(), 0, &code_size);
    if (pos != 0 || bytes.size() <= code_size || bytes.size() - code_size > INT_MAX) return false;
    *nal = bytes.data() + code_size;
    *length = static_cast<int>(bytes.size() - code_size);
    return true;
}

uint32_t readBe32(const unsigned char* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
           (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

uint64_t readBe64(const unsigned char* p) {
    return (uint64_t(readBe32(p)) << 32) | readBe32(p + 4);
}

// 编码输出与当前 RTSP 输入相同，按 Annex B 起始码识别 IDR/SPS/PPS。
size_t startCode(const unsigned char* data, size_t size, size_t pos, size_t* code_size) {
    for (; pos + 3 < size; ++pos) {
        if (data[pos] == 0 && data[pos + 1] == 0) {
            if (data[pos + 2] == 1) { *code_size = 3; return pos; }
            if (pos + 4 < size && data[pos + 2] == 0 && data[pos + 3] == 1) {
                *code_size = 4;
                return pos;
            }
        }
    }
    return size;
}

bool containsIdr(const unsigned char* data, size_t size) {
    size_t code_size = 0;
    for (size_t pos = startCode(data, size, 0, &code_size); pos < size;) {
        const size_t nal = pos + code_size;
        if (nal < size && (data[nal] & 0x1f) == 5) return true;
        pos = startCode(data, size, nal, &code_size);
    }
    return false;
}

bool containsParameterSet(const unsigned char* data, size_t size) {
    size_t code_size = 0;
    for (size_t pos = startCode(data, size, 0, &code_size); pos < size;) {
        const size_t nal = pos + code_size;
        if (nal < size && ((data[nal] & 0x1f) == 7 || (data[nal] & 0x1f) == 8)) return true;
        pos = startCode(data, size, nal, &code_size);
    }
    return false;
}

void collectParameterSets(const unsigned char* data, size_t size,
                          std::vector<unsigned char>& sps, std::vector<unsigned char>& pps) {
    size_t code_size = 0;
    for (size_t pos = startCode(data, size, 0, &code_size); pos < size;) {
        const size_t nal = pos + code_size;
        size_t next_code_size = 0;
        const size_t next = startCode(data, size, nal, &next_code_size);
        if (nal < size) {
            const unsigned type = data[nal] & 0x1f;
            if (type == 7) sps.assign(data + pos, data + next);
            if (type == 8) pps.assign(data + pos, data + next);
        }
        pos = next;
        code_size = next_code_size;
    }
}

struct Clip {
    std::string path;
    uint64_t size;
    time_t modified;
};
} // namespace

Recording::Recording(int width, int height)
    : root_path_(getenv("IRC_RECORD_ROOT") ? getenv("IRC_RECORD_ROOT") :
                 (getenv("IRC_RECORD_MOUNT") ? getenv("IRC_RECORD_MOUNT") : "/root/record")),
      max_bytes_(setting("IRC_RECORD_MAX_MB", 4096) * MiB),
      min_free_bytes_(setting("IRC_RECORD_MIN_FREE_MB", 1024) * MiB),
      max_age_seconds_(setting("IRC_RECORD_MAX_AGE_DAYS", 7) * 86400ULL),
      segment_seconds_(setting("IRC_RECORD_SEGMENT_SECONDS", 60)),
      sync_seconds_(setting("IRC_RECORD_SYNC_SECONDS", 10)),
      max_queue_bytes_(size_t(setting("IRC_RECORD_QUEUE_MB", 8) * MiB)),
      thread_(0), running_(false), dropping_(true), discontinuity_pending_(true),
      queued_bytes_(0), file_(nullptr), mux_(nullptr), track_id_(-1),
      width_(width), height_(height), last_pts_(0), last_duration_(3000), samples_(0),
      segment_start_ms_(0), segment_last_ms_(0),
      segment_bytes_(0), last_maintenance_ms_(0), last_sync_ms_(0),
      interval_mux_us_(0), interval_mux_samples_(0) {
    while (root_path_.size() > 1 && root_path_.back() == '/') root_path_.pop_back();
    directory_ = root_path_ + "/recordings";
    pthread_mutex_init(&mutex_, nullptr);
    pthread_cond_init(&cond_, nullptr);
}

Recording::~Recording() {
    stop();
    pthread_cond_destroy(&cond_);
    pthread_mutex_destroy(&mutex_);
}

bool Recording::prepareDirectory() {
    // Buildroot 根文件系统本身在 SD 卡上，录像根目录无需单独挂载。
    if (root_path_.empty() || root_path_[0] != '/' || root_path_ == "/") {
        fprintf(stderr, "[Recording] 录像根目录必须是非根目录的绝对路径\n");
        return false;
    }
    if (mkdir(root_path_.c_str(), 0755) != 0 && errno != EEXIST) {
        perror("[Recording] 创建录像根目录失败");
        return false;
    }
    struct stat root;
    if (stat(root_path_.c_str(), &root) != 0 || !S_ISDIR(root.st_mode)) return false;
    if (mkdir(directory_.c_str(), 0755) != 0 && errno != EEXIST) {
        perror("[Recording] 创建录像目录失败");
        return false;
    }
    struct stat dir;
    if (stat(directory_.c_str(), &dir) != 0 || !S_ISDIR(dir.st_mode)) {
        fprintf(stderr, "[Recording] 录像目录无效: %s\n", directory_.c_str());
        return false;
    }
    return true;
}

bool Recording::start() {
    if (running_) return true;
    if (!max_bytes_ || !segment_seconds_ || !max_queue_bytes_ || !prepareDirectory()) return false;
    dropping_ = true;
    discontinuity_pending_ = true;
    last_maintenance_ms_ = 0;
    // 分片 MP4 在异常断电后仍有完整片段；启动时截掉未写完的尾部并恢复。
    DIR* dir = opendir(directory_.c_str());
    if (dir) {
        dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            const std::string name(entry->d_name);
            if (name.compare(0, 5, "clip_") == 0 && endsWith(name, ".part"))
                recoverSegment(name);
        }
        closedir(dir);
    }
    running_ = true;
    if (pthread_create(&thread_, nullptr, threadEntry, this) != 0) {
        running_ = false;
        return false;
    }
    printf("[Recording] %s, 最大 %llu MiB, 最低空闲 %llu MiB, 保留 %llu 天, 切片 %llu 秒, 同步 %llu 秒\n",
           directory_.c_str(), (unsigned long long)(max_bytes_ / MiB),
           (unsigned long long)(min_free_bytes_ / MiB),
           (unsigned long long)(max_age_seconds_ / 86400),
           (unsigned long long)segment_seconds_, (unsigned long long)sync_seconds_);
    return true;
}

bool Recording::recoverSegment(const std::string& name) {
    const std::string path = directory_ + "/" + name;
    FILE* input = fopen(path.c_str(), "rb+");
    if (!input) return false;
    if (fseeko(input, 0, SEEK_END) != 0) { fclose(input); return false; }
    const off_t file_size = ftello(input);
    off_t offset = 0;
    off_t valid_end = 0;
    off_t pending_moof = -1;
    bool has_moov = false;
    bool has_media = false;
    unsigned char header[16];
    while (offset >= 0 && offset + 8 <= file_size) {
        if (fseeko(input, offset, SEEK_SET) != 0 || fread(header, 1, 8, input) != 8) break;
        uint64_t box_size = readBe32(header);
        size_t header_size = 8;
        if (box_size == 1) {
            if (offset + 16 > file_size || fread(header + 8, 1, 8, input) != 8) break;
            box_size = readBe64(header + 8);
            header_size = 16;
        } else if (box_size == 0) {
            box_size = uint64_t(file_size - offset);
        }
        if (box_size < header_size || box_size > uint64_t(file_size - offset)) break;
        const uint32_t type = readBe32(header + 4);
        const uint32_t moov = 0x6d6f6f76; // 'moov'
        const uint32_t moof = 0x6d6f6f66; // 'moof'
        const uint32_t mdat = 0x6d646174; // 'mdat'
        if (type == moov) has_moov = true;
        if (type == moof) pending_moof = offset;
        if (type == mdat && pending_moof >= 0) {
            has_media = true;
            pending_moof = -1;
        }
        offset += static_cast<off_t>(box_size);
        valid_end = offset;
    }
    // 不保留没有配对 mdat 的最后一个 moof。
    if (pending_moof >= 0) valid_end = pending_moof;
    struct stat info;
    const bool recoverable = has_moov && has_media && valid_end > 0 &&
                             fstat(fileno(input), &info) == 0 &&
                             ftruncate(fileno(input), valid_end) == 0;
    fclose(input);
    if (!recoverable) {
        unlink(path.c_str());
        fprintf(stderr, "[Recording] 临时录像无法恢复，已删除: %s\n", path.c_str());
        return false;
    }
    const std::string start = name.substr(0, name.size() - strlen(".part"));
    const std::string final_path = directory_ + "/" + start + "_" +
                                   timeName(uint64_t(info.st_mtime) * 1000) + ".mp4";
    if (rename(path.c_str(), final_path.c_str()) != 0) {
        perror("[Recording] 恢复录像重命名失败");
        return false;
    }
    printf("[Recording] 已恢复断电录像: %s\n", final_path.c_str());
    return true;
}

void Recording::stop() {
    pthread_mutex_lock(&mutex_);
    running_ = false;
    pthread_cond_broadcast(&cond_);
    pthread_mutex_unlock(&mutex_);
    if (thread_) {
        pthread_join(thread_, nullptr);
        thread_ = 0;
    }
}

void Recording::enqueue(const void* data, size_t size, uint64_t pts) {
    if (!data || !size) return;
    const unsigned char* bytes = static_cast<const unsigned char*>(data);
    const bool idr = containsIdr(bytes, size);
    pthread_mutex_lock(&mutex_);
    if (!running_) { pthread_mutex_unlock(&mutex_); return; }
    if (size > max_queue_bytes_ || queued_bytes_ + size > max_queue_bytes_) {
        queue_.clear();
        queued_bytes_ = 0;
        dropping_ = true;
        discontinuity_pending_ = true;
        fprintf(stderr, "[Recording] 写入队列已满，丢帧并等待下一关键帧\n");
    }
    if (dropping_ && !idr && !containsParameterSet(bytes, size)) {
        pthread_mutex_unlock(&mutex_);
        return;
    }
    if (size > max_queue_bytes_) { pthread_mutex_unlock(&mutex_); return; }
    Packet packet;
    packet.bytes.assign(bytes, bytes + size);
    packet.pts = pts;
    packet.wall_ms = nowMs();
    packet.idr = idr;
    packet.discontinuity = discontinuity_pending_;
    discontinuity_pending_ = false;
    if (idr) dropping_ = false;
    queued_bytes_ += size;
    queue_.push_back(std::move(packet));
    pthread_cond_signal(&cond_);
    pthread_mutex_unlock(&mutex_);
}

void* Recording::threadEntry(void* arg) {
    static_cast<Recording*>(arg)->run();
    return nullptr;
}

bool Recording::maintainStorage(uint64_t now_ms, size_t incoming) {
    DIR* dir = opendir(directory_.c_str());
    if (!dir) return false;
    std::vector<Clip> clips;
    uint64_t total = segment_bytes_;
    dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name(entry->d_name);
        if (name.compare(0, 5, "clip_") != 0 ||
            (!endsWith(name, ".mp4") && !endsWith(name, ".h264"))) continue;
        std::string path = directory_ + "/" + name;
        struct stat info;
        if (lstat(path.c_str(), &info) != 0 || !S_ISREG(info.st_mode)) continue;
        clips.push_back({path, uint64_t(info.st_size), info.st_mtime});
        total += uint64_t(info.st_size);
    }
    closedir(dir);
    std::sort(clips.begin(), clips.end(), [](const Clip& a, const Clip& b) {
        return a.path < b.path;
    });
    struct statvfs fs;
    if (statvfs(directory_.c_str(), &fs) != 0) return false;
    uint64_t free_bytes = uint64_t(fs.f_bavail) * fs.f_frsize;
    const time_t now_seconds = now_ms / 1000;
    for (const Clip& clip : clips) {
        bool too_old = max_age_seconds_ && now_seconds > clip.modified &&
                       uint64_t(now_seconds - clip.modified) > max_age_seconds_;
        bool too_large = total + incoming > max_bytes_;
        bool too_full = free_bytes < min_free_bytes_ + incoming;
        if (!too_old && !too_large && !too_full) continue;
        if (unlink(clip.path.c_str()) == 0) {
            total -= clip.size;
            free_bytes += clip.size;
            printf("[Recording] 删除旧录像: %s\n", clip.path.c_str());
        }
    }
    return total + incoming <= max_bytes_ && free_bytes >= min_free_bytes_ + incoming;
}

bool Recording::openSegment(const Packet& packet) {
    if (!packet.idr || sps_.empty() || pps_.empty() ||
        !maintainStorage(packet.wall_ms, packet.bytes.size())) return false;
    segment_start_ms_ = packet.wall_ms;
    segment_last_ms_ = packet.wall_ms;
    segment_bytes_ = 0;
    temporary_path_ = directory_ + "/clip_" + timeName(segment_start_ms_) + ".part";
    file_ = fopen(temporary_path_.c_str(), "wb+");
    if (!file_) { perror("[Recording] 打开录像失败"); return false; }
    // 分片 MP4 持续写入 moof/mdat，异常断电后可恢复完整片段。
    mux_ = MP4E_open(0, 1, file_, mp4Write);
    MP4E_track_t track = {};
    track.track_media_kind = e_video;
    track.object_type_indication = MP4_OBJECT_TYPE_AVC;
    track.time_scale = 90000;
    track.default_duration = 3000; // 30 fps; 实际帧时长由 VENC PTS 计算。
    track.u.v.width = width_;
    track.u.v.height = height_;
    const unsigned char* sps = nullptr;
    const unsigned char* pps = nullptr;
    int sps_length = 0, pps_length = 0;
    if (!mux_ || !parameterNal(sps_, &sps, &sps_length) ||
        !parameterNal(pps_, &pps, &pps_length) ||
        (track_id_ = MP4E_add_track(mux_, &track)) < 0 ||
        MP4E_set_sps(mux_, track_id_, sps, sps_length) != MP4E_STATUS_OK ||
        MP4E_set_pps(mux_, track_id_, pps, pps_length) != MP4E_STATUS_OK) {
        fprintf(stderr, "[Recording] MP4 封装初始化失败\n");
        closeSegment(false);
        return false;
    }
    last_pts_ = 0;
    last_duration_ = 3000;
    samples_ = 0;
    last_sync_ms_ = 0;
    interval_mux_us_ = 0;
    interval_mux_samples_ = 0;
    return true;
}

void Recording::closeSegment(bool keep) {
    if (!file_) return;
    bool good = true;
    if (mux_) {
        if (MP4E_close(mux_) != MP4E_STATUS_OK) good = false;
        mux_ = nullptr;
    }
    if (fflush(file_) != 0 || fsync(fileno(file_)) != 0) good = false;
    if (fseeko(file_, 0, SEEK_END) == 0) {
        const off_t actual = ftello(file_);
        if (actual >= 0) segment_bytes_ = static_cast<uint64_t>(actual);
    }
    if (fclose(file_) != 0) good = false;
    file_ = nullptr;
    if (keep && good && samples_) {
        const std::string final_path = directory_ + "/clip_" + timeName(segment_start_ms_) +
                                       "_" + timeName(segment_last_ms_) + ".mp4";
        if (rename(temporary_path_.c_str(), final_path.c_str()) == 0)
            printf("[Recording] 已保存 %s (%llu bytes)\n", final_path.c_str(),
                   (unsigned long long)segment_bytes_);
        else perror("[Recording] 录像重命名失败");
    } else {
        unlink(temporary_path_.c_str());
    }
    segment_bytes_ = 0;
    samples_ = 0;
    track_id_ = -1;
    temporary_path_.clear();
}

bool Recording::writePacket(const Packet& packet) {
    const uint64_t mux_begin_us = monotonicUs();
    std::vector<unsigned char> sample;
    if (!makeAvcSample(packet.bytes.data(), packet.bytes.size(), sample)) return true;
    if (file_ && packet.idr && packet.wall_ms >= segment_start_ms_ &&
        packet.wall_ms - segment_start_ms_ >= segment_seconds_ * 1000)
        closeSegment(true);
    if (!file_ && !openSegment(packet)) return false;
    if (packet.wall_ms < last_maintenance_ms_ || packet.wall_ms - last_maintenance_ms_ >= 5000) {
        last_maintenance_ms_ = packet.wall_ms;
        if (!maintainStorage(packet.wall_ms, packet.bytes.size())) {
            fprintf(stderr, "[Recording] SD 卡空间不足，暂停录像写入\n");
            closeSegment(true);
            return false;
        }
    }
    uint32_t duration = last_duration_;
    if (samples_ && packet.pts > last_pts_) {
        const uint64_t delta_us = packet.pts - last_pts_;
        if (delta_us <= 1000000) {
            duration = static_cast<uint32_t>((delta_us * 90 + 500) / 1000);
            if (!duration) duration = 1;
        }
    }
    int mux_result = MP4E_STATUS_BAD_ARGUMENTS;
    if (sample.size() <= INT_MAX) {
        mux_result = MP4E_put_sample(mux_, track_id_, sample.data(),
                static_cast<int>(sample.size()), static_cast<int>(duration),
                packet.idr ? MP4E_SAMPLE_RANDOM_ACCESS : MP4E_SAMPLE_DEFAULT);
    }
    interval_mux_us_ += monotonicUs() - mux_begin_us;
    ++interval_mux_samples_;
    if (mux_result != MP4E_STATUS_OK) {
        fprintf(stderr, "[Recording] SD 卡写入 MP4 失败\n");
        closeSegment(false);
        return false;
    }
    segment_bytes_ += sample.size();
    last_pts_ = packet.pts;
    last_duration_ = duration;
    ++samples_;
    segment_last_ms_ = packet.wall_ms;
    if (sync_seconds_ && (!last_sync_ms_ || packet.wall_ms < last_sync_ms_ ||
        packet.wall_ms - last_sync_ms_ >= sync_seconds_ * 1000)) {
        const uint64_t flush_begin_us = monotonicUs();
        const int flush_result = fflush(file_);
        const uint64_t flush_end_us = monotonicUs();
        const int sync_result = flush_result == 0 ? fdatasync(fileno(file_)) : -1;
        const uint64_t sync_end_us = monotonicUs();
        if (flush_result != 0 || sync_result != 0) {
            perror("[Recording] MP4 同步到 SD 卡失败");
            closeSegment(false);
            return false;
        }
        printf("[Recording] MP4性能: 封装 %llu 帧/%llu us, fflush %llu us, "
               "fdatasync %llu us, 本次同步总计 %llu us\n",
               (unsigned long long)interval_mux_samples_,
               (unsigned long long)interval_mux_us_,
               (unsigned long long)(flush_end_us - flush_begin_us),
               (unsigned long long)(sync_end_us - flush_end_us),
               (unsigned long long)(sync_end_us - flush_begin_us));
        interval_mux_us_ = 0;
        interval_mux_samples_ = 0;
        last_sync_ms_ = packet.wall_ms;
    }
    return true;
}

void Recording::run() {
    bool need_idr = true;
    for (;;) {
        pthread_mutex_lock(&mutex_);
        while (running_ && queue_.empty()) pthread_cond_wait(&cond_, &mutex_);
        if (!running_ && queue_.empty()) { pthread_mutex_unlock(&mutex_); break; }
        Packet packet = std::move(queue_.front());
        queued_bytes_ -= packet.bytes.size();
        queue_.pop_front();
        pthread_mutex_unlock(&mutex_);
        collectParameterSets(packet.bytes.data(), packet.bytes.size(), sps_, pps_);
        if (packet.discontinuity) {
            closeSegment(true);
            need_idr = true;
        }
        if (need_idr && !packet.idr) continue;
        if (writePacket(packet)) need_idr = false;
        else need_idr = true;
    }
    closeSegment(true);
}
