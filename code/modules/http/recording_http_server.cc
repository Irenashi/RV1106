#include "recording_http_server.h"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <map>
#include <netinet/in.h>
#include <sstream>
#include <sys/sendfile.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

namespace {
const size_t kMaxRequestBytes = 16 * 1024;
std::atomic<int> g_active_clients(0);

struct ClientContext {
    int fd;
    std::string directory;
};

struct RecordingInfo {
    std::string name;
    uint64_t start_ms;
    uint64_t end_ms;
    uint64_t size;
};

int integerSetting(const char* name, int fallback, int minimum, int maximum) {
    const char* value = getenv(name);
    if (!value || !*value) return fallback;
    char* end = nullptr;
    errno = 0;
    const long parsed = strtol(value, &end, 10);
    if (errno || end == value || *end || parsed < minimum || parsed > maximum) return fallback;
    return static_cast<int>(parsed);
}

std::string recordRoot() {
    const char* root = getenv("IRC_RECORD_ROOT");
    if (!root || !*root) root = getenv("IRC_RECORD_MOUNT");
    std::string result = root && *root ? root : "/root/record";
    while (result.size() > 1 && result.back() == '/') result.pop_back();
    return result;
}

std::string trim(const std::string& value) {
    const size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return std::string();
    const size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string lower(std::string value) {
    for (char& c : value) {
        if (c >= 'A' && c <= 'Z') c = char(c - 'A' + 'a');
    }
    return value;
}

bool sendAll(int fd, const void* data, size_t size) {
    const char* bytes = static_cast<const char*>(data);
    while (size) {
        const ssize_t sent = send(fd, bytes, size, MSG_NOSIGNAL);
        if (sent > 0) {
            bytes += sent;
            size -= static_cast<size_t>(sent);
        } else if (sent < 0 && errno == EINTR) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}

const char* reasonPhrase(int status) {
    switch (status) {
        case 200: return "OK";
        case 204: return "No Content";
        case 206: return "Partial Content";
        case 400: return "Bad Request";
        case 404: return "Not Found";
        case 405: return "Method Not Allowed";
        case 416: return "Range Not Satisfiable";
        case 431: return "Request Header Fields Too Large";
        case 500: return "Internal Server Error";
        case 503: return "Service Unavailable";
        default: return "Error";
    }
}

std::string commonHeaders(int status, const std::string& content_type,
                          uint64_t content_length, const std::string& extra = std::string()) {
    std::ostringstream out;
    out << "HTTP/1.1 " << status << ' ' << reasonPhrase(status) << "\r\n"
        << "Server: IRC-Recording/1.0\r\n"
        << "Connection: close\r\n"
        << "Access-Control-Allow-Origin: *\r\n"
        << "Content-Type: " << content_type << "\r\n"
        << "Content-Length: " << content_length << "\r\n"
        << extra << "\r\n";
    return out.str();
}

void sendText(int fd, int status, const std::string& body,
              const char* content_type = "application/json; charset=utf-8",
              const std::string& extra = std::string(), bool head_only = false) {
    const std::string header = commonHeaders(status, content_type, body.size(), extra);
    sendAll(fd, header.data(), header.size());
    if (!head_only && !body.empty()) sendAll(fd, body.data(), body.size());
}

bool parseUnsigned(const std::string& text, uint64_t* result) {
    if (text.empty() || text[0] == '-') return false;
    char* end = nullptr;
    errno = 0;
    const unsigned long long value = strtoull(text.c_str(), &end, 10);
    if (errno || end == text.c_str() || *end) return false;
    *result = static_cast<uint64_t>(value);
    return true;
}

bool leapYear(int year) {
    return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

int daysInMonth(int year, int month) {
    static const int days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month < 1 || month > 12) return 0;
    return month == 2 && leapYear(year) ? 29 : days[month - 1];
}

// Gregorian civil date -> days since 1970-01-01. Avoids timegm/TZ dependencies.
int64_t daysFromCivil(int year, unsigned month, unsigned day) {
    year -= month <= 2;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(year - era * 400);
    const unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return int64_t(era) * 146097 + doe - 719468;
}

bool dateToMs(int year, int month, int day, int hour, int minute, int second,
              int millisecond, uint64_t* result) {
    if (year < 1970 || year > 9999 || month < 1 || month > 12 || day < 1 ||
        day > daysInMonth(year, month) || hour < 0 || hour > 23 || minute < 0 ||
        minute > 59 || second < 0 || second > 60 || millisecond < 0 || millisecond > 999)
        return false;
    const int64_t seconds = daysFromCivil(year, unsigned(month), unsigned(day)) * 86400 +
                            hour * 3600 + minute * 60 + second;
    if (seconds < 0) return false;
    *result = uint64_t(seconds) * 1000 + unsigned(millisecond);
    return true;
}

bool parseRecordingName(const std::string& name, uint64_t* start_ms, uint64_t* end_ms) {
    int sy, sm, sd, sh, smin, ss, sms;
    int ey, em, ed, eh, emin, es, ems;
    int consumed = 0;
    const int matched = sscanf(name.c_str(),
        "clip_%d-%d-%d-%d-%d-%d-%d_%d-%d-%d-%d-%d-%d-%d.mp4%n",
        &sy, &sm, &sd, &sh, &smin, &ss, &sms,
        &ey, &em, &ed, &eh, &emin, &es, &ems, &consumed);
    return matched == 14 && consumed == static_cast<int>(name.size()) &&
           dateToMs(sy, sm, sd, sh, smin, ss, sms, start_ms) &&
           dateToMs(ey, em, ed, eh, emin, es, ems, end_ms) && *end_ms >= *start_ms;
}

std::map<std::string, std::string> parseQuery(const std::string& query) {
    std::map<std::string, std::string> result;
    size_t begin = 0;
    while (begin <= query.size()) {
        const size_t amp = query.find('&', begin);
        const std::string item = query.substr(begin, amp == std::string::npos ? std::string::npos : amp - begin);
        const size_t equal = item.find('=');
        if (equal != std::string::npos) result[item.substr(0, equal)] = item.substr(equal + 1);
        if (amp == std::string::npos) break;
        begin = amp + 1;
    }
    return result;
}

std::string recordingListJson(const std::string& directory, const std::string& query) {
    uint64_t from_ms = 0, to_ms = UINT64_MAX;
    int limit = 200;
    const std::map<std::string, std::string> params = parseQuery(query);
    auto it = params.find("start_ms");
    if (it != params.end() && !parseUnsigned(it->second, &from_ms)) from_ms = 0;
    it = params.find("end_ms");
    if (it != params.end() && !parseUnsigned(it->second, &to_ms)) to_ms = UINT64_MAX;
    it = params.find("limit");
    if (it != params.end()) {
        uint64_t requested = 0;
        if (parseUnsigned(it->second, &requested) && requested >= 1 && requested <= 1000)
            limit = static_cast<int>(requested);
    }

    std::vector<RecordingInfo> recordings;
    DIR* dir = opendir(directory.c_str());
    if (dir) {
        dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            const std::string name(entry->d_name);
            uint64_t start_ms = 0, end_ms = 0;
            if (!parseRecordingName(name, &start_ms, &end_ms) || end_ms < from_ms || start_ms > to_ms)
                continue;
            const std::string path = directory + "/" + name;
            struct stat info;
            if (lstat(path.c_str(), &info) != 0 || !S_ISREG(info.st_mode)) continue;
            recordings.push_back({name, start_ms, end_ms, uint64_t(info.st_size)});
        }
        closedir(dir);
    }
    std::sort(recordings.begin(), recordings.end(), [](const RecordingInfo& a, const RecordingInfo& b) {
        return a.start_ms > b.start_ms;
    });
    if (recordings.size() > static_cast<size_t>(limit)) recordings.resize(limit);

    std::ostringstream json;
    json << "{\"version\":1,\"timezone\":\"UTC\",\"count\":" << recordings.size()
         << ",\"recordings\":[";
    for (size_t i = 0; i < recordings.size(); ++i) {
        const RecordingInfo& item = recordings[i];
        if (i) json << ',';
        json << "{\"id\":\"" << item.name << "\",\"name\":\"" << item.name
             << "\",\"start_ms\":" << item.start_ms << ",\"end_ms\":" << item.end_ms
             << ",\"duration_ms\":" << (item.end_ms - item.start_ms)
             << ",\"size_bytes\":" << item.size
             << ",\"url\":\"/recordings/" << item.name << "\"}";
    }
    json << "]}";
    return json.str();
}

bool parseRange(const std::string& value, uint64_t file_size,
                uint64_t* first, uint64_t* last) {
    if (value.compare(0, 6, "bytes=") != 0 || value.find(',') != std::string::npos || !file_size)
        return false;
    const std::string range = trim(value.substr(6));
    const size_t dash = range.find('-');
    if (dash == std::string::npos) return false;
    const std::string left = trim(range.substr(0, dash));
    const std::string right = trim(range.substr(dash + 1));
    if (left.empty()) {
        uint64_t suffix = 0;
        if (!parseUnsigned(right, &suffix) || !suffix) return false;
        if (suffix > file_size) suffix = file_size;
        *first = file_size - suffix;
        *last = file_size - 1;
        return true;
    }
    if (!parseUnsigned(left, first) || *first >= file_size) return false;
    if (right.empty()) {
        *last = file_size - 1;
        return true;
    }
    if (!parseUnsigned(right, last) || *last < *first) return false;
    if (*last >= file_size) *last = file_size - 1;
    return true;
}

void serveFile(int client, const std::string& directory, const std::string& name,
               const std::string& range, bool head_only) {
    uint64_t ignored_start = 0, ignored_end = 0;
    if (!parseRecordingName(name, &ignored_start, &ignored_end)) {
        sendText(client, 404, "{\"error\":\"recording_not_found\"}");
        return;
    }
    const std::string path = directory + "/" + name;
    const int file = open(path.c_str(), O_RDONLY | O_NOFOLLOW);
    if (file < 0) {
        sendText(client, 404, "{\"error\":\"recording_not_found\"}");
        return;
    }
    struct stat info;
    if (fstat(file, &info) != 0 || !S_ISREG(info.st_mode) || info.st_size < 0) {
        close(file);
        sendText(client, 404, "{\"error\":\"recording_not_found\"}");
        return;
    }
    const uint64_t file_size = static_cast<uint64_t>(info.st_size);
    uint64_t first = 0;
    uint64_t last = file_size ? file_size - 1 : 0;
    int status = 200;
    std::ostringstream extra;
    extra << "Accept-Ranges: bytes\r\nCache-Control: no-store\r\n";
    if (!range.empty()) {
        if (!parseRange(range, file_size, &first, &last)) {
            close(file);
            extra << "Content-Range: bytes */" << file_size << "\r\n";
            sendText(client, 416, "{\"error\":\"range_not_satisfiable\"}",
                     "application/json; charset=utf-8", extra.str(), head_only);
            return;
        }
        status = 206;
        extra << "Content-Range: bytes " << first << '-' << last << '/' << file_size << "\r\n";
    }
    const uint64_t length = file_size ? last - first + 1 : 0;
    const std::string header = commonHeaders(status, "video/mp4", length, extra.str());
    if (!sendAll(client, header.data(), header.size()) || head_only || !length) {
        close(file);
        return;
    }
    off_t offset = static_cast<off_t>(first);
    uint64_t remaining = length;
    while (remaining) {
        const size_t block = static_cast<size_t>(std::min<uint64_t>(remaining, 1024 * 1024));
        const ssize_t sent = sendfile(client, file, &offset, block);
        if (sent > 0) remaining -= static_cast<uint64_t>(sent);
        else if (sent < 0 && errno == EINTR) continue;
        else break;
    }
    close(file);
}

void handleClient(int client, const std::string& directory) {
    timeval timeout = {30, 0};
    setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    std::string request;
    char buffer[2048];
    while (request.find("\r\n\r\n") == std::string::npos && request.size() < kMaxRequestBytes) {
        const ssize_t received = recv(client, buffer, sizeof(buffer), 0);
        if (received > 0) request.append(buffer, static_cast<size_t>(received));
        else if (received < 0 && errno == EINTR) continue;
        else break;
    }
    if (request.size() >= kMaxRequestBytes) {
        sendText(client, 431, "{\"error\":\"headers_too_large\"}");
        return;
    }
    const size_t line_end = request.find("\r\n");
    if (line_end == std::string::npos) {
        sendText(client, 400, "{\"error\":\"bad_request\"}");
        return;
    }
    std::istringstream first_line(request.substr(0, line_end));
    std::string method, target, version;
    first_line >> method >> target >> version;
    if ((method != "GET" && method != "HEAD" && method != "OPTIONS") ||
        target.empty() || version.compare(0, 5, "HTTP/") != 0) {
        sendText(client, method.empty() ? 400 : 405,
                 method.empty() ? "{\"error\":\"bad_request\"}" : "{\"error\":\"method_not_allowed\"}",
                 "application/json; charset=utf-8", "Allow: GET, HEAD, OPTIONS\r\n");
        return;
    }
    std::map<std::string, std::string> headers;
    size_t begin = line_end + 2;
    while (begin < request.size()) {
        const size_t end = request.find("\r\n", begin);
        if (end == std::string::npos || end == begin) break;
        const std::string line = request.substr(begin, end - begin);
        const size_t colon = line.find(':');
        if (colon != std::string::npos)
            headers[lower(trim(line.substr(0, colon)))] = trim(line.substr(colon + 1));
        begin = end + 2;
    }
    if (method == "OPTIONS") {
        sendText(client, 204, std::string(), "text/plain",
                 "Allow: GET, HEAD, OPTIONS\r\nAccess-Control-Allow-Methods: GET, HEAD, OPTIONS\r\n");
        return;
    }
    const bool head_only = method == "HEAD";
    const size_t question = target.find('?');
    const std::string path = target.substr(0, question);
    const std::string query = question == std::string::npos ? std::string() : target.substr(question + 1);
    if (path == "/api/v1/status") {
        sendText(client, 200, "{\"service\":\"recording\",\"version\":1,\"status\":\"ok\"}",
                 "application/json; charset=utf-8", std::string(), head_only);
    } else if (path == "/api/v1/recordings") {
        sendText(client, 200, recordingListJson(directory, query),
                 "application/json; charset=utf-8", "Cache-Control: no-store\r\n", head_only);
    } else if (path.compare(0, 12, "/recordings/") == 0) {
        serveFile(client, directory, path.substr(12), headers["range"], head_only);
    } else {
        sendText(client, 404, "{\"error\":\"not_found\"}",
                 "application/json; charset=utf-8", std::string(), head_only);
    }
}

void* clientThread(void* opaque) {
    ClientContext* context = static_cast<ClientContext*>(opaque);
    handleClient(context->fd, context->directory);
    shutdown(context->fd, SHUT_RDWR);
    close(context->fd);
    delete context;
    --g_active_clients;
    return nullptr;
}
} // namespace

RecordingHttpServer::RecordingHttpServer()
    : root_path_(recordRoot()), directory_(root_path_ + "/recordings"),
      port_(integerSetting("IRC_RECORD_HTTP_PORT", 18080, 1, 65535)),
      max_clients_(integerSetting("IRC_RECORD_HTTP_MAX_CLIENTS", 4, 1, 32)),
      listen_fd_(-1), thread_(0), running_(false) {}

RecordingHttpServer::~RecordingHttpServer() {
    stop();
}

bool RecordingHttpServer::start() {
    if (running_.load()) return true;
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) { perror("[RecordingHTTP] socket"); return false; }
    int reuse = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<uint16_t>(port_));
    if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
        listen(listen_fd_, max_clients_) != 0) {
        perror("[RecordingHTTP] bind/listen");
        close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    running_.store(true);
    if (pthread_create(&thread_, nullptr, acceptThreadEntry, this) != 0) {
        perror("[RecordingHTTP] 创建监听线程失败");
        running_.store(false);
        close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    printf("[RecordingHTTP] 已启动: 0.0.0.0:%d, 目录 %s, 最大连接 %d\n",
           port_, directory_.c_str(), max_clients_);
    return true;
}

void RecordingHttpServer::stop() {
    if (!running_.exchange(false)) return;
    if (listen_fd_ >= 0) {
        shutdown(listen_fd_, SHUT_RDWR);
        close(listen_fd_);
        listen_fd_ = -1;
    }
    if (thread_) {
        pthread_join(thread_, nullptr);
        thread_ = 0;
    }
    printf("[RecordingHTTP] 已停止\n");
}

void* RecordingHttpServer::acceptThreadEntry(void* arg) {
    static_cast<RecordingHttpServer*>(arg)->acceptLoop();
    return nullptr;
}

void RecordingHttpServer::acceptLoop() {
    while (running_.load()) {
        const int client = accept(listen_fd_, nullptr, nullptr);
        if (client < 0) {
            if (errno == EINTR) continue;
            if (running_.load()) perror("[RecordingHTTP] accept");
            break;
        }
        if (g_active_clients.load() >= max_clients_) {
            sendText(client, 503, "{\"error\":\"too_many_clients\"}");
            close(client);
            continue;
        }
        ++g_active_clients;
        ClientContext* context = new ClientContext{client, directory_};
        pthread_t worker;
        pthread_attr_t attributes;
        pthread_attr_init(&attributes);
        pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
        if (pthread_create(&worker, &attributes, clientThread, context) != 0) {
            --g_active_clients;
            close(client);
            delete context;
        }
        pthread_attr_destroy(&attributes);
    }
}
