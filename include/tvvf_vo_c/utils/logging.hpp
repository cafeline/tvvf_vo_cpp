#ifndef TVVF_VO_C_UTILS_LOGGING_HPP_
#define TVVF_VO_C_UTILS_LOGGING_HPP_

#include <string>
#include <fstream>
#include <memory>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <atomic>

namespace tvvf_vo_c {
namespace logging {

/**
 * @brief ログレベル列挙型
 */
enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARN = 2,
    ERROR = 3,
    FATAL = 4
};

/**
 * @brief ログメッセージ構造体
 */
struct LogMessage {
    LogLevel level;
    std::string timestamp;
    std::string category;
    std::string message;
    std::string file;
    int line;
    std::thread::id thread_id;
};

/**
 * @brief ログ出力インターフェース
 */
class LogSink {
public:
    virtual ~LogSink() = default;
    virtual void write(const LogMessage& msg) = 0;
    virtual void flush() = 0;
};

/**
 * @brief コンソール出力用シンク
 */
class ConsoleSink : public LogSink {
public:
    ConsoleSink(bool colored = true);
    void write(const LogMessage& msg) override;
    void flush() override;

private:
    bool colored_;
    std::string get_color_code(LogLevel level) const;
    std::string get_level_string(LogLevel level) const;
};

/**
 * @brief ファイル出力用シンク
 */
class FileSink : public LogSink {
public:
    explicit FileSink(const std::string& filename, bool append = true);
    ~FileSink();
    
    void write(const LogMessage& msg) override;
    void flush() override;
    
    bool is_open() const;

private:
    std::ofstream file_;
    std::mutex mutex_;
};

/**
 * @brief 回転ファイル出力用シンク
 */
class RotatingFileSink : public LogSink {
public:
    RotatingFileSink(const std::string& base_filename, 
                     size_t max_file_size, 
                     size_t max_files = 5);
    ~RotatingFileSink();
    
    void write(const LogMessage& msg) override;
    void flush() override;

private:
    std::string base_filename_;
    size_t max_file_size_;
    size_t max_files_;
    size_t current_size_;
    std::ofstream current_file_;
    std::mutex mutex_;
    
    void rotate_files();
    std::string get_filename(size_t index) const;
};

/**
 * @brief 非同期ログ出力クラス
 */
class AsyncLogger {
public:
    AsyncLogger();
    ~AsyncLogger();
    
    void add_sink(std::shared_ptr<LogSink> sink);
    void set_level(LogLevel level);
    void set_flush_interval(int milliseconds);
    
    void log(LogLevel level, const std::string& category, 
             const std::string& message, const std::string& file, int line);
    
    void flush();
    void stop();

private:
    std::vector<std::shared_ptr<LogSink>> sinks_;
    std::queue<LogMessage> message_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    std::atomic<bool> running_;
    std::atomic<LogLevel> min_level_;
    int flush_interval_ms_;
    
    void worker_function();
    std::string format_message(const LogMessage& msg) const;
};

/**
 * @brief グローバルロガーインスタンス
 */
class Logger {
public:
    static Logger& instance();
    
    void add_console_sink(bool colored = true);
    void add_file_sink(const std::string& filename, bool append = true);
    void add_rotating_file_sink(const std::string& base_filename, 
                               size_t max_file_size, size_t max_files = 5);
    
    void set_level(LogLevel level);
    void set_flush_interval(int milliseconds);
    
    void log(LogLevel level, const std::string& category, 
             const std::string& message, const std::string& file, int line);
    
    void flush();

private:
    Logger();
    ~Logger();
    
    std::unique_ptr<AsyncLogger> async_logger_;
    static std::once_flag initialized_;
};

/**
 * @brief パフォーマンス監視用ログ
 */
class PerformanceLogger {
public:
    struct Metrics {
        double computation_time_ms;
        double memory_usage_mb;
        size_t objects_processed;
        std::string additional_info;
    };
    
    static PerformanceLogger& instance();
    
    void log_metrics(const std::string& category, const Metrics& metrics);
    void set_output_file(const std::string& filename);
    void set_csv_format(bool enable);
    
private:
    PerformanceLogger();
    
    std::ofstream output_file_;
    std::mutex mutex_;
    bool csv_format_;
    bool header_written_;
    
    void write_csv_header();
};

// ログ出力マクロ
#define TVVF_LOG_DEBUG(category, message) \
    tvvf_vo_c::logging::Logger::instance().log( \
        tvvf_vo_c::logging::LogLevel::DEBUG, category, message, __FILE__, __LINE__)

#define TVVF_LOG_INFO(category, message) \
    tvvf_vo_c::logging::Logger::instance().log( \
        tvvf_vo_c::logging::LogLevel::INFO, category, message, __FILE__, __LINE__)

#define TVVF_LOG_WARN(category, message) \
    tvvf_vo_c::logging::Logger::instance().log( \
        tvvf_vo_c::logging::LogLevel::WARN, category, message, __FILE__, __LINE__)

#define TVVF_LOG_ERROR(category, message) \
    tvvf_vo_c::logging::Logger::instance().log( \
        tvvf_vo_c::logging::LogLevel::ERROR, category, message, __FILE__, __LINE__)

#define TVVF_LOG_FATAL(category, message) \
    tvvf_vo_c::logging::Logger::instance().log( \
        tvvf_vo_c::logging::LogLevel::FATAL, category, message, __FILE__, __LINE__)

// パフォーマンスログマクロ
#define TVVF_PERF_LOG(category, metrics) \
    tvvf_vo_c::logging::PerformanceLogger::instance().log_metrics(category, metrics)

// 文字列フォーマット用ヘルパー
template<typename... Args>
std::string format_string(const std::string& format, Args... args) {
    int size = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;
    if (size <= 0) return "";
    
    std::unique_ptr<char[]> buf(new char[size]);
    std::snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1);
}

} // namespace logging
} // namespace tvvf_vo_c

#endif // TVVF_VO_C_UTILS_LOGGING_HPP_