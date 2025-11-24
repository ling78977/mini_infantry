#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <mutex>

namespace util {

enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class Logger; // Forward declaration

class LogStream {
public:
    LogStream(Logger& logger, LogLevel level);
    ~LogStream();

    template<typename T>
    LogStream& operator<<(const T& msg);

    // Special overload for manipulators like std::endl
    LogStream& operator<<(std::ostream& (*manip)(std::ostream&));

private:
    Logger& logger_;
    LogLevel level_;
    std::stringstream stream_;
};

class Logger {
public:
    static Logger& get_instance();

    void set_log_file(const std::string& filename);
    void set_log_level(LogLevel level);
    LogLevel get_log_level() const { return log_level_; } // Add getter for log_level_

    // Method to get a LogStream for logging
    LogStream get_log_stream(LogLevel level);

    // Internal logging methods called by LogStream
    void write_log(LogLevel level, const std::string& message);

private:
    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void log_to_console(LogLevel level, const std::string& message);
    void log_to_file(LogLevel level, const std::string& message);
    std::string get_timestamp();
    std::string get_level_string(LogLevel level);
    std::string get_level_color(LogLevel level);

    std::ofstream log_file_;
    LogLevel log_level_ = LogLevel::INFO; // Default log level
    std::mutex mutex_;
};

// Global log macros
#define LOG_DEBUG util::Logger::get_instance().get_log_stream(util::LogLevel::DEBUG)
#define LOG_INFO  util::Logger::get_instance().get_log_stream(util::LogLevel::INFO)
#define LOG_WARN  util::Logger::get_instance().get_log_stream(util::LogLevel::WARN)
#define LOG_ERROR util::Logger::get_instance().get_log_stream(util::LogLevel::ERROR)
#define LOG_FATAL util::Logger::get_instance().get_log_stream(util::LogLevel::FATAL)

// Template function definition must be in the header after Logger is fully defined
template<typename T>
LogStream& LogStream::operator<<(const T& msg) {
    if (level_ >= logger_.get_log_level()) {
        stream_ << msg;
    }
    return *this;
}

} // namespace util

#endif // LOGGER_HPP