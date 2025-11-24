#include "util/logger.hpp"
#include <iostream>
#include <map>

namespace util {

// --- Logger Implementation ---

// Static member initialization
Logger& Logger::get_instance() {
    static Logger instance;
    return instance;
}

Logger::Logger() {
    // Default log file name
    set_log_file("application.log");
}

Logger::~Logger() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void Logger::set_log_file(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (log_file_.is_open()) {
        log_file_.close();
    }
    log_file_.open(filename, std::ios::app);
    if (!log_file_.is_open()) {
        std::cerr << "Failed to open log file: " << filename << std::endl;
    }
}

void Logger::set_log_level(LogLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_level_ = level;
}

LogStream Logger::get_log_stream(LogLevel level) {
    return LogStream(*this, level);
}

void Logger::write_log(LogLevel level, const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (level >= log_level_) {
        log_to_console(level, message);
        log_to_file(level, message);
    }
}

std::string Logger::get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

std::string Logger::get_level_string(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO";
        case LogLevel::WARN:  return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default:              return "UNKNOWN";
    }
}

// ANSI escape codes for colors
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


std::string Logger::get_level_color(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return CYAN;
        case LogLevel::INFO:  return GREEN;
        case LogLevel::WARN:  return YELLOW;
        case LogLevel::ERROR: return RED;
        case LogLevel::FATAL: return BOLDRED;
        default:              return RESET;
    }
}

void Logger::log_to_console(LogLevel level, const std::string& message) {
    std::cout << get_level_color(level)
              << "[" << get_timestamp() << "]"
              << "[" << get_level_string(level) << "] "
              << message << RESET << std::endl;
}

void Logger::log_to_file(LogLevel level, const std::string& message) {
    if (log_file_.is_open()) {
        log_file_ << "[" << get_timestamp() << "]"
                  << "[" << get_level_string(level) << "] "
                  << message << std::endl;
    }
}

// --- LogStream Implementation ---

LogStream::LogStream(Logger& logger, LogLevel level)
    : logger_(logger), level_(level) {}

LogStream::~LogStream() {
    if (level_ >= logger_.get_log_level()) {
        logger_.write_log(level_, stream_.str());
    }
}

LogStream& LogStream::operator<<(std::ostream& (*manip)(std::ostream&)) {
    if (level_ >= logger_.get_log_level()) {
        manip(stream_);
    }
    return *this;
}

} // namespace util