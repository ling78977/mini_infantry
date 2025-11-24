#include "util/logger.hpp"
#include <iostream>

int main() {
    // Set log file
    util::Logger::get_instance().set_log_file("test_log.log");
    // Set global log level
    util::Logger::get_instance().set_log_level(util::LogLevel::INFO);

    LOG_DEBUG << "This is a debug message. It should not appear by default.";
    LOG_INFO << "This is an info message.";
    LOG_WARN << "This is a warning message.";
    LOG_ERROR << "This is an error message.";
    LOG_FATAL << "This is a fatal message. Application might crash!";

    util::Logger::get_instance().set_log_level(util::LogLevel::DEBUG);
    LOG_DEBUG << "This is a debug message after changing log level.";

    std::cout << "Check test_log.log for file output." << std::endl;

    return 0;
}