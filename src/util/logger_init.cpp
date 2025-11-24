#include "util/logger_init.hpp"
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h> // For console output

namespace util {

    // Global logger instance
    static std::shared_ptr<spdlog::logger> global_logger;

    void init_logger(const std::string& log_file_name, const std::string& program_name, spdlog::level::level_enum console_sink_level,
    spdlog::level::level_enum global_logger_level) {
        if (!global_logger) {
            try {
                // Create a daily file sink with 0 hour and 0 minute (midnight rotation)
                // Rotate every 2 days (48 hours)
                auto daily_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(log_file_name, 0, 0);
                
                // Optional: Add a console sink for immediate feedback
                auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
                console_sink->set_level(console_sink_level);

                std::vector<spdlog::sink_ptr> sinks;
                sinks.push_back(console_sink);
                sinks.push_back(daily_sink);

                global_logger = std::make_shared<spdlog::logger>(program_name, begin(sinks), end(sinks)); // Use program_name here
                global_logger->set_level(global_logger_level); // Default logging level
                global_logger->flush_on(spdlog::level::info); // Flush on info and higher
                spdlog::set_default_logger(global_logger);
                spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v"); // Custom pattern with logger name

                spdlog::info("Logger initialized successfully. Logging to {}", log_file_name);
            } catch (const spdlog::spdlog_ex& ex) {
                // Fallback to basic logger if daily file sink fails
                spdlog::error("Logger initialization failed: {}", ex.what());
                global_logger = spdlog::stdout_color_mt(program_name); // Use program_name here
                global_logger->set_level(spdlog::level::info);
                spdlog::set_default_logger(global_logger);
            }
        }
    }

    std::shared_ptr<spdlog::logger> get_logger() {
        // If logger is not initialized, return a default console logger
        if (!global_logger) {
            global_logger = spdlog::stdout_color_mt("default_logger");
            global_logger->set_level(spdlog::level::info);
        }
        return global_logger;
    }

} // namespace util