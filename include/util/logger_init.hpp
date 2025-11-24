#ifndef MINI_INFANTRY_LOGGER_INIT_HPP
#define MINI_INFANTRY_LOGGER_INIT_HPP

#include <memory>
#include <spdlog/spdlog.h>

namespace util {
    // Function to initialize the global logger
    void init_logger(const std::string& log_file_name, const std::string& program_name, spdlog::level::level_enum console_sink_level=spdlog::level::info, spdlog::level::level_enum global_logger_level=spdlog::level::info);

    // Function to get the global logger instance
    std::shared_ptr<spdlog::logger> get_logger();
} // namespace util

#endif // MINI_INFANTRY_LOGGER_INIT_HPP