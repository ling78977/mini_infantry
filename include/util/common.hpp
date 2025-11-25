#ifndef MINI_INFANTRY_COMMON_HPP
#define MINI_INFANTRY_COMMON_HPP

#include <spdlog/spdlog.h>
#include <string>

namespace util {
    // 根据完整路径获取程序名（不带扩展名）
    inline std::string getProgramName(const std::string& path) {
        if (path.empty()) {
            spdlog::warn("Path is empty, returning default program name 'unknown_program'.");
            return "unknown_program";
        }

        size_t lastSlash = path.find_last_of("/\\");
        std::string filename = (lastSlash == std::string::npos) ? path : path.substr(lastSlash + 1);

        if (filename.empty()) {
            spdlog::warn("Could not extract filename from path '{}', returning default program name 'unknown_program'.", path);            
            return "unknown_program";
        }

        size_t dotPos = filename.find_last_of('.');
        if (dotPos != std::string::npos) {
            filename = filename.substr(0, dotPos);
        }
        return filename;
    }
} // namespace util

#endif