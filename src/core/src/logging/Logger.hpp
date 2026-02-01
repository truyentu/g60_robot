/**
 * @file Logger.hpp
 * @brief Logging framework wrapper using spdlog
 */

#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>
#include <string>

namespace robot_controller {

class Logger {
public:
    /**
     * Initialize the logging system
     * @param log_file Path to log file
     * @param level Log level (trace, debug, info, warn, error)
     * @param max_size Maximum file size in bytes (default 10MB)
     * @param max_files Maximum number of rotated files
     */
    static void init(const std::string& log_file = "logs/core.log",
                     const std::string& level = "info",
                     size_t max_size = 10 * 1024 * 1024,
                     size_t max_files = 5);

    /**
     * Get the logger instance
     */
    static std::shared_ptr<spdlog::logger> get();

private:
    static std::shared_ptr<spdlog::logger> s_logger;
    static bool s_initialized;
};

} // namespace robot_controller

// Convenience macros
#define LOG_TRACE(...) ::robot_controller::Logger::get()->trace(__VA_ARGS__)
#define LOG_DEBUG(...) ::robot_controller::Logger::get()->debug(__VA_ARGS__)
#define LOG_INFO(...)  ::robot_controller::Logger::get()->info(__VA_ARGS__)
#define LOG_WARN(...)  ::robot_controller::Logger::get()->warn(__VA_ARGS__)
#define LOG_ERROR(...) ::robot_controller::Logger::get()->error(__VA_ARGS__)
