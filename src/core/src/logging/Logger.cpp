/**
 * @file Logger.cpp
 * @brief Logger implementation
 */

#include "Logger.hpp"
#include <vector>
#include <filesystem>
#include <iostream>

namespace robot_controller {

std::shared_ptr<spdlog::logger> Logger::s_logger = nullptr;
bool Logger::s_initialized = false;

void Logger::init(const std::string& log_file,
                  const std::string& level,
                  size_t max_size,
                  size_t max_files) {
    if (s_initialized) {
        return;
    }

    try {
        // Create logs directory if needed
        std::filesystem::path log_path(log_file);
        if (log_path.has_parent_path()) {
            std::filesystem::create_directories(log_path.parent_path());
        }

        // Create sinks
        std::vector<spdlog::sink_ptr> sinks;

        // Console sink (colored)
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
        sinks.push_back(console_sink);

        // File sink (rotating)
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_file, max_size, max_files);
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
        sinks.push_back(file_sink);

        // Create logger
        s_logger = std::make_shared<spdlog::logger>("robot_core", sinks.begin(), sinks.end());

        // Set level
        if (level == "trace") s_logger->set_level(spdlog::level::trace);
        else if (level == "debug") s_logger->set_level(spdlog::level::debug);
        else if (level == "info") s_logger->set_level(spdlog::level::info);
        else if (level == "warn") s_logger->set_level(spdlog::level::warn);
        else if (level == "error") s_logger->set_level(spdlog::level::err);
        else s_logger->set_level(spdlog::level::info);

        // Flush on warn or above
        s_logger->flush_on(spdlog::level::warn);

        // Register as default
        spdlog::set_default_logger(s_logger);

        s_initialized = true;

    } catch (const spdlog::spdlog_ex& ex) {
        std::cerr << "Logger initialization failed: " << ex.what() << std::endl;
    }
}

std::shared_ptr<spdlog::logger> Logger::get() {
    if (!s_initialized) {
        init(); // Initialize with defaults
    }
    return s_logger;
}

} // namespace robot_controller
