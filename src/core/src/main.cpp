/**
 * @file main.cpp
 * @brief Robot Controller Core - Entry Point
 */

#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

#include "logging/Logger.hpp"
#include "config/ConfigManager.hpp"
#include "ipc/IpcServer.hpp"
#include "ipc/Message.hpp"

using namespace robot_controller;
using namespace robot_controller::config;
using namespace robot_controller::ipc;

// Global flag for graceful shutdown
std::atomic<bool> g_running{true};

void signalHandler(int signal) {
    LOG_INFO("Received signal {}, shutting down...", signal);
    g_running = false;
}

int main(int argc, char* argv[]) {
    // Setup signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Determine config directory (relative to executable or from arg)
    std::string config_dir = "../../config";
    if (argc > 1) {
        config_dir = argv[1];
    }

    // Initialize logging (basic setup, will reconfigure after loading config)
    Logger::init("../../logs/core.log", "debug");

    LOG_INFO("========================================");
    LOG_INFO("Robot Controller Core v1.0.0");
    LOG_INFO("========================================");
    LOG_INFO("Config directory: {}", config_dir);

    // Load configuration
    auto& config = ConfigManager::instance();
    if (!config.loadAll(config_dir)) {
        LOG_ERROR("Failed to load configuration files");
        LOG_ERROR("Make sure robot_config.yaml and system_config.yaml exist in: {}", config_dir);
        return 1;
    }

    // Reconfigure logger based on loaded config
    const auto& logConfig = config.systemConfig().logging;
    Logger::init(
        "../../" + logConfig.file,
        logConfig.level,
        logConfig.max_size_mb * 1024 * 1024,
        logConfig.max_files
    );

    LOG_INFO("Configuration loaded successfully");
    LOG_INFO("Robot: {} ({})", config.robotConfig().name, config.robotConfig().model);
    LOG_INFO("Joints: {}", config.robotConfig().numJoints());

    // Get IPC config
    const auto& ipcConfig = config.systemConfig().ipc;
    std::string rep_addr = "tcp://" + ipcConfig.bind_address + ":" + std::to_string(ipcConfig.rep_port);
    std::string pub_addr = "tcp://" + ipcConfig.bind_address + ":" + std::to_string(ipcConfig.pub_port);

    // Create IPC server
    IpcServer server(rep_addr, pub_addr);

    // Register message handlers
    server.registerHandler(MessageType::GET_STATUS,
        [&config](const Message& request) -> nlohmann::json {
            LOG_DEBUG("Handling GET_STATUS request");
            const auto& robotConfig = config.robotConfig();
            return {
                {"state", "IDLE"},
                {"mode", "MANUAL"},
                {"joints", robotConfig.home_position},
                {"tcp_position", {500.0, 0.0, 600.0, 0.0, 180.0, 0.0}},
                {"homed", false},
                {"enabled", false},
                {"errors", nlohmann::json::array()}
            };
        });

    server.registerHandler(MessageType::GET_JOINT_POSITIONS,
        [&config](const Message& request) -> nlohmann::json {
            LOG_DEBUG("Handling GET_JOINT_POSITIONS request");
            const auto& robotConfig = config.robotConfig();
            return {
                {"joints", robotConfig.home_position},
                {"unit", "degrees"}
            };
        });

    server.registerHandler(MessageType::GET_CONFIG,
        [&config](const Message& request) -> nlohmann::json {
            LOG_DEBUG("Handling GET_CONFIG request");
            return nlohmann::json::parse(config.robotConfigToJson());
        });

    // Start server
    if (!server.start()) {
        LOG_ERROR("Failed to start IPC server");
        return 1;
    }

    LOG_INFO("IPC Server running on ports {} (REP) and {} (PUB)",
             ipcConfig.rep_port, ipcConfig.pub_port);
    LOG_INFO("Press Ctrl+C to exit");

    // Main loop - publish status periodically
    const auto& controlConfig = config.systemConfig().control;
    int status_interval_ms = 1000 / controlConfig.status_publish_hz;
    auto last_status = std::chrono::steady_clock::now();

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status);

        if (elapsed.count() >= status_interval_ms) {
            // Publish status
            const auto& robotConfig = config.robotConfig();
            nlohmann::json status = {
                {"state", "IDLE"},
                {"mode", "MANUAL"},
                {"joints", robotConfig.home_position},
                {"tcp_position", {500.0, 0.0, 600.0, 0.0, 180.0, 0.0}},
                {"homed", false},
                {"enabled", false},
                {"errors", nlohmann::json::array()}
            };
            server.publishStatus(status);
            last_status = now;
        }

        // Sleep to prevent busy loop
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Shutdown
    LOG_INFO("Stopping server...");
    server.stop();

    LOG_INFO("Robot Controller Core shutdown complete");
    return 0;
}
