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
#include "ipc/IpcServer.hpp"
#include "ipc/Message.hpp"

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

    // Initialize logging
    robot_controller::Logger::init("../../logs/core.log", "debug");
    LOG_INFO("========================================");
    LOG_INFO("Robot Controller Core v1.0.0 starting...");
    LOG_INFO("========================================");

    // Create IPC server
    robot_controller::ipc::IpcServer server("tcp://*:5555", "tcp://*:5556");

    // Register message handlers
    server.registerHandler(robot_controller::ipc::MessageType::GET_STATUS,
        [](const robot_controller::ipc::Message& request) -> nlohmann::json {
            LOG_DEBUG("Handling GET_STATUS request");
            return {
                {"state", "IDLE"},
                {"mode", "MANUAL"},
                {"joints", {0.0, -45.0, 90.0, 0.0, 45.0, 0.0}},
                {"tcp_position", {500.0, 0.0, 600.0, 0.0, 180.0, 0.0}},
                {"homed", false},
                {"enabled", false},
                {"errors", nlohmann::json::array()}
            };
        });

    server.registerHandler(robot_controller::ipc::MessageType::GET_JOINT_POSITIONS,
        [](const robot_controller::ipc::Message& request) -> nlohmann::json {
            LOG_DEBUG("Handling GET_JOINT_POSITIONS request");
            return {
                {"joints", {0.0, -45.0, 90.0, 0.0, 45.0, 0.0}},
                {"unit", "degrees"}
            };
        });

    // Start server
    if (!server.start()) {
        LOG_ERROR("Failed to start IPC server");
        return 1;
    }

    LOG_INFO("IPC Server running on ports 5555 (REP) and 5556 (PUB)");
    LOG_INFO("Press Ctrl+C to exit");

    // Main loop - publish status periodically
    int status_interval_ms = 100;  // 10 Hz
    auto last_status = std::chrono::steady_clock::now();

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status);

        if (elapsed.count() >= status_interval_ms) {
            // Publish status
            nlohmann::json status = {
                {"state", "IDLE"},
                {"mode", "MANUAL"},
                {"joints", {0.0, -45.0, 90.0, 0.0, 45.0, 0.0}},
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
