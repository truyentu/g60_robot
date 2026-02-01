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
#include "controller/RobotController.hpp"

// Global controller pointer for signal handler
robot_controller::RobotController* g_controller = nullptr;

void signalHandler(int signal) {
    LOG_INFO("Received signal {}, shutting down...", signal);
    if (g_controller) {
        g_controller->stop();
    }
}

int main(int argc, char* argv[]) {
    // Setup signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Determine config directory
    std::string configDir = "../../config";
    if (argc > 1) {
        configDir = argv[1];
    }

    // Initialize logging
    robot_controller::Logger::init("../../logs/core.log", "debug");

    LOG_INFO("========================================");
    LOG_INFO("Robot Controller Core v1.0.0");
    LOG_INFO("========================================");

    // Create and initialize controller
    robot_controller::RobotController controller;
    g_controller = &controller;

    if (!controller.initialize(configDir)) {
        LOG_ERROR("Failed to initialize controller");
        return 1;
    }

    // Start controller
    if (!controller.start()) {
        LOG_ERROR("Failed to start controller");
        return 1;
    }

    LOG_INFO("Robot Controller running. Press Ctrl+C to exit.");

    // Wait for shutdown
    while (controller.isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LOG_INFO("Robot Controller Core shutdown complete");
    return 0;
}
