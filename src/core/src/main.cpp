/**
 * @file main.cpp
 * @brief Robot Controller Core - Entry Point
 */

#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <cstdlib>

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

    // Determine workspace and config directories
    std::string configDir = "config";
    std::string workspaceDir = "";

    // Parse command line args
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.rfind("--workspace=", 0) == 0) {
            workspaceDir = arg.substr(12);
        } else {
            configDir = arg;
        }
    }

    // Try ROBOT_WORKSPACE environment variable
    if (workspaceDir.empty()) {
        const char* envWs = std::getenv("ROBOT_WORKSPACE");
        if (envWs && std::ifstream(std::string(envWs) + "/Config/system.yaml").good()) {
            workspaceDir = envWs;
        }
    }

    // Try to find workspace/ relative to executable
    if (workspaceDir.empty()) {
        const char* wsPaths[] = {
            "workspace",
            "../../workspace",
            "../../../workspace",
            "../../../../workspace",
            "../../../../../workspace",
            "../../../../../../workspace",
            "../../../../../../../workspace"
        };
        for (const char* path : wsPaths) {
            if (std::filesystem::exists(std::string(path) + "/Config")) {
                workspaceDir = path;
                break;
            }
        }
    }

    // Fallback: find config directory (backward compatible)
    if (workspaceDir.empty()) {
        const char* configPaths[] = {
            "config",           // Local (VS working dir)
            "../../config",     // From build/bin/Release
            "../../../config",  // From build/bin
            "../../../../config" // Alternative
        };
        for (const char* path : configPaths) {
            std::string testPath = std::string(path) + "/robot_config.yaml";
            if (std::ifstream(testPath).good()) {
                configDir = path;
                break;
            }
        }
    }

    // Initialize logging
    robot_controller::Logger::init("logs/core.log", "debug");

    LOG_INFO("========================================");
    LOG_INFO("Robot Controller Core v1.0.0");
    LOG_INFO("========================================");

    if (!workspaceDir.empty()) {
        LOG_INFO("Workspace: {}", workspaceDir);
    }
    LOG_INFO("Config: {}", configDir);

    // Create and initialize controller
    robot_controller::RobotController controller;
    g_controller = &controller;

    if (!workspaceDir.empty()) {
        controller.setWorkspacePath(workspaceDir);
    }

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
