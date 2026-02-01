/**
 * @file main.cpp
 * @brief Robot Controller Core - Entry Point
 */

#include <iostream>
#include "logging/Logger.hpp"

int main(int argc, char* argv[]) {
    // Initialize logging
    robot_controller::Logger::init("../../logs/core.log", "debug");
    LOG_INFO("Robot Controller Core v1.0.0 starting...");

    LOG_INFO("Core initialized successfully");
    LOG_INFO("Press Ctrl+C to exit");

    // TODO: Initialize IPC Server
    // TODO: Initialize Config Manager
    // TODO: Main loop

    // Temporary: Keep running
    std::cout << "Robot Controller Core running. Press Enter to exit..." << std::endl;
    std::cin.get();

    LOG_INFO("Robot Controller Core shutting down");
    return 0;
}
