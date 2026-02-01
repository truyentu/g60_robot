/**
 * @file test_ipc.cpp
 * @brief IPC tests
 */

#include <gtest/gtest.h>
#include "ipc/IpcServer.hpp"

TEST(IpcServer, CreateInstance) {
    robot_controller::IpcServer server;
    // Basic instantiation test
    SUCCEED();
}

// TODO: Add more tests in IMPL_P1_02
