/**
 * @file test_config.cpp
 * @brief Configuration tests
 */

#include <gtest/gtest.h>
#include "config/ConfigManager.hpp"

TEST(ConfigManager, SingletonInstance) {
    auto& instance1 = robot_controller::config::ConfigManager::instance();
    auto& instance2 = robot_controller::config::ConfigManager::instance();
    EXPECT_EQ(&instance1, &instance2);
}

// TODO: Add more tests in IMPL_P1_03
