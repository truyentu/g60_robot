/**
 * @file ConfigManager.hpp
 * @brief Configuration manager - loads and provides access to configuration
 */

#pragma once

#include <string>
#include <memory>
#include <mutex>
#include "RobotConfig.hpp"
#include "SystemConfig.hpp"

namespace robot_controller {
namespace config {

/**
 * Configuration Manager (Singleton)
 *
 * Manages loading and access to robot and system configuration.
 * Thread-safe for reading after initialization.
 */
class ConfigManager {
public:
    /**
     * Get singleton instance
     */
    static ConfigManager& instance();

    // Delete copy/move
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
    ConfigManager(ConfigManager&&) = delete;
    ConfigManager& operator=(ConfigManager&&) = delete;

    /**
     * Load robot configuration from YAML file
     * @param filepath Path to robot_config.yaml
     * @return true if loaded successfully
     */
    bool loadRobotConfig(const std::string& filepath);

    /**
     * Load system configuration from YAML file
     * @param filepath Path to system_config.yaml
     * @return true if loaded successfully
     */
    bool loadSystemConfig(const std::string& filepath);

    /**
     * Load all configuration files from a directory
     * @param config_dir Path to config directory
     * @return true if all configs loaded successfully
     */
    bool loadAll(const std::string& config_dir = "config");

    /**
     * Save robot configuration to YAML file
     * @param filepath Path to save
     * @return true if saved successfully
     */
    bool saveRobotConfig(const std::string& filepath) const;

    /**
     * Get robot configuration (const reference)
     */
    const RobotConfig& robotConfig() const { return m_robot_config; }

    /**
     * Get system configuration (const reference)
     */
    const SystemConfig& systemConfig() const { return m_system_config; }

    /**
     * Check if configuration is loaded and valid
     */
    bool isLoaded() const { return m_loaded; }

    /**
     * Get configuration as JSON (for IPC)
     */
    std::string robotConfigToJson() const;
    std::string systemConfigToJson() const;

private:
    ConfigManager() = default;
    ~ConfigManager() = default;

    RobotConfig m_robot_config;
    SystemConfig m_system_config;
    bool m_loaded = false;
    mutable std::mutex m_mutex;
};

} // namespace config
} // namespace robot_controller
