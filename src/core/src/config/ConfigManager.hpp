/**
 * @file ConfigManager.hpp
 * @brief Configuration manager - loads and provides access to configuration
 */

#pragma once

#include <string>
#include <memory>
#include <mutex>
#include <functional>
#include "RobotConfig.hpp"
#include "SystemConfig.hpp"
#include "RobotCatalog.hpp"
#include "RobotInstance.hpp"

namespace robot_controller {
namespace config {

/**
 * Configuration Manager (Singleton)
 *
 * Manages loading and access to robot and system configuration.
 * Supports multi-robot configuration through Robot Catalog system.
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

    // ========================================================================
    // Robot Catalog System
    // ========================================================================

    /**
     * Get robot catalog (for browsing available models)
     */
    RobotCatalog& robotCatalog() { return m_robotCatalog; }
    const RobotCatalog& robotCatalog() const { return m_robotCatalog; }

    /**
     * Get instance manager (for managing deployed robots)
     */
    RobotInstanceManager& instanceManager() { return *m_instanceManager; }
    const RobotInstanceManager& instanceManager() const { return *m_instanceManager; }

    /**
     * Select a robot model for the active instance
     * This loads the model from catalog and updates the active config
     * @param modelId Model ID from catalog (e.g., "kuka_kr6_r900")
     * @return true if model was selected successfully
     */
    bool selectRobotModel(const std::string& modelId);

    /**
     * Get currently active robot model ID
     */
    std::string getActiveModelId() const;

    /**
     * Get currently active robot instance ID
     */
    std::string getActiveInstanceId() const;

    /**
     * Check if catalog system is available
     */
    bool hasCatalog() const { return m_robotCatalog.isLoaded(); }

    /**
     * Callback type for robot config changes
     */
    using RobotConfigChangedCallback = std::function<void(const RobotConfig&)>;

    /**
     * Set callback for when robot configuration changes
     * (e.g., when user selects a different robot model)
     */
    void setRobotConfigChangedCallback(RobotConfigChangedCallback callback) {
        m_configChangedCallback = callback;
    }

private:
    ConfigManager();
    ~ConfigManager() = default;

    void notifyConfigChanged();
    bool initializeCatalogSystem(const std::string& config_dir);

    RobotConfig m_robot_config;
    SystemConfig m_system_config;
    bool m_loaded = false;
    mutable std::mutex m_mutex;

    // Robot Catalog System
    RobotCatalog m_robotCatalog;
    std::unique_ptr<RobotInstanceManager> m_instanceManager;
    RobotConfigChangedCallback m_configChangedCallback;
    std::string m_configDir;
};

} // namespace config
} // namespace robot_controller
