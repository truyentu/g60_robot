/**
 * @file RobotInstance.hpp
 * @brief Robot Instance Manager - manages deployed robot instances
 */

#pragma once

#include "RobotCatalog.hpp"
#include <filesystem>
#include <map>
#include <optional>
#include <functional>

namespace robot_controller {
namespace config {

/**
 * Calibration data for a robot instance
 */
struct CalibrationData {
    std::string calibrationDate;
    std::string calibratedBy;
    std::array<double, 6> jointOffsets = {0, 0, 0, 0, 0, 0};
    std::string notes;

    bool hasOffsets() const {
        for (double offset : jointOffsets) {
            if (offset != 0.0) return true;
        }
        return false;
    }
};

/**
 * Robot Instance - a deployed robot with specific configuration and calibration
 */
struct RobotInstance {
    std::string instanceId;         // e.g., "robot_r1"
    std::string baseModelId;        // e.g., "custom_6dof_1200"
    std::string created;
    std::string lastModified;
    bool active = false;

    // Merged configuration (base model + overrides + calibration)
    RobotConfig config;

    // Calibration data
    CalibrationData calibration;

    // TCP override (from instance.yaml)
    TcpOffset tcpOverride;
    bool hasTcpOverride = false;
};

/**
 * Robot Instance Manager
 *
 * Manages deployed robot instances. Each instance is based on a catalog model
 * and can have:
 *   - TCP offset overrides
 *   - Calibration offsets
 */
class RobotInstanceManager {
public:
    /**
     * Constructor
     * @param catalog Reference to the robot catalog
     */
    explicit RobotInstanceManager(RobotCatalog& catalog);

    ~RobotInstanceManager() = default;

    /**
     * Load instances from directory
     * @param instancesDir Path to instances directory
     * @return true if loaded successfully
     */
    bool loadInstances(const std::filesystem::path& instancesDir);

    /**
     * Get active robot instance
     * @return Optional containing active instance, nullopt if none
     */
    std::optional<RobotInstance> getActiveInstance() const;

    /**
     * Get active instance ID
     */
    std::string getActiveInstanceId() const { return m_activeInstanceId; }

    /**
     * Set active instance by ID
     * @param instanceId Instance identifier
     * @return true if instance exists and was activated
     */
    bool setActiveInstance(const std::string& instanceId);

    /**
     * Change the base model of an instance
     * @param instanceId Instance to modify
     * @param modelId New base model ID
     * @return true if successful
     */
    bool changeInstanceModel(const std::string& instanceId, const std::string& modelId);

    /**
     * Create new instance from model
     * @param instanceId New instance ID
     * @param modelId Base model ID from catalog
     * @return true if created successfully
     */
    bool createInstance(const std::string& instanceId, const std::string& modelId);

    /**
     * Save instance configuration to disk
     * @param instanceId Instance to save
     * @return true if saved successfully
     */
    bool saveInstance(const std::string& instanceId);

    /**
     * Save active instance
     */
    bool saveActiveInstance();

    /**
     * Get list of instance IDs
     */
    std::vector<std::string> getInstanceIds() const;

    /**
     * Check if instance exists
     */
    bool hasInstance(const std::string& instanceId) const;

    /**
     * Get instance by ID
     */
    std::optional<RobotInstance> getInstance(const std::string& instanceId) const;

    /**
     * Get number of instances
     */
    size_t instanceCount() const { return m_instances.size(); }

    /**
     * Set callback for when active instance changes
     */
    using InstanceChangedCallback = std::function<void(const RobotInstance&)>;
    void setInstanceChangedCallback(InstanceChangedCallback callback) {
        m_instanceChangedCallback = callback;
    }

private:
    bool loadInstance(const std::filesystem::path& instanceDir);
    bool loadInstanceConfig(const std::filesystem::path& instancePath, RobotInstance& instance);
    bool loadCalibration(const std::filesystem::path& calibPath, RobotInstance& instance);
    RobotConfig mergeModelAndInstance(const RobotModel& model, const RobotInstance& instance) const;
    void notifyInstanceChanged(const RobotInstance& instance);

    RobotCatalog& m_catalog;
    std::filesystem::path m_instancesDir;
    std::map<std::string, RobotInstance> m_instances;
    std::string m_activeInstanceId;
    InstanceChangedCallback m_instanceChangedCallback;
};

} // namespace config
} // namespace robot_controller
