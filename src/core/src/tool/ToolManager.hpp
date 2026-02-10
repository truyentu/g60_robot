#pragma once

#include "ToolTypes.hpp"
#include <map>
#include <mutex>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robot_controller {

// Forward declaration
namespace kinematics {
    class KinematicsService;
}

namespace tool {

/// Callback for tool change events
using ToolChangedCallback = std::function<void(const std::string& toolId)>;

/// Tool Manager - handles tool CRUD, selection, and calibration
class ToolManager {
public:
    explicit ToolManager(kinematics::KinematicsService* kinematics = nullptr);
    ~ToolManager() = default;

    // ========================================================================
    // CRUD Operations
    // ========================================================================

    /// Create a new tool
    bool createTool(const ToolData& tool);

    /// Update existing tool
    bool updateTool(const std::string& id, const ToolData& tool);

    /// Delete a tool
    bool deleteTool(const std::string& id);

    /// Get tool by ID
    std::optional<ToolData> getTool(const std::string& id) const;

    /// Get all tools
    std::vector<ToolData> getAllTools() const;

    /// Check if tool exists
    bool toolExists(const std::string& id) const;

    // ========================================================================
    // Active Tool Management
    // ========================================================================

    /// Set active tool by ID
    bool setActiveTool(const std::string& id);

    /// Get currently active tool
    std::optional<ToolData> getActiveTool() const;

    /// Get active tool ID
    std::string getActiveToolId() const;

    /// Set callback for tool change events
    void setToolChangedCallback(ToolChangedCallback callback);

    // ========================================================================
    // Calibration
    // ========================================================================

    /// Start calibration with specified method
    bool startCalibration(CalibrationMethod method);

    /// Record a calibration point (current joint angles)
    bool recordCalibrationPoint(const std::array<double, 6>& jointAngles);

    /// Calculate TCP from recorded points
    std::optional<ToolTCP> calculateTCP();

    /// Get current calibration status
    CalibrationStatus getCalibrationStatus() const;

    /// Cancel ongoing calibration
    void cancelCalibration();

    /// Apply calibration result to a tool
    bool applyCalibrationToTool(const std::string& toolId, const ToolTCP& tcp);

    // ========================================================================
    // Persistence
    // ========================================================================

    /// Save all tools to directory
    bool saveToDirectory(const std::string& path);

    /// Load tools from directory
    bool loadFromDirectory(const std::string& path);

    /// Save single tool to file
    bool saveToolToFile(const std::string& id, const std::string& filepath);

    /// Load single tool from file
    bool loadToolFromFile(const std::string& filepath);

private:
    // Tool storage
    std::map<std::string, ToolData> m_tools;
    std::string m_activeToolId;
    mutable std::mutex m_mutex;

    // Kinematics reference for FK calculation
    kinematics::KinematicsService* m_kinematics;

    // Calibration state
    CalibrationStatus m_calibStatus;
    std::vector<std::array<double, 6>> m_calibPoints;
    std::vector<Eigen::Matrix4d> m_calibFlangeFrames;

    // Callbacks
    ToolChangedCallback m_toolChangedCallback;

    // Persistence
    std::string m_configDir;  // Directory for auto-save (set by loadFromDirectory)

    // Helper methods
    void notifyToolChanged(const std::string& toolId);
    int getRequiredPoints(CalibrationMethod method) const;
    void autoSave();  // Save to m_configDir if set
};

} // namespace tool
} // namespace robot_controller
