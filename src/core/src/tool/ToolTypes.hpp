#pragma once

#include <string>
#include <array>
#include <vector>
#include <optional>
#include <Eigen/Dense>

namespace robot_controller {
namespace tool {

/// TCP offset from flange
struct ToolTCP {
    double x = 0.0;   // mm - X offset
    double y = 0.0;   // mm - Y offset
    double z = 0.0;   // mm - Z offset
    double rx = 0.0;  // degrees - rotation around X
    double ry = 0.0;  // degrees - rotation around Y
    double rz = 0.0;  // degrees - rotation around Z

    std::array<double, 6> toArray() const {
        return {x, y, z, rx, ry, rz};
    }

    static ToolTCP fromArray(const std::array<double, 6>& arr) {
        return {arr[0], arr[1], arr[2], arr[3], arr[4], arr[5]};
    }

    /// Compute 4x4 homogeneous transform from flange to TCP
    /// T_tool = Translation(x,y,z) * Rz(rz) * Ry(ry) * Rx(rx)
    Eigen::Matrix4d toTransform() const;

    /// Compute inverse: T_tool⁻¹ (TCP to flange)
    Eigen::Matrix4d toInverseTransform() const;

    /// Check if TCP offset is effectively zero (no tool)
    bool isZero() const;
};

/// Tool inertia data for dynamics
struct ToolInertia {
    double mass = 0.0;      // kg
    double cogX = 0.0;      // Center of Gravity X (mm)
    double cogY = 0.0;      // Center of Gravity Y (mm)
    double cogZ = 0.0;      // Center of Gravity Z (mm)
    double ixx = 0.0;       // Inertia tensor (kg*mm^2)
    double iyy = 0.0;
    double izz = 0.0;
};

/// Complete tool data
struct ToolData {
    std::string id;
    std::string name;
    std::string description;
    ToolTCP tcp;
    ToolInertia inertia;
    bool isActive = false;

    // Visual mesh for 3D visualization
    std::string visualMeshPath;      // Path to STL file (relative to tool config dir)

    // Mesh visual offset from flange (if STL origin != flange center)
    double meshOffsetX = 0.0;        // mm
    double meshOffsetY = 0.0;
    double meshOffsetZ = 0.0;
    double meshOffsetRx = 0.0;       // degrees
    double meshOffsetRy = 0.0;
    double meshOffsetRz = 0.0;
    double meshScale = 1.0;            // Scale factor for STL (1.0=mm, 1000.0=meters→mm)
};

/// TCP Calibration methods
enum class CalibrationMethod {
    DIRECT_INPUT,   // Manual TCP input
    FOUR_POINT,     // 4-point TCP calibration (position only)
    SIX_POINT       // 6-point TCP calibration (position + orientation)
};

/// Calibration state
enum class CalibrationState {
    IDLE,
    COLLECTING_POINTS,
    CALCULATING,
    COMPLETED,
    FAILED
};

/// Calibration status
struct CalibrationStatus {
    CalibrationState state = CalibrationState::IDLE;
    CalibrationMethod method = CalibrationMethod::FOUR_POINT;
    int pointsRecorded = 0;
    int pointsRequired = 4;
    std::string errorMessage;
    std::optional<ToolTCP> result;
};

inline std::string calibrationMethodToString(CalibrationMethod method) {
    switch (method) {
        case CalibrationMethod::DIRECT_INPUT: return "DIRECT_INPUT";
        case CalibrationMethod::FOUR_POINT: return "FOUR_POINT";
        case CalibrationMethod::SIX_POINT: return "SIX_POINT";
        default: return "UNKNOWN";
    }
}

inline CalibrationMethod calibrationMethodFromString(const std::string& str) {
    if (str == "DIRECT_INPUT" || str == "direct") return CalibrationMethod::DIRECT_INPUT;
    if (str == "FOUR_POINT" || str == "4point") return CalibrationMethod::FOUR_POINT;
    if (str == "SIX_POINT" || str == "6point") return CalibrationMethod::SIX_POINT;
    return CalibrationMethod::FOUR_POINT;
}

inline std::string calibrationStateToString(CalibrationState state) {
    switch (state) {
        case CalibrationState::IDLE: return "IDLE";
        case CalibrationState::COLLECTING_POINTS: return "COLLECTING_POINTS";
        case CalibrationState::CALCULATING: return "CALCULATING";
        case CalibrationState::COMPLETED: return "COMPLETED";
        case CalibrationState::FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

} // namespace tool
} // namespace robot_controller
