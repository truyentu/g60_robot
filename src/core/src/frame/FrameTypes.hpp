#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <string>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robot_controller {
namespace frame {

/**
 * @brief Represents a 6-DOF frame (position + orientation)
 *
 * Position in mm, orientation in degrees (Euler ZYX convention)
 */
struct Frame {
    double x = 0.0;   // mm
    double y = 0.0;   // mm
    double z = 0.0;   // mm
    double rx = 0.0;  // degrees (rotation around X)
    double ry = 0.0;  // degrees (rotation around Y)
    double rz = 0.0;  // degrees (rotation around Z)

    /**
     * @brief Convert frame to 4x4 homogeneous transformation matrix
     */
    Eigen::Matrix4d toMatrix() const {
        constexpr double DEG_TO_RAD = M_PI / 180.0;

        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();

        // Rotation: ZYX Euler angles
        double cx = std::cos(rx * DEG_TO_RAD);
        double sx = std::sin(rx * DEG_TO_RAD);
        double cy = std::cos(ry * DEG_TO_RAD);
        double sy = std::sin(ry * DEG_TO_RAD);
        double cz = std::cos(rz * DEG_TO_RAD);
        double sz = std::sin(rz * DEG_TO_RAD);

        // ZYX rotation matrix
        mat(0, 0) = cy * cz;
        mat(0, 1) = cz * sy * sx - sz * cx;
        mat(0, 2) = cz * sy * cx + sz * sx;
        mat(1, 0) = cy * sz;
        mat(1, 1) = sz * sy * sx + cz * cx;
        mat(1, 2) = sz * sy * cx - cz * sx;
        mat(2, 0) = -sy;
        mat(2, 1) = cy * sx;
        mat(2, 2) = cy * cx;

        // Translation
        mat(0, 3) = x;
        mat(1, 3) = y;
        mat(2, 3) = z;

        return mat;
    }

    /**
     * @brief Create frame from 4x4 homogeneous transformation matrix
     */
    static Frame fromMatrix(const Eigen::Matrix4d& mat) {
        constexpr double RAD_TO_DEG = 180.0 / M_PI;

        Frame f;
        f.x = mat(0, 3);
        f.y = mat(1, 3);
        f.z = mat(2, 3);

        // Extract Euler angles (ZYX convention)
        double sy = -mat(2, 0);
        if (std::abs(sy) < 0.99999) {
            f.ry = std::asin(sy) * RAD_TO_DEG;
            f.rx = std::atan2(mat(2, 1), mat(2, 2)) * RAD_TO_DEG;
            f.rz = std::atan2(mat(1, 0), mat(0, 0)) * RAD_TO_DEG;
        } else {
            // Gimbal lock
            f.ry = (sy > 0) ? 90.0 : -90.0;
            f.rx = 0.0;
            f.rz = std::atan2(-mat(0, 1), mat(1, 1)) * RAD_TO_DEG;
        }

        return f;
    }

    /**
     * @brief Get position as array
     */
    std::array<double, 3> position() const {
        return {x, y, z};
    }

    /**
     * @brief Get orientation as array
     */
    std::array<double, 3> orientation() const {
        return {rx, ry, rz};
    }

    /**
     * @brief Get as 6-element array [x, y, z, rx, ry, rz]
     */
    std::array<double, 6> toArray() const {
        return {x, y, z, rx, ry, rz};
    }

    /**
     * @brief Create from 6-element array
     */
    static Frame fromArray(const std::array<double, 6>& arr) {
        return Frame{arr[0], arr[1], arr[2], arr[3], arr[4], arr[5]};
    }
};

/**
 * @brief Base/Workpiece coordinate frame
 */
struct BaseFrame {
    std::string id;           // Unique identifier
    std::string name;         // Display name
    std::string description;  // Optional description
    Frame frame;              // Transform from world to this base
    bool isActive = false;    // Currently active base frame
};

/**
 * @brief Method for calibrating base frame
 */
enum class BaseCalibrationMethod {
    DIRECT_INPUT,   // User enters values directly
    THREE_POINT,    // 3-point: Origin, +X direction, XY plane point
    FOUR_POINT      // 4-point: Origin, +X, +Y, +Z (overdetermined)
};

/**
 * @brief State of base calibration process
 */
enum class BaseCalibrationState {
    IDLE,
    IN_PROGRESS,
    COMPLETED,
    ERROR
};

/**
 * @brief Data for a calibration point
 */
struct CalibrationPointData {
    int pointIndex = 0;
    std::string pointName;              // "Origin", "X-Direction", "XY-Plane"
    std::array<double, 6> jointAngles;  // Joint angles when point recorded
    std::array<double, 3> tcpPosition;  // TCP position in world coords
    bool recorded = false;
};

/**
 * @brief Status of base frame calibration
 */
struct BaseCalibrationStatus {
    BaseCalibrationState state = BaseCalibrationState::IDLE;
    BaseCalibrationMethod method = BaseCalibrationMethod::THREE_POINT;
    int pointsRequired = 3;
    int pointsRecorded = 0;
    std::string currentPointName;
    std::string errorMessage;
};

/**
 * @brief Convert method enum to string
 */
inline std::string baseCalibrationMethodToString(BaseCalibrationMethod method) {
    switch (method) {
        case BaseCalibrationMethod::DIRECT_INPUT: return "DIRECT_INPUT";
        case BaseCalibrationMethod::THREE_POINT:  return "THREE_POINT";
        case BaseCalibrationMethod::FOUR_POINT:   return "FOUR_POINT";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Parse method from string
 */
inline BaseCalibrationMethod baseCalibrationMethodFromString(const std::string& str) {
    if (str == "DIRECT_INPUT") return BaseCalibrationMethod::DIRECT_INPUT;
    if (str == "THREE_POINT" || str == "3point")  return BaseCalibrationMethod::THREE_POINT;
    if (str == "FOUR_POINT" || str == "4point")   return BaseCalibrationMethod::FOUR_POINT;
    return BaseCalibrationMethod::THREE_POINT;
}

/**
 * @brief Point names for 3-point calibration
 */
inline std::string getCalibrationPointName(int index) {
    switch (index) {
        case 0: return "Origin";
        case 1: return "X-Direction";
        case 2: return "XY-Plane";
        case 3: return "Z-Direction";
        default: return "Point " + std::to_string(index);
    }
}

} // namespace frame
} // namespace robot_controller
