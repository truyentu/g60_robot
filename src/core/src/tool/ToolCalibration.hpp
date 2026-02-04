#pragma once

#include "ToolTypes.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <optional>

namespace robot_controller {
namespace tool {

/// TCP Calibration algorithms
class ToolCalibration {
public:
    /// Calculate TCP position using 4-point method
    /// The robot TCP touches the same reference point from 4 different orientations
    /// Uses Least Squares optimization to find the TCP offset
    static std::optional<ToolTCP> calculate4Point(
        const std::vector<Eigen::Matrix4d>& flangeFrames
    );

    /// Calculate TCP position and orientation using 6-point method
    /// First 4 points: same as 4-point method (position)
    /// Last 2 points: along Z axis for orientation
    static std::optional<ToolTCP> calculate6Point(
        const std::vector<Eigen::Matrix4d>& flangeFrames
    );

    /// Validate calibration result by checking residual error
    static double calculateResidualError(
        const std::vector<Eigen::Matrix4d>& flangeFrames,
        const ToolTCP& tcp
    );

private:
    /// 4-point algorithm: Find TCP such that all flange frames
    /// transformed by TCP offset converge to the same world point
    static std::optional<Eigen::Vector3d> solveForTCPPosition(
        const std::vector<Eigen::Matrix4d>& flangeFrames
    );
};

} // namespace tool
} // namespace robot_controller
