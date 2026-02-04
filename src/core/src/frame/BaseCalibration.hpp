#pragma once

#include "FrameTypes.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <vector>

namespace robot_controller {
namespace frame {

/**
 * @brief Base frame calibration algorithms
 */
class BaseCalibration {
public:
    /**
     * @brief Calculate base frame from 3 points (3-point method)
     *
     * @param origin Point defining the origin of the base frame
     * @param xDirection Point on the positive X axis
     * @param xyPlane Point on the XY plane (positive Y side)
     * @return Calculated frame if successful
     *
     * Algorithm:
     * 1. X axis = normalize(xDirection - origin)
     * 2. tempY = xyPlane - origin
     * 3. Z axis = normalize(X cross tempY)
     * 4. Y axis = Z cross X (ensures orthogonality)
     */
    static std::optional<Frame> calculateFromThreePoints(
        const Eigen::Vector3d& origin,
        const Eigen::Vector3d& xDirection,
        const Eigen::Vector3d& xyPlane);

    /**
     * @brief Calculate base frame from 4 points (overdetermined)
     *
     * @param origin Point defining the origin
     * @param xDirection Point on positive X axis
     * @param yDirection Point on positive Y axis
     * @param zDirection Point on positive Z axis
     * @return Calculated frame if successful
     */
    static std::optional<Frame> calculateFromFourPoints(
        const Eigen::Vector3d& origin,
        const Eigen::Vector3d& xDirection,
        const Eigen::Vector3d& yDirection,
        const Eigen::Vector3d& zDirection);

    /**
     * @brief Calculate from array of calibration points
     *
     * @param points Vector of calibration point data
     * @param method Calibration method used
     * @return Calculated frame if successful
     */
    static std::optional<Frame> calculateFromPoints(
        const std::vector<CalibrationPointData>& points,
        BaseCalibrationMethod method);

    /**
     * @brief Validate that points are suitable for calibration
     *
     * @param points Calibration points
     * @param method Calibration method
     * @return Error message if invalid, empty if OK
     */
    static std::string validatePoints(
        const std::vector<CalibrationPointData>& points,
        BaseCalibrationMethod method);

private:
    // Minimum distance between points for valid calibration (mm)
    static constexpr double MIN_POINT_DISTANCE = 10.0;

    // Maximum allowed deviation from orthogonality (degrees)
    static constexpr double MAX_ORTHOGONALITY_ERROR = 5.0;
};

} // namespace frame
} // namespace robot_controller
