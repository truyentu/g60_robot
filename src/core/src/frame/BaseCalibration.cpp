#include "BaseCalibration.hpp"
#include "../logging/Logger.hpp"
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace robot_controller {
namespace frame {

std::optional<Frame> BaseCalibration::calculateFromThreePoints(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& xDirection,
    const Eigen::Vector3d& xyPlane) {

    // Vector from origin to X direction point
    Eigen::Vector3d vX = xDirection - origin;

    // Check minimum distance
    if (vX.norm() < MIN_POINT_DISTANCE) {
        LOG_ERROR("X direction point too close to origin: {:.2f}mm", vX.norm());
        return std::nullopt;
    }

    // Normalize to get X axis
    Eigen::Vector3d X = vX.normalized();

    // Vector from origin to XY plane point
    Eigen::Vector3d vXY = xyPlane - origin;

    if (vXY.norm() < MIN_POINT_DISTANCE) {
        LOG_ERROR("XY plane point too close to origin: {:.2f}mm", vXY.norm());
        return std::nullopt;
    }

    // Calculate Z axis as cross product of X and XY vectors
    Eigen::Vector3d Z = X.cross(vXY);

    if (Z.norm() < 0.001) {
        LOG_ERROR("XY plane point is collinear with X axis");
        return std::nullopt;
    }

    Z.normalize();

    // Calculate Y axis to ensure orthogonality
    Eigen::Vector3d Y = Z.cross(X);
    Y.normalize();

    // Build rotation matrix
    Eigen::Matrix3d R;
    R.col(0) = X;
    R.col(1) = Y;
    R.col(2) = Z;

    // Build 4x4 transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = origin;

    // Convert to Frame
    Frame result = Frame::fromMatrix(T);

    LOG_DEBUG("Calculated base frame from 3 points:");
    LOG_DEBUG("  Origin: ({:.1f}, {:.1f}, {:.1f})", origin.x(), origin.y(), origin.z());
    LOG_DEBUG("  X axis: ({:.3f}, {:.3f}, {:.3f})", X.x(), X.y(), X.z());
    LOG_DEBUG("  Y axis: ({:.3f}, {:.3f}, {:.3f})", Y.x(), Y.y(), Y.z());
    LOG_DEBUG("  Z axis: ({:.3f}, {:.3f}, {:.3f})", Z.x(), Z.y(), Z.z());

    return result;
}

std::optional<Frame> BaseCalibration::calculateFromFourPoints(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& xDirection,
    const Eigen::Vector3d& yDirection,
    const Eigen::Vector3d& zDirection) {

    // Calculate axis vectors
    Eigen::Vector3d vX = (xDirection - origin).normalized();
    Eigen::Vector3d vY = (yDirection - origin).normalized();
    Eigen::Vector3d vZ = (zDirection - origin).normalized();

    // Check orthogonality
    double xyAngle = std::acos(std::abs(vX.dot(vY))) * 180.0 / M_PI;
    double xzAngle = std::acos(std::abs(vX.dot(vZ))) * 180.0 / M_PI;
    double yzAngle = std::acos(std::abs(vY.dot(vZ))) * 180.0 / M_PI;

    if (std::abs(xyAngle - 90.0) > MAX_ORTHOGONALITY_ERROR ||
        std::abs(xzAngle - 90.0) > MAX_ORTHOGONALITY_ERROR ||
        std::abs(yzAngle - 90.0) > MAX_ORTHOGONALITY_ERROR) {
        LOG_WARN("Axes not orthogonal - correcting");
    }

    // Use Gram-Schmidt to orthogonalize
    Eigen::Vector3d X = vX;
    Eigen::Vector3d Y = vY - vY.dot(X) * X;
    Y.normalize();
    Eigen::Vector3d Z = X.cross(Y);

    // Build rotation matrix
    Eigen::Matrix3d R;
    R.col(0) = X;
    R.col(1) = Y;
    R.col(2) = Z;

    // Build 4x4 transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = origin;

    return Frame::fromMatrix(T);
}

std::optional<Frame> BaseCalibration::calculateFromPoints(
    const std::vector<CalibrationPointData>& points,
    BaseCalibrationMethod method) {

    switch (method) {
        case BaseCalibrationMethod::THREE_POINT: {
            if (points.size() < 3) {
                LOG_ERROR("Need at least 3 points for 3-point calibration");
                return std::nullopt;
            }

            Eigen::Vector3d origin(points[0].tcpPosition[0],
                                   points[0].tcpPosition[1],
                                   points[0].tcpPosition[2]);
            Eigen::Vector3d xDir(points[1].tcpPosition[0],
                                 points[1].tcpPosition[1],
                                 points[1].tcpPosition[2]);
            Eigen::Vector3d xyPlane(points[2].tcpPosition[0],
                                    points[2].tcpPosition[1],
                                    points[2].tcpPosition[2]);

            return calculateFromThreePoints(origin, xDir, xyPlane);
        }

        case BaseCalibrationMethod::FOUR_POINT: {
            if (points.size() < 4) {
                LOG_ERROR("Need at least 4 points for 4-point calibration");
                return std::nullopt;
            }

            Eigen::Vector3d origin(points[0].tcpPosition[0],
                                   points[0].tcpPosition[1],
                                   points[0].tcpPosition[2]);
            Eigen::Vector3d xDir(points[1].tcpPosition[0],
                                 points[1].tcpPosition[1],
                                 points[1].tcpPosition[2]);
            Eigen::Vector3d yDir(points[2].tcpPosition[0],
                                 points[2].tcpPosition[1],
                                 points[2].tcpPosition[2]);
            Eigen::Vector3d zDir(points[3].tcpPosition[0],
                                 points[3].tcpPosition[1],
                                 points[3].tcpPosition[2]);

            return calculateFromFourPoints(origin, xDir, yDir, zDir);
        }

        case BaseCalibrationMethod::DIRECT_INPUT:
            LOG_ERROR("Direct input method doesn't use calculateFromPoints");
            return std::nullopt;

        default:
            LOG_ERROR("Unknown calibration method");
            return std::nullopt;
    }
}

std::string BaseCalibration::validatePoints(
    const std::vector<CalibrationPointData>& points,
    BaseCalibrationMethod method) {

    // Check all points recorded
    for (const auto& pt : points) {
        if (!pt.recorded) {
            return "Point " + pt.pointName + " not recorded";
        }
    }

    // Check minimum distances
    if (points.size() >= 2) {
        Eigen::Vector3d p0(points[0].tcpPosition[0],
                          points[0].tcpPosition[1],
                          points[0].tcpPosition[2]);
        Eigen::Vector3d p1(points[1].tcpPosition[0],
                          points[1].tcpPosition[1],
                          points[1].tcpPosition[2]);

        if ((p1 - p0).norm() < MIN_POINT_DISTANCE) {
            return "X-Direction point too close to Origin (min " +
                   std::to_string(MIN_POINT_DISTANCE) + "mm)";
        }
    }

    if (points.size() >= 3) {
        Eigen::Vector3d p0(points[0].tcpPosition[0],
                          points[0].tcpPosition[1],
                          points[0].tcpPosition[2]);
        Eigen::Vector3d p2(points[2].tcpPosition[0],
                          points[2].tcpPosition[1],
                          points[2].tcpPosition[2]);

        if ((p2 - p0).norm() < MIN_POINT_DISTANCE) {
            return "XY-Plane point too close to Origin (min " +
                   std::to_string(MIN_POINT_DISTANCE) + "mm)";
        }

        // Check collinearity
        Eigen::Vector3d p1(points[1].tcpPosition[0],
                          points[1].tcpPosition[1],
                          points[1].tcpPosition[2]);

        Eigen::Vector3d v1 = (p1 - p0).normalized();
        Eigen::Vector3d v2 = (p2 - p0).normalized();
        double crossNorm = v1.cross(v2).norm();

        if (crossNorm < 0.1) {  // Nearly collinear
            return "Points are nearly collinear - XY plane point must not be on X axis";
        }
    }

    return "";  // Valid
}

} // namespace frame
} // namespace robot_controller
