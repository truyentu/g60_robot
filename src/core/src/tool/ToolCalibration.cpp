#include "ToolCalibration.hpp"
#include <spdlog/spdlog.h>
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <Eigen/Dense>
#include <Eigen/SVD>

namespace robot_controller {
namespace tool {

std::optional<ToolTCP> ToolCalibration::calculate4Point(
    const std::vector<Eigen::Matrix4d>& flangeFrames)
{
    if (flangeFrames.size() < 4) {
        spdlog::error("[ToolCalibration] Need at least 4 flange frames, got {}",
            flangeFrames.size());
        return std::nullopt;
    }

    auto tcpPosition = solveForTCPPosition(flangeFrames);
    if (!tcpPosition) {
        return std::nullopt;
    }

    ToolTCP result;
    result.x = (*tcpPosition)(0);
    result.y = (*tcpPosition)(1);
    result.z = (*tcpPosition)(2);
    result.rx = 0.0;
    result.ry = 0.0;
    result.rz = 0.0;

    spdlog::info("[ToolCalibration] 4-point result: TCP = ({:.3f}, {:.3f}, {:.3f})",
        result.x, result.y, result.z);

    return result;
}

std::optional<ToolTCP> ToolCalibration::calculate6Point(
    const std::vector<Eigen::Matrix4d>& flangeFrames)
{
    if (flangeFrames.size() < 6) {
        spdlog::error("[ToolCalibration] Need at least 6 flange frames, got {}",
            flangeFrames.size());
        return std::nullopt;
    }

    // First 4 points for TCP position
    std::vector<Eigen::Matrix4d> posFrames(flangeFrames.begin(), flangeFrames.begin() + 4);
    auto tcpPosition = solveForTCPPosition(posFrames);
    if (!tcpPosition) {
        return std::nullopt;
    }

    // Use frames 5 and 6 to determine TCP Z-axis direction
    // The TCP Z-axis should be oriented along the tool
    // For simplicity, we calculate the average Z-axis from the last 2 frames
    Eigen::Vector3d avgZ = Eigen::Vector3d::Zero();
    for (size_t i = 4; i < flangeFrames.size(); ++i) {
        avgZ += flangeFrames[i].block<3, 1>(0, 2);
    }
    avgZ.normalize();

    // Calculate rotation from flange Z to tool Z
    // This gives us the tool orientation offset
    Eigen::Vector3d flangeZ = flangeFrames[0].block<3, 1>(0, 2);

    // Calculate rotation axis and angle
    Eigen::Vector3d rotAxis = flangeZ.cross(avgZ);
    double rotAngle = 0.0;

    if (rotAxis.norm() > 1e-6) {
        rotAxis.normalize();
        rotAngle = std::acos(std::clamp(flangeZ.dot(avgZ), -1.0, 1.0));
    }

    // Convert to Euler angles (simplified - assuming small rotations)
    ToolTCP result;
    result.x = (*tcpPosition)(0);
    result.y = (*tcpPosition)(1);
    result.z = (*tcpPosition)(2);

    // Euler angles in degrees
    result.rx = rotAxis(0) * rotAngle * 180.0 / M_PI;
    result.ry = rotAxis(1) * rotAngle * 180.0 / M_PI;
    result.rz = rotAxis(2) * rotAngle * 180.0 / M_PI;

    spdlog::info("[ToolCalibration] 6-point result: TCP = ({:.3f}, {:.3f}, {:.3f}), "
        "rot = ({:.3f}, {:.3f}, {:.3f})",
        result.x, result.y, result.z, result.rx, result.ry, result.rz);

    return result;
}

double ToolCalibration::calculateResidualError(
    const std::vector<Eigen::Matrix4d>& flangeFrames,
    const ToolTCP& tcp)
{
    if (flangeFrames.empty()) {
        return std::numeric_limits<double>::max();
    }

    Eigen::Vector3d tcpOffset(tcp.x, tcp.y, tcp.z);

    // Calculate TCP world positions for each frame
    std::vector<Eigen::Vector3d> tcpPositions;
    tcpPositions.reserve(flangeFrames.size());

    for (const auto& frame : flangeFrames) {
        Eigen::Vector3d flangePos = frame.block<3, 1>(0, 3);
        Eigen::Matrix3d flangeRot = frame.block<3, 3>(0, 0);
        Eigen::Vector3d worldTcp = flangePos + flangeRot * tcpOffset;
        tcpPositions.push_back(worldTcp);
    }

    // Calculate centroid
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& pos : tcpPositions) {
        centroid += pos;
    }
    centroid /= static_cast<double>(tcpPositions.size());

    // Calculate RMS error from centroid
    double sumSquaredError = 0.0;
    for (const auto& pos : tcpPositions) {
        sumSquaredError += (pos - centroid).squaredNorm();
    }

    return std::sqrt(sumSquaredError / tcpPositions.size());
}

std::optional<Eigen::Vector3d> ToolCalibration::solveForTCPPosition(
    const std::vector<Eigen::Matrix4d>& flangeFrames)
{
    // 4-point TCP calibration algorithm
    // The TCP point P_tcp in flange frame, when transformed to world frame,
    // should be the same for all configurations:
    //
    // P_world = T_flange * P_tcp
    //
    // For two frames i and j:
    // T_i * P_tcp = T_j * P_tcp
    // (T_i - T_j) * P_tcp = 0
    //
    // We set up a least squares problem:
    // A * P_tcp = b
    //
    // where A contains rotation differences and b contains position differences

    const size_t n = flangeFrames.size();
    if (n < 4) {
        return std::nullopt;
    }

    // Build the system of equations
    // For n frames, we have n*(n-1)/2 pairs
    const size_t numPairs = n * (n - 1) / 2;
    Eigen::MatrixXd A(3 * numPairs, 3);
    Eigen::VectorXd b(3 * numPairs);

    size_t row = 0;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            Eigen::Matrix3d Ri = flangeFrames[i].block<3, 3>(0, 0);
            Eigen::Matrix3d Rj = flangeFrames[j].block<3, 3>(0, 0);
            Eigen::Vector3d pi = flangeFrames[i].block<3, 1>(0, 3);
            Eigen::Vector3d pj = flangeFrames[j].block<3, 1>(0, 3);

            // (Ri - Rj) * tcp = pj - pi
            A.block<3, 3>(row * 3, 0) = Ri - Rj;
            b.segment<3>(row * 3) = pj - pi;
            ++row;
        }
    }

    // Solve using SVD (more stable than normal equations)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Check condition number
    double conditionNumber = svd.singularValues()(0) / svd.singularValues()(2);
    if (conditionNumber > 1e6) {
        spdlog::warn("[ToolCalibration] Poor conditioning ({}), calibration may be inaccurate",
            conditionNumber);
    }

    Eigen::Vector3d tcp = svd.solve(b);

    // Validate result
    double residual = (A * tcp - b).norm() / std::sqrt(static_cast<double>(b.size()));
    spdlog::info("[ToolCalibration] Least squares residual: {:.4f} mm", residual);

    if (residual > 10.0) {
        spdlog::warn("[ToolCalibration] High residual error, check calibration points");
    }

    return tcp;
}

} // namespace tool
} // namespace robot_controller
