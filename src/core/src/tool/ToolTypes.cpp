#include "ToolTypes.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace robot_controller {
namespace tool {

Eigen::Matrix4d ToolTCP::toTransform() const {
    double rxr = rx * M_PI / 180.0;
    double ryr = ry * M_PI / 180.0;
    double rzr = rz * M_PI / 180.0;

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(rzr, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(ryr, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(rxr, Eigen::Vector3d::UnitX());

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = Eigen::Vector3d(x, y, z);
    return T;
}

Eigen::Matrix4d ToolTCP::toInverseTransform() const {
    // For rigid body transform: T⁻¹ = [R' | -R'*t]
    Eigen::Matrix4d T = toTransform();
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);

    Eigen::Matrix4d Tinv = Eigen::Matrix4d::Identity();
    Tinv.block<3,3>(0,0) = R.transpose();
    Tinv.block<3,1>(0,3) = -R.transpose() * t;
    return Tinv;
}

bool ToolTCP::isZero() const {
    const double eps = 1e-9;
    return std::abs(x) < eps && std::abs(y) < eps && std::abs(z) < eps &&
           std::abs(rx) < eps && std::abs(ry) < eps && std::abs(rz) < eps;
}

} // namespace tool
} // namespace robot_controller
