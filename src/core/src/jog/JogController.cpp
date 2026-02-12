#include "JogController.hpp"
#include "../logging/Logger.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <fstream>

namespace robot_controller {
namespace jog {

JogController::JogController() {
    LOG_INFO("JogController created");
}

void JogController::setFirmwareDriver(std::shared_ptr<firmware::IFirmwareDriver> driver) {
    m_firmwareDriver = driver;

    // Sync limits to driver
    if (m_firmwareDriver) {
        for (int i = 0; i < 6; i++) {
            m_firmwareDriver->setJointLimits(i, m_minLimits[i], m_maxLimits[i]);
            m_firmwareDriver->setMaxVelocity(i, m_maxVelocities[i]);
        }
    }
}

void JogController::initializeKinematics(const kinematics::RobotKinematicConfig& config) {
    m_fk = std::make_unique<kinematics::ForwardKinematics>(config);
    m_ik = std::make_unique<kinematics::InverseKinematics>(config);

    LOG_INFO("JogController: DH Kinematics initialized (IK only, FK deprecated)");
}

void JogController::initializeUrdfKinematics(
    const std::vector<kinematics::UrdfJointDef>& joints,
    const kinematics::Vector3d& toolOffset)
{
    m_urdfFk = std::make_unique<kinematics::UrdfForwardKinematics>(joints, toolOffset);

    m_kdlKin = std::make_unique<kinematics::KDLKinematics>();
    m_kdlKin->buildFromUrdfJoints(joints, toolOffset);

    // Set joint limits for IK clamping
    std::vector<std::pair<double,double>> limits;
    for (const auto& j : joints) {
        limits.emplace_back(j.minAngle, j.maxAngle);
    }
    m_kdlKin->setJointLimits(limits);

    LOG_INFO("JogController: URDF FK + KDL IK initialized ({} joints)", joints.size());
}

void JogController::setToolTransform(const Eigen::Matrix4d& T_tool) {
    m_toolTransform = T_tool;

    // Efficient inverse: T⁻¹ = [R' | -R'*t]
    Eigen::Matrix3d R = T_tool.block<3,3>(0,0);
    Eigen::Vector3d t = T_tool.block<3,1>(0,3);
    m_toolInvTransform = Eigen::Matrix4d::Identity();
    m_toolInvTransform.block<3,3>(0,0) = R.transpose();
    m_toolInvTransform.block<3,1>(0,3) = -R.transpose() * t;

    m_hasToolOffset = (t.norm() > 1e-9 || !R.isIdentity(1e-9));
}

void JogController::setJointLimits(int joint, double minDeg, double maxDeg) {
    if (joint >= 0 && joint < 6) {
        m_minLimits[joint] = minDeg;
        m_maxLimits[joint] = maxDeg;
        if (m_firmwareDriver) {
            m_firmwareDriver->setJointLimits(joint, minDeg, maxDeg);
        }
    }
}

void JogController::setMaxVelocity(int joint, double degPerSec) {
    if (joint >= 0 && joint < 6) {
        m_maxVelocities[joint] = degPerSec;
        if (m_firmwareDriver) {
            m_firmwareDriver->setMaxVelocity(joint, degPerSec);
        }
    }
}

bool JogController::enable() {
    if (!m_firmwareDriver) {
        LOG_ERROR("JogController: No firmware driver set");
        return false;
    }
    m_enabled = true;
    LOG_INFO("JogController: Jog mode enabled");
    return true;
}

bool JogController::disable() {
    if (m_isJogging) {
        stopJog();
    }
    m_enabled = false;
    LOG_INFO("JogController: Jog mode disabled");
    return true;
}

std::string JogController::startContinuousJog(int mode, int axis, int direction, double speedPercent, int frame) {
    auto error = validateJogRequest(mode, axis, direction, speedPercent);
    if (!error.empty()) return error;

    // Cartesian jog → velocity streaming
    if (mode == 1) {
        return startCartesianJog(axis, direction, speedPercent, frame);
    }

    // Joint jog → G-code to limit (existing approach, works fine)
    auto positions = m_firmwareDriver->getJointPositions();
    double currentPos = positions[axis];
    double target;
    if (direction > 0) {
        target = m_maxLimits[axis] - SOFT_LIMIT_MARGIN;
    } else {
        target = m_minLimits[axis] + SOFT_LIMIT_MARGIN;
    }

    if (std::abs(currentPos - target) < 0.1) {
        return "Joint at soft limit";
    }

    double feedRate = calculateFeedRate(axis, speedPercent);
    std::string gcode = "$J=G90 J" + std::to_string(axis) + ":" +
                        std::to_string(target) + " F" + std::to_string(feedRate);

    LOG_DEBUG("JogController: Continuous jog J{} dir={} speed={}% -> gcode: {}",
              axis, direction, speedPercent, gcode);

    m_firmwareDriver->sendCommand(gcode);

    m_isJogging = true;
    m_currentAxis = axis;
    m_currentDirection = direction;
    m_currentSpeed = speedPercent;
    m_currentMode = mode;

    return "";
}

std::string JogController::stopJog() {
    if (!m_firmwareDriver) return "No firmware driver";

    if (m_cartesianStreaming) {
        // Velocity streaming: set target velocity to 0 for smooth deceleration
        m_targetTcpVelocity = 0.0;
        // Don't clear m_cartesianStreaming yet - updateCartesianVelocityStreaming
        // will clear it once velocity reaches 0
        LOG_DEBUG("JogController: Cartesian streaming stop requested (decelerating)");
        return "";
    }

    // Joint jog: send jog cancel command
    m_firmwareDriver->sendCommand(std::string(1, '\x85'));

    m_isJogging = false;
    m_currentDirection = 0;
    m_currentSpeed = 0.0;

    LOG_DEBUG("JogController: Jog stopped");
    return "";
}

std::string JogController::jogStep(int mode, int axis, int direction, double increment, double speedPercent, int frame) {
    auto error = validateJogRequest(mode, axis, direction, speedPercent);
    if (!error.empty()) return error;

    // Cartesian step jog
    if (mode == 1) {
        return jogCartesianStep(axis, direction, increment, speedPercent, frame);
    }

    // Joint step jog
    if (increment <= 0) {
        return "Increment must be positive";
    }

    auto positions = m_firmwareDriver->getJointPositions();
    double currentPos = positions[axis];
    double target = currentPos + (direction * increment);

    target = std::clamp(target, m_minLimits[axis] + SOFT_LIMIT_MARGIN,
                                m_maxLimits[axis] - SOFT_LIMIT_MARGIN);

    if (std::abs(target - currentPos) < 0.001) {
        return "Joint at soft limit";
    }

    double feedRate = calculateFeedRate(axis, speedPercent);
    std::string gcode = "$J=G90 J" + std::to_string(axis) + ":" +
                        std::to_string(target) + " F" + std::to_string(feedRate);

    LOG_DEBUG("JogController: Step jog J{} dir={} inc={} -> target={} gcode: {}",
              axis, direction, increment, target, gcode);

    m_firmwareDriver->sendCommand(gcode);

    m_isJogging = true;
    m_currentAxis = axis;
    m_currentDirection = direction;
    m_currentSpeed = speedPercent;
    m_currentMode = mode;

    return "";
}

// ========================================================================
// Cartesian Jog Implementation - Velocity Streaming
// ========================================================================

void JogController::transformDeltaToWorld(int frame, int axis, double delta,
                                           const kinematics::TCPPose& currentPose,
                                           kinematics::TCPPose& targetPose) {
    bool isLinear = (axis < 3);

    if (frame == 0 || frame == 1) {
        // WORLD or BASE frame: delta is already in world coordinates
        if (isLinear) {
            targetPose.position[axis] += delta;
        } else {
            double radDelta = delta * M_PI / 180.0;
            targetPose.rpy[axis - 3] += radDelta;
            targetPose.rotation = kinematics::rpyToRotation(targetPose.rpy);
        }
    } else if (frame == 2) {
        // TOOL frame: delta is in TCP's local coordinate system
        kinematics::Matrix3d R_tcp = currentPose.rotation;

        if (isLinear) {
            kinematics::Vector3d delta_tool = kinematics::Vector3d::Zero();
            delta_tool[axis] = delta;
            kinematics::Vector3d delta_world = R_tcp * delta_tool;

            targetPose.position += delta_world;
        } else {
            kinematics::Vector3d rot_axis_tool = kinematics::Vector3d::Zero();
            rot_axis_tool[axis - 3] = 1.0;
            kinematics::Vector3d rot_axis_world = R_tcp * rot_axis_tool;
            double radDelta = delta * M_PI / 180.0;
            kinematics::AngleAxisd aa(radDelta, rot_axis_world);
            targetPose.rotation = aa.toRotationMatrix() * currentPose.rotation;
            targetPose.rpy = kinematics::rotationToRPY(targetPose.rotation);
        }
    } else {
        // USER1/USER2: fall back to world
        if (isLinear) {
            targetPose.position[axis] += delta;
        } else {
            double radDelta = delta * M_PI / 180.0;
            targetPose.rpy[axis - 3] += radDelta;
            targetPose.rotation = kinematics::rpyToRotation(targetPose.rpy);
        }
    }
}

std::string JogController::startCartesianJog(int axis, int direction, double speedPercent, int frame) {
    if (!m_urdfFk || !m_kdlKin) {
        return "Kinematics not initialized for Cartesian jog";
    }

    // Velocity streaming approach: instead of computing a target and sending G-code,
    // we set up streaming parameters. The update() loop will continuously compute
    // small Cartesian increments, solve IK, and command joint positions directly.

    bool isLinear = (axis < 3);
    double maxSpeed = isLinear ? MAX_TCP_LINEAR_SPEED : MAX_TCP_ROT_SPEED;
    double targetSpeed = maxSpeed * (speedPercent / 100.0) * direction;

    m_cartesianStreaming = true;
    m_targetTcpVelocity = targetSpeed;
    m_tcpVelocity = 0.0;  // Start from zero, ramp up

    m_isJogging = true;
    m_currentAxis = axis;
    m_currentDirection = direction;
    m_currentSpeed = speedPercent;
    m_currentMode = 1;
    m_currentFrame = frame;

    LOG_DEBUG("JogController: Cartesian velocity streaming started axis={} dir={} speed={}% frame={} targetV={}",
              axis, direction, speedPercent, frame, targetSpeed);

    return "";
}

void JogController::updateCartesianVelocityStreaming(double dt) {
    if (!m_cartesianStreaming || !m_urdfFk || !m_firmwareDriver) return;
    // Need KDL IK solver
    if (!m_kdlKin) return;

    bool isLinear = (m_currentAxis < 3);
    double accelRate = isLinear ? TCP_ACCEL_RATE : TCP_ROT_ACCEL_RATE;

    // === Velocity ramp ===
    double diff = m_targetTcpVelocity - m_tcpVelocity;
    double maxChange = accelRate * dt;

    if (std::abs(diff) <= maxChange) {
        m_tcpVelocity = m_targetTcpVelocity;
    } else if (diff > 0) {
        m_tcpVelocity += maxChange;
    } else {
        m_tcpVelocity -= maxChange;
    }

    // If velocity has reached zero and target is zero, stop streaming
    if (std::abs(m_tcpVelocity) < 0.001 && std::abs(m_targetTcpVelocity) < 0.001) {
        m_cartesianStreaming = false;
        m_isJogging = false;
        m_tcpVelocity = 0.0;
        m_currentDirection = 0;
        m_currentSpeed = 0.0;
        LOG_DEBUG("JogController: Cartesian streaming fully stopped");
        return;
    }

    // === Compute Cartesian delta for this timestep ===
    double delta = m_tcpVelocity * dt;  // mm or deg

    // Skip if delta too small (avoid unnecessary IK computations)
    if (std::abs(delta) < 0.0001) return;

    // === Get current joint positions and FK ===
    auto positions = m_firmwareDriver->getJointPositions();
    kinematics::JointAngles jointRad;
    for (int i = 0; i < 6; i++) {
        jointRad[i] = positions[i] * M_PI / 180.0;
    }

    // FK returns last-link pose (no flange offset, no tool TCP)
    auto flangePose = m_urdfFk->compute(jointRad);

    // Compute TRUE TCP pose = T_lastLink × T_composed (T_flangeOffset × T_toolTcp)
    kinematics::TCPPose currentTcpPose = flangePose;
    if (m_hasToolOffset) {
        Eigen::Matrix4d T_flange = flangePose.toTransform();
        Eigen::Matrix4d T_tcp = T_flange * m_toolTransform;
        currentTcpPose = kinematics::TCPPose::fromTransform(T_tcp);
    }

    // === Apply Cartesian delta in the selected frame ===
    // Delta is applied on TRUE TCP pose (not flange)
    kinematics::TCPPose targetTcpPose = currentTcpPose;
    transformDeltaToWorld(m_currentFrame, m_currentAxis, delta, currentTcpPose, targetTcpPose);

    // === Back-calculation: T_flange_desired = T_tcp_desired × T_tool⁻¹ ===
    kinematics::TCPPose targetFlangePose = targetTcpPose;
    if (m_hasToolOffset) {
        Eigen::Matrix4d T_tcp_desired = targetTcpPose.toTransform();
        Eigen::Matrix4d T_flange_desired = T_tcp_desired * m_toolInvTransform;
        targetFlangePose = kinematics::TCPPose::fromTransform(T_flange_desired);
    }

    // === Solve IK for FLANGE pose (not TCP!) ===
    std::optional<kinematics::IKSolution> ikResult;
    if (m_kdlKin && m_kdlKin->isInitialized()) {
        ikResult = m_kdlKin->computeIK(targetFlangePose, jointRad);
    }
    if (!ikResult.has_value()) {
        // Workspace limit reached - stop streaming
        m_tcpVelocity = 0.0;
        m_targetTcpVelocity = 0.0;
        m_cartesianStreaming = false;
        m_isJogging = false;
        m_currentDirection = 0;
        m_currentSpeed = 0.0;
        LOG_WARN("JogController: Cartesian streaming stopped - workspace limit reached");
        return;
    }

    // Verify orientation is preserved after IK - REJECT if too far off
    {
        auto verifyPose = m_urdfFk->compute(ikResult->angles);
        double posErr = (verifyPose.position - targetFlangePose.position).norm();
        kinematics::Matrix3d R_err = verifyPose.rotation.transpose() * targetFlangePose.rotation;
        double orientErr = std::acos(std::clamp((R_err.trace() - 1.0) / 2.0, -1.0, 1.0));
        double orientErrDeg = orientErr * 180.0 / M_PI;

        if (orientErrDeg > 0.5) {
            LOG_WARN("JogController: IK orientation drift REJECTED! posErr={:.3f}mm orientErr={:.1f}deg "
                     "target RPY=[{:.1f},{:.1f},{:.1f}] actual RPY=[{:.1f},{:.1f},{:.1f}]",
                     posErr, orientErrDeg,
                     targetFlangePose.rpy[0]*180/M_PI, targetFlangePose.rpy[1]*180/M_PI, targetFlangePose.rpy[2]*180/M_PI,
                     verifyPose.rpy[0]*180/M_PI, verifyPose.rpy[1]*180/M_PI, verifyPose.rpy[2]*180/M_PI);
            // Stop streaming — orientation cannot be maintained
            m_tcpVelocity = 0.0;
            m_targetTcpVelocity = 0.0;
            m_cartesianStreaming = false;
            m_isJogging = false;
            m_currentDirection = 0;
            m_currentSpeed = 0.0;
            return;
        }
    }

    // === Check soft limits and command joint positions ===
    std::array<double, 6> targetDeg;
    for (int i = 0; i < 6; i++) {
        targetDeg[i] = ikResult->angles[i] * 180.0 / M_PI;
        targetDeg[i] = std::clamp(targetDeg[i],
                                  m_minLimits[i] + SOFT_LIMIT_MARGIN,
                                  m_maxLimits[i] - SOFT_LIMIT_MARGIN);
    }

    // Check if any joint hit soft limit
    bool hitLimit = false;
    for (int i = 0; i < 6; i++) {
        double unclamped = ikResult->angles[i] * 180.0 / M_PI;
        if (std::abs(unclamped - targetDeg[i]) > 0.01) {
            hitLimit = true;
            break;
        }
    }

    if (hitLimit) {
        m_tcpVelocity = 0.0;
        m_targetTcpVelocity = 0.0;
        m_cartesianStreaming = false;
        m_isJogging = false;
        m_currentDirection = 0;
        m_currentSpeed = 0.0;
        LOG_WARN("JogController: Cartesian streaming stopped - joint soft limit reached");
        return;
    }

    // === Command positions directly (bypasses G-code/Ruckig pipeline) ===
    m_firmwareDriver->setJointPositionsDirect(targetDeg);
}

std::string JogController::jogCartesianStep(int axis, int direction, double increment, double speedPercent, int frame) {
    if (!m_urdfFk || !m_kdlKin) {
        return "Kinematics not initialized for Cartesian jog";
    }

    if (increment <= 0) {
        return "Increment must be positive";
    }

    auto positions = m_firmwareDriver->getJointPositions();

    kinematics::JointAngles jointRad;
    for (int i = 0; i < 6; i++) {
        jointRad[i] = positions[i] * M_PI / 180.0;
    }

    // FK returns last-link pose (no flange offset, no tool TCP)
    auto flangePose = m_urdfFk->compute(jointRad);

    // Compute TRUE TCP pose = T_lastLink × T_composed (T_flangeOffset × T_toolTcp)
    kinematics::TCPPose tcpPose = flangePose;
    if (m_hasToolOffset) {
        Eigen::Matrix4d T_flange = flangePose.toTransform();
        Eigen::Matrix4d T_tcp = T_flange * m_toolTransform;
        tcpPose = kinematics::TCPPose::fromTransform(T_tcp);
    }

    kinematics::TCPPose targetTcpPose = tcpPose;
    double delta = increment * direction;  // mm for linear, deg for rotation

    // Delta applied on true TCP pose
    transformDeltaToWorld(frame, axis, delta, tcpPose, targetTcpPose);

    // Back-calculation: T_flange_desired = T_tcp_desired × T_tool⁻¹
    kinematics::TCPPose targetFlangePose = targetTcpPose;
    if (m_hasToolOffset) {
        Eigen::Matrix4d T_tcp_desired = targetTcpPose.toTransform();
        Eigen::Matrix4d T_flange_desired = T_tcp_desired * m_toolInvTransform;
        targetFlangePose = kinematics::TCPPose::fromTransform(T_flange_desired);
    }

    // IK solves for FLANGE pose
    std::optional<kinematics::IKSolution> ikResult;
    if (m_kdlKin && m_kdlKin->isInitialized()) {
        ikResult = m_kdlKin->computeIK(targetFlangePose, jointRad);
    }
    if (!ikResult.has_value()) {
        return "IK solution not found (out of workspace)";
    }

    std::ostringstream gcode;
    gcode << "$J=G90";
    double maxFeedRate = 0;
    for (int i = 0; i < 6; i++) {
        double targetDeg = ikResult->angles[i] * 180.0 / M_PI;
        targetDeg = std::clamp(targetDeg, m_minLimits[i] + SOFT_LIMIT_MARGIN,
                                          m_maxLimits[i] - SOFT_LIMIT_MARGIN);
        gcode << " J" << i << ":" << targetDeg;
        maxFeedRate = std::max(maxFeedRate, calculateFeedRate(i, speedPercent));
    }
    gcode << " F" << maxFeedRate;

    LOG_DEBUG("JogController: Cartesian step axis={} dir={} inc={} frame={} -> {}",
              axis, direction, increment, frame, gcode.str());

    m_firmwareDriver->sendCommand(gcode.str());

    m_isJogging = true;
    m_currentAxis = axis;
    m_currentDirection = direction;
    m_currentSpeed = speedPercent;
    m_currentMode = 1;

    return "";
}

// ========================================================================
// State and Position
// ========================================================================

ipc::JogStatePayload JogController::getState() const {
    ipc::JogStatePayload state;
    state.enabled = m_enabled;
    state.isMoving = m_isJogging;
    state.currentMode = m_currentMode;
    state.currentAxis = m_currentAxis;
    state.currentDirection = m_currentDirection;
    state.currentSpeed = m_currentSpeed;
    return state;
}

std::array<double, 6> JogController::getJointPositions() const {
    if (m_firmwareDriver) {
        return m_firmwareDriver->getJointPositions();
    }
    return {};
}

std::array<double, 6> JogController::getTcpPose() const {
    if (!m_urdfFk || !m_firmwareDriver) {
        return {};
    }

    auto positions = m_firmwareDriver->getJointPositions();
    kinematics::JointAngles jointRad;
    for (int i = 0; i < 6; i++) {
        jointRad[i] = positions[i] * M_PI / 180.0;
    }

    // FK returns last-link pose (no flange offset, no tool TCP)
    auto flangePose = m_urdfFk->compute(jointRad);

    // Apply composed transform (T_flangeOffset × T_toolTcp)
    kinematics::TCPPose pose = flangePose;
    if (m_hasToolOffset) {
        Eigen::Matrix4d T_flange = flangePose.toTransform();
        Eigen::Matrix4d T_tcp = T_flange * m_toolTransform;
        pose = kinematics::TCPPose::fromTransform(T_tcp);
    }

    auto vec = pose.toVector();

    std::array<double, 6> result;
    for (int i = 0; i < 6; i++) {
        result[i] = vec[i];
    }
    // Convert RPY from radians to degrees
    for (int i = 3; i < 6; i++) {
        result[i] = result[i] * 180.0 / M_PI;
    }
    return result;
}

std::optional<kinematics::IKSolution> JogController::computeIK(
    const std::array<double, 6>& targetPose,
    const std::array<double, 6>& currentJoints) const
{
    if (!m_kdlKin || !m_kdlKin->isInitialized()) {
        LOG_WARN("JogController::computeIK: KDL kinematics not initialized");
        return std::nullopt;
    }

    // Convert target pose [x,y,z,rx,ry,rz] (mm/deg) to TCPPose
    kinematics::Vector3d position(targetPose[0], targetPose[1], targetPose[2]);
    double rx = targetPose[3] * M_PI / 180.0;
    double ry = targetPose[4] * M_PI / 180.0;
    double rz = targetPose[5] * M_PI / 180.0;

    kinematics::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());

    kinematics::TCPPose tcpTarget;
    tcpTarget.position = position;
    tcpTarget.rotation = rotation;
    tcpTarget.rpy = kinematics::Vector3d(rx, ry, rz);

    // If tool offset exists, back-calculate flange target
    kinematics::TCPPose flangeTarget = tcpTarget;
    if (m_hasToolOffset) {
        Eigen::Matrix4d T_tcp_desired = tcpTarget.toTransform();
        Eigen::Matrix4d T_flange_desired = T_tcp_desired * m_toolInvTransform;
        flangeTarget = kinematics::TCPPose::fromTransform(T_flange_desired);
    }

    // Convert current joints from degrees to radians
    kinematics::JointAngles jointRad;
    for (int i = 0; i < 6; i++) {
        jointRad[i] = currentJoints[i] * M_PI / 180.0;
    }

    // Compute IK via KDL
    auto result = m_kdlKin->computeIK(flangeTarget, jointRad);
    if (!result.has_value()) {
        // Fallback to DH IK if available
        if (m_ik) {
            auto dhResult = m_ik->compute(flangeTarget, jointRad);
            if (dhResult.has_value()) {
                return dhResult;
            }
        }
        return std::nullopt;
    }

    return result;
}

void JogController::update(double dt) {
    if (!m_firmwareDriver || !m_enabled) return;

    // Velocity streaming Cartesian jog runs its own control loop
    if (m_cartesianStreaming) {
        m_firmwareDriver->update(dt);
        updateCartesianVelocityStreaming(dt);
        return;
    }

    m_firmwareDriver->update(dt);

    // Check if G-code motion completed (joint jog or step jog)
    if (m_isJogging) {
        auto resp = m_firmwareDriver->getResponse();
        if (resp == "ok") {
            m_isJogging = false;
            m_currentDirection = 0;
            m_currentSpeed = 0.0;
        }
    }
}

void JogController::emergencyStop() {
    if (m_firmwareDriver) {
        m_firmwareDriver->emergencyStop();
    }
    m_isJogging = false;
    m_cartesianStreaming = false;
    m_tcpVelocity = 0.0;
    m_targetTcpVelocity = 0.0;
    m_enabled = false;
    m_currentDirection = 0;
    m_currentSpeed = 0.0;
    LOG_WARN("JogController: Emergency stop!");
}

// ========================================================================
// Private Helpers
// ========================================================================

std::string JogController::validateJogRequest(int mode, int axis, int direction, double speedPercent) {
    if (!m_enabled) return "Jog mode not enabled";
    if (!m_firmwareDriver) return "No firmware driver";
    if (axis < 0 || axis >= 6) return "Invalid axis: " + std::to_string(axis);
    if (direction != -1 && direction != 1) return "Invalid direction: must be -1 or +1";
    if (speedPercent < 1.0 || speedPercent > 100.0) return "Speed must be 1-100%";
    if (mode != 0 && mode != 1) return "Invalid mode: must be 0 (Joint) or 1 (Cartesian)";
    return "";
}

bool JogController::checkSoftLimits(int axis, double targetAngle) {
    return targetAngle >= (m_minLimits[axis] + SOFT_LIMIT_MARGIN) &&
           targetAngle <= (m_maxLimits[axis] - SOFT_LIMIT_MARGIN);
}

std::string JogController::generateJogGcode(int axis, double targetAngle, double feedRate) {
    return "$J=G90 J" + std::to_string(axis) + ":" +
           std::to_string(targetAngle) + " F" + std::to_string(feedRate);
}

double JogController::calculateFeedRate(int axis, double speedPercent) {
    return m_maxVelocities[axis] * (speedPercent / 100.0) * 60.0;
}

} // namespace jog
} // namespace robot_controller
