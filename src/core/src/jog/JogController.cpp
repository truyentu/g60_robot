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

    // Initialize TWA config from joint limits
    m_twaConfig.psThreshold = 0.3;   // PS≈0.3 ↔ J5≈10° (from survey test)
    m_twaConfig.dlsLambdaMax = 0.1;
    m_twaConfig.jointLimitWeights = kinematics::Vector6d::Ones();
    m_twaConfig.singularityWeights = kinematics::Vector6d::Ones();
    for (size_t i = 0; i < joints.size() && i < kinematics::NUM_JOINTS; ++i) {
        m_twaConfig.jointMidPositions[i] = (joints[i].minAngle + joints[i].maxAngle) / 2.0;
        m_twaConfig.jointMinLimits[i] = joints[i].minAngle;
        m_twaConfig.jointMaxLimits[i] = joints[i].maxAngle;
    }

    LOG_INFO("JogController: URDF FK + KDL IK initialized ({} joints)", joints.size());
    LOG_INFO("JogController: TWA config limits (rad):");
    for (size_t i = 0; i < joints.size() && i < kinematics::NUM_JOINTS; ++i) {
        LOG_INFO("  J{}: min={:.4f} max={:.4f} mid={:.4f} (deg: [{:.1f}, {:.1f}])",
            i + 1,
            m_twaConfig.jointMinLimits[i], m_twaConfig.jointMaxLimits[i],
            m_twaConfig.jointMidPositions[i],
            m_twaConfig.jointMinLimits[i] * 180.0 / M_PI,
            m_twaConfig.jointMaxLimits[i] * 180.0 / M_PI);
    }
    LOG_INFO("JogController: Jog soft limits (deg):");
    for (int i = 0; i < 6; ++i) {
        LOG_INFO("  J{}: [{:.1f}, {:.1f}]", i + 1, m_minLimits[i], m_maxLimits[i]);
    }
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
        LOG_INFO("JogController: setJointLimits J{}=[{:.1f}, {:.1f}] deg", joint + 1, minDeg, maxDeg);
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

    // Joint jog → V2 structured API: jogStart
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

    // Calculate speed in steps/ms
    double feedRateDegPerSec = m_maxVelocities[axis] * (speedPercent / 100.0);
    uint16_t speedStepsPerMs = static_cast<uint16_t>(
        std::clamp(feedRateDegPerSec * m_firmwareDriver->degreesToSteps(axis, 1.0) / 1000.0, 1.0, 65535.0));

    LOG_DEBUG("JogController: Continuous jog J{} dir={} speed={}% -> V2 jogStart(axis={}, dir={}, speed={})",
              axis, direction, speedPercent, axis, direction, speedStepsPerMs);

    bool ok = m_firmwareDriver->jogStart(static_cast<uint8_t>(axis),
                               static_cast<int8_t>(direction),
                               speedStepsPerMs);

    if (!ok) {
        LOG_ERROR("JogController: jogStart failed (drives not enabled?)");
        return "Drives not enabled";
    }

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

    // Joint jog: V2 structured jogStop
    m_firmwareDriver->jogStop();

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

    // Joint step jog — V2: use setJointPositionsDirect for precise step
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

    // Use setJointPositionsDirect for step jog (precise single-step move)
    auto targetPositions = positions;
    targetPositions[axis] = target;

    LOG_DEBUG("JogController: Step jog J{} dir={} inc={} -> target={} (V2 direct)",
              axis, direction, increment, target);

    m_firmwareDriver->setJointPositionsDirect(targetPositions);

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

    // Apply T1 speed cap (KSS 8.3 §3.5.12)
    if (isLinear && maxSpeed * (speedPercent / 100.0) > m_speedCapMmPerS) {
        maxSpeed = m_speedCapMmPerS / (speedPercent / 100.0);
    }

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

    // === Capture initial TCP pose for fixed-reference jogging ===
    // This prevents integration drift: instead of using currentPose.rotation
    // (which drifts due to IK epsilon) to compute each step's direction,
    // we lock the rotation at session start and use it throughout.
    m_jogTotalDistance = 0.0;
    m_jogMaxDistance = 0.0;
    {
        auto positions = m_firmwareDriver->getJointPositions();
        kinematics::JointAngles jointRad;
        for (int i = 0; i < 6; i++) {
            jointRad[i] = positions[i] * M_PI / 180.0;
        }
        auto flangePose = m_urdfFk->compute(jointRad);
        m_jogStartTcpPose = flangePose;
        if (m_hasToolOffset) {
            Eigen::Matrix4d T_flange = flangePose.toTransform();
            Eigen::Matrix4d T_tcp = T_flange * m_toolTransform;
            m_jogStartTcpPose = kinematics::TCPPose::fromTransform(T_tcp);
        }

        // Smart Seed: record start joints as first snapshot
        m_jogJointHistory.clear();
        m_jogJointHistory.push_back({0.0, jointRad});
    }

    LOG_DEBUG("JogController: Cartesian velocity streaming started axis={} dir={} speed={}% frame={} targetV={} "
              "startPos=[{:.1f},{:.1f},{:.1f}]",
              axis, direction, speedPercent, frame, targetSpeed,
              m_jogStartTcpPose.position[0], m_jogStartTcpPose.position[1], m_jogStartTcpPose.position[2]);

    return "";
}

// ============================================================================
// TWA Helper Methods
// ============================================================================

kinematics::Vector3d JogController::getTorchAxis() const {
    if (!m_urdfFk || !m_firmwareDriver) {
        return kinematics::Vector3d(0, 0, 1);
    }
    auto positions = m_firmwareDriver->getJointPositions();
    kinematics::JointAngles jointRad;
    for (int i = 0; i < 6; i++)
        jointRad[i] = positions[i] * M_PI / 180.0;

    auto flangePose = m_urdfFk->compute(jointRad);

    if (m_hasToolOffset) {
        Eigen::Matrix4d T_tcp = flangePose.toTransform() * m_toolTransform;
        auto tcpPose = kinematics::TCPPose::fromTransform(T_tcp);
        return tcpPose.rotation.col(2);
    }

    return flangePose.rotation.col(2);
}

kinematics::Vector6d JogController::buildDesiredTwist(double dt) const {
    kinematics::Vector6d twist = kinematics::Vector6d::Zero();

    bool isLinear = (m_currentAxis < 3);
    double velocity = m_tcpVelocity * m_currentDirection;

    if (isLinear) {
        kinematics::Vector3d direction = kinematics::Vector3d::Zero();

        if (m_currentFrame == 2) {
            // Tool frame: project through initial rotation
            kinematics::Vector3d axis_tool = kinematics::Vector3d::Zero();
            axis_tool[m_currentAxis] = 1.0;
            direction = m_jogStartTcpPose.rotation * axis_tool;
        } else {
            direction[m_currentAxis] = 1.0;
        }

        // KDL twist ordering: [angular(3), linear(3)]
        twist(3) = direction[0] * velocity * dt;
        twist(4) = direction[1] * velocity * dt;
        twist(5) = direction[2] * velocity * dt;
    } else {
        int rotAxis = m_currentAxis - 3;
        kinematics::Vector3d direction = kinematics::Vector3d::Zero();

        if (m_currentFrame == 2) {
            kinematics::Vector3d rot_tool = kinematics::Vector3d::Zero();
            rot_tool[rotAxis] = 1.0;
            direction = m_jogStartTcpPose.rotation * rot_tool;
        } else {
            direction[rotAxis] = 1.0;
        }

        double velocityRad = velocity * M_PI / 180.0;
        twist(0) = direction[0] * velocityRad * dt;
        twist(1) = direction[1] * velocityRad * dt;
        twist(2) = direction[2] * velocityRad * dt;
    }

    return twist;
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
        m_singularityActive = false;
        m_currentPS = 0.0;
        m_currentVelocityScale = 1.0;
        LOG_DEBUG("JogController: Cartesian streaming fully stopped");
        return;
    }

    // === TWA Singularity Check ===
    // Compute PS index from current Jacobian to detect proximity to singularity.
    // For jog mode: only use velocity scaling on position-level IK.
    // Velocity-level IK (TWA branch) is disabled for jog — it causes jumps and
    // orientation drift. Position-level IK handles tool frame jog correctly.
    auto positions = m_firmwareDriver->getJointPositions();
    kinematics::JointAngles jointRadForTWA;
    for (int i = 0; i < 6; i++) {
        jointRadForTWA[i] = positions[i] * M_PI / 180.0;
    }

    kinematics::Jacobian J = m_kdlKin->computeJacobian(jointRadForTWA);
    double ps = kinematics::TwistDecomposition::computePS(J);

    // Guard: PS = 1e6 means degenerate Jacobian (workspace boundary, not wrist singularity)
    static constexpr double PS_DEGENERATE_GUARD = 1e5;
    if (ps >= PS_DEGENERATE_GUARD) {
        ps = 0.0;
    }
    m_currentPS = ps;

    // Hysteresis: enter at psThreshold (0.3), exit at psThresholdExit (0.2)
    double enterThreshold = m_twaConfig.psThreshold;
    double exitThreshold = m_twaConfig.psThresholdExit;

    if (!m_singularityActive && ps > enterThreshold) {
        m_singularityActive = true;
        m_singEntryJoints = jointRadForTWA;
        LOG_WARN("JogController: Entering singularity zone PS={:.2f} (threshold={:.2f})",
                 ps, enterThreshold);
    } else if (m_singularityActive && ps < exitThreshold) {
        LOG_INFO("JogController: Exiting singularity zone PS={:.2f} (exit={:.2f})",
                 ps, exitThreshold);
        m_singularityActive = false;
    }

    // Smooth quadratic velocity scaling (same formula as TwistDecomposition::resolve)
    double velocityScale = 1.0;
    if (m_singularityActive && enterThreshold > 0.0) {
        double ratio = enterThreshold / ps;
        double ratioSq = ratio * ratio;
        velocityScale = std::max(m_twaConfig.velocityScaleMin,
                                  1.0 - (1.0 - m_twaConfig.velocityScaleMin) * (1.0 - ratioSq));
    }
    m_currentVelocityScale = velocityScale;

    // ================================================================
    // Position-level IK path (always used for jog)
    // Velocity scaling from PS is applied via velocityScale above.
    // ================================================================
    double stepDelta = m_tcpVelocity * velocityScale * dt;  // TWA-scaled velocity
    m_jogTotalDistance += stepDelta;

    // === Smart Seed: Record joint snapshots & select IK seed ===
    // When moving forward (totalDistance increasing beyond high-water mark):
    //   - Record joint snapshots every SEED_SNAPSHOT_INTERVAL mm
    //   - Use current joints as IK seed (normal behavior)
    // When moving backward (totalDistance < maxDistance):
    //   - Lookup the closest snapshot at or below current totalDistance
    //   - Use that snapshot's joints as IK seed → solver retraces original branch
    double absDist = std::abs(m_jogTotalDistance);
    bool movingForward = (absDist >= m_jogMaxDistance - 0.01);

    if (movingForward) {
        m_jogMaxDistance = absDist;
        // Record snapshot if passed next interval
        double lastSnapshotDist = m_jogJointHistory.empty() ? 0.0 : m_jogJointHistory.back().distance;
        if (absDist >= lastSnapshotDist + SEED_SNAPSHOT_INTERVAL) {
            m_jogJointHistory.push_back({absDist, jointRadForTWA});
        }
    }

    // Skip if total distance too small (avoid unnecessary IK computations)
    if (std::abs(m_jogTotalDistance) < 0.0001) return;

    // === Select IK seed: historical snapshot when reversing, current when advancing ===
    kinematics::JointAngles jointRad = jointRadForTWA;  // default: current joints
    if (!movingForward && m_jogJointHistory.size() > 1) {
        // Binary search for closest snapshot at or below current distance
        // History is sorted by distance (ascending) since we only append when advancing
        size_t lo = 0, hi = m_jogJointHistory.size() - 1;
        while (lo < hi) {
            size_t mid = lo + (hi - lo + 1) / 2;
            if (m_jogJointHistory[mid].distance <= absDist) {
                lo = mid;
            } else {
                hi = mid - 1;
            }
        }
        jointRad = m_jogJointHistory[lo].joints;

        // Trim history beyond current position (robot won't go forward from here)
        if (lo + 1 < m_jogJointHistory.size()) {
            m_jogJointHistory.resize(lo + 1);
        }
        m_jogMaxDistance = absDist;
    }

    // === Compute target TCP pose ABSOLUTELY from start pose ===
    // Instead of: target = current + delta (drifts due to IK error)
    // We use:     target = start + totalDistance * fixedDirection
    // This ensures the path is perfectly straight regardless of IK accuracy.
    kinematics::TCPPose targetTcpPose = m_jogStartTcpPose;

    if (m_currentFrame == 2) {
        // TOOL frame: use FIXED initial rotation to project direction
        kinematics::Matrix3d R_start = m_jogStartTcpPose.rotation;

        if (isLinear) {
            kinematics::Vector3d axis_tool = kinematics::Vector3d::Zero();
            axis_tool[m_currentAxis] = 1.0;
            kinematics::Vector3d direction_world = R_start * axis_tool;
            targetTcpPose.position = m_jogStartTcpPose.position + m_jogTotalDistance * direction_world;
            // Orientation stays EXACTLY the same as start
            targetTcpPose.rotation = m_jogStartTcpPose.rotation;
            targetTcpPose.rpy = m_jogStartTcpPose.rpy;
            targetTcpPose.quaternion = m_jogStartTcpPose.quaternion;
        } else {
            // Rotation jog in tool frame
            kinematics::Vector3d rot_axis_tool = kinematics::Vector3d::Zero();
            rot_axis_tool[m_currentAxis - 3] = 1.0;
            kinematics::Vector3d rot_axis_world = R_start * rot_axis_tool;
            double totalRad = m_jogTotalDistance * M_PI / 180.0;
            kinematics::AngleAxisd aa(totalRad, rot_axis_world);
            targetTcpPose.rotation = aa.toRotationMatrix() * m_jogStartTcpPose.rotation;
            targetTcpPose.rpy = kinematics::rotationToRPY(targetTcpPose.rotation);
            targetTcpPose.quaternion = kinematics::Quaterniond(targetTcpPose.rotation);
        }
    } else {
        // WORLD/BASE/USER frame: apply total delta directly on start pose
        if (isLinear) {
            targetTcpPose.position[m_currentAxis] = m_jogStartTcpPose.position[m_currentAxis] + m_jogTotalDistance;
        } else {
            double totalRad = m_jogTotalDistance * M_PI / 180.0;
            targetTcpPose.rpy[m_currentAxis - 3] = m_jogStartTcpPose.rpy[m_currentAxis - 3] + totalRad;
            targetTcpPose.rotation = kinematics::rpyToRotation(targetTcpPose.rpy);
            targetTcpPose.quaternion = kinematics::Quaterniond(targetTcpPose.rotation);
        }
    }

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

        // Also verify at TCP level (tool amplifies orientation error)
        double tcpPosErr = 0.0;
        if (m_hasToolOffset) {
            Eigen::Matrix4d T_actual_flange = verifyPose.toTransform();
            Eigen::Matrix4d T_actual_tcp = T_actual_flange * m_toolTransform;
            Eigen::Matrix4d T_desired_tcp = targetTcpPose.toTransform();
            kinematics::Vector3d actualTcpPos = T_actual_tcp.block<3,1>(0,3);
            kinematics::Vector3d desiredTcpPos = T_desired_tcp.block<3,1>(0,3);
            tcpPosErr = (actualTcpPos - desiredTcpPos).norm();
        }

        // Periodic debug logging (every ~100 steps based on totalDistance)
        static int debugCounter = 0;
        if (++debugCounter % 100 == 0) {
            LOG_DEBUG("JogController: [DIAG] frame={} axis={} totalDist={:.1f}mm "
                     "flangeErr={:.3f}mm orientErr={:.3f}deg tcpErr={:.3f}mm "
                     "targetTCP=[{:.1f},{:.1f},{:.1f}] hasToolOff={}",
                     m_currentFrame, m_currentAxis, m_jogTotalDistance,
                     posErr, orientErrDeg, tcpPosErr,
                     targetTcpPose.position[0], targetTcpPose.position[1], targetTcpPose.position[2],
                     m_hasToolOffset);
        }

        // Orientation tolerance: relaxed near singularity (like ABB SingArea\Wrist)
        // At wrist singularity J4/J6 coupling makes exact orientation impossible,
        // but the deviation is only around the tool axis (spin) which doesn't affect welding.
        double orientTolerance = m_singularityActive ? 5.0 : 0.5;  // degrees

        if (orientErrDeg > orientTolerance) {
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

    // V2: Use setJointPositionsDirect for Cartesian step jog
    std::array<double, 6> targetDeg;
    for (int i = 0; i < 6; i++) {
        targetDeg[i] = ikResult->angles[i] * 180.0 / M_PI;
        targetDeg[i] = std::clamp(targetDeg[i], m_minLimits[i] + SOFT_LIMIT_MARGIN,
                                                  m_maxLimits[i] - SOFT_LIMIT_MARGIN);
    }

    LOG_DEBUG("JogController: Cartesian step axis={} dir={} inc={} frame={} (V2 direct)",
              axis, direction, increment, frame);

    m_firmwareDriver->setJointPositionsDirect(targetDeg);

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

kinematics::TCPPose JogController::getTcpPoseObject() const {
    if (!m_urdfFk || !m_firmwareDriver) {
        return {};
    }

    auto positions = m_firmwareDriver->getJointPositions();
    kinematics::JointAngles jointRad;
    for (int i = 0; i < 6; i++) {
        jointRad[i] = positions[i] * M_PI / 180.0;
    }

    auto flangePose = m_urdfFk->compute(jointRad);

    kinematics::TCPPose pose = flangePose;
    if (m_hasToolOffset) {
        Eigen::Matrix4d T_flange = flangePose.toTransform();
        Eigen::Matrix4d T_tcp = T_flange * m_toolTransform;
        pose = kinematics::TCPPose::fromTransform(T_tcp);
    }

    return pose;
}

std::optional<kinematics::IKSolution> JogController::computeIK(
    const std::array<double, 6>& targetPose,
    const std::array<double, 6>& currentJoints,
    bool skipConfigFlipCheck) const
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
    auto result = m_kdlKin->computeIK(flangeTarget, jointRad, skipConfigFlipCheck);
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

    // === Verify IK accuracy at TCP level ===
    if (m_urdfFk) {
        auto verifyFlange = m_urdfFk->compute(result->angles);
        kinematics::TCPPose verifyTcp = verifyFlange;
        if (m_hasToolOffset) {
            Eigen::Matrix4d T_vf = verifyFlange.toTransform();
            Eigen::Matrix4d T_vt = T_vf * m_toolTransform;
            verifyTcp = kinematics::TCPPose::fromTransform(T_vt);
        }
        double tcpPosErr = (verifyTcp.position - tcpTarget.position).norm();
        kinematics::Matrix3d R_err = verifyTcp.rotation.transpose() * tcpTarget.rotation;
        double orientErr = std::acos(std::clamp((R_err.trace() - 1.0) / 2.0, -1.0, 1.0));
        double orientErrDeg = orientErr * 180.0 / M_PI;

        static int ikDiagCounter = 0;
        if (++ikDiagCounter % 10 == 0) {
            LOG_INFO("computeIK [DIAG]: targetTCP=[{:.1f},{:.1f},{:.1f}] actualTCP=[{:.1f},{:.1f},{:.1f}] "
                     "posErr={:.3f}mm orientErr={:.3f}deg hasToolOff={} iter={}",
                     tcpTarget.position[0], tcpTarget.position[1], tcpTarget.position[2],
                     verifyTcp.position[0], verifyTcp.position[1], verifyTcp.position[2],
                     tcpPosErr, orientErrDeg, m_hasToolOffset, result->iterations);
        }
    }

    return result;
}

std::optional<kinematics::IKSolution> JogController::computeIK(
    const kinematics::TCPPose& targetTcpPose,
    const std::array<double, 6>& currentJoints,
    bool skipConfigFlipCheck) const
{
    if (!m_kdlKin || !m_kdlKin->isInitialized()) {
        LOG_WARN("JogController::computeIK(TCPPose): KDL kinematics not initialized");
        return std::nullopt;
    }

    // If tool offset exists, back-calculate flange target
    kinematics::TCPPose flangeTarget = targetTcpPose;
    if (m_hasToolOffset) {
        Eigen::Matrix4d T_tcp_desired = targetTcpPose.toTransform();
        Eigen::Matrix4d T_flange_desired = T_tcp_desired * m_toolInvTransform;
        flangeTarget = kinematics::TCPPose::fromTransform(T_flange_desired);
    }

    // Convert current joints from degrees to radians
    kinematics::JointAngles jointRad;
    for (int i = 0; i < 6; i++) {
        jointRad[i] = currentJoints[i] * M_PI / 180.0;
    }

    // Compute IK via KDL
    auto result = m_kdlKin->computeIK(flangeTarget, jointRad, skipConfigFlipCheck);
    if (!result.has_value()) {
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

int JogController::getCriticalJoint() const {
    auto positions = m_firmwareDriver->getJointPositions();
    kinematics::JointAngles current;
    for (int i = 0; i < 6; i++)
        current[i] = positions[i] * M_PI / 180.0;
    return kinematics::TwistDecomposition::findCriticalJoint(
        current, m_twaConfig.jointMinLimits, m_twaConfig.jointMaxLimits);
}

} // namespace jog
} // namespace robot_controller
