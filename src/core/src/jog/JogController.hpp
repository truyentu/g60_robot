#pragma once

#include "../ipc/JogPayloads.hpp"
#include "../firmware/IFirmwareDriver.hpp"
#include "../kinematics/ForwardKinematics.hpp"
#include "../kinematics/UrdfForwardKinematics.hpp"
#include "../kinematics/KDLKinematics.hpp"
#include "../kinematics/InverseKinematics.hpp"
#include "../kinematics/TwistDecomposition.hpp"
#include <Eigen/Dense>
#include <memory>
#include <array>
#include <optional>
#include <vector>

namespace robot_controller {
namespace jog {

class JogController {
public:
    JogController();
    ~JogController() = default;

    void setFirmwareDriver(std::shared_ptr<firmware::IFirmwareDriver> driver);

    // Initialize kinematics for Cartesian jog
    void initializeKinematics(const kinematics::RobotKinematicConfig& config);
    void initializeUrdfKinematics(const std::vector<kinematics::UrdfJointDef>& joints,
                                   const kinematics::Vector3d& toolOffset);
    bool hasKinematics() const { return m_urdfFk != nullptr && m_kdlKin != nullptr && m_kdlKin->isInitialized(); }

    // Tool transform for True TCP jogging (Phase 11)
    void setToolTransform(const Eigen::Matrix4d& T_tool);

    // Initialize joint limits from robot config
    void setJointLimits(int joint, double minDeg, double maxDeg);
    void setMaxVelocity(int joint, double degPerSec);

    bool enable();
    bool disable();
    bool isEnabled() const { return m_enabled; }

    // Continuous jog - returns error message (empty = success)
    std::string startContinuousJog(int mode, int axis, int direction, double speedPercent, int frame = 0);
    std::string stopJog();

    // Incremental jog
    std::string jogStep(int mode, int axis, int direction, double increment, double speedPercent, int frame = 0);

    ipc::JogStatePayload getState() const;
    std::array<double, 6> getJointPositions() const;

    // Get TCP pose from FK (returns [x,y,z,rx,ry,rz])
    std::array<double, 6> getTcpPose() const;

    // Get TCP pose as full TCPPose object (with rotation matrix, avoids gimbal lock)
    kinematics::TCPPose getTcpPoseObject() const;

    // Compute IK for a target pose (for 3D gizmo jogging)
    // targetPose: [x,y,z,rx,ry,rz] in mm/degrees
    // currentJoints: [j1..j6] in degrees
    // Returns: IKSolution if successful, nullopt if failed
    std::optional<kinematics::IKSolution> computeIK(
        const std::array<double, 6>& targetPose,
        const std::array<double, 6>& currentJoints,
        bool skipConfigFlipCheck = false) const;

    // Compute IK from TCPPose directly (avoids RPY gimbal lock issues)
    std::optional<kinematics::IKSolution> computeIK(
        const kinematics::TCPPose& targetTcpPose,
        const std::array<double, 6>& currentJoints,
        bool skipConfigFlipCheck = false) const;

    void update(double dt);
    void emergencyStop();

private:
    std::string validateJogRequest(int mode, int axis, int direction, double speedPercent);
    bool checkSoftLimits(int axis, double targetAngle);
    std::string generateJogGcode(int axis, double targetAngle, double feedRate);
    double calculateFeedRate(int axis, double speedPercent);

    // Cartesian jog helpers
    std::string startCartesianJog(int axis, int direction, double speedPercent, int frame);
    std::string jogCartesianStep(int axis, int direction, double increment, double speedPercent, int frame);

    // Frame transformation: convert delta from given frame to world frame
    void transformDeltaToWorld(int frame, int axis, double delta,
                               const kinematics::TCPPose& currentPose,
                               kinematics::TCPPose& targetPose);

    // Velocity streaming update for Cartesian jog
    void updateCartesianVelocityStreaming(double dt);

    bool m_enabled{false};
    bool m_isJogging{false};
    int m_currentAxis{0};
    int m_currentDirection{0};
    double m_currentSpeed{0.0};
    int m_currentMode{0};
    int m_currentFrame{0};

    // Velocity streaming state for smooth Cartesian jog
    bool m_cartesianStreaming{false};
    double m_tcpVelocity{0.0};          // Current TCP speed (mm/s or deg/s)
    double m_targetTcpVelocity{0.0};    // Target TCP speed (ramp toward this)

    // Fixed-reference jog state (prevents integration drift in Tool/User frames)
    // When jogging in Tool frame, we store the initial TCP pose and accumulate
    // total distance. Each step computes target = startPose + totalDistance * fixedDirection
    // instead of incrementally adding to currentPose (which drifts due to IK error).
    kinematics::TCPPose m_jogStartTcpPose;      // TCP pose when jog session started
    double m_jogTotalDistance{0.0};               // Accumulated distance (mm or deg)

    // Smart Seed: joint history for IK seed selection when reversing jog direction.
    // Snapshots of joints are recorded every SEED_SNAPSHOT_INTERVAL mm along the path.
    // When totalDistance decreases (jog reversal), the closest snapshot is used as
    // IK seed to guide the solver back to the original configuration branch.
    struct JointSnapshot {
        double distance;
        kinematics::JointAngles joints;  // radians
    };
    std::vector<JointSnapshot> m_jogJointHistory;
    double m_jogMaxDistance{0.0};         // High-water mark of totalDistance
    static constexpr double SEED_SNAPSHOT_INTERVAL = 5.0;  // mm between snapshots

    std::array<double, 6> m_minLimits{-170, -190, -120, -185, -120, -350};
    std::array<double, 6> m_maxLimits{170, 45, 156, 185, 120, 350};
    std::array<double, 6> m_maxVelocities{156, 156, 176, 343, 384, 721};

    std::shared_ptr<firmware::IFirmwareDriver> m_firmwareDriver;

    // Kinematics (for Cartesian jog)
    std::unique_ptr<kinematics::UrdfForwardKinematics> m_urdfFk;   // URDF FK (primary)
    std::unique_ptr<kinematics::KDLKinematics> m_kdlKin;           // KDL IK (primary)
    std::unique_ptr<kinematics::ForwardKinematics> m_fk;           // DH FK (deprecated fallback)
    std::unique_ptr<kinematics::InverseKinematics> m_ik;           // DH IK (deprecated fallback)

    // Tool transform for True TCP (Phase 11)
    Eigen::Matrix4d m_toolTransform = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d m_toolInvTransform = Eigen::Matrix4d::Identity();
    bool m_hasToolOffset = false;

    // T1 mode speed cap: 1000 mm/s
    double m_speedCapMmPerS = 1000.0;

    static constexpr double SOFT_LIMIT_MARGIN = 0.5;
    static constexpr double CONTINUOUS_JOG_RANGE = 9999.0;
    static constexpr double CARTESIAN_JOG_STEP = 5.0;     // mm per incremental jog step
    static constexpr double CARTESIAN_ROT_STEP = 1.0;     // deg per incremental rotation step
    static constexpr double MAX_TCP_LINEAR_SPEED = 1000.0; // mm/s at 100% speed (MA2010 max ~2000)
    static constexpr double MAX_TCP_ROT_SPEED = 180.0;     // deg/s at 100% speed
    static constexpr double TCP_ACCEL_RATE = 2000.0;       // mm/s^2 acceleration
    static constexpr double TCP_ROT_ACCEL_RATE = 600.0;    // deg/s^2 rotation acceleration

    // TWA (Twist Decomposition Algorithm) for singularity avoidance
    kinematics::TWAConfig m_twaConfig;
    kinematics::JointAngles m_singEntryJoints{};    // Recorded at singularity entry
    bool m_singularityActive{false};
    double m_currentPS{0.0};
    double m_currentVelocityScale{1.0};

    // TWA helper methods for full resolution
    kinematics::Vector3d getTorchAxis() const;
    kinematics::Vector6d buildDesiredTwist(double dt) const;

public:
    void setSpeedCap(double capMmPerS) { m_speedCapMmPerS = capMmPerS; }
    double getSpeedCap() const { return m_speedCapMmPerS; }

    // TWA singularity accessors (for RobotController IPC publishing)
    double getCurrentPS() const { return m_currentPS; }
    bool isSingularityActive() const { return m_singularityActive; }
    double getVelocityScale() const { return m_currentVelocityScale; }
    int getCriticalJoint() const;
    kinematics::KDLKinematics* getKDL() const { return m_kdlKin.get(); }
};

} // namespace jog
} // namespace robot_controller
