#pragma once

#include "../ipc/JogPayloads.hpp"
#include "../firmware/IFirmwareDriver.hpp"
#include "../kinematics/ForwardKinematics.hpp"
#include "../kinematics/UrdfForwardKinematics.hpp"
#include "../kinematics/KDLKinematics.hpp"
#include "../kinematics/InverseKinematics.hpp"
#include <Eigen/Dense>
#include <memory>
#include <array>

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

    static constexpr double SOFT_LIMIT_MARGIN = 0.5;
    static constexpr double CONTINUOUS_JOG_RANGE = 9999.0;
    static constexpr double CARTESIAN_JOG_STEP = 5.0;     // mm per incremental jog step
    static constexpr double CARTESIAN_ROT_STEP = 1.0;     // deg per incremental rotation step
    static constexpr double MAX_TCP_LINEAR_SPEED = 1000.0; // mm/s at 100% speed (MA2010 max ~2000)
    static constexpr double MAX_TCP_ROT_SPEED = 180.0;     // deg/s at 100% speed
    static constexpr double TCP_ACCEL_RATE = 2000.0;       // mm/s^2 acceleration
    static constexpr double TCP_ROT_ACCEL_RATE = 600.0;    // deg/s^2 rotation acceleration
};

} // namespace jog
} // namespace robot_controller
