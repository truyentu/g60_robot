/**
 * @file test_safety.cpp
 * @brief Safety system feature tests
 *
 * Tests for features SF001-SF007:
 * - SF001: SafetyMonitor
 * - SF002: Dual-Channel Safety
 * - SF003: E-Stop Handler
 * - SF004: Soft Limits
 * - SF005: Hard Limits
 * - SF006: Deadman Switch
 * - SF007: Velocity Monitoring
 */

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <array>

// Include safety-related headers
#include "state/StateMachine.hpp"
#include "logging/Logger.hpp"

using namespace robot_controller::state;
using namespace std::chrono_literals;

// =============================================================================
// Mock Safety Hardware for Testing
// =============================================================================

namespace robot_controller {
namespace safety {

/**
 * @brief Safety signal channel state
 */
struct SafetyChannel {
    bool estopPressed = false;
    bool safetyDoorOpen = false;
    bool deadmanHeld = false;
    uint64_t lastUpdateTime = 0;
};

/**
 * @brief Dual-channel safety monitor
 * Feature: SF001, SF002
 */
class SafetyMonitor {
public:
    static constexpr uint64_t DISCREPANCY_TIMEOUT_MS = 50;
    static constexpr double MAX_VELOCITY_T1 = 250.0;  // mm/s

    SafetyMonitor() = default;

    void setChannel1(const SafetyChannel& ch) {
        channel1_ = ch;
        channel1_.lastUpdateTime = getCurrentTimeMs();
        checkDiscrepancy();
    }

    void setChannel2(const SafetyChannel& ch) {
        channel2_ = ch;
        channel2_.lastUpdateTime = getCurrentTimeMs();
        checkDiscrepancy();
    }

    bool isEStopActive() const {
        // E-Stop is active if EITHER channel reports it (fail-safe)
        return channel1_.estopPressed || channel2_.estopPressed;
    }

    bool isSafetyDoorOpen() const {
        return channel1_.safetyDoorOpen || channel2_.safetyDoorOpen;
    }

    bool isDeadmanHeld() const {
        // Deadman must be held on BOTH channels (3-position switch)
        return channel1_.deadmanHeld && channel2_.deadmanHeld;
    }

    bool hasDiscrepancy() const {
        return discrepancyDetected_;
    }

    /**
     * @brief Check if velocity is within limits for current mode
     * Feature: SF007
     */
    bool isVelocityWithinLimits(double velocity, RobotMode mode) const {
        if (mode == RobotMode::T1) {
            return velocity <= MAX_VELOCITY_T1;
        }
        return true;  // No limit in T2/AUTO
    }

    /**
     * @brief Check soft limits for joint
     * Feature: SF004
     */
    bool checkSoftLimits(int jointIndex, double position,
                         double minLimit, double maxLimit) const {
        return position >= minLimit && position <= maxLimit;
    }

    /**
     * @brief Check hard limits (firmware level)
     * Feature: SF005
     */
    bool checkHardLimits(int jointIndex, bool limitSwitchTriggered) const {
        return !limitSwitchTriggered;
    }

    void update() {
        checkDiscrepancy();
    }

private:
    SafetyChannel channel1_;
    SafetyChannel channel2_;
    bool discrepancyDetected_ = false;
    uint64_t discrepancyStartTime_ = 0;

    uint64_t getCurrentTimeMs() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    }

    void checkDiscrepancy() {
        // Check if channels disagree
        bool estopMismatch = (channel1_.estopPressed != channel2_.estopPressed);
        bool doorMismatch = (channel1_.safetyDoorOpen != channel2_.safetyDoorOpen);
        bool deadmanMismatch = (channel1_.deadmanHeld != channel2_.deadmanHeld);

        bool currentMismatch = estopMismatch || doorMismatch || deadmanMismatch;

        if (currentMismatch) {
            if (discrepancyStartTime_ == 0) {
                discrepancyStartTime_ = getCurrentTimeMs();
            } else {
                uint64_t elapsed = getCurrentTimeMs() - discrepancyStartTime_;
                if (elapsed > DISCREPANCY_TIMEOUT_MS) {
                    discrepancyDetected_ = true;
                }
            }
        } else {
            discrepancyStartTime_ = 0;
            discrepancyDetected_ = false;
        }
    }
};

/**
 * @brief Deadman switch handler (3-position)
 * Feature: SF006
 *
 * 3-Position Deadman Switch:
 * - Position 1 (Released): NOT safe to move
 * - Position 2 (Pressed): Safe to move (enabling)
 * - Position 3 (Panic/Crushed): NOT safe to move (emergency)
 */
class DeadmanSwitch {
public:
    enum class Position {
        RELEASED = 0,   // Not pressed - disable
        PRESSED = 1,    // Partially pressed - enable
        PANIC = 2       // Fully pressed (panic) - disable
    };

    void setPosition(Position pos) {
        position_ = pos;
    }

    Position getPosition() const {
        return position_;
    }

    bool isEnabling() const {
        // Only middle position enables motion
        return position_ == Position::PRESSED;
    }

    bool isPanic() const {
        return position_ == Position::PANIC;
    }

private:
    Position position_ = Position::RELEASED;
};

/**
 * @brief Velocity monitor
 * Feature: SF007
 */
class VelocityMonitor {
public:
    struct JointVelocities {
        std::array<double, 6> velocities = {0, 0, 0, 0, 0, 0};
    };

    void setLimits(const std::array<double, 6>& maxVelocities) {
        maxVelocities_ = maxVelocities;
    }

    void setT1SpeedLimit(double limit) {
        t1SpeedLimit_ = limit;
    }

    bool checkVelocities(const JointVelocities& current, RobotMode mode) {
        double tcpSpeed = computeTCPSpeed(current);

        // In T1 mode, limit TCP speed
        if (mode == RobotMode::T1) {
            if (tcpSpeed > t1SpeedLimit_) {
                violation_ = true;
                violationMessage_ = "TCP speed exceeds T1 limit";
                return false;
            }
        }

        // Check individual joint velocities
        for (size_t i = 0; i < 6; ++i) {
            if (std::abs(current.velocities[i]) > maxVelocities_[i]) {
                violation_ = true;
                violationMessage_ = "Joint " + std::to_string(i) + " velocity exceeded";
                return false;
            }
        }

        violation_ = false;
        return true;
    }

    bool hasViolation() const { return violation_; }
    std::string getViolationMessage() const { return violationMessage_; }

private:
    std::array<double, 6> maxVelocities_ = {180, 180, 180, 250, 250, 250};  // deg/s
    double t1SpeedLimit_ = 250.0;  // mm/s
    bool violation_ = false;
    std::string violationMessage_;

    double computeTCPSpeed(const JointVelocities& vel) {
        // Simplified: just sum absolute values (actual impl would use Jacobian)
        double sum = 0;
        for (double v : vel.velocities) {
            sum += std::abs(v);
        }
        return sum * 10;  // Rough approximation
    }
};

}  // namespace safety
}  // namespace robot_controller

using namespace robot_controller::safety;

// =============================================================================
// Test Fixtures
// =============================================================================

class SafetyTest : public ::testing::Test {
protected:
    void SetUp() override {
        robot_controller::Logger::init("test_safety.log", "debug");
        monitor_ = std::make_unique<SafetyMonitor>();
        deadman_ = std::make_unique<DeadmanSwitch>();
        velMonitor_ = std::make_unique<VelocityMonitor>();
    }

    std::unique_ptr<SafetyMonitor> monitor_;
    std::unique_ptr<DeadmanSwitch> deadman_;
    std::unique_ptr<VelocityMonitor> velMonitor_;
};

// =============================================================================
// SF001: SafetyMonitor Tests
// =============================================================================

// Feature: SF001 - SafetyMonitor exists and initializes
TEST_F(SafetyTest, SF001_SafetyMonitor_Initializes) {
    EXPECT_FALSE(monitor_->isEStopActive());
    EXPECT_FALSE(monitor_->isSafetyDoorOpen());
    EXPECT_FALSE(monitor_->hasDiscrepancy());
}

// Feature: SF001 - SafetyMonitor processes channels
TEST_F(SafetyTest, SF001_SafetyMonitor_ProcessesChannels) {
    SafetyChannel ch1, ch2;

    ch1.estopPressed = true;
    monitor_->setChannel1(ch1);
    monitor_->setChannel2(ch2);

    EXPECT_TRUE(monitor_->isEStopActive());
}

// =============================================================================
// SF002: Dual-Channel Safety Tests
// =============================================================================

// Feature: SF002 - Dual channels must agree
TEST_F(SafetyTest, SF002_DualChannel_BothChannelsRequired) {
    SafetyChannel ch1, ch2;

    // Only channel 1 says E-Stop - should still be active (fail-safe)
    ch1.estopPressed = true;
    ch2.estopPressed = false;

    monitor_->setChannel1(ch1);
    monitor_->setChannel2(ch2);

    EXPECT_TRUE(monitor_->isEStopActive());
}

// Feature: SF002 - Discrepancy detection within 50ms
TEST_F(SafetyTest, SF002_DualChannel_DetectsDiscrepancyAfter50ms) {
    SafetyChannel ch1, ch2;

    // Create mismatch
    ch1.estopPressed = true;
    ch2.estopPressed = false;

    monitor_->setChannel1(ch1);
    monitor_->setChannel2(ch2);

    // Initially no discrepancy flagged (within timeout)
    EXPECT_FALSE(monitor_->hasDiscrepancy());

    // Wait > 50ms
    std::this_thread::sleep_for(60ms);
    monitor_->update();

    // Now should detect discrepancy
    EXPECT_TRUE(monitor_->hasDiscrepancy());
}

// Feature: SF002 - Discrepancy clears when channels agree
TEST_F(SafetyTest, SF002_DualChannel_ClearsDiscrepancy) {
    SafetyChannel ch1, ch2;

    // Create and wait for discrepancy
    ch1.estopPressed = true;
    ch2.estopPressed = false;
    monitor_->setChannel1(ch1);
    monitor_->setChannel2(ch2);
    std::this_thread::sleep_for(60ms);
    monitor_->update();
    EXPECT_TRUE(monitor_->hasDiscrepancy());

    // Make channels agree
    ch2.estopPressed = true;
    monitor_->setChannel2(ch2);
    monitor_->update();

    EXPECT_FALSE(monitor_->hasDiscrepancy());
}

// =============================================================================
// SF003: E-Stop Handler Tests
// =============================================================================

// Feature: SF003 - E-Stop activates from any channel
TEST_F(SafetyTest, SF003_EStop_ActivatesFromChannel1) {
    SafetyChannel ch1, ch2;
    ch1.estopPressed = true;

    monitor_->setChannel1(ch1);
    monitor_->setChannel2(ch2);

    EXPECT_TRUE(monitor_->isEStopActive());
}

TEST_F(SafetyTest, SF003_EStop_ActivatesFromChannel2) {
    SafetyChannel ch1, ch2;
    ch2.estopPressed = true;

    monitor_->setChannel1(ch1);
    monitor_->setChannel2(ch2);

    EXPECT_TRUE(monitor_->isEStopActive());
}

TEST_F(SafetyTest, SF003_EStop_ActivatesFromBothChannels) {
    SafetyChannel ch1, ch2;
    ch1.estopPressed = true;
    ch2.estopPressed = true;

    monitor_->setChannel1(ch1);
    monitor_->setChannel2(ch2);

    EXPECT_TRUE(monitor_->isEStopActive());
}

// Feature: SF003 - E-Stop must be released on both channels
TEST_F(SafetyTest, SF003_EStop_RequiresBothChannelsToRelease) {
    SafetyChannel ch1, ch2;

    // Both pressed
    ch1.estopPressed = true;
    ch2.estopPressed = true;
    monitor_->setChannel1(ch1);
    monitor_->setChannel2(ch2);
    EXPECT_TRUE(monitor_->isEStopActive());

    // Release only one
    ch1.estopPressed = false;
    monitor_->setChannel1(ch1);
    EXPECT_TRUE(monitor_->isEStopActive());  // Still active

    // Release both
    ch2.estopPressed = false;
    monitor_->setChannel2(ch2);
    EXPECT_FALSE(monitor_->isEStopActive());  // Now released
}

// =============================================================================
// SF004: Soft Limits Tests
// =============================================================================

// Feature: SF004 - Soft limit check within range
TEST_F(SafetyTest, SF004_SoftLimits_WithinRange) {
    double position = 45.0;  // degrees
    double minLimit = -170.0;
    double maxLimit = 170.0;

    EXPECT_TRUE(monitor_->checkSoftLimits(0, position, minLimit, maxLimit));
}

// Feature: SF004 - Soft limit check at boundary
TEST_F(SafetyTest, SF004_SoftLimits_AtBoundary) {
    double minLimit = -170.0;
    double maxLimit = 170.0;

    EXPECT_TRUE(monitor_->checkSoftLimits(0, minLimit, minLimit, maxLimit));
    EXPECT_TRUE(monitor_->checkSoftLimits(0, maxLimit, minLimit, maxLimit));
}

// Feature: SF004 - Soft limit check outside range
TEST_F(SafetyTest, SF004_SoftLimits_OutsideRange) {
    double minLimit = -170.0;
    double maxLimit = 170.0;

    EXPECT_FALSE(monitor_->checkSoftLimits(0, -180.0, minLimit, maxLimit));
    EXPECT_FALSE(monitor_->checkSoftLimits(0, 180.0, minLimit, maxLimit));
}

// Feature: SF004 - Soft limits for all 6 joints
TEST_F(SafetyTest, SF004_SoftLimits_AllJoints) {
    std::array<double, 6> minLimits = {-170, -120, -170, -180, -120, -360};
    std::array<double, 6> maxLimits = {170, 120, 170, 180, 120, 360};

    // Test each joint at center
    for (int i = 0; i < 6; ++i) {
        double center = (minLimits[i] + maxLimits[i]) / 2.0;
        EXPECT_TRUE(monitor_->checkSoftLimits(i, center, minLimits[i], maxLimits[i]))
            << "Joint " << i << " failed at center position";
    }
}

// =============================================================================
// SF005: Hard Limits Tests
// =============================================================================

// Feature: SF005 - Hard limit switch not triggered
TEST_F(SafetyTest, SF005_HardLimits_NotTriggered) {
    EXPECT_TRUE(monitor_->checkHardLimits(0, false));
}

// Feature: SF005 - Hard limit switch triggered
TEST_F(SafetyTest, SF005_HardLimits_Triggered) {
    EXPECT_FALSE(monitor_->checkHardLimits(0, true));
}

// Feature: SF005 - All joints hard limits
TEST_F(SafetyTest, SF005_HardLimits_AllJoints) {
    for (int i = 0; i < 6; ++i) {
        EXPECT_TRUE(monitor_->checkHardLimits(i, false))
            << "Joint " << i << " should pass when limit not triggered";
        EXPECT_FALSE(monitor_->checkHardLimits(i, true))
            << "Joint " << i << " should fail when limit triggered";
    }
}

// =============================================================================
// SF006: Deadman Switch Tests
// =============================================================================

// Feature: SF006 - Deadman released disables motion
TEST_F(SafetyTest, SF006_Deadman_ReleasedDisables) {
    deadman_->setPosition(DeadmanSwitch::Position::RELEASED);

    EXPECT_FALSE(deadman_->isEnabling());
    EXPECT_FALSE(deadman_->isPanic());
}

// Feature: SF006 - Deadman pressed enables motion
TEST_F(SafetyTest, SF006_Deadman_PressedEnables) {
    deadman_->setPosition(DeadmanSwitch::Position::PRESSED);

    EXPECT_TRUE(deadman_->isEnabling());
    EXPECT_FALSE(deadman_->isPanic());
}

// Feature: SF006 - Deadman panic disables motion
TEST_F(SafetyTest, SF006_Deadman_PanicDisables) {
    deadman_->setPosition(DeadmanSwitch::Position::PANIC);

    EXPECT_FALSE(deadman_->isEnabling());
    EXPECT_TRUE(deadman_->isPanic());
}

// Feature: SF006 - Deadman 3-position sequence
TEST_F(SafetyTest, SF006_Deadman_ThreePositionSequence) {
    // Simulate operator pressing and releasing

    // Initially released
    deadman_->setPosition(DeadmanSwitch::Position::RELEASED);
    EXPECT_FALSE(deadman_->isEnabling());

    // Operator presses to middle position
    deadman_->setPosition(DeadmanSwitch::Position::PRESSED);
    EXPECT_TRUE(deadman_->isEnabling());

    // Operator panics (presses fully)
    deadman_->setPosition(DeadmanSwitch::Position::PANIC);
    EXPECT_FALSE(deadman_->isEnabling());
    EXPECT_TRUE(deadman_->isPanic());

    // Operator releases
    deadman_->setPosition(DeadmanSwitch::Position::RELEASED);
    EXPECT_FALSE(deadman_->isEnabling());
}

// =============================================================================
// SF007: Velocity Monitoring Tests
// =============================================================================

// Feature: SF007 - Velocity within T1 limit
TEST_F(SafetyTest, SF007_Velocity_WithinT1Limit) {
    velMonitor_->setT1SpeedLimit(250.0);

    VelocityMonitor::JointVelocities vel;
    vel.velocities = {1, 1, 1, 1, 1, 1};  // Very low velocities (6 * 1 * 10 = 60 < 250)

    EXPECT_TRUE(velMonitor_->checkVelocities(vel, RobotMode::T1));
    EXPECT_FALSE(velMonitor_->hasViolation());
}

// Feature: SF007 - Velocity exceeds T1 limit
TEST_F(SafetyTest, SF007_Velocity_ExceedsT1Limit) {
    velMonitor_->setT1SpeedLimit(250.0);

    VelocityMonitor::JointVelocities vel;
    vel.velocities = {50, 50, 50, 50, 50, 50};  // High velocities -> high TCP speed

    EXPECT_FALSE(velMonitor_->checkVelocities(vel, RobotMode::T1));
    EXPECT_TRUE(velMonitor_->hasViolation());
}

// Feature: SF007 - No limit in AUTO mode
TEST_F(SafetyTest, SF007_Velocity_NoLimitInAutoMode) {
    velMonitor_->setT1SpeedLimit(250.0);

    VelocityMonitor::JointVelocities vel;
    vel.velocities = {100, 100, 100, 100, 100, 100};  // High velocities

    EXPECT_TRUE(velMonitor_->checkVelocities(vel, RobotMode::AUTO));
}

// Feature: SF007 - Individual joint velocity limits
TEST_F(SafetyTest, SF007_Velocity_JointLimitsExceeded) {
    velMonitor_->setLimits({180, 180, 180, 250, 250, 250});

    VelocityMonitor::JointVelocities vel;
    vel.velocities = {200, 0, 0, 0, 0, 0};  // Joint 0 exceeds its 180 deg/s limit

    EXPECT_FALSE(velMonitor_->checkVelocities(vel, RobotMode::AUTO));
    EXPECT_TRUE(velMonitor_->hasViolation());
}

// =============================================================================
// Integration Tests: Safety + StateMachine
// =============================================================================

class SafetyStateMachineTest : public ::testing::Test {
protected:
    void SetUp() override {
        robot_controller::Logger::init("test_safety_sm.log", "debug");
        sm_ = std::make_unique<StateMachine>();
        monitor_ = std::make_unique<SafetyMonitor>();
    }

    std::unique_ptr<StateMachine> sm_;
    std::unique_ptr<SafetyMonitor> monitor_;
};

// Feature: SF001 + SM003 - Safety monitor triggers E-Stop state
TEST_F(SafetyStateMachineTest, SafetyMonitor_TriggersEStopState) {
    sm_->processEvent(StateEvent::INIT_COMPLETE);
    sm_->enable();

    SafetyChannel ch;
    ch.estopPressed = true;
    monitor_->setChannel1(ch);

    if (monitor_->isEStopActive()) {
        sm_->processEvent(StateEvent::ESTOP_PRESSED);
    }

    EXPECT_EQ(sm_->currentState(), RobotState::ESTOP);
}

// Feature: SF006 + SM - Deadman required for jog
TEST_F(SafetyStateMachineTest, Deadman_RequiredForJog) {
    DeadmanSwitch deadman;

    sm_->processEvent(StateEvent::INIT_COMPLETE);
    sm_->enable();
    sm_->processEvent(StateEvent::HOME_REQUEST);
    sm_->processEvent(StateEvent::HOME_COMPLETE);

    // Deadman not pressed - should not allow jog
    deadman.setPosition(DeadmanSwitch::Position::RELEASED);
    bool canJog = sm_->canJog() && deadman.isEnabling();
    EXPECT_FALSE(canJog);

    // Deadman pressed - should allow jog
    deadman.setPosition(DeadmanSwitch::Position::PRESSED);
    canJog = sm_->canJog() && deadman.isEnabling();
    EXPECT_TRUE(canJog);
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
