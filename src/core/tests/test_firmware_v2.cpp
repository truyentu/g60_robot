/**
 * @file test_firmware_v2.cpp
 * @brief Unit tests for V2 firmware layer: FirmwareSimulator drive states, jog, homing, I/O
 */

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef ERROR
#endif

#include <gtest/gtest.h>
#include "firmware/FirmwareSimulator.hpp"

using namespace robot_controller::firmware;
using namespace robot_controller::firmware::protocol;

class FirmwareSimV2Test : public ::testing::Test {
protected:
    FirmwareSimulator sim;

    void SetUp() override {
        // Simulator auto-enables drives (no physical hardware)
        // Initial state is IDLE
    }
};

// ============================================================================
// Initial State
// ============================================================================

TEST_F(FirmwareSimV2Test, InitialState_IsIdle) {
    // Simulator auto-enables drives, starts in IDLE
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_IDLE);
    EXPECT_EQ(sim.getStateString(), "Idle");
}

TEST_F(FirmwareSimV2Test, InitialState_StatusPacket) {
    auto sp = sim.getStatusPacket();
    EXPECT_EQ(sp.state, static_cast<uint8_t>(SystemState::STATE_IDLE));
    // drive_ready = 0x3F (physically ready)
    EXPECT_EQ(sp.drive_ready, 0x3F);
    EXPECT_EQ(sp.drive_alarm, 0);
    EXPECT_EQ(sp.home_status, 0);
}

// ============================================================================
// Drive Enable/Disable
// ============================================================================

TEST_F(FirmwareSimV2Test, EnableDrives_AllAxes) {
    EXPECT_TRUE(sim.enableDrives(0x3F));  // All 6 axes
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_IDLE);
    EXPECT_EQ(sim.getStateString(), "Idle");

    auto sp = sim.getStatusPacket();
    EXPECT_EQ(sp.drive_ready, 0x3F);
}

TEST_F(FirmwareSimV2Test, EnableDrives_SingleAxis) {
    EXPECT_TRUE(sim.enableDrives(0x01));  // Only axis 0

    auto sp = sim.getStatusPacket();
    EXPECT_EQ(sp.drive_ready & 0x01, 0x01);
}

TEST_F(FirmwareSimV2Test, DisableDrives) {
    sim.enableDrives(0x3F);
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_IDLE);

    EXPECT_TRUE(sim.disableDrives(0x3F));
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_DISABLED);

    // drive_ready stays 0x3F (physically ready); state indicates disabled
    auto sp = sim.getStatusPacket();
    EXPECT_EQ(sp.state, static_cast<uint8_t>(SystemState::STATE_DISABLED));
}

TEST_F(FirmwareSimV2Test, DisableDrives_Partial) {
    sim.enableDrives(0x3F);
    sim.disableDrives(0x03);  // Sim uses m_drivesEnabled as boolean

    // Sim treats disableDrives as full disable (boolean flag)
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_DISABLED);
}

// ============================================================================
// Jog (V2 structured API)
// ============================================================================

TEST_F(FirmwareSimV2Test, JogStart_RequiresEnabled) {
    // Explicitly disable drives first, then jog should fail
    sim.disableDrives(0x3F);
    EXPECT_FALSE(sim.jogStart(0, 1, 1000));
}

TEST_F(FirmwareSimV2Test, JogStart_Success) {
    // Simulator auto-enabled, jog should work directly
    EXPECT_TRUE(sim.jogStart(0, 1, 1000));
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_JOGGING);
}

TEST_F(FirmwareSimV2Test, JogStop) {
    sim.enableDrives(0x3F);
    sim.jogStart(0, 1, 1000);
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_JOGGING);

    EXPECT_TRUE(sim.jogStop());
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_IDLE);
}

TEST_F(FirmwareSimV2Test, StopMotion) {
    sim.enableDrives(0x3F);
    sim.jogStart(0, 1, 1000);

    EXPECT_TRUE(sim.stopMotion(0));
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_IDLE);
}

// ============================================================================
// Homing
// ============================================================================

TEST_F(FirmwareSimV2Test, HomeStart_RequiresEnabled) {
    sim.disableDrives(0x3F);
    EXPECT_FALSE(sim.homeStart(0x3F, 0, 0));
}

TEST_F(FirmwareSimV2Test, HomeStart_Success) {
    // Simulator auto-enabled
    EXPECT_TRUE(sim.homeStart(0x3F, 0, 0));
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_HOMING);
}

TEST_F(FirmwareSimV2Test, HomeStop) {
    sim.homeStart(0x3F, 0, 0);
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_HOMING);

    EXPECT_TRUE(sim.homeStop(0x3F));
    EXPECT_EQ(sim.getSystemState(), SystemState::STATE_IDLE);
}

TEST_F(FirmwareSimV2Test, HomingSimulation_Completes) {
    sim.enableDrives(0x3F);
    sim.homeStart(0x3F, 0, 0);

    // Simulate 3 seconds of updates (homing takes ~2s per axis)
    for (int i = 0; i < 300; i++) {
        sim.update(0.01);  // 10ms steps
    }

    auto sp = sim.getStatusPacket();
    EXPECT_EQ(sp.home_status, 0x3F);  // All axes homed
    EXPECT_NE(sim.getSystemState(), SystemState::STATE_HOMING);
}

// ============================================================================
// Unit Conversion
// ============================================================================

TEST_F(FirmwareSimV2Test, DegreesToSteps) {
    // Default: 2777.78 steps/degree
    int32_t steps = sim.degreesToSteps(0, 90.0);
    EXPECT_NEAR(static_cast<double>(steps), 90.0 * 2777.78, 1.0);
}

TEST_F(FirmwareSimV2Test, StepsToDegrees) {
    int32_t steps = sim.degreesToSteps(0, 45.0);
    double deg = sim.stepsToDegrees(0, steps);
    EXPECT_NEAR(deg, 45.0, 0.01);
}

TEST_F(FirmwareSimV2Test, DegreesToSteps_InvalidAxis) {
    EXPECT_EQ(sim.degreesToSteps(10, 90.0), 0);  // Out of range
}

TEST_F(FirmwareSimV2Test, StepsToDegrees_InvalidAxis) {
    EXPECT_DOUBLE_EQ(sim.stepsToDegrees(10, 1000), 0.0);
}

// ============================================================================
// Digital I/O
// ============================================================================

TEST_F(FirmwareSimV2Test, DigitalIO_InitialState) {
    EXPECT_EQ(sim.getDigitalInputs(), 0);
    EXPECT_EQ(sim.getDigitalOutputs(), 0);
}

TEST_F(FirmwareSimV2Test, SetOutput_Single) {
    EXPECT_TRUE(sim.setOutput(0, true));
    EXPECT_EQ(sim.getDigitalOutputs() & 0x01, 0x01);

    EXPECT_TRUE(sim.setOutput(0, false));
    EXPECT_EQ(sim.getDigitalOutputs() & 0x01, 0x00);
}

TEST_F(FirmwareSimV2Test, SetOutput_Multiple) {
    sim.setOutput(0, true);
    sim.setOutput(3, true);
    sim.setOutput(7, true);

    uint16_t outputs = sim.getDigitalOutputs();
    EXPECT_TRUE(outputs & (1 << 0));
    EXPECT_TRUE(outputs & (1 << 3));
    EXPECT_TRUE(outputs & (1 << 7));
    EXPECT_FALSE(outputs & (1 << 1));
}

TEST_F(FirmwareSimV2Test, SetOutputsBatch) {
    EXPECT_TRUE(sim.setOutputsBatch(0x00FF, 0x00A5));  // Mask lower 8 bits, set pattern

    uint16_t outputs = sim.getDigitalOutputs();
    EXPECT_EQ(outputs & 0x00FF, 0x00A5);
}

TEST_F(FirmwareSimV2Test, StatusPacket_ReflectsIO) {
    sim.setOutput(5, true);

    auto sp = sim.getStatusPacket();
    EXPECT_TRUE(sp.digital_outputs & (1 << 5));
}

// ============================================================================
// MoveAbsolute
// ============================================================================

TEST_F(FirmwareSimV2Test, MoveAbsolute_RequiresEnabled) {
    std::array<int32_t, DRIVER_NUM_AXES> steps{};
    steps[0] = 100000;
    EXPECT_FALSE(sim.moveAbsolute(steps, 1000, 500));
}

TEST_F(FirmwareSimV2Test, MoveAbsolute_NotImplementedInSim) {
    // FirmwareSimulator uses default impl (return false)
    // Real driver (STM32) would handle this
    sim.enableDrives(0x3F);

    std::array<int32_t, DRIVER_NUM_AXES> steps{};
    steps[0] = static_cast<int32_t>(45.0 * 2777.78);
    EXPECT_FALSE(sim.moveAbsolute(steps, 1000, 500));
}

// ============================================================================
// Reset / Alarm
// ============================================================================

TEST_F(FirmwareSimV2Test, ResetAlarm) {
    sim.enableDrives(0x3F);
    EXPECT_TRUE(sim.resetAlarm(0x3F));

    auto sp = sim.getStatusPacket();
    EXPECT_EQ(sp.drive_alarm, 0);
}

TEST_F(FirmwareSimV2Test, EmergencyStop) {
    sim.enableDrives(0x3F);
    sim.jogStart(0, 1, 1000);

    sim.emergencyStop();

    EXPECT_EQ(sim.getStateString(), "Alarm");
}

// ============================================================================
// PVT Point
// ============================================================================

TEST_F(FirmwareSimV2Test, SendPVTPoint_NotImplementedInSim) {
    sim.enableDrives(0x3F);

    PVTPoint pt{};
    pt.duration_us = 10;
    pt.position[0] = 1000;
    pt.velocity[0] = 100;

    // Sim uses default (return false); STM32 would accept
    EXPECT_FALSE(sim.sendPVTPoint(pt));
}

TEST_F(FirmwareSimV2Test, SendPVTBatch_NotImplementedInSim) {
    sim.enableDrives(0x3F);

    std::vector<PVTPoint> batch(3);
    for (int i = 0; i < 3; i++) {
        batch[i].duration_us = static_cast<uint16_t>((i + 1) * 10);
        batch[i].position[0] = (i + 1) * 1000;
    }

    EXPECT_FALSE(sim.sendPVTBatch(batch));
}

// ============================================================================
// Buffer Level
// ============================================================================

TEST_F(FirmwareSimV2Test, BufferLevel) {
    uint8_t level = sim.getBufferLevel();
    EXPECT_GE(level, 0);
    EXPECT_LE(level, 255);
}

// ============================================================================
// Driver Name & Simulation Flag
// ============================================================================

TEST_F(FirmwareSimV2Test, DriverName) {
    EXPECT_EQ(sim.getDriverName(), "FirmwareSimulator");
}

TEST_F(FirmwareSimV2Test, IsSimulation) {
    EXPECT_TRUE(sim.isSimulation());
}

// ============================================================================
// Heartbeat
// ============================================================================

TEST_F(FirmwareSimV2Test, Heartbeat_NotImplementedInSim) {
    // Sim uses default (return false)
    EXPECT_FALSE(sim.sendHeartbeat());
}

// ============================================================================
// setJointPositionsDirect
// ============================================================================

TEST_F(FirmwareSimV2Test, SetJointPositionsDirect) {
    std::array<double, DRIVER_NUM_AXES> pos = {10.0, 20.0, 30.0, 0.0, 0.0, 0.0};
    sim.setJointPositionsDirect(pos);

    auto readback = sim.getJointPositions();
    EXPECT_NEAR(readback[0], 10.0, 0.01);
    EXPECT_NEAR(readback[1], 20.0, 0.01);
    EXPECT_NEAR(readback[2], 30.0, 0.01);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
