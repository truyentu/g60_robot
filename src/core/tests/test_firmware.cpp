/**
 * @file test_firmware.cpp
 * @brief Unit tests for firmware communication
 */

// Windows defines ERROR macro which conflicts with our enum
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#undef ERROR
#endif

#include <gtest/gtest.h>
#include "firmware/FirmwareProtocol.hpp"
#include "firmware/SerialPort.hpp"
#include "firmware/FirmwareInterface.hpp"

using namespace robot_controller::firmware;

// ============================================================================
// Protocol Tests
// ============================================================================

TEST(ProtocolTest, Checksum) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t checksum = calculateChecksum(data, 4);

    EXPECT_EQ(checksum, 0x01 ^ 0x02 ^ 0x03 ^ 0x04);
}

TEST(ProtocolTest, ChecksumEmpty) {
    uint8_t checksum = calculateChecksum(nullptr, 0);
    EXPECT_EQ(checksum, 0);
}

TEST(ProtocolTest, FormatMoveGCode) {
    std::array<double, NUM_AXES> pos = {10.0, 20.0, 30.0, 45.0, 60.0, 90.0};

    std::string cmd = formatMoveGCode(pos, 1000.0, false);

    EXPECT_TRUE(cmd.find("G1") != std::string::npos);
    EXPECT_TRUE(cmd.find("X10.0000") != std::string::npos);
    EXPECT_TRUE(cmd.find("Y20.0000") != std::string::npos);
    EXPECT_TRUE(cmd.find("Z30.0000") != std::string::npos);
    EXPECT_TRUE(cmd.find("A45.0000") != std::string::npos);
    EXPECT_TRUE(cmd.find("B60.0000") != std::string::npos);
    EXPECT_TRUE(cmd.find("C90.0000") != std::string::npos);
    EXPECT_TRUE(cmd.find("F1000.0") != std::string::npos);
}

TEST(ProtocolTest, FormatRapidGCode) {
    std::array<double, NUM_AXES> pos = {0, 0, 0, 0, 0, 0};

    std::string cmd = formatMoveGCode(pos, 0, true);

    EXPECT_TRUE(cmd.find("G0") != std::string::npos);
    EXPECT_TRUE(cmd.find("F") == std::string::npos);  // No feed rate for rapid
}

TEST(ProtocolTest, ParseStatusIdle) {
    std::string response = "<Idle|MPos:0.000,0.000,0.000,0.000,0.000,0.000|WPos:0.000,0.000,0.000>";

    MachineStatus status;
    bool success = parseStatusResponse(response, status);

    EXPECT_TRUE(success);
    EXPECT_EQ(status.state, MachineState::IDLE);
}

TEST(ProtocolTest, ParseStatusRun) {
    std::string response = "<Run|MPos:100.500,50.250,30.000,45.000,60.000,90.000>";

    MachineStatus status;
    bool success = parseStatusResponse(response, status);

    EXPECT_TRUE(success);
    EXPECT_EQ(status.state, MachineState::RUN);
    EXPECT_EQ(status.position.axis[0], 100500);  // 100.5mm in um
    EXPECT_EQ(status.position.axis[1], 50250);
}

TEST(ProtocolTest, ParseStatusAlarm) {
    std::string response = "<Alarm:1|MPos:0.000,0.000,0.000,0.000,0.000,0.000>";

    MachineStatus status;
    bool success = parseStatusResponse(response, status);

    EXPECT_TRUE(success);
    EXPECT_EQ(status.state, MachineState::ALARM);
}

TEST(ProtocolTest, ParseStatusHoming) {
    std::string response = "<Home|MPos:0.000,0.000,0.000,0.000,0.000,0.000>";

    MachineStatus status;
    bool success = parseStatusResponse(response, status);

    EXPECT_TRUE(success);
    EXPECT_EQ(status.state, MachineState::HOMING);
}

TEST(ProtocolTest, ParseStatusHold) {
    std::string response = "<Hold|MPos:50.000,50.000,50.000,0.000,0.000,0.000>";

    MachineStatus status;
    bool success = parseStatusResponse(response, status);

    EXPECT_TRUE(success);
    EXPECT_EQ(status.state, MachineState::HOLD);
}

TEST(ProtocolTest, ParseStatusEmpty) {
    MachineStatus status;
    bool success = parseStatusResponse("", status);
    EXPECT_FALSE(success);
}

TEST(ProtocolTest, ParseStatusInvalid) {
    MachineStatus status;
    bool success = parseStatusResponse("invalid", status);
    EXPECT_FALSE(success);
}

// ============================================================================
// Data Structure Tests
// ============================================================================

TEST(DataStructTest, PositionData) {
    PositionData pos;

    for (int i = 0; i < NUM_AXES; ++i) {
        EXPECT_EQ(pos.axis[i], 0);
    }

    pos.axis[0] = 1000;
    EXPECT_EQ(pos.axis[0], 1000);
}

TEST(DataStructTest, VelocityData) {
    VelocityData vel;

    for (int i = 0; i < NUM_AXES; ++i) {
        EXPECT_EQ(vel.axis[i], 0);
    }
}

TEST(DataStructTest, LimitStatus) {
    LimitStatus limits;
    limits.minLimits = 0b00000101;  // Axis 0 and 2
    limits.maxLimits = 0b00001010;  // Axis 1 and 3
    limits.homeSwitches = 0b00111111;  // All axes

    EXPECT_TRUE(limits.isMinLimit(0));
    EXPECT_FALSE(limits.isMinLimit(1));
    EXPECT_TRUE(limits.isMinLimit(2));

    EXPECT_FALSE(limits.isMaxLimit(0));
    EXPECT_TRUE(limits.isMaxLimit(1));

    for (int i = 0; i < NUM_AXES; ++i) {
        EXPECT_TRUE(limits.isHome(i));
    }
}

TEST(DataStructTest, MachineStatus) {
    MachineStatus status;

    status.homingStatus = 0b00111111;  // All homed
    EXPECT_TRUE(status.isAllHomed());

    status.homingStatus = 0b00111110;  // Axis 0 not homed
    EXPECT_FALSE(status.isAllHomed());
    EXPECT_FALSE(status.isHomed(0));
    EXPECT_TRUE(status.isHomed(1));
}

TEST(DataStructTest, MachineStatusDefaults) {
    MachineStatus status;

    EXPECT_EQ(status.state, MachineState::IDLE);
    EXPECT_EQ(status.alarm, AlarmCode::NONE);
    EXPECT_EQ(status.feedOverride, 100);
    EXPECT_EQ(status.rapidOverride, 100);
}

TEST(DataStructTest, IOStatus) {
    IOStatus io;

    EXPECT_EQ(io.digitalInputs, 0);
    EXPECT_EQ(io.digitalOutputs, 0);
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(io.analogInputs[i], 0);
    }
}

// ============================================================================
// Serial Port Tests (Mock/Offline)
// ============================================================================

TEST(SerialPortTest, GetAvailablePorts) {
    auto ports = SerialPortManager::getAvailablePorts();

    // Just check it doesn't crash
    EXPECT_GE(ports.size(), 0u);
}

TEST(SerialPortTest, OpenInvalidPort) {
    SerialPortManager serial;

    SerialConfig config;
    config.portName = "INVALID_PORT_12345";
    config.baudRate = 115200;

    bool opened = serial.open(config);
    EXPECT_FALSE(opened);
}

TEST(SerialPortTest, StatisticsReset) {
    SerialPortManager serial;

    serial.resetStatistics();
    EXPECT_EQ(serial.getBytesReceived(), 0u);
    EXPECT_EQ(serial.getBytesSent(), 0u);
}

TEST(SerialPortTest, IsOpenWhenNotOpened) {
    SerialPortManager serial;
    EXPECT_FALSE(serial.isOpen());
}

// ============================================================================
// Command Code Tests
// ============================================================================

TEST(CommandCodeTest, Values) {
    EXPECT_EQ(static_cast<uint8_t>(CommandCode::PING), 0x01);
    EXPECT_EQ(static_cast<uint8_t>(CommandCode::RESET), 0x02);
    EXPECT_EQ(static_cast<uint8_t>(CommandCode::MOVE_JOINT), 0x10);
    EXPECT_EQ(static_cast<uint8_t>(CommandCode::HOME_ALL), 0x20);
    EXPECT_EQ(static_cast<uint8_t>(CommandCode::GET_POSITION), 0x30);
    EXPECT_EQ(static_cast<uint8_t>(CommandCode::GET_LIMITS), 0x50);
    EXPECT_EQ(static_cast<uint8_t>(CommandCode::GCODE), 0xF0);
}

TEST(ResponseCodeTest, Values) {
    EXPECT_EQ(static_cast<uint8_t>(ResponseCode::OK), 0x00);
    EXPECT_EQ(static_cast<uint8_t>(ResponseCode::ERROR), 0x01);
    EXPECT_EQ(static_cast<uint8_t>(ResponseCode::BUSY), 0x02);
    EXPECT_EQ(static_cast<uint8_t>(ResponseCode::MOTION_COMPLETE), 0x20);
}

TEST(MachineStateTest, Values) {
    EXPECT_EQ(static_cast<uint8_t>(MachineState::IDLE), 0);
    EXPECT_EQ(static_cast<uint8_t>(MachineState::RUN), 1);
    EXPECT_EQ(static_cast<uint8_t>(MachineState::HOLD), 2);
    EXPECT_EQ(static_cast<uint8_t>(MachineState::ALARM), 5);
}

TEST(AlarmCodeTest, Values) {
    EXPECT_EQ(static_cast<uint8_t>(AlarmCode::NONE), 0);
    EXPECT_EQ(static_cast<uint8_t>(AlarmCode::HARD_LIMIT), 1);
    EXPECT_EQ(static_cast<uint8_t>(AlarmCode::ESTOP), 12);
}

// ============================================================================
// G-code Constants
// ============================================================================

TEST(GCodeTest, RealTimeCommands) {
    EXPECT_EQ(GCode::SOFT_RESET, 0x18);
    EXPECT_EQ(GCode::STATUS_QUERY, '?');
    EXPECT_EQ(GCode::CYCLE_START, '~');
    EXPECT_EQ(GCode::FEED_HOLD, '!');
}

TEST(GCodeTest, FeedOverrides) {
    EXPECT_EQ(GCode::FEED_100, '\x90');
    EXPECT_EQ(GCode::FEED_INCREASE_10, '\x91');
    EXPECT_EQ(GCode::FEED_DECREASE_10, '\x92');
}

TEST(GCodeTest, RapidOverrides) {
    EXPECT_EQ(GCode::RAPID_100, '\x95');
    EXPECT_EQ(GCode::RAPID_50, '\x96');
    EXPECT_EQ(GCode::RAPID_25, '\x97');
}

// ============================================================================
// Utility Function Tests
// ============================================================================

TEST(UtilityTest, RadToDeg) {
    EXPECT_NEAR(radToDeg(0), 0, 0.0001);
    EXPECT_NEAR(radToDeg(3.14159265358979323846), 180.0, 0.0001);
    EXPECT_NEAR(radToDeg(3.14159265358979323846 / 2), 90.0, 0.0001);
}

TEST(UtilityTest, DegToRad) {
    EXPECT_NEAR(degToRad(0), 0, 0.0001);
    EXPECT_NEAR(degToRad(180.0), 3.14159265358979323846, 0.0001);
    EXPECT_NEAR(degToRad(90.0), 3.14159265358979323846 / 2, 0.0001);
}

// ============================================================================
// Firmware Interface Tests (without hardware)
// ============================================================================

TEST(FirmwareInterfaceTest, NotConnectedByDefault) {
    FirmwareInterface firmware;
    EXPECT_FALSE(firmware.isConnected());
}

TEST(FirmwareInterfaceTest, GetStatusWhenNotConnected) {
    FirmwareInterface firmware;
    auto status = firmware.getCachedStatus();

    EXPECT_EQ(status.state, MachineState::IDLE);
    EXPECT_EQ(status.alarm, AlarmCode::NONE);
}

TEST(FirmwareInterfaceTest, IsHomedWhenNotConnected) {
    FirmwareInterface firmware;

    for (int i = 0; i < NUM_AXES; ++i) {
        EXPECT_FALSE(firmware.isHomed(i));
    }
    EXPECT_FALSE(firmware.isAllHomed());
}

TEST(FirmwareInterfaceTest, MoveWhenNotConnected) {
    FirmwareInterface firmware;
    std::array<double, NUM_AXES> positions = {0, 0, 0, 0, 0, 0};

    EXPECT_FALSE(firmware.moveJoints(positions, 10.0, false));
    EXPECT_FALSE(firmware.moveRapid(positions));
}

TEST(FirmwareInterfaceTest, JogWhenNotConnected) {
    FirmwareInterface firmware;

    EXPECT_FALSE(firmware.jogStart(0, 1, 10.0));
    EXPECT_FALSE(firmware.jogStop());
}

TEST(FirmwareInterfaceTest, HomingWhenNotConnected) {
    FirmwareInterface firmware;

    EXPECT_FALSE(firmware.homeAll(false));
    EXPECT_FALSE(firmware.homeAxis(0, false));
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
