/**
 * @file test_binary_protocol.cpp
 * @brief Unit tests for V2 binary protocol: CRC, packet framing, struct sizes
 */

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef ERROR
#endif

#include <gtest/gtest.h>
#include "firmware/protocol/PacketTypes.hpp"
#include "firmware/protocol/PVTPoint.hpp"
#include "firmware/protocol/StatusPacket.hpp"
#include "firmware/protocol/BinaryProtocol.hpp"

using namespace robot_controller::firmware::protocol;

// ============================================================================
// Struct Size Verification (wire format must match firmware)
// ============================================================================

TEST(BinaryProtocol_StructSizes, PVTPoint) {
    EXPECT_EQ(sizeof(PVTPoint), 41u);
}

TEST(BinaryProtocol_StructSizes, MoveAbsoluteCmd) {
    EXPECT_EQ(sizeof(MoveAbsoluteCmd), 28u);
}

TEST(BinaryProtocol_StructSizes, JogStartCmd) {
    EXPECT_EQ(sizeof(JogStartCmd), 4u);
}

TEST(BinaryProtocol_StructSizes, HomeStartCmd) {
    EXPECT_EQ(sizeof(HomeStartCmd), 3u);
}

TEST(BinaryProtocol_StructSizes, HomeSetParamsCmd) {
    EXPECT_EQ(sizeof(HomeSetParamsCmd), 13u);
}

TEST(BinaryProtocol_StructSizes, DriveControlCmd) {
    EXPECT_EQ(sizeof(DriveControlCmd), 1u);
}

TEST(BinaryProtocol_StructSizes, AxisParamsCmd) {
    EXPECT_EQ(sizeof(AxisParamsCmd), 17u);
}

TEST(BinaryProtocol_StructSizes, StatusPacket) {
    EXPECT_EQ(sizeof(StatusPacket), 74u);
}

TEST(BinaryProtocol_StructSizes, AlarmPacket) {
    EXPECT_EQ(sizeof(AlarmPacket), 6u);
}

TEST(BinaryProtocol_StructSizes, HomeCompletePacket) {
    EXPECT_EQ(sizeof(HomeCompletePacket), 6u);
}

TEST(BinaryProtocol_StructSizes, AckPacket) {
    EXPECT_EQ(sizeof(AckPacket), 3u);
}

TEST(BinaryProtocol_StructSizes, PacketHeader) {
    EXPECT_EQ(sizeof(PacketHeader), 6u);
}

// ============================================================================
// CRC-CCITT Tests
// ============================================================================

TEST(BinaryProtocol_CRC, EmptyData) {
    uint16_t crc = crc16_ccitt(nullptr, 0);
    EXPECT_EQ(crc, 0xFFFF);  // Initial value with no data
}

TEST(BinaryProtocol_CRC, KnownData) {
    uint8_t data[] = {0x01, 0x02, 0x03};
    uint16_t crc1 = crc16_ccitt(data, 3);

    // Same data should give same CRC
    uint16_t crc2 = crc16_ccitt(data, 3);
    EXPECT_EQ(crc1, crc2);

    // Different data should give different CRC
    data[0] = 0xFF;
    uint16_t crc3 = crc16_ccitt(data, 3);
    EXPECT_NE(crc1, crc3);
}

TEST(BinaryProtocol_CRC, SingleByte) {
    uint8_t data = 0x00;
    uint16_t crc = crc16_ccitt(&data, 1);
    EXPECT_NE(crc, 0xFFFF);  // Should differ from empty
}

// ============================================================================
// Packet Serialization/Parsing Round-Trip
// ============================================================================

TEST(BinaryProtocol_Packet, SerializeAndParse) {
    uint8_t payload[] = {0xAA, 0xBB, 0xCC};
    auto packet = serializePacket(42, 0x10, payload, 3);

    EXPECT_GT(packet.size(), 0u);

    PacketHeader header;
    const uint8_t* payloadData = nullptr;
    uint16_t payloadLen = 0;

    bool ok = parsePacket(packet.data(), packet.size(), header, &payloadData, &payloadLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(header.seq, 42);
    EXPECT_EQ(header.type, 0x10);
    EXPECT_EQ(payloadLen, 3u);
    EXPECT_EQ(payloadData[0], 0xAA);
    EXPECT_EQ(payloadData[1], 0xBB);
    EXPECT_EQ(payloadData[2], 0xCC);
}

TEST(BinaryProtocol_Packet, EmptyPayload) {
    auto packet = serializePacket(1, 0x20, nullptr, 0);
    EXPECT_GT(packet.size(), 0u);

    PacketHeader header;
    const uint8_t* payloadData = nullptr;
    uint16_t payloadLen = 0;

    bool ok = parsePacket(packet.data(), packet.size(), header, &payloadData, &payloadLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(header.seq, 1);
    EXPECT_EQ(header.type, 0x20);
    EXPECT_EQ(payloadLen, 0u);
}

TEST(BinaryProtocol_Packet, CRCVerification) {
    uint8_t payload[] = {0x01};
    auto packet = serializePacket(0, 0x10, payload, 1);

    EXPECT_TRUE(verifyPacketCRC(packet.data(), packet.size()));

    // Corrupt a byte in the middle
    if (packet.size() > 4) {
        packet[4] ^= 0xFF;
        EXPECT_FALSE(verifyPacketCRC(packet.data(), packet.size()));
    }
}

TEST(BinaryProtocol_Packet, TooShortPacket) {
    uint8_t data[] = {0xAA, 0x55};  // Only sync word, too short
    PacketHeader header;
    const uint8_t* payloadData = nullptr;
    uint16_t payloadLen = 0;

    bool ok = parsePacket(data, 2, header, &payloadData, &payloadLen);
    EXPECT_FALSE(ok);
}

TEST(BinaryProtocol_Packet, BadSyncWord) {
    uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    PacketHeader header;
    const uint8_t* payloadData = nullptr;
    uint16_t payloadLen = 0;

    bool ok = parsePacket(data, 8, header, &payloadData, &payloadLen);
    EXPECT_FALSE(ok);
}

// ============================================================================
// Typed Command Serialization
// ============================================================================

TEST(BinaryProtocol_TypedCmd, MoveAbsolute) {
    MoveAbsoluteCmd cmd{};
    cmd.position[0] = 100000;
    cmd.position[1] = -50000;
    cmd.max_speed = 1000;
    cmd.accel = 500;

    auto packet = serializeCommand(7, CommandType::CMD_MOVE_ABSOLUTE, cmd);
    EXPECT_GT(packet.size(), 0u);

    PacketHeader header;
    const uint8_t* payloadData = nullptr;
    uint16_t payloadLen = 0;

    bool ok = parsePacket(packet.data(), packet.size(), header, &payloadData, &payloadLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(payloadLen, sizeof(MoveAbsoluteCmd));

    MoveAbsoluteCmd parsed;
    EXPECT_TRUE(extractPayload(payloadData, payloadLen, parsed));
    EXPECT_EQ(parsed.position[0], 100000);
    EXPECT_EQ(parsed.position[1], -50000);
    EXPECT_EQ(parsed.max_speed, 1000);
    EXPECT_EQ(parsed.accel, 500);
}

TEST(BinaryProtocol_TypedCmd, JogStart) {
    JogStartCmd cmd{2, -1, 5000};

    auto packet = serializeCommand(0, CommandType::CMD_JOG_START, cmd);

    PacketHeader header;
    const uint8_t* payloadData = nullptr;
    uint16_t payloadLen = 0;

    bool ok = parsePacket(packet.data(), packet.size(), header, &payloadData, &payloadLen);
    EXPECT_TRUE(ok);

    JogStartCmd parsed;
    EXPECT_TRUE(extractPayload(payloadData, payloadLen, parsed));
    EXPECT_EQ(parsed.axis, 2);
    EXPECT_EQ(parsed.direction, -1);
    EXPECT_EQ(parsed.speed, 5000);
}

TEST(BinaryProtocol_TypedCmd, PVTPoint) {
    PVTPoint pt{};
    pt.duration_us = 100;
    pt.position[0] = 1000;
    pt.position[3] = -500;
    pt.velocity[0] = 100;

    auto packet = serializeCommand(0, CommandType::CMD_PVT_POINT, pt);

    PacketHeader header;
    const uint8_t* payloadData = nullptr;
    uint16_t payloadLen = 0;

    bool ok = parsePacket(packet.data(), packet.size(), header, &payloadData, &payloadLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(payloadLen, sizeof(PVTPoint));

    PVTPoint parsed;
    EXPECT_TRUE(extractPayload(payloadData, payloadLen, parsed));
    EXPECT_EQ(parsed.duration_us, 100u);
    EXPECT_EQ(parsed.position[0], 1000);
    EXPECT_EQ(parsed.position[3], -500);
    EXPECT_EQ(parsed.velocity[0], 100);
}

// ============================================================================
// StatusPacket Helper Methods
// ============================================================================

TEST(BinaryProtocol_StatusPacket, Helpers) {
    StatusPacket sp{};
    sp.state = static_cast<uint8_t>(SystemState::STATE_IDLE);
    sp.drive_ready = 0x3F;   // All 6 drives ready
    sp.drive_alarm = 0x00;   // No alarms
    sp.home_status = 0x15;   // Axes 0, 2, 4 homed (0b00010101)

    EXPECT_TRUE(sp.isDriveReady(0));
    EXPECT_TRUE(sp.isDriveReady(5));
    EXPECT_FALSE(sp.isDriveAlarm(0));

    EXPECT_TRUE(sp.isAxisHomed(0));
    EXPECT_FALSE(sp.isAxisHomed(1));
    EXPECT_TRUE(sp.isAxisHomed(2));
    EXPECT_FALSE(sp.isAxisHomed(3));
    EXPECT_TRUE(sp.isAxisHomed(4));
    EXPECT_FALSE(sp.isAxisHomed(5));

    EXPECT_FALSE(sp.isAllHomed());

    sp.home_status = 0x3F;  // All 6 homed
    EXPECT_TRUE(sp.isAllHomed());
}

TEST(BinaryProtocol_StatusPacket, Clear) {
    StatusPacket sp{};
    sp.state = 5;
    sp.drive_ready = 0xFF;
    sp.actual_pos[0] = 12345;

    sp.clear();

    EXPECT_EQ(sp.state, 0);
    EXPECT_EQ(sp.drive_ready, 0);
    EXPECT_EQ(sp.actual_pos[0], 0);
}

// ============================================================================
// Enum String Conversion
// ============================================================================

TEST(BinaryProtocol_Enums, SystemStateToString) {
    EXPECT_EQ(systemStateToString(SystemState::STATE_DISABLED), "Disabled");
    EXPECT_EQ(systemStateToString(SystemState::STATE_IDLE), "Idle");
    EXPECT_EQ(systemStateToString(SystemState::STATE_MOVING), "Moving");
    EXPECT_EQ(systemStateToString(SystemState::STATE_JOGGING), "Jogging");
    EXPECT_EQ(systemStateToString(SystemState::STATE_HOMING), "Homing");
    EXPECT_EQ(systemStateToString(SystemState::STATE_ALARM), "Alarm");
    EXPECT_EQ(systemStateToString(SystemState::STATE_ESTOP), "EStop");
}

TEST(BinaryProtocol_Enums, CommandTypeToString) {
    EXPECT_EQ(commandTypeToString(CommandType::CMD_PVT_POINT), "CMD_PVT_POINT");
    EXPECT_EQ(commandTypeToString(CommandType::CMD_MOVE_ABSOLUTE), "CMD_MOVE_ABSOLUTE");
    EXPECT_EQ(commandTypeToString(CommandType::CMD_JOG_START), "CMD_JOG_START");
    EXPECT_EQ(commandTypeToString(CommandType::CMD_HEARTBEAT), "CMD_HEARTBEAT");
}

// ============================================================================
// FindSyncWord
// ============================================================================

TEST(BinaryProtocol_Sync, FindSyncWord) {
    // SYNC_WORD = 0xAA55, on LE stored as {0x55, 0xAA}
    uint8_t data[] = {0x00, 0x00, 0x55, 0xAA, 0x01, 0x10, 0x00, 0x00};

    size_t pos = findSyncWord(data, 8);
    EXPECT_EQ(pos, 2u);  // Found at offset 2
}

TEST(BinaryProtocol_Sync, FindSyncWordAtStart) {
    uint8_t data[] = {0x55, 0xAA, 0x01, 0x10};

    size_t pos = findSyncWord(data, 4);
    EXPECT_EQ(pos, 0u);
}

TEST(BinaryProtocol_Sync, FindSyncWordNotFound) {
    uint8_t data[] = {0x00, 0x00, 0x00, 0x00};

    size_t pos = findSyncWord(data, 4);
    EXPECT_EQ(pos, 4u);  // Returns dataLen when not found
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
