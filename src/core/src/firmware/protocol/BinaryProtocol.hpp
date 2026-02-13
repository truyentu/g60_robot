/**
 * @file BinaryProtocol.hpp
 * @brief V2 Binary Protocol — Packet framing, CRC-CCITT, serialize/parse
 *
 * Packet format:
 *   [SYNC_WORD:2][SEQ:1][TYPE:1][LENGTH:2][PAYLOAD:0..MAX_PAYLOAD][CRC16:2]
 *
 * - SYNC_WORD: 0xAA55 (little-endian)
 * - SEQ: sequence counter (0-255, wrapping)
 * - TYPE: CommandType or ResponseType
 * - LENGTH: payload length in bytes (0..MAX_PAYLOAD)
 * - PAYLOAD: command/response-specific data
 * - CRC16: CRC-CCITT over [SEQ..PAYLOAD] (excludes SYNC_WORD and CRC itself)
 */

#pragma once

#include "PacketTypes.hpp"
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>

namespace robot_controller {
namespace firmware {
namespace protocol {

// ============================================================================
// Constants
// ============================================================================

constexpr uint16_t SYNC_WORD     = 0xAA55;
constexpr size_t   HEADER_SIZE   = 6;    // SYNC(2) + SEQ(1) + TYPE(1) + LEN(2)
constexpr size_t   CRC_SIZE      = 2;
constexpr size_t   MIN_PACKET    = HEADER_SIZE + CRC_SIZE;  // 8 bytes minimum
constexpr size_t   MAX_PAYLOAD   = 512;
constexpr size_t   MAX_PACKET    = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE;

// ============================================================================
// Packet Header
// ============================================================================

#pragma pack(push, 1)
struct PacketHeader {
    uint16_t sync;     // Must be SYNC_WORD (0xAA55)
    uint8_t  seq;      // Sequence number
    uint8_t  type;     // CommandType or ResponseType
    uint16_t length;   // Payload length
};
static_assert(sizeof(PacketHeader) == HEADER_SIZE, "PacketHeader must be 6 bytes");
#pragma pack(pop)

// ============================================================================
// CRC-CCITT (0xFFFF initial, polynomial 0x1021)
// ============================================================================

/**
 * Calculate CRC-CCITT over a data buffer.
 * Initial value 0xFFFF, polynomial 0x1021.
 */
inline uint16_t crc16_ccitt(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ============================================================================
// Serialize
// ============================================================================

/**
 * Serialize a command/response into a byte buffer.
 *
 * @param buffer     Output buffer (must be >= HEADER_SIZE + payloadLen + CRC_SIZE)
 * @param bufferSize Size of output buffer
 * @param seq        Sequence number
 * @param type       Command or Response type
 * @param payload    Payload data (may be nullptr if payloadLen == 0)
 * @param payloadLen Payload length in bytes
 * @return Total packet size written, or 0 on error
 */
inline size_t serializePacket(uint8_t* buffer, size_t bufferSize,
                              uint8_t seq, uint8_t type,
                              const uint8_t* payload, uint16_t payloadLen) {
    size_t totalSize = HEADER_SIZE + payloadLen + CRC_SIZE;

    if (!buffer || bufferSize < totalSize || payloadLen > MAX_PAYLOAD) {
        return 0;
    }

    // Write header
    PacketHeader header;
    header.sync = SYNC_WORD;
    header.seq = seq;
    header.type = type;
    header.length = payloadLen;
    std::memcpy(buffer, &header, HEADER_SIZE);

    // Write payload
    if (payload && payloadLen > 0) {
        std::memcpy(buffer + HEADER_SIZE, payload, payloadLen);
    }

    // Calculate CRC over [SEQ, TYPE, LENGTH, PAYLOAD] (skip SYNC_WORD)
    uint16_t crc = crc16_ccitt(buffer + 2, 4 + payloadLen);

    // Write CRC (little-endian)
    std::memcpy(buffer + HEADER_SIZE + payloadLen, &crc, CRC_SIZE);

    return totalSize;
}

/**
 * Convenience: serialize into a std::vector<uint8_t>.
 */
inline std::vector<uint8_t> serializePacket(uint8_t seq, uint8_t type,
                                             const uint8_t* payload, uint16_t payloadLen) {
    size_t totalSize = HEADER_SIZE + payloadLen + CRC_SIZE;
    std::vector<uint8_t> buffer(totalSize);
    size_t written = serializePacket(buffer.data(), buffer.size(), seq, type, payload, payloadLen);
    if (written == 0) {
        buffer.clear();
    }
    return buffer;
}

/**
 * Convenience: serialize a typed payload struct.
 */
template<typename T>
inline std::vector<uint8_t> serializeCommand(uint8_t seq, CommandType cmd, const T& payload) {
    return serializePacket(seq, static_cast<uint8_t>(cmd),
                           reinterpret_cast<const uint8_t*>(&payload),
                           static_cast<uint16_t>(sizeof(T)));
}

/**
 * Serialize a command with no payload.
 */
inline std::vector<uint8_t> serializeCommand(uint8_t seq, CommandType cmd) {
    return serializePacket(seq, static_cast<uint8_t>(cmd), nullptr, 0);
}

// ============================================================================
// Parse / Validate
// ============================================================================

/**
 * Verify CRC of a complete packet.
 *
 * @param packet    Complete packet data
 * @param packetLen Total packet length
 * @return true if CRC is valid
 */
inline bool verifyPacketCRC(const uint8_t* packet, size_t packetLen) {
    if (!packet || packetLen < MIN_PACKET) {
        return false;
    }

    // Read header to get payload length
    PacketHeader header;
    std::memcpy(&header, packet, HEADER_SIZE);

    if (header.sync != SYNC_WORD) {
        return false;
    }

    size_t expectedLen = HEADER_SIZE + header.length + CRC_SIZE;
    if (packetLen < expectedLen) {
        return false;
    }

    // Calculate CRC over [SEQ, TYPE, LENGTH, PAYLOAD]
    uint16_t calcCrc = crc16_ccitt(packet + 2, 4 + header.length);

    // Read stored CRC
    uint16_t storedCrc;
    std::memcpy(&storedCrc, packet + HEADER_SIZE + header.length, CRC_SIZE);

    return calcCrc == storedCrc;
}

/**
 * Parse a received packet.
 *
 * @param data         Raw received data
 * @param dataLen      Length of received data
 * @param header       [out] Parsed header
 * @param payloadOut   [out] Pointer to payload within data buffer
 * @param payloadLenOut [out] Payload length
 * @return true if a valid packet was found and parsed
 */
inline bool parsePacket(const uint8_t* data, size_t dataLen,
                        PacketHeader& header,
                        const uint8_t** payloadOut, uint16_t* payloadLenOut) {
    if (!data || dataLen < MIN_PACKET) {
        return false;
    }

    // Read header
    std::memcpy(&header, data, HEADER_SIZE);

    // Validate sync word
    if (header.sync != SYNC_WORD) {
        return false;
    }

    // Validate total length
    size_t expectedLen = HEADER_SIZE + header.length + CRC_SIZE;
    if (dataLen < expectedLen) {
        return false;
    }

    // Validate payload length
    if (header.length > MAX_PAYLOAD) {
        return false;
    }

    // Verify CRC
    if (!verifyPacketCRC(data, expectedLen)) {
        return false;
    }

    // Output payload pointer and length
    if (payloadOut) {
        *payloadOut = (header.length > 0) ? (data + HEADER_SIZE) : nullptr;
    }
    if (payloadLenOut) {
        *payloadLenOut = header.length;
    }

    return true;
}

/**
 * Extract typed payload from parsed packet data.
 * Caller must ensure payloadLen >= sizeof(T).
 */
template<typename T>
inline bool extractPayload(const uint8_t* payloadData, uint16_t payloadLen, T& out) {
    if (!payloadData || payloadLen < sizeof(T)) {
        return false;
    }
    std::memcpy(&out, payloadData, sizeof(T));
    return true;
}

/**
 * Find the start of the next valid packet in a data stream.
 * Useful for re-synchronization after corruption.
 *
 * @param data    Data buffer
 * @param dataLen Buffer length
 * @return Offset of SYNC_WORD, or dataLen if not found
 */
inline size_t findSyncWord(const uint8_t* data, size_t dataLen) {
    for (size_t i = 0; i + 1 < dataLen; ++i) {
        uint16_t word;
        std::memcpy(&word, data + i, 2);
        if (word == SYNC_WORD) {
            return i;
        }
    }
    return dataLen;
}

} // namespace protocol
} // namespace firmware
} // namespace robot_controller
