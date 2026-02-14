/**
 * @file protocol_packet.h
 * @brief Packet framing, CRC-CCITT, serialize/parse — C implementation
 *
 * Mirrors: src/core/src/firmware/protocol/BinaryProtocol.hpp
 * Same packet format, same CRC algorithm, same byte layout.
 *
 * Packet format:
 *   [SYNC_WORD:2][SEQ:1][TYPE:1][LENGTH:2][PAYLOAD:0..512][CRC16:2]
 */

#ifndef PROTOCOL_PACKET_H
#define PROTOCOL_PACKET_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ========================================================================= */
/*  Constants                                                                */
/* ========================================================================= */

#define PROTO_SYNC_WORD     0xAA55
#define PROTO_HEADER_SIZE   6       /* SYNC(2) + SEQ(1) + TYPE(1) + LEN(2) */
#define PROTO_CRC_SIZE      2
#define PROTO_MIN_PACKET    (PROTO_HEADER_SIZE + PROTO_CRC_SIZE)  /* 8 bytes */
#define PROTO_MAX_PAYLOAD   512
#define PROTO_MAX_PACKET    (PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD + PROTO_CRC_SIZE)

/* ========================================================================= */
/*  Packet Header                                                            */
/* ========================================================================= */

#pragma pack(push, 1)
typedef struct {
    uint16_t sync;     /* Must be PROTO_SYNC_WORD (0xAA55) */
    uint8_t  seq;      /* Sequence number                  */
    uint8_t  type;     /* CommandType or ResponseType       */
    uint16_t length;   /* Payload length                   */
} PacketHeader;
_Static_assert(sizeof(PacketHeader) == 6, "PacketHeader must be 6 bytes");
#pragma pack(pop)

/* ========================================================================= */
/*  Functions                                                                */
/* ========================================================================= */

/**
 * Calculate CRC-CCITT over a data buffer.
 * Initial value 0xFFFF, polynomial 0x1021.
 *
 * @param data   Input data
 * @param length Number of bytes
 * @return CRC16 value
 */
uint16_t protocol_crc16(const uint8_t *data, size_t length);

/**
 * Serialize a packet into a byte buffer.
 *
 * @param buffer      Output buffer (must be >= PROTO_HEADER_SIZE + payload_len + PROTO_CRC_SIZE)
 * @param buffer_size Size of output buffer
 * @param seq         Sequence number
 * @param type        Command or Response type (uint8_t)
 * @param payload     Payload data (NULL if payload_len == 0)
 * @param payload_len Payload length
 * @return Total packet size written, or 0 on error
 */
size_t protocol_serialize(uint8_t *buffer, size_t buffer_size,
                          uint8_t seq, uint8_t type,
                          const uint8_t *payload, uint16_t payload_len);

/**
 * Parse and validate a received packet.
 *
 * @param data            Raw received data
 * @param data_len        Length of received data
 * @param header_out      [out] Parsed header
 * @param payload_out     [out] Pointer to payload within data buffer
 * @param payload_len_out [out] Payload length
 * @return true if valid packet found and parsed
 */
bool protocol_parse(const uint8_t *data, size_t data_len,
                    PacketHeader *header_out,
                    const uint8_t **payload_out, uint16_t *payload_len_out);

/**
 * Verify CRC of a complete packet.
 *
 * @param packet     Complete packet data
 * @param packet_len Total packet length
 * @return true if CRC is valid
 */
bool protocol_verify_crc(const uint8_t *packet, size_t packet_len);

/**
 * Find the next sync word in a data stream (for re-synchronization).
 *
 * @param data     Data buffer
 * @param data_len Buffer length
 * @return Offset of sync word, or data_len if not found
 */
size_t protocol_find_sync(const uint8_t *data, size_t data_len);

#endif /* PROTOCOL_PACKET_H */
