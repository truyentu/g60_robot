/**
 * @file protocol_packet.c
 * @brief Packet framing implementation — CRC-CCITT, serialize, parse
 *
 * Same algorithm as BinaryProtocol.hpp (C++ PC-side).
 */

#include "protocol_packet.h"
#include <string.h>

/* ========================================================================= */
/*  CRC-CCITT (0xFFFF initial, polynomial 0x1021)                            */
/* ========================================================================= */

uint16_t protocol_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}

/* ========================================================================= */
/*  Serialize                                                                */
/* ========================================================================= */

size_t protocol_serialize(uint8_t *buffer, size_t buffer_size,
                          uint8_t seq, uint8_t type,
                          const uint8_t *payload, uint16_t payload_len)
{
    size_t total_size = PROTO_HEADER_SIZE + payload_len + PROTO_CRC_SIZE;

    if (!buffer || buffer_size < total_size || payload_len > PROTO_MAX_PAYLOAD) {
        return 0;
    }

    /* Write header */
    PacketHeader header;
    header.sync   = PROTO_SYNC_WORD;
    header.seq    = seq;
    header.type   = type;
    header.length = payload_len;
    memcpy(buffer, &header, PROTO_HEADER_SIZE);

    /* Write payload */
    if (payload && payload_len > 0) {
        memcpy(buffer + PROTO_HEADER_SIZE, payload, payload_len);
    }

    /* CRC over [SEQ, TYPE, LENGTH, PAYLOAD] — skip SYNC_WORD (2 bytes) */
    uint16_t crc = protocol_crc16(buffer + 2, 4 + payload_len);

    /* Write CRC (little-endian, native on Cortex-M) */
    memcpy(buffer + PROTO_HEADER_SIZE + payload_len, &crc, PROTO_CRC_SIZE);

    return total_size;
}

/* ========================================================================= */
/*  Parse / Validate                                                         */
/* ========================================================================= */

bool protocol_verify_crc(const uint8_t *packet, size_t packet_len)
{
    if (!packet || packet_len < PROTO_MIN_PACKET) {
        return false;
    }

    PacketHeader header;
    memcpy(&header, packet, PROTO_HEADER_SIZE);

    if (header.sync != PROTO_SYNC_WORD) {
        return false;
    }

    size_t expected_len = PROTO_HEADER_SIZE + header.length + PROTO_CRC_SIZE;
    if (packet_len < expected_len) {
        return false;
    }

    /* Calculate CRC over [SEQ, TYPE, LENGTH, PAYLOAD] */
    uint16_t calc_crc = protocol_crc16(packet + 2, 4 + header.length);

    /* Read stored CRC */
    uint16_t stored_crc;
    memcpy(&stored_crc, packet + PROTO_HEADER_SIZE + header.length, PROTO_CRC_SIZE);

    return calc_crc == stored_crc;
}

bool protocol_parse(const uint8_t *data, size_t data_len,
                    PacketHeader *header_out,
                    const uint8_t **payload_out, uint16_t *payload_len_out)
{
    if (!data || data_len < PROTO_MIN_PACKET) {
        return false;
    }

    /* Read header */
    PacketHeader header;
    memcpy(&header, data, PROTO_HEADER_SIZE);

    /* Validate sync word */
    if (header.sync != PROTO_SYNC_WORD) {
        return false;
    }

    /* Validate payload length */
    if (header.length > PROTO_MAX_PAYLOAD) {
        return false;
    }

    /* Validate total length */
    size_t expected_len = PROTO_HEADER_SIZE + header.length + PROTO_CRC_SIZE;
    if (data_len < expected_len) {
        return false;
    }

    /* Verify CRC */
    if (!protocol_verify_crc(data, expected_len)) {
        return false;
    }

    /* Output */
    if (header_out) {
        *header_out = header;
    }
    if (payload_out) {
        *payload_out = (header.length > 0) ? (data + PROTO_HEADER_SIZE) : NULL;
    }
    if (payload_len_out) {
        *payload_len_out = header.length;
    }

    return true;
}

/* ========================================================================= */
/*  Re-synchronization                                                       */
/* ========================================================================= */

size_t protocol_find_sync(const uint8_t *data, size_t data_len)
{
    for (size_t i = 0; i + 1 < data_len; ++i) {
        uint16_t word;
        memcpy(&word, data + i, 2);
        if (word == PROTO_SYNC_WORD) {
            return i;
        }
    }
    return data_len;
}
