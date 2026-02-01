/**
 * @file MessageTypes.hpp
 * @brief IPC Message type definitions
 */

#pragma once

#include <string>

namespace robot_controller {
namespace ipc {

/**
 * Message types for IPC communication
 */
enum class MessageType {
    // Connection
    PING,
    PONG,

    // Status
    GET_STATUS,
    STATUS,

    // Joint positions
    GET_JOINT_POSITIONS,
    JOINT_POSITIONS,

    // Configuration
    GET_CONFIG,
    CONFIG,
    SET_CONFIG,
    CONFIG_ACK,

    // Commands
    COMMAND,
    COMMAND_ACK,

    // Errors
    ERROR,

    // Unknown
    UNKNOWN
};

/**
 * Convert MessageType to string
 */
inline std::string messageTypeToString(MessageType type) {
    switch (type) {
        case MessageType::PING:                return "PING";
        case MessageType::PONG:                return "PONG";
        case MessageType::GET_STATUS:          return "GET_STATUS";
        case MessageType::STATUS:              return "STATUS";
        case MessageType::GET_JOINT_POSITIONS: return "GET_JOINT_POSITIONS";
        case MessageType::JOINT_POSITIONS:     return "JOINT_POSITIONS";
        case MessageType::GET_CONFIG:          return "GET_CONFIG";
        case MessageType::CONFIG:              return "CONFIG";
        case MessageType::SET_CONFIG:          return "SET_CONFIG";
        case MessageType::CONFIG_ACK:          return "CONFIG_ACK";
        case MessageType::COMMAND:             return "COMMAND";
        case MessageType::COMMAND_ACK:         return "COMMAND_ACK";
        case MessageType::ERROR:               return "ERROR";
        default:                               return "UNKNOWN";
    }
}

/**
 * Convert string to MessageType
 */
inline MessageType stringToMessageType(const std::string& str) {
    if (str == "PING")                return MessageType::PING;
    if (str == "PONG")                return MessageType::PONG;
    if (str == "GET_STATUS")          return MessageType::GET_STATUS;
    if (str == "STATUS")              return MessageType::STATUS;
    if (str == "GET_JOINT_POSITIONS") return MessageType::GET_JOINT_POSITIONS;
    if (str == "JOINT_POSITIONS")     return MessageType::JOINT_POSITIONS;
    if (str == "GET_CONFIG")          return MessageType::GET_CONFIG;
    if (str == "CONFIG")              return MessageType::CONFIG;
    if (str == "SET_CONFIG")          return MessageType::SET_CONFIG;
    if (str == "CONFIG_ACK")          return MessageType::CONFIG_ACK;
    if (str == "COMMAND")             return MessageType::COMMAND;
    if (str == "COMMAND_ACK")         return MessageType::COMMAND_ACK;
    if (str == "ERROR")               return MessageType::ERROR;
    return MessageType::UNKNOWN;
}

} // namespace ipc
} // namespace robot_controller
