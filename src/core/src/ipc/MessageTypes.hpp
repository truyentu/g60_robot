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

    // Robot Catalog
    GET_ROBOT_CATALOG,
    ROBOT_CATALOG,
    SELECT_ROBOT_MODEL,
    SELECT_ROBOT_MODEL_ACK,
    GET_ACTIVE_ROBOT,
    ACTIVE_ROBOT,
    ROBOT_CONFIG_CHANGED,

    // Homing
    START_HOMING,
    STOP_HOMING,
    GET_HOMING_STATE,
    HOMING_STATE_CHANGED,

    // Tool Management
    GET_TOOL_LIST,
    GET_TOOL,
    CREATE_TOOL,
    UPDATE_TOOL,
    DELETE_TOOL,
    SELECT_TOOL,
    GET_ACTIVE_TOOL,
    START_TCP_CALIBRATION,
    RECORD_CALIBRATION_POINT,
    FINISH_CALIBRATION,
    CANCEL_CALIBRATION,
    GET_CALIBRATION_STATUS,
    TOOL_CHANGED,

    // Operation Mode
    GET_OPERATION_MODE,
    SET_OPERATION_MODE,
    GET_MODE_REQUIREMENTS,
    OPERATION_MODE_CHANGED,

    // Base Frame Management
    GET_BASE_LIST,
    GET_BASE,
    CREATE_BASE,
    UPDATE_BASE,
    DELETE_BASE,
    SELECT_BASE,
    GET_ACTIVE_BASE,
    START_BASE_CALIBRATION,
    RECORD_BASE_POINT,
    FINISH_BASE_CALIBRATION,
    CANCEL_BASE_CALIBRATION,
    GET_BASE_CALIBRATION_STATUS,
    BASE_CHANGED,

    // Override Control
    SET_OVERRIDE,
    GET_OVERRIDE,
    OVERRIDE_CHANGED,

    // Robot Package (Virtual Simulation)
    GET_ROBOT_PACKAGES,
    LOAD_ROBOT_PACKAGE,
    GET_ACTIVE_PACKAGE,
    ROBOT_PACKAGE_CHANGED,
    RELOAD_PACKAGES,

    // Program Execution (Virtual Simulation)
    LOAD_PROGRAM,
    RUN_PROGRAM,
    STEP_PROGRAM,
    PAUSE_PROGRAM,
    STOP_PROGRAM,
    RESET_PROGRAM,
    GET_PROGRAM_STATE,
    PROGRAM_STATE_CHANGED,
    SET_POINT,
    GET_POINTS,

    // URDF Import (Auto robot package creation)
    PARSE_URDF,
    GENERATE_ROBOT_YAML,

    // Jog Control
    JOG_START,
    JOG_STOP,
    JOG_MOVE,
    JOG_STEP,

    // Kinematics (3D Jogging)
    COMPUTE_IK,
    SET_JOINTS,

    // Firmware Control
    FIRMWARE_CONNECT,
    FIRMWARE_DISCONNECT,
    FIRMWARE_GET_MODE,
    FIRMWARE_SET_MODE,
    FIRMWARE_SCAN_PORTS,

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
        case MessageType::GET_ROBOT_CATALOG:   return "GET_ROBOT_CATALOG";
        case MessageType::ROBOT_CATALOG:       return "ROBOT_CATALOG";
        case MessageType::SELECT_ROBOT_MODEL:  return "SELECT_ROBOT_MODEL";
        case MessageType::SELECT_ROBOT_MODEL_ACK: return "SELECT_ROBOT_MODEL_ACK";
        case MessageType::GET_ACTIVE_ROBOT:    return "GET_ACTIVE_ROBOT";
        case MessageType::ACTIVE_ROBOT:        return "ACTIVE_ROBOT";
        case MessageType::ROBOT_CONFIG_CHANGED: return "ROBOT_CONFIG_CHANGED";
        case MessageType::START_HOMING:        return "START_HOMING";
        case MessageType::STOP_HOMING:         return "STOP_HOMING";
        case MessageType::GET_HOMING_STATE:    return "GET_HOMING_STATE";
        case MessageType::HOMING_STATE_CHANGED: return "HOMING_STATE_CHANGED";
        case MessageType::GET_TOOL_LIST:       return "GET_TOOL_LIST";
        case MessageType::GET_TOOL:            return "GET_TOOL";
        case MessageType::CREATE_TOOL:         return "CREATE_TOOL";
        case MessageType::UPDATE_TOOL:         return "UPDATE_TOOL";
        case MessageType::DELETE_TOOL:         return "DELETE_TOOL";
        case MessageType::SELECT_TOOL:         return "SELECT_TOOL";
        case MessageType::GET_ACTIVE_TOOL:     return "GET_ACTIVE_TOOL";
        case MessageType::START_TCP_CALIBRATION: return "START_TCP_CALIBRATION";
        case MessageType::RECORD_CALIBRATION_POINT: return "RECORD_CALIBRATION_POINT";
        case MessageType::FINISH_CALIBRATION:  return "FINISH_CALIBRATION";
        case MessageType::CANCEL_CALIBRATION:  return "CANCEL_CALIBRATION";
        case MessageType::GET_CALIBRATION_STATUS: return "GET_CALIBRATION_STATUS";
        case MessageType::TOOL_CHANGED:        return "TOOL_CHANGED";
        case MessageType::GET_OPERATION_MODE:  return "GET_OPERATION_MODE";
        case MessageType::SET_OPERATION_MODE:  return "SET_OPERATION_MODE";
        case MessageType::GET_MODE_REQUIREMENTS: return "GET_MODE_REQUIREMENTS";
        case MessageType::OPERATION_MODE_CHANGED: return "OPERATION_MODE_CHANGED";
        case MessageType::GET_BASE_LIST:       return "GET_BASE_LIST";
        case MessageType::GET_BASE:            return "GET_BASE";
        case MessageType::CREATE_BASE:         return "CREATE_BASE";
        case MessageType::UPDATE_BASE:         return "UPDATE_BASE";
        case MessageType::DELETE_BASE:         return "DELETE_BASE";
        case MessageType::SELECT_BASE:         return "SELECT_BASE";
        case MessageType::GET_ACTIVE_BASE:     return "GET_ACTIVE_BASE";
        case MessageType::START_BASE_CALIBRATION: return "START_BASE_CALIBRATION";
        case MessageType::RECORD_BASE_POINT:   return "RECORD_BASE_POINT";
        case MessageType::FINISH_BASE_CALIBRATION: return "FINISH_BASE_CALIBRATION";
        case MessageType::CANCEL_BASE_CALIBRATION: return "CANCEL_BASE_CALIBRATION";
        case MessageType::GET_BASE_CALIBRATION_STATUS: return "GET_BASE_CALIBRATION_STATUS";
        case MessageType::BASE_CHANGED:        return "BASE_CHANGED";
        case MessageType::SET_OVERRIDE:        return "SET_OVERRIDE";
        case MessageType::GET_OVERRIDE:        return "GET_OVERRIDE";
        case MessageType::OVERRIDE_CHANGED:    return "OVERRIDE_CHANGED";
        case MessageType::GET_ROBOT_PACKAGES:  return "GET_ROBOT_PACKAGES";
        case MessageType::LOAD_ROBOT_PACKAGE:  return "LOAD_ROBOT_PACKAGE";
        case MessageType::GET_ACTIVE_PACKAGE:  return "GET_ACTIVE_PACKAGE";
        case MessageType::RELOAD_PACKAGES:     return "RELOAD_PACKAGES";
        case MessageType::ROBOT_PACKAGE_CHANGED: return "ROBOT_PACKAGE_CHANGED";
        case MessageType::LOAD_PROGRAM:        return "LOAD_PROGRAM";
        case MessageType::RUN_PROGRAM:         return "RUN_PROGRAM";
        case MessageType::STEP_PROGRAM:        return "STEP_PROGRAM";
        case MessageType::PAUSE_PROGRAM:       return "PAUSE_PROGRAM";
        case MessageType::STOP_PROGRAM:        return "STOP_PROGRAM";
        case MessageType::RESET_PROGRAM:       return "RESET_PROGRAM";
        case MessageType::GET_PROGRAM_STATE:   return "GET_PROGRAM_STATE";
        case MessageType::PROGRAM_STATE_CHANGED: return "PROGRAM_STATE_CHANGED";
        case MessageType::SET_POINT:           return "SET_POINT";
        case MessageType::GET_POINTS:          return "GET_POINTS";
        case MessageType::PARSE_URDF:          return "PARSE_URDF";
        case MessageType::GENERATE_ROBOT_YAML: return "GENERATE_ROBOT_YAML";
        case MessageType::JOG_START:          return "JOG_START";
        case MessageType::JOG_STOP:           return "JOG_STOP";
        case MessageType::JOG_MOVE:           return "JOG_MOVE";
        case MessageType::JOG_STEP:           return "JOG_STEP";
        case MessageType::COMPUTE_IK:         return "COMPUTE_IK";
        case MessageType::SET_JOINTS:         return "SET_JOINTS";
        case MessageType::FIRMWARE_CONNECT:   return "FIRMWARE_CONNECT";
        case MessageType::FIRMWARE_DISCONNECT: return "FIRMWARE_DISCONNECT";
        case MessageType::FIRMWARE_GET_MODE:  return "FIRMWARE_GET_MODE";
        case MessageType::FIRMWARE_SET_MODE:  return "FIRMWARE_SET_MODE";
        case MessageType::FIRMWARE_SCAN_PORTS: return "FIRMWARE_SCAN_PORTS";
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
    if (str == "GET_ROBOT_CATALOG")   return MessageType::GET_ROBOT_CATALOG;
    if (str == "ROBOT_CATALOG")       return MessageType::ROBOT_CATALOG;
    if (str == "SELECT_ROBOT_MODEL")  return MessageType::SELECT_ROBOT_MODEL;
    if (str == "SELECT_ROBOT_MODEL_ACK") return MessageType::SELECT_ROBOT_MODEL_ACK;
    if (str == "GET_ACTIVE_ROBOT")    return MessageType::GET_ACTIVE_ROBOT;
    if (str == "ACTIVE_ROBOT")        return MessageType::ACTIVE_ROBOT;
    if (str == "ROBOT_CONFIG_CHANGED") return MessageType::ROBOT_CONFIG_CHANGED;
    if (str == "START_HOMING")        return MessageType::START_HOMING;
    if (str == "STOP_HOMING")         return MessageType::STOP_HOMING;
    if (str == "GET_HOMING_STATE")    return MessageType::GET_HOMING_STATE;
    if (str == "HOMING_STATE_CHANGED") return MessageType::HOMING_STATE_CHANGED;
    if (str == "GET_TOOL_LIST")       return MessageType::GET_TOOL_LIST;
    if (str == "GET_TOOL")            return MessageType::GET_TOOL;
    if (str == "CREATE_TOOL")         return MessageType::CREATE_TOOL;
    if (str == "UPDATE_TOOL")         return MessageType::UPDATE_TOOL;
    if (str == "DELETE_TOOL")         return MessageType::DELETE_TOOL;
    if (str == "SELECT_TOOL")         return MessageType::SELECT_TOOL;
    if (str == "GET_ACTIVE_TOOL")     return MessageType::GET_ACTIVE_TOOL;
    if (str == "START_TCP_CALIBRATION") return MessageType::START_TCP_CALIBRATION;
    if (str == "RECORD_CALIBRATION_POINT") return MessageType::RECORD_CALIBRATION_POINT;
    if (str == "FINISH_CALIBRATION")  return MessageType::FINISH_CALIBRATION;
    if (str == "CANCEL_CALIBRATION")  return MessageType::CANCEL_CALIBRATION;
    if (str == "GET_CALIBRATION_STATUS") return MessageType::GET_CALIBRATION_STATUS;
    if (str == "TOOL_CHANGED")        return MessageType::TOOL_CHANGED;
    if (str == "GET_OPERATION_MODE")  return MessageType::GET_OPERATION_MODE;
    if (str == "SET_OPERATION_MODE")  return MessageType::SET_OPERATION_MODE;
    if (str == "GET_MODE_REQUIREMENTS") return MessageType::GET_MODE_REQUIREMENTS;
    if (str == "OPERATION_MODE_CHANGED") return MessageType::OPERATION_MODE_CHANGED;
    if (str == "GET_BASE_LIST")       return MessageType::GET_BASE_LIST;
    if (str == "GET_BASE")            return MessageType::GET_BASE;
    if (str == "CREATE_BASE")         return MessageType::CREATE_BASE;
    if (str == "UPDATE_BASE")         return MessageType::UPDATE_BASE;
    if (str == "DELETE_BASE")         return MessageType::DELETE_BASE;
    if (str == "SELECT_BASE")         return MessageType::SELECT_BASE;
    if (str == "GET_ACTIVE_BASE")     return MessageType::GET_ACTIVE_BASE;
    if (str == "START_BASE_CALIBRATION") return MessageType::START_BASE_CALIBRATION;
    if (str == "RECORD_BASE_POINT")   return MessageType::RECORD_BASE_POINT;
    if (str == "FINISH_BASE_CALIBRATION") return MessageType::FINISH_BASE_CALIBRATION;
    if (str == "CANCEL_BASE_CALIBRATION") return MessageType::CANCEL_BASE_CALIBRATION;
    if (str == "GET_BASE_CALIBRATION_STATUS") return MessageType::GET_BASE_CALIBRATION_STATUS;
    if (str == "BASE_CHANGED")        return MessageType::BASE_CHANGED;
    if (str == "SET_OVERRIDE")        return MessageType::SET_OVERRIDE;
    if (str == "GET_OVERRIDE")        return MessageType::GET_OVERRIDE;
    if (str == "OVERRIDE_CHANGED")    return MessageType::OVERRIDE_CHANGED;
    if (str == "GET_ROBOT_PACKAGES")  return MessageType::GET_ROBOT_PACKAGES;
    if (str == "LOAD_ROBOT_PACKAGE")  return MessageType::LOAD_ROBOT_PACKAGE;
    if (str == "GET_ACTIVE_PACKAGE")  return MessageType::GET_ACTIVE_PACKAGE;
    if (str == "RELOAD_PACKAGES")     return MessageType::RELOAD_PACKAGES;
    if (str == "ROBOT_PACKAGE_CHANGED") return MessageType::ROBOT_PACKAGE_CHANGED;
    if (str == "LOAD_PROGRAM")        return MessageType::LOAD_PROGRAM;
    if (str == "RUN_PROGRAM")         return MessageType::RUN_PROGRAM;
    if (str == "STEP_PROGRAM")        return MessageType::STEP_PROGRAM;
    if (str == "PAUSE_PROGRAM")       return MessageType::PAUSE_PROGRAM;
    if (str == "STOP_PROGRAM")        return MessageType::STOP_PROGRAM;
    if (str == "RESET_PROGRAM")       return MessageType::RESET_PROGRAM;
    if (str == "GET_PROGRAM_STATE")   return MessageType::GET_PROGRAM_STATE;
    if (str == "PROGRAM_STATE_CHANGED") return MessageType::PROGRAM_STATE_CHANGED;
    if (str == "SET_POINT")           return MessageType::SET_POINT;
    if (str == "GET_POINTS")          return MessageType::GET_POINTS;
    if (str == "PARSE_URDF")          return MessageType::PARSE_URDF;
    if (str == "GENERATE_ROBOT_YAML") return MessageType::GENERATE_ROBOT_YAML;
    if (str == "JOG_START")          return MessageType::JOG_START;
    if (str == "JOG_STOP")           return MessageType::JOG_STOP;
    if (str == "JOG_MOVE")           return MessageType::JOG_MOVE;
    if (str == "JOG_STEP")           return MessageType::JOG_STEP;
    if (str == "COMPUTE_IK")         return MessageType::COMPUTE_IK;
    if (str == "SET_JOINTS")         return MessageType::SET_JOINTS;
    if (str == "FIRMWARE_CONNECT")   return MessageType::FIRMWARE_CONNECT;
    if (str == "FIRMWARE_DISCONNECT") return MessageType::FIRMWARE_DISCONNECT;
    if (str == "FIRMWARE_GET_MODE")  return MessageType::FIRMWARE_GET_MODE;
    if (str == "FIRMWARE_SET_MODE")  return MessageType::FIRMWARE_SET_MODE;
    if (str == "FIRMWARE_SCAN_PORTS") return MessageType::FIRMWARE_SCAN_PORTS;
    if (str == "COMMAND")             return MessageType::COMMAND;
    if (str == "COMMAND_ACK")         return MessageType::COMMAND_ACK;
    if (str == "ERROR")               return MessageType::ERROR;
    return MessageType::UNKNOWN;
}

} // namespace ipc
} // namespace robot_controller
