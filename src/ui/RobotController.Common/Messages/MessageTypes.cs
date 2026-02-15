namespace RobotController.Common.Messages;

/// <summary>
/// IPC Message type constants
/// </summary>
public static class MessageTypes
{
    // Connection
    public const string PING = "PING";
    public const string PONG = "PONG";

    // Status
    public const string GET_STATUS = "GET_STATUS";
    public const string STATUS = "STATUS";

    // Joint positions
    public const string GET_JOINT_POSITIONS = "GET_JOINT_POSITIONS";
    public const string JOINT_POSITIONS = "JOINT_POSITIONS";

    // Configuration
    public const string GET_CONFIG = "GET_CONFIG";
    public const string CONFIG = "CONFIG";
    public const string SET_CONFIG = "SET_CONFIG";
    public const string CONFIG_ACK = "CONFIG_ACK";

    // Robot Catalog
    public const string GET_ROBOT_CATALOG = "GET_ROBOT_CATALOG";
    public const string ROBOT_CATALOG = "ROBOT_CATALOG";
    public const string SELECT_ROBOT_MODEL = "SELECT_ROBOT_MODEL";
    public const string SELECT_ROBOT_MODEL_ACK = "SELECT_ROBOT_MODEL_ACK";
    public const string GET_ACTIVE_ROBOT = "GET_ACTIVE_ROBOT";
    public const string ACTIVE_ROBOT = "ACTIVE_ROBOT";
    public const string ROBOT_CONFIG_CHANGED = "ROBOT_CONFIG_CHANGED";

    // Homing
    public const string START_HOMING = "START_HOMING";
    public const string STOP_HOMING = "STOP_HOMING";
    public const string GET_HOMING_STATE = "GET_HOMING_STATE";
    public const string HOMING_STATE_CHANGED = "HOMING_STATE_CHANGED";

    // Tool Management
    public const string GET_TOOL_LIST = "GET_TOOL_LIST";
    public const string GET_TOOL = "GET_TOOL";
    public const string CREATE_TOOL = "CREATE_TOOL";
    public const string UPDATE_TOOL = "UPDATE_TOOL";
    public const string DELETE_TOOL = "DELETE_TOOL";
    public const string SELECT_TOOL = "SELECT_TOOL";
    public const string GET_ACTIVE_TOOL = "GET_ACTIVE_TOOL";
    public const string START_TCP_CALIBRATION = "START_TCP_CALIBRATION";
    public const string RECORD_CALIBRATION_POINT = "RECORD_CALIBRATION_POINT";
    public const string FINISH_CALIBRATION = "FINISH_CALIBRATION";
    public const string CANCEL_CALIBRATION = "CANCEL_CALIBRATION";
    public const string GET_CALIBRATION_STATUS = "GET_CALIBRATION_STATUS";
    public const string TOOL_CHANGED = "TOOL_CHANGED";

    // Operation Mode
    public const string GET_OPERATION_MODE = "GET_OPERATION_MODE";
    public const string SET_OPERATION_MODE = "SET_OPERATION_MODE";
    public const string GET_MODE_REQUIREMENTS = "GET_MODE_REQUIREMENTS";
    public const string OPERATION_MODE_CHANGED = "OPERATION_MODE_CHANGED";

    // Base Frame Management
    public const string GET_BASE_LIST = "GET_BASE_LIST";
    public const string GET_BASE = "GET_BASE";
    public const string CREATE_BASE = "CREATE_BASE";
    public const string UPDATE_BASE = "UPDATE_BASE";
    public const string DELETE_BASE = "DELETE_BASE";
    public const string SELECT_BASE = "SELECT_BASE";
    public const string GET_ACTIVE_BASE = "GET_ACTIVE_BASE";
    public const string START_BASE_CALIBRATION = "START_BASE_CALIBRATION";
    public const string RECORD_BASE_POINT = "RECORD_BASE_POINT";
    public const string FINISH_BASE_CALIBRATION = "FINISH_BASE_CALIBRATION";
    public const string CANCEL_BASE_CALIBRATION = "CANCEL_BASE_CALIBRATION";
    public const string GET_BASE_CALIBRATION_STATUS = "GET_BASE_CALIBRATION_STATUS";
    public const string BASE_CHANGED = "BASE_CHANGED";

    // Override Control
    public const string SET_OVERRIDE = "SET_OVERRIDE";
    public const string GET_OVERRIDE = "GET_OVERRIDE";
    public const string OVERRIDE_CHANGED = "OVERRIDE_CHANGED";

    // Robot Package (Virtual Simulation)
    public const string GET_ROBOT_PACKAGES = "GET_ROBOT_PACKAGES";
    public const string LOAD_ROBOT_PACKAGE = "LOAD_ROBOT_PACKAGE";
    public const string GET_ACTIVE_PACKAGE = "GET_ACTIVE_PACKAGE";
    public const string ROBOT_PACKAGE_CHANGED = "ROBOT_PACKAGE_CHANGED";
    public const string RELOAD_PACKAGES = "RELOAD_PACKAGES";

    // Program Execution (Virtual Simulation)
    public const string LOAD_PROGRAM = "LOAD_PROGRAM";
    public const string RUN_PROGRAM = "RUN_PROGRAM";
    public const string STEP_PROGRAM = "STEP_PROGRAM";
    public const string PAUSE_PROGRAM = "PAUSE_PROGRAM";
    public const string STOP_PROGRAM = "STOP_PROGRAM";
    public const string RESET_PROGRAM = "RESET_PROGRAM";
    public const string GET_PROGRAM_STATE = "GET_PROGRAM_STATE";
    public const string PROGRAM_STATE_CHANGED = "PROGRAM_STATE_CHANGED";
    public const string SET_POINT = "SET_POINT";
    public const string GET_POINTS = "GET_POINTS";
    public const string BLOCK_SELECT = "BLOCK_SELECT";
    public const string BACKWARD_STEP = "BACKWARD_STEP";

    // URDF Import (Auto robot package creation)
    public const string PARSE_URDF = "PARSE_URDF";
    public const string GENERATE_ROBOT_YAML = "GENERATE_ROBOT_YAML";

    // Jog Control
    public const string JOG_START = "JOG_START";
    public const string JOG_STOP = "JOG_STOP";
    public const string JOG_MOVE = "JOG_MOVE";
    public const string JOG_STEP = "JOG_STEP";

    // Kinematics (3D Jogging)
    public const string COMPUTE_IK = "COMPUTE_IK";
    public const string SET_JOINTS = "SET_JOINTS";

    // Firmware Control
    public const string FIRMWARE_CONNECT = "FIRMWARE_CONNECT";
    public const string FIRMWARE_DISCONNECT = "FIRMWARE_DISCONNECT";
    public const string FIRMWARE_GET_MODE = "FIRMWARE_GET_MODE";
    public const string FIRMWARE_SET_MODE = "FIRMWARE_SET_MODE";
    public const string FIRMWARE_SCAN_PORTS = "FIRMWARE_SCAN_PORTS";

    // V2: Drive/Home Control
    public const string ENABLE_DRIVES = "ENABLE_DRIVES";
    public const string DISABLE_DRIVES = "DISABLE_DRIVES";
    public const string RESET_ALARM = "RESET_ALARM";
    public const string HOME_ALL = "HOME_ALL";
    public const string HOME_AXIS = "HOME_AXIS";
    public const string GET_DRIVE_STATUS = "GET_DRIVE_STATUS";
    public const string DRIVE_STATUS_CHANGED = "DRIVE_STATUS_CHANGED";

    // V2: STM32 Ethernet Connection
    public const string STM32_CONNECT = "STM32_CONNECT";
    public const string STM32_DISCONNECT = "STM32_DISCONNECT";
    public const string GET_IO_STATE = "GET_IO_STATE";
    public const string SET_IO_OUTPUT = "SET_IO_OUTPUT";

    // V2: Firmware Packet Logging
    public const string FIRMWARE_PACKET_LOG = "FIRMWARE_PACKET_LOG";

    // Commands
    public const string COMMAND = "COMMAND";
    public const string COMMAND_ACK = "COMMAND_ACK";

    // Errors
    public const string ERROR = "ERROR";
}
