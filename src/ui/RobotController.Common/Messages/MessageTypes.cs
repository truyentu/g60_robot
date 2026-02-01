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

    // Commands
    public const string COMMAND = "COMMAND";
    public const string COMMAND_ACK = "COMMAND_ACK";

    // Errors
    public const string ERROR = "ERROR";
}
