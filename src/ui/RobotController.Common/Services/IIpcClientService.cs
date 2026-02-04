using RobotController.Common.Messages;

namespace RobotController.Common.Services;

public interface IIpcClientService : IDisposable
{
    bool IsConnected { get; }

    event EventHandler<StatusPayload>? StatusReceived;
    event EventHandler<bool>? ConnectionStateChanged;
    event EventHandler<string>? ErrorOccurred;
    event EventHandler<RobotConfigChangedEvent>? RobotConfigChanged;
    event EventHandler<HomingStateChangedEvent>? HomingStateChanged;
    event EventHandler<ToolChangedEvent>? ToolChanged;
    event EventHandler<OperationModeChangedEvent>? OperationModeChanged;
    event EventHandler<BaseChangedEvent>? BaseChanged;
    event EventHandler<OverrideChangedEvent>? OverrideChanged;

    Task<bool> ConnectAsync(string requestAddress, string subscribeAddress, CancellationToken cancellationToken = default);

    void Disconnect();

    Task<PongPayload?> PingAsync(CancellationToken cancellationToken = default);

    Task<StatusPayload?> GetStatusAsync(CancellationToken cancellationToken = default);

    Task<JointPositionsPayload?> GetJointPositionsAsync(CancellationToken cancellationToken = default);

    Task<bool> SendCommandAsync(string command, object? parameters = null, CancellationToken cancellationToken = default);

    // Robot Catalog operations
    Task<GetRobotCatalogResponse?> GetRobotCatalogAsync(CancellationToken cancellationToken = default);

    Task<SelectRobotModelResponse?> SelectRobotModelAsync(string modelId, CancellationToken cancellationToken = default);

    Task<GetActiveRobotResponse?> GetActiveRobotAsync(CancellationToken cancellationToken = default);

    // Homing operations
    Task<StartHomingResponse?> StartHomingAsync(int jointIndex = -1, string method = "LIMIT_SWITCH", CancellationToken cancellationToken = default);

    Task<StopHomingResponse?> StopHomingAsync(int jointIndex = -1, CancellationToken cancellationToken = default);

    Task<HomingStateResponse?> GetHomingStateAsync(CancellationToken cancellationToken = default);

    // Tool Management operations
    Task<GetToolListResponse?> GetToolListAsync(CancellationToken cancellationToken = default);

    Task<GetToolResponse?> GetToolAsync(string toolId, CancellationToken cancellationToken = default);

    Task<CreateToolResponse?> CreateToolAsync(ToolData tool, CancellationToken cancellationToken = default);

    Task<UpdateToolResponse?> UpdateToolAsync(string toolId, ToolData tool, CancellationToken cancellationToken = default);

    Task<DeleteToolResponse?> DeleteToolAsync(string toolId, CancellationToken cancellationToken = default);

    Task<SelectToolResponse?> SelectToolAsync(string toolId, CancellationToken cancellationToken = default);

    Task<GetActiveToolResponse?> GetActiveToolAsync(CancellationToken cancellationToken = default);

    // Tool Calibration operations
    Task<StartCalibrationResponse?> StartTcpCalibrationAsync(string method = "FOUR_POINT", CancellationToken cancellationToken = default);

    Task<RecordCalibrationPointResponse?> RecordCalibrationPointAsync(List<double> jointAngles, CancellationToken cancellationToken = default);

    Task<FinishCalibrationResponse?> FinishCalibrationAsync(CancellationToken cancellationToken = default);

    Task<bool> CancelCalibrationAsync(CancellationToken cancellationToken = default);

    Task<CalibrationStatusResponse?> GetCalibrationStatusAsync(CancellationToken cancellationToken = default);

    // Operation Mode operations
    Task<GetOperationModeResponse?> GetOperationModeAsync(CancellationToken cancellationToken = default);

    Task<SetOperationModeResponse?> SetOperationModeAsync(string mode, CancellationToken cancellationToken = default);

    Task<GetModeRequirementsResponse?> GetModeRequirementsAsync(string targetMode, CancellationToken cancellationToken = default);

    // Base Frame operations
    Task<GetBaseListResponse?> GetBaseListAsync(CancellationToken cancellationToken = default);

    Task<GetBaseResponse?> GetBaseAsync(string baseId, CancellationToken cancellationToken = default);

    Task<CreateBaseResponse?> CreateBaseAsync(BaseFrameData baseFrame, CancellationToken cancellationToken = default);

    Task<UpdateBaseResponse?> UpdateBaseAsync(string baseId, BaseFrameData baseFrame, CancellationToken cancellationToken = default);

    Task<DeleteBaseResponse?> DeleteBaseAsync(string baseId, CancellationToken cancellationToken = default);

    Task<SelectBaseResponse?> SelectBaseAsync(string baseId, CancellationToken cancellationToken = default);

    Task<GetActiveBaseResponse?> GetActiveBaseAsync(CancellationToken cancellationToken = default);

    // Base Frame Calibration
    Task<StartBaseCalibrationResponse?> StartBaseCalibrationAsync(string method = "THREE_POINT", CancellationToken cancellationToken = default);

    Task<RecordBasePointResponse?> RecordBasePointAsync(int pointIndex, List<double> jointAngles, List<double> tcpPosition, CancellationToken cancellationToken = default);

    Task<FinishBaseCalibrationResponse?> FinishBaseCalibrationAsync(CancellationToken cancellationToken = default);

    Task<bool> CancelBaseCalibrationAsync(CancellationToken cancellationToken = default);

    Task<BaseCalibrationStatusResponse?> GetBaseCalibrationStatusAsync(CancellationToken cancellationToken = default);

    // Override Control operations
    Task<GetOverrideResponse?> GetOverrideAsync(CancellationToken cancellationToken = default);

    Task<SetOverrideResponse?> SetOverrideAsync(int programOverride = -1, int jogOverride = -1, int manualOverride = -1, CancellationToken cancellationToken = default);

    // Robot Package operations (Virtual Simulation)
    Task<GetRobotPackagesResponse?> GetRobotPackagesAsync(CancellationToken cancellationToken = default);

    Task<LoadRobotPackageResponse?> LoadRobotPackageAsync(string packageId, CancellationToken cancellationToken = default);

    // Program Execution operations (Virtual Simulation)
    Task<LoadProgramResponse?> LoadProgramAsync(string source, CancellationToken cancellationToken = default);

    Task<bool> RunProgramAsync(CancellationToken cancellationToken = default);

    Task<StepProgramResponse?> StepProgramAsync(CancellationToken cancellationToken = default);

    Task<bool> PauseProgramAsync(CancellationToken cancellationToken = default);

    Task<bool> StopProgramAsync(CancellationToken cancellationToken = default);

    Task<bool> ResetProgramAsync(CancellationToken cancellationToken = default);

    Task<GetProgramStateResponse?> GetProgramStateAsync(CancellationToken cancellationToken = default);

    Task<bool> SetPointAsync(string name, double[] values, CancellationToken cancellationToken = default);

    Task<GetPointsResponse?> GetPointsAsync(CancellationToken cancellationToken = default);
}
