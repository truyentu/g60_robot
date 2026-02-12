using System.Text.Json;
using Microsoft.Extensions.Logging;
using NetMQ;
using NetMQ.Sockets;
using RobotController.Common.Messages;

namespace RobotController.Common.Services;

public class IpcClientService : IIpcClientService
{
    private readonly ILogger<IpcClientService> _logger;
    private RequestSocket? _requestSocket;
    private SubscriberSocket? _subscriberSocket;
    private NetMQPoller? _poller;
    private Task? _pollerTask;
    private CancellationTokenSource? _pollerCts;
    private bool _isConnected;
    private bool _disposed;
    private readonly SemaphoreSlim _requestLock = new(1, 1);

    public bool IsConnected => _isConnected;

    public event EventHandler<StatusPayload>? StatusReceived;
    public event EventHandler<bool>? ConnectionStateChanged;
    public event EventHandler<string>? ErrorOccurred;
    public event EventHandler<RobotConfigChangedEvent>? RobotConfigChanged;
    public event EventHandler<HomingStateChangedEvent>? HomingStateChanged;
    public event EventHandler<ToolChangedEvent>? ToolChanged;
    public event EventHandler<OperationModeChangedEvent>? OperationModeChanged;
    public event EventHandler<BaseChangedEvent>? BaseChanged;
    public event EventHandler<OverrideChangedEvent>? OverrideChanged;

    public IpcClientService(ILogger<IpcClientService> logger)
    {
        _logger = logger;
    }

    public Task<bool> ConnectAsync(string requestAddress, string subscribeAddress, CancellationToken cancellationToken = default)
    {
        try
        {
            _logger.LogInformation("Connecting to Core: REQ={RequestAddr}, SUB={SubscribeAddr}", requestAddress, subscribeAddress);

            _requestSocket = new RequestSocket();
            _requestSocket.Options.Linger = TimeSpan.Zero;
            _requestSocket.Connect(requestAddress);

            _subscriberSocket = new SubscriberSocket();
            _subscriberSocket.Options.Linger = TimeSpan.Zero;
            _subscriberSocket.Connect(subscribeAddress);
            _subscriberSocket.SubscribeToAnyTopic();

            _pollerCts = new CancellationTokenSource();
            _poller = new NetMQPoller { _subscriberSocket };
            _subscriberSocket.ReceiveReady += OnSubscriberReceiveReady;

            _pollerTask = Task.Run(() =>
            {
                try
                {
                    _poller.Run();
                }
                catch (Exception ex)
                {
                    _logger.LogError(ex, "Poller error");
                }
            }, _pollerCts.Token);

            _isConnected = true;
            ConnectionStateChanged?.Invoke(this, true);
            _logger.LogInformation("Connected to Core successfully");

            return Task.FromResult(true);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Failed to connect to Core");
            return Task.FromResult(false);
        }
    }

    public void Disconnect()
    {
        if (!_isConnected)
            return;

        _logger.LogInformation("Disconnecting from Core...");

        try
        {
            _pollerCts?.Cancel();
            _poller?.Stop();

            if (_subscriberSocket != null)
            {
                _subscriberSocket.ReceiveReady -= OnSubscriberReceiveReady;
            }

            _requestSocket?.Dispose();
            _subscriberSocket?.Dispose();
            _poller?.Dispose();

            _requestSocket = null;
            _subscriberSocket = null;
            _poller = null;

            _isConnected = false;
            ConnectionStateChanged?.Invoke(this, false);
            _logger.LogInformation("Disconnected from Core");
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error during disconnect");
        }
    }

    public async Task<PongPayload?> PingAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.PING);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response?.Type == MessageTypes.PONG && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<PongPayload>();
        }

        return null;
    }

    public async Task<StatusPayload?> GetStatusAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_STATUS);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response?.Type == MessageTypes.STATUS && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<StatusPayload>();
        }

        return null;
    }

    public async Task<JointPositionsPayload?> GetJointPositionsAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_JOINT_POSITIONS);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response?.Type == MessageTypes.JOINT_POSITIONS && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<JointPositionsPayload>();
        }

        return null;
    }

    public async Task<bool> SendCommandAsync(string command, object? parameters = null, CancellationToken cancellationToken = default)
    {
        var payload = new CommandPayload
        {
            Command = command,
            Parameters = parameters != null ? JsonSerializer.SerializeToElement(parameters, IpcMessage.CamelCaseOptions) : default
        };

        var request = IpcMessage.Create(MessageTypes.COMMAND, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        return response?.Type == MessageTypes.COMMAND_ACK;
    }

    public async Task<GetRobotCatalogResponse?> GetRobotCatalogAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_ROBOT_CATALOG);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetRobotCatalogResponse>();
        }

        return null;
    }

    public async Task<SelectRobotModelResponse?> SelectRobotModelAsync(string modelId, CancellationToken cancellationToken = default)
    {
        var payload = new SelectRobotModelRequest { ModelId = modelId };
        var request = IpcMessage.Create(MessageTypes.SELECT_ROBOT_MODEL, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<SelectRobotModelResponse>();
        }

        return null;
    }

    public async Task<GetActiveRobotResponse?> GetActiveRobotAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_ACTIVE_ROBOT);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetActiveRobotResponse>();
        }

        return null;
    }

    public async Task<StartHomingResponse?> StartHomingAsync(int jointIndex = -1, string method = "LIMIT_SWITCH", CancellationToken cancellationToken = default)
    {
        var payload = new StartHomingRequest { JointIndex = jointIndex, Method = method };
        var request = IpcMessage.Create(MessageTypes.START_HOMING, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<StartHomingResponse>();
        }

        return null;
    }

    public async Task<StopHomingResponse?> StopHomingAsync(int jointIndex = -1, CancellationToken cancellationToken = default)
    {
        var payload = new StopHomingRequest { JointIndex = jointIndex };
        var request = IpcMessage.Create(MessageTypes.STOP_HOMING, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<StopHomingResponse>();
        }

        return null;
    }

    public async Task<HomingStateResponse?> GetHomingStateAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_HOMING_STATE);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<HomingStateResponse>();
        }

        return null;
    }

    // ========================================================================
    // Tool Management
    // ========================================================================

    public async Task<GetToolListResponse?> GetToolListAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_TOOL_LIST);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetToolListResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    public async Task<GetToolResponse?> GetToolAsync(string toolId, CancellationToken cancellationToken = default)
    {
        var payload = new GetToolRequest { ToolId = toolId };
        var request = IpcMessage.Create(MessageTypes.GET_TOOL, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetToolResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    public async Task<CreateToolResponse?> CreateToolAsync(ToolData tool, CancellationToken cancellationToken = default)
    {
        var payload = new CreateToolRequest { Tool = tool };
        var request = IpcMessage.Create(MessageTypes.CREATE_TOOL, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<CreateToolResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    public async Task<UpdateToolResponse?> UpdateToolAsync(string toolId, ToolData tool, CancellationToken cancellationToken = default)
    {
        var payload = new UpdateToolRequest { ToolId = toolId, Tool = tool };
        var request = IpcMessage.Create(MessageTypes.UPDATE_TOOL, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<UpdateToolResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    public async Task<DeleteToolResponse?> DeleteToolAsync(string toolId, CancellationToken cancellationToken = default)
    {
        var payload = new DeleteToolRequest { ToolId = toolId };
        var request = IpcMessage.Create(MessageTypes.DELETE_TOOL, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<DeleteToolResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    public async Task<SelectToolResponse?> SelectToolAsync(string toolId, CancellationToken cancellationToken = default)
    {
        var payload = new SelectToolRequest { ToolId = toolId };
        var request = IpcMessage.Create(MessageTypes.SELECT_TOOL, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<SelectToolResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    public async Task<GetActiveToolResponse?> GetActiveToolAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_ACTIVE_TOOL);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetActiveToolResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    // ========================================================================
    // Tool Calibration
    // ========================================================================

    public async Task<StartCalibrationResponse?> StartTcpCalibrationAsync(string method = "FOUR_POINT", CancellationToken cancellationToken = default)
    {
        var payload = new StartCalibrationRequest { Method = method };
        var request = IpcMessage.Create(MessageTypes.START_TCP_CALIBRATION, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<StartCalibrationResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    public async Task<RecordCalibrationPointResponse?> RecordCalibrationPointAsync(List<double> jointAngles, CancellationToken cancellationToken = default)
    {
        var payload = new RecordCalibrationPointRequest { JointAngles = jointAngles };
        var request = IpcMessage.Create(MessageTypes.RECORD_CALIBRATION_POINT, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<RecordCalibrationPointResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    public async Task<FinishCalibrationResponse?> FinishCalibrationAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.FINISH_CALIBRATION);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<FinishCalibrationResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    public async Task<bool> CancelCalibrationAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.CANCEL_CALIBRATION);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            var result = response.Payload.Deserialize<JsonElement>();
            return result.GetProperty("success").GetBoolean();
        }

        return false;
    }

    public async Task<CalibrationStatusResponse?> GetCalibrationStatusAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_CALIBRATION_STATUS);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<CalibrationStatusResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    // ========================================================================
    // Operation Mode
    // ========================================================================

    public async Task<GetOperationModeResponse?> GetOperationModeAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_OPERATION_MODE);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetOperationModeResponse>();
        }

        return null;
    }

    public async Task<SetOperationModeResponse?> SetOperationModeAsync(string mode, CancellationToken cancellationToken = default)
    {
        var payload = new SetOperationModeRequest { Mode = mode };
        var request = IpcMessage.Create(MessageTypes.SET_OPERATION_MODE, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<SetOperationModeResponse>();
        }

        return null;
    }

    public async Task<GetModeRequirementsResponse?> GetModeRequirementsAsync(string targetMode, CancellationToken cancellationToken = default)
    {
        var payload = new GetModeRequirementsRequest { TargetMode = targetMode };
        var request = IpcMessage.Create(MessageTypes.GET_MODE_REQUIREMENTS, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetModeRequirementsResponse>();
        }

        return null;
    }

    // ========================================================================
    // Base Frame Management
    // ========================================================================

    public async Task<GetBaseListResponse?> GetBaseListAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_BASE_LIST);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetBaseListResponse>();
        }

        return null;
    }

    public async Task<GetBaseResponse?> GetBaseAsync(string baseId, CancellationToken cancellationToken = default)
    {
        var payload = new GetBaseRequest { BaseId = baseId };
        var request = IpcMessage.Create(MessageTypes.GET_BASE, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetBaseResponse>();
        }

        return null;
    }

    public async Task<CreateBaseResponse?> CreateBaseAsync(BaseFrameData baseFrame, CancellationToken cancellationToken = default)
    {
        var payload = new CreateBaseRequest { Base = baseFrame };
        var request = IpcMessage.Create(MessageTypes.CREATE_BASE, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<CreateBaseResponse>();
        }

        return null;
    }

    public async Task<UpdateBaseResponse?> UpdateBaseAsync(string baseId, BaseFrameData baseFrame, CancellationToken cancellationToken = default)
    {
        var payload = new UpdateBaseRequest { BaseId = baseId, Base = baseFrame };
        var request = IpcMessage.Create(MessageTypes.UPDATE_BASE, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<UpdateBaseResponse>();
        }

        return null;
    }

    public async Task<DeleteBaseResponse?> DeleteBaseAsync(string baseId, CancellationToken cancellationToken = default)
    {
        var payload = new DeleteBaseRequest { BaseId = baseId };
        var request = IpcMessage.Create(MessageTypes.DELETE_BASE, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<DeleteBaseResponse>();
        }

        return null;
    }

    public async Task<SelectBaseResponse?> SelectBaseAsync(string baseId, CancellationToken cancellationToken = default)
    {
        var payload = new SelectBaseRequest { BaseId = baseId };
        var request = IpcMessage.Create(MessageTypes.SELECT_BASE, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<SelectBaseResponse>();
        }

        return null;
    }

    public async Task<GetActiveBaseResponse?> GetActiveBaseAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_ACTIVE_BASE);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetActiveBaseResponse>();
        }

        return null;
    }

    // ========================================================================
    // Base Frame Calibration
    // ========================================================================

    public async Task<StartBaseCalibrationResponse?> StartBaseCalibrationAsync(string method = "THREE_POINT", CancellationToken cancellationToken = default)
    {
        var payload = new StartBaseCalibrationRequest { Method = method };
        var request = IpcMessage.Create(MessageTypes.START_BASE_CALIBRATION, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<StartBaseCalibrationResponse>();
        }

        return null;
    }

    public async Task<RecordBasePointResponse?> RecordBasePointAsync(int pointIndex, List<double> jointAngles, List<double> tcpPosition, CancellationToken cancellationToken = default)
    {
        var payload = new RecordBasePointRequest { PointIndex = pointIndex, JointAngles = jointAngles, TcpPosition = tcpPosition };
        var request = IpcMessage.Create(MessageTypes.RECORD_BASE_POINT, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<RecordBasePointResponse>();
        }

        return null;
    }

    public async Task<FinishBaseCalibrationResponse?> FinishBaseCalibrationAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.FINISH_BASE_CALIBRATION);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<FinishBaseCalibrationResponse>();
        }

        return null;
    }

    public async Task<bool> CancelBaseCalibrationAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.CANCEL_BASE_CALIBRATION);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            var result = response.Payload.Deserialize<JsonElement>();
            return result.GetProperty("success").GetBoolean();
        }

        return false;
    }

    public async Task<BaseCalibrationStatusResponse?> GetBaseCalibrationStatusAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_BASE_CALIBRATION_STATUS);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<BaseCalibrationStatusResponse>();
        }

        return null;
    }

    // ========================================================================
    // Override Control
    // ========================================================================

    public async Task<GetOverrideResponse?> GetOverrideAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_OVERRIDE);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetOverrideResponse>();
        }

        return null;
    }

    public async Task<SetOverrideResponse?> SetOverrideAsync(int programOverride = -1, int jogOverride = -1, int manualOverride = -1, CancellationToken cancellationToken = default)
    {
        var payload = new SetOverrideRequest
        {
            ProgramOverride = programOverride,
            JogOverride = jogOverride,
            ManualOverride = manualOverride
        };
        var request = IpcMessage.Create(MessageTypes.SET_OVERRIDE, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));

        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<SetOverrideResponse>();
        }

        return null;
    }

    // ========================================================================
    // Robot Package (Virtual Simulation)
    // ========================================================================

    public async Task<GetRobotPackagesResponse?> GetRobotPackagesAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_ROBOT_PACKAGES);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetRobotPackagesResponse>();
        }

        return null;
    }

    public async Task<LoadRobotPackageResponse?> LoadRobotPackageAsync(string packageId, CancellationToken cancellationToken = default)
    {
        var payload = new LoadRobotPackageRequest { PackageId = packageId };
        var request = IpcMessage.Create(MessageTypes.LOAD_ROBOT_PACKAGE, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<LoadRobotPackageResponse>();
        }

        return null;
    }

    public async Task<ReloadPackagesResponse?> ReloadPackagesAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.RELOAD_PACKAGES);
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<ReloadPackagesResponse>();
        }

        return null;
    }

    // ========================================================================
    // Program Execution (Virtual Simulation)
    // ========================================================================

    public async Task<LoadProgramResponse?> LoadProgramAsync(string source, CancellationToken cancellationToken = default)
    {
        var payload = new { source };
        var request = IpcMessage.Create(MessageTypes.LOAD_PROGRAM, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<LoadProgramResponse>();
        }

        return null;
    }

    public async Task<bool> RunProgramAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.RUN_PROGRAM, JsonSerializer.SerializeToElement(new { }, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.GetProperty("success").GetBoolean();
        }

        return false;
    }

    public async Task<StepProgramResponse?> StepProgramAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.STEP_PROGRAM, JsonSerializer.SerializeToElement(new { }, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<StepProgramResponse>();
        }

        return null;
    }

    public async Task<bool> PauseProgramAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.PAUSE_PROGRAM, JsonSerializer.SerializeToElement(new { }, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.GetProperty("success").GetBoolean();
        }

        return false;
    }

    public async Task<bool> StopProgramAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.STOP_PROGRAM, JsonSerializer.SerializeToElement(new { }, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.GetProperty("success").GetBoolean();
        }

        return false;
    }

    public async Task<bool> ResetProgramAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.RESET_PROGRAM, JsonSerializer.SerializeToElement(new { }, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.GetProperty("success").GetBoolean();
        }

        return false;
    }

    public async Task<GetProgramStateResponse?> GetProgramStateAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_PROGRAM_STATE, JsonSerializer.SerializeToElement(new { }, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetProgramStateResponse>();
        }

        return null;
    }

    public async Task<bool> SetPointAsync(string name, double[] values, CancellationToken cancellationToken = default)
    {
        var payload = new { name, values };
        var request = IpcMessage.Create(MessageTypes.SET_POINT, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.GetProperty("success").GetBoolean();
        }

        return false;
    }

    public async Task<GetPointsResponse?> GetPointsAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.GET_POINTS, JsonSerializer.SerializeToElement(new { }, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GetPointsResponse>();
        }

        return null;
    }

    // ========================================================================
    // URDF Import Operations (Auto robot package creation)
    // ========================================================================

    public async Task<ParseUrdfResponse?> ParseUrdfAsync(string urdfContent, bool isFilePath = false, CancellationToken cancellationToken = default)
    {
        var payload = new ParseUrdfRequest
        {
            UrdfContent = urdfContent,
            IsFilePath = isFilePath
        };

        var request = IpcMessage.Create(MessageTypes.PARSE_URDF, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<ParseUrdfResponse>();
        }

        return null;
    }

    public async Task<GenerateRobotYamlResponse?> GenerateRobotYamlAsync(
        string urdfContent,
        bool isFilePath,
        string robotName,
        string manufacturer = "Unknown",
        string outputPath = "",
        CancellationToken cancellationToken = default)
    {
        var payload = new GenerateRobotYamlRequest
        {
            UrdfContent = urdfContent,
            IsFilePath = isFilePath,
            RobotName = robotName,
            Manufacturer = manufacturer,
            OutputPath = outputPath
        };

        var request = IpcMessage.Create(MessageTypes.GENERATE_ROBOT_YAML, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<GenerateRobotYamlResponse>();
        }

        return null;
    }

    // ========================================================================
    // Jog Control
    // ========================================================================

    public async Task<JogStartResponse?> StartJogModeAsync(bool enableDeadman = true, CancellationToken cancellationToken = default)
    {
        var payload = new JogStartRequest { EnableDeadman = enableDeadman };
        var request = IpcMessage.Create(MessageTypes.JOG_START, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<JogStartResponse>();
        }

        return null;
    }

    public async Task<JogStartResponse?> StopJogModeAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.JOG_STOP, JsonSerializer.SerializeToElement(new { }, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<JogStartResponse>();
        }

        return null;
    }

    public async Task<JogMoveResponse?> JogMoveAsync(JogMode mode, int axis, int direction, double speedPercent, JogFrame frame = JogFrame.World, CancellationToken cancellationToken = default)
    {
        var payload = new JogMoveRequest
        {
            Mode = (int)mode,
            Axis = axis,
            Direction = direction,
            SpeedPercent = speedPercent,
            Frame = (int)frame
        };
        var request = IpcMessage.Create(MessageTypes.JOG_MOVE, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<JogMoveResponse>();
        }

        return null;
    }

    public async Task<JogMoveResponse?> JogStepAsync(JogMode mode, int axis, int direction, double increment, double speedPercent, JogFrame frame = JogFrame.World, CancellationToken cancellationToken = default)
    {
        var payload = new JogStepRequest
        {
            Mode = (int)mode,
            Axis = axis,
            Direction = direction,
            Increment = increment,
            SpeedPercent = speedPercent,
            Frame = (int)frame
        };
        var request = IpcMessage.Create(MessageTypes.JOG_STEP, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<JogMoveResponse>();
        }

        return null;
    }

    // ========================================================================
    // Kinematics (3D Jogging)
    // ========================================================================

    public async Task<ComputeIKResponse?> ComputeIKAsync(double[] targetPose, double[] currentJoints, bool apply = false, CancellationToken cancellationToken = default)
    {
        var payload = new ComputeIKRequest
        {
            TargetPose = targetPose,
            CurrentJoints = currentJoints,
            Apply = apply
        };
        var request = IpcMessage.Create(MessageTypes.COMPUTE_IK, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<ComputeIKResponse>(IpcMessage.CamelCaseOptions);
        }

        return null;
    }

    public async Task<bool> SetJointsAsync(double[] jointsDegrees, CancellationToken cancellationToken = default)
    {
        var payload = new { joints = jointsDegrees };
        var request = IpcMessage.Create(MessageTypes.SET_JOINTS, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            var result = response.Payload.Deserialize<JsonElement>(IpcMessage.CamelCaseOptions);
            return result.TryGetProperty("success", out var s) && s.GetBoolean();
        }

        return false;
    }

    // ========================================================================
    // Firmware Control
    // ========================================================================

    public async Task<FirmwareConnectResponse?> ConnectFirmwareAsync(string port = "", int baudRate = 115200, CancellationToken cancellationToken = default)
    {
        var payload = new FirmwareConnectRequest { Port = port, BaudRate = baudRate };
        var request = IpcMessage.Create(MessageTypes.FIRMWARE_CONNECT, JsonSerializer.SerializeToElement(payload, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<FirmwareConnectResponse>();
        }

        return null;
    }

    public async Task<FirmwareConnectResponse?> DisconnectFirmwareAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.FIRMWARE_DISCONNECT, JsonSerializer.SerializeToElement(new { }, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<FirmwareConnectResponse>();
        }

        return null;
    }

    public async Task<FirmwareModeResponse?> GetFirmwareModeAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.FIRMWARE_GET_MODE, JsonSerializer.SerializeToElement(new { }, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<FirmwareModeResponse>();
        }

        return null;
    }

    public async Task<ScanPortsResponse?> ScanPortsAsync(CancellationToken cancellationToken = default)
    {
        var request = IpcMessage.Create(MessageTypes.FIRMWARE_SCAN_PORTS, JsonSerializer.SerializeToElement(new { }, IpcMessage.CamelCaseOptions));
        var response = await SendRequestAsync(request, cancellationToken);

        if (response != null && response.Payload.ValueKind != JsonValueKind.Undefined)
        {
            return response.Payload.Deserialize<ScanPortsResponse>();
        }

        return null;
    }

    private async Task<IpcMessage?> SendRequestAsync(IpcMessage request, CancellationToken cancellationToken)
    {
        if (!_isConnected || _requestSocket == null)
        {
            _logger.LogWarning("Cannot send request: not connected");
            return null;
        }

        await _requestLock.WaitAsync(cancellationToken);
        try
        {
            var json = request.Serialize();
            _logger.LogDebug("Sending: {Message}", json);

            if (!_requestSocket.TrySendFrame(TimeSpan.FromSeconds(5), json))
            {
                _logger.LogWarning("Send timeout");
                return null;
            }

            if (!_requestSocket.TryReceiveFrameString(TimeSpan.FromSeconds(5), out var responseJson))
            {
                _logger.LogWarning("Receive timeout");
                return null;
            }

            _logger.LogDebug("Received: {Message}", responseJson);
            return IpcMessage.Deserialize(responseJson);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error sending request");
            return null;
        }
        finally
        {
            _requestLock.Release();
        }
    }

    private void OnSubscriberReceiveReady(object? sender, NetMQSocketEventArgs e)
    {
        try
        {
            var json = e.Socket.ReceiveFrameString();
            _logger.LogTrace("SUB received: {Message}", json);

            var message = IpcMessage.Deserialize(json);
            if (message?.Payload.ValueKind != JsonValueKind.Undefined)
            {
                if (message.Type == MessageTypes.STATUS)
                {
                    var status = message.Payload.Deserialize<StatusPayload>();
                    if (status != null)
                    {
                        StatusReceived?.Invoke(this, status);
                    }
                }
                else if (message.Type == MessageTypes.ROBOT_CONFIG_CHANGED)
                {
                    var configChanged = message.Payload.Deserialize<RobotConfigChangedEvent>();
                    if (configChanged != null)
                    {
                        RobotConfigChanged?.Invoke(this, configChanged);
                    }
                }
                else if (message.Type == MessageTypes.HOMING_STATE_CHANGED)
                {
                    var homingChanged = message.Payload.Deserialize<HomingStateChangedEvent>();
                    if (homingChanged != null)
                    {
                        HomingStateChanged?.Invoke(this, homingChanged);
                    }
                }
                else if (message.Type == MessageTypes.TOOL_CHANGED)
                {
                    var toolChanged = message.Payload.Deserialize<ToolChangedEvent>(IpcMessage.CamelCaseOptions);
                    if (toolChanged != null)
                    {
                        ToolChanged?.Invoke(this, toolChanged);
                    }
                }
                else if (message.Type == MessageTypes.OPERATION_MODE_CHANGED)
                {
                    var modeChanged = message.Payload.Deserialize<OperationModeChangedEvent>();
                    if (modeChanged != null)
                    {
                        OperationModeChanged?.Invoke(this, modeChanged);
                    }
                }
                else if (message.Type == MessageTypes.BASE_CHANGED)
                {
                    var baseChanged = message.Payload.Deserialize<BaseChangedEvent>();
                    if (baseChanged != null)
                    {
                        BaseChanged?.Invoke(this, baseChanged);
                    }
                }
                else if (message.Type == MessageTypes.OVERRIDE_CHANGED)
                {
                    var overrideChanged = message.Payload.Deserialize<OverrideChangedEvent>();
                    if (overrideChanged != null)
                    {
                        OverrideChanged?.Invoke(this, overrideChanged);
                    }
                }
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error processing SUB message");
        }
    }

    public void Dispose()
    {
        if (_disposed)
            return;

        Disconnect();
        _pollerCts?.Dispose();
        _disposed = true;
    }
}
