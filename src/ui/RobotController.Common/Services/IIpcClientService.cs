using RobotController.Common.Messages;

namespace RobotController.Common.Services;

public interface IIpcClientService : IDisposable
{
    bool IsConnected { get; }

    event EventHandler<StatusPayload>? StatusReceived;
    event EventHandler<bool>? ConnectionStateChanged;
    event EventHandler<string>? ErrorOccurred;

    Task<bool> ConnectAsync(string requestAddress, string subscribeAddress, CancellationToken cancellationToken = default);

    void Disconnect();

    Task<PongPayload?> PingAsync(CancellationToken cancellationToken = default);

    Task<StatusPayload?> GetStatusAsync(CancellationToken cancellationToken = default);

    Task<JointPositionsPayload?> GetJointPositionsAsync(CancellationToken cancellationToken = default);

    Task<bool> SendCommandAsync(string command, object? parameters = null, CancellationToken cancellationToken = default);
}
