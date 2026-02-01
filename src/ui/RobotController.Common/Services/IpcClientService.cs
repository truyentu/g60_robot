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

    public bool IsConnected => _isConnected;

    public event EventHandler<StatusPayload>? StatusReceived;
    public event EventHandler<bool>? ConnectionStateChanged;
    public event EventHandler<string>? ErrorOccurred;

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
            Parameters = parameters != null ? JsonSerializer.SerializeToElement(parameters) : default
        };

        var request = IpcMessage.Create(MessageTypes.COMMAND, JsonSerializer.SerializeToElement(payload));
        var response = await SendRequestAsync(request, cancellationToken);

        return response?.Type == MessageTypes.COMMAND_ACK;
    }

    private Task<IpcMessage?> SendRequestAsync(IpcMessage request, CancellationToken cancellationToken)
    {
        if (!_isConnected || _requestSocket == null)
        {
            _logger.LogWarning("Cannot send request: not connected");
            return Task.FromResult<IpcMessage?>(null);
        }

        try
        {
            var json = request.Serialize();
            _logger.LogDebug("Sending: {Message}", json);

            if (!_requestSocket.TrySendFrame(TimeSpan.FromSeconds(5), json))
            {
                _logger.LogWarning("Send timeout");
                return Task.FromResult<IpcMessage?>(null);
            }

            if (!_requestSocket.TryReceiveFrameString(TimeSpan.FromSeconds(5), out var responseJson))
            {
                _logger.LogWarning("Receive timeout");
                return Task.FromResult<IpcMessage?>(null);
            }

            _logger.LogDebug("Received: {Message}", responseJson);
            return Task.FromResult(IpcMessage.Deserialize(responseJson));
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error sending request");
            return Task.FromResult<IpcMessage?>(null);
        }
    }

    private void OnSubscriberReceiveReady(object? sender, NetMQSocketEventArgs e)
    {
        try
        {
            var json = e.Socket.ReceiveFrameString();
            _logger.LogTrace("SUB received: {Message}", json);

            var message = IpcMessage.Deserialize(json);
            if (message?.Type == MessageTypes.STATUS && message.Payload.ValueKind != JsonValueKind.Undefined)
            {
                var status = message.Payload.Deserialize<StatusPayload>();
                if (status != null)
                {
                    StatusReceived?.Invoke(this, status);
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
