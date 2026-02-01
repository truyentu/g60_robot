using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Microsoft.Extensions.Logging;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using RobotController.UI.Services;
using System.Collections.ObjectModel;
using System.Windows.Media;
using System.Windows.Threading;

namespace RobotController.UI.ViewModels;

public partial class MainViewModel : ObservableObject, IDisposable
{
    private readonly ILogger<MainViewModel> _logger;
    private readonly IIpcClientService _ipcClient;
    private readonly IConfigService _configService;
    private readonly Dispatcher _dispatcher;
    private bool _disposed;

    [ObservableProperty]
    private string _connectionStatus = "Disconnected";

    [ObservableProperty]
    private Brush _connectionStatusColor = Brushes.Red;

    [ObservableProperty]
    private string _robotState = "UNKNOWN";

    [ObservableProperty]
    private string _robotMode = "MANUAL";

    [ObservableProperty]
    private bool _isHomed;

    [ObservableProperty]
    private bool _isEnabled;

    [ObservableProperty]
    private double _tcpX;

    [ObservableProperty]
    private double _tcpY;

    [ObservableProperty]
    private double _tcpZ;

    [ObservableProperty]
    private double _tcpRx;

    [ObservableProperty]
    private double _tcpRy;

    [ObservableProperty]
    private double _tcpRz;

    [ObservableProperty]
    private string _coreVersion = "";

    [ObservableProperty]
    private long _coreUptime;

    public ObservableCollection<JointPosition> JointPositions { get; } = new()
    {
        new JointPosition { Name = "J1", Value = 0.0 },
        new JointPosition { Name = "J2", Value = 0.0 },
        new JointPosition { Name = "J3", Value = 0.0 },
        new JointPosition { Name = "J4", Value = 0.0 },
        new JointPosition { Name = "J5", Value = 0.0 },
        new JointPosition { Name = "J6", Value = 0.0 },
    };

    public MainViewModel(
        ILogger<MainViewModel> logger,
        IIpcClientService ipcClient,
        IConfigService configService)
    {
        _logger = logger;
        _ipcClient = ipcClient;
        _configService = configService;
        _dispatcher = Dispatcher.CurrentDispatcher;

        // Subscribe to events
        _ipcClient.StatusReceived += OnStatusReceived;
        _ipcClient.ConnectionStateChanged += OnConnectionStateChanged;
        _ipcClient.ErrorOccurred += OnErrorOccurred;

        // Auto-connect if configured
        if (_configService.Config.Connection.AutoConnect)
        {
            _ = ConnectAsync();
        }
    }

    [RelayCommand]
    private async Task ConnectAsync()
    {
        if (_ipcClient.IsConnected)
        {
            _ipcClient.Disconnect();
            return;
        }

        ConnectionStatus = "Connecting...";
        ConnectionStatusColor = Brushes.Yellow;

        var connConfig = _configService.Config.Connection;
        _logger.LogInformation("Connecting to {Address}...", connConfig.RepAddress);

        bool success = await _ipcClient.ConnectAsync(
            connConfig.RepAddress,
            connConfig.SubAddress
        );

        if (success)
        {
            // Get version info
            var pong = await _ipcClient.PingAsync();
            if (pong != null)
            {
                CoreVersion = pong.CoreVersion;
                CoreUptime = pong.UptimeMs;
            }

            // Get initial status
            var status = await _ipcClient.GetStatusAsync();
            if (status != null)
            {
                UpdateStatus(status);
            }
        }
    }

    [RelayCommand]
    private async Task RefreshStatusAsync()
    {
        if (!_ipcClient.IsConnected)
        {
            return;
        }

        var status = await _ipcClient.GetStatusAsync();
        if (status != null)
        {
            UpdateStatus(status);
        }
    }

    private void OnStatusReceived(object? sender, StatusPayload status)
    {
        _dispatcher.InvokeAsync(() => UpdateStatus(status));
    }

    private void OnConnectionStateChanged(object? sender, bool isConnected)
    {
        _dispatcher.InvokeAsync(() =>
        {
            if (isConnected)
            {
                ConnectionStatus = "Connected";
                ConnectionStatusColor = Brushes.LimeGreen;
            }
            else
            {
                ConnectionStatus = "Disconnected";
                ConnectionStatusColor = Brushes.Red;
                RobotState = "UNKNOWN";
            }
        });
    }

    private void OnErrorOccurred(object? sender, string error)
    {
        _logger.LogError("IPC Error: {Error}", error);
    }

    private void UpdateStatus(StatusPayload status)
    {
        RobotState = status.State;
        RobotMode = status.Mode;
        IsHomed = status.Homed;
        IsEnabled = status.Enabled;

        // Update joint positions
        if (status.Joints.Count >= 6)
        {
            for (int i = 0; i < 6; i++)
            {
                JointPositions[i].Value = status.Joints[i];
            }
            OnPropertyChanged(nameof(JointPositions));
        }

        // Update TCP position
        if (status.TcpPosition.Count >= 6)
        {
            TcpX = status.TcpPosition[0];
            TcpY = status.TcpPosition[1];
            TcpZ = status.TcpPosition[2];
            TcpRx = status.TcpPosition[3];
            TcpRy = status.TcpPosition[4];
            TcpRz = status.TcpPosition[5];
        }
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _ipcClient.StatusReceived -= OnStatusReceived;
        _ipcClient.ConnectionStateChanged -= OnConnectionStateChanged;
        _ipcClient.ErrorOccurred -= OnErrorOccurred;
        _ipcClient.Dispose();

        _disposed = true;
        GC.SuppressFinalize(this);
    }
}

public partial class JointPosition : ObservableObject
{
    [ObservableProperty]
    private string _name = string.Empty;

    [ObservableProperty]
    private double _value;
}
