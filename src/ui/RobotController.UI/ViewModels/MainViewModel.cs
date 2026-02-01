using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Microsoft.Extensions.Logging;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using System.Collections.ObjectModel;
using System.Windows.Media;

namespace RobotController.UI.ViewModels;

public partial class MainViewModel : ObservableObject
{
    private readonly ILogger<MainViewModel> _logger;
    private readonly IIpcClientService _ipcClient;

    [ObservableProperty]
    private string _connectionStatus = "Disconnected";

    [ObservableProperty]
    private Brush _connectionStatusColor = Brushes.Red;

    [ObservableProperty]
    private string _robotState = "IDLE";

    [ObservableProperty]
    private string _robotMode = "MANUAL";

    [ObservableProperty]
    private bool _isHomed;

    [ObservableProperty]
    private bool _isEnabled;

    [ObservableProperty]
    private double _tcpX = 500.0;

    [ObservableProperty]
    private double _tcpY = 0.0;

    [ObservableProperty]
    private double _tcpZ = 600.0;

    [ObservableProperty]
    private double _tcpRx = 0.0;

    [ObservableProperty]
    private double _tcpRy = 180.0;

    [ObservableProperty]
    private double _tcpRz = 0.0;

    [ObservableProperty]
    private bool _isConnected;

    public ObservableCollection<JointPosition> JointPositions { get; } = new()
    {
        new JointPosition { Name = "J1", Value = 0.0 },
        new JointPosition { Name = "J2", Value = -45.0 },
        new JointPosition { Name = "J3", Value = 90.0 },
        new JointPosition { Name = "J4", Value = 0.0 },
        new JointPosition { Name = "J5", Value = 45.0 },
        new JointPosition { Name = "J6", Value = 0.0 },
    };

    public MainViewModel(ILogger<MainViewModel> logger, IIpcClientService ipcClient)
    {
        _logger = logger;
        _ipcClient = ipcClient;

        _ipcClient.StatusReceived += OnStatusReceived;
        _ipcClient.ConnectionStateChanged += OnConnectionStateChanged;
    }

    [RelayCommand]
    private async Task ConnectAsync()
    {
        if (IsConnected)
        {
            _ipcClient.Disconnect();
            return;
        }

        _logger.LogInformation("Connecting to Core...");
        ConnectionStatus = "Connecting...";
        ConnectionStatusColor = Brushes.Yellow;

        var success = await _ipcClient.ConnectAsync("tcp://localhost:5555", "tcp://localhost:5556");

        if (success)
        {
            var pong = await _ipcClient.PingAsync();
            if (pong != null)
            {
                _logger.LogInformation("Core version: {Version}, Uptime: {Uptime}ms",
                    pong.CoreVersion, pong.UptimeMs);
            }
        }
    }

    [RelayCommand]
    private async Task RefreshStatusAsync()
    {
        if (!IsConnected)
            return;

        var status = await _ipcClient.GetStatusAsync();
        if (status != null)
        {
            UpdateFromStatus(status);
        }
    }

    private void OnStatusReceived(object? sender, StatusPayload status)
    {
        System.Windows.Application.Current?.Dispatcher.Invoke(() =>
        {
            UpdateFromStatus(status);
        });
    }

    private void OnConnectionStateChanged(object? sender, bool connected)
    {
        System.Windows.Application.Current?.Dispatcher.Invoke(() =>
        {
            IsConnected = connected;
            if (connected)
            {
                ConnectionStatus = "Connected";
                ConnectionStatusColor = Brushes.Green;
            }
            else
            {
                ConnectionStatus = "Disconnected";
                ConnectionStatusColor = Brushes.Red;
            }
        });
    }

    private void UpdateFromStatus(StatusPayload status)
    {
        RobotState = status.State;
        RobotMode = status.Mode;
        IsHomed = status.Homed;
        IsEnabled = status.Enabled;

        if (status.TcpPosition?.Count >= 6)
        {
            TcpX = status.TcpPosition[0];
            TcpY = status.TcpPosition[1];
            TcpZ = status.TcpPosition[2];
            TcpRx = status.TcpPosition[3];
            TcpRy = status.TcpPosition[4];
            TcpRz = status.TcpPosition[5];
        }

        if (status.Joints?.Count >= 6)
        {
            for (int i = 0; i < 6; i++)
            {
                JointPositions[i].Value = status.Joints[i];
            }
        }
    }
}

public class JointPosition : ObservableObject
{
    private string _name = string.Empty;
    private double _value;

    public string Name
    {
        get => _name;
        set => SetProperty(ref _name, value);
    }

    public double Value
    {
        get => _value;
        set => SetProperty(ref _value, value);
    }
}
