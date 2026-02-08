using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using RobotController.UI.Models;
using RobotController.UI.Services;
using RobotController.UI.ViewModels.Pages;
using Serilog;
using System;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text.Json;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Threading;

namespace RobotController.UI.ViewModels;

public partial class MainViewModel : ObservableObject, IDisposable
{
    private readonly IIpcClientService _ipcClient;
    private readonly IConfigService _configService;
    private readonly IViewportService _viewportService;
    private readonly Dispatcher _dispatcher;
    private bool _disposed;
    private bool _robotInitialized;

    // Motion Control
    [ObservableProperty]
    private MotionControlViewModel? _motionControl;

    // Connection
    [ObservableProperty]
    private string _connectionStatus = "Disconnected";

    [ObservableProperty]
    private Brush _connectionStatusColor = Brushes.Red;

    [ObservableProperty]
    private bool _isConnected;

    // Robot Status
    [ObservableProperty]
    private string _robotState = "UNKNOWN";

    [ObservableProperty]
    private string _robotMode = "MANUAL";

    [ObservableProperty]
    private bool _isHomed;

    [ObservableProperty]
    private bool _isEnabled;

    // E-Stop State
    [ObservableProperty]
    private bool _isEStopActive;

    [ObservableProperty]
    private Brush _eStopButtonColor = Brushes.Red;

    [ObservableProperty]
    private string _eStopButtonText = "E-STOP";

    [ObservableProperty]
    private bool _showEStopConfirmDialog;

    // TCP Position
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

    // Core Info
    [ObservableProperty]
    private string _coreVersion = "";

    [ObservableProperty]
    private long _coreUptime;

    [ObservableProperty]
    private string _robotName = "";

    [ObservableProperty]
    private string _robotModel = "";

    // UI State
    [ObservableProperty]
    private int _selectedNavIndex = 0;

    [ObservableProperty]
    private double _speedOverride = 100;

    [ObservableProperty]
    private bool _showGrid = true;

    [ObservableProperty]
    private bool _showAxes = true;

    [ObservableProperty]
    private bool _showTcp = true;

    // Page ViewModels
    [ObservableProperty]
    private ProgramViewModel? _programViewModel;

    [ObservableProperty]
    private IOViewModel? _ioViewModel;

    [ObservableProperty]
    private ConfigurationViewModel? _configurationViewModel;

    [ObservableProperty]
    private DiagnosticsViewModel? _diagnosticsViewModel;

    [ObservableProperty]
    private RobotPackageBrowserViewModel? _robotCatalogViewModel;

    [ObservableProperty]
    private UrdfImportViewModel? _urdfImportViewModel;

    [ObservableProperty]
    private HomingViewModel? _homing;

    [ObservableProperty]
    private ToolViewModel? _tool;

    [ObservableProperty]
    private ModeViewModel? _mode;

    [ObservableProperty]
    private BaseFrameViewModel? _baseFrame;

    [ObservableProperty]
    private OverrideViewModel? _override;

    [ObservableProperty]
    private PositionDisplayViewModel? _positionDisplay;

    [ObservableProperty]
    private ProgramEditorViewModel? _programEditor;

    // 3D Model
    public Model3DGroup? RobotModelGroup => _viewportService.GetModelGroup();
    public Model3DGroup? TcpMarkerGroup { get; private set; }

    public event EventHandler? RobotModelUpdated;

    public ObservableCollection<JointPosition> JointPositions { get; } = new()
    {
        new JointPosition { Name = "J1", Value = 0.0 },
        new JointPosition { Name = "J2", Value = 0.0 },
        new JointPosition { Name = "J3", Value = 0.0 },
        new JointPosition { Name = "J4", Value = 0.0 },
        new JointPosition { Name = "J5", Value = 0.0 },
        new JointPosition { Name = "J6", Value = 0.0 },
    };

    /// <summary>
    /// Runtime constructor with DI
    /// </summary>
    public MainViewModel(
        IIpcClientService ipcClient,
        IConfigService configService,
        IViewportService viewportService,
        RobotPackageBrowserViewModel robotCatalogViewModel,
        UrdfImportViewModel urdfImportViewModel)
    {
        _ipcClient = ipcClient;
        _configService = configService;
        _viewportService = viewportService;
        _dispatcher = Dispatcher.CurrentDispatcher;

        // Subscribe to IPC events
        _ipcClient.StatusReceived += OnStatusReceived;
        _ipcClient.ConnectionStateChanged += OnConnectionStateChanged;
        _ipcClient.ErrorOccurred += OnErrorOccurred;

        // Subscribe to viewport events
        _viewportService.ModelUpdated += OnViewportModelUpdated;

        // Create TCP marker
        TcpMarkerGroup = _viewportService.CreateTcpMarker();

        // Initialize motion control
        MotionControl = new MotionControlViewModel(_ipcClient);

        // Initialize page view models
        ProgramViewModel = new ProgramViewModel();
        IoViewModel = new IOViewModel(_ipcClient);
        ConfigurationViewModel = new ConfigurationViewModel();
        DiagnosticsViewModel = new DiagnosticsViewModel(_ipcClient);
        RobotCatalogViewModel = robotCatalogViewModel;
        UrdfImportViewModel = urdfImportViewModel;
        Homing = new HomingViewModel(_ipcClient);
        Tool = new ToolViewModel(_ipcClient);
        Mode = new ModeViewModel(_ipcClient);
        BaseFrame = new BaseFrameViewModel(_ipcClient);
        Override = new OverrideViewModel(_ipcClient);
        PositionDisplay = new PositionDisplayViewModel(_ipcClient);
        ProgramEditor = new ProgramEditorViewModel(_ipcClient);

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
            Log.Information("Already connected");
            return;
        }

        ConnectionStatus = "Connecting...";
        ConnectionStatusColor = Brushes.Yellow;

        var connConfig = _configService.Config.Connection;
        Log.Information("Connecting to {Address}...", connConfig.RepAddress);

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

            // Initialize robot 3D model with default config (for demo without Core)
            await InitializeRobotModelWithDefaultAsync();

            // Get initial status
            var status = await _ipcClient.GetStatusAsync();
            if (status != null)
            {
                UpdateStatus(status);
            }
        }
    }

    private async Task InitializeRobotModelWithDefaultAsync()
    {
        if (_robotInitialized) return;

        try
        {
            // Create default robot config for demonstration
            var robotConfig = new RobotConfigData
            {
                Name = "Robot6DOF",
                Model = "6-Axis Industrial",
                DHParameters = new List<DHParameterData>
                {
                    new() { Joint = 1, A = 0, Alpha = -90, D = 400, ThetaOffset = 0 },
                    new() { Joint = 2, A = 560, Alpha = 0, D = 0, ThetaOffset = -90 },
                    new() { Joint = 3, A = 35, Alpha = -90, D = 0, ThetaOffset = 0 },
                    new() { Joint = 4, A = 0, Alpha = 90, D = 515, ThetaOffset = 0 },
                    new() { Joint = 5, A = 0, Alpha = -90, D = 0, ThetaOffset = 0 },
                    new() { Joint = 6, A = 0, Alpha = 0, D = 80, ThetaOffset = 0 },
                },
                HomePosition = new double[] { 0, -45, 90, 0, 45, 0 }
            };

            RobotName = robotConfig.Name;
            RobotModel = robotConfig.Model;

            // Initialize 3D model
            bool modelOk = await _viewportService.InitializeAsync(robotConfig);
            if (modelOk)
            {
                _robotInitialized = true;
                _dispatcher.Invoke(() =>
                {
                    OnPropertyChanged(nameof(RobotModelGroup));
                    RobotModelUpdated?.Invoke(this, EventArgs.Empty);
                });
                Log.Information("Robot 3D model initialized: {Name}", robotConfig.Name);
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to initialize robot model");
        }
    }

    [RelayCommand]
    private void Disconnect()
    {
        _ipcClient.Disconnect();
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
            IsConnected = isConnected;
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
        Log.Error("IPC Error: {Error}", error);
    }

    private void OnViewportModelUpdated(object? sender, EventArgs e)
    {
        _dispatcher.InvokeAsync(() =>
        {
            OnPropertyChanged(nameof(RobotModelGroup));
            RobotModelUpdated?.Invoke(this, EventArgs.Empty);
        });
    }

    private int _statusLogCounter = 0;

    private void UpdateStatus(StatusPayload status)
    {
        RobotState = status.State;
        RobotMode = status.Mode;
        IsHomed = status.Homed;
        IsEnabled = status.Enabled;

        // Update joint positions
        if (status.Joints.Count >= 6)
        {
            double[] angles = new double[6];
            for (int i = 0; i < 6; i++)
            {
                JointPositions[i].Value = status.Joints[i];
                angles[i] = status.Joints[i];
            }
            OnPropertyChanged(nameof(JointPositions));

            // Debug log: only log every 50th status or when any joint != 0
            _statusLogCounter++;
            bool anyNonZero = false;
            for (int i = 0; i < 6; i++) { if (Math.Abs(angles[i]) > 0.001) anyNonZero = true; }
            if (anyNonZero || _statusLogCounter % 50 == 0)
            {
                try
                {
                    var line = $"[{DateTime.Now:HH:mm:ss.fff}] [UI-STATUS] state={status.State} joints=[{string.Join(",", angles.Select(a => a.ToString("F2")))}] homed={status.Homed}";
                    System.IO.File.AppendAllText(
                        System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "jog_debug.log"),
                        line + Environment.NewLine);
                }
                catch { }
            }

            // Update 3D model with TCP pose from Core FK
            double[]? tcpPose = status.TcpPosition.Count >= 6
                ? status.TcpPosition.ToArray()
                : null;
            _viewportService.UpdateJointAngles(angles, tcpPose);
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

    [RelayCommand]
    private async Task TriggerEStopAsync()
    {
        if (IsEStopActive)
        {
            // Already in E-Stop, show reset confirmation
            ShowEStopConfirmDialog = true;
            return;
        }

        // Trigger E-Stop
        Log.Warning("E-STOP triggered by operator");
        IsEStopActive = true;
        EStopButtonColor = Brushes.DarkRed;
        EStopButtonText = "E-STOP ACTIVE";

        if (_ipcClient.IsConnected)
        {
            await _ipcClient.SendCommandAsync("e_stop", new { action = "trigger" });
        }
    }

    [RelayCommand]
    private async Task ResetEStopAsync()
    {
        ShowEStopConfirmDialog = false;

        Log.Information("E-STOP reset by operator");
        IsEStopActive = false;
        EStopButtonColor = Brushes.Red;
        EStopButtonText = "E-STOP";

        if (_ipcClient.IsConnected)
        {
            await _ipcClient.SendCommandAsync("e_stop", new { action = "reset" });
        }
    }

    [RelayCommand]
    private void CancelEStopReset()
    {
        ShowEStopConfirmDialog = false;
    }

    public void Dispose()
    {
        if (_disposed) return;

        _ipcClient.StatusReceived -= OnStatusReceived;
        _ipcClient.ConnectionStateChanged -= OnConnectionStateChanged;
        _ipcClient.ErrorOccurred -= OnErrorOccurred;
        _viewportService.ModelUpdated -= OnViewportModelUpdated;
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
