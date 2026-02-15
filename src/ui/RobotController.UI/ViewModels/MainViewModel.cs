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
using System.Windows;
using System.Windows.Input;
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

    /// <summary>Get IPC client for direct access (used by MainWindow for gizmo IK calls)</summary>
    public IIpcClientService GetIpcClient() => _ipcClient;

    // ====== KUKA SmartHMI Shell Properties ======

    // Navigation Drawer
    [ObservableProperty]
    private bool _isNavDrawerOpen = true;

    // Status Bar - Program Info
    [ObservableProperty]
    private string _currentProgramName = "";

    // Status Bar - Tool/Base Display
    [ObservableProperty]
    private string _activeToolDisplay = "[0]";

    [ObservableProperty]
    private string _activeBaseDisplay = "[0]";

    // Status Bar - Interpreter State
    [ObservableProperty]
    private string _interpreterState = "---";

    // Status Bar - KUKA Interpreter Indicators (KSS 8.3, Section 4.2.2)
    // Submit Interpreter (S): Green=running, Red=stopped, Gray=deselected
    [ObservableProperty]
    private Brush _submitInterpreterColor = Brushes.Gray;

    // Drives Status: Symbol I=ON, O=OFF; Color Green=can start, Gray=cannot
    [ObservableProperty]
    private Brush _drivesStatusColor = Brushes.Gray;

    [ObservableProperty]
    private string _drivesStatusSymbol = "O";

    // Robot Interpreter (R): Gray=none, Yellow=ready, Green=running, Red=stopped, Black=end
    [ObservableProperty]
    private Brush _robotInterpreterColor = Brushes.Gray;

    // Program Run Mode: #GO, #MSTEP, #ISTEP, #BSTEP, #PSTEP, #CSTEP
    [ObservableProperty]
    private string _programRunMode = "#GO";

    // Incremental Jogging: ∞=Continuous, 100mm, 10mm, 1mm, 0.1mm
    [ObservableProperty]
    private string _incrementalJogMode = "∞";

    // Message Window
    [ObservableProperty]
    private string _currentMessage = "No active messages";

    [ObservableProperty]
    private string _messageTimestamp = "";

    // Softkeys (7 context-sensitive buttons)
    [ObservableProperty]
    private string _softkey1Text = "Connect";

    [ObservableProperty]
    private string _softkey2Text = "Disconnect";

    [ObservableProperty]
    private string _softkey3Text = "Refresh";

    [ObservableProperty]
    private string _softkey4Text = "Home";

    [ObservableProperty]
    private string _softkey5Text = "Run";

    [ObservableProperty]
    private string _softkey6Text = "Stop";

    [ObservableProperty]
    private string _softkey7Text = "";

    [ObservableProperty]
    private Visibility _softkey7Visible = Visibility.Collapsed;

    // Dynamic softkey commands (swapped based on nav context)
    [ObservableProperty]
    private ICommand? _softkey1Command;

    [ObservableProperty]
    private ICommand? _softkey2Command;

    [ObservableProperty]
    private ICommand? _softkey3Command;

    [ObservableProperty]
    private ICommand? _softkey4Command;

    [ObservableProperty]
    private ICommand? _softkey5Command;

    [ObservableProperty]
    private ICommand? _softkey6Command;

    // Navigator dialog state
    [ObservableProperty]
    private bool _showNewProgramDialog;

    [ObservableProperty]
    private string _newProgramName = "";

    [ObservableProperty]
    private string _newProgramDescription = "";

    [ObservableProperty]
    private bool _showDeleteConfirmDialog;

    [ObservableProperty]
    private string _deleteTargetName = "";

    [ObservableProperty]
    private bool _showRenameDialog;

    [ObservableProperty]
    private string _renameProgramName = "";

    // STM32 Connection Dialog
    [ObservableProperty]
    private bool _showStm32ConnectDialog;

    [ObservableProperty]
    private string _stm32IpAddress = "127.0.0.1";

    [ObservableProperty]
    private int _stm32Port = 5001;

    [ObservableProperty]
    private bool _isStm32Connected;

    [ObservableProperty]
    private string _stm32StatusText = "Disconnected";

    // 3D Jog Mode
    [ObservableProperty]
    private bool _is3DJogEnabled;

    [ObservableProperty]
    private bool _isGizmoWorldFrame = true;

    [ObservableProperty]
    private bool _is3DJogLiveMode;

    [ObservableProperty]
    private string _gizmoDragStatus = "";

    partial void OnIs3DJogEnabledChanged(bool value)
    {
        _viewportService.ShowTcpGizmo(value && SelectedNavIndex == 0);
    }

    partial void OnIsGizmoWorldFrameChanged(bool value)
    {
        _viewportService.SetGizmoFrame(value ? GizmoFrame.World : GizmoFrame.Tool);
    }

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

    private int _prePickNavIndex = -1; // Nav index before TCP picking

    [ObservableProperty]
    private double _speedOverride = 100;

    [ObservableProperty]
    private bool _showGrid = true;

    [ObservableProperty]
    private bool _showAxes = true;

    [ObservableProperty]
    private bool _showTcp = true;

    [ObservableProperty]
    private bool _showTcpTrace;

    partial void OnShowTcpTraceChanged(bool value)
    {
        _viewportService.SetTcpTraceEnabled(value);
    }

    // Page ViewModels
    [ObservableProperty]
    private ProgramViewModel? _programViewModel;

    /// <summary>Convenience alias for code-behind access</summary>
    public ProgramViewModel? ProgramVm => ProgramViewModel;

    [ObservableProperty]
    private IOViewModel? _ioViewModel;

    [ObservableProperty]
    private ConfigurationViewModel? _configurationViewModel;

    [ObservableProperty]
    private DiagnosticsViewModel? _diagnosticsViewModel;

    [ObservableProperty]
    private bool _showUdpMonitor;

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
    private StationSetupViewModel? _stationSetup;

    [ObservableProperty]
    private PositionDisplayViewModel? _positionDisplay;

    [ObservableProperty]
    private ProgramEditorViewModel? _programEditor;

    // 3D Model
    public Model3DGroup? RobotModelGroup => _viewportService.GetModelGroup();
    public Model3DGroup? TcpMarkerGroup { get; private set; }
    public Model3DGroup? SceneObjectsGroup => _viewportService.GetSceneGroup();
    public Model3DGroup? BaseFrameAxesGroup => _viewportService.GetBaseFrameGroup();

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
        _viewportService.TcpPickModeChanged += OnTcpPickModeChanged;

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
        Tool = new ToolViewModel(_ipcClient, _viewportService);
        Mode = new ModeViewModel(_ipcClient);
        BaseFrame = new BaseFrameViewModel(_ipcClient);
        Override = new OverrideViewModel(_ipcClient);
        StationSetup = new StationSetupViewModel(_viewportService, _ipcClient);
        PositionDisplay = new PositionDisplayViewModel(_ipcClient);
        ProgramEditor = new ProgramEditorViewModel(_ipcClient, _viewportService);

        // Subscribe to tool change for viewport update
        _ipcClient.ToolChanged += OnToolChangedForViewport;

        // Initialize context-sensitive softkeys
        InitializeSoftkeys();

        // Auto-connect if configured
        if (_configService.Config.Connection.AutoConnect)
        {
            _ = ConnectAsync();
        }
    }

    // ====== KUKA Shell Commands ======

    [RelayCommand]
    private void ClosePage()
    {
        SelectedNavIndex = 0;
    }

    [RelayCommand]
    private void ToggleNavDrawer()
    {
        IsNavDrawerOpen = !IsNavDrawerOpen;
    }

    [RelayCommand]
    private void ToggleUdpMonitor()
    {
        ShowUdpMonitor = !ShowUdpMonitor;
    }

    [RelayCommand]
    private void AcknowledgeMessage()
    {
        CurrentMessage = "No active messages";
        MessageTimestamp = "";
    }

    // Softkey commands initialized after constructor
    private void InitializeSoftkeys()
    {
        UpdateSoftkeysForContext(SelectedNavIndex);
    }

    partial void OnSelectedNavIndexChanged(int value)
    {
        UpdateSoftkeysForContext(value);

        // Hide TCP gizmo when not on Teach & Program
        _viewportService.ShowTcpGizmo(Is3DJogEnabled && value == 0);
    }

    private void UpdateSoftkeysForContext(int navIndex)
    {
        if (navIndex == 0) // Teach & Program → Navigator mode
        {
            Softkey1Text = "New";
            Softkey2Text = "Select";
            Softkey3Text = "Duplicate";
            Softkey4Text = "Archive";
            Softkey5Text = "Delete";
            Softkey6Text = "Open";
            Softkey7Text = "Edit";
            Softkey7Visible = Visibility.Visible;

            Softkey1Command = new RelayCommand(NavNew);
            Softkey2Command = new AsyncRelayCommand(NavSelectAsync);
            Softkey3Command = new RelayCommand(NavDuplicate);
            Softkey4Command = new AsyncRelayCommand(NavArchiveAsync);
            Softkey5Command = new RelayCommand(NavDelete);
            Softkey6Command = new RelayCommand(NavOpen);
        }
        else // Default mode
        {
            Softkey1Text = "Connect";
            Softkey2Text = "Disconnect";
            Softkey3Text = "Refresh";
            Softkey4Text = "Home";
            Softkey5Text = "Run";
            Softkey6Text = "Stop";
            Softkey7Text = "";
            Softkey7Visible = Visibility.Collapsed;

            Softkey1Command = new AsyncRelayCommand(ConnectAsync);
            Softkey2Command = new RelayCommand(Disconnect);
            Softkey3Command = new AsyncRelayCommand(RefreshStatusAsync);
            Softkey4Command = new RelayCommand(() => { /* Home placeholder */ });
            Softkey5Command = new RelayCommand(() => { /* Run placeholder */ });
            Softkey6Command = new RelayCommand(() => { /* Stop placeholder */ });
        }
    }

    // ====== Navigator Softkey Actions ======

    private void NavNew()
    {
        NewProgramName = "";
        NewProgramDescription = "";
        ShowNewProgramDialog = true;
    }

    [RelayCommand]
    private void ConfirmNewProgram()
    {
        ShowNewProgramDialog = false;
        var name = string.IsNullOrWhiteSpace(NewProgramName)
            ? $"NewProgram{(ProgramViewModel?.Programs.Count ?? 0) + 1}"
            : NewProgramName.Trim();
        ProgramViewModel?.CreateProgramWithName(name, NewProgramDescription.Trim());
    }

    [RelayCommand]
    private void CancelNewProgram()
    {
        ShowNewProgramDialog = false;
    }

    private async Task NavSelectAsync()
    {
        if (ProgramViewModel?.SelectedProgram == null) return;

        var program = ProgramViewModel.SelectedProgram;
        CurrentProgramName = program.Name;

        if (_ipcClient.IsConnected && !string.IsNullOrEmpty(ProgramViewModel.ProgramCode))
        {
            await _ipcClient.LoadProgramAsync(ProgramViewModel.ProgramCode);
        }

        ProgramViewModel.ProgramStatus = "Selected";
        Log.Information("Program selected: {Name}", program.Name);
    }

    private void NavDuplicate()
    {
        ProgramViewModel?.DuplicateSelectedProgram();
    }

    private async Task NavArchiveAsync()
    {
        if (ProgramViewModel == null) return;
        await ProgramViewModel.ArchiveProgramsAsync();
    }

    private void NavDelete()
    {
        if (ProgramViewModel?.SelectedProgram == null) return;
        DeleteTargetName = ProgramViewModel.SelectedProgram.Name;
        ShowDeleteConfirmDialog = true;
    }

    [RelayCommand]
    private void ConfirmDelete()
    {
        ShowDeleteConfirmDialog = false;
        ProgramViewModel?.DeleteProgram();
        Log.Information("Program deleted: {Name}", DeleteTargetName);
    }

    [RelayCommand]
    private void CancelDelete()
    {
        ShowDeleteConfirmDialog = false;
    }

    private void NavOpen()
    {
        if (ProgramViewModel?.SelectedProgram == null) return;

        // Load the selected program code into the editor and switch to editor view
        if (ProgramEditor != null && !string.IsNullOrEmpty(ProgramViewModel.ProgramCode))
        {
            ProgramEditor.ProgramSource = ProgramViewModel.ProgramCode;
            ProgramEditor.ProgramName = ProgramViewModel.SelectedProgram.Name;
        }
        Log.Information("Program opened: {Name}", ProgramViewModel.SelectedProgram.Name);
    }

    // Rename dialog
    [RelayCommand]
    private void ShowRenameProgramDialog()
    {
        if (ProgramViewModel?.SelectedProgram == null) return;
        RenameProgramName = ProgramViewModel.SelectedProgram.Name;
        ShowRenameDialog = true;
    }

    [RelayCommand]
    private void ConfirmRename()
    {
        ShowRenameDialog = false;
        if (!string.IsNullOrWhiteSpace(RenameProgramName))
        {
            ProgramViewModel?.RenameSelectedProgram(RenameProgramName.Trim());
        }
    }

    [RelayCommand]
    private void CancelRename()
    {
        ShowRenameDialog = false;
    }

    // ====== STM32 Connection Commands ======

    [RelayCommand]
    private void ShowStm32Dialog()
    {
        ShowStm32ConnectDialog = true;
    }

    [RelayCommand]
    private void CancelStm32Connect()
    {
        ShowStm32ConnectDialog = false;
    }

    [RelayCommand]
    private async Task ConfirmStm32ConnectAsync()
    {
        ShowStm32ConnectDialog = false;
        Stm32StatusText = "Connecting...";

        var result = await _ipcClient.ConnectStm32Async(Stm32IpAddress, Stm32Port);
        if (result != null && result.Success)
        {
            IsStm32Connected = true;
            Stm32StatusText = $"STM32: {Stm32IpAddress}:{Stm32Port}";
            // Reset jog state so next jog press re-sends JOG_START to new driver
            if (MotionControl != null)
                MotionControl.IsJogEnabled = false;
            Log.Information("STM32 connected: {Ip}:{Port} driver={Driver}", result.Ip, result.Port, result.DriverName);
        }
        else
        {
            IsStm32Connected = false;
            Stm32StatusText = $"Failed: {result?.Error ?? "No response"}";
            Log.Warning("STM32 connect failed: {Error}", result?.Error ?? "No response");
        }
    }

    [RelayCommand]
    private async Task DisconnectStm32Async()
    {
        var result = await _ipcClient.DisconnectStm32Async();
        IsStm32Connected = false;
        Stm32StatusText = "Disconnected";
        // Reset jog state so next jog press re-sends JOG_START to sim driver
        if (MotionControl != null)
            MotionControl.IsJogEnabled = false;
        Log.Information("STM32 disconnected");
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

            // Auto-restore last session (robot package + tool)
            bool restored = await AutoRestoreSessionAsync();

            if (!restored)
            {
                // Fall back to default robot model (for demo without Core)
                await InitializeRobotModelWithDefaultAsync();
            }

            // Get initial status
            var status = await _ipcClient.GetStatusAsync();
            if (status != null)
            {
                UpdateStatus(status);
            }

            // Auto-load active tool mesh
            await AutoLoadActiveToolMeshAsync();
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

    /// <summary>
    /// Auto-restore last active robot package from config.
    /// Returns true if package was successfully loaded.
    /// </summary>
    private async Task<bool> AutoRestoreSessionAsync()
    {
        try
        {
            var lastPackageId = _configService.Config.LastActivePackageId;
            if (string.IsNullOrEmpty(lastPackageId))
            {
                Log.Information("[MainVM] No last active package in config, skipping auto-restore");
                return false;
            }

            Log.Information("[MainVM] Auto-restoring last active package: {PackageId}", lastPackageId);

            var response = await _ipcClient.LoadRobotPackageAsync(lastPackageId);
            if (response == null || !response.Success || response.Package == null)
            {
                Log.Warning("[MainVM] Failed to auto-load package '{PackageId}': {Error}",
                    lastPackageId, response?.Error ?? "null response");
                return false;
            }

            var pkg = response.Package;

            // Build package path from ID
            var packagePath = System.IO.Path.GetFullPath(
                System.IO.Path.Combine(
                    AppDomain.CurrentDomain.BaseDirectory,
                    "config", "robots", pkg.Id));

            var packageData = new RobotPackageData
            {
                Name = pkg.Name,
                Id = pkg.Id,
                Manufacturer = pkg.Manufacturer,
                ModelType = pkg.ModelType,
                PayloadKg = pkg.PayloadKg,
                ReachMm = pkg.ReachMm,
                DhConvention = pkg.DhConvention,
                PackagePath = packagePath
            };

            // Parse joints
            foreach (var j in pkg.Joints)
            {
                var joint = new JointDefinitionData
                {
                    Name = j.Name,
                    Type = j.Type,
                    DhA = j.DhA,
                    DhAlpha = j.DhAlpha,
                    DhD = j.DhD,
                    DhThetaOffset = j.DhThetaOffset,
                    OriginXyz = j.OriginXyz,
                    OriginRpy = j.OriginRpy,
                    Axis = j.Axis,
                    LimitMin = j.LimitMin,
                    LimitMax = j.LimitMax,
                    VelocityMax = j.VelocityMax,
                    AccelerationMax = j.AccelerationMax
                };

                if (j.Mesh != null)
                {
                    joint.Mesh = new JointMeshData
                    {
                        VisualMesh = j.Mesh.VisualMesh,
                        CollisionMesh = j.Mesh.CollisionMesh
                    };
                }

                packageData.Joints.Add(joint);
            }

            packageData.BaseMesh = pkg.BaseMesh ?? "";
            packageData.HomePosition = pkg.HomePosition;
            packageData.FlangeOffset = pkg.FlangeOffset ?? new double[3];

            // Update viewport
            await _viewportService.InitializeFromPackageAsync(packageData);
            _robotInitialized = true;

            // Update RobotCatalogViewModel if available
            if (RobotCatalogViewModel != null)
            {
                RobotCatalogViewModel.LoadedPackage = packageData;
            }

            Log.Information("[MainVM] Auto-restored robot package: {Name} ({Id})", pkg.Name, pkg.Id);
            return true;
        }
        catch (Exception ex)
        {
            Log.Error(ex, "[MainVM] Error during auto-restore session");
            return false;
        }
    }

    /// <summary>
    /// Auto-load active tool mesh on startup
    /// </summary>
    private async Task AutoLoadActiveToolMeshAsync()
    {
        try
        {
            var response = await _ipcClient.GetActiveToolAsync();
            if (response?.Success != true || response.Tool == null)
            {
                Log.Debug("[MainVM] No active tool to auto-load");
                return;
            }

            var toolData = response.Tool;
            Log.Information("[MainVM] Auto-loading active tool mesh: {Name}, MeshPath='{Path}'",
                toolData.Name, toolData.VisualMeshPath);

            _dispatcher.Invoke(() =>
            {
                // Set tool TCP offset
                var tcpOffset = new[] {
                    toolData.Tcp.X, toolData.Tcp.Y, toolData.Tcp.Z,
                    toolData.Tcp.Rx, toolData.Tcp.Ry, toolData.Tcp.Rz
                };
                _viewportService.SetToolTcpOffset(tcpOffset);

                if (!string.IsNullOrEmpty(toolData.VisualMeshPath) &&
                    System.IO.File.Exists(toolData.VisualMeshPath))
                {
                    _viewportService.SetToolMesh(toolData.VisualMeshPath, toolData.MeshOffset, toolData.MeshScale);
                    Log.Information("[MainVM] Auto-loaded tool mesh: {Path}", toolData.VisualMeshPath);
                }
            });
        }
        catch (Exception ex)
        {
            Log.Warning(ex, "[MainVM] Error auto-loading active tool mesh");
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

    private void OnTcpPickModeChanged(object? sender, bool isPickMode)
    {
        _dispatcher.InvokeAsync(() =>
        {
            if (isPickMode)
            {
                // Save current page and navigate to viewport (index 0 = Manual Jog)
                _prePickNavIndex = SelectedNavIndex;
                SelectedNavIndex = 0;
                Log.Information("[MainVM] TCP pick mode ON - navigated to viewport (was page {Prev})", _prePickNavIndex);
            }
            else
            {
                // Navigate back to Tool Management page
                if (_prePickNavIndex >= 0)
                {
                    SelectedNavIndex = _prePickNavIndex;
                    Log.Information("[MainVM] TCP pick mode OFF - returned to page {Page}", _prePickNavIndex);
                    _prePickNavIndex = -1;
                }
            }
        });
    }

    private async void OnToolChangedForViewport(object? sender, ToolChangedEvent e)
    {
        try
        {
            Log.Information("[MainVM] OnToolChangedForViewport called: ToolId={ToolId}, ToolName={ToolName}", e.ToolId, e.ToolName);

            // Small delay to avoid REQ socket race condition:
            // TOOL_CHANGED event may arrive before the SELECT_TOOL response is received,
            // and ZeroMQ REQ socket requires strict send-recv-send-recv pattern.
            await Task.Delay(100);

            // Fetch the full tool data to get visualMeshPath
            var response = await Task.Run(() => _ipcClient.GetActiveToolAsync());
            if (response?.Success != true || response.Tool == null)
            {
                Log.Warning("[MainVM] GetActiveToolAsync failed: Success={Success}, Tool={Tool}",
                    response?.Success, response?.Tool != null ? "exists" : "null");
                _dispatcher.Invoke(() =>
                {
                    _viewportService.SetToolTcpOffset(new double[6]);
                    _viewportService.ClearToolMesh();
                });
                return;
            }

            var toolData = response.Tool;
            Log.Information("[MainVM] Active tool data: Id={Id}, Name={Name}, VisualMeshPath='{Path}', MeshOffset=[{Offset}]",
                toolData.Id, toolData.Name, toolData.VisualMeshPath,
                toolData.MeshOffset != null ? string.Join(",", toolData.MeshOffset) : "null");

            _dispatcher.Invoke(() =>
            {
                // Set tool TCP offset for TCP marker positioning
                var tcpOffset = new[] {
                    toolData.Tcp.X, toolData.Tcp.Y, toolData.Tcp.Z,
                    toolData.Tcp.Rx, toolData.Tcp.Ry, toolData.Tcp.Rz
                };
                _viewportService.SetToolTcpOffset(tcpOffset);

                if (!string.IsNullOrEmpty(toolData.VisualMeshPath))
                {
                    bool fileExists = System.IO.File.Exists(toolData.VisualMeshPath);
                    Log.Information("[MainVM] VisualMeshPath='{Path}', File.Exists={Exists}", toolData.VisualMeshPath, fileExists);

                    if (fileExists)
                    {
                        bool loaded = _viewportService.SetToolMesh(toolData.VisualMeshPath, toolData.MeshOffset, toolData.MeshScale);
                        Log.Information("[MainVM] SetToolMesh result: {Result}", loaded);
                    }
                    else
                    {
                        _viewportService.ClearToolMesh();
                        Log.Warning("[MainVM] Tool mesh file not found: {Path}", toolData.VisualMeshPath);
                    }
                }
                else
                {
                    _viewportService.ClearToolMesh();
                    Log.Debug("[MainVM] Tool '{ToolId}' has no visual mesh (empty path), cleared viewport", e.ToolId);
                }
            });
        }
        catch (Exception ex)
        {
            Log.Warning(ex, "[MainVM] Error updating tool mesh in viewport");
        }
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
        _viewportService.TcpPickModeChanged -= OnTcpPickModeChanged;
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
