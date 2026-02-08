using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using System;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Input;

namespace RobotController.UI.ViewModels;

/// <summary>
/// Main view model for motion control panel
/// </summary>
public partial class MotionControlViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;

    // ========================================================================
    // Observable Properties
    // ========================================================================

    // Joint positions
    public ObservableCollection<JointPositionViewModel> Joints { get; } = new();

    // Cartesian position
    [ObservableProperty]
    private CartesianPositionViewModel _tcpPosition = new();

    // Machine state
    [ObservableProperty]
    private string _machineState = "Disconnected";

    [ObservableProperty]
    private string _machineStateColor = "#808080";

    [ObservableProperty]
    private bool _isConnected;

    [ObservableProperty]
    private bool _isHomed;

    [ObservableProperty]
    private bool _isMoving;

    [ObservableProperty]
    private bool _isPaused;

    [ObservableProperty]
    private bool _hasAlarm;

    [ObservableProperty]
    private string _alarmMessage = "";

    // Jog settings
    [ObservableProperty]
    private double _jogSpeed = 50;  // Percentage

    [ObservableProperty]
    private double _jogIncrement = 1.0;  // Degrees or mm

    [ObservableProperty]
    private bool _jogContinuous = true;

    [ObservableProperty]
    private bool _jogJointMode = true;  // true=Joint, false=Cartesian

    [ObservableProperty]
    private bool _isJogEnabled;  // Jog mode enabled via IPC

    [ObservableProperty]
    private int _selectedJointIndex = 0;

    [ObservableProperty]
    private int _selectedCartesianAxis = 0;  // 0=X, 1=Y, 2=Z, 3=Rx, 4=Ry, 5=Rz

    // Cartesian axes for selection
    public ObservableCollection<CartesianAxisViewModel> CartesianAxes { get; } = new()
    {
        new CartesianAxisViewModel { Index = 0, Name = "X", IsLinear = true, IsSelected = true },
        new CartesianAxisViewModel { Index = 1, Name = "Y", IsLinear = true },
        new CartesianAxisViewModel { Index = 2, Name = "Z", IsLinear = true },
        new CartesianAxisViewModel { Index = 3, Name = "Rx", IsLinear = false },
        new CartesianAxisViewModel { Index = 4, Name = "Ry", IsLinear = false },
        new CartesianAxisViewModel { Index = 5, Name = "Rz", IsLinear = false },
    };

    // Feed override
    [ObservableProperty]
    private int _feedOverride = 100;

    [ObservableProperty]
    private int _rapidOverride = 100;

    // Trajectory progress
    [ObservableProperty]
    private double _trajectoryProgress;

    [ObservableProperty]
    private string _trajectoryState = "Idle";

    // Coordinate system
    [ObservableProperty]
    private string _coordinateSystem = "World";

    public ObservableCollection<string> CoordinateSystems { get; } = new()
    {
        "World", "Base", "Tool", "User1", "User2"
    };

    // Jog increments
    public ObservableCollection<double> JogIncrements { get; } = new()
    {
        0.1, 0.5, 1.0, 5.0, 10.0, 45.0, 90.0
    };

    // ========================================================================
    // Constructor
    // ========================================================================

    public MotionControlViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;

        // Initialize 6 joints
        string[] jointNames = { "J1 Base", "J2 Shoulder", "J3 Elbow",
                                "J4 Wrist1", "J5 Wrist2", "J6 Tool" };
        for (int i = 0; i < 6; i++)
        {
            Joints.Add(new JointPositionViewModel(i, jointNames[i]));
        }

        // Subscribe to events
        _ipcClient.StatusReceived += OnStatusReceived;
        _ipcClient.ConnectionStateChanged += OnConnectionChanged;
    }

    // ========================================================================
    // Commands - Jogging
    // ========================================================================

    private static readonly string _jogDebugLogPath = Path.Combine(
        AppDomain.CurrentDomain.BaseDirectory, "jog_debug.log");

    private static void JogLog(string message)
    {
        try
        {
            var line = $"[{DateTime.Now:HH:mm:ss.fff}] [UI] {message}";
            File.AppendAllText(_jogDebugLogPath, line + Environment.NewLine);
            System.Diagnostics.Debug.WriteLine(line);
        }
        catch { }
    }

    [RelayCommand]
    private async Task JogStartAsync(string direction)
    {
        JogLog($"JogStartAsync called: direction={direction}, IsConnected={IsConnected}, IsJogEnabled={IsJogEnabled}, JogContinuous={JogContinuous}, JogJointMode={JogJointMode}, SelectedJointIndex={SelectedJointIndex}, SelectedCartesianAxis={SelectedCartesianAxis}, JogIncrement={JogIncrement}, JogSpeed={JogSpeed}, IsHomed={IsHomed}, CoordinateSystem={CoordinateSystem}");

        if (!IsConnected)
        {
            JogLog("ABORT: Not connected");
            return;
        }

        // Auto-enable jog mode if not already enabled
        if (!IsJogEnabled)
        {
            JogLog("Calling StartJogModeAsync...");
            var enableResult = await _ipcClient.StartJogModeAsync();
            JogLog($"StartJogModeAsync result: Success={enableResult?.Success}, Error={enableResult?.Error}");
            if (enableResult?.Success != true)
            {
                JogLog($"ABORT: Failed to enable jog: {enableResult?.Error}");
                return;
            }
            IsJogEnabled = true;
        }

        int dir = direction == "+" ? 1 : -1;
        var jogFrame = CoordinateSystemToJogFrame(CoordinateSystem);

        if (JogContinuous)
        {
            // Continuous jog via dedicated IPC
            var mode = JogJointMode ? JogMode.Joint : JogMode.Cartesian;
            int axis = JogJointMode ? SelectedJointIndex : SelectedCartesianAxis;

            JogLog($"Calling JogMoveAsync: mode={mode}, axis={axis}, dir={dir}, speed={JogSpeed}, frame={jogFrame}");
            var response = await _ipcClient.JogMoveAsync(mode, axis, dir, JogSpeed, jogFrame);
            JogLog($"JogMoveAsync result: Success={response?.Success}, Error={response?.Error}");
            if (response?.Success != true)
            {
                JogLog($"JogMove FAILED: {response?.Error}");
            }
        }
        else
        {
            // Incremental jog
            var mode = JogJointMode ? JogMode.Joint : JogMode.Cartesian;
            int axis = JogJointMode ? SelectedJointIndex : SelectedCartesianAxis;

            JogLog($"Calling JogStepAsync: mode={mode}, axis={axis}, dir={dir}, increment={JogIncrement}, speed={JogSpeed}, frame={jogFrame}");
            var response = await _ipcClient.JogStepAsync(mode, axis, dir, JogIncrement, JogSpeed, jogFrame);
            JogLog($"JogStepAsync result: Success={response?.Success}, Error={response?.Error}");
            if (response?.Success != true)
            {
                JogLog($"JogStep FAILED: {response?.Error}");
            }
        }
        IsMoving = true;
        JogLog("JogStartAsync completed, IsMoving=true");
    }

    [RelayCommand]
    private async Task JogStopAsync()
    {
        if (!IsConnected) return;

        // Stop current jog motion (jog mode stays enabled on Core side)
        await _ipcClient.StopJogModeAsync();
        IsMoving = false;
    }

    [RelayCommand]
    private async Task JogIncrementAsync(string direction)
    {
        if (!IsConnected) return;

        // Auto-enable jog mode if not already enabled
        if (!IsJogEnabled)
        {
            var enableResult = await _ipcClient.StartJogModeAsync();
            if (enableResult?.Success != true) return;
            IsJogEnabled = true;
        }

        int dir = direction == "+" ? 1 : -1;
        var mode = JogJointMode ? JogMode.Joint : JogMode.Cartesian;
        int axis = JogJointMode ? SelectedJointIndex : SelectedCartesianAxis;
        var jogFrame = CoordinateSystemToJogFrame(CoordinateSystem);

        var response = await _ipcClient.JogStepAsync(mode, axis, dir, JogIncrement, JogSpeed, jogFrame);
        if (response?.Success != true)
        {
            System.Diagnostics.Debug.WriteLine($"Jog increment failed: {response?.Error}");
        }
    }

    // ========================================================================
    // Commands - Motion Control
    // ========================================================================

    [RelayCommand]
    private async Task StopMotionAsync()
    {
        if (!IsConnected) return;

        await _ipcClient.SendCommandAsync("stop", new { });
        IsMoving = false;
        IsPaused = false;
    }

    [RelayCommand]
    private async Task PauseMotionAsync()
    {
        if (!IsConnected || !IsMoving) return;

        await _ipcClient.SendCommandAsync("pause", new { });
        IsPaused = true;
    }

    [RelayCommand]
    private async Task ResumeMotionAsync()
    {
        if (!IsConnected || !IsPaused) return;

        await _ipcClient.SendCommandAsync("resume", new { });
        IsPaused = false;
    }

    [RelayCommand]
    private async Task ClearAlarmAsync()
    {
        if (!IsConnected) return;

        await _ipcClient.SendCommandAsync("clear_alarm", new { });
        HasAlarm = false;
        AlarmMessage = "";
        MachineState = "Idle";
        MachineStateColor = "#00AA00";
    }

    // ========================================================================
    // Commands - Homing
    // ========================================================================

    [RelayCommand]
    private async Task HomeAllAsync()
    {
        if (!IsConnected) return;

        MachineState = "Homing...";
        MachineStateColor = "#AAAA00";

        await _ipcClient.SendCommandAsync("home_all", new { });
    }

    [RelayCommand]
    private async Task HomeAxisAsync(int axis)
    {
        if (!IsConnected || axis < 0 || axis >= 6) return;

        await _ipcClient.SendCommandAsync("home_axis", new { Axis = axis });
    }

    // ========================================================================
    // Commands - Feed Override
    // ========================================================================

    [RelayCommand]
    private async Task SetFeedOverrideAsync(int percent)
    {
        FeedOverride = Math.Clamp(percent, 10, 200);
        await _ipcClient.SendCommandAsync("set_feed_override", new { Percent = FeedOverride });
    }

    [RelayCommand]
    private void IncreaseFeedOverride()
    {
        FeedOverride = Math.Min(200, FeedOverride + 10);
        _ = _ipcClient.SendCommandAsync("set_feed_override", new { Percent = FeedOverride });
    }

    [RelayCommand]
    private void DecreaseFeedOverride()
    {
        FeedOverride = Math.Max(10, FeedOverride - 10);
        _ = _ipcClient.SendCommandAsync("set_feed_override", new { Percent = FeedOverride });
    }

    [RelayCommand]
    private void ResetFeedOverride()
    {
        FeedOverride = 100;
        _ = _ipcClient.SendCommandAsync("set_feed_override", new { Percent = 100 });
    }

    // ========================================================================
    // Commands - Coordinate System
    // ========================================================================

    private static JogFrame CoordinateSystemToJogFrame(string coordinateSystem)
    {
        return coordinateSystem switch
        {
            "Base" => JogFrame.Base,
            "Tool" => JogFrame.Tool,
            "User1" => JogFrame.User1,
            "User2" => JogFrame.User2,
            _ => JogFrame.World,
        };
    }

    [RelayCommand]
    private void ToggleJogMode()
    {
        JogJointMode = !JogJointMode;
    }

    [RelayCommand]
    private void SelectJoint(object? parameter)
    {
        if (parameter is int index || (parameter is string s && int.TryParse(s, out index)))
        {
            if (index >= 0 && index < 6)
            {
                foreach (var joint in Joints)
                {
                    joint.IsSelected = false;
                }
                Joints[index].IsSelected = true;
                SelectedJointIndex = index;
            }
        }
    }

    [RelayCommand]
    private void SelectCartesianAxis(object? parameter)
    {
        if (parameter is int index || (parameter is string s && int.TryParse(s, out index)))
        {
            if (index >= 0 && index < 6)
            {
                foreach (var axis in CartesianAxes)
                {
                    axis.IsSelected = false;
                }
                CartesianAxes[index].IsSelected = true;
                SelectedCartesianAxis = index;
            }
        }
    }

    // ========================================================================
    // Event Handlers
    // ========================================================================

    private void OnStatusReceived(object? sender, StatusPayload status)
    {
        App.Current?.Dispatcher.Invoke(() =>
        {
            // Update joint positions
            for (int i = 0; i < 6 && i < status.Joints.Count; i++)
            {
                Joints[i].UpdatePosition(status.Joints[i], 0);
                Joints[i].IsHomed = status.Homed;
            }

            // Update machine state
            UpdateMachineState(status.State);

            // Update flags
            IsHomed = status.Homed;
            IsMoving = status.State == "RUN" || status.State == "JOG";
            IsPaused = status.State == "HOLD";

            // Check alarm
            if (status.State == "ALARM")
            {
                HasAlarm = true;
                AlarmMessage = "ALARM Active";
            }

            // Update TCP position
            if (status.TcpPosition.Count >= 6)
            {
                TcpPosition.Update(
                    status.TcpPosition[0],
                    status.TcpPosition[1],
                    status.TcpPosition[2],
                    status.TcpPosition[3],
                    status.TcpPosition[4],
                    status.TcpPosition[5]
                );
            }
        });
    }

    private void UpdateMachineState(string state)
    {
        switch (state.ToUpper())
        {
            case "IDLE":
                MachineState = "Idle";
                MachineStateColor = "#00AA00";
                break;
            case "RUN":
                MachineState = "Running";
                MachineStateColor = "#00FF00";
                break;
            case "HOLD":
                MachineState = "Paused";
                MachineStateColor = "#FFAA00";
                break;
            case "JOG":
                MachineState = "Jogging";
                MachineStateColor = "#00AAFF";
                break;
            case "HOMING":
                MachineState = "Homing";
                MachineStateColor = "#AAAA00";
                break;
            case "ALARM":
                MachineState = "ALARM";
                MachineStateColor = "#FF0000";
                break;
            default:
                MachineState = state;
                MachineStateColor = "#808080";
                break;
        }
    }

    private void OnConnectionChanged(object? sender, bool connected)
    {
        App.Current?.Dispatcher.Invoke(() =>
        {
            IsConnected = connected;
            if (!connected)
            {
                MachineState = "Disconnected";
                MachineStateColor = "#808080";
            }
        });
    }

    // ========================================================================
    // Keyboard Shortcuts
    // ========================================================================

    public void HandleKeyDown(Key key)
    {
        switch (key)
        {
            // Jog keys (NumPad or Arrow keys)
            case Key.NumPad4:
            case Key.Left:
                _ = JogStartAsync("-");
                break;
            case Key.NumPad6:
            case Key.Right:
                _ = JogStartAsync("+");
                break;

            // Joint selection (1-6 keys)
            case Key.D1:
            case Key.NumPad1:
                SelectJoint(0);
                break;
            case Key.D2:
            case Key.NumPad2:
                SelectJoint(1);
                break;
            case Key.D3:
            case Key.NumPad3:
                SelectJoint(2);
                break;
            case Key.D4:
                SelectJoint(3);
                break;
            case Key.D5:
                SelectJoint(4);
                break;
            case Key.D6:
                SelectJoint(5);
                break;

            // Motion control
            case Key.Space:
                if (IsMoving && !IsPaused)
                    _ = PauseMotionAsync();
                else if (IsPaused)
                    _ = ResumeMotionAsync();
                break;
            case Key.Escape:
                _ = StopMotionAsync();
                break;

            // Feed override
            case Key.Add:
            case Key.OemPlus:
                IncreaseFeedOverride();
                break;
            case Key.Subtract:
            case Key.OemMinus:
                DecreaseFeedOverride();
                break;
        }
    }

    public void HandleKeyUp(Key key)
    {
        // Stop jog on key release (for continuous mode)
        if (JogContinuous)
        {
            switch (key)
            {
                case Key.NumPad4:
                case Key.NumPad6:
                case Key.Left:
                case Key.Right:
                    _ = JogStopAsync();
                    break;
            }
        }
    }
}

// ========================================================================
// Command DTOs
// ========================================================================

public class JogCommand
{
    public int Axis { get; set; }
    public int Direction { get; set; }
    public double Speed { get; set; }
    public bool Continuous { get; set; }
}

public class JogIncrementCommand
{
    public int Axis { get; set; }
    public double TargetPosition { get; set; }
    public double Speed { get; set; }
}

public class CartesianJogCommand
{
    public int Axis { get; set; }  // 0=X, 1=Y, 2=Z, 3=Rx, 4=Ry, 5=Rz
    public int Direction { get; set; }
    public double Speed { get; set; }
    public bool Continuous { get; set; }
    public string CoordinateSystem { get; set; } = "World";
}

public class CartesianJogIncrementCommand
{
    public int Axis { get; set; }
    public double TargetValue { get; set; }
    public double Speed { get; set; }
    public string CoordinateSystem { get; set; } = "World";
}

public partial class CartesianAxisViewModel : ObservableObject
{
    public int Index { get; set; }
    public string Name { get; set; } = "";
    public bool IsLinear { get; set; }  // true = mm, false = degrees

    [ObservableProperty]
    private bool _isSelected;
}
