using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using System;
using System.Collections.ObjectModel;
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
    private int _selectedJointIndex = 0;

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

    [RelayCommand]
    private async Task JogStartAsync(string direction)
    {
        if (!IsConnected || !IsHomed) return;

        int axis = SelectedJointIndex;
        int dir = direction == "+" ? 1 : -1;
        double speed = JogSpeed / 100.0 * 180.0;  // Max 180 deg/s

        // Send jog command via IPC
        var cmd = new JogCommand
        {
            Axis = axis,
            Direction = dir,
            Speed = speed,
            Continuous = JogContinuous
        };
        await _ipcClient.SendCommandAsync("jog_start", cmd);
        IsMoving = true;
    }

    [RelayCommand]
    private async Task JogStopAsync()
    {
        if (!IsConnected) return;

        await _ipcClient.SendCommandAsync("jog_stop", new { });
        IsMoving = false;
    }

    [RelayCommand]
    private async Task JogIncrementAsync(string direction)
    {
        if (!IsConnected || !IsHomed) return;

        int axis = SelectedJointIndex;
        int dir = direction == "+" ? 1 : -1;
        double distance = JogIncrement * dir;

        // Get current position and add increment
        double currentPos = Joints[axis].Position;
        double targetPos = currentPos + distance;

        // Clamp to limits
        targetPos = Math.Clamp(targetPos, Joints[axis].MinLimit, Joints[axis].MaxLimit);

        var cmd = new JogIncrementCommand
        {
            Axis = axis,
            TargetPosition = targetPos,
            Speed = JogSpeed / 100.0
        };
        await _ipcClient.SendCommandAsync("jog_increment", cmd);
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
