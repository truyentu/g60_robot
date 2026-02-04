# IMPL_P2_05: Motion HMI

| Metadata | Value |
|----------|-------|
| Plan ID | IMPL_P2_05 |
| Phase | 2 - Motion Core |
| Priority | P1 |
| Depends On | IMPL_P2_04 (Firmware Communication) |
| Status | Ready for Implementation |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/Thiết kế HMI Robot KUKA WPF.md` | MVVM pattern, jog panel layout, industrial UI |

---

## Overview

Implementation plan cho Motion HMI - giao diện điều khiển chuyển động robot:
- **Jog Panel:** Điều khiển jogging từng trục với keyboard/mouse
- **Position Display:** Hiển thị vị trí real-time (Joint & Cartesian)
- **Motion Controls:** Start, Stop, Pause, Resume, E-Stop
- **Feed Override:** Điều chỉnh tốc độ chuyển động
- **Homing Panel:** Điều khiển homing từng trục hoặc tất cả
- **Status Indicators:** Trạng thái máy, alarms, limits

---

## Prerequisites

- [ ] IMPL_P2_04 (Firmware Communication) đã hoàn thành
- [ ] IMPL_P1_04 (HMI & 3D Visualization) đã hoàn thành
- [ ] WPF project builds successfully
- [ ] IPC communication working

---

## Step 1: Create Motion View Models

### 1.1 Create JointPositionViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/JointPositionViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using System;

namespace RobotController.UI.ViewModels;

/// <summary>
/// View model for a single joint position display and control
/// </summary>
public partial class JointPositionViewModel : ObservableObject
{
    [ObservableProperty]
    private int _index;

    [ObservableProperty]
    private string _name = "";

    [ObservableProperty]
    private double _position;  // Current position in degrees

    [ObservableProperty]
    private double _velocity;  // Current velocity in deg/s

    [ObservableProperty]
    private double _targetPosition;

    [ObservableProperty]
    private double _minLimit;

    [ObservableProperty]
    private double _maxLimit;

    [ObservableProperty]
    private bool _isHomed;

    [ObservableProperty]
    private bool _isAtMinLimit;

    [ObservableProperty]
    private bool _isAtMaxLimit;

    [ObservableProperty]
    private bool _isMoving;

    [ObservableProperty]
    private bool _isSelected;

    // Computed properties
    public double PositionPercent =>
        (MaxLimit - MinLimit) > 0
            ? (Position - MinLimit) / (MaxLimit - MinLimit) * 100
            : 50;

    public string PositionDisplay => $"{Position:F2}°";
    public string VelocityDisplay => $"{Velocity:F1}°/s";
    public string LimitsDisplay => $"[{MinLimit:F0}° ... {MaxLimit:F0}°]";

    public JointPositionViewModel(int index, string name)
    {
        Index = index;
        Name = name;
        MinLimit = -180;
        MaxLimit = 180;
    }

    public void UpdatePosition(double position, double velocity = 0)
    {
        Position = position;
        Velocity = velocity;
        IsMoving = Math.Abs(velocity) > 0.1;

        OnPropertyChanged(nameof(PositionPercent));
        OnPropertyChanged(nameof(PositionDisplay));
        OnPropertyChanged(nameof(VelocityDisplay));
    }

    public void UpdateLimits(double min, double max)
    {
        MinLimit = min;
        MaxLimit = max;
        OnPropertyChanged(nameof(LimitsDisplay));
        OnPropertyChanged(nameof(PositionPercent));
    }
}
```

### 1.2 Create CartesianPositionViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/CartesianPositionViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;

namespace RobotController.UI.ViewModels;

/// <summary>
/// View model for Cartesian (TCP) position display
/// </summary>
public partial class CartesianPositionViewModel : ObservableObject
{
    // Position (mm)
    [ObservableProperty]
    private double _x;

    [ObservableProperty]
    private double _y;

    [ObservableProperty]
    private double _z;

    // Orientation (degrees)
    [ObservableProperty]
    private double _roll;

    [ObservableProperty]
    private double _pitch;

    [ObservableProperty]
    private double _yaw;

    // Velocity
    [ObservableProperty]
    private double _linearVelocity;  // mm/s

    [ObservableProperty]
    private double _angularVelocity; // deg/s

    // Display strings
    public string PositionDisplay => $"X:{X:F2} Y:{Y:F2} Z:{Z:F2}";
    public string OrientationDisplay => $"R:{Roll:F1}° P:{Pitch:F1}° Y:{Yaw:F1}°";
    public string VelocityDisplay => $"{LinearVelocity:F1} mm/s";

    public void Update(double x, double y, double z, double roll, double pitch, double yaw)
    {
        X = x;
        Y = y;
        Z = z;
        Roll = roll;
        Pitch = pitch;
        Yaw = yaw;

        OnPropertyChanged(nameof(PositionDisplay));
        OnPropertyChanged(nameof(OrientationDisplay));
    }

    public void UpdateVelocity(double linear, double angular)
    {
        LinearVelocity = linear;
        AngularVelocity = angular;
        OnPropertyChanged(nameof(VelocityDisplay));
    }
}
```

### 1.3 Create MotionControlViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/MotionControlViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Core.IPC;
using RobotController.Core.Services;
using System;
using System.Collections.ObjectModel;
using System.Threading.Tasks;
using System.Windows.Input;

namespace RobotController.UI.ViewModels;

/// <summary>
/// Main view model for motion control panel
/// </summary>
public partial class MotionControlViewModel : ObservableObject
{
    private readonly IFirmwareClientService _firmware;
    private readonly IKinematicsClientService _kinematics;
    private readonly ITrajectoryClientService _trajectory;

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

    public MotionControlViewModel(
        IFirmwareClientService firmware,
        IKinematicsClientService kinematics,
        ITrajectoryClientService trajectory)
    {
        _firmware = firmware;
        _kinematics = kinematics;
        _trajectory = trajectory;

        // Initialize 6 joints
        string[] jointNames = { "J1 Base", "J2 Shoulder", "J3 Elbow",
                                "J4 Wrist1", "J5 Wrist2", "J6 Tool" };
        for (int i = 0; i < 6; i++)
        {
            Joints.Add(new JointPositionViewModel(i, jointNames[i]));
        }

        // Subscribe to events
        _firmware.StatusUpdated += OnStatusUpdated;
        _firmware.AlarmTriggered += OnAlarmTriggered;
        _firmware.ConnectionChanged += OnConnectionChanged;
    }

    // ========================================================================
    // Commands - Connection
    // ========================================================================

    [RelayCommand]
    private async Task ConnectAsync()
    {
        var result = await _firmware.AutoConnectAsync();
        if (result.Success)
        {
            IsConnected = true;
            MachineState = "Connected";
            MachineStateColor = "#00AA00";
        }
        else
        {
            MachineState = "Connection Failed";
            MachineStateColor = "#AA0000";
        }
    }

    [RelayCommand]
    private async Task DisconnectAsync()
    {
        await _firmware.DisconnectAsync();
        IsConnected = false;
        MachineState = "Disconnected";
        MachineStateColor = "#808080";
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

        var result = await _firmware.HomeAllAsync();

        if (result.Success)
        {
            IsHomed = true;
            foreach (var joint in Joints)
            {
                joint.IsHomed = true;
            }
            MachineState = "Homed";
            MachineStateColor = "#00AA00";
        }
        else
        {
            MachineState = "Homing Failed";
            MachineStateColor = "#AA0000";
        }
    }

    [RelayCommand]
    private async Task HomeAxisAsync(int axis)
    {
        if (!IsConnected || axis < 0 || axis >= 6) return;

        var result = await _firmware.HomeAxisAsync(axis);

        if (result.Success)
        {
            Joints[axis].IsHomed = true;
            IsHomed = Joints.All(j => j.IsHomed);
        }
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

        await _firmware.JogStartAsync(axis, dir, speed);
        IsMoving = true;
    }

    [RelayCommand]
    private async Task JogStopAsync()
    {
        if (!IsConnected) return;

        await _firmware.JogStopAsync();
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

        // Create motion command
        var request = new MotionCommandRequest
        {
            MotionType = "PTP",
            TargetJoints = Joints.Select(j => j.Position * Math.PI / 180.0).ToArray(),
            VelocityScale = JogSpeed / 100.0
        };
        request.TargetJoints[axis] = targetPos * Math.PI / 180.0;

        await _trajectory.SendMotionCommandAsync(request);
    }

    // ========================================================================
    // Commands - Motion Control
    // ========================================================================

    [RelayCommand]
    private async Task StopMotionAsync()
    {
        if (!IsConnected) return;

        await _firmware.StopMotionAsync();
        IsMoving = false;
        IsPaused = false;
    }

    [RelayCommand]
    private async Task PauseMotionAsync()
    {
        if (!IsConnected || !IsMoving) return;

        await _firmware.FeedHoldAsync();
        IsPaused = true;
    }

    [RelayCommand]
    private async Task ResumeMotionAsync()
    {
        if (!IsConnected || !IsPaused) return;

        await _firmware.CycleStartAsync();
        IsPaused = false;
    }

    [RelayCommand]
    private async Task ClearAlarmAsync()
    {
        if (!IsConnected) return;

        var success = await _firmware.ClearAlarmAsync();
        if (success)
        {
            HasAlarm = false;
            AlarmMessage = "";
            MachineState = "Idle";
            MachineStateColor = "#00AA00";
        }
    }

    // ========================================================================
    // Commands - Feed Override
    // ========================================================================

    [RelayCommand]
    private async Task SetFeedOverrideAsync(int percent)
    {
        FeedOverride = Math.Clamp(percent, 10, 200);
        await _firmware.SetFeedOverrideAsync(FeedOverride);
    }

    [RelayCommand]
    private void IncreaseFeedOverride()
    {
        FeedOverride = Math.Min(200, FeedOverride + 10);
        _ = _firmware.SetFeedOverrideAsync(FeedOverride);
    }

    [RelayCommand]
    private void DecreaseFeedOverride()
    {
        FeedOverride = Math.Max(10, FeedOverride - 10);
        _ = _firmware.SetFeedOverrideAsync(FeedOverride);
    }

    [RelayCommand]
    private void ResetFeedOverride()
    {
        FeedOverride = 100;
        _ = _firmware.SetFeedOverrideAsync(100);
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
    private void SelectJoint(int index)
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

    // ========================================================================
    // Event Handlers
    // ========================================================================

    private void OnStatusUpdated(object? sender, MachineStatusData status)
    {
        // Update on UI thread
        App.Current.Dispatcher.Invoke(() =>
        {
            // Update joint positions
            for (int i = 0; i < 6 && i < status.Position.Length; i++)
            {
                double vel = i < status.Velocity.Length ? status.Velocity[i] : 0;
                Joints[i].UpdatePosition(status.Position[i], vel);
                Joints[i].IsHomed = status.IsHomed(i);
            }

            // Update machine state
            UpdateMachineState(status.GetState());

            // Update flags
            IsHomed = status.IsAllHomed;
            IsMoving = status.GetState() == MachineState.Run ||
                       status.GetState() == MachineState.Jog;
            IsPaused = status.GetState() == MachineState.Hold;

            // Check alarm
            if (status.GetAlarm() != AlarmCode.None)
            {
                HasAlarm = true;
                AlarmMessage = $"Alarm: {status.GetAlarm()}";
            }

            // Update feed override
            FeedOverride = status.FeedOverride;

            // Update TCP position via kinematics
            _ = UpdateTcpPositionAsync();
        });
    }

    private async Task UpdateTcpPositionAsync()
    {
        var jointAngles = Joints.Select(j => j.Position * Math.PI / 180.0).ToArray();
        var fkResult = await _kinematics.ComputeFKAsync(jointAngles);

        if (fkResult.Success)
        {
            TcpPosition.Update(
                fkResult.Position[0],
                fkResult.Position[1],
                fkResult.Position[2],
                fkResult.RPY[0] * 180.0 / Math.PI,
                fkResult.RPY[1] * 180.0 / Math.PI,
                fkResult.RPY[2] * 180.0 / Math.PI
            );
        }
    }

    private void UpdateMachineState(MachineState state)
    {
        switch (state)
        {
            case Core.IPC.MachineState.Idle:
                MachineState = "Idle";
                MachineStateColor = "#00AA00";
                break;
            case Core.IPC.MachineState.Run:
                MachineState = "Running";
                MachineStateColor = "#00FF00";
                break;
            case Core.IPC.MachineState.Hold:
                MachineState = "Paused";
                MachineStateColor = "#FFAA00";
                break;
            case Core.IPC.MachineState.Jog:
                MachineState = "Jogging";
                MachineStateColor = "#00AAFF";
                break;
            case Core.IPC.MachineState.Homing:
                MachineState = "Homing";
                MachineStateColor = "#AAAA00";
                break;
            case Core.IPC.MachineState.Alarm:
                MachineState = "ALARM";
                MachineStateColor = "#FF0000";
                break;
            default:
                MachineState = state.ToString();
                MachineStateColor = "#808080";
                break;
        }
    }

    private void OnAlarmTriggered(object? sender, AlarmCode alarm)
    {
        App.Current.Dispatcher.Invoke(() =>
        {
            HasAlarm = true;
            AlarmMessage = $"ALARM: {alarm}";
            MachineState = "ALARM";
            MachineStateColor = "#FF0000";
        });
    }

    private void OnConnectionChanged(object? sender, bool connected)
    {
        App.Current.Dispatcher.Invoke(() =>
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
```

---

## Step 2: Create Motion Control Views

### 2.1 Create JogPanel.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/JogPanel.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.JogPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             xmlns:d="http://schemas.microsoft.com/expression/blend/2008"
             xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006"
             mc:Ignorable="d"
             d:DesignHeight="400" d:DesignWidth="300">

    <UserControl.Resources>
        <!-- Jog Button Style -->
        <Style x:Key="JogButtonStyle" TargetType="RepeatButton">
            <Setter Property="Width" Value="60"/>
            <Setter Property="Height" Value="40"/>
            <Setter Property="FontSize" Value="16"/>
            <Setter Property="FontWeight" Value="Bold"/>
            <Setter Property="Background" Value="#404040"/>
            <Setter Property="Foreground" Value="White"/>
            <Setter Property="BorderBrush" Value="#606060"/>
            <Setter Property="BorderThickness" Value="1"/>
            <Setter Property="Delay" Value="200"/>
            <Setter Property="Interval" Value="50"/>
            <Setter Property="Template">
                <Setter.Value>
                    <ControlTemplate TargetType="RepeatButton">
                        <Border Background="{TemplateBinding Background}"
                                BorderBrush="{TemplateBinding BorderBrush}"
                                BorderThickness="{TemplateBinding BorderThickness}"
                                CornerRadius="4">
                            <ContentPresenter HorizontalAlignment="Center"
                                            VerticalAlignment="Center"/>
                        </Border>
                    </ControlTemplate>
                </Setter.Value>
            </Setter>
            <Style.Triggers>
                <Trigger Property="IsPressed" Value="True">
                    <Setter Property="Background" Value="#00AA00"/>
                </Trigger>
                <Trigger Property="IsMouseOver" Value="True">
                    <Setter Property="Background" Value="#505050"/>
                </Trigger>
                <Trigger Property="IsEnabled" Value="False">
                    <Setter Property="Background" Value="#303030"/>
                    <Setter Property="Foreground" Value="#606060"/>
                </Trigger>
            </Style.Triggers>
        </Style>

        <!-- Joint Button Style -->
        <Style x:Key="JointSelectStyle" TargetType="ToggleButton">
            <Setter Property="Width" Value="80"/>
            <Setter Property="Height" Value="32"/>
            <Setter Property="Margin" Value="2"/>
            <Setter Property="Background" Value="#353535"/>
            <Setter Property="Foreground" Value="White"/>
            <Setter Property="BorderBrush" Value="#505050"/>
            <Setter Property="Template">
                <Setter.Value>
                    <ControlTemplate TargetType="ToggleButton">
                        <Border Background="{TemplateBinding Background}"
                                BorderBrush="{TemplateBinding BorderBrush}"
                                BorderThickness="1"
                                CornerRadius="3">
                            <ContentPresenter HorizontalAlignment="Center"
                                            VerticalAlignment="Center"/>
                        </Border>
                    </ControlTemplate>
                </Setter.Value>
            </Setter>
            <Style.Triggers>
                <Trigger Property="IsChecked" Value="True">
                    <Setter Property="Background" Value="#0078D7"/>
                    <Setter Property="BorderBrush" Value="#00A0FF"/>
                </Trigger>
                <Trigger Property="IsMouseOver" Value="True">
                    <Setter Property="BorderBrush" Value="#00A0FF"/>
                </Trigger>
            </Style.Triggers>
        </Style>
    </UserControl.Resources>

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <StackPanel>
            <!-- Header -->
            <TextBlock Text="JOG CONTROL"
                       FontSize="14" FontWeight="Bold"
                       Foreground="#AAAAAA"
                       Margin="0,0,0,12"/>

            <!-- Mode Toggle -->
            <StackPanel Orientation="Horizontal" Margin="0,0,0,12">
                <RadioButton Content="Joint"
                           IsChecked="{Binding JogJointMode}"
                           GroupName="JogMode"
                           Foreground="White" Margin="0,0,16,0"/>
                <RadioButton Content="Cartesian"
                           IsChecked="{Binding JogJointMode, Converter={StaticResource InverseBoolConverter}}"
                           GroupName="JogMode"
                           Foreground="White"/>
            </StackPanel>

            <!-- Joint Selection -->
            <TextBlock Text="Select Axis:" Foreground="#888888" Margin="0,0,0,4"/>
            <WrapPanel Margin="0,0,0,12">
                <ToggleButton Content="J1" Style="{StaticResource JointSelectStyle}"
                            IsChecked="{Binding Joints[0].IsSelected}"
                            Command="{Binding SelectJointCommand}" CommandParameter="0"/>
                <ToggleButton Content="J2" Style="{StaticResource JointSelectStyle}"
                            IsChecked="{Binding Joints[1].IsSelected}"
                            Command="{Binding SelectJointCommand}" CommandParameter="1"/>
                <ToggleButton Content="J3" Style="{StaticResource JointSelectStyle}"
                            IsChecked="{Binding Joints[2].IsSelected}"
                            Command="{Binding SelectJointCommand}" CommandParameter="2"/>
                <ToggleButton Content="J4" Style="{StaticResource JointSelectStyle}"
                            IsChecked="{Binding Joints[3].IsSelected}"
                            Command="{Binding SelectJointCommand}" CommandParameter="3"/>
                <ToggleButton Content="J5" Style="{StaticResource JointSelectStyle}"
                            IsChecked="{Binding Joints[4].IsSelected}"
                            Command="{Binding SelectJointCommand}" CommandParameter="4"/>
                <ToggleButton Content="J6" Style="{StaticResource JointSelectStyle}"
                            IsChecked="{Binding Joints[5].IsSelected}"
                            Command="{Binding SelectJointCommand}" CommandParameter="5"/>
            </WrapPanel>

            <!-- Jog Buttons -->
            <Grid Margin="0,0,0,12">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="Auto"/>
                    <ColumnDefinition Width="Auto"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>

                <RepeatButton Grid.Column="1" Content="−"
                            Style="{StaticResource JogButtonStyle}"
                            PreviewMouseDown="JogMinus_MouseDown"
                            PreviewMouseUp="Jog_MouseUp"
                            IsEnabled="{Binding IsHomed}"/>

                <RepeatButton Grid.Column="2" Content="+"
                            Style="{StaticResource JogButtonStyle}"
                            PreviewMouseDown="JogPlus_MouseDown"
                            PreviewMouseUp="Jog_MouseUp"
                            IsEnabled="{Binding IsHomed}"
                            Margin="8,0,0,0"/>
            </Grid>

            <!-- Jog Mode -->
            <StackPanel Orientation="Horizontal" Margin="0,0,0,8">
                <RadioButton Content="Continuous"
                           IsChecked="{Binding JogContinuous}"
                           GroupName="JogType"
                           Foreground="White" Margin="0,0,16,0"/>
                <RadioButton Content="Increment"
                           IsChecked="{Binding JogContinuous, Converter={StaticResource InverseBoolConverter}}"
                           GroupName="JogType"
                           Foreground="White"/>
            </StackPanel>

            <!-- Increment Selection -->
            <StackPanel Orientation="Horizontal" Margin="0,0,0,12"
                       Visibility="{Binding JogContinuous, Converter={StaticResource InverseBoolToVisibility}}">
                <TextBlock Text="Increment:" Foreground="#888888"
                          VerticalAlignment="Center" Margin="0,0,8,0"/>
                <ComboBox ItemsSource="{Binding JogIncrements}"
                         SelectedItem="{Binding JogIncrement}"
                         Width="80"/>
                <TextBlock Text="°" Foreground="#888888"
                          VerticalAlignment="Center" Margin="4,0,0,0"/>
            </StackPanel>

            <!-- Speed Slider -->
            <TextBlock Text="Jog Speed:" Foreground="#888888" Margin="0,0,0,4"/>
            <Grid>
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="50"/>
                </Grid.ColumnDefinitions>
                <Slider Grid.Column="0"
                       Value="{Binding JogSpeed}"
                       Minimum="1" Maximum="100"
                       TickFrequency="10"
                       IsSnapToTickEnabled="False"/>
                <TextBlock Grid.Column="1"
                          Text="{Binding JogSpeed, StringFormat={}{0:F0}%}"
                          Foreground="White"
                          TextAlignment="Right"
                          VerticalAlignment="Center"/>
            </Grid>

            <!-- Keyboard Hint -->
            <Border Background="#252525" CornerRadius="4"
                   Padding="8" Margin="0,16,0,0">
                <StackPanel>
                    <TextBlock Text="Keyboard Shortcuts:"
                              FontWeight="Bold" Foreground="#888888"/>
                    <TextBlock Text="1-6: Select joint" Foreground="#666666"/>
                    <TextBlock Text="←/→: Jog selected axis" Foreground="#666666"/>
                    <TextBlock Text="Space: Pause/Resume" Foreground="#666666"/>
                    <TextBlock Text="Esc: Stop" Foreground="#666666"/>
                    <TextBlock Text="+/-: Feed override" Foreground="#666666"/>
                </StackPanel>
            </Border>
        </StackPanel>
    </Border>
</UserControl>
```

### 2.2 Create JogPanel.xaml.cs

**File:** `src/csharp/RobotController.UI/Views/Controls/JogPanel.xaml.cs`

```csharp
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using RobotController.UI.ViewModels;

namespace RobotController.UI.Views.Controls;

public partial class JogPanel : UserControl
{
    public JogPanel()
    {
        InitializeComponent();
    }

    private MotionControlViewModel? ViewModel => DataContext as MotionControlViewModel;

    private void JogMinus_MouseDown(object sender, MouseButtonEventArgs e)
    {
        ViewModel?.JogStartCommand.Execute("-");
    }

    private void JogPlus_MouseDown(object sender, MouseButtonEventArgs e)
    {
        ViewModel?.JogStartCommand.Execute("+");
    }

    private void Jog_MouseUp(object sender, MouseButtonEventArgs e)
    {
        if (ViewModel?.JogContinuous == true)
        {
            ViewModel?.JogStopCommand.Execute(null);
        }
    }
}
```

### 2.3 Create PositionDisplay.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/PositionDisplay.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.PositionDisplay"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             xmlns:d="http://schemas.microsoft.com/expression/blend/2008"
             xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006"
             mc:Ignorable="d"
             d:DesignHeight="500" d:DesignWidth="350">

    <UserControl.Resources>
        <!-- Position Row Style -->
        <Style x:Key="PositionRowStyle" TargetType="Border">
            <Setter Property="Background" Value="#353535"/>
            <Setter Property="CornerRadius" Value="4"/>
            <Setter Property="Padding" Value="8,6"/>
            <Setter Property="Margin" Value="0,2"/>
        </Style>
    </UserControl.Resources>

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <StackPanel>
            <!-- Header with coordinate system -->
            <Grid Margin="0,0,0,12">
                <TextBlock Text="POSITION"
                          FontSize="14" FontWeight="Bold"
                          Foreground="#AAAAAA"/>
                <ComboBox ItemsSource="{Binding CoordinateSystems}"
                         SelectedItem="{Binding CoordinateSystem}"
                         HorizontalAlignment="Right"
                         Width="100"/>
            </Grid>

            <!-- Joint Positions -->
            <TextBlock Text="Joint Positions" Foreground="#888888"
                      FontWeight="SemiBold" Margin="0,0,0,4"/>

            <ItemsControl ItemsSource="{Binding Joints}">
                <ItemsControl.ItemTemplate>
                    <DataTemplate>
                        <Border Style="{StaticResource PositionRowStyle}">
                            <Grid>
                                <Grid.ColumnDefinitions>
                                    <ColumnDefinition Width="70"/>
                                    <ColumnDefinition Width="*"/>
                                    <ColumnDefinition Width="80"/>
                                    <ColumnDefinition Width="20"/>
                                </Grid.ColumnDefinitions>

                                <!-- Joint name -->
                                <TextBlock Grid.Column="0"
                                          Text="{Binding Name}"
                                          Foreground="#AAAAAA"
                                          VerticalAlignment="Center"/>

                                <!-- Position bar -->
                                <Grid Grid.Column="1" Margin="8,0">
                                    <ProgressBar Value="{Binding PositionPercent}"
                                               Minimum="0" Maximum="100"
                                               Height="8"
                                               Background="#252525"
                                               Foreground="{Binding IsMoving,
                                                   Converter={StaticResource BoolToColorConverter},
                                                   ConverterParameter='#00AA00|#0078D7'}"/>
                                </Grid>

                                <!-- Position value -->
                                <TextBlock Grid.Column="2"
                                          Text="{Binding PositionDisplay}"
                                          Foreground="White"
                                          FontFamily="Consolas"
                                          FontSize="13"
                                          TextAlignment="Right"
                                          VerticalAlignment="Center"/>

                                <!-- Homed indicator -->
                                <Ellipse Grid.Column="3"
                                        Width="8" Height="8"
                                        Fill="{Binding IsHomed,
                                            Converter={StaticResource BoolToColorConverter},
                                            ConverterParameter='#00AA00|#AA0000'}"
                                        VerticalAlignment="Center"
                                        HorizontalAlignment="Center"/>
                            </Grid>
                        </Border>
                    </DataTemplate>
                </ItemsControl.ItemTemplate>
            </ItemsControl>

            <!-- Separator -->
            <Rectangle Height="1" Fill="#404040" Margin="0,12"/>

            <!-- TCP Position -->
            <TextBlock Text="TCP Position" Foreground="#888888"
                      FontWeight="SemiBold" Margin="0,0,0,8"/>

            <Grid>
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>
                <Grid.RowDefinitions>
                    <RowDefinition Height="Auto"/>
                    <RowDefinition Height="Auto"/>
                    <RowDefinition Height="Auto"/>
                    <RowDefinition Height="Auto"/>
                    <RowDefinition Height="Auto"/>
                    <RowDefinition Height="Auto"/>
                </Grid.RowDefinitions>

                <!-- X -->
                <TextBlock Grid.Row="0" Grid.Column="0" Text="X:"
                          Foreground="#888888" Margin="0,2"/>
                <TextBlock Grid.Row="0" Grid.Column="1"
                          Text="{Binding TcpPosition.X, StringFormat={}{0:F2} mm}"
                          Foreground="White" FontFamily="Consolas"
                          TextAlignment="Right" Margin="0,2"/>

                <!-- Y -->
                <TextBlock Grid.Row="1" Grid.Column="0" Text="Y:"
                          Foreground="#888888" Margin="0,2"/>
                <TextBlock Grid.Row="1" Grid.Column="1"
                          Text="{Binding TcpPosition.Y, StringFormat={}{0:F2} mm}"
                          Foreground="White" FontFamily="Consolas"
                          TextAlignment="Right" Margin="0,2"/>

                <!-- Z -->
                <TextBlock Grid.Row="2" Grid.Column="0" Text="Z:"
                          Foreground="#888888" Margin="0,2"/>
                <TextBlock Grid.Row="2" Grid.Column="1"
                          Text="{Binding TcpPosition.Z, StringFormat={}{0:F2} mm}"
                          Foreground="White" FontFamily="Consolas"
                          TextAlignment="Right" Margin="0,2"/>

                <!-- Roll -->
                <TextBlock Grid.Row="3" Grid.Column="0" Text="Roll:"
                          Foreground="#888888" Margin="0,2"/>
                <TextBlock Grid.Row="3" Grid.Column="1"
                          Text="{Binding TcpPosition.Roll, StringFormat={}{0:F1}°}"
                          Foreground="White" FontFamily="Consolas"
                          TextAlignment="Right" Margin="0,2"/>

                <!-- Pitch -->
                <TextBlock Grid.Row="4" Grid.Column="0" Text="Pitch:"
                          Foreground="#888888" Margin="0,2"/>
                <TextBlock Grid.Row="4" Grid.Column="1"
                          Text="{Binding TcpPosition.Pitch, StringFormat={}{0:F1}°}"
                          Foreground="White" FontFamily="Consolas"
                          TextAlignment="Right" Margin="0,2"/>

                <!-- Yaw -->
                <TextBlock Grid.Row="5" Grid.Column="0" Text="Yaw:"
                          Foreground="#888888" Margin="0,2"/>
                <TextBlock Grid.Row="5" Grid.Column="1"
                          Text="{Binding TcpPosition.Yaw, StringFormat={}{0:F1}°}"
                          Foreground="White" FontFamily="Consolas"
                          TextAlignment="Right" Margin="0,2"/>
            </Grid>

            <!-- Velocity -->
            <Border Background="#353535" CornerRadius="4"
                   Padding="8" Margin="0,12,0,0">
                <Grid>
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="Auto"/>
                    </Grid.ColumnDefinitions>
                    <TextBlock Text="Velocity:" Foreground="#888888"/>
                    <TextBlock Grid.Column="1"
                              Text="{Binding TcpPosition.VelocityDisplay}"
                              Foreground="White" FontFamily="Consolas"/>
                </Grid>
            </Border>
        </StackPanel>
    </Border>
</UserControl>
```

### 2.4 Create MotionControlPanel.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/MotionControlPanel.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.MotionControlPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             xmlns:d="http://schemas.microsoft.com/expression/blend/2008"
             xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006"
             mc:Ignorable="d"
             d:DesignHeight="300" d:DesignWidth="400">

    <UserControl.Resources>
        <!-- Control Button Style -->
        <Style x:Key="ControlButtonStyle" TargetType="Button">
            <Setter Property="Height" Value="48"/>
            <Setter Property="FontSize" Value="14"/>
            <Setter Property="FontWeight" Value="Bold"/>
            <Setter Property="Foreground" Value="White"/>
            <Setter Property="BorderThickness" Value="0"/>
            <Setter Property="Cursor" Value="Hand"/>
            <Setter Property="Template">
                <Setter.Value>
                    <ControlTemplate TargetType="Button">
                        <Border Background="{TemplateBinding Background}"
                                CornerRadius="6">
                            <ContentPresenter HorizontalAlignment="Center"
                                            VerticalAlignment="Center"/>
                        </Border>
                    </ControlTemplate>
                </Setter.Value>
            </Setter>
        </Style>
    </UserControl.Resources>

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <StackPanel>
            <!-- Status Bar -->
            <Border Background="{Binding MachineStateColor}"
                   CornerRadius="4" Padding="12,8" Margin="0,0,0,12">
                <Grid>
                    <TextBlock Text="{Binding MachineState}"
                              FontSize="16" FontWeight="Bold"
                              Foreground="White"/>
                    <TextBlock Text="{Binding AlarmMessage}"
                              HorizontalAlignment="Right"
                              Foreground="White"
                              Visibility="{Binding HasAlarm, Converter={StaticResource BoolToVisibility}}"/>
                </Grid>
            </Border>

            <!-- Connection Buttons -->
            <Grid Margin="0,0,0,12">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>

                <Button Grid.Column="0" Content="CONNECT"
                       Style="{StaticResource ControlButtonStyle}"
                       Background="#0078D7"
                       Command="{Binding ConnectCommand}"
                       Visibility="{Binding IsConnected, Converter={StaticResource InverseBoolToVisibility}}"
                       Margin="0,0,4,0"/>

                <Button Grid.Column="0" Content="DISCONNECT"
                       Style="{StaticResource ControlButtonStyle}"
                       Background="#606060"
                       Command="{Binding DisconnectCommand}"
                       Visibility="{Binding IsConnected, Converter={StaticResource BoolToVisibility}}"
                       Margin="0,0,4,0"/>

                <Button Grid.Column="1" Content="HOME ALL"
                       Style="{StaticResource ControlButtonStyle}"
                       Background="#AA6600"
                       Command="{Binding HomeAllCommand}"
                       IsEnabled="{Binding IsConnected}"
                       Margin="4,0,0,0"/>
            </Grid>

            <!-- Motion Control Buttons -->
            <Grid Margin="0,0,0,12">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>

                <!-- Stop button (always visible and red) -->
                <Button Grid.Column="0" Content="⏹ STOP"
                       Style="{StaticResource ControlButtonStyle}"
                       Background="#CC0000"
                       Command="{Binding StopMotionCommand}"
                       IsEnabled="{Binding IsConnected}"
                       Margin="0,0,4,0"/>

                <!-- Pause/Resume -->
                <Button Grid.Column="1" Content="⏸ PAUSE"
                       Style="{StaticResource ControlButtonStyle}"
                       Background="#AA8800"
                       Command="{Binding PauseMotionCommand}"
                       Visibility="{Binding IsPaused, Converter={StaticResource InverseBoolToVisibility}}"
                       IsEnabled="{Binding IsMoving}"
                       Margin="4,0,4,0"/>

                <Button Grid.Column="1" Content="▶ RESUME"
                       Style="{StaticResource ControlButtonStyle}"
                       Background="#00AA00"
                       Command="{Binding ResumeMotionCommand}"
                       Visibility="{Binding IsPaused, Converter={StaticResource BoolToVisibility}}"
                       Margin="4,0,4,0"/>

                <!-- Clear Alarm -->
                <Button Grid.Column="2" Content="CLEAR ALARM"
                       Style="{StaticResource ControlButtonStyle}"
                       Background="#AA0066"
                       Command="{Binding ClearAlarmCommand}"
                       IsEnabled="{Binding HasAlarm}"
                       Margin="4,0,0,0"/>
            </Grid>

            <!-- Feed Override -->
            <Border Background="#353535" CornerRadius="4" Padding="12">
                <StackPanel>
                    <Grid Margin="0,0,0,8">
                        <TextBlock Text="Feed Override"
                                  Foreground="#888888" FontWeight="SemiBold"/>
                        <TextBlock Text="{Binding FeedOverride, StringFormat={}{0}%}"
                                  Foreground="White" FontSize="18" FontWeight="Bold"
                                  HorizontalAlignment="Right"/>
                    </Grid>

                    <Grid>
                        <Grid.ColumnDefinitions>
                            <ColumnDefinition Width="Auto"/>
                            <ColumnDefinition Width="*"/>
                            <ColumnDefinition Width="Auto"/>
                            <ColumnDefinition Width="Auto"/>
                        </Grid.ColumnDefinitions>

                        <Button Grid.Column="0" Content="-"
                               Width="36" Height="36"
                               Background="#404040" Foreground="White"
                               FontSize="20" FontWeight="Bold"
                               BorderThickness="0"
                               Command="{Binding DecreaseFeedOverrideCommand}"/>

                        <Slider Grid.Column="1"
                               Value="{Binding FeedOverride}"
                               Minimum="10" Maximum="200"
                               TickFrequency="10"
                               IsSnapToTickEnabled="True"
                               Margin="8,0"/>

                        <Button Grid.Column="2" Content="+"
                               Width="36" Height="36"
                               Background="#404040" Foreground="White"
                               FontSize="20" FontWeight="Bold"
                               BorderThickness="0"
                               Command="{Binding IncreaseFeedOverrideCommand}"/>

                        <Button Grid.Column="3" Content="100%"
                               Width="50" Height="36"
                               Background="#505050" Foreground="White"
                               BorderThickness="0"
                               Command="{Binding ResetFeedOverrideCommand}"
                               Margin="8,0,0,0"/>
                    </Grid>
                </StackPanel>
            </Border>

            <!-- Trajectory Progress -->
            <Border Background="#353535" CornerRadius="4"
                   Padding="12" Margin="0,12,0,0"
                   Visibility="{Binding IsMoving, Converter={StaticResource BoolToVisibility}}">
                <StackPanel>
                    <Grid Margin="0,0,0,4">
                        <TextBlock Text="Trajectory:" Foreground="#888888"/>
                        <TextBlock Text="{Binding TrajectoryState}"
                                  Foreground="White"
                                  HorizontalAlignment="Right"/>
                    </Grid>
                    <ProgressBar Value="{Binding TrajectoryProgress}"
                               Minimum="0" Maximum="100"
                               Height="8"
                               Background="#252525"
                               Foreground="#0078D7"/>
                </StackPanel>
            </Border>
        </StackPanel>
    </Border>
</UserControl>
```

---

## Step 3: Create Value Converters

### 3.1 Create MotionConverters.cs

**File:** `src/csharp/RobotController.UI/Converters/MotionConverters.cs`

```csharp
using System;
using System.Globalization;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;

namespace RobotController.UI.Converters;

/// <summary>
/// Converts boolean to color (parameter format: "TrueColor|FalseColor")
/// </summary>
public class BoolToColorConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        bool boolValue = value is bool b && b;
        string param = parameter?.ToString() ?? "#00FF00|#FF0000";
        string[] colors = param.Split('|');

        string colorStr = boolValue ? colors[0] : (colors.Length > 1 ? colors[1] : colors[0]);

        if (targetType == typeof(Brush))
        {
            return new SolidColorBrush((Color)ColorConverter.ConvertFromString(colorStr));
        }

        return colorStr;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}

/// <summary>
/// Converts boolean to visibility
/// </summary>
public class BoolToVisibilityConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        bool boolValue = value is bool b && b;
        return boolValue ? Visibility.Visible : Visibility.Collapsed;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        return value is Visibility v && v == Visibility.Visible;
    }
}

/// <summary>
/// Inverts boolean then converts to visibility
/// </summary>
public class InverseBoolToVisibilityConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        bool boolValue = value is bool b && b;
        return boolValue ? Visibility.Collapsed : Visibility.Visible;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        return value is Visibility v && v == Visibility.Collapsed;
    }
}

/// <summary>
/// Inverts boolean value
/// </summary>
public class InverseBoolConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        return value is bool b && !b;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        return value is bool b && !b;
    }
}

/// <summary>
/// Converts degrees to radians display
/// </summary>
public class DegreesToRadiansConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        if (value is double degrees)
        {
            double radians = degrees * Math.PI / 180.0;
            return $"{radians:F3} rad";
        }
        return "0 rad";
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}

/// <summary>
/// Converts machine state to icon
/// </summary>
public class MachineStateToIconConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        string state = value?.ToString() ?? "";
        return state switch
        {
            "Idle" => "⏺",
            "Running" => "▶",
            "Paused" => "⏸",
            "Jogging" => "↔",
            "Homing" => "🏠",
            "ALARM" => "⚠",
            _ => "○"
        };
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}
```

---

## Step 4: Update MainWindow

### 4.1 Update MainWindow.xaml

**File:** `src/csharp/RobotController.UI/Views/MainWindow.xaml`

**Add motion controls to the right panel:**

```xml
<!-- Add namespace -->
xmlns:controls="clr-namespace:RobotController.UI.Views.Controls"

<!-- Add to Resources -->
<Window.Resources>
    <converters:BoolToColorConverter x:Key="BoolToColorConverter"/>
    <converters:BoolToVisibilityConverter x:Key="BoolToVisibility"/>
    <converters:InverseBoolToVisibilityConverter x:Key="InverseBoolToVisibility"/>
    <converters:InverseBoolConverter x:Key="InverseBoolConverter"/>
</Window.Resources>

<!-- Right Panel Content -->
<Grid Grid.Column="2">
    <Grid.RowDefinitions>
        <RowDefinition Height="Auto"/>
        <RowDefinition Height="*"/>
        <RowDefinition Height="Auto"/>
    </Grid.RowDefinitions>

    <!-- Motion Control Panel -->
    <controls:MotionControlPanel Grid.Row="0"
                                  DataContext="{Binding MotionControl}"
                                  Margin="8"/>

    <!-- Position Display -->
    <controls:PositionDisplay Grid.Row="1"
                               DataContext="{Binding MotionControl}"
                               Margin="8"/>

    <!-- Jog Panel -->
    <controls:JogPanel Grid.Row="2"
                        DataContext="{Binding MotionControl}"
                        Margin="8"/>
</Grid>
```

### 4.2 Update MainWindow.xaml.cs for Keyboard

**File:** `src/csharp/RobotController.UI/Views/MainWindow.xaml.cs`

```csharp
using System.Windows;
using System.Windows.Input;
using RobotController.UI.ViewModels;

namespace RobotController.UI.Views;

public partial class MainWindow : Window
{
    public MainWindow()
    {
        InitializeComponent();

        // Handle keyboard events for jogging
        PreviewKeyDown += MainWindow_PreviewKeyDown;
        PreviewKeyUp += MainWindow_PreviewKeyUp;
    }

    private MainViewModel? ViewModel => DataContext as MainViewModel;

    private void MainWindow_PreviewKeyDown(object sender, KeyEventArgs e)
    {
        // Don't handle if typing in a text box
        if (e.OriginalSource is System.Windows.Controls.TextBox)
            return;

        ViewModel?.MotionControl?.HandleKeyDown(e.Key);
    }

    private void MainWindow_PreviewKeyUp(object sender, KeyEventArgs e)
    {
        if (e.OriginalSource is System.Windows.Controls.TextBox)
            return;

        ViewModel?.MotionControl?.HandleKeyUp(e.Key);
    }
}
```

### 4.3 Update MainViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/MainViewModel.cs`

**Add MotionControl property:**

```csharp
public partial class MainViewModel : ObservableObject
{
    // ... existing code ...

    [ObservableProperty]
    private MotionControlViewModel _motionControl;

    public MainViewModel(
        IIpcClientService ipc,
        IFirmwareClientService firmware,
        IKinematicsClientService kinematics,
        ITrajectoryClientService trajectory)
    {
        // ... existing initialization ...

        MotionControl = new MotionControlViewModel(firmware, kinematics, trajectory);
    }
}
```

---

## Step 5: Create Homing Panel

### 5.1 Create HomingPanel.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/HomingPanel.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.HomingPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             d:DesignHeight="200" d:DesignWidth="350">

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <StackPanel>
            <TextBlock Text="HOMING"
                      FontSize="14" FontWeight="Bold"
                      Foreground="#AAAAAA"
                      Margin="0,0,0,12"/>

            <!-- Home All Button -->
            <Button Content="🏠 HOME ALL AXES"
                   Height="40"
                   Background="#AA6600"
                   Foreground="White"
                   FontWeight="Bold"
                   BorderThickness="0"
                   Command="{Binding HomeAllCommand}"
                   IsEnabled="{Binding IsConnected}"
                   Margin="0,0,0,12"/>

            <!-- Individual Axis Homing -->
            <TextBlock Text="Individual Axes:"
                      Foreground="#888888"
                      Margin="0,0,0,8"/>

            <UniformGrid Columns="3" Rows="2">
                <Button Content="J1" Margin="2"
                       Command="{Binding HomeAxisCommand}"
                       CommandParameter="0"
                       Background="{Binding Joints[0].IsHomed,
                           Converter={StaticResource BoolToColorConverter},
                           ConverterParameter='#00AA00|#404040'}"
                       Foreground="White" Height="32" BorderThickness="0"/>
                <Button Content="J2" Margin="2"
                       Command="{Binding HomeAxisCommand}"
                       CommandParameter="1"
                       Background="{Binding Joints[1].IsHomed,
                           Converter={StaticResource BoolToColorConverter},
                           ConverterParameter='#00AA00|#404040'}"
                       Foreground="White" Height="32" BorderThickness="0"/>
                <Button Content="J3" Margin="2"
                       Command="{Binding HomeAxisCommand}"
                       CommandParameter="2"
                       Background="{Binding Joints[2].IsHomed,
                           Converter={StaticResource BoolToColorConverter},
                           ConverterParameter='#00AA00|#404040'}"
                       Foreground="White" Height="32" BorderThickness="0"/>
                <Button Content="J4" Margin="2"
                       Command="{Binding HomeAxisCommand}"
                       CommandParameter="3"
                       Background="{Binding Joints[3].IsHomed,
                           Converter={StaticResource BoolToColorConverter},
                           ConverterParameter='#00AA00|#404040'}"
                       Foreground="White" Height="32" BorderThickness="0"/>
                <Button Content="J5" Margin="2"
                       Command="{Binding HomeAxisCommand}"
                       CommandParameter="4"
                       Background="{Binding Joints[4].IsHomed,
                           Converter={StaticResource BoolToColorConverter},
                           ConverterParameter='#00AA00|#404040'}"
                       Foreground="White" Height="32" BorderThickness="0"/>
                <Button Content="J6" Margin="2"
                       Command="{Binding HomeAxisCommand}"
                       CommandParameter="5"
                       Background="{Binding Joints[5].IsHomed,
                           Converter={StaticResource BoolToColorConverter},
                           ConverterParameter='#00AA00|#404040'}"
                       Foreground="White" Height="32" BorderThickness="0"/>
            </UniformGrid>

            <!-- Homing Status -->
            <Border Background="#353535" CornerRadius="4"
                   Padding="8" Margin="0,12,0,0">
                <Grid>
                    <TextBlock Text="Homing Status:" Foreground="#888888"/>
                    <TextBlock HorizontalAlignment="Right"
                              Foreground="{Binding IsHomed,
                                  Converter={StaticResource BoolToColorConverter},
                                  ConverterParameter='#00FF00|#FFAA00'}">
                        <TextBlock.Text>
                            <MultiBinding StringFormat="{}{0}">
                                <Binding Path="IsHomed"
                                        Converter="{StaticResource BoolToTextConverter}"
                                        ConverterParameter="All Homed|Not Homed"/>
                            </MultiBinding>
                        </TextBlock.Text>
                    </TextBlock>
                </Grid>
            </Border>
        </StackPanel>
    </Border>
</UserControl>
```

---

## Step 6: Unit Tests

### 6.1 Create MotionViewModelTests.cs

**File:** `src/csharp/RobotController.Tests/ViewModels/MotionViewModelTests.cs`

```csharp
using Xunit;
using Moq;
using RobotController.UI.ViewModels;
using RobotController.Core.Services;
using RobotController.Core.IPC;

namespace RobotController.Tests.ViewModels;

public class MotionViewModelTests
{
    private readonly Mock<IFirmwareClientService> _firmwareMock;
    private readonly Mock<IKinematicsClientService> _kinematicsMock;
    private readonly Mock<ITrajectoryClientService> _trajectoryMock;
    private readonly MotionControlViewModel _viewModel;

    public MotionViewModelTests()
    {
        _firmwareMock = new Mock<IFirmwareClientService>();
        _kinematicsMock = new Mock<IKinematicsClientService>();
        _trajectoryMock = new Mock<ITrajectoryClientService>();

        _viewModel = new MotionControlViewModel(
            _firmwareMock.Object,
            _kinematicsMock.Object,
            _trajectoryMock.Object);
    }

    [Fact]
    public void Constructor_InitializesSixJoints()
    {
        Assert.Equal(6, _viewModel.Joints.Count);
    }

    [Fact]
    public void Constructor_JointsHaveCorrectNames()
    {
        Assert.Equal("J1 Base", _viewModel.Joints[0].Name);
        Assert.Equal("J6 Tool", _viewModel.Joints[5].Name);
    }

    [Fact]
    public void JogSpeed_DefaultsTo50Percent()
    {
        Assert.Equal(50, _viewModel.JogSpeed);
    }

    [Fact]
    public void FeedOverride_DefaultsTo100Percent()
    {
        Assert.Equal(100, _viewModel.FeedOverride);
    }

    [Fact]
    public void SelectJoint_UpdatesSelectedIndex()
    {
        _viewModel.SelectJointCommand.Execute(3);

        Assert.Equal(3, _viewModel.SelectedJointIndex);
        Assert.True(_viewModel.Joints[3].IsSelected);
    }

    [Fact]
    public void IncreaseFeedOverride_IncreasesBy10()
    {
        _viewModel.FeedOverride = 100;
        _viewModel.IncreaseFeedOverrideCommand.Execute(null);

        Assert.Equal(110, _viewModel.FeedOverride);
    }

    [Fact]
    public void DecreaseFeedOverride_DecreasesBy10()
    {
        _viewModel.FeedOverride = 100;
        _viewModel.DecreaseFeedOverrideCommand.Execute(null);

        Assert.Equal(90, _viewModel.FeedOverride);
    }

    [Fact]
    public void FeedOverride_ClampsTo200Max()
    {
        _viewModel.FeedOverride = 200;
        _viewModel.IncreaseFeedOverrideCommand.Execute(null);

        Assert.Equal(200, _viewModel.FeedOverride);
    }

    [Fact]
    public void FeedOverride_ClampsTo10Min()
    {
        _viewModel.FeedOverride = 10;
        _viewModel.DecreaseFeedOverrideCommand.Execute(null);

        Assert.Equal(10, _viewModel.FeedOverride);
    }

    [Fact]
    public void ResetFeedOverride_SetsTo100()
    {
        _viewModel.FeedOverride = 150;
        _viewModel.ResetFeedOverrideCommand.Execute(null);

        Assert.Equal(100, _viewModel.FeedOverride);
    }
}

public class JointPositionViewModelTests
{
    [Fact]
    public void UpdatePosition_UpdatesPositionAndVelocity()
    {
        var joint = new JointPositionViewModel(0, "J1");

        joint.UpdatePosition(45.5, 10.0);

        Assert.Equal(45.5, joint.Position);
        Assert.Equal(10.0, joint.Velocity);
        Assert.True(joint.IsMoving);
    }

    [Fact]
    public void IsMoving_FalseWhenVelocityNearZero()
    {
        var joint = new JointPositionViewModel(0, "J1");

        joint.UpdatePosition(45.5, 0.05);

        Assert.False(joint.IsMoving);
    }

    [Fact]
    public void PositionPercent_CalculatesCorrectly()
    {
        var joint = new JointPositionViewModel(0, "J1");
        joint.UpdateLimits(-180, 180);
        joint.UpdatePosition(0, 0);

        Assert.Equal(50, joint.PositionPercent);
    }

    [Fact]
    public void PositionDisplay_FormatsCorrectly()
    {
        var joint = new JointPositionViewModel(0, "J1");
        joint.UpdatePosition(45.678, 0);

        Assert.Equal("45.68°", joint.PositionDisplay);
    }
}

public class CartesianPositionViewModelTests
{
    [Fact]
    public void Update_SetsAllValues()
    {
        var tcp = new CartesianPositionViewModel();

        tcp.Update(100.5, 200.25, 300.75, 10.0, 20.0, 30.0);

        Assert.Equal(100.5, tcp.X);
        Assert.Equal(200.25, tcp.Y);
        Assert.Equal(300.75, tcp.Z);
        Assert.Equal(10.0, tcp.Roll);
        Assert.Equal(20.0, tcp.Pitch);
        Assert.Equal(30.0, tcp.Yaw);
    }

    [Fact]
    public void PositionDisplay_FormatsCorrectly()
    {
        var tcp = new CartesianPositionViewModel();
        tcp.Update(100.5, 200.25, 300.75, 0, 0, 0);

        Assert.Contains("X:100.50", tcp.PositionDisplay);
        Assert.Contains("Y:200.25", tcp.PositionDisplay);
        Assert.Contains("Z:300.75", tcp.PositionDisplay);
    }
}
```

---

## Step 7: Validation

### 7.1 Build and Test

**Commands:**
```powershell
cd E:\DEV_CONTEXT_PROJECTs\Robot_controller\src\csharp

# Build
dotnet build

# Run tests
dotnet test

# Run application
dotnet run --project RobotController.UI
```

**Expected:**
- Application launches with motion control panels
- Jog panel displays 6 joints
- Position display shows joint and TCP positions
- Keyboard shortcuts work for jogging

---

## Completion Checklist

- [ ] JointPositionViewModel.cs created
- [ ] CartesianPositionViewModel.cs created
- [ ] MotionControlViewModel.cs created with commands
- [ ] JogPanel.xaml/cs created
- [ ] PositionDisplay.xaml created
- [ ] MotionControlPanel.xaml created
- [ ] HomingPanel.xaml created
- [ ] MotionConverters.cs created
- [ ] MainWindow updated with motion controls
- [ ] Keyboard shortcuts implemented
- [ ] Unit tests created
- [ ] All tests pass

---

## Troubleshooting

### Jog Not Working
- Verify firmware is connected
- Check all axes are homed
- Verify jog speed is > 0

### Position Not Updating
- Check IPC connection
- Verify status polling is active
- Check for errors in firmware communication

### Keyboard Shortcuts Not Working
- Focus must be on main window
- Not working when focus is in TextBox

---

## Keyboard Shortcuts Summary

| Key | Action |
|-----|--------|
| 1-6 | Select joint J1-J6 |
| ← / NumPad4 | Jog negative |
| → / NumPad6 | Jog positive |
| Space | Pause/Resume motion |
| Escape | Emergency stop |
| +/- | Increase/Decrease feed override |

---

## Git Commit

```powershell
git add -A
git commit -m "IMPL_P2_05: Add motion HMI controls

- Create JointPositionViewModel and CartesianPositionViewModel
- Implement MotionControlViewModel with jog, homing, motion commands
- Add JogPanel with axis selection and speed control
- Add PositionDisplay for joint and TCP positions
- Add MotionControlPanel with stop, pause, resume, feed override
- Add HomingPanel for individual and all-axis homing
- Implement keyboard shortcuts for jogging
- Create value converters for UI bindings
- Add unit tests for view models

Co-Authored-By: Claude <noreply@anthropic.com>"
```
