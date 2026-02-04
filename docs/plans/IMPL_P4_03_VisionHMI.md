# IMPL_P4_03: Vision HMI

| Metadata | Value |
|----------|-------|
| Plan ID | IMPL_P4_03 |
| Phase | 4 - Vision Integration |
| Priority | P1 |
| Depends On | IMPL_P4_01 (Sensor Drivers), IMPL_P4_02 (Seam Detection), IMPL_P1_04 (HMI) |
| Status | Ready for Implementation |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/Thiết kế HMI Robot KUKA WPF.md` | MVVM pattern, industrial UI, control layouts |
| P0 | `ressearch_doc_md/Robot Hàn_ Cảm Biến Laser & Mã Nguồn.md` | Profile display, sensor UI requirements |

---

## Overview

Implementation plan cho Vision HMI - giao diện điều khiển vision system:
- **Profile Display:** Hiển thị laser profile real-time với detected features
- **Point Cloud View:** 3D visualization của scan data
- **Seam Overlay:** Hiển thị detected seam, deviation, tracking status
- **Calibration UI:** Hand-eye calibration wizard
- **Scan-to-Path UI:** Thu thập scan data và tạo weld path
- **Configuration Panels:** Sensor settings, detection parameters

---

## HMI Layout Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          VISION CONTROL HMI                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌────────────────────────────────────┐  ┌───────────────────────────┐  │
│  │         PROFILE DISPLAY            │  │    TRACKING STATUS        │  │
│  │                                    │  │                           │  │
│  │    ┌────────────────────────┐     │  │   ◉ TRACKING ACTIVE      │  │
│  │    │     /\      V-Groove   │     │  │                           │  │
│  │    │    /  \    Gap: 3.2mm  │     │  │   Quality: ████████ 92%  │  │
│  │    │   /    \   Angle: 45°  │     │  │   Deviation: +0.3mm      │  │
│  │    │  /      \              │     │  │   Height: -0.1mm         │  │
│  │    │ ▼ Root   ▼             │     │  │                           │  │
│  │    └────────────────────────┘     │  │   Frames: 12,456         │  │
│  │                                    │  │   Latency: 8.2ms         │  │
│  │    [───────────●────────────]     │  │                           │  │
│  │        Profile Position            │  └───────────────────────────┘  │
│  └────────────────────────────────────┘                                 │
│                                                                          │
│  ┌────────────────────────────────────┐  ┌───────────────────────────┐  │
│  │       3D POINT CLOUD VIEW          │  │   SENSOR STATUS           │  │
│  │                                    │  │                           │  │
│  │       ░░░░░░░░░░░░░░░░░░░░        │  │   Laser: ● Connected     │  │
│  │      ░░░░░░░░░░░░░░░░░░░░░░       │  │   Frame Rate: 98.5 Hz    │  │
│  │     ░░░░░▓▓▓▓▓▓▓▓▓▓░░░░░░░░      │  │   Frames: 156,782        │  │
│  │      ░░░░░░░░░░░░░░░░░░░░░░       │  │   Dropped: 12            │  │
│  │       ░░░░░░░░░░░░░░░░░░░░        │  │                           │  │
│  │                                    │  │   Exposure: 1000 µs      │  │
│  │   [Rotate] [Pan] [Zoom] [Reset]   │  │   Gain: 1.0              │  │
│  └────────────────────────────────────┘  └───────────────────────────┘  │
│                                                                          │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │  [ START SCAN ]  [ STOP ]  [ CALIBRATE ]  │  Mode: [Tracking ▼]  │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

- [ ] IMPL_P4_01 (Sensor Drivers) đã hoàn thành
- [ ] IMPL_P4_02 (Seam Detection) đã hoàn thành
- [ ] IMPL_P1_04 (HMI & 3D Visualization) đã hoàn thành
- [ ] WPF project builds successfully
- [ ] Helix Toolkit configured

---

## Step 1: Create Vision View Models

### 1.1 Create ProfileDisplayViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/Vision/ProfileDisplayViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Core.Services;
using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Media;

namespace RobotController.UI.ViewModels.Vision;

/// <summary>
/// View model for laser profile display with seam detection overlay
/// </summary>
public partial class ProfileDisplayViewModel : ObservableObject
{
    private readonly ISensorClientService? _sensorService;
    private readonly ISeamDetectionService? _seamService;

    // ========================================================================
    // Profile Data
    // ========================================================================

    [ObservableProperty]
    private PointCollection _profilePoints = new();

    [ObservableProperty]
    private double _canvasWidth = 400;

    [ObservableProperty]
    private double _canvasHeight = 200;

    [ObservableProperty]
    private double _scaleX = 2.0;

    [ObservableProperty]
    private double _scaleZ = 2.0;

    [ObservableProperty]
    private double _offsetX = 200;

    [ObservableProperty]
    private double _offsetZ = 150;

    // ========================================================================
    // Seam Feature Overlay
    // ========================================================================

    [ObservableProperty]
    private bool _showSeamOverlay = true;

    [ObservableProperty]
    private Point _rootPoint;

    [ObservableProperty]
    private Point _leftLineStart;

    [ObservableProperty]
    private Point _leftLineEnd;

    [ObservableProperty]
    private Point _rightLineStart;

    [ObservableProperty]
    private Point _rightLineEnd;

    [ObservableProperty]
    private bool _hasValidFeature;

    // ========================================================================
    // Feature Info
    // ========================================================================

    [ObservableProperty]
    private string _jointType = "Unknown";

    [ObservableProperty]
    private double _gapWidth;

    [ObservableProperty]
    private double _leftAngle;

    [ObservableProperty]
    private double _rightAngle;

    [ObservableProperty]
    private double _depth;

    [ObservableProperty]
    private double _confidence;

    // ========================================================================
    // Display Settings
    // ========================================================================

    [ObservableProperty]
    private bool _autoScale = true;

    [ObservableProperty]
    private bool _showGrid = true;

    [ObservableProperty]
    private bool _showIntensity;

    [ObservableProperty]
    private string _profileColor = "#00FF00";

    [ObservableProperty]
    private string _seamColor = "#FF0000";

    // ========================================================================
    // Statistics
    // ========================================================================

    [ObservableProperty]
    private int _pointCount;

    [ObservableProperty]
    private double _minZ;

    [ObservableProperty]
    private double _maxZ;

    [ObservableProperty]
    private ulong _frameId;

    public ProfileDisplayViewModel(
        ISensorClientService? sensorService = null,
        ISeamDetectionService? seamService = null)
    {
        _sensorService = sensorService;
        _seamService = seamService;

        if (_sensorService != null)
        {
            _sensorService.ProfileReceived += OnProfileReceived;
        }

        if (_seamService != null)
        {
            _seamService.FeatureDetected += OnFeatureDetected;
        }
    }

    private void OnProfileReceived(object? sender, ProfileReceivedEventArgs e)
    {
        App.Current.Dispatcher.Invoke(() => UpdateProfile(e));
    }

    private void OnFeatureDetected(object? sender, SeamFeatureEventArgs e)
    {
        App.Current.Dispatcher.Invoke(() => UpdateSeamFeature(e.Feature));
    }

    public void UpdateProfile(ProfileReceivedEventArgs data)
    {
        if (data.PointsX.Length == 0) return;

        var points = new PointCollection();

        // Auto-scale
        if (AutoScale && data.PointsZ.Length > 0)
        {
            float minX = data.PointsX.Min();
            float maxX = data.PointsX.Max();
            float minZ = data.PointsZ.Min();
            float maxZ = data.PointsZ.Max();

            float rangeX = maxX - minX;
            float rangeZ = maxZ - minZ;

            if (rangeX > 0) ScaleX = (CanvasWidth - 40) / rangeX;
            if (rangeZ > 0) ScaleZ = (CanvasHeight - 40) / rangeZ;

            OffsetX = -minX * ScaleX + 20;
            OffsetZ = maxZ * ScaleZ + 20;

            MinZ = minZ;
            MaxZ = maxZ;
        }

        // Convert to canvas coordinates
        for (int i = 0; i < data.PointsX.Length; i++)
        {
            double x = data.PointsX[i] * ScaleX + OffsetX;
            double z = -data.PointsZ[i] * ScaleZ + OffsetZ;  // Flip Z

            if (x >= 0 && x <= CanvasWidth && z >= 0 && z <= CanvasHeight)
            {
                points.Add(new Point(x, z));
            }
        }

        ProfilePoints = points;
        PointCount = data.PointsX.Length;
        FrameId = data.FrameId;
    }

    public void UpdateSeamFeature(SeamFeatureData feature)
    {
        HasValidFeature = feature.Valid;

        if (!feature.Valid)
        {
            JointType = "Not detected";
            return;
        }

        JointType = feature.JointType;
        GapWidth = feature.GapWidth;
        LeftAngle = feature.LeftAngle;
        RightAngle = feature.RightAngle;
        Depth = feature.Depth;
        Confidence = feature.Confidence;

        // Convert root point to canvas coordinates
        double rootX = feature.RootX * ScaleX + OffsetX;
        double rootZ = -feature.RootZ * ScaleZ + OffsetZ;
        RootPoint = new Point(rootX, rootZ);

        // Draw groove lines if V-groove
        if (feature.JointType == "V-Groove" || feature.JointType == "VGroove")
        {
            // Left line (from root going up-left)
            double leftSlope = Math.Tan(feature.LeftAngle * Math.PI / 180);
            LeftLineStart = RootPoint;
            LeftLineEnd = new Point(
                rootX - 30,
                rootZ - 30 * leftSlope);

            // Right line (from root going up-right)
            double rightSlope = Math.Tan(feature.RightAngle * Math.PI / 180);
            RightLineStart = RootPoint;
            RightLineEnd = new Point(
                rootX + 30,
                rootZ - 30 * rightSlope);
        }
    }

    [RelayCommand]
    private void ResetView()
    {
        ScaleX = 2.0;
        ScaleZ = 2.0;
        OffsetX = CanvasWidth / 2;
        OffsetZ = CanvasHeight / 2;
        AutoScale = true;
    }

    [RelayCommand]
    private void ZoomIn()
    {
        AutoScale = false;
        ScaleX *= 1.2;
        ScaleZ *= 1.2;
    }

    [RelayCommand]
    private void ZoomOut()
    {
        AutoScale = false;
        ScaleX /= 1.2;
        ScaleZ /= 1.2;
    }
}
```

### 1.2 Create TrackingStatusViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/Vision/TrackingStatusViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using RobotController.Core.Services;

namespace RobotController.UI.ViewModels.Vision;

/// <summary>
/// View model for seam tracking status display
/// </summary>
public partial class TrackingStatusViewModel : ObservableObject
{
    private readonly ISeamDetectionService? _seamService;

    // ========================================================================
    // Tracking State
    // ========================================================================

    [ObservableProperty]
    private bool _isTracking;

    [ObservableProperty]
    private bool _isTrackingLost;

    [ObservableProperty]
    private string _trackingStatus = "IDLE";

    [ObservableProperty]
    private string _statusColor = "#808080";

    // ========================================================================
    // Quality Metrics
    // ========================================================================

    [ObservableProperty]
    private double _trackingQuality;

    [ObservableProperty]
    private int _qualityPercent;

    [ObservableProperty]
    private string _qualityColor = "#808080";

    // ========================================================================
    // Corrections
    // ========================================================================

    [ObservableProperty]
    private double _lateralDeviation;

    [ObservableProperty]
    private double _heightOffset;

    [ObservableProperty]
    private double _rollCorrection;

    [ObservableProperty]
    private string _deviationColor = "#00FF00";

    // ========================================================================
    // Predictions
    // ========================================================================

    [ObservableProperty]
    private double _predictedX;

    [ObservableProperty]
    private double _predictedZ;

    // ========================================================================
    // Statistics
    // ========================================================================

    [ObservableProperty]
    private ulong _framesProcessed;

    [ObservableProperty]
    private ulong _detectionsValid;

    [ObservableProperty]
    private double _detectionRate;

    [ObservableProperty]
    private double _avgLatency;

    [ObservableProperty]
    private int _lostFrameCount;

    // Display strings
    public string DeviationDisplay => $"{LateralDeviation:+0.00;-0.00;0.00} mm";
    public string HeightDisplay => $"{HeightOffset:+0.00;-0.00;0.00} mm";
    public string LatencyDisplay => $"{AvgLatency:F1} ms";
    public string FramesDisplay => $"{FramesProcessed:N0}";
    public string DetectionRateDisplay => $"{DetectionRate:F1}%";

    public TrackingStatusViewModel(ISeamDetectionService? seamService = null)
    {
        _seamService = seamService;

        if (_seamService != null)
        {
            _seamService.TrackingStateChanged += OnTrackingStateChanged;
        }
    }

    private void OnTrackingStateChanged(object? sender, TrackingStateEventArgs e)
    {
        App.Current.Dispatcher.Invoke(() => UpdateState(e.State));
    }

    public void UpdateState(TrackingStateData state)
    {
        IsTracking = state.Tracking;
        IsTrackingLost = state.TrackingLost;

        // Status display
        if (state.TrackingLost)
        {
            TrackingStatus = "LOST";
            StatusColor = "#FF0000";
        }
        else if (state.Tracking)
        {
            TrackingStatus = "TRACKING";
            StatusColor = "#00FF00";
        }
        else
        {
            TrackingStatus = "IDLE";
            StatusColor = "#808080";
        }

        // Quality
        TrackingQuality = state.TrackingQuality;
        QualityPercent = (int)(state.TrackingQuality * 100);
        QualityColor = state.TrackingQuality switch
        {
            > 0.8 => "#00FF00",
            > 0.5 => "#FFFF00",
            > 0.2 => "#FF8800",
            _ => "#FF0000"
        };

        // Corrections
        LateralDeviation = state.LateralCorrection;
        HeightOffset = state.HeightCorrection;

        // Deviation color (green = small, red = large)
        double absDeviation = Math.Abs(state.LateralCorrection);
        DeviationColor = absDeviation switch
        {
            < 0.5 => "#00FF00",
            < 1.0 => "#FFFF00",
            < 2.0 => "#FF8800",
            _ => "#FF0000"
        };

        // Predictions
        PredictedX = state.PredictedX;
        PredictedZ = state.PredictedZ;

        // Statistics
        FramesProcessed = state.FramesProcessed;
        DetectionsValid = state.DetectionsValid;
        AvgLatency = state.AvgLatency;

        if (state.FramesProcessed > 0)
        {
            DetectionRate = (double)state.DetectionsValid / state.FramesProcessed * 100;
        }

        // Notify display properties
        OnPropertyChanged(nameof(DeviationDisplay));
        OnPropertyChanged(nameof(HeightDisplay));
        OnPropertyChanged(nameof(LatencyDisplay));
        OnPropertyChanged(nameof(FramesDisplay));
        OnPropertyChanged(nameof(DetectionRateDisplay));
    }

    public void Reset()
    {
        IsTracking = false;
        IsTrackingLost = false;
        TrackingStatus = "IDLE";
        StatusColor = "#808080";
        TrackingQuality = 0;
        QualityPercent = 0;
        LateralDeviation = 0;
        HeightOffset = 0;
        FramesProcessed = 0;
        DetectionsValid = 0;
        AvgLatency = 0;
    }
}
```

### 1.3 Create SensorStatusViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/Vision/SensorStatusViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Core.Services;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels.Vision;

/// <summary>
/// View model for sensor connection and status
/// </summary>
public partial class SensorStatusViewModel : ObservableObject
{
    private readonly ISensorClientService _sensorService;

    // ========================================================================
    // Connection
    // ========================================================================

    [ObservableProperty]
    private bool _isConnected;

    [ObservableProperty]
    private bool _isAcquiring;

    [ObservableProperty]
    private string _connectionStatus = "Disconnected";

    [ObservableProperty]
    private string _statusColor = "#808080";

    [ObservableProperty]
    private string _selectedDevice = "";

    public ObservableCollection<string> AvailableDevices { get; } = new();

    // ========================================================================
    // Sensor Info
    // ========================================================================

    [ObservableProperty]
    private string _deviceId = "";

    [ObservableProperty]
    private string _serialNumber = "";

    [ObservableProperty]
    private string _modelName = "";

    // ========================================================================
    // Performance
    // ========================================================================

    [ObservableProperty]
    private double _frameRate;

    [ObservableProperty]
    private ulong _framesReceived;

    [ObservableProperty]
    private ulong _framesDropped;

    [ObservableProperty]
    private double _dropRate;

    // ========================================================================
    // Configuration
    // ========================================================================

    [ObservableProperty]
    private double _exposureTime = 1000;

    [ObservableProperty]
    private double _gain = 1.0;

    [ObservableProperty]
    private string _triggerMode = "FreeRun";

    [ObservableProperty]
    private uint _encoderDivider = 1;

    public ObservableCollection<string> TriggerModes { get; } = new()
    {
        "FreeRun",
        "Software",
        "Encoder",
        "Hardware"
    };

    // Display strings
    public string FrameRateDisplay => $"{FrameRate:F1} Hz";
    public string FramesDisplay => $"{FramesReceived:N0}";
    public string DroppedDisplay => $"{FramesDropped:N0} ({DropRate:F2}%)";

    public SensorStatusViewModel(ISensorClientService sensorService)
    {
        _sensorService = sensorService;

        _sensorService.ConnectionChanged += OnConnectionChanged;
    }

    private void OnConnectionChanged(object? sender, SensorConnectionEventArgs e)
    {
        App.Current.Dispatcher.Invoke(() =>
        {
            IsConnected = e.Connected;
            DeviceId = e.DeviceId;

            if (e.Connected)
            {
                ConnectionStatus = "Connected";
                StatusColor = "#00FF00";
            }
            else
            {
                ConnectionStatus = e.Error ?? "Disconnected";
                StatusColor = string.IsNullOrEmpty(e.Error) ? "#808080" : "#FF0000";
            }
        });
    }

    [RelayCommand]
    private async Task EnumerateDevicesAsync()
    {
        var devices = await _sensorService.EnumerateLaserProfilersAsync();

        AvailableDevices.Clear();
        foreach (var device in devices)
        {
            AvailableDevices.Add(device.DeviceId);
        }

        if (AvailableDevices.Count > 0 && string.IsNullOrEmpty(SelectedDevice))
        {
            SelectedDevice = AvailableDevices[0];
        }
    }

    [RelayCommand]
    private async Task ConnectAsync()
    {
        if (string.IsNullOrEmpty(SelectedDevice)) return;

        ConnectionStatus = "Connecting...";
        StatusColor = "#FFAA00";

        var result = await _sensorService.ConnectLaserAsync(SelectedDevice);

        if (!result.Success)
        {
            ConnectionStatus = result.Message;
            StatusColor = "#FF0000";
        }
    }

    [RelayCommand]
    private async Task DisconnectAsync()
    {
        await _sensorService.DisconnectLaserAsync();
    }

    [RelayCommand]
    private async Task StartAcquisitionAsync()
    {
        var result = await _sensorService.StartLaserAcquisitionAsync();
        if (result.Success)
        {
            IsAcquiring = true;
        }
    }

    [RelayCommand]
    private async Task StopAcquisitionAsync()
    {
        var result = await _sensorService.StopLaserAcquisitionAsync();
        if (result.Success)
        {
            IsAcquiring = false;
        }
    }

    [RelayCommand]
    private async Task ApplyConfigAsync()
    {
        var config = new LaserProfilerConfigData
        {
            ExposureTime = (float)ExposureTime,
            Gain = (float)Gain,
            TriggerMode = TriggerMode.ToLower(),
            EncoderDivider = EncoderDivider
        };

        await _sensorService.ConfigureLaserAsync(config);
    }

    public async Task RefreshStatusAsync()
    {
        var status = await _sensorService.GetLaserStatusAsync();

        if (status.Success)
        {
            FrameRate = status.FrameRate;
            FramesReceived = status.FramesReceived;
            FramesDropped = status.FramesDropped;

            if (status.FramesReceived > 0)
            {
                DropRate = (double)status.FramesDropped / status.FramesReceived * 100;
            }

            SerialNumber = status.SerialNumber;
            ModelName = status.ModelName;

            OnPropertyChanged(nameof(FrameRateDisplay));
            OnPropertyChanged(nameof(FramesDisplay));
            OnPropertyChanged(nameof(DroppedDisplay));
        }
    }
}
```

### 1.4 Create CalibrationViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/Vision/CalibrationViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Core.Services;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels.Vision;

/// <summary>
/// View model for hand-eye calibration wizard
/// </summary>
public partial class CalibrationViewModel : ObservableObject
{
    private readonly ISensorClientService _sensorService;
    private readonly IKinematicsClientService? _kinematicsService;

    // ========================================================================
    // Wizard State
    // ========================================================================

    [ObservableProperty]
    private int _currentStep;

    [ObservableProperty]
    private int _totalSteps = 5;

    [ObservableProperty]
    private string _stepDescription = "Move robot to first calibration position";

    [ObservableProperty]
    private bool _canGoNext;

    [ObservableProperty]
    private bool _canGoPrevious;

    [ObservableProperty]
    private bool _isCalibrating;

    // ========================================================================
    // Calibration Data
    // ========================================================================

    public ObservableCollection<CalibrationPoseEntry> CapturedPoses { get; } = new();

    [ObservableProperty]
    private int _requiredPoses = 10;

    [ObservableProperty]
    private int _capturedPoseCount;

    // Current robot pose
    [ObservableProperty]
    private double[] _currentJoints = new double[6];

    [ObservableProperty]
    private double[] _currentPosition = new double[3];

    // ========================================================================
    // Results
    // ========================================================================

    [ObservableProperty]
    private bool _calibrationComplete;

    [ObservableProperty]
    private double _reprojectionError;

    [ObservableProperty]
    private double[] _translation = new double[3];

    [ObservableProperty]
    private double[] _rotation = new double[3];

    [ObservableProperty]
    private string _calibrationStatus = "";

    // Display
    public string ProgressText => $"Pose {CapturedPoseCount} / {RequiredPoses}";
    public double ProgressPercent => (double)CapturedPoseCount / RequiredPoses * 100;

    public CalibrationViewModel(
        ISensorClientService sensorService,
        IKinematicsClientService? kinematicsService = null)
    {
        _sensorService = sensorService;
        _kinematicsService = kinematicsService;

        CurrentStep = 0;
        UpdateStepDescription();
    }

    [RelayCommand]
    private async Task CapturePoseAsync()
    {
        // Get current robot pose
        // In real implementation, get from kinematics service
        var robotJoints = CurrentJoints.ToArray();

        // Get profile and detect calibration target
        var profile = await _sensorService.GetProfileAsync(1000);

        if (!profile.Success)
        {
            CalibrationStatus = "Failed to get profile";
            return;
        }

        // Detect calibration target (e.g., corner, sphere)
        // This is simplified - real implementation would detect target
        var targetPose = new double[16];  // Identity matrix placeholder
        for (int i = 0; i < 4; i++) targetPose[i * 4 + i] = 1.0;

        var entry = new CalibrationPoseEntry
        {
            Index = CapturedPoseCount + 1,
            RobotJoints = robotJoints,
            TargetPose = targetPose,
            Timestamp = DateTime.Now
        };

        CapturedPoses.Add(entry);
        CapturedPoseCount = CapturedPoses.Count;

        OnPropertyChanged(nameof(ProgressText));
        OnPropertyChanged(nameof(ProgressPercent));

        CalibrationStatus = $"Captured pose {CapturedPoseCount}";

        if (CapturedPoseCount >= RequiredPoses)
        {
            CanGoNext = true;
        }
    }

    [RelayCommand]
    private void DeletePose(CalibrationPoseEntry? entry)
    {
        if (entry != null)
        {
            CapturedPoses.Remove(entry);
            CapturedPoseCount = CapturedPoses.Count;

            // Re-index
            for (int i = 0; i < CapturedPoses.Count; i++)
            {
                CapturedPoses[i].Index = i + 1;
            }

            OnPropertyChanged(nameof(ProgressText));
            OnPropertyChanged(nameof(ProgressPercent));
        }
    }

    [RelayCommand]
    private async Task PerformCalibrationAsync()
    {
        if (CapturedPoses.Count < 3)
        {
            CalibrationStatus = "Need at least 3 poses";
            return;
        }

        IsCalibrating = true;
        CalibrationStatus = "Calibrating...";

        var robotJoints = CapturedPoses.Select(p => p.RobotJoints).ToList();
        var targetPoses = CapturedPoses.Select(p => p.TargetPose).ToList();

        var result = await _sensorService.PerformHandEyeCalibrationAsync(robotJoints, targetPoses);

        IsCalibrating = false;

        if (result.Success)
        {
            CalibrationComplete = true;
            ReprojectionError = result.ReprojectionError;
            Translation = result.Translation;

            CalibrationStatus = $"Calibration complete! Error: {ReprojectionError:F3} mm";
            CurrentStep = TotalSteps - 1;
        }
        else
        {
            CalibrationStatus = result.ErrorMessage;
        }
    }

    [RelayCommand]
    private async Task SaveCalibrationAsync()
    {
        string filepath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "Calibration", "hand_eye_calibration.json");

        Directory.CreateDirectory(Path.GetDirectoryName(filepath)!);

        var result = await _sensorService.SaveCalibrationAsync(filepath);

        CalibrationStatus = result.Success
            ? $"Saved to {filepath}"
            : result.Message;
    }

    [RelayCommand]
    private async Task LoadCalibrationAsync()
    {
        string filepath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "Calibration", "hand_eye_calibration.json");

        var result = await _sensorService.LoadCalibrationAsync(filepath);

        if (result.Success)
        {
            CalibrationStatus = "Calibration loaded";
            CalibrationComplete = true;

            // Get and display loaded calibration
            var cal = await _sensorService.GetCalibrationAsync();
            if (cal.Success)
            {
                ReprojectionError = cal.ReprojectionError;
                Translation = cal.Translation;
            }
        }
        else
        {
            CalibrationStatus = result.Message;
        }
    }

    [RelayCommand]
    private void NextStep()
    {
        if (CurrentStep < TotalSteps - 1)
        {
            CurrentStep++;
            UpdateStepDescription();
        }
    }

    [RelayCommand]
    private void PreviousStep()
    {
        if (CurrentStep > 0)
        {
            CurrentStep--;
            UpdateStepDescription();
        }
    }

    [RelayCommand]
    private void Reset()
    {
        CapturedPoses.Clear();
        CapturedPoseCount = 0;
        CalibrationComplete = false;
        ReprojectionError = 0;
        CurrentStep = 0;
        CalibrationStatus = "";
        UpdateStepDescription();
    }

    private void UpdateStepDescription()
    {
        StepDescription = CurrentStep switch
        {
            0 => "Step 1: Connect sensor and verify it's working",
            1 => "Step 2: Position calibration target in sensor view",
            2 => $"Step 3: Capture {RequiredPoses} poses from different angles",
            3 => "Step 4: Run calibration algorithm",
            4 => "Step 5: Verify and save calibration",
            _ => ""
        };

        CanGoPrevious = CurrentStep > 0;
        CanGoNext = CurrentStep switch
        {
            2 => CapturedPoseCount >= RequiredPoses,
            3 => CalibrationComplete,
            _ => true
        };
    }
}

public class CalibrationPoseEntry
{
    public int Index { get; set; }
    public double[] RobotJoints { get; set; } = new double[6];
    public double[] TargetPose { get; set; } = new double[16];
    public DateTime Timestamp { get; set; }

    public string JointsDisplay =>
        string.Join(", ", RobotJoints.Select(j => $"{j * 180 / Math.PI:F1}°"));
}
```

### 1.5 Create ScanToPathViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/Vision/ScanToPathViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Core.Services;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels.Vision;

/// <summary>
/// View model for Scan-to-Path functionality
/// </summary>
public partial class ScanToPathViewModel : ObservableObject
{
    private readonly ISensorClientService _sensorService;
    private readonly ISeamDetectionService _seamService;

    // ========================================================================
    // Scan Settings
    // ========================================================================

    [ObservableProperty]
    private uint _profileCount = 500;

    [ObservableProperty]
    private uint _timeoutMs = 30000;

    [ObservableProperty]
    private float _voxelSize = 1.0f;

    [ObservableProperty]
    private bool _enableDownsample = true;

    // ========================================================================
    // Scan State
    // ========================================================================

    [ObservableProperty]
    private bool _isScanning;

    [ObservableProperty]
    private double _scanProgress;

    [ObservableProperty]
    private string _scanStatus = "Ready";

    // ========================================================================
    // Results
    // ========================================================================

    [ObservableProperty]
    private SeamPathData? _scannedPath;

    [ObservableProperty]
    private bool _hasPath;

    [ObservableProperty]
    private int _pathPointCount;

    [ObservableProperty]
    private double _pathLength;

    [ObservableProperty]
    private double _avgGapWidth;

    [ObservableProperty]
    private string _detectedJointType = "";

    // ========================================================================
    // Path List
    // ========================================================================

    public ObservableCollection<SavedPathEntry> SavedPaths { get; } = new();

    [ObservableProperty]
    private SavedPathEntry? _selectedPath;

    public ScanToPathViewModel(
        ISensorClientService sensorService,
        ISeamDetectionService seamService)
    {
        _sensorService = sensorService;
        _seamService = seamService;

        LoadSavedPaths();
    }

    [RelayCommand]
    private async Task StartScanAsync()
    {
        IsScanning = true;
        ScanStatus = "Scanning...";
        ScanProgress = 0;

        try
        {
            // Start scan
            var result = await _seamService.ScanSeamPathAsync(ProfileCount, TimeoutMs);

            if (result.Valid)
            {
                ScannedPath = result;
                HasPath = true;
                PathPointCount = result.PointCount;
                PathLength = result.TotalLength;
                AvgGapWidth = result.AvgGapWidth;
                DetectedJointType = result.JointType;

                ScanStatus = $"Scan complete: {PathPointCount} points, {PathLength:F1}mm";
            }
            else
            {
                ScanStatus = "Scan failed";
                HasPath = false;
            }
        }
        catch (Exception ex)
        {
            ScanStatus = $"Error: {ex.Message}";
        }
        finally
        {
            IsScanning = false;
            ScanProgress = 100;
        }
    }

    [RelayCommand]
    private void StopScan()
    {
        // Signal to stop (implementation depends on how scan is running)
        IsScanning = false;
        ScanStatus = "Scan stopped";
    }

    [RelayCommand]
    private async Task SetAsNominalPathAsync()
    {
        if (ScannedPath == null) return;

        var result = await _seamService.SetNominalPathAsync(ScannedPath);

        ScanStatus = result.Success
            ? "Set as nominal path"
            : $"Failed: {result.Message}";
    }

    [RelayCommand]
    private async Task SavePathAsync()
    {
        if (ScannedPath == null) return;

        string name = $"Path_{DateTime.Now:yyyyMMdd_HHmmss}";
        string pathsDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "SeamPaths");

        Directory.CreateDirectory(pathsDir);

        string filepath = Path.Combine(pathsDir, $"{name}.json");

        // Serialize and save
        var json = System.Text.Json.JsonSerializer.Serialize(ScannedPath);
        await File.WriteAllTextAsync(filepath, json);

        var entry = new SavedPathEntry
        {
            Name = name,
            FilePath = filepath,
            PointCount = PathPointCount,
            Length = PathLength,
            CreatedAt = DateTime.Now
        };

        SavedPaths.Add(entry);
        ScanStatus = $"Saved as {name}";
    }

    [RelayCommand]
    private async Task LoadPathAsync(SavedPathEntry? entry)
    {
        if (entry == null || !File.Exists(entry.FilePath)) return;

        var json = await File.ReadAllTextAsync(entry.FilePath);
        var path = System.Text.Json.JsonSerializer.Deserialize<SeamPathData>(json);

        if (path != null)
        {
            ScannedPath = path;
            HasPath = true;
            PathPointCount = path.PointCount;
            PathLength = path.TotalLength;
            AvgGapWidth = path.AvgGapWidth;
            DetectedJointType = path.JointType;

            ScanStatus = $"Loaded {entry.Name}";
        }
    }

    [RelayCommand]
    private void DeletePath(SavedPathEntry? entry)
    {
        if (entry == null) return;

        if (File.Exists(entry.FilePath))
        {
            File.Delete(entry.FilePath);
        }

        SavedPaths.Remove(entry);
    }

    private void LoadSavedPaths()
    {
        string pathsDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "SeamPaths");

        if (!Directory.Exists(pathsDir)) return;

        foreach (var file in Directory.GetFiles(pathsDir, "*.json"))
        {
            var info = new FileInfo(file);
            SavedPaths.Add(new SavedPathEntry
            {
                Name = Path.GetFileNameWithoutExtension(file),
                FilePath = file,
                CreatedAt = info.CreationTime
            });
        }
    }
}

public class SavedPathEntry
{
    public string Name { get; set; } = "";
    public string FilePath { get; set; } = "";
    public int PointCount { get; set; }
    public double Length { get; set; }
    public DateTime CreatedAt { get; set; }

    public string DisplayText => $"{Name} ({PointCount} pts, {Length:F0}mm)";
}
```

### 1.6 Create VisionControlViewModel.cs (Main)

**File:** `src/csharp/RobotController.UI/ViewModels/Vision/VisionControlViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Core.Services;

namespace RobotController.UI.ViewModels.Vision;

/// <summary>
/// Main view model for vision control panel
/// </summary>
public partial class VisionControlViewModel : ObservableObject
{
    private readonly ISensorClientService _sensorService;
    private readonly ISeamDetectionService _seamService;

    // ========================================================================
    // Sub-ViewModels
    // ========================================================================

    public ProfileDisplayViewModel ProfileDisplay { get; }
    public TrackingStatusViewModel TrackingStatus { get; }
    public SensorStatusViewModel SensorStatus { get; }
    public CalibrationViewModel Calibration { get; }
    public ScanToPathViewModel ScanToPath { get; }

    // ========================================================================
    // Mode
    // ========================================================================

    public enum VisionMode
    {
        Live,
        Tracking,
        Scanning,
        Calibration
    }

    [ObservableProperty]
    private VisionMode _currentMode = VisionMode.Live;

    [ObservableProperty]
    private string _modeDisplay = "Live View";

    // ========================================================================
    // Detection Config
    // ========================================================================

    [ObservableProperty]
    private string _jointType = "Auto";

    [ObservableProperty]
    private double _roiXMin = -50;

    [ObservableProperty]
    private double _roiXMax = 50;

    [ObservableProperty]
    private double _roiZMin = 0;

    [ObservableProperty]
    private double _roiZMax = 100;

    public List<string> JointTypes { get; } = new()
    {
        "Auto",
        "V-Groove",
        "Lap Joint",
        "Fillet"
    };

    // ========================================================================
    // State
    // ========================================================================

    [ObservableProperty]
    private bool _isConnected;

    [ObservableProperty]
    private bool _isTracking;

    [ObservableProperty]
    private bool _canStartTracking;

    [ObservableProperty]
    private bool _canStopTracking;

    public VisionControlViewModel(
        ISensorClientService sensorService,
        ISeamDetectionService seamService)
    {
        _sensorService = sensorService;
        _seamService = seamService;

        // Create sub-viewmodels
        ProfileDisplay = new ProfileDisplayViewModel(sensorService, seamService);
        TrackingStatus = new TrackingStatusViewModel(seamService);
        SensorStatus = new SensorStatusViewModel(sensorService);
        Calibration = new CalibrationViewModel(sensorService);
        ScanToPath = new ScanToPathViewModel(sensorService, seamService);

        // Wire up events
        _sensorService.ConnectionChanged += (s, e) =>
        {
            IsConnected = e.Connected;
            UpdateCanExecute();
        };

        _seamService.TrackingStateChanged += (s, e) =>
        {
            IsTracking = e.State.Tracking;
            UpdateCanExecute();
        };

        UpdateCanExecute();
    }

    [RelayCommand]
    private void SetMode(VisionMode mode)
    {
        CurrentMode = mode;
        ModeDisplay = mode switch
        {
            VisionMode.Live => "Live View",
            VisionMode.Tracking => "Seam Tracking",
            VisionMode.Scanning => "Scan-to-Path",
            VisionMode.Calibration => "Calibration",
            _ => "Unknown"
        };
    }

    [RelayCommand(CanExecute = nameof(CanStartTracking))]
    private async Task StartTrackingAsync()
    {
        // Apply detection config first
        var config = new SeamDetectionConfigData
        {
            JointType = JointType.ToLower().Replace("-", "").Replace(" ", ""),
            RoiXMin = (float)RoiXMin,
            RoiXMax = (float)RoiXMax,
            RoiZMin = (float)RoiZMin,
            RoiZMax = (float)RoiZMax,
            EnableTracking = true
        };

        await _seamService.ConfigureAsync(config);
        await _seamService.StartTrackingAsync();

        SetMode(VisionMode.Tracking);
    }

    [RelayCommand(CanExecute = nameof(CanStopTracking))]
    private async Task StopTrackingAsync()
    {
        await _seamService.StopTrackingAsync();
        TrackingStatus.Reset();
        SetMode(VisionMode.Live);
    }

    [RelayCommand]
    private async Task ResetTrackingAsync()
    {
        await _seamService.ResetTrackerAsync();
        TrackingStatus.Reset();
    }

    [RelayCommand]
    private void ShowCalibration()
    {
        SetMode(VisionMode.Calibration);
    }

    [RelayCommand]
    private void ShowScanToPath()
    {
        SetMode(VisionMode.Scanning);
    }

    private void UpdateCanExecute()
    {
        CanStartTracking = IsConnected && !IsTracking;
        CanStopTracking = IsTracking;

        StartTrackingCommand.NotifyCanExecuteChanged();
        StopTrackingCommand.NotifyCanExecuteChanged();
    }
}
```

---

## Step 2: Create Vision Control Views

### 2.1 Create ProfileDisplayControl.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/Vision/ProfileDisplayControl.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.Vision.ProfileDisplayControl"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             d:DesignHeight="300" d:DesignWidth="500">

    <Border Background="#1A1A1A" CornerRadius="8" Padding="12">
        <Grid>
            <Grid.RowDefinitions>
                <RowDefinition Height="Auto"/>
                <RowDefinition Height="*"/>
                <RowDefinition Height="Auto"/>
            </Grid.RowDefinitions>

            <!-- Header -->
            <Grid Grid.Row="0" Margin="0,0,0,8">
                <TextBlock Text="LASER PROFILE" FontSize="12" FontWeight="Bold"
                          Foreground="#888888"/>
                <StackPanel Orientation="Horizontal" HorizontalAlignment="Right">
                    <TextBlock Text="{Binding JointType}" Foreground="#00AAFF"
                              FontWeight="Bold" Margin="0,0,16,0"/>
                    <TextBlock Text="{Binding PointCount, StringFormat='{}{0} pts'}"
                              Foreground="#666666"/>
                </StackPanel>
            </Grid>

            <!-- Profile Canvas -->
            <Border Grid.Row="1" Background="#0A0A0A" CornerRadius="4">
                <Canvas x:Name="ProfileCanvas" ClipToBounds="True">
                    <!-- Grid lines -->
                    <Line X1="0" Y1="{Binding CanvasHeight, Converter={StaticResource HalfConverter}}"
                         X2="{Binding CanvasWidth}"
                         Y2="{Binding CanvasHeight, Converter={StaticResource HalfConverter}}"
                         Stroke="#333333" StrokeDashArray="4,2"
                         Visibility="{Binding ShowGrid, Converter={StaticResource BoolToVisibility}}"/>

                    <Line Y1="0" X1="{Binding CanvasWidth, Converter={StaticResource HalfConverter}}"
                         Y2="{Binding CanvasHeight}"
                         X2="{Binding CanvasWidth, Converter={StaticResource HalfConverter}}"
                         Stroke="#333333" StrokeDashArray="4,2"
                         Visibility="{Binding ShowGrid, Converter={StaticResource BoolToVisibility}}"/>

                    <!-- Profile polyline -->
                    <Polyline Points="{Binding ProfilePoints}"
                             Stroke="{Binding ProfileColor}"
                             StrokeThickness="1.5"/>

                    <!-- Seam detection overlay -->
                    <Canvas Visibility="{Binding ShowSeamOverlay, Converter={StaticResource BoolToVisibility}}">
                        <!-- Root point marker -->
                        <Ellipse Width="10" Height="10" Fill="#FF0000"
                                Canvas.Left="{Binding RootPoint.X}"
                                Canvas.Top="{Binding RootPoint.Y}"
                                Margin="-5,-5,0,0"
                                Visibility="{Binding HasValidFeature, Converter={StaticResource BoolToVisibility}}"/>

                        <!-- Left groove line -->
                        <Line X1="{Binding LeftLineStart.X}" Y1="{Binding LeftLineStart.Y}"
                             X2="{Binding LeftLineEnd.X}" Y2="{Binding LeftLineEnd.Y}"
                             Stroke="#FFAA00" StrokeThickness="2"
                             Visibility="{Binding HasValidFeature, Converter={StaticResource BoolToVisibility}}"/>

                        <!-- Right groove line -->
                        <Line X1="{Binding RightLineStart.X}" Y1="{Binding RightLineStart.Y}"
                             X2="{Binding RightLineEnd.X}" Y2="{Binding RightLineEnd.Y}"
                             Stroke="#FFAA00" StrokeThickness="2"
                             Visibility="{Binding HasValidFeature, Converter={StaticResource BoolToVisibility}}"/>
                    </Canvas>
                </Canvas>
            </Border>

            <!-- Feature info -->
            <Grid Grid.Row="2" Margin="0,8,0,0"
                 Visibility="{Binding HasValidFeature, Converter={StaticResource BoolToVisibility}}">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>

                <StackPanel Grid.Column="0">
                    <TextBlock Text="Gap Width" Foreground="#666666" FontSize="10"/>
                    <TextBlock Text="{Binding GapWidth, StringFormat='{}{0:F2} mm'}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>

                <StackPanel Grid.Column="1">
                    <TextBlock Text="Left Angle" Foreground="#666666" FontSize="10"/>
                    <TextBlock Text="{Binding LeftAngle, StringFormat='{}{0:F1}°'}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>

                <StackPanel Grid.Column="2">
                    <TextBlock Text="Right Angle" Foreground="#666666" FontSize="10"/>
                    <TextBlock Text="{Binding RightAngle, StringFormat='{}{0:F1}°'}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>

                <StackPanel Grid.Column="3">
                    <TextBlock Text="Confidence" Foreground="#666666" FontSize="10"/>
                    <TextBlock Text="{Binding Confidence, StringFormat='{}{0:P0}'}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>
            </Grid>

            <!-- Toolbar -->
            <StackPanel Grid.Row="2" Orientation="Horizontal"
                       HorizontalAlignment="Right" Margin="0,8,0,0">
                <Button Content="+" Width="24" Height="24" Margin="2"
                       Command="{Binding ZoomInCommand}"/>
                <Button Content="-" Width="24" Height="24" Margin="2"
                       Command="{Binding ZoomOutCommand}"/>
                <Button Content="⟲" Width="24" Height="24" Margin="2"
                       Command="{Binding ResetViewCommand}"/>
            </StackPanel>
        </Grid>
    </Border>
</UserControl>
```

### 2.2 Create TrackingStatusPanel.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/Vision/TrackingStatusPanel.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.Vision.TrackingStatusPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             d:DesignHeight="280" d:DesignWidth="280">

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <StackPanel>
            <!-- Header with status -->
            <Grid Margin="0,0,0,12">
                <TextBlock Text="TRACKING STATUS" FontSize="12" FontWeight="Bold"
                          Foreground="#888888"/>
                <Border HorizontalAlignment="Right"
                       Background="{Binding StatusColor}"
                       CornerRadius="4" Padding="12,4">
                    <TextBlock Text="{Binding TrackingStatus}"
                              FontWeight="Bold" Foreground="White"/>
                </Border>
            </Grid>

            <!-- Quality bar -->
            <StackPanel Margin="0,0,0,12">
                <Grid>
                    <TextBlock Text="Quality" Foreground="#888888"/>
                    <TextBlock Text="{Binding QualityPercent, StringFormat='{}{0}%'}"
                              Foreground="White" HorizontalAlignment="Right"/>
                </Grid>
                <ProgressBar Value="{Binding QualityPercent}" Maximum="100"
                            Height="8" Margin="0,4,0,0"
                            Background="#353535"
                            Foreground="{Binding QualityColor}"/>
            </StackPanel>

            <!-- Deviation display -->
            <Border Background="#252525" CornerRadius="4" Padding="12" Margin="0,0,0,12">
                <Grid>
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="*"/>
                    </Grid.ColumnDefinitions>

                    <StackPanel Grid.Column="0" HorizontalAlignment="Center">
                        <TextBlock Text="LATERAL" Foreground="#666666"
                                  FontSize="10" TextAlignment="Center"/>
                        <TextBlock Text="{Binding DeviationDisplay}"
                                  Foreground="{Binding DeviationColor}"
                                  FontFamily="Consolas" FontSize="20"
                                  FontWeight="Bold" TextAlignment="Center"/>
                    </StackPanel>

                    <StackPanel Grid.Column="1" HorizontalAlignment="Center">
                        <TextBlock Text="HEIGHT" Foreground="#666666"
                                  FontSize="10" TextAlignment="Center"/>
                        <TextBlock Text="{Binding HeightDisplay}"
                                  Foreground="White"
                                  FontFamily="Consolas" FontSize="20"
                                  FontWeight="Bold" TextAlignment="Center"/>
                    </StackPanel>
                </Grid>
            </Border>

            <!-- Prediction -->
            <Grid Margin="0,0,0,8">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>

                <StackPanel Grid.Column="0">
                    <TextBlock Text="Predicted X" Foreground="#666666" FontSize="10"/>
                    <TextBlock Text="{Binding PredictedX, StringFormat='{}{0:F2} mm'}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>

                <StackPanel Grid.Column="1">
                    <TextBlock Text="Predicted Z" Foreground="#666666" FontSize="10"/>
                    <TextBlock Text="{Binding PredictedZ, StringFormat='{}{0:F2} mm'}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>
            </Grid>

            <!-- Statistics -->
            <Rectangle Height="1" Fill="#404040" Margin="0,8"/>

            <Grid>
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>
                <Grid.RowDefinitions>
                    <RowDefinition Height="Auto"/>
                    <RowDefinition Height="Auto"/>
                </Grid.RowDefinitions>

                <StackPanel Grid.Row="0" Grid.Column="0">
                    <TextBlock Text="Frames" Foreground="#666666" FontSize="10"/>
                    <TextBlock Text="{Binding FramesDisplay}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>

                <StackPanel Grid.Row="0" Grid.Column="1">
                    <TextBlock Text="Latency" Foreground="#666666" FontSize="10"/>
                    <TextBlock Text="{Binding LatencyDisplay}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>

                <StackPanel Grid.Row="1" Grid.Column="0" Margin="0,8,0,0">
                    <TextBlock Text="Detection Rate" Foreground="#666666" FontSize="10"/>
                    <TextBlock Text="{Binding DetectionRateDisplay}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>

                <StackPanel Grid.Row="1" Grid.Column="1" Margin="0,8,0,0">
                    <TextBlock Text="Valid" Foreground="#666666" FontSize="10"/>
                    <TextBlock Text="{Binding DetectionsValid}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>
            </Grid>
        </StackPanel>
    </Border>
</UserControl>
```

### 2.3 Create SensorStatusPanel.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/Vision/SensorStatusPanel.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.Vision.SensorStatusPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             d:DesignHeight="300" d:DesignWidth="300">

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <StackPanel>
            <TextBlock Text="SENSOR STATUS" FontSize="12" FontWeight="Bold"
                      Foreground="#888888" Margin="0,0,0,12"/>

            <!-- Connection status -->
            <Grid Margin="0,0,0,8">
                <StackPanel Orientation="Horizontal">
                    <Ellipse Width="12" Height="12"
                            Fill="{Binding StatusColor}" Margin="0,0,8,0"/>
                    <TextBlock Text="{Binding ConnectionStatus}"
                              Foreground="White"/>
                </StackPanel>
            </Grid>

            <!-- Device selection -->
            <Grid Margin="0,0,0,8">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="Auto"/>
                </Grid.ColumnDefinitions>

                <ComboBox Grid.Column="0"
                         ItemsSource="{Binding AvailableDevices}"
                         SelectedItem="{Binding SelectedDevice}"/>
                <Button Grid.Column="1" Content="↻" Width="30" Margin="4,0,0,0"
                       Command="{Binding EnumerateDevicesCommand}"/>
            </Grid>

            <!-- Connect/Disconnect buttons -->
            <StackPanel Orientation="Horizontal" Margin="0,0,0,12">
                <Button Content="Connect" Padding="16,4" Margin="0,0,4,0"
                       Command="{Binding ConnectCommand}"
                       IsEnabled="{Binding IsConnected, Converter={StaticResource InverseBool}}"/>
                <Button Content="Disconnect" Padding="16,4"
                       Command="{Binding DisconnectCommand}"
                       IsEnabled="{Binding IsConnected}"/>
            </StackPanel>

            <Rectangle Height="1" Fill="#404040" Margin="0,0,0,12"/>

            <!-- Device info -->
            <StackPanel Visibility="{Binding IsConnected, Converter={StaticResource BoolToVisibility}}">
                <Grid Margin="0,0,0,4">
                    <TextBlock Text="Model:" Foreground="#888888"/>
                    <TextBlock Text="{Binding ModelName}" Foreground="White"
                              HorizontalAlignment="Right"/>
                </Grid>

                <Grid Margin="0,0,0,4">
                    <TextBlock Text="Serial:" Foreground="#888888"/>
                    <TextBlock Text="{Binding SerialNumber}" Foreground="White"
                              HorizontalAlignment="Right"/>
                </Grid>

                <Grid Margin="0,0,0,4">
                    <TextBlock Text="Frame Rate:" Foreground="#888888"/>
                    <TextBlock Text="{Binding FrameRateDisplay}" Foreground="White"
                              HorizontalAlignment="Right"/>
                </Grid>

                <Grid Margin="0,0,0,4">
                    <TextBlock Text="Frames:" Foreground="#888888"/>
                    <TextBlock Text="{Binding FramesDisplay}" Foreground="White"
                              HorizontalAlignment="Right"/>
                </Grid>

                <Grid Margin="0,0,0,8">
                    <TextBlock Text="Dropped:" Foreground="#888888"/>
                    <TextBlock Text="{Binding DroppedDisplay}" Foreground="#FF6666"
                              HorizontalAlignment="Right"/>
                </Grid>

                <!-- Acquisition controls -->
                <StackPanel Orientation="Horizontal" Margin="0,8,0,0">
                    <Button Content="Start" Padding="16,4" Margin="0,0,4,0"
                           Command="{Binding StartAcquisitionCommand}"
                           IsEnabled="{Binding IsAcquiring, Converter={StaticResource InverseBool}}"/>
                    <Button Content="Stop" Padding="16,4"
                           Command="{Binding StopAcquisitionCommand}"
                           IsEnabled="{Binding IsAcquiring}"/>
                </StackPanel>
            </StackPanel>

            <Rectangle Height="1" Fill="#404040" Margin="0,12"/>

            <!-- Configuration -->
            <Expander Header="Configuration" Foreground="#888888">
                <StackPanel Margin="0,8,0,0">
                    <Grid Margin="0,0,0,4">
                        <TextBlock Text="Exposure (µs):" Foreground="#888888"/>
                        <TextBox Text="{Binding ExposureTime}" Width="80"
                                HorizontalAlignment="Right"/>
                    </Grid>

                    <Grid Margin="0,0,0,4">
                        <TextBlock Text="Gain:" Foreground="#888888"/>
                        <TextBox Text="{Binding Gain}" Width="80"
                                HorizontalAlignment="Right"/>
                    </Grid>

                    <Grid Margin="0,0,0,4">
                        <TextBlock Text="Trigger:" Foreground="#888888"/>
                        <ComboBox ItemsSource="{Binding TriggerModes}"
                                 SelectedItem="{Binding TriggerMode}"
                                 Width="100" HorizontalAlignment="Right"/>
                    </Grid>

                    <Button Content="Apply" Padding="16,4" Margin="0,8,0,0"
                           Command="{Binding ApplyConfigCommand}"
                           HorizontalAlignment="Right"/>
                </StackPanel>
            </Expander>
        </StackPanel>
    </Border>
</UserControl>
```

### 2.4 Create VisionMainPanel.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/Vision/VisionMainPanel.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.Vision.VisionMainPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             xmlns:vision="clr-namespace:RobotController.UI.Views.Controls.Vision"
             d:DesignHeight="700" d:DesignWidth="1100">

    <Grid>
        <Grid.RowDefinitions>
            <RowDefinition Height="Auto"/>
            <RowDefinition Height="*"/>
            <RowDefinition Height="Auto"/>
        </Grid.RowDefinitions>

        <!-- Mode selector -->
        <Border Grid.Row="0" Background="#2D2D2D" CornerRadius="8"
               Padding="8" Margin="0,0,0,8">
            <StackPanel Orientation="Horizontal">
                <RadioButton Content="Live View" GroupName="Mode"
                            IsChecked="{Binding CurrentMode, Converter={StaticResource EnumToBool},
                                ConverterParameter=Live}"
                            Style="{StaticResource ModeRadioButton}"/>
                <RadioButton Content="Tracking" GroupName="Mode"
                            IsChecked="{Binding CurrentMode, Converter={StaticResource EnumToBool},
                                ConverterParameter=Tracking}"
                            Style="{StaticResource ModeRadioButton}"/>
                <RadioButton Content="Scan-to-Path" GroupName="Mode"
                            IsChecked="{Binding CurrentMode, Converter={StaticResource EnumToBool},
                                ConverterParameter=Scanning}"
                            Style="{StaticResource ModeRadioButton}"/>
                <RadioButton Content="Calibration" GroupName="Mode"
                            IsChecked="{Binding CurrentMode, Converter={StaticResource EnumToBool},
                                ConverterParameter=Calibration}"
                            Style="{StaticResource ModeRadioButton}"/>
            </StackPanel>
        </Border>

        <!-- Main content area -->
        <Grid Grid.Row="1">
            <Grid.ColumnDefinitions>
                <ColumnDefinition Width="*"/>
                <ColumnDefinition Width="300"/>
            </Grid.ColumnDefinitions>

            <!-- Left: Profile display -->
            <vision:ProfileDisplayControl Grid.Column="0"
                                         DataContext="{Binding ProfileDisplay}"
                                         Margin="0,0,8,0"/>

            <!-- Right: Status panels -->
            <StackPanel Grid.Column="1">
                <vision:TrackingStatusPanel DataContext="{Binding TrackingStatus}"
                                           Margin="0,0,0,8"/>
                <vision:SensorStatusPanel DataContext="{Binding SensorStatus}"/>
            </StackPanel>
        </Grid>

        <!-- Bottom: Control buttons -->
        <Border Grid.Row="2" Background="#2D2D2D" CornerRadius="8"
               Padding="12" Margin="0,8,0,0">
            <Grid>
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="Auto"/>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="Auto"/>
                </Grid.ColumnDefinitions>

                <!-- Tracking controls -->
                <StackPanel Grid.Column="0" Orientation="Horizontal">
                    <Button Content="START TRACKING" Padding="20,8"
                           Background="#00AA00" Foreground="White"
                           Command="{Binding StartTrackingCommand}"
                           Margin="0,0,8,0"/>
                    <Button Content="STOP" Padding="20,8"
                           Background="#AA0000" Foreground="White"
                           Command="{Binding StopTrackingCommand}"
                           Margin="0,0,8,0"/>
                    <Button Content="Reset" Padding="12,8"
                           Command="{Binding ResetTrackingCommand}"/>
                </StackPanel>

                <!-- Detection config -->
                <StackPanel Grid.Column="1" Orientation="Horizontal"
                           HorizontalAlignment="Center">
                    <TextBlock Text="Joint Type:" Foreground="#888888"
                              VerticalAlignment="Center" Margin="0,0,8,0"/>
                    <ComboBox ItemsSource="{Binding JointTypes}"
                             SelectedItem="{Binding JointType}"
                             Width="100"/>
                </StackPanel>

                <!-- Mode buttons -->
                <StackPanel Grid.Column="2" Orientation="Horizontal">
                    <Button Content="Calibrate" Padding="12,8"
                           Command="{Binding ShowCalibrationCommand}"
                           Margin="0,0,8,0"/>
                    <Button Content="Scan Path" Padding="12,8"
                           Command="{Binding ShowScanToPathCommand}"/>
                </StackPanel>
            </Grid>
        </Border>
    </Grid>
</UserControl>
```

---

## Step 3: Unit Tests

### 3.1 Create VisionViewModelTests.cs

**File:** `src/csharp/RobotController.Tests/ViewModels/VisionViewModelTests.cs`

```csharp
using Xunit;
using Moq;
using RobotController.UI.ViewModels.Vision;
using RobotController.Core.Services;
using System.Windows;

namespace RobotController.Tests.ViewModels;

public class ProfileDisplayViewModelTests
{
    [Fact]
    public void UpdateProfile_SetsPointCount()
    {
        var vm = new ProfileDisplayViewModel();

        var data = new ProfileReceivedEventArgs(
            new float[] { 0, 1, 2, 3, 4 },
            new float[] { 10, 11, 12, 13, 14 },
            new float[] { 100, 100, 100, 100, 100 },
            12345,
            1);

        vm.UpdateProfile(data);

        Assert.Equal(5, vm.PointCount);
        Assert.Equal(1ul, vm.FrameId);
    }

    [Fact]
    public void UpdateSeamFeature_VGroove_SetsAngles()
    {
        var vm = new ProfileDisplayViewModel();

        var feature = new SeamFeatureData
        {
            Valid = true,
            JointType = "V-Groove",
            RootX = 0,
            RootZ = 50,
            GapWidth = 3.0,
            LeftAngle = 45,
            RightAngle = 45,
            Confidence = 0.95
        };

        vm.UpdateSeamFeature(feature);

        Assert.True(vm.HasValidFeature);
        Assert.Equal("V-Groove", vm.JointType);
        Assert.Equal(3.0, vm.GapWidth);
        Assert.Equal(45, vm.LeftAngle);
    }

    [Fact]
    public void ResetView_ResetsScale()
    {
        var vm = new ProfileDisplayViewModel();
        vm.ScaleX = 5.0;
        vm.ScaleZ = 5.0;
        vm.AutoScale = false;

        vm.ResetViewCommand.Execute(null);

        Assert.Equal(2.0, vm.ScaleX);
        Assert.Equal(2.0, vm.ScaleZ);
        Assert.True(vm.AutoScale);
    }
}

public class TrackingStatusViewModelTests
{
    [Fact]
    public void UpdateState_Tracking_SetsGreenStatus()
    {
        var vm = new TrackingStatusViewModel();

        var state = new TrackingStateData
        {
            Tracking = true,
            TrackingLost = false,
            TrackingQuality = 0.85,
            LateralCorrection = 0.5,
            HeightCorrection = -0.2,
            FramesProcessed = 1000,
            DetectionsValid = 950,
            AvgLatency = 8.5
        };

        vm.UpdateState(state);

        Assert.True(vm.IsTracking);
        Assert.False(vm.IsTrackingLost);
        Assert.Equal("TRACKING", vm.TrackingStatus);
        Assert.Equal("#00FF00", vm.StatusColor);
        Assert.Equal(85, vm.QualityPercent);
    }

    [Fact]
    public void UpdateState_Lost_SetsRedStatus()
    {
        var vm = new TrackingStatusViewModel();

        var state = new TrackingStateData
        {
            Tracking = false,
            TrackingLost = true
        };

        vm.UpdateState(state);

        Assert.True(vm.IsTrackingLost);
        Assert.Equal("LOST", vm.TrackingStatus);
        Assert.Equal("#FF0000", vm.StatusColor);
    }

    [Fact]
    public void UpdateState_ComputesDetectionRate()
    {
        var vm = new TrackingStatusViewModel();

        var state = new TrackingStateData
        {
            FramesProcessed = 1000,
            DetectionsValid = 900
        };

        vm.UpdateState(state);

        Assert.Equal(90.0, vm.DetectionRate);
    }

    [Fact]
    public void Reset_ClearsAllValues()
    {
        var vm = new TrackingStatusViewModel();
        vm.IsTracking = true;
        vm.FramesProcessed = 1000;

        vm.Reset();

        Assert.False(vm.IsTracking);
        Assert.Equal(0ul, vm.FramesProcessed);
        Assert.Equal("IDLE", vm.TrackingStatus);
    }
}

public class CalibrationViewModelTests
{
    [Fact]
    public void Constructor_SetsInitialStep()
    {
        var sensorMock = new Mock<ISensorClientService>();
        var vm = new CalibrationViewModel(sensorMock.Object);

        Assert.Equal(0, vm.CurrentStep);
        Assert.Equal(5, vm.TotalSteps);
    }

    [Fact]
    public void DeletePose_RemovesAndReindexes()
    {
        var sensorMock = new Mock<ISensorClientService>();
        var vm = new CalibrationViewModel(sensorMock.Object);

        vm.CapturedPoses.Add(new CalibrationPoseEntry { Index = 1 });
        vm.CapturedPoses.Add(new CalibrationPoseEntry { Index = 2 });
        vm.CapturedPoses.Add(new CalibrationPoseEntry { Index = 3 });
        vm.CapturedPoseCount = 3;

        vm.DeletePoseCommand.Execute(vm.CapturedPoses[1]);

        Assert.Equal(2, vm.CapturedPoses.Count);
        Assert.Equal(1, vm.CapturedPoses[0].Index);
        Assert.Equal(2, vm.CapturedPoses[1].Index);
    }

    [Fact]
    public void Reset_ClearsAll()
    {
        var sensorMock = new Mock<ISensorClientService>();
        var vm = new CalibrationViewModel(sensorMock.Object);

        vm.CapturedPoses.Add(new CalibrationPoseEntry());
        vm.CalibrationComplete = true;
        vm.CurrentStep = 3;

        vm.ResetCommand.Execute(null);

        Assert.Empty(vm.CapturedPoses);
        Assert.False(vm.CalibrationComplete);
        Assert.Equal(0, vm.CurrentStep);
    }
}

public class ScanToPathViewModelTests
{
    [Fact]
    public void Constructor_SetsDefaults()
    {
        var sensorMock = new Mock<ISensorClientService>();
        var seamMock = new Mock<ISeamDetectionService>();

        var vm = new ScanToPathViewModel(sensorMock.Object, seamMock.Object);

        Assert.Equal(500u, vm.ProfileCount);
        Assert.Equal(30000u, vm.TimeoutMs);
        Assert.False(vm.IsScanning);
    }
}
```

---

## Completion Checklist

- [ ] ProfileDisplayViewModel.cs created
- [ ] TrackingStatusViewModel.cs created
- [ ] SensorStatusViewModel.cs created
- [ ] CalibrationViewModel.cs created
- [ ] ScanToPathViewModel.cs created
- [ ] VisionControlViewModel.cs created (main)
- [ ] ProfileDisplayControl.xaml created
- [ ] TrackingStatusPanel.xaml created
- [ ] SensorStatusPanel.xaml created
- [ ] VisionMainPanel.xaml created
- [ ] Unit tests created (15+ tests)
- [ ] MainWindow updated with Vision tab
- [ ] All tests pass

---

## Troubleshooting

### Profile Not Displaying
- Check sensor is connected and acquiring
- Verify IPC subscription to profile events
- Check canvas size and scaling

### Tracking Status Not Updating
- Verify seam detection service is running
- Check IPC event subscriptions
- Verify tracking is started

### Calibration Failing
- Need at least 3 poses for calibration
- Ensure calibration target is visible
- Check robot pose synchronization

---

## Keyboard Shortcuts (Vision)

| Key | Action |
|-----|--------|
| F7 | Start Tracking |
| F8 | Stop Tracking |
| C | Capture Pose (Calibration) |
| R | Reset View |
| +/- | Zoom In/Out |

---

## Git Commit

```powershell
git add -A
git commit -m "IMPL_P4_03: Add vision HMI controls

- Create ProfileDisplayViewModel for laser profile visualization
- Create TrackingStatusViewModel for seam tracking display
- Create SensorStatusViewModel for sensor connection/config
- Create CalibrationViewModel for hand-eye calibration wizard
- Create ScanToPathViewModel for scan-to-path workflow
- Add ProfileDisplayControl with seam overlay
- Add TrackingStatusPanel with quality and deviation display
- Add SensorStatusPanel with connection and config
- Add VisionMainPanel combining all vision controls
- Integrate with MainWindow as Vision tab
- Add 15+ unit tests for vision ViewModels
- Support live profile display with detected features

Co-Authored-By: Claude <noreply@anthropic.com>"
```
