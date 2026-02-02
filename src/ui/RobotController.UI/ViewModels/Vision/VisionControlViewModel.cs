using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Services;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace RobotController.UI.ViewModels.Vision;

/// <summary>
/// Main view model for vision control panel
/// </summary>
public partial class VisionControlViewModel : ObservableObject
{
    private readonly ISensorClientService? _sensorService;
    private readonly ISeamDetectionService? _seamService;

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
        ISensorClientService? sensorService = null,
        ISeamDetectionService? seamService = null)
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
        if (_seamService != null)
        {
            _seamService.TrackingStateChanged += (s, e) =>
            {
                IsTracking = e.State.TrackingActive;
                UpdateCanExecute();
            };
        }

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

    [RelayCommand]
    private async Task StartTrackingAsync()
    {
        if (_seamService == null) return;

        // Convert joint type string to enum
        var jointTypeEnum = JointType.ToLower().Replace("-", "").Replace(" ", "") switch
        {
            "vgroove" => Common.Services.JointType.VGroove,
            "lapjoint" => Common.Services.JointType.LapJoint,
            "fillet" => Common.Services.JointType.FilletJoint,
            _ => Common.Services.JointType.Unknown
        };

        var roi = new SeamRoiConfig
        {
            XMin = (float)RoiXMin,
            XMax = (float)RoiXMax,
            ZMin = (float)RoiZMin,
            ZMax = (float)RoiZMax
        };

        await _seamService.StartTrackingAsync(jointTypeEnum, roi);
        SetMode(VisionMode.Tracking);
    }

    [RelayCommand]
    private async Task StopTrackingAsync()
    {
        if (_seamService == null) return;

        await _seamService.StopTrackingAsync();
        TrackingStatus.Reset();
        SetMode(VisionMode.Live);
    }

    [RelayCommand]
    private async Task ResetTrackingAsync()
    {
        if (_seamService == null) return;

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
        CanStartTracking = !IsTracking;
        CanStopTracking = IsTracking;
    }
}
