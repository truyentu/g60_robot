using CommunityToolkit.Mvvm.ComponentModel;
using RobotController.Common.Services;
using System;
using System.Windows;

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
        Application.Current?.Dispatcher.Invoke(() => UpdateState(e.State));
    }

    public void UpdateState(TrackingStateData state)
    {
        IsTracking = state.TrackingActive;
        IsTrackingLost = state.TrackingLost;

        // Status display
        if (state.TrackingLost)
        {
            TrackingStatus = "LOST";
            StatusColor = "#FF0000";
        }
        else if (state.TrackingActive)
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
            > 0.8f => "#00FF00",
            > 0.5f => "#FFFF00",
            > 0.2f => "#FF8800",
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
        DetectionsValid = state.ValidDetections;
        AvgLatency = state.AvgLatencyMs;

        if (state.FramesProcessed > 0)
        {
            DetectionRate = (double)state.ValidDetections / state.FramesProcessed * 100;
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
