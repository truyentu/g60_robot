using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Services;
using System;
using System.Linq;
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

        if (_seamService != null)
        {
            _seamService.FeatureDetected += OnFeatureDetected;
        }
    }

    private void OnFeatureDetected(object? sender, SeamFeatureEventArgs e)
    {
        Application.Current?.Dispatcher.Invoke(() => UpdateSeamFeature(e.Feature));
    }

    public void UpdateProfile(float[] pointsX, float[] pointsZ, ulong frameId)
    {
        if (pointsX.Length == 0) return;

        var points = new PointCollection();

        // Auto-scale
        if (AutoScale && pointsZ.Length > 0)
        {
            float minX = pointsX.Min();
            float maxX = pointsX.Max();
            float minZ = pointsZ.Min();
            float maxZ = pointsZ.Max();

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
        for (int i = 0; i < pointsX.Length; i++)
        {
            double x = pointsX[i] * ScaleX + OffsetX;
            double z = -pointsZ[i] * ScaleZ + OffsetZ;  // Flip Z

            if (x >= 0 && x <= CanvasWidth && z >= 0 && z <= CanvasHeight)
            {
                points.Add(new Point(x, z));
            }
        }

        ProfilePoints = points;
        PointCount = pointsX.Length;
        FrameId = frameId;
    }

    public void UpdateSeamFeature(SeamFeatureData feature)
    {
        HasValidFeature = feature.Success;

        if (!feature.Success)
        {
            JointType = "Not detected";
            return;
        }

        JointType = feature.JointType.ToString();
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
        if (feature.JointType == Common.Services.JointType.VGroove)
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
