using CommunityToolkit.Mvvm.ComponentModel;
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
