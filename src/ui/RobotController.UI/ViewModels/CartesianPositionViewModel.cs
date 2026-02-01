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
