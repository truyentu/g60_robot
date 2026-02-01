using CommunityToolkit.Mvvm.ComponentModel;
using System;

namespace RobotController.UI.ViewModels.Welding;

/// <summary>
/// View model for real-time welding feedback display
/// </summary>
public partial class WeldingFeedbackViewModel : ObservableObject
{
    // Electrical
    [ObservableProperty]
    private double _actualCurrent;

    [ObservableProperty]
    private double _actualVoltage;

    [ObservableProperty]
    private double _actualWireSpeed;

    [ObservableProperty]
    private double _actualPower;

    // Status indicators
    [ObservableProperty]
    private bool _arcPresent;

    [ObservableProperty]
    private bool _gasFlowOk;

    [ObservableProperty]
    private bool _wireFeeding;

    [ObservableProperty]
    private bool _powerSourceReady;

    // Timing
    [ObservableProperty]
    private TimeSpan _weldTime;

    [ObservableProperty]
    private double _weldDistance;

    [ObservableProperty]
    private double _wireConsumed;

    // Heat input
    [ObservableProperty]
    private double _heatInput;

    [ObservableProperty]
    private double _arcLength;

    // Display properties
    public string CurrentDisplay => $"{ActualCurrent:F1} A";
    public string VoltageDisplay => $"{ActualVoltage:F1} V";
    public string WireSpeedDisplay => $"{ActualWireSpeed:F1} m/min";
    public string PowerDisplay => $"{ActualPower:F2} kW";
    public string WeldTimeDisplay => $"{WeldTime:mm\\:ss}";
    public string DistanceDisplay => $"{WeldDistance:F1} mm";
    public string WireConsumedDisplay => $"{WireConsumed:F2} m";
    public string HeatInputDisplay => $"{HeatInput:F2} kJ/mm";

    public string ArcStatusText => ArcPresent ? "ARC OK" : "NO ARC";
    public string ArcStatusColor => ArcPresent ? "#00FF00" : "#FF6600";

    public string GasStatusText => GasFlowOk ? "GAS OK" : "NO GAS";
    public string GasStatusColor => GasFlowOk ? "#00FF00" : "#FF0000";

    public string WireStatusText => WireFeeding ? "WIRE OK" : "WIRE STOP";
    public string WireStatusColor => WireFeeding ? "#00FF00" : "#FFAA00";

    public void Update(
        double current,
        double voltage,
        double wireSpeed,
        bool arcPresent,
        bool gasOk,
        bool wireFeeding,
        uint weldTimeMs,
        double weldDistance,
        double wireConsumed)
    {
        ActualCurrent = current;
        ActualVoltage = voltage;
        ActualWireSpeed = wireSpeed;
        ActualPower = current * voltage / 1000.0;  // kW
        ArcPresent = arcPresent;
        GasFlowOk = gasOk;
        WireFeeding = wireFeeding;
        WeldTime = TimeSpan.FromMilliseconds(weldTimeMs);
        WeldDistance = weldDistance;
        WireConsumed = wireConsumed;

        // Calculate heat input (kJ/mm)
        // Heat Input = (Voltage × Current × 60) / (Travel Speed × 1000)
        // Simplified: Power / Travel Speed
        if (weldDistance > 0 && weldTimeMs > 0)
        {
            double travelSpeed = weldDistance / (weldTimeMs / 1000.0);  // mm/s
            if (travelSpeed > 0)
            {
                HeatInput = (ActualVoltage * ActualCurrent) / (travelSpeed * 1000.0);
            }
        }

        // Notify all display properties
        OnPropertyChanged(nameof(CurrentDisplay));
        OnPropertyChanged(nameof(VoltageDisplay));
        OnPropertyChanged(nameof(WireSpeedDisplay));
        OnPropertyChanged(nameof(PowerDisplay));
        OnPropertyChanged(nameof(WeldTimeDisplay));
        OnPropertyChanged(nameof(DistanceDisplay));
        OnPropertyChanged(nameof(WireConsumedDisplay));
        OnPropertyChanged(nameof(HeatInputDisplay));
        OnPropertyChanged(nameof(ArcStatusText));
        OnPropertyChanged(nameof(ArcStatusColor));
        OnPropertyChanged(nameof(GasStatusText));
        OnPropertyChanged(nameof(GasStatusColor));
        OnPropertyChanged(nameof(WireStatusText));
        OnPropertyChanged(nameof(WireStatusColor));
    }

    public void Reset()
    {
        ActualCurrent = 0;
        ActualVoltage = 0;
        ActualWireSpeed = 0;
        ActualPower = 0;
        ArcPresent = false;
        WireFeeding = false;
        WeldTime = TimeSpan.Zero;
        WeldDistance = 0;
        WireConsumed = 0;
        HeatInput = 0;
    }
}
