using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using System;

namespace RobotController.UI.ViewModels.Welding;

/// <summary>
/// View model for a single welding parameter with min/max limits
/// </summary>
public partial class WeldingParameterViewModel : ObservableObject
{
    [ObservableProperty]
    private string _name = "";

    [ObservableProperty]
    private string _unit = "";

    [ObservableProperty]
    private double _value;

    [ObservableProperty]
    private double _targetValue;

    [ObservableProperty]
    private double _minValue;

    [ObservableProperty]
    private double _maxValue;

    [ObservableProperty]
    private double _step = 1.0;

    [ObservableProperty]
    private bool _isReadOnly;

    [ObservableProperty]
    private bool _isHighlighted;

    // Display properties
    public string ValueDisplay => $"{Value:F1} {Unit}";
    public string TargetDisplay => $"{TargetValue:F1} {Unit}";
    public string RangeDisplay => $"{MinValue:F0} - {MaxValue:F0} {Unit}";

    public double ValuePercent =>
        (MaxValue - MinValue) > 0
            ? (Value - MinValue) / (MaxValue - MinValue) * 100
            : 50;

    public event EventHandler<double>? ValueChanged;

    public WeldingParameterViewModel(
        string name,
        string unit,
        double min,
        double max,
        double defaultValue,
        double step = 1.0)
    {
        Name = name;
        Unit = unit;
        MinValue = min;
        MaxValue = max;
        Value = defaultValue;
        TargetValue = defaultValue;
        Step = step;
    }

    public void UpdateActualValue(double actualValue)
    {
        Value = actualValue;
        OnPropertyChanged(nameof(ValueDisplay));
        OnPropertyChanged(nameof(ValuePercent));

        // Highlight if actual differs from target
        IsHighlighted = Math.Abs(Value - TargetValue) > Step * 2;
    }

    public void SetTarget(double target)
    {
        TargetValue = Math.Clamp(target, MinValue, MaxValue);
        OnPropertyChanged(nameof(TargetDisplay));
        ValueChanged?.Invoke(this, TargetValue);
    }

    [RelayCommand]
    private void Increase()
    {
        SetTarget(TargetValue + Step);
    }

    [RelayCommand]
    private void Decrease()
    {
        SetTarget(TargetValue - Step);
    }

    [RelayCommand]
    private void Reset()
    {
        SetTarget((MaxValue + MinValue) / 2);
    }
}
