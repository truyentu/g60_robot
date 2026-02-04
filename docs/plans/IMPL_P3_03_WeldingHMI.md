# IMPL_P3_03: Welding HMI

| Metadata | Value |
|----------|-------|
| Plan ID | IMPL_P3_03 |
| Phase | 3 - Welding Integration |
| Priority | P1 |
| Depends On | IMPL_P3_01 (Welding Sequencer), IMPL_P3_02 (Weaving Patterns), IMPL_P2_05 (Motion HMI) |
| Status | Ready for Implementation |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/Thiết kế HMI Robot KUKA WPF.md` | MVVM pattern, industrial UI, control layouts |
| P0 | `ressearch_doc_md/Thiết Kế Module Điều Khiển Hàn MIG_MAG.md` | Welding parameters, monitoring UI requirements |

---

## Overview

Implementation plan cho Welding HMI - giao diện điều khiển hàn:
- **Parameter Panel:** Điều chỉnh current, voltage, wire speed, travel speed
- **Welding Monitor:** Real-time hiển thị feedback, arc status, gas flow
- **Weave Preview:** 2D preview của weave pattern
- **Job Management:** Load/Save welding jobs, presets
- **Timing Display:** Pre-flow, post-flow, crater fill timing
- **State Visualization:** FSM state display với transitions

---

## HMI Layout Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         WELDING CONTROL HMI                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌────────────────────┐  ┌────────────────────┐  ┌──────────────────┐  │
│  │   STATE DISPLAY    │  │   LIVE FEEDBACK    │  │   WEAVE PREVIEW  │  │
│  │                    │  │                    │  │                  │  │
│  │   ◉ WELDING       │  │  Current: 182.5 A  │  │    /\/\/\/\      │  │
│  │                    │  │  Voltage: 22.1 V   │  │   /      \     │  │
│  │  Pre → Arc → Weld  │  │  Wire:    8.2 m/min│  │  ───────────   │  │
│  │       → End → Post │  │  Power:   4.03 kW  │  │                  │  │
│  │                    │  │                    │  │  Pattern: Sine   │  │
│  │  Time: 00:45       │  │  ■ Arc OK          │  │  Amp: 3mm        │  │
│  │  Distance: 125mm   │  │  ■ Gas OK          │  │  Freq: 2Hz       │  │
│  └────────────────────┘  └────────────────────┘  └──────────────────┘  │
│                                                                          │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                      PARAMETER CONTROL                             │  │
│  │                                                                    │  │
│  │  Current       ◄──────────────●────────────────►  185 A          │  │
│  │  Voltage       ◄──────────────●────────────────►  22.0 V         │  │
│  │  Wire Speed    ◄──────────────●────────────────►  8.5 m/min      │  │
│  │  Travel Speed  ◄──────────────●────────────────►  12.0 mm/s      │  │
│  │                                                                    │  │
│  │  [Synergic Mode ✓]  Program: #12 Steel 1.0mm                      │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                          │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │  [ START WELD ]  [ STOP ]  [ E-STOP ]  │  Job: Steel_Butt_3mm    │ │
│  │                                         │  [ SAVE ] [ LOAD ]       │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

- [ ] IMPL_P3_01 (Welding Sequencer) đã hoàn thành
- [ ] IMPL_P3_02 (Weaving Patterns) đã hoàn thành
- [ ] IMPL_P2_05 (Motion HMI) đã hoàn thành
- [ ] WPF project builds successfully
- [ ] IPC communication working

---

## Step 1: Create Welding View Models

### 1.1 Create WeldingParameterViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/Welding/WeldingParameterViewModel.cs`

```csharp
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
```

### 1.2 Create WeldingFeedbackViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/Welding/WeldingFeedbackViewModel.cs`

```csharp
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
```

### 1.3 Create WeldingStateViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/Welding/WeldingStateViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using RobotController.Core.IPC;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels.Welding;

/// <summary>
/// View model for welding state machine visualization
/// </summary>
public partial class WeldingStateViewModel : ObservableObject
{
    [ObservableProperty]
    private WeldingState _currentState = WeldingState.Idle;

    [ObservableProperty]
    private WeldingFault _currentFault = WeldingFault.None;

    [ObservableProperty]
    private string _stateDisplay = "IDLE";

    [ObservableProperty]
    private string _stateColor = "#808080";

    [ObservableProperty]
    private string _faultDisplay = "";

    [ObservableProperty]
    private bool _hasFault;

    [ObservableProperty]
    private uint _stateTime;

    [ObservableProperty]
    private uint _arcRetries;

    // State indicators for FSM visualization
    [ObservableProperty]
    private bool _isIdle;

    [ObservableProperty]
    private bool _isPreFlow;

    [ObservableProperty]
    private bool _isArcStart;

    [ObservableProperty]
    private bool _isWelding;

    [ObservableProperty]
    private bool _isCraterFill;

    [ObservableProperty]
    private bool _isArcEnd;

    [ObservableProperty]
    private bool _isBurnBack;

    [ObservableProperty]
    private bool _isPostFlow;

    [ObservableProperty]
    private bool _isError;

    // State history for logging
    public ObservableCollection<StateLogEntry> StateHistory { get; } = new();

    public void UpdateState(WeldingState state, WeldingFault fault, uint stateTime)
    {
        var previousState = CurrentState;
        CurrentState = state;
        CurrentFault = fault;
        StateTime = stateTime;
        HasFault = fault != WeldingFault.None;

        // Update display
        UpdateStateDisplay(state);
        UpdateFaultDisplay(fault);
        UpdateStateIndicators(state);

        // Log state change
        if (previousState != state)
        {
            StateHistory.Insert(0, new StateLogEntry
            {
                Timestamp = DateTime.Now,
                FromState = previousState.ToString(),
                ToState = state.ToString()
            });

            // Keep only last 50 entries
            while (StateHistory.Count > 50)
            {
                StateHistory.RemoveAt(StateHistory.Count - 1);
            }
        }
    }

    private void UpdateStateDisplay(WeldingState state)
    {
        (StateDisplay, StateColor) = state switch
        {
            WeldingState.Idle => ("IDLE", "#808080"),
            WeldingState.PreFlow => ("PRE-FLOW", "#AAAA00"),
            WeldingState.ArcStart => ("ARC START", "#FF8800"),
            WeldingState.Welding => ("WELDING", "#00FF00"),
            WeldingState.CraterFill => ("CRATER FILL", "#00CC00"),
            WeldingState.ArcEnd => ("ARC END", "#FF8800"),
            WeldingState.BurnBack => ("BURN-BACK", "#AA6600"),
            WeldingState.PostFlow => ("POST-FLOW", "#AAAA00"),
            WeldingState.Error => ("ERROR", "#FF0000"),
            WeldingState.EmergencyStop => ("E-STOP", "#FF0000"),
            _ => (state.ToString(), "#808080")
        };
    }

    private void UpdateFaultDisplay(WeldingFault fault)
    {
        FaultDisplay = fault switch
        {
            WeldingFault.None => "",
            WeldingFault.NoGas => "FAULT: No gas flow",
            WeldingFault.NoWire => "FAULT: No wire",
            WeldingFault.ArcFail => "FAULT: Arc failed to start",
            WeldingFault.ArcLost => "FAULT: Arc lost",
            WeldingFault.OverCurrent => "FAULT: Over current",
            WeldingFault.OverVoltage => "FAULT: Over voltage",
            WeldingFault.StickWire => "FAULT: Wire stuck",
            WeldingFault.PowerSupplyFault => "FAULT: Power source error",
            WeldingFault.CommunicationFault => "FAULT: Communication lost",
            WeldingFault.SafetyFault => "FAULT: Safety interlock",
            WeldingFault.ThermalFault => "FAULT: Overheating",
            _ => $"FAULT: {fault}"
        };
    }

    private void UpdateStateIndicators(WeldingState state)
    {
        IsIdle = state == WeldingState.Idle;
        IsPreFlow = state == WeldingState.PreFlow;
        IsArcStart = state == WeldingState.ArcStart;
        IsWelding = state == WeldingState.Welding;
        IsCraterFill = state == WeldingState.CraterFill;
        IsArcEnd = state == WeldingState.ArcEnd;
        IsBurnBack = state == WeldingState.BurnBack;
        IsPostFlow = state == WeldingState.PostFlow;
        IsError = state == WeldingState.Error || state == WeldingState.EmergencyStop;
    }
}

public class StateLogEntry
{
    public DateTime Timestamp { get; init; }
    public string FromState { get; init; } = "";
    public string ToState { get; init; } = "";
    public string TimestampDisplay => Timestamp.ToString("HH:mm:ss.fff");
}
```

### 1.4 Create WeavePreviewViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/Welding/WeavePreviewViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Core.IPC;
using RobotController.Core.Services;
using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Media;

namespace RobotController.UI.ViewModels.Welding;

/// <summary>
/// View model for weave pattern preview
/// </summary>
public partial class WeavePreviewViewModel : ObservableObject
{
    private readonly IWeaveClientService? _weaveService;

    // Pattern selection
    [ObservableProperty]
    private WeavePatternType _selectedPattern = WeavePatternType.Sinusoidal;

    [ObservableProperty]
    private string _selectedPreset = "Medium Gap";

    // Parameters
    [ObservableProperty]
    private double _amplitude = 3.0;

    [ObservableProperty]
    private double _wavelength = 10.0;

    [ObservableProperty]
    private double _frequency = 1.0;

    [ObservableProperty]
    private double _dwellLeft;

    [ObservableProperty]
    private double _dwellRight;

    [ObservableProperty]
    private double _speedAtEdge = 0.8;

    [ObservableProperty]
    private bool _useFrequency;

    [ObservableProperty]
    private bool _isEnabled;

    // Preview data
    [ObservableProperty]
    private PointCollection _previewPoints = new();

    [ObservableProperty]
    private double _previewWidth = 200;

    [ObservableProperty]
    private double _previewHeight = 80;

    // Status
    [ObservableProperty]
    private bool _isActive;

    [ObservableProperty]
    private double _currentPhase;

    [ObservableProperty]
    private int _cycleCount;

    [ObservableProperty]
    private double _pathLengthRatio = 1.0;

    // Collections
    public ObservableCollection<WeavePatternType> PatternTypes { get; } = new()
    {
        WeavePatternType.None,
        WeavePatternType.Linear,
        WeavePatternType.Triangular,
        WeavePatternType.Sinusoidal,
        WeavePatternType.Circular,
        WeavePatternType.Figure8,
        WeavePatternType.Crescent
    };

    public ObservableCollection<string> Presets { get; } = new()
    {
        "Narrow Gap",
        "Medium Gap",
        "Wide Gap",
        "Vertical Up",
        "Overhead",
        "Root Pass",
        "Cap Pass"
    };

    public WeavePreviewViewModel(IWeaveClientService? weaveService = null)
    {
        _weaveService = weaveService;
        GeneratePreviewLocal();
    }

    partial void OnSelectedPatternChanged(WeavePatternType value)
    {
        GeneratePreviewLocal();
    }

    partial void OnAmplitudeChanged(double value)
    {
        GeneratePreviewLocal();
    }

    partial void OnWavelengthChanged(double value)
    {
        GeneratePreviewLocal();
    }

    [RelayCommand]
    private void ApplyPreset(string presetName)
    {
        SelectedPreset = presetName;

        switch (presetName)
        {
            case "Narrow Gap":
                SelectedPattern = WeavePatternType.Triangular;
                Amplitude = 2.0;
                Wavelength = 6.0;
                DwellLeft = 50;
                DwellRight = 50;
                break;

            case "Medium Gap":
                SelectedPattern = WeavePatternType.Sinusoidal;
                Amplitude = 4.0;
                Wavelength = 10.0;
                DwellLeft = 100;
                DwellRight = 100;
                break;

            case "Wide Gap":
                SelectedPattern = WeavePatternType.Triangular;
                Amplitude = 6.0;
                Wavelength = 12.0;
                DwellLeft = 150;
                DwellRight = 150;
                SpeedAtEdge = 0.6;
                break;

            case "Vertical Up":
                SelectedPattern = WeavePatternType.Crescent;
                Amplitude = 5.0;
                Wavelength = 8.0;
                DwellLeft = 200;
                DwellRight = 200;
                break;

            case "Overhead":
                SelectedPattern = WeavePatternType.Linear;
                Amplitude = 3.0;
                Wavelength = 6.0;
                DwellLeft = 100;
                DwellRight = 100;
                SpeedAtEdge = 0.7;
                break;

            case "Root Pass":
                SelectedPattern = WeavePatternType.Linear;
                Amplitude = 1.5;
                Wavelength = 4.0;
                DwellLeft = 0;
                DwellRight = 0;
                break;

            case "Cap Pass":
                SelectedPattern = WeavePatternType.Sinusoidal;
                Amplitude = 5.0;
                Wavelength = 8.0;
                DwellLeft = 80;
                DwellRight = 80;
                break;
        }

        GeneratePreviewLocal();
    }

    [RelayCommand]
    private async Task ApplyToControllerAsync()
    {
        if (_weaveService == null) return;

        var paramsData = new WeaveParamsData
        {
            PatternType = (int)SelectedPattern,
            Amplitude = Amplitude,
            Wavelength = Wavelength,
            Frequency = Frequency,
            DwellLeft = DwellLeft,
            DwellRight = DwellRight,
            SpeedAtEdge = SpeedAtEdge,
            SpeedAtCenter = 1.0,
            UseFrequency = UseFrequency
        };

        await _weaveService.SetParamsAsync(paramsData);
    }

    [RelayCommand]
    private async Task EnableWeaveAsync()
    {
        if (_weaveService == null) return;
        await _weaveService.EnableAsync(true);
        IsEnabled = true;
    }

    [RelayCommand]
    private async Task DisableWeaveAsync()
    {
        if (_weaveService == null) return;
        await _weaveService.EnableAsync(false);
        IsEnabled = false;
    }

    public void UpdateStatus(WeaveStatusResponse status)
    {
        IsEnabled = status.Enabled;
        IsActive = status.Active;
        CurrentPhase = status.CurrentPhase;
        CycleCount = status.CycleCount;
    }

    private void GeneratePreviewLocal()
    {
        var points = new PointCollection();
        int numCycles = 3;
        int pointsPerCycle = 32;
        int totalPoints = numCycles * pointsPerCycle;

        double scaleX = PreviewWidth / (numCycles * Wavelength);
        double scaleY = PreviewHeight / (Amplitude * 2.5);
        double centerY = PreviewHeight / 2;

        for (int i = 0; i < totalPoints; i++)
        {
            double progress = (double)i / totalPoints;
            double phase = (progress * numCycles) % 1.0;
            double x = progress * PreviewWidth;
            double y = centerY;

            switch (SelectedPattern)
            {
                case WeavePatternType.Sinusoidal:
                    y = centerY - Amplitude * Math.Sin(2 * Math.PI * phase) * scaleY;
                    break;

                case WeavePatternType.Linear:
                case WeavePatternType.Triangular:
                    if (phase < 0.5)
                        y = centerY - Amplitude * (phase / 0.5 * 2 - 1) * scaleY;
                    else
                        y = centerY - Amplitude * (1 - (phase - 0.5) / 0.5 * 2) * scaleY;
                    break;

                case WeavePatternType.Circular:
                    y = centerY - Amplitude * Math.Cos(2 * Math.PI * phase) * scaleY;
                    break;

                case WeavePatternType.Figure8:
                    y = centerY - Amplitude * Math.Sin(2 * Math.PI * phase) * scaleY;
                    break;

                case WeavePatternType.Crescent:
                    if (phase < 0.5)
                        y = centerY - Amplitude * Math.Sin(Math.PI * phase / 0.5) * scaleY;
                    else
                        y = centerY + Amplitude * Math.Sin(Math.PI * (phase - 0.5) / 0.5) * scaleY;
                    break;
            }

            points.Add(new Point(x, y));
        }

        PreviewPoints = points;

        // Approximate path length ratio
        PathLengthRatio = SelectedPattern switch
        {
            WeavePatternType.None => 1.0,
            WeavePatternType.Linear or WeavePatternType.Triangular =>
                Math.Sqrt(1 + Math.Pow(4 * Amplitude / Wavelength, 2)),
            WeavePatternType.Sinusoidal =>
                Math.Sqrt(1 + Math.Pow(2 * Math.PI * Amplitude / Wavelength, 2) / 2),
            WeavePatternType.Circular => 2 * Math.PI * Amplitude / Wavelength,
            _ => 1.2
        };
    }
}
```

### 1.5 Create WeldingJobViewModel.cs

**File:** `src/csharp/RobotController.UI/ViewModels/Welding/WeldingJobViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Core.IPC;
using System.Collections.ObjectModel;
using System.Text.Json;

namespace RobotController.UI.ViewModels.Welding;

/// <summary>
/// View model for welding job management
/// </summary>
public partial class WeldingJobViewModel : ObservableObject
{
    [ObservableProperty]
    private string _jobName = "Default";

    [ObservableProperty]
    private WeldingProcess _process = WeldingProcess.MigMag;

    [ObservableProperty]
    private int _transferMode;

    [ObservableProperty]
    private bool _synergicMode = true;

    [ObservableProperty]
    private int _synergicProgram = 1;

    // Timing settings
    [ObservableProperty]
    private uint _preFlowTime = 300;

    [ObservableProperty]
    private uint _postFlowTime = 500;

    [ObservableProperty]
    private uint _craterFillTime = 500;

    [ObservableProperty]
    private uint _burnBackTime = 50;

    [ObservableProperty]
    private uint _arcRetryDelay = 200;

    [ObservableProperty]
    private int _arcRetryCount = 3;

    // Gas settings
    [ObservableProperty]
    private string _gasType = "Ar/CO2 82/18";

    [ObservableProperty]
    private double _gasFlowRate = 15.0;

    // Wire settings
    [ObservableProperty]
    private string _wireMaterial = "ER70S-6";

    [ObservableProperty]
    private double _wireDiameter = 1.0;

    // Job list
    public ObservableCollection<string> SavedJobs { get; } = new()
    {
        "Default",
        "Steel_Butt_3mm",
        "Steel_Fillet_5mm",
        "Aluminum_Butt_2mm"
    };

    [ObservableProperty]
    private string? _selectedJob;

    [ObservableProperty]
    private bool _isDirty;

    // Process options
    public ObservableCollection<WeldingProcess> ProcessOptions { get; } = new()
    {
        WeldingProcess.MigMag,
        WeldingProcess.Tig,
        WeldingProcess.Spot,
        WeldingProcess.Plasma
    };

    public ObservableCollection<string> GasTypes { get; } = new()
    {
        "Ar/CO2 82/18",
        "Ar/CO2 75/25",
        "Pure Argon",
        "Pure CO2",
        "Ar/O2 98/2"
    };

    public ObservableCollection<string> WireMaterials { get; } = new()
    {
        "ER70S-6",
        "ER70S-3",
        "ER308L",
        "ER316L",
        "ER4043"
    };

    public ObservableCollection<double> WireDiameters { get; } = new()
    {
        0.8, 0.9, 1.0, 1.2, 1.6
    };

    public WeldingJobData ToJobData(
        float current,
        float voltage,
        float wireSpeed,
        float travelSpeed)
    {
        return new WeldingJobData
        {
            Name = JobName,
            Process = (int)Process,
            TransferMode = TransferMode,
            Current = current,
            Voltage = voltage,
            WireSpeed = wireSpeed,
            TravelSpeed = travelSpeed,
            PreFlowTime = PreFlowTime,
            PostFlowTime = PostFlowTime,
            CraterFillTime = CraterFillTime,
            SynergicMode = SynergicMode,
            SynergicProgram = SynergicProgram
        };
    }

    public void LoadFromJobData(WeldingJobData data)
    {
        JobName = data.Name;
        Process = (WeldingProcess)data.Process;
        TransferMode = data.TransferMode;
        PreFlowTime = data.PreFlowTime;
        PostFlowTime = data.PostFlowTime;
        CraterFillTime = data.CraterFillTime;
        SynergicMode = data.SynergicMode;
        SynergicProgram = data.SynergicProgram;
        IsDirty = false;
    }

    [RelayCommand]
    private void NewJob()
    {
        JobName = "New Job";
        PreFlowTime = 300;
        PostFlowTime = 500;
        CraterFillTime = 500;
        BurnBackTime = 50;
        SynergicMode = true;
        SynergicProgram = 1;
        IsDirty = true;
    }

    [RelayCommand]
    private async Task SaveJobAsync()
    {
        // Save to file
        string jobsPath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "WeldingJobs");

        Directory.CreateDirectory(jobsPath);

        string filePath = Path.Combine(jobsPath, $"{JobName}.json");

        var jobData = new
        {
            Name = JobName,
            Process = Process,
            TransferMode = TransferMode,
            SynergicMode = SynergicMode,
            SynergicProgram = SynergicProgram,
            PreFlowTime = PreFlowTime,
            PostFlowTime = PostFlowTime,
            CraterFillTime = CraterFillTime,
            BurnBackTime = BurnBackTime,
            ArcRetryDelay = ArcRetryDelay,
            ArcRetryCount = ArcRetryCount,
            GasType = GasType,
            GasFlowRate = GasFlowRate,
            WireMaterial = WireMaterial,
            WireDiameter = WireDiameter
        };

        string json = JsonSerializer.Serialize(jobData, new JsonSerializerOptions
        {
            WriteIndented = true
        });

        await File.WriteAllTextAsync(filePath, json);

        if (!SavedJobs.Contains(JobName))
        {
            SavedJobs.Add(JobName);
        }

        IsDirty = false;
    }

    [RelayCommand]
    private async Task LoadJobAsync(string? jobName)
    {
        if (string.IsNullOrEmpty(jobName)) return;

        string jobsPath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "WeldingJobs");

        string filePath = Path.Combine(jobsPath, $"{jobName}.json");

        if (!File.Exists(filePath)) return;

        string json = await File.ReadAllTextAsync(filePath);
        var jobData = JsonSerializer.Deserialize<JsonElement>(json);

        JobName = jobData.GetProperty("Name").GetString() ?? jobName;
        Process = (WeldingProcess)jobData.GetProperty("Process").GetInt32();
        SynergicMode = jobData.GetProperty("SynergicMode").GetBoolean();
        PreFlowTime = jobData.GetProperty("PreFlowTime").GetUInt32();
        PostFlowTime = jobData.GetProperty("PostFlowTime").GetUInt32();
        CraterFillTime = jobData.GetProperty("CraterFillTime").GetUInt32();

        if (jobData.TryGetProperty("GasType", out var gasType))
            GasType = gasType.GetString() ?? "Ar/CO2 82/18";

        if (jobData.TryGetProperty("WireMaterial", out var wireMat))
            WireMaterial = wireMat.GetString() ?? "ER70S-6";

        SelectedJob = JobName;
        IsDirty = false;
    }

    [RelayCommand]
    private void DeleteJob(string? jobName)
    {
        if (string.IsNullOrEmpty(jobName) || jobName == "Default") return;

        string jobsPath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "WeldingJobs");

        string filePath = Path.Combine(jobsPath, $"{jobName}.json");

        if (File.Exists(filePath))
        {
            File.Delete(filePath);
        }

        SavedJobs.Remove(jobName);

        if (JobName == jobName)
        {
            NewJob();
        }
    }

    // Mark as dirty when any property changes
    partial void OnPreFlowTimeChanged(uint value) => IsDirty = true;
    partial void OnPostFlowTimeChanged(uint value) => IsDirty = true;
    partial void OnCraterFillTimeChanged(uint value) => IsDirty = true;
    partial void OnSynergicModeChanged(bool value) => IsDirty = true;
}
```

### 1.6 Create WeldingControlViewModel.cs (Main)

**File:** `src/csharp/RobotController.UI/ViewModels/Welding/WeldingControlViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Core.IPC;
using RobotController.Core.Services;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels.Welding;

/// <summary>
/// Main view model for welding control panel
/// </summary>
public partial class WeldingControlViewModel : ObservableObject
{
    private readonly IWeldingClientService _weldingService;
    private readonly IWeaveClientService? _weaveService;

    // ========================================================================
    // Sub-ViewModels
    // ========================================================================

    public WeldingStateViewModel State { get; } = new();
    public WeldingFeedbackViewModel Feedback { get; } = new();
    public WeavePreviewViewModel WeavePreview { get; }
    public WeldingJobViewModel Job { get; } = new();

    // ========================================================================
    // Parameters
    // ========================================================================

    public WeldingParameterViewModel Current { get; }
    public WeldingParameterViewModel Voltage { get; }
    public WeldingParameterViewModel WireSpeed { get; }
    public WeldingParameterViewModel TravelSpeed { get; }

    public ObservableCollection<WeldingParameterViewModel> Parameters { get; } = new();

    // ========================================================================
    // Control State
    // ========================================================================

    [ObservableProperty]
    private bool _isConnected;

    [ObservableProperty]
    private bool _isReady;

    [ObservableProperty]
    private bool _isWelding;

    [ObservableProperty]
    private bool _canStartWeld;

    [ObservableProperty]
    private bool _canStopWeld;

    // ========================================================================
    // Constructor
    // ========================================================================

    public WeldingControlViewModel(
        IWeldingClientService weldingService,
        IWeaveClientService? weaveService = null)
    {
        _weldingService = weldingService;
        _weaveService = weaveService;

        WeavePreview = new WeavePreviewViewModel(weaveService);

        // Initialize parameters
        Current = new WeldingParameterViewModel("Current", "A", 50, 400, 180, 5);
        Voltage = new WeldingParameterViewModel("Voltage", "V", 14, 40, 22, 0.5);
        WireSpeed = new WeldingParameterViewModel("Wire Speed", "m/min", 2, 20, 8, 0.5);
        TravelSpeed = new WeldingParameterViewModel("Travel Speed", "mm/s", 1, 50, 10, 1);

        Parameters.Add(Current);
        Parameters.Add(Voltage);
        Parameters.Add(WireSpeed);
        Parameters.Add(TravelSpeed);

        // Wire up parameter change events
        Current.ValueChanged += async (s, v) => await AdjustParameterAsync("current", v);
        Voltage.ValueChanged += async (s, v) => await AdjustParameterAsync("voltage", v);
        WireSpeed.ValueChanged += async (s, v) => await AdjustParameterAsync("wireSpeed", v);
        TravelSpeed.ValueChanged += async (s, v) => await AdjustParameterAsync("travelSpeed", v);

        // Subscribe to service events
        _weldingService.StatusUpdated += OnStatusUpdated;
        _weldingService.FaultOccurred += OnFaultOccurred;
        _weldingService.StateChanged += OnStateChanged;

        UpdateCanExecute();
    }

    // ========================================================================
    // Commands
    // ========================================================================

    [RelayCommand(CanExecute = nameof(CanStartWeld))]
    private async Task StartWeldAsync()
    {
        var jobData = Job.ToJobData(
            (float)Current.TargetValue,
            (float)Voltage.TargetValue,
            (float)WireSpeed.TargetValue,
            (float)TravelSpeed.TargetValue);

        var result = await _weldingService.StartWeldAsync(jobData);

        if (!result.Success)
        {
            // Handle error
            State.UpdateState(WeldingState.Error, WeldingFault.None, 0);
        }
    }

    [RelayCommand(CanExecute = nameof(CanStopWeld))]
    private async Task StopWeldAsync()
    {
        await _weldingService.StopWeldAsync();
    }

    [RelayCommand]
    private async Task AbortWeldAsync()
    {
        await _weldingService.AbortWeldAsync();
    }

    [RelayCommand]
    private async Task ResetFaultAsync()
    {
        await _weldingService.ResetAsync();
    }

    [RelayCommand]
    private async Task RefreshStatusAsync()
    {
        var status = await _weldingService.GetStatusAsync();
        ProcessStatus(status);
    }

    // ========================================================================
    // Parameter Adjustment
    // ========================================================================

    private async Task AdjustParameterAsync(string parameter, double value)
    {
        if (!IsWelding) return;

        WeldingAdjustResponse? result = parameter switch
        {
            "current" => await _weldingService.AdjustCurrentAsync((float)value),
            "voltage" => await _weldingService.AdjustVoltageAsync((float)value),
            "wireSpeed" => await _weldingService.AdjustWireSpeedAsync((float)value),
            "travelSpeed" => await _weldingService.AdjustTravelSpeedAsync((float)value),
            _ => null
        };
    }

    // ========================================================================
    // Event Handlers
    // ========================================================================

    private void OnStatusUpdated(object? sender, WeldingStatusResponse status)
    {
        App.Current.Dispatcher.Invoke(() => ProcessStatus(status));
    }

    private void OnFaultOccurred(object? sender, WeldingFault fault)
    {
        App.Current.Dispatcher.Invoke(() =>
        {
            State.UpdateState(WeldingState.Error, fault, 0);
            UpdateCanExecute();
        });
    }

    private void OnStateChanged(object? sender, WeldingState state)
    {
        App.Current.Dispatcher.Invoke(() =>
        {
            IsWelding = state == WeldingState.Welding || state == WeldingState.CraterFill;
            UpdateCanExecute();
        });
    }

    private void ProcessStatus(WeldingStatusResponse status)
    {
        // Parse state
        if (Enum.TryParse<WeldingState>(status.State, out var state))
        {
            WeldingFault fault = WeldingFault.None;
            if (Enum.TryParse<WeldingFault>(status.Fault, out var f))
            {
                fault = f;
            }

            State.UpdateState(state, fault, status.WeldTime);
        }

        // Update feedback
        Feedback.Update(
            status.ActualCurrent,
            status.ActualVoltage,
            status.ActualWireSpeed,
            status.ArcPresent,
            status.GasFlowOk,
            status.ArcPresent,  // Wire feeding if arc present
            status.WeldTime,
            status.WeldDistance,
            0  // Wire consumed not in status
        );

        // Update actual parameter values
        Current.UpdateActualValue(status.ActualCurrent);
        Voltage.UpdateActualValue(status.ActualVoltage);
        WireSpeed.UpdateActualValue(status.ActualWireSpeed);

        // Update flags
        IsReady = status.IsReady;
        IsWelding = status.IsWelding;

        UpdateCanExecute();
    }

    private void UpdateCanExecute()
    {
        CanStartWeld = IsConnected && IsReady && !IsWelding && !State.HasFault;
        CanStopWeld = IsWelding;

        StartWeldCommand.NotifyCanExecuteChanged();
        StopWeldCommand.NotifyCanExecuteChanged();
    }

    // ========================================================================
    // Public Methods
    // ========================================================================

    public void SetConnected(bool connected)
    {
        IsConnected = connected;
        UpdateCanExecute();
    }
}
```

---

## Step 2: Create Welding Control Views

### 2.1 Create WeldingStateDisplay.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/Welding/WeldingStateDisplay.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.Welding.WeldingStateDisplay"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             d:DesignHeight="200" d:DesignWidth="280">

    <UserControl.Resources>
        <!-- State indicator style -->
        <Style x:Key="StateIndicator" TargetType="Border">
            <Setter Property="Width" Value="24"/>
            <Setter Property="Height" Value="24"/>
            <Setter Property="CornerRadius" Value="12"/>
            <Setter Property="Margin" Value="2"/>
            <Setter Property="Background" Value="#404040"/>
        </Style>

        <Style x:Key="StateIndicatorActive" TargetType="Border"
               BasedOn="{StaticResource StateIndicator}">
            <Setter Property="Background" Value="#00FF00"/>
            <Setter Property="Effect">
                <Setter.Value>
                    <DropShadowEffect Color="#00FF00" BlurRadius="8"
                                     ShadowDepth="0" Opacity="0.6"/>
                </Setter.Value>
            </Setter>
        </Style>
    </UserControl.Resources>

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <StackPanel>
            <!-- Header with current state -->
            <Grid Margin="0,0,0,12">
                <TextBlock Text="WELDING STATE"
                          FontSize="12" FontWeight="Bold"
                          Foreground="#888888"/>
                <Border HorizontalAlignment="Right"
                       Background="{Binding StateColor}"
                       CornerRadius="4" Padding="12,4">
                    <TextBlock Text="{Binding StateDisplay}"
                              FontSize="14" FontWeight="Bold"
                              Foreground="White"/>
                </Border>
            </Grid>

            <!-- State Machine Visualization -->
            <Border Background="#252525" CornerRadius="4" Padding="8" Margin="0,0,0,8">
                <Grid>
                    <Grid.RowDefinitions>
                        <RowDefinition Height="Auto"/>
                        <RowDefinition Height="Auto"/>
                    </Grid.RowDefinitions>

                    <!-- Top row: Pre-flow -> Arc Start -> Welding -> Crater -->
                    <StackPanel Grid.Row="0" Orientation="Horizontal"
                               HorizontalAlignment="Center">
                        <Border Style="{Binding IsPreFlow,
                                Converter={StaticResource BoolToStyleConverter},
                                ConverterParameter='StateIndicatorActive|StateIndicator'}">
                            <TextBlock Text="P" Foreground="White" FontWeight="Bold"
                                      HorizontalAlignment="Center" VerticalAlignment="Center"/>
                        </Border>
                        <TextBlock Text="→" Foreground="#606060" VerticalAlignment="Center" Margin="2"/>

                        <Border Style="{Binding IsArcStart,
                                Converter={StaticResource BoolToStyleConverter},
                                ConverterParameter='StateIndicatorActive|StateIndicator'}">
                            <TextBlock Text="A" Foreground="White" FontWeight="Bold"
                                      HorizontalAlignment="Center" VerticalAlignment="Center"/>
                        </Border>
                        <TextBlock Text="→" Foreground="#606060" VerticalAlignment="Center" Margin="2"/>

                        <Border Width="32" Height="32" CornerRadius="16" Margin="2"
                               Background="{Binding IsWelding,
                                   Converter={StaticResource BoolToColorConverter},
                                   ConverterParameter='#00FF00|#404040'}">
                            <TextBlock Text="W" Foreground="White" FontWeight="Bold"
                                      FontSize="14"
                                      HorizontalAlignment="Center" VerticalAlignment="Center"/>
                        </Border>
                        <TextBlock Text="→" Foreground="#606060" VerticalAlignment="Center" Margin="2"/>

                        <Border Style="{Binding IsCraterFill,
                                Converter={StaticResource BoolToStyleConverter},
                                ConverterParameter='StateIndicatorActive|StateIndicator'}">
                            <TextBlock Text="C" Foreground="White" FontWeight="Bold"
                                      HorizontalAlignment="Center" VerticalAlignment="Center"/>
                        </Border>
                    </StackPanel>

                    <!-- Bottom row: Arc End -> Burn Back -> Post Flow -->
                    <StackPanel Grid.Row="1" Orientation="Horizontal"
                               HorizontalAlignment="Center" Margin="0,4,0,0">
                        <Border Style="{Binding IsArcEnd,
                                Converter={StaticResource BoolToStyleConverter},
                                ConverterParameter='StateIndicatorActive|StateIndicator'}">
                            <TextBlock Text="E" Foreground="White" FontWeight="Bold"
                                      HorizontalAlignment="Center" VerticalAlignment="Center"/>
                        </Border>
                        <TextBlock Text="→" Foreground="#606060" VerticalAlignment="Center" Margin="2"/>

                        <Border Style="{Binding IsBurnBack,
                                Converter={StaticResource BoolToStyleConverter},
                                ConverterParameter='StateIndicatorActive|StateIndicator'}">
                            <TextBlock Text="B" Foreground="White" FontWeight="Bold"
                                      HorizontalAlignment="Center" VerticalAlignment="Center"/>
                        </Border>
                        <TextBlock Text="→" Foreground="#606060" VerticalAlignment="Center" Margin="2"/>

                        <Border Style="{Binding IsPostFlow,
                                Converter={StaticResource BoolToStyleConverter},
                                ConverterParameter='StateIndicatorActive|StateIndicator'}">
                            <TextBlock Text="F" Foreground="White" FontWeight="Bold"
                                      HorizontalAlignment="Center" VerticalAlignment="Center"/>
                        </Border>
                    </StackPanel>
                </Grid>
            </Border>

            <!-- Legend -->
            <TextBlock Text="P=PreFlow A=ArcStart W=Weld C=Crater E=End B=Burn F=PostFlow"
                      Foreground="#606060" FontSize="9"
                      TextAlignment="Center" Margin="0,0,0,8"/>

            <!-- State time -->
            <Grid>
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>

                <StackPanel Grid.Column="0">
                    <TextBlock Text="State Time" Foreground="#888888" FontSize="10"/>
                    <TextBlock Text="{Binding StateTime, StringFormat={}{0} ms}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>

                <StackPanel Grid.Column="1">
                    <TextBlock Text="Arc Retries" Foreground="#888888" FontSize="10"/>
                    <TextBlock Text="{Binding ArcRetries}"
                              Foreground="White" FontFamily="Consolas"/>
                </StackPanel>
            </Grid>

            <!-- Fault display -->
            <Border Background="#660000" CornerRadius="4" Padding="8" Margin="0,8,0,0"
                   Visibility="{Binding HasFault, Converter={StaticResource BoolToVisibility}}">
                <TextBlock Text="{Binding FaultDisplay}"
                          Foreground="#FF6666" FontWeight="Bold"
                          TextWrapping="Wrap"/>
            </Border>
        </StackPanel>
    </Border>
</UserControl>
```

### 2.2 Create WeldingFeedbackPanel.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/Welding/WeldingFeedbackPanel.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.Welding.WeldingFeedbackPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             d:DesignHeight="280" d:DesignWidth="300">

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <StackPanel>
            <TextBlock Text="LIVE FEEDBACK"
                      FontSize="12" FontWeight="Bold"
                      Foreground="#888888" Margin="0,0,0,12"/>

            <!-- Main readings -->
            <Grid>
                <Grid.RowDefinitions>
                    <RowDefinition Height="Auto"/>
                    <RowDefinition Height="Auto"/>
                    <RowDefinition Height="Auto"/>
                    <RowDefinition Height="Auto"/>
                </Grid.RowDefinitions>
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="Auto"/>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="Auto"/>
                </Grid.ColumnDefinitions>

                <!-- Current -->
                <TextBlock Grid.Row="0" Grid.Column="0" Text="Current:"
                          Foreground="#888888" Margin="0,4"/>
                <ProgressBar Grid.Row="0" Grid.Column="1"
                            Value="{Binding ActualCurrent}"
                            Maximum="400" Height="8" Margin="8,4"
                            Background="#353535" Foreground="#FF8800"/>
                <TextBlock Grid.Row="0" Grid.Column="2"
                          Text="{Binding CurrentDisplay}"
                          Foreground="White" FontFamily="Consolas"
                          FontSize="14" FontWeight="Bold"
                          Width="80" TextAlignment="Right"/>

                <!-- Voltage -->
                <TextBlock Grid.Row="1" Grid.Column="0" Text="Voltage:"
                          Foreground="#888888" Margin="0,4"/>
                <ProgressBar Grid.Row="1" Grid.Column="1"
                            Value="{Binding ActualVoltage}"
                            Maximum="40" Height="8" Margin="8,4"
                            Background="#353535" Foreground="#00AAFF"/>
                <TextBlock Grid.Row="1" Grid.Column="2"
                          Text="{Binding VoltageDisplay}"
                          Foreground="White" FontFamily="Consolas"
                          FontSize="14" FontWeight="Bold"
                          Width="80" TextAlignment="Right"/>

                <!-- Wire Speed -->
                <TextBlock Grid.Row="2" Grid.Column="0" Text="Wire:"
                          Foreground="#888888" Margin="0,4"/>
                <ProgressBar Grid.Row="2" Grid.Column="1"
                            Value="{Binding ActualWireSpeed}"
                            Maximum="20" Height="8" Margin="8,4"
                            Background="#353535" Foreground="#AA00FF"/>
                <TextBlock Grid.Row="2" Grid.Column="2"
                          Text="{Binding WireSpeedDisplay}"
                          Foreground="White" FontFamily="Consolas"
                          FontSize="14" FontWeight="Bold"
                          Width="80" TextAlignment="Right"/>

                <!-- Power -->
                <TextBlock Grid.Row="3" Grid.Column="0" Text="Power:"
                          Foreground="#888888" Margin="0,4"/>
                <ProgressBar Grid.Row="3" Grid.Column="1"
                            Value="{Binding ActualPower}"
                            Maximum="10" Height="8" Margin="8,4"
                            Background="#353535" Foreground="#FFFF00"/>
                <TextBlock Grid.Row="3" Grid.Column="2"
                          Text="{Binding PowerDisplay}"
                          Foreground="White" FontFamily="Consolas"
                          FontSize="14" FontWeight="Bold"
                          Width="80" TextAlignment="Right"/>
            </Grid>

            <!-- Separator -->
            <Rectangle Height="1" Fill="#404040" Margin="0,12"/>

            <!-- Status indicators -->
            <UniformGrid Columns="3" Margin="0,0,0,12">
                <!-- Arc Status -->
                <Border Background="{Binding ArcStatusColor}"
                       CornerRadius="4" Padding="8,4" Margin="2">
                    <TextBlock Text="{Binding ArcStatusText}"
                              Foreground="White" FontWeight="Bold"
                              FontSize="11" TextAlignment="Center"/>
                </Border>

                <!-- Gas Status -->
                <Border Background="{Binding GasStatusColor}"
                       CornerRadius="4" Padding="8,4" Margin="2">
                    <TextBlock Text="{Binding GasStatusText}"
                              Foreground="White" FontWeight="Bold"
                              FontSize="11" TextAlignment="Center"/>
                </Border>

                <!-- Wire Status -->
                <Border Background="{Binding WireStatusColor}"
                       CornerRadius="4" Padding="8,4" Margin="2">
                    <TextBlock Text="{Binding WireStatusText}"
                              Foreground="White" FontWeight="Bold"
                              FontSize="11" TextAlignment="Center"/>
                </Border>
            </UniformGrid>

            <!-- Statistics -->
            <Grid>
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>

                <StackPanel Grid.Column="0" HorizontalAlignment="Center">
                    <TextBlock Text="TIME" Foreground="#666666" FontSize="10"
                              TextAlignment="Center"/>
                    <TextBlock Text="{Binding WeldTimeDisplay}"
                              Foreground="White" FontFamily="Consolas"
                              FontSize="16" FontWeight="Bold"
                              TextAlignment="Center"/>
                </StackPanel>

                <StackPanel Grid.Column="1" HorizontalAlignment="Center">
                    <TextBlock Text="DISTANCE" Foreground="#666666" FontSize="10"
                              TextAlignment="Center"/>
                    <TextBlock Text="{Binding DistanceDisplay}"
                              Foreground="White" FontFamily="Consolas"
                              FontSize="16" FontWeight="Bold"
                              TextAlignment="Center"/>
                </StackPanel>

                <StackPanel Grid.Column="2" HorizontalAlignment="Center">
                    <TextBlock Text="HEAT INPUT" Foreground="#666666" FontSize="10"
                              TextAlignment="Center"/>
                    <TextBlock Text="{Binding HeatInputDisplay}"
                              Foreground="White" FontFamily="Consolas"
                              FontSize="16" FontWeight="Bold"
                              TextAlignment="Center"/>
                </StackPanel>
            </Grid>
        </StackPanel>
    </Border>
</UserControl>
```

### 2.3 Create WeldingParameterPanel.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/Welding/WeldingParameterPanel.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.Welding.WeldingParameterPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             d:DesignHeight="250" d:DesignWidth="500">

    <UserControl.Resources>
        <Style x:Key="ParamSlider" TargetType="Slider">
            <Setter Property="Height" Value="24"/>
            <Setter Property="Margin" Value="8,0"/>
        </Style>

        <Style x:Key="ParamButton" TargetType="RepeatButton">
            <Setter Property="Width" Value="32"/>
            <Setter Property="Height" Value="32"/>
            <Setter Property="FontSize" Value="18"/>
            <Setter Property="FontWeight" Value="Bold"/>
            <Setter Property="Background" Value="#404040"/>
            <Setter Property="Foreground" Value="White"/>
            <Setter Property="BorderThickness" Value="0"/>
            <Setter Property="Delay" Value="300"/>
            <Setter Property="Interval" Value="50"/>
        </Style>
    </UserControl.Resources>

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <StackPanel>
            <TextBlock Text="WELDING PARAMETERS"
                      FontSize="12" FontWeight="Bold"
                      Foreground="#888888" Margin="0,0,0,12"/>

            <!-- Parameter list -->
            <ItemsControl ItemsSource="{Binding Parameters}">
                <ItemsControl.ItemTemplate>
                    <DataTemplate>
                        <Grid Margin="0,4">
                            <Grid.ColumnDefinitions>
                                <ColumnDefinition Width="100"/>
                                <ColumnDefinition Width="Auto"/>
                                <ColumnDefinition Width="*"/>
                                <ColumnDefinition Width="Auto"/>
                                <ColumnDefinition Width="90"/>
                            </Grid.ColumnDefinitions>

                            <!-- Parameter name -->
                            <TextBlock Grid.Column="0"
                                      Text="{Binding Name}"
                                      Foreground="#AAAAAA"
                                      VerticalAlignment="Center"/>

                            <!-- Decrease button -->
                            <RepeatButton Grid.Column="1" Content="−"
                                        Style="{StaticResource ParamButton}"
                                        Command="{Binding DecreaseCommand}"/>

                            <!-- Slider -->
                            <Slider Grid.Column="2"
                                   Style="{StaticResource ParamSlider}"
                                   Value="{Binding TargetValue}"
                                   Minimum="{Binding MinValue}"
                                   Maximum="{Binding MaxValue}"
                                   SmallChange="{Binding Step}"
                                   LargeChange="{Binding Step}"/>

                            <!-- Increase button -->
                            <RepeatButton Grid.Column="3" Content="+"
                                        Style="{StaticResource ParamButton}"
                                        Command="{Binding IncreaseCommand}"/>

                            <!-- Value display -->
                            <Border Grid.Column="4" Background="#353535"
                                   CornerRadius="4" Padding="8,4" Margin="8,0,0,0">
                                <StackPanel Orientation="Horizontal"
                                           HorizontalAlignment="Right">
                                    <TextBlock Text="{Binding TargetValue, StringFormat={}{0:F1}}"
                                              Foreground="White" FontFamily="Consolas"
                                              FontWeight="Bold" FontSize="14"/>
                                    <TextBlock Text="{Binding Unit}"
                                              Foreground="#888888" FontSize="12"
                                              Margin="4,0,0,0" VerticalAlignment="Bottom"/>
                                </StackPanel>
                            </Border>
                        </Grid>
                    </DataTemplate>
                </ItemsControl.ItemTemplate>
            </ItemsControl>

            <!-- Separator -->
            <Rectangle Height="1" Fill="#404040" Margin="0,12"/>

            <!-- Synergic mode -->
            <Grid>
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="Auto"/>
                </Grid.ColumnDefinitions>

                <CheckBox Grid.Column="0" Content="Synergic Mode"
                         IsChecked="{Binding Job.SynergicMode}"
                         Foreground="White"/>

                <StackPanel Grid.Column="1" Orientation="Horizontal"
                           Visibility="{Binding Job.SynergicMode,
                               Converter={StaticResource BoolToVisibility}}">
                    <TextBlock Text="Program:" Foreground="#888888"
                              VerticalAlignment="Center" Margin="0,0,8,0"/>
                    <ComboBox Width="80"
                             SelectedIndex="{Binding Job.SynergicProgram}">
                        <ComboBoxItem Content="#1 Steel 0.8"/>
                        <ComboBoxItem Content="#2 Steel 1.0"/>
                        <ComboBoxItem Content="#3 Steel 1.2"/>
                        <ComboBoxItem Content="#4 SS 1.0"/>
                        <ComboBoxItem Content="#5 Aluminum"/>
                    </ComboBox>
                </StackPanel>
            </Grid>
        </StackPanel>
    </Border>
</UserControl>
```

### 2.4 Create WeavePreviewPanel.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/Welding/WeavePreviewPanel.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.Welding.WeavePreviewPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             d:DesignHeight="300" d:DesignWidth="280">

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <StackPanel>
            <Grid Margin="0,0,0,8">
                <TextBlock Text="WEAVE PATTERN"
                          FontSize="12" FontWeight="Bold"
                          Foreground="#888888"/>
                <ToggleButton HorizontalAlignment="Right"
                             IsChecked="{Binding IsEnabled}"
                             Content="{Binding IsEnabled,
                                 Converter={StaticResource BoolToTextConverter},
                                 ConverterParameter='ON|OFF'}"
                             Padding="12,4"/>
            </Grid>

            <!-- Preview canvas -->
            <Border Background="#1A1A1A" CornerRadius="4"
                   Height="80" Margin="0,0,0,12">
                <Canvas ClipToBounds="True">
                    <!-- Center line -->
                    <Line X1="0" Y1="40" X2="200" Y2="40"
                         Stroke="#333333" StrokeDashArray="4,2"/>

                    <!-- Weave path -->
                    <Polyline Points="{Binding PreviewPoints}"
                             Stroke="#00FF00" StrokeThickness="2"/>

                    <!-- Current position indicator -->
                    <Ellipse Width="8" Height="8" Fill="#FFFF00"
                            Visibility="{Binding IsActive,
                                Converter={StaticResource BoolToVisibility}}"
                            Canvas.Left="{Binding CurrentPhase,
                                Converter={StaticResource PhaseToXConverter}}"
                            Canvas.Top="36"/>
                </Canvas>
            </Border>

            <!-- Pattern selection -->
            <Grid Margin="0,0,0,8">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="Auto"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>

                <TextBlock Text="Pattern:" Foreground="#888888"
                          VerticalAlignment="Center"/>
                <ComboBox Grid.Column="1" Margin="8,0,0,0"
                         ItemsSource="{Binding PatternTypes}"
                         SelectedItem="{Binding SelectedPattern}"/>
            </Grid>

            <!-- Parameters -->
            <Grid Margin="0,0,0,4">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="Auto"/>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="60"/>
                </Grid.ColumnDefinitions>

                <TextBlock Text="Amplitude:" Foreground="#888888"
                          VerticalAlignment="Center"/>
                <Slider Grid.Column="1" Value="{Binding Amplitude}"
                       Minimum="0.5" Maximum="10" Margin="8,0"/>
                <TextBlock Grid.Column="2"
                          Text="{Binding Amplitude, StringFormat={}{0:F1} mm}"
                          Foreground="White" FontFamily="Consolas"/>
            </Grid>

            <Grid Margin="0,0,0,4">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="Auto"/>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="60"/>
                </Grid.ColumnDefinitions>

                <TextBlock Text="Wavelength:" Foreground="#888888"
                          VerticalAlignment="Center"/>
                <Slider Grid.Column="1" Value="{Binding Wavelength}"
                       Minimum="2" Maximum="20" Margin="8,0"/>
                <TextBlock Grid.Column="2"
                          Text="{Binding Wavelength, StringFormat={}{0:F1} mm}"
                          Foreground="White" FontFamily="Consolas"/>
            </Grid>

            <!-- Dwell times -->
            <Grid Margin="0,0,0,4">
                <Grid.ColumnDefinitions>
                    <ColumnDefinition Width="*"/>
                    <ColumnDefinition Width="*"/>
                </Grid.ColumnDefinitions>

                <StackPanel Grid.Column="0" Margin="0,0,4,0">
                    <TextBlock Text="Left Dwell (ms):" Foreground="#888888" FontSize="11"/>
                    <TextBox Text="{Binding DwellLeft}" Background="#353535"
                            Foreground="White" BorderThickness="0" Padding="4"/>
                </StackPanel>

                <StackPanel Grid.Column="1" Margin="4,0,0,0">
                    <TextBlock Text="Right Dwell (ms):" Foreground="#888888" FontSize="11"/>
                    <TextBox Text="{Binding DwellRight}" Background="#353535"
                            Foreground="White" BorderThickness="0" Padding="4"/>
                </StackPanel>
            </Grid>

            <!-- Presets -->
            <TextBlock Text="Presets:" Foreground="#888888" Margin="0,8,0,4"/>
            <WrapPanel>
                <Button Content="Narrow" Margin="2" Padding="8,4"
                       Command="{Binding ApplyPresetCommand}"
                       CommandParameter="Narrow Gap"/>
                <Button Content="Medium" Margin="2" Padding="8,4"
                       Command="{Binding ApplyPresetCommand}"
                       CommandParameter="Medium Gap"/>
                <Button Content="Wide" Margin="2" Padding="8,4"
                       Command="{Binding ApplyPresetCommand}"
                       CommandParameter="Wide Gap"/>
                <Button Content="V-Up" Margin="2" Padding="8,4"
                       Command="{Binding ApplyPresetCommand}"
                       CommandParameter="Vertical Up"/>
            </WrapPanel>

            <!-- Status -->
            <Border Background="#353535" CornerRadius="4"
                   Padding="8" Margin="0,8,0,0"
                   Visibility="{Binding IsActive, Converter={StaticResource BoolToVisibility}}">
                <Grid>
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="*"/>
                    </Grid.ColumnDefinitions>

                    <StackPanel Grid.Column="0">
                        <TextBlock Text="Cycles" Foreground="#888888" FontSize="10"/>
                        <TextBlock Text="{Binding CycleCount}"
                                  Foreground="White" FontWeight="Bold"/>
                    </StackPanel>

                    <StackPanel Grid.Column="1">
                        <TextBlock Text="Path Ratio" Foreground="#888888" FontSize="10"/>
                        <TextBlock Text="{Binding PathLengthRatio, StringFormat={}{0:F2}x}"
                                  Foreground="White" FontWeight="Bold"/>
                    </StackPanel>
                </Grid>
            </Border>
        </StackPanel>
    </Border>
</UserControl>
```

### 2.5 Create WeldingControlButtons.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/Welding/WeldingControlButtons.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.Welding.WeldingControlButtons"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             d:DesignHeight="80" d:DesignWidth="500">

    <UserControl.Resources>
        <Style x:Key="WeldButton" TargetType="Button">
            <Setter Property="Height" Value="50"/>
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
            <Style.Triggers>
                <Trigger Property="IsEnabled" Value="False">
                    <Setter Property="Opacity" Value="0.5"/>
                </Trigger>
                <Trigger Property="IsMouseOver" Value="True">
                    <Setter Property="Opacity" Value="0.9"/>
                </Trigger>
            </Style.Triggers>
        </Style>
    </UserControl.Resources>

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <Grid>
            <Grid.ColumnDefinitions>
                <ColumnDefinition Width="*"/>
                <ColumnDefinition Width="*"/>
                <ColumnDefinition Width="*"/>
                <ColumnDefinition Width="Auto"/>
            </Grid.ColumnDefinitions>

            <!-- Start Weld -->
            <Button Grid.Column="0" Content="START WELD"
                   Style="{StaticResource WeldButton}"
                   Background="#00AA00"
                   Command="{Binding StartWeldCommand}"
                   Visibility="{Binding IsWelding,
                       Converter={StaticResource InverseBoolToVisibility}}"
                   Margin="0,0,4,0"/>

            <!-- Stop Weld -->
            <Button Grid.Column="0" Content="STOP WELD"
                   Style="{StaticResource WeldButton}"
                   Background="#AA6600"
                   Command="{Binding StopWeldCommand}"
                   Visibility="{Binding IsWelding,
                       Converter={StaticResource BoolToVisibility}}"
                   Margin="0,0,4,0"/>

            <!-- Abort -->
            <Button Grid.Column="1" Content="ABORT"
                   Style="{StaticResource WeldButton}"
                   Background="#CC0000"
                   Command="{Binding AbortWeldCommand}"
                   Margin="4,0"/>

            <!-- Reset Fault -->
            <Button Grid.Column="2" Content="RESET FAULT"
                   Style="{StaticResource WeldButton}"
                   Background="#AA0066"
                   Command="{Binding ResetFaultCommand}"
                   IsEnabled="{Binding State.HasFault}"
                   Margin="4,0,0,0"/>

            <!-- Job management -->
            <StackPanel Grid.Column="3" Orientation="Horizontal" Margin="16,0,0,0">
                <Border Background="#353535" CornerRadius="4" Padding="8,0">
                    <StackPanel Orientation="Horizontal" VerticalAlignment="Center">
                        <TextBlock Text="Job:" Foreground="#888888" Margin="0,0,8,0"
                                  VerticalAlignment="Center"/>
                        <ComboBox ItemsSource="{Binding Job.SavedJobs}"
                                 SelectedItem="{Binding Job.SelectedJob}"
                                 Width="120"/>
                        <Button Content="Load" Margin="8,0,0,0" Padding="8,4"
                               Command="{Binding Job.LoadJobCommand}"
                               CommandParameter="{Binding Job.SelectedJob}"/>
                        <Button Content="Save" Margin="4,0,0,0" Padding="8,4"
                               Command="{Binding Job.SaveJobCommand}"/>
                    </StackPanel>
                </Border>
            </StackPanel>
        </Grid>
    </Border>
</UserControl>
```

### 2.6 Create WeldingJobPanel.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/Welding/WeldingJobPanel.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.Welding.WeldingJobPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             d:DesignHeight="400" d:DesignWidth="300">

    <Border Background="#2D2D2D" CornerRadius="8" Padding="12">
        <ScrollViewer VerticalScrollBarVisibility="Auto">
            <StackPanel>
                <TextBlock Text="JOB SETTINGS"
                          FontSize="12" FontWeight="Bold"
                          Foreground="#888888" Margin="0,0,0,12"/>

                <!-- Job Name -->
                <StackPanel Margin="0,0,0,8">
                    <TextBlock Text="Job Name:" Foreground="#888888" FontSize="11"/>
                    <TextBox Text="{Binding JobName}"
                            Background="#353535" Foreground="White"
                            BorderThickness="0" Padding="8,4"/>
                </StackPanel>

                <!-- Process Type -->
                <StackPanel Margin="0,0,0,8">
                    <TextBlock Text="Process:" Foreground="#888888" FontSize="11"/>
                    <ComboBox ItemsSource="{Binding ProcessOptions}"
                             SelectedItem="{Binding Process}"/>
                </StackPanel>

                <!-- Separator -->
                <Rectangle Height="1" Fill="#404040" Margin="0,8"/>

                <!-- Timing Settings -->
                <TextBlock Text="TIMING" Foreground="#666666"
                          FontSize="11" FontWeight="Bold" Margin="0,8,0,8"/>

                <Grid Margin="0,0,0,4">
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="80"/>
                    </Grid.ColumnDefinitions>
                    <TextBlock Text="Pre-Flow Time:" Foreground="#AAAAAA"/>
                    <StackPanel Grid.Column="1" Orientation="Horizontal">
                        <TextBox Text="{Binding PreFlowTime}" Width="50"
                                Background="#353535" Foreground="White"
                                BorderThickness="0" TextAlignment="Right"/>
                        <TextBlock Text="ms" Foreground="#888888" Margin="4,0,0,0"/>
                    </StackPanel>
                </Grid>

                <Grid Margin="0,0,0,4">
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="80"/>
                    </Grid.ColumnDefinitions>
                    <TextBlock Text="Post-Flow Time:" Foreground="#AAAAAA"/>
                    <StackPanel Grid.Column="1" Orientation="Horizontal">
                        <TextBox Text="{Binding PostFlowTime}" Width="50"
                                Background="#353535" Foreground="White"
                                BorderThickness="0" TextAlignment="Right"/>
                        <TextBlock Text="ms" Foreground="#888888" Margin="4,0,0,0"/>
                    </StackPanel>
                </Grid>

                <Grid Margin="0,0,0,4">
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="80"/>
                    </Grid.ColumnDefinitions>
                    <TextBlock Text="Crater Fill Time:" Foreground="#AAAAAA"/>
                    <StackPanel Grid.Column="1" Orientation="Horizontal">
                        <TextBox Text="{Binding CraterFillTime}" Width="50"
                                Background="#353535" Foreground="White"
                                BorderThickness="0" TextAlignment="Right"/>
                        <TextBlock Text="ms" Foreground="#888888" Margin="4,0,0,0"/>
                    </StackPanel>
                </Grid>

                <Grid Margin="0,0,0,4">
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="80"/>
                    </Grid.ColumnDefinitions>
                    <TextBlock Text="Burn-Back Time:" Foreground="#AAAAAA"/>
                    <StackPanel Grid.Column="1" Orientation="Horizontal">
                        <TextBox Text="{Binding BurnBackTime}" Width="50"
                                Background="#353535" Foreground="White"
                                BorderThickness="0" TextAlignment="Right"/>
                        <TextBlock Text="ms" Foreground="#888888" Margin="4,0,0,0"/>
                    </StackPanel>
                </Grid>

                <!-- Separator -->
                <Rectangle Height="1" Fill="#404040" Margin="0,8"/>

                <!-- Gas Settings -->
                <TextBlock Text="GAS" Foreground="#666666"
                          FontSize="11" FontWeight="Bold" Margin="0,8,0,8"/>

                <Grid Margin="0,0,0,4">
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="*"/>
                    </Grid.ColumnDefinitions>
                    <TextBlock Text="Type:" Foreground="#AAAAAA"/>
                    <ComboBox Grid.Column="1"
                             ItemsSource="{Binding GasTypes}"
                             SelectedItem="{Binding GasType}"/>
                </Grid>

                <Grid Margin="0,0,0,4">
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="80"/>
                    </Grid.ColumnDefinitions>
                    <TextBlock Text="Flow Rate:" Foreground="#AAAAAA"/>
                    <StackPanel Grid.Column="1" Orientation="Horizontal">
                        <TextBox Text="{Binding GasFlowRate}" Width="40"
                                Background="#353535" Foreground="White"
                                BorderThickness="0" TextAlignment="Right"/>
                        <TextBlock Text="L/min" Foreground="#888888" Margin="4,0,0,0"/>
                    </StackPanel>
                </Grid>

                <!-- Separator -->
                <Rectangle Height="1" Fill="#404040" Margin="0,8"/>

                <!-- Wire Settings -->
                <TextBlock Text="WIRE" Foreground="#666666"
                          FontSize="11" FontWeight="Bold" Margin="0,8,0,8"/>

                <Grid Margin="0,0,0,4">
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="*"/>
                    </Grid.ColumnDefinitions>
                    <TextBlock Text="Material:" Foreground="#AAAAAA"/>
                    <ComboBox Grid.Column="1"
                             ItemsSource="{Binding WireMaterials}"
                             SelectedItem="{Binding WireMaterial}"/>
                </Grid>

                <Grid Margin="0,0,0,4">
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition Width="*"/>
                        <ColumnDefinition Width="*"/>
                    </Grid.ColumnDefinitions>
                    <TextBlock Text="Diameter:" Foreground="#AAAAAA"/>
                    <ComboBox Grid.Column="1"
                             ItemsSource="{Binding WireDiameters}"
                             SelectedItem="{Binding WireDiameter}"/>
                </Grid>

                <!-- Dirty indicator -->
                <Border Background="#664400" CornerRadius="4"
                       Padding="8" Margin="0,12,0,0"
                       Visibility="{Binding IsDirty, Converter={StaticResource BoolToVisibility}}">
                    <TextBlock Text="Unsaved changes"
                              Foreground="#FFAA00" FontStyle="Italic"/>
                </Border>
            </StackPanel>
        </ScrollViewer>
    </Border>
</UserControl>
```

---

## Step 3: Create Main Welding Panel

### 3.1 Create WeldingMainPanel.xaml

**File:** `src/csharp/RobotController.UI/Views/Controls/Welding/WeldingMainPanel.xaml`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.Welding.WeldingMainPanel"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             xmlns:controls="clr-namespace:RobotController.UI.Views.Controls.Welding"
             d:DesignHeight="700" d:DesignWidth="1000">

    <Grid>
        <Grid.RowDefinitions>
            <RowDefinition Height="Auto"/>
            <RowDefinition Height="*"/>
            <RowDefinition Height="Auto"/>
        </Grid.RowDefinitions>

        <!-- Top row: State, Feedback, Weave Preview -->
        <Grid Grid.Row="0" Margin="0,0,0,8">
            <Grid.ColumnDefinitions>
                <ColumnDefinition Width="*"/>
                <ColumnDefinition Width="*"/>
                <ColumnDefinition Width="280"/>
            </Grid.ColumnDefinitions>

            <controls:WeldingStateDisplay Grid.Column="0"
                                         DataContext="{Binding State}"
                                         Margin="0,0,4,0"/>

            <controls:WeldingFeedbackPanel Grid.Column="1"
                                          DataContext="{Binding Feedback}"
                                          Margin="4,0,4,0"/>

            <controls:WeavePreviewPanel Grid.Column="2"
                                       DataContext="{Binding WeavePreview}"
                                       Margin="4,0,0,0"/>
        </Grid>

        <!-- Middle row: Parameters and Job -->
        <Grid Grid.Row="1">
            <Grid.ColumnDefinitions>
                <ColumnDefinition Width="*"/>
                <ColumnDefinition Width="300"/>
            </Grid.ColumnDefinitions>

            <controls:WeldingParameterPanel Grid.Column="0"
                                           DataContext="{Binding}"
                                           Margin="0,0,4,0"/>

            <controls:WeldingJobPanel Grid.Column="1"
                                     DataContext="{Binding Job}"
                                     Margin="4,0,0,0"/>
        </Grid>

        <!-- Bottom row: Control buttons -->
        <controls:WeldingControlButtons Grid.Row="2"
                                       DataContext="{Binding}"
                                       Margin="0,8,0,0"/>
    </Grid>
</UserControl>
```

---

## Step 4: Create Unit Tests

### 4.1 Create WeldingViewModelTests.cs

**File:** `src/csharp/RobotController.Tests/ViewModels/WeldingViewModelTests.cs`

```csharp
using Xunit;
using Moq;
using RobotController.UI.ViewModels.Welding;
using RobotController.Core.Services;
using RobotController.Core.IPC;

namespace RobotController.Tests.ViewModels;

public class WeldingParameterViewModelTests
{
    [Fact]
    public void Constructor_InitializesCorrectly()
    {
        var param = new WeldingParameterViewModel("Current", "A", 50, 400, 180, 5);

        Assert.Equal("Current", param.Name);
        Assert.Equal("A", param.Unit);
        Assert.Equal(50, param.MinValue);
        Assert.Equal(400, param.MaxValue);
        Assert.Equal(180, param.Value);
        Assert.Equal(180, param.TargetValue);
        Assert.Equal(5, param.Step);
    }

    [Fact]
    public void Increase_AddsStep()
    {
        var param = new WeldingParameterViewModel("Current", "A", 50, 400, 180, 5);

        param.IncreaseCommand.Execute(null);

        Assert.Equal(185, param.TargetValue);
    }

    [Fact]
    public void Decrease_SubtractsStep()
    {
        var param = new WeldingParameterViewModel("Current", "A", 50, 400, 180, 5);

        param.DecreaseCommand.Execute(null);

        Assert.Equal(175, param.TargetValue);
    }

    [Fact]
    public void SetTarget_ClampsToMax()
    {
        var param = new WeldingParameterViewModel("Current", "A", 50, 400, 180, 5);

        param.SetTarget(500);

        Assert.Equal(400, param.TargetValue);
    }

    [Fact]
    public void SetTarget_ClampsToMin()
    {
        var param = new WeldingParameterViewModel("Current", "A", 50, 400, 180, 5);

        param.SetTarget(10);

        Assert.Equal(50, param.TargetValue);
    }

    [Fact]
    public void UpdateActualValue_HighlightsWhenDifferent()
    {
        var param = new WeldingParameterViewModel("Current", "A", 50, 400, 180, 5);

        param.UpdateActualValue(150);

        Assert.True(param.IsHighlighted);
    }

    [Fact]
    public void ValuePercent_CalculatesCorrectly()
    {
        var param = new WeldingParameterViewModel("Current", "A", 0, 100, 50, 1);

        Assert.Equal(50, param.ValuePercent);

        param.SetTarget(75);
        param.UpdateActualValue(75);

        Assert.Equal(75, param.ValuePercent);
    }
}

public class WeldingStateViewModelTests
{
    [Fact]
    public void UpdateState_SetsCorrectDisplay()
    {
        var vm = new WeldingStateViewModel();

        vm.UpdateState(WeldingState.Welding, WeldingFault.None, 1000);

        Assert.Equal("WELDING", vm.StateDisplay);
        Assert.Equal("#00FF00", vm.StateColor);
        Assert.True(vm.IsWelding);
        Assert.False(vm.HasFault);
    }

    [Fact]
    public void UpdateState_WithFault_ShowsFault()
    {
        var vm = new WeldingStateViewModel();

        vm.UpdateState(WeldingState.Error, WeldingFault.ArcFail, 0);

        Assert.Equal("ERROR", vm.StateDisplay);
        Assert.True(vm.HasFault);
        Assert.Contains("Arc failed", vm.FaultDisplay);
    }

    [Fact]
    public void UpdateState_LogsStateChange()
    {
        var vm = new WeldingStateViewModel();

        vm.UpdateState(WeldingState.PreFlow, WeldingFault.None, 0);
        vm.UpdateState(WeldingState.ArcStart, WeldingFault.None, 100);

        Assert.Equal(2, vm.StateHistory.Count);
        Assert.Equal("ArcStart", vm.StateHistory[0].ToState);
    }

    [Fact]
    public void StateIndicators_ReflectCurrentState()
    {
        var vm = new WeldingStateViewModel();

        vm.UpdateState(WeldingState.PreFlow, WeldingFault.None, 0);
        Assert.True(vm.IsPreFlow);
        Assert.False(vm.IsWelding);

        vm.UpdateState(WeldingState.Welding, WeldingFault.None, 0);
        Assert.False(vm.IsPreFlow);
        Assert.True(vm.IsWelding);
    }
}

public class WeldingFeedbackViewModelTests
{
    [Fact]
    public void Update_SetsAllValues()
    {
        var vm = new WeldingFeedbackViewModel();

        vm.Update(180, 22, 8.5, true, true, true, 5000, 50, 0.5);

        Assert.Equal(180, vm.ActualCurrent);
        Assert.Equal(22, vm.ActualVoltage);
        Assert.Equal(8.5, vm.ActualWireSpeed);
        Assert.True(vm.ArcPresent);
        Assert.True(vm.GasFlowOk);
        Assert.Equal(5000, vm.WeldTime.TotalMilliseconds);
        Assert.Equal(50, vm.WeldDistance);
    }

    [Fact]
    public void PowerCalculation_IsCorrect()
    {
        var vm = new WeldingFeedbackViewModel();

        vm.Update(200, 25, 0, false, false, false, 0, 0, 0);

        Assert.Equal(5.0, vm.ActualPower);  // 200 * 25 / 1000 = 5 kW
    }

    [Fact]
    public void StatusText_ReflectsState()
    {
        var vm = new WeldingFeedbackViewModel();

        vm.Update(180, 22, 8, true, true, true, 0, 0, 0);

        Assert.Equal("ARC OK", vm.ArcStatusText);
        Assert.Equal("#00FF00", vm.ArcStatusColor);

        vm.Update(0, 0, 0, false, false, false, 0, 0, 0);

        Assert.Equal("NO ARC", vm.ArcStatusText);
    }

    [Fact]
    public void Reset_ClearsAllValues()
    {
        var vm = new WeldingFeedbackViewModel();
        vm.Update(180, 22, 8, true, true, true, 5000, 50, 0.5);

        vm.Reset();

        Assert.Equal(0, vm.ActualCurrent);
        Assert.Equal(0, vm.ActualVoltage);
        Assert.False(vm.ArcPresent);
    }
}

public class WeavePreviewViewModelTests
{
    [Fact]
    public void ApplyPreset_SetsParameters()
    {
        var vm = new WeavePreviewViewModel();

        vm.ApplyPresetCommand.Execute("Wide Gap");

        Assert.Equal(WeavePatternType.Triangular, vm.SelectedPattern);
        Assert.Equal(6.0, vm.Amplitude);
        Assert.Equal(12.0, vm.Wavelength);
    }

    [Fact]
    public void GeneratePreview_CreatesPoints()
    {
        var vm = new WeavePreviewViewModel();

        Assert.NotEmpty(vm.PreviewPoints);
    }

    [Fact]
    public void PatternChange_RegeneratesPreview()
    {
        var vm = new WeavePreviewViewModel();
        var initialCount = vm.PreviewPoints.Count;

        vm.SelectedPattern = WeavePatternType.Circular;

        Assert.Equal(initialCount, vm.PreviewPoints.Count);
    }
}

public class WeldingJobViewModelTests
{
    [Fact]
    public void NewJob_ResetsToDefaults()
    {
        var vm = new WeldingJobViewModel();
        vm.PreFlowTime = 500;
        vm.PostFlowTime = 1000;

        vm.NewJobCommand.Execute(null);

        Assert.Equal("New Job", vm.JobName);
        Assert.Equal(300u, vm.PreFlowTime);
        Assert.Equal(500u, vm.PostFlowTime);
    }

    [Fact]
    public void ToJobData_CreatesCorrectData()
    {
        var vm = new WeldingJobViewModel
        {
            JobName = "Test Job",
            PreFlowTime = 400,
            SynergicMode = true
        };

        var data = vm.ToJobData(180, 22, 8, 10);

        Assert.Equal("Test Job", data.Name);
        Assert.Equal(180, data.Current);
        Assert.Equal(400u, data.PreFlowTime);
        Assert.True(data.SynergicMode);
    }

    [Fact]
    public void PropertyChange_MarksDirty()
    {
        var vm = new WeldingJobViewModel();
        vm.IsDirty = false;

        vm.PreFlowTime = 400;

        Assert.True(vm.IsDirty);
    }
}

public class WeldingControlViewModelTests
{
    private readonly Mock<IWeldingClientService> _weldingMock;
    private readonly WeldingControlViewModel _vm;

    public WeldingControlViewModelTests()
    {
        _weldingMock = new Mock<IWeldingClientService>();
        _vm = new WeldingControlViewModel(_weldingMock.Object);
    }

    [Fact]
    public void Constructor_InitializesParameters()
    {
        Assert.NotNull(_vm.Current);
        Assert.NotNull(_vm.Voltage);
        Assert.NotNull(_vm.WireSpeed);
        Assert.NotNull(_vm.TravelSpeed);
        Assert.Equal(4, _vm.Parameters.Count);
    }

    [Fact]
    public void CanStartWeld_RequiresConnectionAndReady()
    {
        Assert.False(_vm.CanStartWeld);

        _vm.SetConnected(true);
        Assert.False(_vm.CanStartWeld);  // Still not ready

        // Simulate ready state
        _vm.IsReady = true;
        // Would need to trigger update
    }

    [Fact]
    public void SetConnected_UpdatesCanExecute()
    {
        _vm.SetConnected(true);

        Assert.True(_vm.IsConnected);
    }
}
```

---

## Step 5: Add Converters for Welding UI

### 5.1 Create WeldingConverters.cs

**File:** `src/csharp/RobotController.UI/Converters/WeldingConverters.cs`

```csharp
using System;
using System.Globalization;
using System.Windows;
using System.Windows.Data;

namespace RobotController.UI.Converters;

/// <summary>
/// Converts boolean to text (parameter format: "TrueText|FalseText")
/// </summary>
public class BoolToTextConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        bool boolValue = value is bool b && b;
        string param = parameter?.ToString() ?? "True|False";
        string[] texts = param.Split('|');

        return boolValue ? texts[0] : (texts.Length > 1 ? texts[1] : texts[0]);
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}

/// <summary>
/// Converts boolean to Style (parameter format: "TrueStyleKey|FalseStyleKey")
/// </summary>
public class BoolToStyleConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        bool boolValue = value is bool b && b;
        string param = parameter?.ToString() ?? "";
        string[] styleKeys = param.Split('|');

        string styleKey = boolValue ? styleKeys[0] :
            (styleKeys.Length > 1 ? styleKeys[1] : styleKeys[0]);

        // Try to find the style in application resources
        if (Application.Current.TryFindResource(styleKey) is Style style)
        {
            return style;
        }

        return DependencyProperty.UnsetValue;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}

/// <summary>
/// Converts phase (0-1) to X position for weave preview
/// </summary>
public class PhaseToXConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        if (value is double phase)
        {
            double width = 200;  // Preview width
            return phase * width;
        }
        return 0.0;
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}

/// <summary>
/// Converts welding state enum to display color
/// </summary>
public class WeldingStateToColorConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        if (value is string state)
        {
            return state switch
            {
                "Idle" => "#808080",
                "PreFlow" or "PostFlow" => "#AAAA00",
                "ArcStart" or "ArcEnd" => "#FF8800",
                "Welding" => "#00FF00",
                "CraterFill" => "#00CC00",
                "BurnBack" => "#AA6600",
                "Error" or "EmergencyStop" => "#FF0000",
                _ => "#808080"
            };
        }
        return "#808080";
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}

/// <summary>
/// Converts milliseconds to time display (mm:ss)
/// </summary>
public class MillisecondsToTimeConverter : IValueConverter
{
    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        if (value is uint ms)
        {
            var ts = TimeSpan.FromMilliseconds(ms);
            return ts.ToString(@"mm\:ss");
        }
        if (value is int msInt)
        {
            var ts = TimeSpan.FromMilliseconds(msInt);
            return ts.ToString(@"mm\:ss");
        }
        return "00:00";
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
    {
        throw new NotImplementedException();
    }
}
```

---

## Step 6: Update MainWindow for Welding Panel

### 6.1 Update MainViewModel.cs

**Add WeldingControl property:**

```csharp
// In MainViewModel.cs

[ObservableProperty]
private WeldingControlViewModel _weldingControl;

public MainViewModel(
    IIpcClientService ipc,
    IFirmwareClientService firmware,
    IKinematicsClientService kinematics,
    ITrajectoryClientService trajectory,
    IWeldingClientService welding,
    IWeaveClientService weave)
{
    // ... existing initialization ...

    WeldingControl = new WeldingControlViewModel(welding, weave);

    // Wire up connection state
    firmware.ConnectionChanged += (s, connected) =>
    {
        WeldingControl.SetConnected(connected);
    };
}
```

### 6.2 Add Welding Tab to MainWindow

**Update MainWindow.xaml:**

```xml
<!-- Add namespace -->
xmlns:welding="clr-namespace:RobotController.UI.Views.Controls.Welding"

<!-- Add to Resources -->
<converters:BoolToTextConverter x:Key="BoolToTextConverter"/>
<converters:BoolToStyleConverter x:Key="BoolToStyleConverter"/>
<converters:PhaseToXConverter x:Key="PhaseToXConverter"/>

<!-- Add tab for welding -->
<TabControl Grid.Row="1">
    <TabItem Header="Motion">
        <!-- Existing motion controls -->
    </TabItem>

    <TabItem Header="Welding">
        <welding:WeldingMainPanel DataContext="{Binding WeldingControl}"/>
    </TabItem>

    <TabItem Header="3D View">
        <!-- Existing 3D viewport -->
    </TabItem>
</TabControl>
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
dotnet test --filter "FullyQualifiedName~Welding"

# Run application
dotnet run --project RobotController.UI
```

**Expected:**
- Application launches with Welding tab
- Parameter sliders adjust values
- State display shows FSM states
- Weave preview generates pattern
- Job save/load works

---

## Completion Checklist

- [ ] WeldingParameterViewModel.cs created
- [ ] WeldingFeedbackViewModel.cs created
- [ ] WeldingStateViewModel.cs created
- [ ] WeavePreviewViewModel.cs created
- [ ] WeldingJobViewModel.cs created
- [ ] WeldingControlViewModel.cs created (main)
- [ ] WeldingStateDisplay.xaml created
- [ ] WeldingFeedbackPanel.xaml created
- [ ] WeldingParameterPanel.xaml created
- [ ] WeavePreviewPanel.xaml created
- [ ] WeldingControlButtons.xaml created
- [ ] WeldingJobPanel.xaml created
- [ ] WeldingMainPanel.xaml created
- [ ] WeldingConverters.cs created
- [ ] Unit tests created with 20+ tests
- [ ] MainWindow updated with Welding tab
- [ ] All tests pass

---

## Troubleshooting

### Parameters Not Updating
- Check IPC connection
- Verify WeldingClientService is injected
- Check event subscriptions

### Weave Preview Not Showing
- Verify PointCollection binding
- Check Canvas dimensions
- Verify pattern type is not None

### Job Save/Load Failing
- Check folder permissions
- Verify JSON serialization
- Check file path exists

---

## Keyboard Shortcuts (Welding)

| Key | Action |
|-----|--------|
| F5 | Start Weld |
| F6 | Stop Weld |
| Escape | Abort |
| F9 | Reset Fault |
| W | Toggle Weave |

---

## Git Commit

```powershell
git add -A
git commit -m "IMPL_P3_03: Add welding HMI controls

- Create WeldingParameterViewModel for adjustable parameters
- Create WeldingFeedbackViewModel for real-time display
- Create WeldingStateViewModel for FSM visualization
- Create WeavePreviewViewModel for pattern preview
- Create WeldingJobViewModel for job management
- Add WeldingStateDisplay with state machine graph
- Add WeldingFeedbackPanel with live readings
- Add WeldingParameterPanel with sliders
- Add WeavePreviewPanel with 2D preview
- Add WeldingJobPanel for timing and gas settings
- Add WeldingControlButtons for start/stop/abort
- Create WeldingConverters for UI bindings
- Integrate with MainWindow as Welding tab
- Add 20+ unit tests for welding ViewModels

Co-Authored-By: Claude <noreply@anthropic.com>"
```
