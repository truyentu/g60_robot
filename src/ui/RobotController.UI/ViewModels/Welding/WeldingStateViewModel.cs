using CommunityToolkit.Mvvm.ComponentModel;
using RobotController.Common.Messages;
using System;
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
            WeldingState.Fault => ("FAULT", "#FF0000"),
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
        IsError = state == WeldingState.Fault || state == WeldingState.EmergencyStop;
    }
}

public class StateLogEntry
{
    public DateTime Timestamp { get; init; }
    public string FromState { get; init; } = "";
    public string ToState { get; init; } = "";
    public string TimestampDisplay => Timestamp.ToString("HH:mm:ss.fff");
}
