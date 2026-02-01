using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using System;
using System.Collections.ObjectModel;
using System.Threading.Tasks;
using System.Windows;

namespace RobotController.UI.ViewModels.Welding;

/// <summary>
/// Main view model for welding control panel
/// </summary>
public partial class WeldingControlViewModel : ObservableObject
{
    private readonly IWeldingClientService? _weldingService;
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
        IWeldingClientService? weldingService = null,
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
        if (_weldingService != null)
        {
            _weldingService.StatusUpdated += OnStatusUpdated;
            _weldingService.FaultOccurred += OnFaultOccurred;
            _weldingService.StateChanged += OnStateChanged;
        }

        UpdateCanExecute();
    }

    // ========================================================================
    // Commands
    // ========================================================================

    [RelayCommand(CanExecute = nameof(CanStartWeld))]
    private async Task StartWeldAsync()
    {
        if (_weldingService == null) return;

        var jobData = Job.ToJobData(
            (float)Current.TargetValue,
            (float)Voltage.TargetValue,
            (float)WireSpeed.TargetValue,
            (float)TravelSpeed.TargetValue);

        var result = await _weldingService.StartWeldAsync(jobData);

        if (!result.Success)
        {
            // Handle error
            State.UpdateState(WeldingState.Fault, WeldingFault.None, 0);
        }
    }

    [RelayCommand(CanExecute = nameof(CanStopWeld))]
    private async Task StopWeldAsync()
    {
        if (_weldingService == null) return;
        await _weldingService.StopWeldAsync();
    }

    [RelayCommand]
    private async Task AbortWeldAsync()
    {
        if (_weldingService == null) return;
        await _weldingService.AbortWeldAsync();
    }

    [RelayCommand]
    private async Task ResetFaultAsync()
    {
        if (_weldingService == null) return;
        await _weldingService.ResetAsync();
    }

    [RelayCommand]
    private async Task RefreshStatusAsync()
    {
        if (_weldingService == null) return;
        var status = await _weldingService.GetStatusAsync();
        ProcessStatus(status);
    }

    // ========================================================================
    // Parameter Adjustment
    // ========================================================================

    private async Task AdjustParameterAsync(string parameter, double value)
    {
        if (!IsWelding || _weldingService == null) return;

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
        Application.Current?.Dispatcher.Invoke(() => ProcessStatus(status));
    }

    private void OnFaultOccurred(object? sender, WeldingFault fault)
    {
        Application.Current?.Dispatcher.Invoke(() =>
        {
            State.UpdateState(WeldingState.Fault, fault, 0);
            UpdateCanExecute();
        });
    }

    private void OnStateChanged(object? sender, WeldingState state)
    {
        Application.Current?.Dispatcher.Invoke(() =>
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
