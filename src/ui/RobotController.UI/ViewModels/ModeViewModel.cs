using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using Serilog;
using System.Collections.ObjectModel;
using System.Windows.Media;

namespace RobotController.UI.ViewModels;

/// <summary>
/// Represents an available operation mode
/// </summary>
public partial class ModeOption : ObservableObject
{
    [ObservableProperty]
    private string _name = "";

    [ObservableProperty]
    private string _displayName = "";

    [ObservableProperty]
    private string _description = "";

    [ObservableProperty]
    private Brush _color = Brushes.Gray;

    [ObservableProperty]
    private bool _isSelected;
}

/// <summary>
/// ViewModel for Operation Mode management
/// </summary>
public partial class ModeViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;

    // Current mode
    [ObservableProperty]
    private string _currentMode = "MANUAL";

    [ObservableProperty]
    private Brush _modeColor = Brushes.Yellow;

    [ObservableProperty]
    private double _maxLinearVelocity = 250.0;

    [ObservableProperty]
    private double _maxJointVelocity = 30.0;

    [ObservableProperty]
    private bool _deadmanRequired = true;

    [ObservableProperty]
    private bool _safetyFenceRequired;

    // Mode selection
    [ObservableProperty]
    private ModeOption? _selectedMode;

    [ObservableProperty]
    private bool _canSwitchMode = true;

    [ObservableProperty]
    private bool _isSwitching;

    [ObservableProperty]
    private string _switchError = "";

    // Requirements check
    [ObservableProperty]
    private bool _isCheckingRequirements;

    public ObservableCollection<ModeOption> AvailableModes { get; } = new();
    public ObservableCollection<string> MissingRequirements { get; } = new();

    public ModeViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;

        // Initialize available modes
        InitializeModes();

        // Subscribe to mode change events
        _ipcClient.OperationModeChanged += OnOperationModeChanged;
    }

    private void InitializeModes()
    {
        AvailableModes.Add(new ModeOption
        {
            Name = "MANUAL",
            DisplayName = "MANUAL (T1)",
            Description = "Teaching mode: Max 250mm/s, Deadman required",
            Color = Brushes.Gold
        });

        AvailableModes.Add(new ModeOption
        {
            Name = "TEST",
            DisplayName = "TEST (T2)",
            Description = "Test mode: Full speed, Operator supervision",
            Color = Brushes.Orange
        });

        AvailableModes.Add(new ModeOption
        {
            Name = "AUTO",
            DisplayName = "AUTO",
            Description = "Automatic mode: Full speed, Safety fence required",
            Color = Brushes.LimeGreen
        });

        AvailableModes.Add(new ModeOption
        {
            Name = "REMOTE",
            DisplayName = "REMOTE",
            Description = "External control: PLC control, Full safety system",
            Color = Brushes.DodgerBlue
        });

        // Select MANUAL by default
        SelectedMode = AvailableModes[0];
        AvailableModes[0].IsSelected = true;
    }

    private Brush GetModeColor(string mode)
    {
        return mode switch
        {
            "MANUAL" => Brushes.Gold,
            "TEST" => Brushes.Orange,
            "AUTO" => Brushes.LimeGreen,
            "REMOTE" => Brushes.DodgerBlue,
            _ => Brushes.Gray
        };
    }

    [RelayCommand]
    private async Task RefreshModeAsync()
    {
        if (!_ipcClient.IsConnected)
            return;

        try
        {
            var response = await _ipcClient.GetOperationModeAsync();
            if (response != null)
            {
                CurrentMode = response.Mode;
                ModeColor = GetModeColor(response.Mode);
                MaxLinearVelocity = response.MaxLinearVelocity;
                MaxJointVelocity = response.MaxJointVelocity;
                DeadmanRequired = response.DeadmanRequired;
                SafetyFenceRequired = response.SafetyFenceRequired;

                // Update selection
                foreach (var mode in AvailableModes)
                {
                    mode.IsSelected = mode.Name == response.Mode;
                    if (mode.IsSelected)
                    {
                        SelectedMode = mode;
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to refresh operation mode");
        }
    }

    [RelayCommand]
    private async Task SelectModeAsync(ModeOption? mode)
    {
        if (mode == null || mode.Name == CurrentMode)
            return;

        SelectedMode = mode;

        // Check requirements for this mode
        await CheckRequirementsAsync(mode.Name);
    }

    [RelayCommand]
    private async Task CheckRequirementsAsync(string mode)
    {
        if (!_ipcClient.IsConnected)
            return;

        IsCheckingRequirements = true;
        MissingRequirements.Clear();
        SwitchError = "";

        try
        {
            var response = await _ipcClient.GetModeRequirementsAsync(mode);
            if (response != null)
            {
                foreach (var item in response.MissingItems)
                {
                    MissingRequirements.Add(item);
                }

                CanSwitchMode = response.CanTransition;
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to check mode requirements");
            CanSwitchMode = false;
        }
        finally
        {
            IsCheckingRequirements = false;
        }
    }

    [RelayCommand]
    private async Task SwitchModeAsync()
    {
        if (SelectedMode == null || !CanSwitchMode || !_ipcClient.IsConnected)
            return;

        IsSwitching = true;
        SwitchError = "";

        try
        {
            var response = await _ipcClient.SetOperationModeAsync(SelectedMode.Name);
            if (response != null)
            {
                if (response.Success)
                {
                    CurrentMode = response.NewMode;
                    ModeColor = GetModeColor(response.NewMode);
                    Log.Information("Mode switched to {Mode}", response.NewMode);

                    // Refresh to get full mode info
                    await RefreshModeAsync();
                }
                else
                {
                    SwitchError = response.Error;
                    if (response.MissingRequirements.Count > 0)
                    {
                        MissingRequirements.Clear();
                        foreach (var req in response.MissingRequirements)
                        {
                            MissingRequirements.Add(req);
                        }
                    }
                    Log.Warning("Failed to switch mode: {Error}", response.Error);
                }
            }
        }
        catch (Exception ex)
        {
            SwitchError = "Failed to switch mode";
            Log.Error(ex, "Exception switching mode");
        }
        finally
        {
            IsSwitching = false;
        }
    }

    private void OnOperationModeChanged(object? sender, OperationModeChangedEvent e)
    {
        System.Windows.Application.Current?.Dispatcher.InvokeAsync(() =>
        {
            CurrentMode = e.NewMode;
            ModeColor = GetModeColor(e.NewMode);
            MaxLinearVelocity = e.MaxLinearVelocity;
            MaxJointVelocity = e.MaxJointVelocity;

            // Update selection
            foreach (var mode in AvailableModes)
            {
                mode.IsSelected = mode.Name == e.NewMode;
                if (mode.IsSelected)
                {
                    SelectedMode = mode;
                }
            }

            Log.Information("Operation mode changed: {Previous} -> {New}", e.PreviousMode, e.NewMode);
        });
    }
}
