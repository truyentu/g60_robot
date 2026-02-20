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
/// ViewModel for Operation Mode management (KUKA KSS 8.3 §3.5.3, §4.12)
/// Modes: T1 (Manual), T2 (Test), AUT (Automatic), EXT (Automatic External)
/// </summary>
public partial class ModeViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;

    // Current mode
    [ObservableProperty]
    private string _currentMode = "T1";

    [ObservableProperty]
    private Brush _modeColor = Brushes.Gold;

    [ObservableProperty]
    private double _maxLinearVelocity = 250.0;

    [ObservableProperty]
    private double _maxJointVelocity = 30.0;

    [ObservableProperty]
    private bool _deadmanRequired = true;

    [ObservableProperty]
    private bool _safetyFenceRequired;

    // Mode-dependent feature flags
    [ObservableProperty]
    private bool _isJogEnabled = true;

    [ObservableProperty]
    private bool _isTeachEnabled = true;

    [ObservableProperty]
    private bool _isProgramStartEnabled = true;

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

    // Popup open state
    [ObservableProperty]
    private bool _isModePopupOpen;

    public ObservableCollection<ModeOption> AvailableModes { get; } = new();
    public ObservableCollection<string> MissingRequirements { get; } = new();

    public ModeViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;
        InitializeModes();
        _ipcClient.OperationModeChanged += OnOperationModeChanged;
    }

    private void InitializeModes()
    {
        AvailableModes.Add(new ModeOption
        {
            Name = "T1",
            DisplayName = "T1",
            Description = "Manual Reduced Velocity — Max 250mm/s, Jog + Teach enabled",
            Color = Brushes.Gold
        });

        AvailableModes.Add(new ModeOption
        {
            Name = "T2",
            DisplayName = "T2",
            Description = "Manual High Velocity — Full speed, Program run only",
            Color = Brushes.Orange
        });

        AvailableModes.Add(new ModeOption
        {
            Name = "AUT",
            DisplayName = "AUT",
            Description = "Automatic — Full speed, Start/Stop from UI",
            Color = Brushes.LimeGreen
        });

        AvailableModes.Add(new ModeOption
        {
            Name = "EXT",
            DisplayName = "EXT",
            Description = "Automatic External — PLC control",
            Color = Brushes.DodgerBlue
        });

        // Select T1 by default
        SelectedMode = AvailableModes[0];
        AvailableModes[0].IsSelected = true;
    }

    private Brush GetModeColor(string mode)
    {
        return mode switch
        {
            "T1" => Brushes.Gold,
            "T2" => Brushes.Orange,
            "AUT" => Brushes.LimeGreen,
            "EXT" => Brushes.DodgerBlue,
            // Legacy names for backward compat with IPC
            "MANUAL" => Brushes.Gold,
            "TEST" => Brushes.Orange,
            "AUTO" => Brushes.LimeGreen,
            "REMOTE" => Brushes.DodgerBlue,
            _ => Brushes.Gray
        };
    }

    private string NormalizeModeToKuka(string mode)
    {
        return mode switch
        {
            "MANUAL" => "T1",
            "TEST" => "T2",
            "AUTO" => "AUT",
            "REMOTE" => "EXT",
            _ => mode
        };
    }

    private void ApplyModeConstraints(string mode)
    {
        switch (mode)
        {
            case "T1":
                IsJogEnabled = true;
                IsTeachEnabled = true;
                IsProgramStartEnabled = true;
                MaxLinearVelocity = 250.0;
                DeadmanRequired = true;
                SafetyFenceRequired = false;
                break;
            case "T2":
                IsJogEnabled = false;
                IsTeachEnabled = false;
                IsProgramStartEnabled = true;
                MaxLinearVelocity = double.MaxValue;
                DeadmanRequired = true;
                SafetyFenceRequired = false;
                break;
            case "AUT":
                IsJogEnabled = false;
                IsTeachEnabled = false;
                IsProgramStartEnabled = true;
                MaxLinearVelocity = double.MaxValue;
                DeadmanRequired = false;
                SafetyFenceRequired = true;
                break;
            case "EXT":
                IsJogEnabled = false;
                IsTeachEnabled = false;
                IsProgramStartEnabled = false; // PLC controls start
                MaxLinearVelocity = double.MaxValue;
                DeadmanRequired = false;
                SafetyFenceRequired = true;
                break;
        }
    }

    [RelayCommand]
    private void ToggleModePopup()
    {
        IsModePopupOpen = !IsModePopupOpen;
    }

    [RelayCommand]
    private void SetMode(ModeOption? mode)
    {
        if (mode == null || mode.Name == CurrentMode)
        {
            IsModePopupOpen = false;
            return;
        }

        // Confirmation: warn if switching away from T1 (jog/teach will be disabled)
        if (CurrentMode == "T1" && (mode.Name == "T2" || mode.Name == "AUT" || mode.Name == "EXT"))
        {
            var result = System.Windows.MessageBox.Show(
                $"Switch to {mode.DisplayName}?\n\n" +
                "Jog and Teach functions will be disabled.\n" +
                (mode.Name == "T2" ? "Speed limit (250mm/s) will be removed." : "") +
                (mode.Name == "AUT" ? "Automatic mode — safety gate required." : "") +
                (mode.Name == "EXT" ? "External mode — PLC controls Start/Stop." : ""),
                "Change Operating Mode",
                System.Windows.MessageBoxButton.OKCancel,
                System.Windows.MessageBoxImage.Warning);

            if (result != System.Windows.MessageBoxResult.OK)
            {
                IsModePopupOpen = false;
                return;
            }
        }

        // Update selection
        foreach (var m in AvailableModes)
            m.IsSelected = m.Name == mode.Name;

        SelectedMode = mode;
        CurrentMode = mode.Name;
        ModeColor = GetModeColor(mode.Name);
        ApplyModeConstraints(mode.Name);

        IsModePopupOpen = false;
        Log.Information("Mode switched to {Mode}", mode.Name);

        // Also push to IPC if connected
        _ = PushModeToIpcAsync(mode.Name);
    }

    private async Task PushModeToIpcAsync(string mode)
    {
        if (!_ipcClient.IsConnected)
            return;

        try
        {
            await _ipcClient.SetOperationModeAsync(mode);
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to push mode {Mode} to IPC", mode);
        }
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
                var kukaMode = NormalizeModeToKuka(response.Mode);
                CurrentMode = kukaMode;
                ModeColor = GetModeColor(kukaMode);
                MaxJointVelocity = response.MaxJointVelocity;
                ApplyModeConstraints(kukaMode);

                foreach (var mode in AvailableModes)
                {
                    mode.IsSelected = mode.Name == kukaMode;
                    if (mode.IsSelected)
                        SelectedMode = mode;
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
            var kukaMode = NormalizeModeToKuka(e.NewMode);
            CurrentMode = kukaMode;
            ModeColor = GetModeColor(kukaMode);
            MaxJointVelocity = e.MaxJointVelocity;
            ApplyModeConstraints(kukaMode);

            foreach (var mode in AvailableModes)
            {
                mode.IsSelected = mode.Name == kukaMode;
                if (mode.IsSelected)
                    SelectedMode = mode;
            }

            Log.Information("Operation mode changed: {Previous} -> {New}",
                NormalizeModeToKuka(e.PreviousMode), kukaMode);
        });
    }
}
