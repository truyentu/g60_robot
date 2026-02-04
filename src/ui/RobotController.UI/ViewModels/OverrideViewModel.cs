using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using Serilog;

namespace RobotController.UI.ViewModels;

/// <summary>
/// ViewModel for override control panel (KUKA-inspired triple override)
/// </summary>
public partial class OverrideViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;

    [ObservableProperty]
    private int _programOverride = 100;

    [ObservableProperty]
    private int _jogOverride = 100;

    [ObservableProperty]
    private int _manualOverride = 100;

    [ObservableProperty]
    private bool _isUpdating;

    public OverrideViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;

        _ipcClient.StatusReceived += OnStatusReceived;
        _ipcClient.OverrideChanged += OnOverrideChanged;
    }

    [RelayCommand]
    private async Task SetProgramOverrideAsync(int value)
    {
        if (IsUpdating) return;
        IsUpdating = true;

        try
        {
            var response = await _ipcClient.SetOverrideAsync(programOverride: value);
            if (response?.Success == true)
            {
                ProgramOverride = response.ProgramOverride;
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to set program override");
        }
        finally
        {
            IsUpdating = false;
        }
    }

    [RelayCommand]
    private async Task SetJogOverrideAsync(int value)
    {
        if (IsUpdating) return;
        IsUpdating = true;

        try
        {
            var response = await _ipcClient.SetOverrideAsync(jogOverride: value);
            if (response?.Success == true)
            {
                JogOverride = response.JogOverride;
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to set jog override");
        }
        finally
        {
            IsUpdating = false;
        }
    }

    [RelayCommand]
    private async Task SetManualOverrideAsync(int value)
    {
        if (IsUpdating) return;
        IsUpdating = true;

        try
        {
            var response = await _ipcClient.SetOverrideAsync(manualOverride: value);
            if (response?.Success == true)
            {
                ManualOverride = response.ManualOverride;
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to set manual override");
        }
        finally
        {
            IsUpdating = false;
        }
    }

    [RelayCommand]
    private async Task LoadOverridesAsync()
    {
        try
        {
            var response = await _ipcClient.GetOverrideAsync();
            if (response != null)
            {
                ProgramOverride = response.ProgramOverride;
                JogOverride = response.JogOverride;
                ManualOverride = response.ManualOverride;
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to load overrides");
        }
    }

    private void OnStatusReceived(object? sender, StatusPayload status)
    {
        if (!IsUpdating)
        {
            ProgramOverride = status.ProgramOverride;
            JogOverride = status.JogOverride;
            ManualOverride = status.ManualOverride;
        }
    }

    private void OnOverrideChanged(object? sender, OverrideChangedEvent e)
    {
        if (!IsUpdating)
        {
            ProgramOverride = e.ProgramOverride;
            JogOverride = e.JogOverride;
            ManualOverride = e.ManualOverride;
        }
    }
}
