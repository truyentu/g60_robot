using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Microsoft.Extensions.Logging;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels;

/// <summary>
/// View model for joint homing status item
/// </summary>
public partial class JointHomingItemViewModel : ObservableObject
{
    [ObservableProperty]
    private int _jointIndex;

    [ObservableProperty]
    private string _jointName = string.Empty;

    [ObservableProperty]
    private string _state = "NOT_HOMED";

    [ObservableProperty]
    private double _progress;

    [ObservableProperty]
    private string _errorMessage = string.Empty;

    [ObservableProperty]
    private bool _limitSwitchActive;

    [ObservableProperty]
    private double _currentPosition;

    public bool IsHomed => State == "HOMED";
    public bool IsHoming => State == "HOMING_IN_PROGRESS";
    public bool HasError => State == "HOMING_ERROR";
    public bool IsNotHomed => State == "NOT_HOMED";

    public string StatusIcon => State switch
    {
        "HOMED" => "✓",
        "HOMING_IN_PROGRESS" => "⟳",
        "HOMING_ERROR" => "✗",
        _ => "○"
    };

    public void UpdateFrom(JointHomingStatus status)
    {
        State = status.State;
        Progress = status.Progress;
        ErrorMessage = status.ErrorMessage;
        LimitSwitchActive = status.LimitSwitchActive;
        CurrentPosition = status.CurrentPosition;

        OnPropertyChanged(nameof(IsHomed));
        OnPropertyChanged(nameof(IsHoming));
        OnPropertyChanged(nameof(HasError));
        OnPropertyChanged(nameof(IsNotHomed));
        OnPropertyChanged(nameof(StatusIcon));
    }
}

/// <summary>
/// ViewModel for Robot Homing management
/// </summary>
public partial class HomingViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;
    private readonly ILogger<HomingViewModel>? _logger;

    // ========================================================================
    // Observable Properties
    // ========================================================================

    public ObservableCollection<JointHomingItemViewModel> JointStates { get; } = new();

    [ObservableProperty]
    private bool _isLoading;

    [ObservableProperty]
    private bool _isHomingInProgress;

    [ObservableProperty]
    private bool _isAllHomed;

    [ObservableProperty]
    private bool _hasError;

    [ObservableProperty]
    private string? _errorMessage;

    [ObservableProperty]
    private string _selectedMethod = "LIMIT_SWITCH";

    [ObservableProperty]
    private int _homedCount;

    [ObservableProperty]
    private int _totalJoints = 6;

    public string[] AvailableMethods { get; } = new[]
    {
        "LIMIT_SWITCH",
        "INDEX_PULSE",
        "MANUAL",
        "ABSOLUTE_ENCODER"
    };

    // ========================================================================
    // Constructor
    // ========================================================================

    public HomingViewModel(IIpcClientService ipcClient, ILogger<HomingViewModel>? logger = null)
    {
        _ipcClient = ipcClient;
        _logger = logger;

        // Initialize joint states
        for (int i = 0; i < 6; i++)
        {
            JointStates.Add(new JointHomingItemViewModel
            {
                JointIndex = i,
                JointName = $"J{i + 1}",
                State = "NOT_HOMED"
            });
        }

        // Subscribe to events
        _ipcClient.HomingStateChanged += OnHomingStateChanged;
        _ipcClient.ConnectionStateChanged += OnConnectionStateChanged;
    }

    // ========================================================================
    // Commands
    // ========================================================================

    [RelayCommand]
    private async Task LoadHomingStateAsync()
    {
        if (!_ipcClient.IsConnected)
        {
            ErrorMessage = "Not connected to Core";
            HasError = true;
            return;
        }

        try
        {
            IsLoading = true;
            HasError = false;
            ErrorMessage = null;

            var response = await _ipcClient.GetHomingStateAsync();
            if (response != null)
            {
                UpdateFromResponse(response);
                _logger?.LogInformation("Loaded homing state: {HomedCount}/{TotalJoints} homed",
                    response.HomedCount, response.TotalJoints);
            }
            else
            {
                ErrorMessage = "Failed to get homing state";
                HasError = true;
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error loading homing state");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task StartHomingAllAsync()
    {
        if (!_ipcClient.IsConnected || IsHomingInProgress)
            return;

        try
        {
            IsLoading = true;
            HasError = false;
            ErrorMessage = null;

            _logger?.LogInformation("Starting homing for all joints, method: {Method}", SelectedMethod);

            var response = await _ipcClient.StartHomingAsync(-1, SelectedMethod);
            if (response != null)
            {
                if (response.Success)
                {
                    IsHomingInProgress = true;
                    _logger?.LogInformation("Homing started successfully");
                }
                else
                {
                    ErrorMessage = response.Error ?? "Failed to start homing";
                    HasError = true;
                }
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error starting homing");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task StartHomingSingleAsync(int jointIndex)
    {
        if (!_ipcClient.IsConnected || IsHomingInProgress)
            return;

        try
        {
            IsLoading = true;
            HasError = false;
            ErrorMessage = null;

            _logger?.LogInformation("Starting homing for joint {Joint}, method: {Method}", jointIndex, SelectedMethod);

            var response = await _ipcClient.StartHomingAsync(jointIndex, SelectedMethod);
            if (response != null)
            {
                if (response.Success)
                {
                    IsHomingInProgress = true;
                    if (jointIndex >= 0 && jointIndex < JointStates.Count)
                    {
                        JointStates[jointIndex].State = "HOMING_IN_PROGRESS";
                    }
                }
                else
                {
                    ErrorMessage = response.Error ?? "Failed to start homing";
                    HasError = true;
                }
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error starting homing for joint {Joint}", jointIndex);
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task StopHomingAsync()
    {
        if (!_ipcClient.IsConnected)
            return;

        try
        {
            IsLoading = true;

            _logger?.LogInformation("Stopping homing");

            var response = await _ipcClient.StopHomingAsync(-1);
            if (response != null && response.Success)
            {
                IsHomingInProgress = false;
                await LoadHomingStateAsync();
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error stopping homing");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task RefreshAsync()
    {
        await LoadHomingStateAsync();
    }

    // ========================================================================
    // Event Handlers
    // ========================================================================

    private void OnHomingStateChanged(object? sender, HomingStateChangedEvent e)
    {
        System.Windows.Application.Current?.Dispatcher.Invoke(() =>
        {
            if (e.JointIndex >= 0 && e.JointIndex < JointStates.Count)
            {
                var joint = JointStates[e.JointIndex];
                joint.State = e.NewState;
                joint.ErrorMessage = e.ErrorMessage;

                _logger?.LogInformation("Joint {Joint} state changed: {OldState} -> {NewState}",
                    e.JointIndex, e.PreviousState, e.NewState);
            }

            // Update summary
            UpdateSummary();
        });
    }

    private async void OnConnectionStateChanged(object? sender, bool isConnected)
    {
        if (isConnected)
        {
            await LoadHomingStateAsync();
        }
        else
        {
            System.Windows.Application.Current?.Dispatcher.Invoke(() =>
            {
                foreach (var joint in JointStates)
                {
                    joint.State = "NOT_HOMED";
                    joint.Progress = 0;
                    joint.ErrorMessage = string.Empty;
                }
                IsHomingInProgress = false;
                IsAllHomed = false;
                HomedCount = 0;
            });
        }
    }

    // ========================================================================
    // Private Methods
    // ========================================================================

    private void UpdateFromResponse(HomingStateResponse response)
    {
        TotalJoints = response.TotalJoints;
        HomedCount = response.HomedCount;
        IsAllHomed = response.AllHomed;
        IsHomingInProgress = response.AnyHoming;
        HasError = response.AnyError;

        for (int i = 0; i < response.Joints.Count && i < JointStates.Count; i++)
        {
            JointStates[i].UpdateFrom(response.Joints[i]);
        }
    }

    private void UpdateSummary()
    {
        int homed = 0;
        bool anyHoming = false;
        bool anyError = false;

        foreach (var joint in JointStates)
        {
            if (joint.IsHomed) homed++;
            if (joint.IsHoming) anyHoming = true;
            if (joint.HasError) anyError = true;
        }

        HomedCount = homed;
        IsAllHomed = homed == JointStates.Count;
        IsHomingInProgress = anyHoming;
        HasError = anyError;
    }
}
