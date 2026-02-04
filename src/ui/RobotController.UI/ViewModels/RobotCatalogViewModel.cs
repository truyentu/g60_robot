using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Microsoft.Extensions.Logging;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels;

/// <summary>
/// View model for robot model item in catalog list
/// </summary>
public partial class RobotModelItemViewModel : ObservableObject
{
    [ObservableProperty]
    private string _id = string.Empty;

    [ObservableProperty]
    private string _name = string.Empty;

    [ObservableProperty]
    private string _manufacturer = string.Empty;

    [ObservableProperty]
    private int _dof = 6;

    [ObservableProperty]
    private double _maxPayloadKg;

    [ObservableProperty]
    private double _reachMm;

    [ObservableProperty]
    private bool _isSelected;

    public string DisplayInfo => $"{Manufacturer} | {Dof} DOF | {MaxPayloadKg:F1}kg | {ReachMm:F0}mm";
}

/// <summary>
/// ViewModel for Robot Catalog management
/// </summary>
public partial class RobotCatalogViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;
    private readonly ILogger<RobotCatalogViewModel>? _logger;

    // ========================================================================
    // Observable Properties
    // ========================================================================

    public ObservableCollection<RobotModelItemViewModel> AvailableModels { get; } = new();

    [ObservableProperty]
    private RobotModelItemViewModel? _selectedModel;

    [ObservableProperty]
    private string _activeModelId = string.Empty;

    [ObservableProperty]
    private string _activeModelName = string.Empty;

    [ObservableProperty]
    private string _activeInstanceId = string.Empty;

    [ObservableProperty]
    private bool _isLoading;

    [ObservableProperty]
    private string? _errorMessage;

    [ObservableProperty]
    private bool _hasError;

    // ========================================================================
    // Constructor
    // ========================================================================

    public RobotCatalogViewModel(IIpcClientService ipcClient, ILogger<RobotCatalogViewModel>? logger = null)
    {
        _ipcClient = ipcClient;
        _logger = logger;

        _ipcClient.RobotConfigChanged += OnRobotConfigChanged;
        _ipcClient.ConnectionStateChanged += OnConnectionStateChanged;
    }

    // ========================================================================
    // Commands
    // ========================================================================

    [RelayCommand]
    private async Task LoadCatalogAsync()
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

            var response = await _ipcClient.GetRobotCatalogAsync();
            if (response != null)
            {
                AvailableModels.Clear();
                foreach (var model in response.Models)
                {
                    var vm = new RobotModelItemViewModel
                    {
                        Id = model.Id,
                        Name = model.Name,
                        Manufacturer = model.Manufacturer,
                        Dof = model.Dof,
                        MaxPayloadKg = model.MaxPayloadKg,
                        ReachMm = model.ReachMm,
                        IsSelected = model.Id == response.ActiveModelId
                    };
                    AvailableModels.Add(vm);

                    if (vm.IsSelected)
                    {
                        SelectedModel = vm;
                    }
                }

                ActiveModelId = response.ActiveModelId;
                ActiveInstanceId = response.ActiveInstanceId;

                _logger?.LogInformation("Loaded {Count} robot models from catalog", AvailableModels.Count);
            }
            else
            {
                ErrorMessage = "Failed to load catalog";
                HasError = true;
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error loading robot catalog");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task SelectModelAsync(RobotModelItemViewModel? model)
    {
        if (model == null || !_ipcClient.IsConnected)
            return;

        try
        {
            IsLoading = true;
            HasError = false;
            ErrorMessage = null;

            var response = await _ipcClient.SelectRobotModelAsync(model.Id);
            if (response != null)
            {
                if (response.Success)
                {
                    // Update selection state
                    foreach (var m in AvailableModels)
                    {
                        m.IsSelected = m.Id == model.Id;
                    }
                    SelectedModel = model;
                    ActiveModelId = response.ModelId;
                    ActiveModelName = response.ModelName;

                    _logger?.LogInformation("Selected robot model: {ModelId}", model.Id);
                }
                else
                {
                    ErrorMessage = response.Error ?? "Failed to select model";
                    HasError = true;
                }
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error selecting robot model");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task RefreshActiveRobotAsync()
    {
        if (!_ipcClient.IsConnected)
            return;

        try
        {
            var response = await _ipcClient.GetActiveRobotAsync();
            if (response != null)
            {
                ActiveModelId = response.ModelId;
                ActiveModelName = response.ModelName;
                ActiveInstanceId = response.InstanceId;

                // Update selection in list
                foreach (var model in AvailableModels)
                {
                    model.IsSelected = model.Id == response.ModelId;
                    if (model.IsSelected)
                    {
                        SelectedModel = model;
                    }
                }
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error refreshing active robot");
        }
    }

    // ========================================================================
    // Event Handlers
    // ========================================================================

    private void OnRobotConfigChanged(object? sender, RobotConfigChangedEvent e)
    {
        // Update on UI thread
        System.Windows.Application.Current?.Dispatcher.Invoke(() =>
        {
            ActiveModelId = e.ModelId;
            ActiveModelName = e.ModelName;
            ActiveInstanceId = e.InstanceId;

            // Update selection
            foreach (var model in AvailableModels)
            {
                model.IsSelected = model.Id == e.ModelId;
                if (model.IsSelected)
                {
                    SelectedModel = model;
                }
            }
        });
    }

    private async void OnConnectionStateChanged(object? sender, bool isConnected)
    {
        if (isConnected)
        {
            await LoadCatalogAsync();
        }
        else
        {
            System.Windows.Application.Current?.Dispatcher.Invoke(() =>
            {
                AvailableModels.Clear();
                SelectedModel = null;
                ActiveModelId = string.Empty;
                ActiveModelName = string.Empty;
            });
        }
    }
}
