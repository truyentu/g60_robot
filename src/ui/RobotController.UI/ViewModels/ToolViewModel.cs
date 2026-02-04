using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Microsoft.Extensions.Logging;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels;

/// <summary>
/// View model for a single tool item in the list
/// </summary>
public partial class ToolItemViewModel : ObservableObject
{
    [ObservableProperty]
    private string _id = string.Empty;

    [ObservableProperty]
    private string _name = string.Empty;

    [ObservableProperty]
    private string _description = string.Empty;

    [ObservableProperty]
    private bool _isActive;

    [ObservableProperty]
    private double _tcpX;

    [ObservableProperty]
    private double _tcpY;

    [ObservableProperty]
    private double _tcpZ;

    [ObservableProperty]
    private double _tcpRx;

    [ObservableProperty]
    private double _tcpRy;

    [ObservableProperty]
    private double _tcpRz;

    [ObservableProperty]
    private double _mass;

    public static ToolItemViewModel FromToolData(ToolData data)
    {
        return new ToolItemViewModel
        {
            Id = data.Id,
            Name = data.Name,
            Description = data.Description,
            IsActive = data.IsActive,
            TcpX = data.Tcp?.X ?? 0,
            TcpY = data.Tcp?.Y ?? 0,
            TcpZ = data.Tcp?.Z ?? 0,
            TcpRx = data.Tcp?.Rx ?? 0,
            TcpRy = data.Tcp?.Ry ?? 0,
            TcpRz = data.Tcp?.Rz ?? 0,
            Mass = data.Inertia?.Mass ?? 0
        };
    }

    public ToolData ToToolData()
    {
        return new ToolData
        {
            Id = Id,
            Name = Name,
            Description = Description,
            IsActive = IsActive,
            Tcp = new ToolTCPData
            {
                X = TcpX,
                Y = TcpY,
                Z = TcpZ,
                Rx = TcpRx,
                Ry = TcpRy,
                Rz = TcpRz
            },
            Inertia = new ToolInertiaData
            {
                Mass = Mass
            }
        };
    }
}

/// <summary>
/// ViewModel for Tool Management panel
/// </summary>
public partial class ToolViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;
    private readonly ILogger<ToolViewModel>? _logger;

    // ========================================================================
    // Observable Collections
    // ========================================================================

    public ObservableCollection<ToolItemViewModel> Tools { get; } = new();

    // ========================================================================
    // Observable Properties
    // ========================================================================

    [ObservableProperty]
    private ToolItemViewModel? _selectedTool;

    [ObservableProperty]
    private ToolItemViewModel? _activeTool;

    [ObservableProperty]
    private bool _isLoading;

    [ObservableProperty]
    private bool _isEditing;

    [ObservableProperty]
    private bool _isCreatingNew;

    [ObservableProperty]
    private string? _errorMessage;

    [ObservableProperty]
    private bool _hasError;

    // Edit form fields
    [ObservableProperty]
    private string _editId = string.Empty;

    [ObservableProperty]
    private string _editName = string.Empty;

    [ObservableProperty]
    private string _editDescription = string.Empty;

    [ObservableProperty]
    private double _editTcpX;

    [ObservableProperty]
    private double _editTcpY;

    [ObservableProperty]
    private double _editTcpZ;

    [ObservableProperty]
    private double _editTcpRx;

    [ObservableProperty]
    private double _editTcpRy;

    [ObservableProperty]
    private double _editTcpRz;

    [ObservableProperty]
    private double _editMass;

    // Calibration state
    [ObservableProperty]
    private bool _isCalibrating;

    [ObservableProperty]
    private string _calibrationMethod = "FOUR_POINT";

    [ObservableProperty]
    private int _calibrationPointsRecorded;

    [ObservableProperty]
    private int _calibrationPointsRequired = 4;

    [ObservableProperty]
    private string _calibrationStatus = "IDLE";

    public string[] AvailableCalibrationMethods { get; } = new[]
    {
        "FOUR_POINT",
        "SIX_POINT"
    };

    // ========================================================================
    // Constructor
    // ========================================================================

    public ToolViewModel(IIpcClientService ipcClient, ILogger<ToolViewModel>? logger = null)
    {
        _ipcClient = ipcClient;
        _logger = logger;

        // Subscribe to events
        _ipcClient.ToolChanged += OnToolChanged;
        _ipcClient.ConnectionStateChanged += OnConnectionStateChanged;
    }

    // ========================================================================
    // Commands - Tool CRUD
    // ========================================================================

    [RelayCommand]
    private async Task LoadToolsAsync()
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

            var response = await _ipcClient.GetToolListAsync();
            if (response != null)
            {
                Tools.Clear();
                foreach (var tool in response.Tools)
                {
                    var vm = ToolItemViewModel.FromToolData(tool);
                    Tools.Add(vm);

                    if (tool.Id == response.ActiveToolId)
                    {
                        ActiveTool = vm;
                    }
                }

                _logger?.LogInformation("Loaded {Count} tools, active: {ActiveId}",
                    response.Tools.Count, response.ActiveToolId);
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error loading tools");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private void StartCreateTool()
    {
        IsCreatingNew = true;
        IsEditing = true;

        // Clear edit form
        EditId = $"tool_{DateTime.Now:yyyyMMddHHmmss}";
        EditName = "New Tool";
        EditDescription = "";
        EditTcpX = 0;
        EditTcpY = 0;
        EditTcpZ = 0;
        EditTcpRx = 0;
        EditTcpRy = 0;
        EditTcpRz = 0;
        EditMass = 0;
    }

    [RelayCommand]
    private void StartEditTool()
    {
        if (SelectedTool == null) return;

        IsCreatingNew = false;
        IsEditing = true;

        // Populate edit form from selected tool
        EditId = SelectedTool.Id;
        EditName = SelectedTool.Name;
        EditDescription = SelectedTool.Description;
        EditTcpX = SelectedTool.TcpX;
        EditTcpY = SelectedTool.TcpY;
        EditTcpZ = SelectedTool.TcpZ;
        EditTcpRx = SelectedTool.TcpRx;
        EditTcpRy = SelectedTool.TcpRy;
        EditTcpRz = SelectedTool.TcpRz;
        EditMass = SelectedTool.Mass;
    }

    [RelayCommand]
    private async Task SaveToolAsync()
    {
        if (!_ipcClient.IsConnected) return;

        try
        {
            IsLoading = true;
            HasError = false;

            var toolData = new ToolData
            {
                Id = EditId,
                Name = EditName,
                Description = EditDescription,
                Tcp = new ToolTCPData
                {
                    X = EditTcpX,
                    Y = EditTcpY,
                    Z = EditTcpZ,
                    Rx = EditTcpRx,
                    Ry = EditTcpRy,
                    Rz = EditTcpRz
                },
                Inertia = new ToolInertiaData
                {
                    Mass = EditMass
                }
            };

            if (IsCreatingNew)
            {
                var response = await _ipcClient.CreateToolAsync(toolData);
                if (response?.Success == true)
                {
                    _logger?.LogInformation("Created tool: {Id}", EditId);
                    await LoadToolsAsync();
                }
                else
                {
                    ErrorMessage = response?.Error ?? "Failed to create tool";
                    HasError = true;
                }
            }
            else
            {
                var response = await _ipcClient.UpdateToolAsync(EditId, toolData);
                if (response?.Success == true)
                {
                    _logger?.LogInformation("Updated tool: {Id}", EditId);
                    await LoadToolsAsync();
                }
                else
                {
                    ErrorMessage = response?.Error ?? "Failed to update tool";
                    HasError = true;
                }
            }

            if (!HasError)
            {
                IsEditing = false;
                IsCreatingNew = false;
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error saving tool");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private void CancelEdit()
    {
        IsEditing = false;
        IsCreatingNew = false;
    }

    [RelayCommand]
    private async Task DeleteToolAsync()
    {
        if (SelectedTool == null || !_ipcClient.IsConnected) return;

        if (SelectedTool.Id == "tool_default")
        {
            ErrorMessage = "Cannot delete default tool";
            HasError = true;
            return;
        }

        try
        {
            IsLoading = true;
            HasError = false;

            var response = await _ipcClient.DeleteToolAsync(SelectedTool.Id);
            if (response?.Success == true)
            {
                _logger?.LogInformation("Deleted tool: {Id}", SelectedTool.Id);
                await LoadToolsAsync();
            }
            else
            {
                ErrorMessage = response?.Error ?? "Failed to delete tool";
                HasError = true;
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error deleting tool");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task SelectToolAsync(ToolItemViewModel? tool)
    {
        if (tool == null || !_ipcClient.IsConnected) return;

        try
        {
            IsLoading = true;
            HasError = false;

            var response = await _ipcClient.SelectToolAsync(tool.Id);
            if (response?.Success == true)
            {
                // Update active status
                foreach (var t in Tools)
                {
                    t.IsActive = (t.Id == tool.Id);
                }
                ActiveTool = tool;

                _logger?.LogInformation("Selected tool: {Id}", tool.Id);
            }
            else
            {
                ErrorMessage = response?.Error ?? "Failed to select tool";
                HasError = true;
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error selecting tool");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    // ========================================================================
    // Commands - Calibration
    // ========================================================================

    [RelayCommand]
    private async Task StartCalibrationAsync()
    {
        if (!_ipcClient.IsConnected) return;

        try
        {
            IsLoading = true;
            HasError = false;

            var response = await _ipcClient.StartTcpCalibrationAsync(CalibrationMethod);
            if (response?.Success == true)
            {
                IsCalibrating = true;
                CalibrationPointsRequired = response.PointsRequired;
                CalibrationPointsRecorded = 0;
                CalibrationStatus = "COLLECTING_POINTS";

                _logger?.LogInformation("Started {Method} calibration, {Points} points required",
                    CalibrationMethod, response.PointsRequired);
            }
            else
            {
                ErrorMessage = response?.Error ?? "Failed to start calibration";
                HasError = true;
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error starting calibration");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task RecordCalibrationPointAsync()
    {
        if (!_ipcClient.IsConnected || !IsCalibrating) return;

        try
        {
            IsLoading = true;
            HasError = false;

            // Get current joint positions (would be from MainViewModel or status)
            var jointAngles = new List<double> { 0, 0, 0, 0, 0, 0 };

            var response = await _ipcClient.RecordCalibrationPointAsync(jointAngles);
            if (response?.Success == true)
            {
                CalibrationPointsRecorded = response.PointIndex;

                _logger?.LogInformation("Recorded calibration point {Current}/{Total}",
                    response.PointIndex, response.TotalRequired);

                if (response.IsComplete)
                {
                    CalibrationStatus = "READY_TO_CALCULATE";
                }
            }
            else
            {
                ErrorMessage = response?.Error ?? "Failed to record point";
                HasError = true;
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error recording calibration point");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task FinishCalibrationAsync()
    {
        if (!_ipcClient.IsConnected || !IsCalibrating) return;

        try
        {
            IsLoading = true;
            HasError = false;

            var response = await _ipcClient.FinishCalibrationAsync();
            if (response?.Success == true && response.CalculatedTcp != null)
            {
                // Apply to edit form
                EditTcpX = response.CalculatedTcp.X;
                EditTcpY = response.CalculatedTcp.Y;
                EditTcpZ = response.CalculatedTcp.Z;
                EditTcpRx = response.CalculatedTcp.Rx;
                EditTcpRy = response.CalculatedTcp.Ry;
                EditTcpRz = response.CalculatedTcp.Rz;

                CalibrationStatus = "COMPLETED";
                IsCalibrating = false;

                _logger?.LogInformation("Calibration complete: TCP=({X:F2}, {Y:F2}, {Z:F2}), error={Error:F3}mm",
                    response.CalculatedTcp.X, response.CalculatedTcp.Y, response.CalculatedTcp.Z,
                    response.ResidualError);
            }
            else
            {
                ErrorMessage = response?.Error ?? "Calibration calculation failed";
                HasError = true;
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error finishing calibration");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task CancelCalibrationAsync()
    {
        if (!_ipcClient.IsConnected) return;

        try
        {
            await _ipcClient.CancelCalibrationAsync();
            IsCalibrating = false;
            CalibrationStatus = "IDLE";
            CalibrationPointsRecorded = 0;

            _logger?.LogInformation("Calibration cancelled");
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error cancelling calibration");
        }
    }

    // ========================================================================
    // Event Handlers
    // ========================================================================

    private void OnToolChanged(object? sender, ToolChangedEvent e)
    {
        System.Windows.Application.Current?.Dispatcher.Invoke(() =>
        {
            _logger?.LogInformation("Tool changed: {Id} ({Name})", e.ToolId, e.ToolName);

            // Update active tool
            foreach (var tool in Tools)
            {
                tool.IsActive = (tool.Id == e.ToolId);
                if (tool.IsActive)
                {
                    ActiveTool = tool;
                }
            }
        });
    }

    private async void OnConnectionStateChanged(object? sender, bool isConnected)
    {
        if (isConnected)
        {
            await LoadToolsAsync();
        }
        else
        {
            System.Windows.Application.Current?.Dispatcher.Invoke(() =>
            {
                Tools.Clear();
                ActiveTool = null;
                SelectedTool = null;
            });
        }
    }
}
