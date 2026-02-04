using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Microsoft.Extensions.Logging;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels;

/// <summary>
/// View model for a single base frame item in the list
/// </summary>
public partial class BaseFrameItemViewModel : ObservableObject
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
    private double _x;

    [ObservableProperty]
    private double _y;

    [ObservableProperty]
    private double _z;

    [ObservableProperty]
    private double _rx;

    [ObservableProperty]
    private double _ry;

    [ObservableProperty]
    private double _rz;

    public static BaseFrameItemViewModel FromBaseFrameData(BaseFrameData data)
    {
        return new BaseFrameItemViewModel
        {
            Id = data.Id,
            Name = data.Name,
            Description = data.Description,
            IsActive = data.IsActive,
            X = data.Frame?.X ?? 0,
            Y = data.Frame?.Y ?? 0,
            Z = data.Frame?.Z ?? 0,
            Rx = data.Frame?.Rx ?? 0,
            Ry = data.Frame?.Ry ?? 0,
            Rz = data.Frame?.Rz ?? 0
        };
    }

    public BaseFrameData ToBaseFrameData()
    {
        return new BaseFrameData
        {
            Id = Id,
            Name = Name,
            Description = Description,
            IsActive = IsActive,
            Frame = new FrameData
            {
                X = X,
                Y = Y,
                Z = Z,
                Rx = Rx,
                Ry = Ry,
                Rz = Rz
            }
        };
    }
}

/// <summary>
/// Calibration point status for UI display
/// </summary>
public partial class CalibrationPointStatus : ObservableObject
{
    [ObservableProperty]
    private int _pointIndex;

    [ObservableProperty]
    private string _pointName = string.Empty;

    [ObservableProperty]
    private bool _isRecorded;

    [ObservableProperty]
    private bool _isCurrent;

    [ObservableProperty]
    private double _recordedX;

    [ObservableProperty]
    private double _recordedY;

    [ObservableProperty]
    private double _recordedZ;
}

/// <summary>
/// ViewModel for Base Frame Management panel
/// </summary>
public partial class BaseFrameViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;
    private readonly ILogger<BaseFrameViewModel>? _logger;

    // ========================================================================
    // Observable Collections
    // ========================================================================

    public ObservableCollection<BaseFrameItemViewModel> Bases { get; } = new();
    public ObservableCollection<CalibrationPointStatus> CalibrationPoints { get; } = new();

    // ========================================================================
    // Observable Properties
    // ========================================================================

    [ObservableProperty]
    private BaseFrameItemViewModel? _selectedBase;

    [ObservableProperty]
    private BaseFrameItemViewModel? _activeBase;

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
    private double _editX;

    [ObservableProperty]
    private double _editY;

    [ObservableProperty]
    private double _editZ;

    [ObservableProperty]
    private double _editRx;

    [ObservableProperty]
    private double _editRy;

    [ObservableProperty]
    private double _editRz;

    // Calibration state
    [ObservableProperty]
    private bool _isCalibrating;

    [ObservableProperty]
    private string _calibrationMethod = "THREE_POINT";

    [ObservableProperty]
    private int _calibrationPointsRecorded;

    [ObservableProperty]
    private int _calibrationPointsRequired = 3;

    [ObservableProperty]
    private string _calibrationStatus = "IDLE";

    [ObservableProperty]
    private string _currentPointName = string.Empty;

    public string[] AvailableCalibrationMethods { get; } = new[]
    {
        "THREE_POINT",
        "FOUR_POINT"
    };

    // ========================================================================
    // Constructor
    // ========================================================================

    public BaseFrameViewModel(IIpcClientService ipcClient, ILogger<BaseFrameViewModel>? logger = null)
    {
        _ipcClient = ipcClient;
        _logger = logger;

        // Subscribe to events
        _ipcClient.BaseChanged += OnBaseChanged;
        _ipcClient.ConnectionStateChanged += OnConnectionStateChanged;

        // Initialize calibration points for 3-point method
        InitializeCalibrationPoints();
    }

    private void InitializeCalibrationPoints()
    {
        CalibrationPoints.Clear();
        CalibrationPoints.Add(new CalibrationPointStatus
        {
            PointIndex = 0,
            PointName = "Origin",
            IsRecorded = false,
            IsCurrent = true
        });
        CalibrationPoints.Add(new CalibrationPointStatus
        {
            PointIndex = 1,
            PointName = "X-Direction",
            IsRecorded = false,
            IsCurrent = false
        });
        CalibrationPoints.Add(new CalibrationPointStatus
        {
            PointIndex = 2,
            PointName = "XY-Plane",
            IsRecorded = false,
            IsCurrent = false
        });
    }

    // ========================================================================
    // Commands - Base Frame CRUD
    // ========================================================================

    [RelayCommand]
    private async Task LoadBasesAsync()
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

            var response = await _ipcClient.GetBaseListAsync();
            if (response != null)
            {
                Bases.Clear();
                foreach (var baseFrame in response.Bases)
                {
                    var vm = BaseFrameItemViewModel.FromBaseFrameData(baseFrame);
                    Bases.Add(vm);

                    if (baseFrame.Id == response.ActiveBaseId)
                    {
                        ActiveBase = vm;
                    }
                }

                _logger?.LogInformation("Loaded {Count} base frames, active: {ActiveId}",
                    response.Bases.Count, response.ActiveBaseId);
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error loading base frames");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private void StartCreateBase()
    {
        IsCreatingNew = true;
        IsEditing = true;

        // Clear edit form
        EditId = $"base_{DateTime.Now:yyyyMMddHHmmss}";
        EditName = "New Base";
        EditDescription = "";
        EditX = 0;
        EditY = 0;
        EditZ = 0;
        EditRx = 0;
        EditRy = 0;
        EditRz = 0;
    }

    [RelayCommand]
    private void StartEditBase()
    {
        if (SelectedBase == null) return;

        IsCreatingNew = false;
        IsEditing = true;

        // Populate edit form from selected base
        EditId = SelectedBase.Id;
        EditName = SelectedBase.Name;
        EditDescription = SelectedBase.Description;
        EditX = SelectedBase.X;
        EditY = SelectedBase.Y;
        EditZ = SelectedBase.Z;
        EditRx = SelectedBase.Rx;
        EditRy = SelectedBase.Ry;
        EditRz = SelectedBase.Rz;
    }

    [RelayCommand]
    private async Task SaveBaseAsync()
    {
        if (!_ipcClient.IsConnected) return;

        try
        {
            IsLoading = true;
            HasError = false;

            var baseData = new BaseFrameData
            {
                Id = EditId,
                Name = EditName,
                Description = EditDescription,
                Frame = new FrameData
                {
                    X = EditX,
                    Y = EditY,
                    Z = EditZ,
                    Rx = EditRx,
                    Ry = EditRy,
                    Rz = EditRz
                }
            };

            if (IsCreatingNew)
            {
                var response = await _ipcClient.CreateBaseAsync(baseData);
                if (response?.Success == true)
                {
                    _logger?.LogInformation("Created base frame: {Id}", EditId);
                    await LoadBasesAsync();
                }
                else
                {
                    ErrorMessage = response?.Error ?? "Failed to create base frame";
                    HasError = true;
                }
            }
            else
            {
                var response = await _ipcClient.UpdateBaseAsync(EditId, baseData);
                if (response?.Success == true)
                {
                    _logger?.LogInformation("Updated base frame: {Id}", EditId);
                    await LoadBasesAsync();
                }
                else
                {
                    ErrorMessage = response?.Error ?? "Failed to update base frame";
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
            _logger?.LogError(ex, "Error saving base frame");
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
    private async Task DeleteBaseAsync()
    {
        if (SelectedBase == null || !_ipcClient.IsConnected) return;

        if (SelectedBase.Id == "world")
        {
            ErrorMessage = "Cannot delete world frame";
            HasError = true;
            return;
        }

        try
        {
            IsLoading = true;
            HasError = false;

            var response = await _ipcClient.DeleteBaseAsync(SelectedBase.Id);
            if (response?.Success == true)
            {
                _logger?.LogInformation("Deleted base frame: {Id}", SelectedBase.Id);
                await LoadBasesAsync();
            }
            else
            {
                ErrorMessage = response?.Error ?? "Failed to delete base frame";
                HasError = true;
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error deleting base frame");
            ErrorMessage = ex.Message;
            HasError = true;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task SelectBaseAsync(BaseFrameItemViewModel? baseFrame)
    {
        if (baseFrame == null || !_ipcClient.IsConnected) return;

        try
        {
            IsLoading = true;
            HasError = false;

            var response = await _ipcClient.SelectBaseAsync(baseFrame.Id);
            if (response?.Success == true)
            {
                // Update active status
                foreach (var b in Bases)
                {
                    b.IsActive = (b.Id == baseFrame.Id);
                }
                ActiveBase = baseFrame;

                _logger?.LogInformation("Selected base frame: {Id}", baseFrame.Id);
            }
            else
            {
                ErrorMessage = response?.Error ?? "Failed to select base frame";
                HasError = true;
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error selecting base frame");
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

            var response = await _ipcClient.StartBaseCalibrationAsync(CalibrationMethod);
            if (response?.Success == true)
            {
                IsCalibrating = true;
                CalibrationPointsRequired = response.PointsRequired;
                CalibrationPointsRecorded = 0;
                CalibrationStatus = "COLLECTING_POINTS";

                // Reset calibration points
                InitializeCalibrationPoints();
                if (CalibrationMethod == "FOUR_POINT" && CalibrationPoints.Count < 4)
                {
                    CalibrationPoints.Add(new CalibrationPointStatus
                    {
                        PointIndex = 3,
                        PointName = "Z-Direction",
                        IsRecorded = false,
                        IsCurrent = false
                    });
                }

                CurrentPointName = CalibrationPoints[0].PointName;

                _logger?.LogInformation("Started {Method} base calibration, {Points} points required",
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
            _logger?.LogError(ex, "Error starting base calibration");
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

            // Get current joint positions and TCP position from robot
            var jointAngles = new List<double> { 0, 0, 0, 0, 0, 0 };
            var tcpPosition = new List<double> { 0, 0, 0, 0, 0, 0 };

            var response = await _ipcClient.RecordBasePointAsync(
                CalibrationPointsRecorded,
                jointAngles,
                tcpPosition);

            if (response?.Success == true)
            {
                // Update calibration point status
                if (CalibrationPointsRecorded < CalibrationPoints.Count)
                {
                    CalibrationPoints[CalibrationPointsRecorded].IsRecorded = true;
                    CalibrationPoints[CalibrationPointsRecorded].IsCurrent = false;
                }

                CalibrationPointsRecorded = response.RecordedPoints;

                // Move current marker to next point
                if (CalibrationPointsRecorded < CalibrationPoints.Count)
                {
                    CalibrationPoints[CalibrationPointsRecorded].IsCurrent = true;
                    CurrentPointName = CalibrationPoints[CalibrationPointsRecorded].PointName;
                }

                _logger?.LogInformation("Recorded base calibration point {Current}/{Total}: {Name}",
                    response.RecordedPoints, response.TotalPoints, response.PointName);

                if (response.RecordedPoints >= response.TotalPoints)
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
            _logger?.LogError(ex, "Error recording base calibration point");
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

            var response = await _ipcClient.FinishBaseCalibrationAsync();
            if (response?.Success == true && response.CalculatedFrame != null)
            {
                // Apply calculated frame to edit form
                EditX = response.CalculatedFrame.X;
                EditY = response.CalculatedFrame.Y;
                EditZ = response.CalculatedFrame.Z;
                EditRx = response.CalculatedFrame.Rx;
                EditRy = response.CalculatedFrame.Ry;
                EditRz = response.CalculatedFrame.Rz;

                CalibrationStatus = "COMPLETED";
                IsCalibrating = false;
                IsEditing = true;
                IsCreatingNew = true;

                _logger?.LogInformation("Base calibration complete: ({X:F2}, {Y:F2}, {Z:F2}) / ({Rx:F2}, {Ry:F2}, {Rz:F2})",
                    response.CalculatedFrame.X, response.CalculatedFrame.Y, response.CalculatedFrame.Z,
                    response.CalculatedFrame.Rx, response.CalculatedFrame.Ry, response.CalculatedFrame.Rz);
            }
            else
            {
                ErrorMessage = response?.Error ?? "Calibration calculation failed";
                HasError = true;
            }
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error finishing base calibration");
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
            await _ipcClient.CancelBaseCalibrationAsync();
            IsCalibrating = false;
            CalibrationStatus = "IDLE";
            CalibrationPointsRecorded = 0;
            InitializeCalibrationPoints();

            _logger?.LogInformation("Base calibration cancelled");
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Error cancelling base calibration");
        }
    }

    // ========================================================================
    // Event Handlers
    // ========================================================================

    private void OnBaseChanged(object? sender, BaseChangedEvent e)
    {
        System.Windows.Application.Current?.Dispatcher.Invoke(() =>
        {
            _logger?.LogInformation("Base frame changed: {Id} ({Name})", e.BaseId, e.BaseName);

            // Update active base
            foreach (var baseFrame in Bases)
            {
                baseFrame.IsActive = (baseFrame.Id == e.BaseId);
                if (baseFrame.IsActive)
                {
                    ActiveBase = baseFrame;
                }
            }
        });
    }

    private async void OnConnectionStateChanged(object? sender, bool isConnected)
    {
        if (isConnected)
        {
            await LoadBasesAsync();
        }
        else
        {
            System.Windows.Application.Current?.Dispatcher.Invoke(() =>
            {
                Bases.Clear();
                ActiveBase = null;
                SelectedBase = null;
            });
        }
    }
}
