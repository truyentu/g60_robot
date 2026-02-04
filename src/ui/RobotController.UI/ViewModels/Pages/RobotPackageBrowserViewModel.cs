using System.Collections.ObjectModel;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using RobotController.UI.Models;
using RobotController.UI.Services;
using Serilog;

namespace RobotController.UI.ViewModels.Pages;

/// <summary>
/// Robot package info for display in browser
/// </summary>
public partial class RobotPackageInfoViewModel : ObservableObject
{
    [ObservableProperty]
    private string _id = "";

    [ObservableProperty]
    private string _name = "";

    [ObservableProperty]
    private string _manufacturer = "";

    [ObservableProperty]
    private string _modelType = "";

    [ObservableProperty]
    private double _payloadKg;

    [ObservableProperty]
    private double _reachMm;

    [ObservableProperty]
    private int _dof = 6;

    [ObservableProperty]
    private bool _hasMeshes;

    [ObservableProperty]
    private string _thumbnailPath = "";
}

/// <summary>
/// ViewModel for Robot Package Browser
/// </summary>
public partial class RobotPackageBrowserViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;
    private readonly IViewportService _viewportService;

    [ObservableProperty]
    private ObservableCollection<RobotPackageInfoViewModel> _availablePackages = new();

    [ObservableProperty]
    private RobotPackageInfoViewModel? _selectedPackage;

    [ObservableProperty]
    private bool _isLoading;

    [ObservableProperty]
    private string _statusMessage = "Ready";

    [ObservableProperty]
    private RobotPackageData? _loadedPackage;

    public RobotPackageBrowserViewModel(IIpcClientService ipcClient, IViewportService viewportService)
    {
        _ipcClient = ipcClient;
        _viewportService = viewportService;
    }

    [RelayCommand]
    private async Task RefreshPackagesAsync()
    {
        try
        {
            IsLoading = true;
            StatusMessage = "Loading packages...";

            var response = await _ipcClient.GetRobotPackagesAsync();

            if (response != null && response.Success)
            {
                AvailablePackages.Clear();

                foreach (var pkg in response.Packages)
                {
                    var info = new RobotPackageInfoViewModel
                    {
                        Id = pkg.Id,
                        Name = pkg.Name,
                        Manufacturer = pkg.Manufacturer,
                        ModelType = pkg.ModelType,
                        PayloadKg = pkg.PayloadKg,
                        ReachMm = pkg.ReachMm,
                        Dof = pkg.Dof,
                        HasMeshes = pkg.HasMeshes,
                        ThumbnailPath = pkg.ThumbnailPath
                    };
                    AvailablePackages.Add(info);
                }

                StatusMessage = $"Found {AvailablePackages.Count} packages";
            }
            else
            {
                StatusMessage = "Failed to load packages";
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to refresh packages");
            StatusMessage = "Error: " + ex.Message;
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task LoadSelectedPackageAsync()
    {
        if (SelectedPackage == null) return;

        try
        {
            IsLoading = true;
            StatusMessage = $"Loading {SelectedPackage.Name}...";

            var response = await _ipcClient.LoadRobotPackageAsync(SelectedPackage.Id);

            if (response != null && response.Success && response.Package != null)
            {
                var pkg = response.Package;

                // Parse package data
                LoadedPackage = new RobotPackageData
                {
                    Name = pkg.Name,
                    Id = pkg.Id,
                    Manufacturer = pkg.Manufacturer,
                    ModelType = pkg.ModelType,
                    PayloadKg = pkg.PayloadKg,
                    ReachMm = pkg.ReachMm,
                    DhConvention = pkg.DhConvention,
                    PackagePath = pkg.PackagePath
                };

                // Parse joints
                foreach (var j in pkg.Joints)
                {
                    var joint = new JointDefinitionData
                    {
                        Name = j.Name,
                        Type = j.Type,
                        DhA = j.DhA,
                        DhAlpha = j.DhAlpha,
                        DhD = j.DhD,
                        DhThetaOffset = j.DhThetaOffset,
                        LimitMin = j.LimitMin,
                        LimitMax = j.LimitMax,
                        VelocityMax = j.VelocityMax,
                        AccelerationMax = j.AccelerationMax
                    };

                    if (j.Mesh != null)
                    {
                        joint.Mesh = new JointMeshData
                        {
                            VisualMesh = j.Mesh.VisualMesh,
                            CollisionMesh = j.Mesh.CollisionMesh
                        };
                    }

                    LoadedPackage.Joints.Add(joint);
                }

                // Parse home position
                LoadedPackage.HomePosition = pkg.HomePosition;

                // Update viewport
                await _viewportService.InitializeFromPackageAsync(LoadedPackage);

                StatusMessage = $"Loaded: {LoadedPackage.Name}";
            }
            else
            {
                var error = response?.Error ?? "Unknown error";
                StatusMessage = $"Failed: {error}";
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to load package");
            StatusMessage = "Error: " + ex.Message;
        }
        finally
        {
            IsLoading = false;
        }
    }
}
