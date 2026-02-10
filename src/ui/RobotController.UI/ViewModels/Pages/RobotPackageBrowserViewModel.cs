using System.Collections.ObjectModel;
using System.Windows;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using RobotController.UI.Models;
using RobotController.UI.Services;
using RobotController.UI.ViewModels.Dialogs;
using RobotController.UI.Views.Dialogs;
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
    private readonly IUrdfImportService _urdfImportService;
    private readonly IRobotPackageGenerator _packageGenerator;
    private readonly IConfigService? _configService;

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

    public RobotPackageBrowserViewModel(
        IIpcClientService ipcClient,
        IViewportService viewportService,
        IUrdfImportService urdfImportService,
        IRobotPackageGenerator packageGenerator,
        IConfigService? configService = null)
    {
        _ipcClient = ipcClient;
        _viewportService = viewportService;
        _urdfImportService = urdfImportService;
        _packageGenerator = packageGenerator;
        _configService = configService;
    }

    /// <summary>
    /// Design-time constructor
    /// </summary>
    public RobotPackageBrowserViewModel()
        : this(null!, null!, new UrdfImportService(), new RobotPackageGenerator(), null)
    {
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

                // Build package path from ID - UI meshes are in config/robots/<id>/
                // Core returns relative path like "../../config/robots/rx160" which doesn't work for UI
                // So we construct the path ourselves based on package ID
                var packagePath = System.IO.Path.GetFullPath(
                    System.IO.Path.Combine(
                        AppDomain.CurrentDomain.BaseDirectory,
                        "config", "robots", pkg.Id));

                Log.Information("Package {Id} path resolved to: {Path}", pkg.Id, packagePath);

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
                    PackagePath = packagePath
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
                        // URDF origin data for visualization
                        OriginXyz = j.OriginXyz,
                        OriginRpy = j.OriginRpy,
                        Axis = j.Axis,
                        LimitMin = j.LimitMin,
                        LimitMax = j.LimitMax,
                        VelocityMax = j.VelocityMax,
                        AccelerationMax = j.AccelerationMax
                    };

                    // Debug: Log URDF data mapping
                    if (j.OriginXyz != null)
                    {
                        Log.Debug("Joint {Name} has URDF origin_xyz: [{X}, {Y}, {Z}]",
                            j.Name, j.OriginXyz[0], j.OriginXyz[1], j.OriginXyz[2]);
                    }
                    else
                    {
                        Log.Warning("Joint {Name} missing URDF origin_xyz in payload", j.Name);
                    }

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

                // Parse base mesh and home position
                LoadedPackage.BaseMesh = pkg.BaseMesh ?? "";
                LoadedPackage.HomePosition = pkg.HomePosition;
                LoadedPackage.FlangeOffset = pkg.FlangeOffset ?? new double[3];

                // Update viewport
                await _viewportService.InitializeFromPackageAsync(LoadedPackage);

                // Save last active package to config for auto-restore on next startup
                if (_configService != null)
                {
                    _configService.Config.LastActivePackageId = pkg.Id;
                    _configService.Save();
                    Log.Information("Saved last active package: {Id}", pkg.Id);
                }

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

    /// <summary>
    /// Open Import Robot Dialog to import from URDF/XACRO
    /// </summary>
    [RelayCommand]
    private async Task OpenImportDialogAsync()
    {
        try
        {
            var viewModel = new ImportRobotViewModel(_urdfImportService, _packageGenerator);
            var dialog = new ImportRobotDialog(viewModel)
            {
                Owner = Application.Current.MainWindow
            };

            var result = dialog.ShowDialog();

            if (result == true)
            {
                // Import successful - refresh package list
                StatusMessage = "Import successful! Refreshing packages...";
                await RefreshPackagesAsync();
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Error opening import dialog");
            StatusMessage = $"Error: {ex.Message}";
        }
    }

    /// <summary>
    /// Delete the selected robot package from both Core and UI locations
    /// </summary>
    [RelayCommand]
    private async Task DeleteSelectedPackageAsync()
    {
        if (SelectedPackage == null)
        {
            StatusMessage = "No package selected";
            return;
        }

        var packageId = SelectedPackage.Id;
        var packageName = SelectedPackage.Name;

        // Confirm deletion
        var result = MessageBox.Show(
            $"Are you sure you want to delete '{packageName}' ({packageId})?\n\nThis will remove the package from both Core and UI locations.",
            "Delete Robot Package",
            MessageBoxButton.YesNo,
            MessageBoxImage.Warning);

        if (result != MessageBoxResult.Yes)
            return;

        try
        {
            IsLoading = true;
            StatusMessage = $"Deleting {packageName}...";

            var baseDir = AppDomain.CurrentDomain.BaseDirectory;
            int deletedLocations = 0;

            // Delete from UI location
            var uiPath = System.IO.Path.Combine(baseDir, "config", "robots", packageId);
            if (System.IO.Directory.Exists(uiPath))
            {
                await Task.Run(() => System.IO.Directory.Delete(uiPath, recursive: true));
                Log.Information("Deleted package from UI: {Path}", uiPath);
                deletedLocations++;
            }

            // Delete from Core location
            var corePath = System.IO.Path.GetFullPath(System.IO.Path.Combine(
                baseDir, "..", "..", "..", "..", "..", "..", "src", "core", "build", "config", "robots", packageId));
            if (System.IO.Directory.Exists(corePath))
            {
                await Task.Run(() => System.IO.Directory.Delete(corePath, recursive: true));
                Log.Information("Deleted package from Core: {Path}", corePath);
                deletedLocations++;
            }

            // Clear selection if deleted package was selected
            if (LoadedPackage?.Id == packageId)
            {
                LoadedPackage = null;
            }

            // Tell Core to reload packages (invalidate cache)
            await _ipcClient.ReloadPackagesAsync();

            // Refresh package list
            await RefreshPackagesAsync();

            StatusMessage = $"Deleted '{packageName}' from {deletedLocations} location(s)";
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to delete package {Id}", packageId);
            StatusMessage = $"Error deleting package: {ex.Message}";
        }
        finally
        {
            IsLoading = false;
        }
    }
}
