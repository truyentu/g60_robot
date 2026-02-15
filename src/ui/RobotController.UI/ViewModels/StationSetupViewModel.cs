using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Microsoft.Win32;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using RobotController.UI.Models;
using RobotController.UI.Services;
using Serilog;
using System.Collections.ObjectModel;
using System.IO;
using System.Text.Json;
using System.Windows.Media;

namespace RobotController.UI.ViewModels;

/// <summary>
/// View model for a single scene object item in the list
/// </summary>
public partial class SceneObjectItemViewModel : ObservableObject
{
    [ObservableProperty] private string _id = string.Empty;
    [ObservableProperty] private string _name = string.Empty;
    [ObservableProperty] private string _meshPath = string.Empty;
    [ObservableProperty] private double _x;
    [ObservableProperty] private double _y;
    [ObservableProperty] private double _z;
    [ObservableProperty] private double _rx;
    [ObservableProperty] private double _ry;
    [ObservableProperty] private double _rz;
    [ObservableProperty] private double _scale = 1.0;
    [ObservableProperty] private string? _parentFrameId;
    [ObservableProperty] private string _parentFrameName = "World";
    [ObservableProperty] private Color _color = Color.FromRgb(160, 160, 160);
    [ObservableProperty] private ObjectCategory _category = ObjectCategory.Other;

    public static SceneObjectItemViewModel FromSceneObject(SceneObject obj)
    {
        return new SceneObjectItemViewModel
        {
            Id = obj.Id,
            Name = obj.Name,
            MeshPath = obj.MeshPath,
            X = obj.X, Y = obj.Y, Z = obj.Z,
            Rx = obj.Rx, Ry = obj.Ry, Rz = obj.Rz,
            Scale = obj.Scale,
            ParentFrameId = obj.ParentFrameId,
            ParentFrameName = string.IsNullOrEmpty(obj.ParentFrameId) ? "World" : obj.ParentFrameId,
            Color = obj.Color,
            Category = obj.Category
        };
    }
}

/// <summary>
/// ViewModel for Station Setup page - manages environment objects and frame visualization
/// </summary>
public partial class StationSetupViewModel : ObservableObject
{
    private readonly IViewportService _viewportService;
    private readonly IIpcClientService _ipcClient;
    private WorkspaceService? _workspaceService;

    public ObservableCollection<SceneObjectItemViewModel> Objects { get; } = new();
    public ObservableCollection<string> AvailableFrames { get; } = new() { "World" };

    // Available categories for ComboBox binding
    public ObjectCategory[] AvailableCategories { get; } = Enum.GetValues<ObjectCategory>();

    // Preset colors for color picker
    public static readonly (string Name, Color Color)[] PresetColors =
    {
        ("Gray", Color.FromRgb(160, 160, 160)),
        ("Steel Blue", Color.FromRgb(70, 130, 180)),
        ("Orange", Color.FromRgb(255, 140, 0)),
        ("Green", Color.FromRgb(34, 139, 34)),
        ("Red", Color.FromRgb(220, 20, 60)),
        ("Yellow", Color.FromRgb(255, 215, 0)),
    };

    [ObservableProperty] private SceneObjectItemViewModel? _selectedObject;
    [ObservableProperty] private bool _isEditing;
    [ObservableProperty] private string? _errorMessage;
    [ObservableProperty] private bool _hasError;

    // Edit form
    [ObservableProperty] private string _editName = string.Empty;
    [ObservableProperty] private double _editX;
    [ObservableProperty] private double _editY;
    [ObservableProperty] private double _editZ;
    [ObservableProperty] private double _editRx;
    [ObservableProperty] private double _editRy;
    [ObservableProperty] private double _editRz;
    [ObservableProperty] private double _editScale = 1.0;
    [ObservableProperty] private string _editParentFrame = "World";
    [ObservableProperty] private ObjectCategory _editCategory = ObjectCategory.Other;
    [ObservableProperty] private Color _editColor = Color.FromRgb(160, 160, 160);

    // Robot base height (Z of robot J1 base, from pedestal or default 0)
    [ObservableProperty] private double _robotBaseHeight;
    [ObservableProperty] private string _robotBaseHeightInfo = "Robot Base Z: 0 mm (floor)";

    // Selected object height info
    [ObservableProperty] private string _selectedObjectHeightInfo = "";
    [ObservableProperty] private bool _hasHeightInfo;

    // Snap-to-grid
    [ObservableProperty] private bool _isSnapToGridEnabled;
    [ObservableProperty] private double _gridSize = 50.0;

    // Available grid sizes for ComboBox
    public double[] AvailableGridSizes { get; } = { 10, 25, 50, 100, 200 };

    // Rotation snap
    [ObservableProperty] private bool _isRotationSnapEnabled;
    [ObservableProperty] private double _rotationSnapIncrement = 15.0;
    public double[] AvailableRotationSnaps { get; } = { 5, 15, 45, 90 };

    // Event for gizmo management in MainWindow
    public event EventHandler<string?>? SelectedObjectChanged;

    public StationSetupViewModel(IViewportService viewportService, IIpcClientService ipcClient)
    {
        _viewportService = viewportService;
        _ipcClient = ipcClient;

        _ipcClient.BaseChanged += OnBaseChanged;
        _ipcClient.ConnectionStateChanged += OnConnectionStateChanged;
    }

    /// <summary>Set workspace for station file path resolution</summary>
    public void SetWorkspaceService(WorkspaceService workspace)
    {
        _workspaceService = workspace;
        // Auto-load station when workspace is set (in case IPC connected before workspace was set)
        _ = LoadStationAsync();
    }

    /// <summary>Resolve station file path — workspace/Station/ if available with real data, fallback to config/station/</summary>
    private string GetStationFilePath()
    {
        if (_workspaceService != null)
        {
            var wsStation = Path.Combine(_workspaceService.StationDir, "station.json");
            if (File.Exists(wsStation))
            {
                return wsStation;
            }
        }
        // Fallback to bin config
        var binStation = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "config", "station", "station.json");
        return File.Exists(binStation) ? binStation :
               Path.Combine(_workspaceService?.StationDir ??
                   Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "config", "station"), "station.json");
    }

    // ========================================================================
    // Commands
    // ========================================================================

    [RelayCommand]
    private void ImportObject()
    {
        try
        {
            HasError = false;

            var dialog = new OpenFileDialog
            {
                Title = "Import 3D Object",
                Filter = "STL Files (*.stl)|*.stl|All Files (*.*)|*.*",
                Multiselect = false
            };

            if (dialog.ShowDialog() != true) return;

            var fileName = Path.GetFileNameWithoutExtension(dialog.FileName);
            var obj = _viewportService.AddSceneObject(fileName, dialog.FileName, 1.0);

            if (obj != null)
            {
                var vm = SceneObjectItemViewModel.FromSceneObject(obj);
                Objects.Add(vm);
                SelectedObject = vm;
                StartEditObject();

                Log.Information("[StationSetup] Imported object: {Name} from {Path}", obj.Name, dialog.FileName);
            }
            else
            {
                ErrorMessage = "Failed to load STL file";
                HasError = true;
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "[StationSetup] Error importing object");
            ErrorMessage = ex.Message;
            HasError = true;
        }
    }

    [RelayCommand]
    private void DeleteObject()
    {
        if (SelectedObject == null) return;

        var wasCategory = SelectedObject.Category;
        _viewportService.RemoveSceneObject(SelectedObject.Id);
        Objects.Remove(SelectedObject);
        SelectedObject = null;
        IsEditing = false;
        HasHeightInfo = false;
        SelectedObjectChanged?.Invoke(this, null);

        // Recalculate if a pedestal was removed
        if (wasCategory == ObjectCategory.Pedestal)
            ComputeRobotBaseHeight();

        _viewportService.HideHeightIndicator();
    }

    [RelayCommand]
    private void StartEditObject()
    {
        if (SelectedObject == null) return;

        IsEditing = true;
        EditName = SelectedObject.Name;
        EditX = SelectedObject.X;
        EditY = SelectedObject.Y;
        EditZ = SelectedObject.Z;
        EditRx = SelectedObject.Rx;
        EditRy = SelectedObject.Ry;
        EditRz = SelectedObject.Rz;
        EditScale = SelectedObject.Scale;
        EditParentFrame = SelectedObject.ParentFrameName;
        EditCategory = SelectedObject.Category;
        EditColor = SelectedObject.Color;

        UpdateHeightInfo();

        // Show height indicator in viewport
        _viewportService.ShowHeightIndicator(SelectedObject.Id, RobotBaseHeight);
    }

    [RelayCommand]
    private void ApplyTransform()
    {
        if (SelectedObject == null) return;

        // Update ViewModel
        SelectedObject.Name = EditName;
        SelectedObject.X = EditX;
        SelectedObject.Y = EditY;
        SelectedObject.Z = EditZ;
        SelectedObject.Rx = EditRx;
        SelectedObject.Ry = EditRy;
        SelectedObject.Rz = EditRz;
        SelectedObject.Scale = EditScale;
        SelectedObject.Category = EditCategory;
        SelectedObject.Color = EditColor;

        // Pedestal auto-snap to XY origin (Z can be adjusted)
        if (EditCategory == ObjectCategory.Pedestal)
        {
            SelectedObject.X = 0; SelectedObject.Y = 0;
            EditX = 0; EditY = 0;
        }

        // Apply parent frame
        string? frameId = EditParentFrame == "World" ? null : EditParentFrame;
        SelectedObject.ParentFrameId = frameId;
        SelectedObject.ParentFrameName = EditParentFrame;

        // Update viewport
        _viewportService.UpdateSceneObjectTransform(
            SelectedObject.Id,
            new[] { SelectedObject.X, SelectedObject.Y, SelectedObject.Z },
            new[] { EditRx, EditRy, EditRz },
            EditScale);

        _viewportService.SetSceneObjectParent(SelectedObject.Id, frameId);
        _viewportService.SetSceneObjectColor(SelectedObject.Id, EditColor);

        // Update SceneObject category
        var sceneObj = _viewportService.SceneObjects.Find(o => o.Id == SelectedObject.Id);
        if (sceneObj != null) sceneObj.Category = EditCategory;

        // Recalculate robot base height (in case pedestal was changed)
        ComputeRobotBaseHeight();

        IsEditing = false;
        HasError = false;

        _viewportService.HideHeightIndicator();
    }

    [RelayCommand]
    private void CancelEdit()
    {
        // Restore original transform if we were previewing
        if (SelectedObject != null)
        {
            _viewportService.UpdateSceneObjectTransform(
                SelectedObject.Id,
                new[] { SelectedObject.X, SelectedObject.Y, SelectedObject.Z },
                new[] { SelectedObject.Rx, SelectedObject.Ry, SelectedObject.Rz },
                SelectedObject.Scale);
        }
        IsEditing = false;
        _viewportService.HideHeightIndicator();
    }

    [RelayCommand]
    private void SetColor(Color color)
    {
        EditColor = color;
    }

    // ========================================================================
    // Quick Flip & Preset Orientation Commands
    // ========================================================================

    [RelayCommand]
    private void FlipX() => ApplyQuickRotation(90, 0, 0);

    [RelayCommand]
    private void FlipY() => ApplyQuickRotation(0, 90, 0);

    [RelayCommand]
    private void FlipZ() => ApplyQuickRotation(0, 0, 90);

    [RelayCommand]
    private void ApplyPresetOrientation(string preset)
    {
        switch (preset)
        {
            case "Flat":       EditRx = 0;   EditRy = 0;  EditRz = 0;   break;
            case "StandX":     EditRx = 90;  EditRy = 0;  EditRz = 0;   break;
            case "StandY":     EditRx = 0;   EditRy = 90; EditRz = 0;   break;
            case "Tilt45":     EditRx = 45;  EditRy = 0;  EditRz = 0;   break;
            case "UpsideDown": EditRx = 180; EditRy = 0;  EditRz = 0;   break;
            case "Rotate90":   EditRx = 0;   EditRy = 0;  EditRz = 90;  break;
        }
        PreviewCurrentRotation();
    }

    private void ApplyQuickRotation(double dRx, double dRy, double dRz)
    {
        EditRx = NormalizeAngle(SnapAngle(EditRx + dRx));
        EditRy = NormalizeAngle(SnapAngle(EditRy + dRy));
        EditRz = NormalizeAngle(SnapAngle(EditRz + dRz));
        PreviewCurrentRotation();
    }

    private void PreviewCurrentRotation()
    {
        if (SelectedObject == null) return;

        _viewportService.UpdateSceneObjectTransform(
            SelectedObject.Id,
            new[] { EditX, EditY, EditZ },
            new[] { EditRx, EditRy, EditRz },
            EditScale);

        UpdateHeightInfo();
        _viewportService.ShowHeightIndicator(SelectedObject.Id, RobotBaseHeight);
    }

    private static double NormalizeAngle(double angle)
    {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    private double SnapAngle(double angle)
    {
        if (!IsRotationSnapEnabled || RotationSnapIncrement <= 0) return angle;
        return Math.Round(angle / RotationSnapIncrement) * RotationSnapIncrement;
    }

    /// <summary>
    /// Drop the selected object so its bottom face sits on the ground plane (Z=0).
    /// Uses rotation-aware bounding box to calculate the offset correctly after flips.
    /// </summary>
    [RelayCommand]
    private void DropToGround()
    {
        if (SelectedObject == null) return;

        double rx = EditRx, ry = EditRy, rz = EditRz;
        double scale = EditScale > 0 ? EditScale : 1.0;

        var bounds = _viewportService.GetTransformedSceneObjectBounds(SelectedObject.Id, rx, ry, rz, scale);
        if (bounds == null)
        {
            Log.Warning("[StationSetup] Cannot get transformed bounds for object {Id}", SelectedObject.Id);
            return;
        }

        // bounds.Z is the min Z of the rotated+scaled mesh (centered at origin).
        // We want: obj.Z + bounds.Z = 0 (on ground)
        // So: obj.Z = -bounds.Z
        double newZ = -bounds.Value.Z;

        EditZ = newZ;
        PreviewCurrentRotation();

        Log.Information("[StationSetup] Dropped {Name} to ground: Z = {Z:F1} mm", SelectedObject.Name, newZ);
    }

    /// <summary>
    /// Drop the selected object so its bottom face sits on top of the welding table.
    /// Finds the first WeldingTable in the scene and uses its top surface Z (max Z of transformed bounds).
    /// If no WeldingTable category is found, falls back to any other non-selected object as the "table".
    /// </summary>
    [RelayCommand]
    private void DropToTable()
    {
        if (SelectedObject == null) return;

        // Find welding table(s) in the scene — prefer explicit WeldingTable category
        var tables = _viewportService.SceneObjects
            .Where(o => o.Category == ObjectCategory.WeldingTable && o.Id != SelectedObject.Id)
            .ToList();

        // Fallback: if no WeldingTable, use any other object (pedestal, fixture, etc.) as the surface
        if (tables.Count == 0)
        {
            tables = _viewportService.SceneObjects
                .Where(o => o.Id != SelectedObject.Id)
                .ToList();
        }

        if (tables.Count == 0)
        {
            Log.Warning("[StationSetup] No other objects in scene to use as table surface.");
            return;
        }

        // Find the highest top-Z among all candidate table objects
        double tableTopZ = double.MinValue;
        string tableName = "";
        foreach (var table in tables)
        {
            double tScale = table.Scale > 0 ? table.Scale : 1.0;
            var tableBounds = _viewportService.GetTransformedSceneObjectBounds(
                table.Id, table.Rx, table.Ry, table.Rz, tScale);
            if (tableBounds == null) continue;

            double topZ = table.Z + tableBounds.Value.Z + tableBounds.Value.SizeZ;
            if (topZ > tableTopZ)
            {
                tableTopZ = topZ;
                tableName = table.Name;
            }
        }

        if (tableTopZ == double.MinValue)
        {
            Log.Warning("[StationSetup] Cannot compute table surface height.");
            return;
        }

        // Get the bottom of the workpiece (rotation-aware, using current edit rotation)
        double rx = EditRx, ry = EditRy, rz = EditRz;
        double scale = EditScale > 0 ? EditScale : 1.0;

        var objBounds = _viewportService.GetTransformedSceneObjectBounds(SelectedObject.Id, rx, ry, rz, scale);
        if (objBounds == null)
        {
            Log.Warning("[StationSetup] Cannot get transformed bounds for object {Id}", SelectedObject.Id);
            return;
        }

        // We want: obj.Z + objBounds.Z = tableTopZ  (bottom of workpiece touches top of table)
        double newZ = tableTopZ - objBounds.Value.Z;

        EditZ = newZ;
        PreviewCurrentRotation();

        Log.Information("[StationSetup] Dropped {Name} to table ({TableName}): Z = {Z:F1} mm (table top = {TableTop:F1})",
            SelectedObject.Name, tableName, newZ, tableTopZ);
    }

    /// <summary>
    /// Align the selected object's top surface with the robot base height.
    /// Uses rotation-aware bounding box for correct calculation after flips.
    /// </summary>
    [RelayCommand]
    private void AlignToRobotBase()
    {
        if (SelectedObject == null) return;

        double rx = EditRx, ry = EditRy, rz = EditRz;
        double scale = EditScale > 0 ? EditScale : 1.0;

        var bounds = _viewportService.GetTransformedSceneObjectBounds(SelectedObject.Id, rx, ry, rz, scale);
        if (bounds == null)
        {
            Log.Warning("[StationSetup] Cannot get transformed bounds for object {Id}", SelectedObject.Id);
            return;
        }

        // bounds after rotation: top = bounds.Z + bounds.SizeZ
        // We want: obj.Z + (bounds.Z + bounds.SizeZ) = RobotBaseHeight
        // So: obj.Z = RobotBaseHeight - bounds.Z - bounds.SizeZ
        double newZ = RobotBaseHeight - bounds.Value.Z - bounds.Value.SizeZ;

        EditZ = newZ;
        PreviewCurrentRotation();

        Log.Information("[StationSetup] Aligned {Name} top to robot base: Z = {Z:F1} mm (base = {Base:F1})",
            SelectedObject.Name, newZ, RobotBaseHeight);
    }

    /// <summary>
    /// Compute and update the height relationship info between selected object and robot base.
    /// </summary>
    private void UpdateHeightInfo()
    {
        if (SelectedObject == null)
        {
            HasHeightInfo = false;
            SelectedObjectHeightInfo = "";
            return;
        }

        double rx = EditRx, ry = EditRy, rz = EditRz;
        double scale = EditScale > 0 ? EditScale : 1.0;

        var bounds = _viewportService.GetTransformedSceneObjectBounds(SelectedObject.Id, rx, ry, rz, scale);
        if (bounds == null)
        {
            HasHeightInfo = false;
            return;
        }

        double currentZ = EditZ;
        double meshBottomZ = currentZ + bounds.Value.Z;
        double meshTopZ = currentZ + bounds.Value.Z + bounds.Value.SizeZ;
        double meshHeight = bounds.Value.SizeZ;
        double diffFromBase = meshTopZ - RobotBaseHeight;

        string relation;
        if (Math.Abs(diffFromBase) < 1.0)
            relation = "= Robot Base";
        else if (diffFromBase > 0)
            relation = $"+{diffFromBase:F0} mm above Robot Base";
        else
            relation = $"{diffFromBase:F0} mm below Robot Base";

        SelectedObjectHeightInfo = $"Bottom: {meshBottomZ:F0} mm | Top: {meshTopZ:F0} mm | Height: {meshHeight:F0} mm\n{relation}";
        HasHeightInfo = true;
    }

    /// <summary>
    /// Compute robot base height from pedestal objects in the scene.
    /// If a Pedestal object exists, robot base Z = pedestal top surface Z.
    /// </summary>
    private void ComputeRobotBaseHeight()
    {
        // Find pedestal objects
        var pedestals = _viewportService.SceneObjects.Where(o => o.Category == ObjectCategory.Pedestal).ToList();

        if (pedestals.Count == 0)
        {
            RobotBaseHeight = 0;
            RobotBaseHeightInfo = "Robot Base Z: 0 mm (floor mount)";
            return;
        }

        // Use the first pedestal's top surface
        var pedestal = pedestals[0];
        double pScale = pedestal.Scale > 0 ? pedestal.Scale : 1.0;
        var bounds = _viewportService.GetTransformedSceneObjectBounds(
            pedestal.Id, pedestal.Rx, pedestal.Ry, pedestal.Rz, pScale);
        if (bounds != null)
        {
            double pedestalTopZ = pedestal.Z + bounds.Value.Z + bounds.Value.SizeZ;
            RobotBaseHeight = pedestalTopZ;
            RobotBaseHeightInfo = $"Robot Base Z: {pedestalTopZ:F0} mm (on {pedestal.Name})";
        }
        else
        {
            RobotBaseHeight = 0;
            RobotBaseHeightInfo = $"Robot Base Z: 0 mm ({pedestal.Name} - no bounds)";
        }
    }

    /// <summary>
    /// Snap a value to the nearest grid point.
    /// </summary>
    public double SnapValue(double value)
    {
        if (!IsSnapToGridEnabled || GridSize <= 0) return value;
        return Math.Round(value / GridSize) * GridSize;
    }

    // ========================================================================
    // Public Methods (called from MainWindow for gizmo sync)
    // ========================================================================

    public void SelectObjectById(string id)
    {
        var obj = Objects.FirstOrDefault(o => o.Id == id);
        if (obj == null) return;

        SelectedObject = obj;
        StartEditObject();
        SelectedObjectChanged?.Invoke(this, id);
    }

    public void UpdateObjectTransformFromGizmo(string id, double x, double y, double z,
        double rx, double ry, double rz)
    {
        var obj = Objects.FirstOrDefault(o => o.Id == id);
        if (obj == null) return;

        obj.X = x; obj.Y = y; obj.Z = z;
        obj.Rx = rx; obj.Ry = ry; obj.Rz = rz;

        // Sync to edit fields if this object is being edited
        if (IsEditing && SelectedObject?.Id == id)
        {
            EditX = x; EditY = y; EditZ = z;
            EditRx = rx; EditRy = ry; EditRz = rz;
        }

        // Update SceneObject data (without re-transforming visual)
        _viewportService.UpdateSceneObjectData(id, x, y, z, rx, ry, rz);
    }

    // ========================================================================
    // Frames
    // ========================================================================

    [RelayCommand]
    private async Task RefreshFramesAsync()
    {
        if (!_ipcClient.IsConnected) return;

        try
        {
            var response = await _ipcClient.GetBaseListAsync();
            if (response == null) return;

            System.Windows.Application.Current?.Dispatcher.Invoke(() =>
            {
                AvailableFrames.Clear();
                AvailableFrames.Add("World");

                foreach (var baseFrame in response.Bases)
                {
                    // Skip "world"/"World" since we always add it above
                    if (string.Equals(baseFrame.Id, "World", StringComparison.OrdinalIgnoreCase))
                        continue;
                    AvailableFrames.Add(baseFrame.Id);
                }

                // Update base frame visuals in viewport
                _viewportService.UpdateBaseFrameVisuals(response.Bases, response.ActiveBaseId);
            });
        }
        catch (Exception ex)
        {
            Log.Error(ex, "[StationSetup] Error refreshing frames");
        }
    }

    [RelayCommand]
    private async Task SaveStationAsync()
    {
        try
        {
            HasError = false;

            var stationFile = GetStationFilePath();
            Directory.CreateDirectory(Path.GetDirectoryName(stationFile)!);

            var stationData = new StationData
            {
                Objects = _viewportService.SceneObjects.Select(o => new StationObjectData
                {
                    Id = o.Id,
                    Name = o.Name,
                    MeshPath = o.MeshPath,
                    X = o.X, Y = o.Y, Z = o.Z,
                    Rx = o.Rx, Ry = o.Ry, Rz = o.Rz,
                    Scale = o.Scale,
                    ParentFrameId = o.ParentFrameId,
                    ColorR = o.Color.R, ColorG = o.Color.G, ColorB = o.Color.B,
                    Category = o.Category.ToString()
                }).ToList()
            };

            var json = JsonSerializer.Serialize(stationData, new JsonSerializerOptions { WriteIndented = true });
            await File.WriteAllTextAsync(stationFile, json);

            Log.Information("[StationSetup] Station saved to {Path} ({Count} objects)", stationFile, stationData.Objects.Count);
        }
        catch (Exception ex)
        {
            Log.Error(ex, "[StationSetup] Error saving station");
            ErrorMessage = ex.Message;
            HasError = true;
        }
    }

    [RelayCommand]
    private async Task LoadStationAsync()
    {
        try
        {
            HasError = false;

            var stationFile = GetStationFilePath();
            Log.Information("[StationSetup] Loading station from: {Path}", stationFile);
            if (!File.Exists(stationFile))
            {
                Log.Information("[StationSetup] No station file found at {Path}", stationFile);
                return;
            }

            var json = await File.ReadAllTextAsync(stationFile);
            var stationData = JsonSerializer.Deserialize<StationData>(json);
            Log.Information("[StationSetup] Deserialized: Objects={Count}", stationData?.Objects?.Count ?? -1);
            if (stationData?.Objects == null) return;

            // Clear existing
            foreach (var obj in _viewportService.SceneObjects.ToList())
            {
                _viewportService.RemoveSceneObject(obj.Id);
            }
            Objects.Clear();

            // Load each object
            foreach (var objData in stationData.Objects)
            {
                if (!File.Exists(objData.MeshPath))
                {
                    Log.Warning("[StationSetup] Mesh file not found, skipping: {Path}", objData.MeshPath);
                    continue;
                }

                var obj = _viewportService.AddSceneObject(objData.Name, objData.MeshPath, objData.Scale);
                if (obj == null) continue;

                // Apply saved transforms
                obj.X = objData.X; obj.Y = objData.Y; obj.Z = objData.Z;
                obj.Rx = objData.Rx; obj.Ry = objData.Ry; obj.Rz = objData.Rz;
                obj.Scale = objData.Scale;
                obj.ParentFrameId = objData.ParentFrameId;
                obj.Color = Color.FromRgb(objData.ColorR, objData.ColorG, objData.ColorB);

                if (Enum.TryParse<ObjectCategory>(objData.Category, out var cat))
                    obj.Category = cat;

                _viewportService.UpdateSceneObjectTransform(obj.Id,
                    new[] { obj.X, obj.Y, obj.Z },
                    new[] { obj.Rx, obj.Ry, obj.Rz },
                    obj.Scale);

                if (!string.IsNullOrEmpty(obj.ParentFrameId))
                    _viewportService.SetSceneObjectParent(obj.Id, obj.ParentFrameId);

                _viewportService.SetSceneObjectColor(obj.Id, obj.Color);

                Objects.Add(SceneObjectItemViewModel.FromSceneObject(obj));
            }

            Log.Information("[StationSetup] Station loaded: {Count} objects from {Path}", Objects.Count, stationFile);

            // Recalculate robot base height from loaded pedestals
            ComputeRobotBaseHeight();
        }
        catch (Exception ex)
        {
            Log.Error(ex, "[StationSetup] Error loading station");
            ErrorMessage = ex.Message;
            HasError = true;
        }
    }

    // ========================================================================
    // Event Handlers
    // ========================================================================

    private void OnBaseChanged(object? sender, BaseChangedEvent e)
    {
        _ = RefreshFramesAsync();
    }

    private async void OnConnectionStateChanged(object? sender, bool isConnected)
    {
        if (isConnected)
        {
            await RefreshFramesAsync();
            await LoadStationAsync();
        }
    }
}

// ========================================================================
// Station persistence data
// ========================================================================

internal class StationData
{
    public List<StationObjectData> Objects { get; set; } = new();
}

internal class StationObjectData
{
    public string Id { get; set; } = "";
    public string Name { get; set; } = "";
    public string MeshPath { get; set; } = "";
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double Rx { get; set; }
    public double Ry { get; set; }
    public double Rz { get; set; }
    public double Scale { get; set; } = 1.0;
    public string? ParentFrameId { get; set; }
    public byte ColorR { get; set; } = 160;
    public byte ColorG { get; set; } = 160;
    public byte ColorB { get; set; } = 160;
    public string Category { get; set; } = "Other";
}
