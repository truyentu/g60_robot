using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using RobotController.Common.Messages;
using RobotController.UI.Models;
using Serilog;

namespace RobotController.UI.Services;

/// <summary>
/// Interface for 3D viewport service
/// </summary>
public interface IViewportService
{
    /// <summary>Robot 3D model</summary>
    RobotModel3D? Robot { get; }

    /// <summary>Is robot model loaded</summary>
    bool IsLoaded { get; }

    /// <summary>Initialize viewport with robot model</summary>
    Task<bool> InitializeAsync(RobotConfigData config);

    /// <summary>Initialize viewport from robot package</summary>
    Task<bool> InitializeFromPackageAsync(RobotPackageData package);

    /// <summary>Update joint angles from status</summary>
    void UpdateJointAngles(double[] anglesDegrees);

    /// <summary>Update joint angles and TCP pose from Core FK</summary>
    void UpdateJointAngles(double[] anglesDegrees, double[]? tcpPose);

    /// <summary>Get the 3D model group for binding</summary>
    Model3DGroup? GetModelGroup();

    /// <summary>Create default lighting</summary>
    Model3DGroup CreateLighting();

    /// <summary>Create grid</summary>
    Model3DGroup CreateGrid(double size = 2000, double spacing = 100);

    /// <summary>Create coordinate axes</summary>
    Model3DGroup CreateAxes(double length = 200);

    /// <summary>Create TCP marker</summary>
    Model3DGroup CreateTcpMarker();

    /// <summary>Set tool mesh for 3D visualization</summary>
    bool SetToolMesh(string stlPath, double[] meshOffset, double meshScale = 1.0);

    /// <summary>Set tool TCP offset from flange (for TCP marker positioning at wire tip)</summary>
    void SetToolTcpOffset(double[] tcpOffset);

    /// <summary>Remove tool mesh from viewport</summary>
    void ClearToolMesh();

    /// <summary>Enable/disable TCP pick mode</summary>
    bool IsPickingTcp { get; }
    void EnableTcpPicking();
    void DisableTcpPicking();

    /// <summary>Process a 3D hit point for TCP picking. Returns TCP offset [x,y,z] relative to flange, or null.</summary>
    double[]? ProcessPickedPoint(Point3D worldPoint);

    /// <summary>Event when a TCP point is picked</summary>
    event EventHandler<TcpPickedEventArgs>? TcpPointPicked;

    /// <summary>Event when TCP pick mode changes (for navigation)</summary>
    event EventHandler<bool>? TcpPickModeChanged;

    /// <summary>Event when robot model updates</summary>
    event EventHandler? ModelUpdated;

    // ========================================================================
    // Scene Object Management
    // ========================================================================

    /// <summary>All scene objects in the viewport</summary>
    List<SceneObject> SceneObjects { get; }

    /// <summary>Add a scene object from STL file</summary>
    SceneObject? AddSceneObject(string name, string stlPath, double scale = 1.0);

    /// <summary>Remove a scene object by ID</summary>
    bool RemoveSceneObject(string id);

    /// <summary>Update a scene object's transform (updates visual + data)</summary>
    void UpdateSceneObjectTransform(string id, double[] position, double[] rotation, double scale);

    /// <summary>Update scene object data only (no visual re-transform, used by gizmo sync)</summary>
    void UpdateSceneObjectData(string id, double x, double y, double z, double rx, double ry, double rz);

    /// <summary>Set parent frame for a scene object</summary>
    void SetSceneObjectParent(string id, string? frameId);

    /// <summary>Update scene object color</summary>
    void SetSceneObjectColor(string id, Color color);

    /// <summary>Find scene object ID by its Visual3D reference (for hit testing)</summary>
    string? FindSceneObjectByVisual(Visual3D visual);

    /// <summary>Get the ModelVisual3D wrapper for a scene object (for gizmo binding)</summary>
    ModelVisual3D? GetSceneObjectVisual(string id);

    /// <summary>Highlight a selected scene object</summary>
    void HighlightSceneObject(string id);

    /// <summary>Remove highlight from a scene object</summary>
    void UnhighlightSceneObject(string id);

    /// <summary>Get the axis-aligned bounding box of a scene object's mesh (in local/unscaled coords)</summary>
    Rect3D? GetSceneObjectBounds(string id);

    /// <summary>Get the AABB after applying scale + rotation (but NOT translation). Used for rotation-aware placement.</summary>
    Rect3D? GetTransformedSceneObjectBounds(string id, double rx, double ry, double rz, double scale);

    /// <summary>Show a vertical height indicator line from ground to object, plus robot base reference</summary>
    void ShowHeightIndicator(string objectId, double robotBaseHeight);

    /// <summary>Hide the height indicator</summary>
    void HideHeightIndicator();

    /// <summary>Event when scene objects change</summary>
    event EventHandler? SceneObjectsChanged;

    // ========================================================================
    // Base Frame Visualization
    // ========================================================================

    /// <summary>Update base frame coordinate axes in the viewport</summary>
    void UpdateBaseFrameVisuals(List<BaseFrameData> frames, string activeFrameId);

    /// <summary>Get the scene objects model group (for MainWindow binding)</summary>
    Model3DGroup? GetSceneGroup();

    /// <summary>Get the base frame axes model group (for MainWindow binding)</summary>
    Model3DGroup? GetBaseFrameGroup();

    /// <summary>Get the scene objects container visual (for MainWindow to add to viewport)</summary>
    ModelVisual3D SceneObjectsContainer { get; }
}

/// <summary>
/// Event args for TCP point picking
/// </summary>
public class TcpPickedEventArgs : EventArgs
{
    public double X { get; init; }
    public double Y { get; init; }
    public double Z { get; init; }
    public Point3D WorldPoint { get; init; }
}

/// <summary>
/// 3D Viewport service implementation
/// </summary>
public class ViewportService : IViewportService
{
    private RobotModel3D? _robot;
    private readonly string _modelsPath;
    private Model3DGroup? _tcpMarker;
    private bool _isPickingTcp;

    // Scene objects
    private readonly List<SceneObject> _sceneObjects = new();
    private Model3DGroup? _sceneGroup;
    private readonly ModelVisual3D _sceneObjectsContainer = new();
    private readonly Dictionary<string, ModelVisual3D> _sceneVisuals = new();

    // Selection highlight
    private readonly Dictionary<string, Material> _originalMaterials = new();

    // Base frame visuals
    private Model3DGroup? _baseFrameGroup;
    private readonly Dictionary<string, BaseFrameData> _baseFrames = new();

    // Height indicator visual
    private ModelVisual3D? _heightIndicatorVisual;

    public RobotModel3D? Robot => _robot;
    public bool IsLoaded => _robot != null;
    public bool IsPickingTcp => _isPickingTcp;
    public List<SceneObject> SceneObjects => _sceneObjects;
    public ModelVisual3D SceneObjectsContainer => _sceneObjectsContainer;

    public event EventHandler? ModelUpdated;
    public event EventHandler<TcpPickedEventArgs>? TcpPointPicked;
    public event EventHandler<bool>? TcpPickModeChanged;
    public event EventHandler? SceneObjectsChanged;

    public ViewportService()
    {
        // Path to models relative to executable
        _modelsPath = Path.Combine(
            AppDomain.CurrentDomain.BaseDirectory,
            "..", "..", "..", "..", "..", "resources", "models"
        );
    }

    public async Task<bool> InitializeAsync(RobotConfigData config)
    {
        try
        {
            Log.Information("Initializing viewport with robot: {Name}", config.Name);

            // WPF 3D objects must be created on UI thread, but we can use Task to not block
            await System.Windows.Application.Current.Dispatcher.InvokeAsync(() =>
            {
                _robot = new RobotModel3D();
                _robot.Initialize(config, _modelsPath);
            });

            if (_robot != null)
            {
                ModelUpdated?.Invoke(this, EventArgs.Empty);
                return true;
            }

            return false;
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to initialize viewport");
            return false;
        }
    }

    public async Task<bool> InitializeFromPackageAsync(RobotPackageData package)
    {
        try
        {
            Log.Information("[ViewportService] InitializeFromPackageAsync called for: {Name}", package.Name);
            Log.Information("[ViewportService] Package path: {Path}", package.PackagePath);
            Log.Information("[ViewportService] Base mesh: {Mesh}", package.BaseMesh);
            Log.Information("[ViewportService] Joints count: {Count}", package.Joints.Count);

            await System.Windows.Application.Current.Dispatcher.InvokeAsync(() =>
            {
                Log.Information("[ViewportService] Creating new RobotModel3D on UI thread");
                _robot = new RobotModel3D();

                Log.Information("[ViewportService] Calling InitializeFromPackage...");
                var result = _robot.InitializeFromPackage(package);
                Log.Information("[ViewportService] InitializeFromPackage returned: {Result}", result);
            });

            if (_robot != null)
            {
                Log.Information("[ViewportService] Robot model created successfully, raising ModelUpdated event");
                ModelUpdated?.Invoke(this, EventArgs.Empty);
                return true;
            }

            Log.Warning("[ViewportService] Robot model is null after initialization!");
            return false;
        }
        catch (Exception ex)
        {
            Log.Error(ex, "[ViewportService] EXCEPTION in InitializeFromPackageAsync");
            return false;
        }
    }

    public void UpdateJointAngles(double[] anglesDegrees)
    {
        UpdateJointAngles(anglesDegrees, null);
    }

    public void UpdateJointAngles(double[] anglesDegrees, double[]? tcpPose)
    {
        if (_robot == null) return;

        _robot.SetAllJointAngles(anglesDegrees);

        // Update TCP marker position and orientation
        if (_tcpMarker != null)
        {
            var tcpPos = _robot.TcpPosition;
            var tcpOri = _robot.TcpOrientation;

            var rotOnly = new Matrix3D(
                tcpOri.M11, tcpOri.M12, tcpOri.M13, 0,
                tcpOri.M21, tcpOri.M22, tcpOri.M23, 0,
                tcpOri.M31, tcpOri.M32, tcpOri.M33, 0,
                0, 0, 0, 1);

            var transformGroup = new Transform3DGroup();
            transformGroup.Children.Add(new MatrixTransform3D(rotOnly));
            transformGroup.Children.Add(new TranslateTransform3D(tcpPos.X, tcpPos.Y, tcpPos.Z));
            _tcpMarker.Transform = transformGroup;
        }

        ModelUpdated?.Invoke(this, EventArgs.Empty);
    }

    public bool SetToolMesh(string stlPath, double[] meshOffset, double meshScale = 1.0)
    {
        if (_robot == null) return false;

        bool result = false;
        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            result = _robot.SetToolMesh(stlPath, meshOffset, meshScale);
        });

        if (result)
        {
            ModelUpdated?.Invoke(this, EventArgs.Empty);
        }
        return result;
    }

    public void ClearToolMesh()
    {
        if (_robot == null) return;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            _robot.ClearToolMesh();
        });

        ModelUpdated?.Invoke(this, EventArgs.Empty);
    }

    public void SetToolTcpOffset(double[] tcpOffset)
    {
        if (_robot == null) return;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            _robot.SetToolTcpOffset(tcpOffset);
        });

        ModelUpdated?.Invoke(this, EventArgs.Empty);
    }

    public void EnableTcpPicking()
    {
        _isPickingTcp = true;
        Log.Information("[ViewportService] TCP picking mode ENABLED");
        TcpPickModeChanged?.Invoke(this, true);
    }

    public void DisableTcpPicking()
    {
        _isPickingTcp = false;
        Log.Information("[ViewportService] TCP picking mode DISABLED");
        TcpPickModeChanged?.Invoke(this, false);
    }

    public double[]? ProcessPickedPoint(Point3D worldPoint)
    {
        if (_robot == null) return null;

        // Get flange world transform (includes flange offset)
        var flangeWorld = _robot.GetFlangeWorldTransform();

        // Invert flange transform to get flange-local coordinates
        var flangeInverse = flangeWorld;
        if (!flangeInverse.HasInverse)
        {
            Log.Warning("[ViewportService] Flange transform is not invertible!");
            return null;
        }
        flangeInverse.Invert();

        // Transform picked world point to flange-local coordinates
        var localPoint = flangeInverse.Transform(worldPoint);

        Log.Information("[ViewportService] Picked world=({Wx:F1},{Wy:F1},{Wz:F1}) -> flange-local=({Lx:F1},{Ly:F1},{Lz:F1})",
            worldPoint.X, worldPoint.Y, worldPoint.Z,
            localPoint.X, localPoint.Y, localPoint.Z);

        var args = new TcpPickedEventArgs
        {
            X = localPoint.X,
            Y = localPoint.Y,
            Z = localPoint.Z,
            WorldPoint = worldPoint
        };

        // Disable pick mode after successful pick
        _isPickingTcp = false;

        TcpPointPicked?.Invoke(this, args);
        TcpPickModeChanged?.Invoke(this, false);

        return new[] { localPoint.X, localPoint.Y, localPoint.Z };
    }

    public Model3DGroup? GetModelGroup()
    {
        return _robot?.ModelGroup;
    }

    public Model3DGroup CreateLighting()
    {
        var group = new Model3DGroup();
        group.Children.Add(new AmbientLight(Color.FromRgb(60, 60, 60)));
        group.Children.Add(new DirectionalLight(Color.FromRgb(200, 200, 200), new Vector3D(-1, -1, -1)));
        group.Children.Add(new DirectionalLight(Color.FromRgb(100, 100, 100), new Vector3D(1, 0.5, 0.5)));
        group.Children.Add(new DirectionalLight(Color.FromRgb(80, 80, 80), new Vector3D(0, 1, 1)));
        return group;
    }

    public Model3DGroup CreateGrid(double size = 2000, double spacing = 100)
    {
        var group = new Model3DGroup();
        var builder = new MeshBuilder();

        double halfSize = size / 2;
        double lineThickness = 1;

        var gridColor = Color.FromRgb(60, 60, 60);

        for (double y = -halfSize; y <= halfSize; y += spacing)
        {
            bool isMajor = Math.Abs(y) < 0.001 || Math.Abs(y % (spacing * 5)) < 0.001;
            builder.AddPipe(
                new Point3D(-halfSize, y, 0),
                new Point3D(halfSize, y, 0),
                0, isMajor ? lineThickness * 2 : lineThickness, 8);
        }

        for (double x = -halfSize; x <= halfSize; x += spacing)
        {
            bool isMajor = Math.Abs(x) < 0.001 || Math.Abs(x % (spacing * 5)) < 0.001;
            builder.AddPipe(
                new Point3D(x, -halfSize, 0),
                new Point3D(x, halfSize, 0),
                0, isMajor ? lineThickness * 2 : lineThickness, 8);
        }

        var mesh = builder.ToMesh();
        var material = new DiffuseMaterial(new SolidColorBrush(gridColor));
        group.Children.Add(new GeometryModel3D(mesh, material));

        return group;
    }

    public Model3DGroup CreateAxes(double length = 200)
    {
        var group = new Model3DGroup();
        double radius = length / 40;
        double arrowLength = length / 5;
        double labelOffset = length + 30;

        AddAxis(group, new Point3D(0, 0, 0), new Point3D(length, 0, 0), radius, arrowLength, Colors.Red);
        AddAxisLabel(group, "X", new Point3D(labelOffset, 0, 0), Colors.Red);
        AddAxis(group, new Point3D(0, 0, 0), new Point3D(0, length, 0), radius, arrowLength, Colors.LimeGreen);
        AddAxisLabel(group, "Y", new Point3D(0, labelOffset, 0), Colors.LimeGreen);
        AddAxis(group, new Point3D(0, 0, 0), new Point3D(0, 0, length), radius, arrowLength, Colors.DodgerBlue);
        AddAxisLabel(group, "Z", new Point3D(0, 0, labelOffset), Colors.DodgerBlue);

        return group;
    }

    private void AddAxis(Model3DGroup group, Point3D start, Point3D end,
        double radius, double arrowLength, Color color)
    {
        var builder = new MeshBuilder();
        var direction = end - start;
        direction.Normalize();
        var shaftEnd = end - direction * arrowLength;
        builder.AddCylinder(start, shaftEnd, radius, 16);
        builder.AddCone(shaftEnd, direction, radius * 2.5, radius * 2.5, arrowLength, true, true, 16);

        var mesh = builder.ToMesh();
        var material = new DiffuseMaterial(new SolidColorBrush(color));
        group.Children.Add(new GeometryModel3D(mesh, material) { BackMaterial = material });
    }

    public Model3DGroup CreateTcpMarker()
    {
        var group = new Model3DGroup();

        var sphereBuilder = new MeshBuilder();
        sphereBuilder.AddSphere(new Point3D(0, 0, 0), 12, 16, 16);
        var sphereMat = new DiffuseMaterial(new SolidColorBrush(Colors.Magenta));
        group.Children.Add(new GeometryModel3D(sphereBuilder.ToMesh(), sphereMat) { BackMaterial = sphereMat });

        double axisLength = 60;
        double axisRadius = 2;
        double coneRadius = 5;
        double coneLength = 12;

        AddColoredAxis(group, new Point3D(0, 0, 0), new Point3D(axisLength, 0, 0),
            axisRadius, coneRadius, coneLength, Colors.Red);
        AddAxisLabel(group, "X", new Point3D(axisLength + 15, 0, 0), Colors.Red);

        AddColoredAxis(group, new Point3D(0, 0, 0), new Point3D(0, axisLength, 0),
            axisRadius, coneRadius, coneLength, Colors.LimeGreen);
        AddAxisLabel(group, "Y", new Point3D(0, axisLength + 15, 0), Colors.LimeGreen);

        AddColoredAxis(group, new Point3D(0, 0, 0), new Point3D(0, 0, axisLength),
            axisRadius, coneRadius, coneLength, Colors.DodgerBlue);
        AddAxisLabel(group, "Z", new Point3D(0, 0, axisLength + 15), Colors.DodgerBlue);

        _tcpMarker = group;
        return group;
    }

    private static void AddColoredAxis(Model3DGroup group, Point3D from, Point3D to,
        double radius, double coneRadius, double coneLength, Color color)
    {
        var builder = new MeshBuilder();
        builder.AddCylinder(from, to, radius, 8);

        var dir = to - from;
        dir.Normalize();
        builder.AddCone(to, dir, coneRadius, 0, coneLength, false, true, 12);

        var material = new DiffuseMaterial(new SolidColorBrush(color));
        group.Children.Add(new GeometryModel3D(builder.ToMesh(), material) { BackMaterial = material });
    }

    private static void AddAxisLabel(Model3DGroup group, string text, Point3D position, Color color)
    {
        double size = 12;
        var builder = new MeshBuilder();
        var p0 = new Point3D(position.X - size / 2, position.Y - size / 2, position.Z - size / 2);
        var p1 = new Point3D(position.X + size / 2, position.Y - size / 2, position.Z - size / 2);
        var p2 = new Point3D(position.X + size / 2, position.Y + size / 2, position.Z + size / 2);
        var p3 = new Point3D(position.X - size / 2, position.Y + size / 2, position.Z + size / 2);
        builder.AddQuad(p0, p1, p2, p3);

        var textBrush = CreateTextBrush(text, color);
        var material = new DiffuseMaterial(textBrush);
        var emissiveMat = new EmissiveMaterial(textBrush);
        var matGroup = new MaterialGroup();
        matGroup.Children.Add(material);
        matGroup.Children.Add(emissiveMat);
        group.Children.Add(new GeometryModel3D(builder.ToMesh(), matGroup) { BackMaterial = matGroup });
    }

    private static Brush CreateTextBrush(string text, Color color)
    {
        var visual = new DrawingVisual();
        using (var dc = visual.RenderOpen())
        {
            var formattedText = new FormattedText(
                text,
                CultureInfo.InvariantCulture,
                FlowDirection.LeftToRight,
                new Typeface("Arial"),
                48,
                new SolidColorBrush(color),
                1.0);

            dc.DrawText(formattedText, new Point(
                (64 - formattedText.Width) / 2,
                (64 - formattedText.Height) / 2));
        }

        var bmp = new System.Windows.Media.Imaging.RenderTargetBitmap(64, 64, 96, 96, PixelFormats.Pbgra32);
        bmp.Render(visual);

        return new ImageBrush(bmp) { Opacity = 1.0 };
    }

    // ========================================================================
    // Scene Object Management
    // ========================================================================

    public Model3DGroup? GetSceneGroup() => _sceneGroup;
    public Model3DGroup? GetBaseFrameGroup() => _baseFrameGroup;

    public SceneObject? AddSceneObject(string name, string stlPath, double scale = 1.0)
    {
        try
        {
            if (!File.Exists(stlPath))
            {
                Log.Warning("[ViewportService] STL file not found: {Path}", stlPath);
                return null;
            }

            SceneObject? obj = null;

            System.Windows.Application.Current.Dispatcher.Invoke(() =>
            {
                var reader = new StLReader();
                var meshes = reader.Read(stlPath);
                if (meshes == null || meshes.Children.Count == 0)
                {
                    Log.Warning("[ViewportService] No geometry in STL: {Path}", stlPath);
                    return;
                }

                var sourceGeom = meshes.Children[0] as GeometryModel3D;
                if (sourceGeom?.Geometry == null) return;

                var color = Color.FromRgb(160, 160, 160);
                var material = new DiffuseMaterial(new SolidColorBrush(color));

                var geometry = new GeometryModel3D(sourceGeom.Geometry, material)
                {
                    BackMaterial = material
                };

                // Wrap in ModelVisual3D for gizmo binding and hit testing
                var visual = new ModelVisual3D { Content = geometry };

                obj = new SceneObject
                {
                    Id = $"obj_{Guid.NewGuid():N}",
                    Name = name,
                    MeshPath = stlPath,
                    Scale = scale,
                    Color = color,
                    Geometry = geometry,
                    Visual = visual
                };

                // Apply initial transform
                UpdateSceneObjectVisualTransform(obj);

                // Add visual to container
                _sceneObjectsContainer.Children.Add(visual);
                _sceneVisuals[obj.Id] = visual;
                _sceneObjects.Add(obj);

                // Keep _sceneGroup for backward compat
                _sceneGroup ??= new Model3DGroup();
            });

            if (obj != null)
            {
                Log.Information("[ViewportService] Added scene object: {Name} ({Id}) from {Path}",
                    obj.Name, obj.Id, stlPath);
                SceneObjectsChanged?.Invoke(this, EventArgs.Empty);
                ModelUpdated?.Invoke(this, EventArgs.Empty);
            }

            return obj;
        }
        catch (Exception ex)
        {
            Log.Error(ex, "[ViewportService] Failed to add scene object from {Path}", stlPath);
            return null;
        }
    }

    public bool RemoveSceneObject(string id)
    {
        var obj = _sceneObjects.Find(o => o.Id == id);
        if (obj == null) return false;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            if (obj.Visual != null)
            {
                _sceneObjectsContainer.Children.Remove(obj.Visual);
            }
            _sceneVisuals.Remove(id);
            _originalMaterials.Remove(id);
        });

        _sceneObjects.Remove(obj);

        Log.Information("[ViewportService] Removed scene object: {Name} ({Id})", obj.Name, obj.Id);
        SceneObjectsChanged?.Invoke(this, EventArgs.Empty);
        ModelUpdated?.Invoke(this, EventArgs.Empty);
        return true;
    }

    public void UpdateSceneObjectTransform(string id, double[] position, double[] rotation, double scale)
    {
        var obj = _sceneObjects.Find(o => o.Id == id);
        if (obj == null) return;

        obj.X = position.Length > 0 ? position[0] : 0;
        obj.Y = position.Length > 1 ? position[1] : 0;
        obj.Z = position.Length > 2 ? position[2] : 0;
        obj.Rx = rotation.Length > 0 ? rotation[0] : 0;
        obj.Ry = rotation.Length > 1 ? rotation[1] : 0;
        obj.Rz = rotation.Length > 2 ? rotation[2] : 0;
        obj.Scale = scale > 0 ? scale : 1.0;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            UpdateSceneObjectVisualTransform(obj);
        });

        ModelUpdated?.Invoke(this, EventArgs.Empty);
    }

    public void UpdateSceneObjectData(string id, double x, double y, double z,
        double rx, double ry, double rz)
    {
        var obj = _sceneObjects.Find(o => o.Id == id);
        if (obj == null) return;

        obj.X = x; obj.Y = y; obj.Z = z;
        obj.Rx = rx; obj.Ry = ry; obj.Rz = rz;
    }

    public void SetSceneObjectParent(string id, string? frameId)
    {
        var obj = _sceneObjects.Find(o => o.Id == id);
        if (obj == null) return;

        obj.ParentFrameId = frameId;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            UpdateSceneObjectVisualTransform(obj);
        });

        Log.Information("[ViewportService] Set parent frame of {Name} to {Frame}",
            obj.Name, frameId ?? "World");
        SceneObjectsChanged?.Invoke(this, EventArgs.Empty);
        ModelUpdated?.Invoke(this, EventArgs.Empty);
    }

    public void SetSceneObjectColor(string id, Color color)
    {
        var obj = _sceneObjects.Find(o => o.Id == id);
        if (obj == null) return;

        obj.Color = color;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            if (obj.Geometry != null)
            {
                var material = new DiffuseMaterial(new SolidColorBrush(color));
                obj.Geometry.Material = material;
                obj.Geometry.BackMaterial = material;
            }
        });

        ModelUpdated?.Invoke(this, EventArgs.Empty);
    }

    public string? FindSceneObjectByVisual(Visual3D visual)
    {
        // Check direct match
        foreach (var kvp in _sceneVisuals)
        {
            if (kvp.Value == visual)
                return kvp.Key;
        }

        // Check if hit visual is a child/content of one of our visuals
        // (FindHits may return the ModelVisual3D or a sub-visual)
        if (visual is ModelVisual3D mv)
        {
            foreach (var kvp in _sceneVisuals)
            {
                if (kvp.Value == mv)
                    return kvp.Key;
            }
        }

        // Walk up the visual tree (multiple levels)
        var current = VisualTreeHelper.GetParent(visual) as Visual3D;
        while (current != null)
        {
            foreach (var kvp in _sceneVisuals)
            {
                if (kvp.Value == current)
                    return kvp.Key;
            }
            current = VisualTreeHelper.GetParent(current) as Visual3D;
        }

        return null;
    }

    public ModelVisual3D? GetSceneObjectVisual(string id)
    {
        return _sceneVisuals.GetValueOrDefault(id);
    }

    public void HighlightSceneObject(string id)
    {
        var obj = _sceneObjects.Find(o => o.Id == id);
        if (obj?.Geometry == null) return;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            _originalMaterials[id] = obj.Geometry.Material;

            var highlightColor = Color.FromArgb(255,
                (byte)Math.Min(255, obj.Color.R + 60),
                (byte)Math.Min(255, obj.Color.G + 60),
                (byte)Math.Min(255, obj.Color.B + 60));

            var matGroup = new MaterialGroup();
            matGroup.Children.Add(new DiffuseMaterial(new SolidColorBrush(highlightColor)));
            matGroup.Children.Add(new EmissiveMaterial(new SolidColorBrush(
                Color.FromArgb(80, obj.Color.R, obj.Color.G, obj.Color.B))));

            obj.Geometry.Material = matGroup;
            obj.Geometry.BackMaterial = matGroup;
        });
    }

    public void UnhighlightSceneObject(string id)
    {
        var obj = _sceneObjects.Find(o => o.Id == id);
        if (obj?.Geometry == null) return;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            if (_originalMaterials.TryGetValue(id, out var original))
            {
                obj.Geometry.Material = original;
                obj.Geometry.BackMaterial = original;
                _originalMaterials.Remove(id);
            }
        });
    }

    public Rect3D? GetSceneObjectBounds(string id)
    {
        var obj = _sceneObjects.Find(o => o.Id == id);
        if (obj?.Geometry == null) return null;

        Rect3D bounds = Rect3D.Empty;
        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            bounds = obj.Geometry.Bounds;
        });

        return bounds.IsEmpty ? null : bounds;
    }

    public Rect3D? GetTransformedSceneObjectBounds(string id, double rx, double ry, double rz, double scale)
    {
        var obj = _sceneObjects.Find(o => o.Id == id);
        if (obj?.Geometry == null) return null;

        Rect3D bounds = Rect3D.Empty;
        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            var mesh = obj.Geometry.Geometry as MeshGeometry3D;
            if (mesh == null || mesh.Positions.Count == 0) return;

            // Build scale + rotation transform (same order as UpdateSceneObjectVisualTransform, NO translation)
            var tg = new Transform3DGroup();
            if (Math.Abs(scale - 1.0) > 0.0001)
                tg.Children.Add(new ScaleTransform3D(scale, scale, scale));
            if (Math.Abs(rx) > 0.0001)
                tg.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), rx)));
            if (Math.Abs(ry) > 0.0001)
                tg.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), ry)));
            if (Math.Abs(rz) > 0.0001)
                tg.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), rz)));

            var matrix = tg.Value;
            foreach (var p in mesh.Positions)
            {
                bounds.Union(matrix.Transform(p));
            }
        });

        return bounds.IsEmpty ? null : bounds;
    }

    public void ShowHeightIndicator(string objectId, double robotBaseHeight)
    {
        var obj = _sceneObjects.Find(o => o.Id == objectId);
        if (obj?.Geometry == null) return;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            HideHeightIndicator(); // Remove existing

            var bounds = obj.Geometry.Bounds;
            if (bounds.IsEmpty) return;

            double scale = obj.Scale > 0 ? obj.Scale : 1.0;
            double meshBottomZ = obj.Z + bounds.Z * scale;
            double meshTopZ = obj.Z + (bounds.Z + bounds.SizeZ) * scale;

            var group = new Model3DGroup();
            double lineThickness = 2.0;

            // Use center X,Y of the object for indicator position
            double cx = obj.X + (bounds.X + bounds.SizeX / 2) * scale;
            double cy = obj.Y + (bounds.Y + bounds.SizeY / 2) * scale;
            // Offset slightly so it doesn't clip inside the mesh
            double indicatorX = cx + bounds.SizeX * scale * 0.6;
            double indicatorY = cy;

            // 1. Vertical line: ground (Z=0) → object top
            var maxZ = Math.Max(meshTopZ, robotBaseHeight);
            AddLineToGroup(group, new Point3D(indicatorX, indicatorY, 0),
                new Point3D(indicatorX, indicatorY, maxZ + 50), Colors.Cyan, lineThickness);

            // 2. Horizontal tick at ground (Z=0)
            double tickLen = 30;
            AddLineToGroup(group, new Point3D(indicatorX - tickLen, indicatorY, 0),
                new Point3D(indicatorX + tickLen, indicatorY, 0), Colors.Gray, lineThickness);

            // 3. Horizontal tick at object bottom
            AddLineToGroup(group, new Point3D(indicatorX - tickLen, indicatorY, meshBottomZ),
                new Point3D(indicatorX + tickLen, indicatorY, meshBottomZ), Colors.Cyan, lineThickness);

            // 4. Horizontal tick at object top
            AddLineToGroup(group, new Point3D(indicatorX - tickLen, indicatorY, meshTopZ),
                new Point3D(indicatorX + tickLen, indicatorY, meshTopZ), Colors.Cyan, lineThickness);

            // 5. Robot base height reference line (longer, distinct color)
            if (Math.Abs(robotBaseHeight) > 0.1)
            {
                double refLen = 80;
                AddLineToGroup(group, new Point3D(indicatorX - refLen, indicatorY, robotBaseHeight),
                    new Point3D(indicatorX + refLen, indicatorY, robotBaseHeight), Colors.Yellow, lineThickness + 1);
            }

            _heightIndicatorVisual = new ModelVisual3D { Content = group };
            _sceneObjectsContainer.Children.Add(_heightIndicatorVisual);
        });
    }

    public void HideHeightIndicator()
    {
        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            if (_heightIndicatorVisual != null)
            {
                _sceneObjectsContainer.Children.Remove(_heightIndicatorVisual);
                _heightIndicatorVisual = null;
            }
        });
    }

    private static void AddLineToGroup(Model3DGroup group, Point3D p1, Point3D p2, Color color, double thickness)
    {
        // Create a thin box (beam) between two points to represent a line
        // For vertical/horizontal lines this works well
        var dx = p2.X - p1.X;
        var dy = p2.Y - p1.Y;
        var dz = p2.Z - p1.Z;
        var length = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (length < 0.01) return;

        // Use a simple approach: create mesh positions for a thin quad
        var mesh = new MeshGeometry3D();
        var halfW = thickness / 2.0;

        // Determine perpendicular direction for width
        Vector3D dir = new Vector3D(dx, dy, dz);
        dir.Normalize();
        Vector3D perp;
        if (Math.Abs(dir.Z) > 0.9)
            perp = Vector3D.CrossProduct(dir, new Vector3D(1, 0, 0));
        else
            perp = Vector3D.CrossProduct(dir, new Vector3D(0, 0, 1));
        perp.Normalize();

        // 4 corners of a thin quad
        var p1a = new Point3D(p1.X + perp.X * halfW, p1.Y + perp.Y * halfW, p1.Z + perp.Z * halfW);
        var p1b = new Point3D(p1.X - perp.X * halfW, p1.Y - perp.Y * halfW, p1.Z - perp.Z * halfW);
        var p2a = new Point3D(p2.X + perp.X * halfW, p2.Y + perp.Y * halfW, p2.Z + perp.Z * halfW);
        var p2b = new Point3D(p2.X - perp.X * halfW, p2.Y - perp.Y * halfW, p2.Z - perp.Z * halfW);

        mesh.Positions.Add(p1a); // 0
        mesh.Positions.Add(p1b); // 1
        mesh.Positions.Add(p2b); // 2
        mesh.Positions.Add(p2a); // 3

        mesh.TriangleIndices.Add(0); mesh.TriangleIndices.Add(1); mesh.TriangleIndices.Add(2);
        mesh.TriangleIndices.Add(0); mesh.TriangleIndices.Add(2); mesh.TriangleIndices.Add(3);
        // Back face
        mesh.TriangleIndices.Add(2); mesh.TriangleIndices.Add(1); mesh.TriangleIndices.Add(0);
        mesh.TriangleIndices.Add(3); mesh.TriangleIndices.Add(2); mesh.TriangleIndices.Add(0);

        var material = new EmissiveMaterial(new SolidColorBrush(color));
        group.Children.Add(new GeometryModel3D(mesh, material) { BackMaterial = material });
    }

    private void UpdateSceneObjectVisualTransform(SceneObject obj)
    {
        if (obj.Visual == null) return;

        var transformGroup = new Transform3DGroup();

        // 1. Scale
        if (Math.Abs(obj.Scale - 1.0) > 0.0001)
        {
            transformGroup.Children.Add(new ScaleTransform3D(obj.Scale, obj.Scale, obj.Scale));
        }

        // 2. Local rotation (Rz * Ry * Rx)
        if (Math.Abs(obj.Rx) > 0.0001) transformGroup.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), obj.Rx)));
        if (Math.Abs(obj.Ry) > 0.0001) transformGroup.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), obj.Ry)));
        if (Math.Abs(obj.Rz) > 0.0001) transformGroup.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), obj.Rz)));

        // 3. Local translation
        transformGroup.Children.Add(new TranslateTransform3D(obj.X, obj.Y, obj.Z));

        // 4. If parented to a base frame, apply frame transform
        if (!string.IsNullOrEmpty(obj.ParentFrameId) && obj.ParentFrameId != "world"
            && _baseFrames.TryGetValue(obj.ParentFrameId, out var frameData) && frameData.Frame != null)
        {
            var frameTransform = CreateFrameTransform(frameData.Frame);
            transformGroup.Children.Add(new MatrixTransform3D(frameTransform));
        }

        obj.Visual.Transform = transformGroup;
    }

    /// <summary>
    /// Build a 4x4 transform matrix from FrameData (Translation + RPY rotation)
    /// </summary>
    private static Matrix3D CreateFrameTransform(FrameData frame)
    {
        var transform = new Transform3DGroup();

        if (Math.Abs(frame.Rx) > 0.0001) transform.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), frame.Rx)));
        if (Math.Abs(frame.Ry) > 0.0001) transform.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), frame.Ry)));
        if (Math.Abs(frame.Rz) > 0.0001) transform.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), frame.Rz)));

        transform.Children.Add(new TranslateTransform3D(frame.X, frame.Y, frame.Z));

        return transform.Value;
    }

    // ========================================================================
    // Base Frame Visualization
    // ========================================================================

    public void UpdateBaseFrameVisuals(List<BaseFrameData> frames, string activeFrameId)
    {
        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            _baseFrameGroup ??= new Model3DGroup();
            _baseFrameGroup.Children.Clear();
            _baseFrames.Clear();

            foreach (var frame in frames)
            {
                _baseFrames[frame.Id] = frame;

                if (frame.Id == "world") continue;

                bool isActive = frame.Id == activeFrameId;
                var frameAxes = CreateFrameAxes(frame, isActive);
                _baseFrameGroup.Children.Add(frameAxes);
            }

            // Recompute parented scene objects
            foreach (var obj in _sceneObjects)
            {
                if (!string.IsNullOrEmpty(obj.ParentFrameId) && obj.ParentFrameId != "world")
                {
                    UpdateSceneObjectVisualTransform(obj);
                }
            }
        });

        ModelUpdated?.Invoke(this, EventArgs.Empty);
    }

    private Model3DGroup CreateFrameAxes(BaseFrameData frame, bool isActive)
    {
        var group = new Model3DGroup();

        double axisLength = isActive ? 120 : 80;
        double axisRadius = isActive ? 3 : 2;
        double coneRadius = isActive ? 7 : 5;
        double coneLength = isActive ? 14 : 10;

        AddColoredAxis(group, new Point3D(0, 0, 0), new Point3D(axisLength, 0, 0),
            axisRadius, coneRadius, coneLength, Colors.Red);
        AddColoredAxis(group, new Point3D(0, 0, 0), new Point3D(0, axisLength, 0),
            axisRadius, coneRadius, coneLength, Colors.LimeGreen);
        AddColoredAxis(group, new Point3D(0, 0, 0), new Point3D(0, 0, axisLength),
            axisRadius, coneRadius, coneLength, Colors.DodgerBlue);

        var sphereBuilder = new MeshBuilder();
        var sphereColor = isActive ? Colors.Yellow : Colors.Orange;
        sphereBuilder.AddSphere(new Point3D(0, 0, 0), isActive ? 8 : 5, 12, 12);
        var sphereMat = new DiffuseMaterial(new SolidColorBrush(sphereColor));
        group.Children.Add(new GeometryModel3D(sphereBuilder.ToMesh(), sphereMat) { BackMaterial = sphereMat });

        AddAxisLabel(group, frame.Name, new Point3D(axisLength + 20, 0, 10),
            isActive ? Colors.Yellow : Colors.Orange);

        if (frame.Frame != null)
        {
            var frameTransform = CreateFrameTransform(frame.Frame);
            group.Transform = new MatrixTransform3D(frameTransform);
        }

        return group;
    }
}
