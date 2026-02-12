using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using CommunityToolkit.Mvvm.Messaging;
using HelixToolkit.Wpf;
using RobotController.Common.Messages;
using RobotController.UI.Messages;
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

    // ========================================================================
    // TCP Gizmo (3D Interactive Jogging)
    // ========================================================================

    /// <summary>Create TCP gizmo with translate arrows and rotation rings</summary>
    ModelVisual3D CreateTcpGizmo();

    /// <summary>Update gizmo position/orientation to current TCP</summary>
    void UpdateTcpGizmoTransform();

    /// <summary>Show or hide the TCP gizmo</summary>
    void ShowTcpGizmo(bool show);

    /// <summary>Is gizmo currently visible</summary>
    bool IsTcpGizmoVisible { get; }

    /// <summary>Hit test gizmo - returns axis name ("tx","ty","tz","rx","ry","rz") or null</summary>
    string? HitTestGizmo(Visual3D hitVisual, GeometryModel3D? hitModel);

    /// <summary>Get current TCP pose as [x,y,z,rx,ry,rz] in mm/degrees</summary>
    double[]? GetCurrentTcpPose();

    /// <summary>Set gizmo frame mode (World or Tool)</summary>
    void SetGizmoFrame(GizmoFrame frame);

    /// <summary>Current gizmo frame mode</summary>
    GizmoFrame CurrentGizmoFrame { get; }

    /// <summary>Get the gizmo orientation matrix (Identity for World, TcpOrientation for Tool)</summary>
    Matrix3D GetGizmoOrientation();

    // ========================================================================
    // Ghost Robot (3D Jogging Preview)
    // ========================================================================

    /// <summary>Create ghost robot visual (semi-transparent clone of real robot)</summary>
    ModelVisual3D? CreateGhostRobot();

    /// <summary>Update ghost robot joint angles</summary>
    void UpdateGhostJoints(double[] jointsDegrees);

    /// <summary>Show or hide ghost robot</summary>
    void ShowGhost(bool show);

    /// <summary>Is ghost currently visible</summary>
    bool IsGhostVisible { get; }

    /// <summary>Set ghost reachability color (green=reachable, red=unreachable)</summary>
    void SetGhostReachable(bool reachable);

    // ========================================================================
    // Path Highlight (Editor <-> Viewport Sync)
    // ========================================================================

    /// <summary>Highlight a point in 3D viewport at given coordinates (orange sphere)</summary>
    void HighlightPathPoint(double x, double y, double z, string label = "");

    /// <summary>Clear the path highlight</summary>
    void ClearPathHighlight();

    // ========================================================================
    // TCP Path Trace (3D trail visualization)
    // ========================================================================

    /// <summary>Enable or disable TCP path trace recording</summary>
    void SetTcpTraceEnabled(bool enabled);

    /// <summary>Is TCP trace currently enabled</summary>
    bool IsTcpTraceEnabled { get; }

    /// <summary>Clear all recorded trace points</summary>
    void ClearTcpTrace();

    /// <summary>Get the trace visual for adding to viewport</summary>
    ModelVisual3D TcpTraceContainer { get; }

    /// <summary>Set the current motion type for color-coding the trace</summary>
    void SetTraceMotionType(string motionType);
}

/// <summary>
/// Gizmo coordinate frame mode
/// </summary>
public enum GizmoFrame
{
    World,
    Tool
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

    // TCP Gizmo (3D Interactive Jogging)
    private ModelVisual3D? _tcpGizmoVisual;
    private bool _tcpGizmoVisible;
    private readonly Dictionary<GeometryModel3D, string> _gizmoAxisMap = new();
    // Gizmo geometry references for hit testing
    private GeometryModel3D? _gizmoArrowX, _gizmoArrowY, _gizmoArrowZ;
    private GeometryModel3D? _gizmoRingX, _gizmoRingY, _gizmoRingZ;
    private GeometryModel3D? _gizmoPlaneXY, _gizmoPlaneXZ, _gizmoPlaneYZ;
    private GeometryModel3D? _gizmoCenterSphere;
    private GizmoFrame _gizmoFrame = GizmoFrame.World;

    // Ghost Robot (3D Jogging Preview)
    private ModelVisual3D? _ghostVisual;
    private Model3DGroup? _ghostModelGroup;
    private bool _ghostVisible;

    // Path highlight (Editor <-> Viewport sync)
    private ModelVisual3D? _pathHighlightVisual;
    private Model3DGroup? _pathHighlightGroup;

    // TCP Path Trace (3D trail visualization)
    private readonly ModelVisual3D _tcpTraceContainer = new();
    private bool _tcpTraceEnabled;
    private string _traceMotionType = "LIN";
    private Point3D? _traceLastPoint;
    private const int MaxTraceSegments = 10000;
    private const double MinTraceDistance = 0.5; // mm - skip points too close together

    // Per-motion-type trace groups (separate LinesVisual3D for color coding)
    private readonly Dictionary<string, (LinesVisual3D visual, int count)> _traceLines = new();

    // TCP pose from C++ Core FK (authoritative for IK requests)
    private double[]? _coreTcpPose;

    public RobotModel3D? Robot => _robot;
    public bool IsLoaded => _robot != null;
    public bool IsPickingTcp => _isPickingTcp;
    public bool IsTcpGizmoVisible => _tcpGizmoVisible;
    public GizmoFrame CurrentGizmoFrame => _gizmoFrame;
    public bool IsGhostVisible => _ghostVisible;
    public List<SceneObject> SceneObjects => _sceneObjects;
    public ModelVisual3D SceneObjectsContainer => _sceneObjectsContainer;
    public bool IsTcpTraceEnabled => _tcpTraceEnabled;
    public ModelVisual3D TcpTraceContainer => _tcpTraceContainer;

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

        // Subscribe to editor -> viewport messages for path highlighting
        WeakReferenceMessenger.Default.Register<EditorCaretOnMotionLineMessage>(this, (r, m) =>
        {
            var info = m.Value;
            if (info.HasPosition)
            {
                ((ViewportService)r).HighlightPathPoint(info.X, info.Y, info.Z, info.TargetName);
            }
        });

        WeakReferenceMessenger.Default.Register<EditorCaretOffMotionLineMessage>(this, (r, _) =>
        {
            ((ViewportService)r).ClearPathHighlight();
        });
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

        // Store C++ Core TCP pose (authoritative for IK)
        if (tcpPose != null && tcpPose.Length >= 6)
        {
            _coreTcpPose = tcpPose;
        }

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

        // Update TCP gizmo position to follow TCP
        if (_tcpGizmoVisible)
        {
            UpdateTcpGizmoTransform();
        }

        // Record TCP path trace point
        if (_tcpTraceEnabled && _robot != null)
        {
            RecordTracePoint(_robot.TcpPosition);
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

    // ========================================================================
    // TCP Gizmo (3D Interactive Jogging)
    // ========================================================================

    public ModelVisual3D CreateTcpGizmo()
    {
        _tcpGizmoVisual = new ModelVisual3D();
        var group = new Model3DGroup();
        _gizmoAxisMap.Clear();

        double arrowLength = 150;
        double arrowRadius = 4;
        double coneRadius = 10;
        double coneLength = 25;
        double ringRadius = 120;
        double ringTubeRadius = 4;

        // Translation arrows (thicker than TCP marker)
        _gizmoArrowX = CreateGizmoArrow(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), arrowLength, arrowRadius, coneRadius, coneLength, Colors.Red);
        _gizmoArrowY = CreateGizmoArrow(new Point3D(0, 0, 0), new Vector3D(0, 1, 0), arrowLength, arrowRadius, coneRadius, coneLength, Colors.LimeGreen);
        _gizmoArrowZ = CreateGizmoArrow(new Point3D(0, 0, 0), new Vector3D(0, 0, 1), arrowLength, arrowRadius, coneRadius, coneLength, Colors.DodgerBlue);

        group.Children.Add(_gizmoArrowX);
        group.Children.Add(_gizmoArrowY);
        group.Children.Add(_gizmoArrowZ);

        _gizmoAxisMap[_gizmoArrowX] = "tx";
        _gizmoAxisMap[_gizmoArrowY] = "ty";
        _gizmoAxisMap[_gizmoArrowZ] = "tz";

        // Rotation rings (torus around each axis)
        _gizmoRingX = CreateGizmoRing(new Vector3D(1, 0, 0), ringRadius, ringTubeRadius, Colors.Red);
        _gizmoRingY = CreateGizmoRing(new Vector3D(0, 1, 0), ringRadius, ringTubeRadius, Colors.LimeGreen);
        _gizmoRingZ = CreateGizmoRing(new Vector3D(0, 0, 1), ringRadius, ringTubeRadius, Colors.DodgerBlue);

        group.Children.Add(_gizmoRingX);
        group.Children.Add(_gizmoRingY);
        group.Children.Add(_gizmoRingZ);

        _gizmoAxisMap[_gizmoRingX] = "rx";
        _gizmoAxisMap[_gizmoRingY] = "ry";
        _gizmoAxisMap[_gizmoRingZ] = "rz";

        // Plane drag squares (small quads between axis pairs)
        double planeOffset = 40; // distance from origin along each axis
        double planeSize = 30;   // size of the square

        _gizmoPlaneXY = CreateGizmoPlaneSquare(
            new Vector3D(1, 0, 0), new Vector3D(0, 1, 0),
            planeOffset, planeSize, Color.FromArgb(120, 255, 255, 0)); // Yellow semi-transparent
        _gizmoPlaneXZ = CreateGizmoPlaneSquare(
            new Vector3D(1, 0, 0), new Vector3D(0, 0, 1),
            planeOffset, planeSize, Color.FromArgb(120, 255, 0, 255)); // Magenta semi-transparent
        _gizmoPlaneYZ = CreateGizmoPlaneSquare(
            new Vector3D(0, 1, 0), new Vector3D(0, 0, 1),
            planeOffset, planeSize, Color.FromArgb(120, 0, 255, 255)); // Cyan semi-transparent

        group.Children.Add(_gizmoPlaneXY);
        group.Children.Add(_gizmoPlaneXZ);
        group.Children.Add(_gizmoPlaneYZ);

        _gizmoAxisMap[_gizmoPlaneXY] = "txy";
        _gizmoAxisMap[_gizmoPlaneXZ] = "txz";
        _gizmoAxisMap[_gizmoPlaneYZ] = "tyz";

        // Center sphere for 3D free-drag (all XYZ simultaneously)
        var sphereBuilder = new MeshBuilder();
        sphereBuilder.AddSphere(new Point3D(0, 0, 0), 12, 16, 16);
        var sphereColor = Color.FromArgb(200, 255, 255, 255); // White semi-transparent
        var sphereMat = new DiffuseMaterial(new SolidColorBrush(sphereColor));
        _gizmoCenterSphere = new GeometryModel3D(sphereBuilder.ToMesh(), sphereMat) { BackMaterial = sphereMat };
        group.Children.Add(_gizmoCenterSphere);
        _gizmoAxisMap[_gizmoCenterSphere] = "txyz";

        _tcpGizmoVisual.Content = group;
        _tcpGizmoVisible = false;
        _tcpGizmoVisual.SetValue(ModelVisual3D.TransformProperty, Transform3D.Identity);

        return _tcpGizmoVisual;
    }

    private static GeometryModel3D CreateGizmoArrow(Point3D origin, Vector3D direction, double length, double radius, double coneRadius, double coneLength, Color color)
    {
        var builder = new MeshBuilder();
        var end = origin + direction * length;
        builder.AddCylinder(origin, end, radius, 8);
        builder.AddCone(end, direction, coneRadius, 0, coneLength, false, true, 12);

        var material = new DiffuseMaterial(new SolidColorBrush(color));
        return new GeometryModel3D(builder.ToMesh(), material) { BackMaterial = material };
    }

    private static GeometryModel3D CreateGizmoRing(Vector3D axis, double radius, double tubeRadius, Color color)
    {
        var builder = new MeshBuilder();

        // Build torus as segments
        int segments = 36;
        int tubeSegments = 12;

        // Determine two perpendicular vectors to the axis
        Vector3D u, v;
        if (Math.Abs(axis.Z) > 0.9)
        {
            u = Vector3D.CrossProduct(axis, new Vector3D(1, 0, 0));
        }
        else
        {
            u = Vector3D.CrossProduct(axis, new Vector3D(0, 0, 1));
        }
        u.Normalize();
        v = Vector3D.CrossProduct(axis, u);
        v.Normalize();

        var positions = new List<Point3D>();
        var indices = new List<int>();

        for (int i = 0; i < segments; i++)
        {
            double theta = 2 * Math.PI * i / segments;
            // Center of tube cross-section on the ring
            var ringCenter = new Point3D(
                radius * (u.X * Math.Cos(theta) + v.X * Math.Sin(theta)),
                radius * (u.Y * Math.Cos(theta) + v.Y * Math.Sin(theta)),
                radius * (u.Z * Math.Cos(theta) + v.Z * Math.Sin(theta)));

            // Radial direction from ring center
            var radial = new Vector3D(ringCenter.X, ringCenter.Y, ringCenter.Z);
            radial.Normalize();

            for (int j = 0; j < tubeSegments; j++)
            {
                double phi = 2 * Math.PI * j / tubeSegments;
                var tubePoint = ringCenter +
                    tubeRadius * Math.Cos(phi) * radial +
                    tubeRadius * Math.Sin(phi) * axis;
                positions.Add(tubePoint);
            }
        }

        // Build triangle indices
        for (int i = 0; i < segments; i++)
        {
            int nextI = (i + 1) % segments;
            for (int j = 0; j < tubeSegments; j++)
            {
                int nextJ = (j + 1) % tubeSegments;
                int a = i * tubeSegments + j;
                int b = nextI * tubeSegments + j;
                int c = nextI * tubeSegments + nextJ;
                int d = i * tubeSegments + nextJ;
                indices.Add(a); indices.Add(b); indices.Add(c);
                indices.Add(a); indices.Add(c); indices.Add(d);
            }
        }

        var mesh = new MeshGeometry3D();
        foreach (var p in positions) mesh.Positions.Add(p);
        foreach (var idx in indices) mesh.TriangleIndices.Add(idx);

        var material = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(180, color.R, color.G, color.B)));
        return new GeometryModel3D(mesh, material) { BackMaterial = material };
    }

    private static GeometryModel3D CreateGizmoPlaneSquare(Vector3D axis1, Vector3D axis2, double offset, double size, Color color)
    {
        // Create a small quad between two axes at 'offset' distance from origin
        var mesh = new MeshGeometry3D();
        var p0 = new Point3D(0, 0, 0) + axis1 * offset + axis2 * offset;
        var p1 = p0 + axis1 * size;
        var p2 = p0 + axis1 * size + axis2 * size;
        var p3 = p0 + axis2 * size;

        mesh.Positions.Add(p0);
        mesh.Positions.Add(p1);
        mesh.Positions.Add(p2);
        mesh.Positions.Add(p3);

        // Two triangles for quad
        mesh.TriangleIndices.Add(0); mesh.TriangleIndices.Add(1); mesh.TriangleIndices.Add(2);
        mesh.TriangleIndices.Add(0); mesh.TriangleIndices.Add(2); mesh.TriangleIndices.Add(3);

        var material = new DiffuseMaterial(new SolidColorBrush(color));
        return new GeometryModel3D(mesh, material) { BackMaterial = material };
    }

    public void UpdateTcpGizmoTransform()
    {
        if (_tcpGizmoVisual == null || _robot == null) return;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            var tcpPos = _robot.TcpPosition;

            var transformGroup = new Transform3DGroup();

            if (_gizmoFrame == GizmoFrame.Tool)
            {
                // Tool mode: rotate gizmo by TCP orientation (rotation only, no translation from matrix)
                var ori = _robot.TcpOrientation;
                var rotMatrix = new Matrix3D(
                    ori.M11, ori.M12, ori.M13, 0,
                    ori.M21, ori.M22, ori.M23, 0,
                    ori.M31, ori.M32, ori.M33, 0,
                    0, 0, 0, 1);
                transformGroup.Children.Add(new MatrixTransform3D(rotMatrix));
            }

            // Position at TCP (applied after rotation)
            transformGroup.Children.Add(new TranslateTransform3D(tcpPos.X, tcpPos.Y, tcpPos.Z));

            _tcpGizmoVisual.Transform = transformGroup;
        });
    }

    public void ShowTcpGizmo(bool show)
    {
        _tcpGizmoVisible = show;
        if (_tcpGizmoVisual != null)
        {
            System.Windows.Application.Current.Dispatcher.Invoke(() =>
            {
                if (show)
                {
                    UpdateTcpGizmoTransform();
                    // Make visible by setting non-empty content
                    if (_tcpGizmoVisual.Content is Model3DGroup g)
                    {
                        foreach (var child in g.Children)
                        {
                            if (child is GeometryModel3D gm)
                                gm.SetValue(GeometryModel3D.MaterialProperty, gm.Material);
                        }
                    }
                }
                else
                {
                    // Hide by moving far away
                    _tcpGizmoVisual.Transform = new TranslateTransform3D(0, 0, -99999);
                }
            });
        }
    }

    public string? HitTestGizmo(Visual3D hitVisual, GeometryModel3D? hitModel)
    {
        if (!_tcpGizmoVisible || hitModel == null) return null;

        if (_gizmoAxisMap.TryGetValue(hitModel, out var axis))
        {
            return axis;
        }

        return null;
    }

    public double[]? GetCurrentTcpPose()
    {
        // Prefer C++ Core TCP pose (matches IK solver exactly)
        if (_coreTcpPose != null)
            return _coreTcpPose;

        // Fallback to C# FK
        if (_robot == null) return null;
        return _robot.GetTcpPose();
    }

    public void SetGizmoFrame(GizmoFrame frame)
    {
        _gizmoFrame = frame;
        if (_tcpGizmoVisible)
        {
            UpdateTcpGizmoTransform();
        }
    }

    public Matrix3D GetGizmoOrientation()
    {
        if (_gizmoFrame == GizmoFrame.Tool && _robot != null)
        {
            // Return rotation-only part of TcpOrientation
            var ori = _robot.TcpOrientation;
            return new Matrix3D(
                ori.M11, ori.M12, ori.M13, 0,
                ori.M21, ori.M22, ori.M23, 0,
                ori.M31, ori.M32, ori.M33, 0,
                0, 0, 0, 1);
        }
        return Matrix3D.Identity;
    }

    // ========================================================================
    // Ghost Robot (3D Jogging Preview)
    // ========================================================================

    public ModelVisual3D? CreateGhostRobot()
    {
        if (_robot == null) return null;

        _ghostModelGroup = _robot.CreateGhostModel(Colors.Yellow, 100);
        _ghostVisual = new ModelVisual3D();
        _ghostVisible = false;

        // Hidden initially: Content is null (nothing to render)

        return _ghostVisual;
    }

    public void UpdateGhostJoints(double[] jointsDegrees)
    {
        if (_ghostModelGroup == null || _robot == null || !_ghostVisible) return;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            _robot.UpdateGhostFK(_ghostModelGroup, jointsDegrees);
        });
    }

    public void ShowGhost(bool show)
    {
        _ghostVisible = show;
        if (_ghostVisual == null) return;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            if (show)
            {
                // Show: set Content and update FK to current robot pose
                _ghostVisual.Content = _ghostModelGroup;
                _ghostVisual.Transform = Transform3D.Identity;
                if (_robot != null && _ghostModelGroup != null)
                {
                    _robot.UpdateGhostFK(_ghostModelGroup, _robot.JointAnglesDegrees);
                }
            }
            else
            {
                // Hide: remove Content entirely (no geometry = nothing rendered)
                _ghostVisual.Content = null;
            }
        });
    }

    public void SetGhostReachable(bool reachable)
    {
        if (_ghostModelGroup == null) return;

        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            var color = reachable
                ? Color.FromArgb(100, 0, 255, 100)    // semi-transparent green
                : Color.FromArgb(100, 255, 50, 50);   // semi-transparent red
            var material = new DiffuseMaterial(new SolidColorBrush(color));

            foreach (var child in _ghostModelGroup.Children)
            {
                if (child is GeometryModel3D geo && geo.Geometry != null)
                {
                    geo.Material = material;
                    geo.BackMaterial = material;
                }
            }
        });
    }

    // ========================================================================
    // Path Highlight (Editor <-> Viewport Sync)
    // ========================================================================

    // ========================================================================
    // TCP Path Trace (3D trail visualization)
    // ========================================================================

    private static Color GetMotionTypeColor(string motionType)
    {
        return motionType.ToUpperInvariant() switch
        {
            "PTP" or "MOVEJ" or "MOVJ" => Color.FromRgb(60, 140, 255),    // Blue - joint move
            "LIN" or "MOVEL" or "MOVL" => Color.FromRgb(80, 255, 80),     // Green - linear
            "CIRC" or "MOVEC" or "MOVC" => Color.FromRgb(255, 100, 50),   // Orange-red - circular
            _ => Color.FromRgb(200, 200, 200)                              // Gray - unknown
        };
    }

    private LinesVisual3D GetOrCreateTraceLine(string motionType)
    {
        if (_traceLines.TryGetValue(motionType, out var entry))
        {
            return entry.visual;
        }

        var line = new LinesVisual3D
        {
            Color = GetMotionTypeColor(motionType),
            Thickness = 2.0
        };
        _traceLines[motionType] = (line, 0);
        _tcpTraceContainer.Children.Add(line);
        return line;
    }

    private void RecordTracePoint(Point3D newPoint)
    {
        if (_traceLastPoint.HasValue)
        {
            var dx = newPoint.X - _traceLastPoint.Value.X;
            var dy = newPoint.Y - _traceLastPoint.Value.Y;
            var dz = newPoint.Z - _traceLastPoint.Value.Z;
            double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            if (dist < MinTraceDistance) return;

            var line = GetOrCreateTraceLine(_traceMotionType);
            var points = line.Points;

            // LinesVisual3D uses point pairs: [start, end, start, end, ...]
            points.Add(_traceLastPoint.Value);
            points.Add(newPoint);

            // Update segment count
            if (_traceLines.TryGetValue(_traceMotionType, out var entry))
            {
                _traceLines[_traceMotionType] = (entry.visual, entry.count + 1);
            }

            // Evict oldest segments across all lines if total exceeds limit
            int totalSegments = 0;
            foreach (var kvp in _traceLines) totalSegments += kvp.Value.count;

            if (totalSegments > MaxTraceSegments)
            {
                // Remove from the line with the most segments
                string? maxKey = null;
                int maxCount = 0;
                foreach (var kvp in _traceLines)
                {
                    if (kvp.Value.count > maxCount)
                    {
                        maxCount = kvp.Value.count;
                        maxKey = kvp.Key;
                    }
                }
                if (maxKey != null && _traceLines[maxKey].visual.Points.Count >= 2)
                {
                    _traceLines[maxKey].visual.Points.RemoveAt(0);
                    _traceLines[maxKey].visual.Points.RemoveAt(0);
                    _traceLines[maxKey] = (_traceLines[maxKey].visual, _traceLines[maxKey].count - 1);
                }
            }
        }

        _traceLastPoint = newPoint;
    }

    public void SetTcpTraceEnabled(bool enabled)
    {
        _tcpTraceEnabled = enabled;
        if (enabled)
        {
            // Capture current TCP position as starting point
            if (_robot != null)
            {
                _traceLastPoint = _robot.TcpPosition;
            }
            Log.Information("[ViewportService] TCP trace ENABLED");
        }
        else
        {
            _traceLastPoint = null;
            Log.Information("[ViewportService] TCP trace DISABLED");
        }
    }

    public void ClearTcpTrace()
    {
        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            foreach (var kvp in _traceLines)
            {
                kvp.Value.visual.Points.Clear();
            }
            _traceLines.Clear();
            _tcpTraceContainer.Children.Clear();
            _traceLastPoint = null;
        });
        Log.Information("[ViewportService] TCP trace cleared");
    }

    public void SetTraceMotionType(string motionType)
    {
        _traceMotionType = motionType;
    }

    public void HighlightPathPoint(double x, double y, double z, string label = "")
    {
        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            // Create or reuse highlight group
            if (_pathHighlightGroup == null)
            {
                _pathHighlightGroup = new Model3DGroup();
                _pathHighlightVisual = new ModelVisual3D { Content = _pathHighlightGroup };
            }

            _pathHighlightGroup.Children.Clear();

            // Orange sphere at the target point
            var sphereMesh = new MeshGeometry3D();
            double radius = 8.0;
            int segments = 12;
            var center = new Point3D(x, y, z);

            // Generate UV sphere
            for (int lat = 0; lat <= segments; lat++)
            {
                double theta = lat * Math.PI / segments;
                double sinTheta = Math.Sin(theta);
                double cosTheta = Math.Cos(theta);

                for (int lon = 0; lon <= segments; lon++)
                {
                    double phi = lon * 2 * Math.PI / segments;
                    double sinPhi = Math.Sin(phi);
                    double cosPhi = Math.Cos(phi);

                    double px = center.X + radius * sinTheta * cosPhi;
                    double py = center.Y + radius * sinTheta * sinPhi;
                    double pz = center.Z + radius * cosTheta;

                    sphereMesh.Positions.Add(new Point3D(px, py, pz));
                    sphereMesh.Normals.Add(new Vector3D(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta));

                    if (lat < segments && lon < segments)
                    {
                        int current = lat * (segments + 1) + lon;
                        int next = current + segments + 1;

                        sphereMesh.TriangleIndices.Add(current);
                        sphereMesh.TriangleIndices.Add(next);
                        sphereMesh.TriangleIndices.Add(current + 1);

                        sphereMesh.TriangleIndices.Add(current + 1);
                        sphereMesh.TriangleIndices.Add(next);
                        sphereMesh.TriangleIndices.Add(next + 1);
                    }
                }
            }

            var orangeMaterial = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(200, 255, 165, 0)));
            var sphereModel = new GeometryModel3D(sphereMesh, orangeMaterial)
            {
                BackMaterial = orangeMaterial
            };

            _pathHighlightGroup.Children.Add(sphereModel);

            // Add emissive glow
            var glowMaterial = new EmissiveMaterial(new SolidColorBrush(Color.FromArgb(80, 255, 165, 0)));
            var glowModel = new GeometryModel3D(sphereMesh, glowMaterial);
            _pathHighlightGroup.Children.Add(glowModel);

            Log.Debug("Path highlight at ({X:F1}, {Y:F1}, {Z:F1}) {Label}", x, y, z, label);
        });
    }

    public void ClearPathHighlight()
    {
        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            _pathHighlightGroup?.Children.Clear();
        });
    }
}
