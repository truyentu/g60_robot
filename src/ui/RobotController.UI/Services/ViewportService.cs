using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
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

    /// <summary>Event when robot model updates</summary>
    event EventHandler? ModelUpdated;
}

/// <summary>
/// 3D Viewport service implementation
/// </summary>
public class ViewportService : IViewportService
{
    private RobotModel3D? _robot;
    private readonly string _modelsPath;
    private Model3DGroup? _tcpMarker;

    public RobotModel3D? Robot => _robot;
    public bool IsLoaded => _robot != null;

    public event EventHandler? ModelUpdated;

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
        // Uses URDF transform (consistent with viewport mesh rendering)
        // Note: DH FK orientation differs from URDF due to convention mismatch
        // (Modified DH params + URDF origins from standard ROS convention)
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

    public Model3DGroup? GetModelGroup()
    {
        return _robot?.ModelGroup;
    }

    public Model3DGroup CreateLighting()
    {
        var group = new Model3DGroup();

        // Ambient light
        group.Children.Add(new AmbientLight(Color.FromRgb(60, 60, 60)));

        // Main directional light
        group.Children.Add(new DirectionalLight(
            Color.FromRgb(200, 200, 200),
            new Vector3D(-1, -1, -1)));

        // Fill light
        group.Children.Add(new DirectionalLight(
            Color.FromRgb(100, 100, 100),
            new Vector3D(1, 0.5, 0.5)));

        // Back light
        group.Children.Add(new DirectionalLight(
            Color.FromRgb(80, 80, 80),
            new Vector3D(0, 1, 1)));

        return group;
    }

    public Model3DGroup CreateGrid(double size = 2000, double spacing = 100)
    {
        var group = new Model3DGroup();
        var builder = new MeshBuilder();

        double halfSize = size / 2;
        double lineThickness = 1;

        var gridColor = Color.FromRgb(60, 60, 60);

        // Create grid lines along X
        for (double y = -halfSize; y <= halfSize; y += spacing)
        {
            bool isMajor = Math.Abs(y) < 0.001 || Math.Abs(y % (spacing * 5)) < 0.001;
            builder.AddPipe(
                new Point3D(-halfSize, y, 0),
                new Point3D(halfSize, y, 0),
                0, isMajor ? lineThickness * 2 : lineThickness, 8);
        }

        // Create grid lines along Y
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

        // X axis - Red
        AddAxis(group, new Point3D(0, 0, 0), new Point3D(length, 0, 0),
            radius, arrowLength, Colors.Red);
        AddAxisLabel(group, "X", new Point3D(labelOffset, 0, 0), Colors.Red);

        // Y axis - Green
        AddAxis(group, new Point3D(0, 0, 0), new Point3D(0, length, 0),
            radius, arrowLength, Colors.LimeGreen);
        AddAxisLabel(group, "Y", new Point3D(0, labelOffset, 0), Colors.LimeGreen);

        // Z axis - Blue
        AddAxis(group, new Point3D(0, 0, 0), new Point3D(0, 0, length),
            radius, arrowLength, Colors.DodgerBlue);
        AddAxisLabel(group, "Z", new Point3D(0, 0, labelOffset), Colors.DodgerBlue);

        return group;
    }

    private void AddAxis(Model3DGroup group, Point3D start, Point3D end,
        double radius, double arrowLength, Color color)
    {
        var builder = new MeshBuilder();

        // Shaft
        var direction = end - start;
        direction.Normalize();
        var shaftEnd = end - direction * arrowLength;
        builder.AddCylinder(start, shaftEnd, radius, 16);

        // Arrow head
        builder.AddCone(shaftEnd, direction, radius * 2.5, radius * 2.5, arrowLength, true, true, 16);

        var mesh = builder.ToMesh();
        var material = new DiffuseMaterial(new SolidColorBrush(color));
        group.Children.Add(new GeometryModel3D(mesh, material) { BackMaterial = material });
    }

    public Model3DGroup CreateTcpMarker()
    {
        var group = new Model3DGroup();

        // Small sphere at TCP (magenta)
        var sphereBuilder = new MeshBuilder();
        sphereBuilder.AddSphere(new Point3D(0, 0, 0), 12, 16, 16);
        var sphereMat = new DiffuseMaterial(new SolidColorBrush(Colors.Magenta));
        group.Children.Add(new GeometryModel3D(sphereBuilder.ToMesh(), sphereMat) { BackMaterial = sphereMat });

        double axisLength = 60;
        double axisRadius = 2;
        double coneRadius = 5;
        double coneLength = 12;

        // X axis (Red)
        AddColoredAxis(group, new Point3D(0, 0, 0), new Point3D(axisLength, 0, 0),
            axisRadius, coneRadius, coneLength, Colors.Red);
        AddAxisLabel(group, "X", new Point3D(axisLength + 15, 0, 0), Colors.Red);

        // Y axis (Green)
        AddColoredAxis(group, new Point3D(0, 0, 0), new Point3D(0, axisLength, 0),
            axisRadius, coneRadius, coneLength, Colors.LimeGreen);
        AddAxisLabel(group, "Y", new Point3D(0, axisLength + 15, 0), Colors.LimeGreen);

        // Z axis (Blue)
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

        // Arrow cone at the tip
        var dir = to - from;
        dir.Normalize();
        var coneEnd = to + dir * coneLength;
        builder.AddCone(to, dir, coneRadius, 0, coneLength, false, true, 12);

        var material = new DiffuseMaterial(new SolidColorBrush(color));
        group.Children.Add(new GeometryModel3D(builder.ToMesh(), material) { BackMaterial = material });
    }

    private static void AddAxisLabel(Model3DGroup group, string text, Point3D position, Color color)
    {
        // Create text as a textured quad
        double size = 12;

        // Create a simple quad mesh at the label position
        var builder = new MeshBuilder();
        // Billboard-like quad facing camera (approximate - face towards +Y for now)
        var p0 = new Point3D(position.X - size / 2, position.Y - size / 2, position.Z - size / 2);
        var p1 = new Point3D(position.X + size / 2, position.Y - size / 2, position.Z - size / 2);
        var p2 = new Point3D(position.X + size / 2, position.Y + size / 2, position.Z + size / 2);
        var p3 = new Point3D(position.X - size / 2, position.Y + size / 2, position.Z + size / 2);

        builder.AddQuad(p0, p1, p2, p3);

        // Render text to a brush
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

            // Center the text
            dc.DrawText(formattedText, new Point(
                (64 - formattedText.Width) / 2,
                (64 - formattedText.Height) / 2));
        }

        var bmp = new System.Windows.Media.Imaging.RenderTargetBitmap(64, 64, 96, 96, PixelFormats.Pbgra32);
        bmp.Render(visual);

        return new ImageBrush(bmp) { Opacity = 1.0 };
    }
}
