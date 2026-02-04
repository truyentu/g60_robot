using System.IO;
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
            Log.Information("Initializing viewport from package: {Name}", package.Name);

            await System.Windows.Application.Current.Dispatcher.InvokeAsync(() =>
            {
                _robot = new RobotModel3D();
                _robot.InitializeFromPackage(package);
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
            Log.Error(ex, "Failed to initialize viewport from package");
            return false;
        }
    }

    public void UpdateJointAngles(double[] anglesDegrees)
    {
        if (_robot == null) return;

        _robot.SetAllJointAngles(anglesDegrees);

        // Update TCP marker position
        if (_tcpMarker != null)
        {
            var tcpPos = _robot.TcpPosition;
            _tcpMarker.Transform = new TranslateTransform3D(tcpPos.X, tcpPos.Y, tcpPos.Z);
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

        // X axis - Red
        AddAxis(group, new Point3D(0, 0, 0), new Point3D(length, 0, 0),
            radius, arrowLength, Colors.Red);

        // Y axis - Green
        AddAxis(group, new Point3D(0, 0, 0), new Point3D(0, length, 0),
            radius, arrowLength, Colors.LimeGreen);

        // Z axis - Blue
        AddAxis(group, new Point3D(0, 0, 0), new Point3D(0, 0, length),
            radius, arrowLength, Colors.DodgerBlue);

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
        var builder = new MeshBuilder();

        // Small sphere at TCP
        builder.AddSphere(new Point3D(0, 0, 0), 15, 16, 16);

        // Small coordinate frame
        double axisLength = 50;
        double axisRadius = 2;

        // X
        builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(axisLength, 0, 0), axisRadius, 8);
        // Y
        builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, axisLength, 0), axisRadius, 8);
        // Z
        builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, axisLength), axisRadius, 8);

        var mesh = builder.ToMesh();
        var material = new DiffuseMaterial(new SolidColorBrush(Colors.Magenta));
        group.Children.Add(new GeometryModel3D(mesh, material) { BackMaterial = material });

        _tcpMarker = group;
        return group;
    }
}
