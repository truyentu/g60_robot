using System.IO;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using Serilog;

namespace RobotController.UI.Models;

/// <summary>
/// 3D Robot Model with Forward Kinematics
/// </summary>
public class RobotModel3D
{
    private readonly List<RobotLink> _links = new();
    private readonly double[] _jointAngles = new double[6];
    private Model3DGroup? _modelGroup;
    private static readonly string _debugLogPath = Path.Combine(
        AppDomain.CurrentDomain.BaseDirectory, "stl_debug.log");

    /// <summary>Robot name from config</summary>
    public string Name { get; private set; } = "Robot";

    /// <summary>Number of joints (always 6)</summary>
    public int NumJoints => 6;

    /// <summary>All robot links including base</summary>
    public IReadOnlyList<RobotLink> Links => _links;

    /// <summary>Current joint angles in degrees</summary>
    public double[] JointAnglesDegrees => _jointAngles.Select(a => a * 180.0 / Math.PI).ToArray();

    /// <summary>3D model group for viewport</summary>
    public Model3DGroup? ModelGroup => _modelGroup;

    /// <summary>TCP (Tool Center Point) position</summary>
    public Point3D TcpPosition { get; private set; }

    /// <summary>TCP orientation as rotation matrix</summary>
    public Matrix3D TcpOrientation { get; private set; } = Matrix3D.Identity;

    /// <summary>
    /// Initialize robot model from configuration
    /// </summary>
    public bool Initialize(RobotConfigData config, string modelsPath)
    {
        try
        {
            Name = config.Name;
            _links.Clear();
            _modelGroup = new Model3DGroup();

            Log.Information("Initializing robot model: {Name}", Name);

            // Create base link (fixed)
            var baseLink = CreateLink(0, "Base", null, modelsPath);
            if (baseLink != null)
            {
                _links.Add(baseLink);
                if (baseLink.Geometry != null)
                    _modelGroup.Children.Add(baseLink.Geometry);
            }

            // Create joint links (1-6)
            for (int i = 0; i < config.DHParameters.Count && i < 6; i++)
            {
                var dhParam = config.DHParameters[i];
                var link = CreateLink(i + 1, $"Link{i + 1}", dhParam, modelsPath);
                if (link != null)
                {
                    _links.Add(link);
                    if (link.Geometry != null)
                        _modelGroup.Children.Add(link.Geometry);
                }
            }

            // Set home position
            if (config.HomePosition.Length >= 6)
            {
                for (int i = 0; i < 6; i++)
                {
                    _jointAngles[i] = config.HomePosition[i] * Math.PI / 180.0;
                }
            }

            // Initial FK update
            UpdateForwardKinematics();

            Log.Information("Robot model initialized with {Count} links", _links.Count);
            return true;
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to initialize robot model");
            return false;
        }
    }

    /// <summary>
    /// Initialize robot model from package data (with mesh paths)
    /// </summary>
    public bool InitializeFromPackage(RobotPackageData package)
    {
        try
        {
            DebugLog($"\n========================================");
            DebugLog($"InitializeFromPackage START");
            DebugLog($"========================================");
            DebugLog($"Package Name: {package.Name}");
            DebugLog($"Package ID: {package.Id}");
            DebugLog($"Package Path: {package.PackagePath}");
            DebugLog($"Base Mesh: {package.BaseMesh ?? "NULL"}");
            DebugLog($"Joints Count: {package.Joints.Count}");

            Name = package.Name;
            _links.Clear();
            _modelGroup = new Model3DGroup();

            Log.Information("Initializing robot model from package: {Name}", Name);

            // Create base link
            DebugLog($"\n--- Creating Base Link ---");
            var baseLink = CreateLinkFromPackage(0, "Base", null, package.PackagePath, package.BaseMesh);
            if (baseLink != null)
            {
                _links.Add(baseLink);
                if (baseLink.Geometry != null)
                {
                    _modelGroup.Children.Add(baseLink.Geometry);
                    DebugLog($"Base link geometry added to ModelGroup");
                }
                else
                {
                    DebugLog($"WARNING: Base link has NULL geometry");
                }
            }
            else
            {
                DebugLog($"ERROR: Base link creation returned NULL");
            }

            // Create joint links
            for (int i = 0; i < package.Joints.Count && i < 6; i++)
            {
                DebugLog($"\n--- Creating Joint Link {i + 1} ---");
                var jointDef = package.Joints[i];
                DebugLog($"Joint Name: {jointDef.Name}");
                DebugLog($"Joint Type: {jointDef.Type}");
                DebugLog($"Mesh VisualMesh: {jointDef.Mesh?.VisualMesh ?? "NULL"}");

                var link = CreateLinkFromPackage(i + 1, jointDef.Name, jointDef, package.PackagePath, jointDef.Mesh?.VisualMesh ?? "");
                if (link != null)
                {
                    _links.Add(link);
                    if (link.Geometry != null)
                    {
                        _modelGroup.Children.Add(link.Geometry);
                        DebugLog($"Joint link {i + 1} geometry added to ModelGroup");
                    }
                    else
                    {
                        DebugLog($"WARNING: Joint link {i + 1} has NULL geometry");
                    }
                }
                else
                {
                    DebugLog($"ERROR: Joint link {i + 1} creation returned NULL");
                }
            }

            // Set home position
            if (package.HomePosition.Length >= 6)
            {
                for (int i = 0; i < 6; i++)
                {
                    _jointAngles[i] = package.HomePosition[i] * Math.PI / 180.0;
                }
                DebugLog($"Home position set: [{string.Join(", ", package.HomePosition)}]");
            }

            UpdateForwardKinematics();

            DebugLog($"\nFinal Summary:");
            DebugLog($"Total links created: {_links.Count}");
            DebugLog($"ModelGroup children count: {_modelGroup.Children.Count}");
            DebugLog($"========================================");
            DebugLog($"InitializeFromPackage END (SUCCESS)");
            DebugLog($"========================================\n");

            Log.Information("Robot model initialized from package with {Count} links", _links.Count);
            return true;
        }
        catch (Exception ex)
        {
            DebugLog($"\nEXCEPTION in InitializeFromPackage:");
            DebugLog($"Type: {ex.GetType().Name}");
            DebugLog($"Message: {ex.Message}");
            DebugLog($"StackTrace: {ex.StackTrace}");
            DebugLog($"========================================");
            DebugLog($"InitializeFromPackage END (EXCEPTION)");
            DebugLog($"========================================\n");

            Log.Error(ex, "Failed to initialize robot model from package");
            return false;
        }
    }

    /// <summary>
    /// Create a robot link from package data
    /// </summary>
    private RobotLink? CreateLinkFromPackage(int jointIndex, string name, JointDefinitionData? jointDef, string packagePath, string meshPath)
    {
        var link = new RobotLink
        {
            Name = name,
            JointIndex = jointIndex
        };

        // DEBUG: Write to text file
        DebugLog($"=== CreateLinkFromPackage ===");
        DebugLog($"JointIndex: {jointIndex}, Name: {name}");
        DebugLog($"PackagePath: {packagePath ?? "NULL"}");
        DebugLog($"MeshPath: {meshPath ?? "NULL"}");

        // Set DH parameters from joint definition
        if (jointDef != null)
        {
            link.DH = new DHParameter
            {
                Joint = jointIndex,
                A = jointDef.DhA,
                Alpha = jointDef.DhAlpha,
                D = jointDef.DhD,
                ThetaOffset = jointDef.DhThetaOffset
            };

            // Set URDF origin data (for visualization)
            link.OriginXyz = jointDef.OriginXyz;
            link.OriginRpy = jointDef.OriginRpy;
            link.Axis = jointDef.Axis;

            // Debug: Log URDF data
            if (jointDef.OriginXyz != null)
            {
                DebugLog($"URDF OriginXyz: [{string.Join(", ", jointDef.OriginXyz)}]");
                DebugLog($"URDF OriginRpy: [{string.Join(", ", jointDef.OriginRpy ?? new double[3])}]");
                DebugLog($"URDF Axis: [{string.Join(", ", jointDef.Axis ?? new double[3])}]");
            }
            else
            {
                DebugLog($"WARNING: No URDF data - will use DH parameters for visualization");
            }
        }

        GeometryModel3D? geometry = null;

        // Try to load mesh from package
        if (!string.IsNullOrEmpty(meshPath) && !string.IsNullOrEmpty(packagePath))
        {
            // Normalize mesh path (convert forward slashes to backslashes on Windows)
            string normalizedMeshPath = meshPath.Replace('/', Path.DirectorySeparatorChar);
            string fullPath = Path.Combine(packagePath, normalizedMeshPath);

            DebugLog($"NormalizedMeshPath: {normalizedMeshPath}");
            DebugLog($"FullPath: {fullPath}");

            Log.Information("Attempting to load mesh - PackagePath: {PackagePath}, MeshPath: {MeshPath}, NormalizedMeshPath: {NormalizedMeshPath}, FullPath: {FullPath}",
                packagePath, meshPath, normalizedMeshPath, fullPath);

            if (File.Exists(fullPath))
            {
                DebugLog($"FILE EXISTS: Size = {new FileInfo(fullPath).Length} bytes");
                geometry = LoadStlGeometry(fullPath, GetLinkColor(jointIndex));
                link.StlPath = fullPath;
                if (geometry != null)
                {
                    DebugLog($"SUCCESS: LoadStlGeometry returned geometry");
                    Log.Information("SUCCESS: Loaded STL from package for {Name}: {Path}", name, fullPath);
                }
                else
                {
                    DebugLog($"FAILED: LoadStlGeometry returned NULL");
                    Log.Warning("FAILED: LoadStlGeometry returned null for {Name}: {Path}", name, fullPath);
                }
            }
            else
            {
                DebugLog($"FILE NOT FOUND at: {fullPath}");
                Log.Warning("FILE NOT FOUND: STL file does not exist at: {Path}", fullPath);
            }
        }
        else
        {
            DebugLog($"Skipping mesh load - meshPath or packagePath is empty/null");
            Log.Debug("Skipping mesh load - meshPath: {MeshPath}, packagePath: {PackagePath}",
                meshPath ?? "null", packagePath ?? "null");
        }

        // Fallback to placeholder
        if (geometry == null)
        {
            DebugLog($"Creating placeholder geometry for {name}");
            geometry = CreatePlaceholderGeometry(jointIndex);
            Log.Debug("Created placeholder geometry for {Name}", name);
        }

        link.Geometry = geometry;
        DebugLog($"=== End CreateLinkFromPackage ===\n");
        return link;
    }

    /// <summary>
    /// Create a robot link with geometry
    /// </summary>
    private RobotLink? CreateLink(int jointIndex, string name, DHParameterData? dhData, string modelsPath)
    {
        var link = new RobotLink
        {
            Name = name,
            JointIndex = jointIndex
        };

        // Set DH parameters
        if (dhData != null)
        {
            link.DH = new DHParameter
            {
                Joint = dhData.Joint,
                A = dhData.A,
                Alpha = dhData.Alpha,
                D = dhData.D,
                ThetaOffset = dhData.ThetaOffset
            };
        }

        // Try to load STL
        string stlPath = Path.Combine(modelsPath, $"{name.ToLower()}.stl");
        if (!File.Exists(stlPath))
        {
            stlPath = Path.Combine(modelsPath, $"link{jointIndex}.stl");
        }

        GeometryModel3D? geometry = null;

        if (File.Exists(stlPath))
        {
            geometry = LoadStlGeometry(stlPath, GetLinkColor(jointIndex));
            link.StlPath = stlPath;
            Log.Debug("Loaded STL for {Name}: {Path}", name, stlPath);
        }
        else
        {
            // Create placeholder geometry
            geometry = CreatePlaceholderGeometry(jointIndex);
            Log.Debug("Created placeholder geometry for {Name}", name);
        }

        link.Geometry = geometry;
        return link;
    }

    /// <summary>
    /// Load geometry from STL file
    /// </summary>
    private GeometryModel3D? LoadStlGeometry(string path, Color color)
    {
        try
        {
            DebugLog($">>> LoadStlGeometry START <<<");
            DebugLog($"Path: {path}");
            DebugLog($"Color: {color}");

            Log.Information("Attempting to load STL: {Path}", path);

            // Verify file exists
            if (!System.IO.File.Exists(path))
            {
                DebugLog($"ERROR: File does not exist!");
                Log.Warning("STL file not found: {Path}", path);
                return null;
            }

            var fileInfo = new System.IO.FileInfo(path);
            DebugLog($"File exists, size: {fileInfo.Length} bytes");
            Log.Information("STL file exists, size: {Size} bytes", fileInfo.Length);

            DebugLog($"Creating StLReader...");
            var reader = new StLReader();

            DebugLog($"Calling StLReader.Read()...");
            var model = reader.Read(path);

            if (model == null)
            {
                DebugLog($"ERROR: StLReader.Read() returned NULL!");
                Log.Warning("StLReader.Read returned null for: {Path}", path);
                return null;
            }

            DebugLog($"Model loaded successfully");
            DebugLog($"Model.Children.Count: {model.Children.Count}");
            Log.Information("Model loaded, children count: {Count}", model.Children.Count);

            if (model.Children.Count > 0 && model.Children[0] is GeometryModel3D geo)
            {
                DebugLog($"Found GeometryModel3D in Children[0]");

                // CRITICAL FIX: ROS-Industrial STL files use METERS, but our DH params use MILLIMETERS
                // Apply 1000x scale to convert meters → millimeters
                const double SCALE_M_TO_MM = 1000.0;
                geo.Transform = new ScaleTransform3D(SCALE_M_TO_MM, SCALE_M_TO_MM, SCALE_M_TO_MM);
                DebugLog($"Applied scale transform: {SCALE_M_TO_MM}x (meters → millimeters)");

                // Log bounds for debugging
                var bounds = geo.Bounds;
                DebugLog($"Geometry bounds: X=[{bounds.X:F1}, {bounds.X + bounds.SizeX:F1}], Y=[{bounds.Y:F1}, {bounds.Y + bounds.SizeY:F1}], Z=[{bounds.Z:F1}, {bounds.Z + bounds.SizeZ:F1}]");

                DebugLog($"Setting material color...");
                geo.Material = new DiffuseMaterial(new SolidColorBrush(color));
                geo.BackMaterial = new DiffuseMaterial(new SolidColorBrush(color));

                DebugLog($"SUCCESS: GeometryModel3D created from STL with scaling");
                Log.Information("Successfully created GeometryModel3D from STL: {Path}", path);
                DebugLog($"<<< LoadStlGeometry END (SUCCESS) <<<\n");
                return geo;
            }

            DebugLog($"ERROR: Model has no GeometryModel3D children");
            Log.Warning("Model has no geometry children: {Path}", path);
            DebugLog($"<<< LoadStlGeometry END (FAILED) <<<\n");
            return null;
        }
        catch (Exception ex)
        {
            DebugLog($"EXCEPTION in LoadStlGeometry:");
            DebugLog($"Type: {ex.GetType().Name}");
            DebugLog($"Message: {ex.Message}");
            DebugLog($"StackTrace: {ex.StackTrace}");
            Log.Error(ex, "Failed to load STL: {Path}", path);
            DebugLog($"<<< LoadStlGeometry END (EXCEPTION) <<<\n");
            return null;
        }
    }

    /// <summary>
    /// Debug logging to text file
    /// </summary>
    private static void DebugLog(string message)
    {
        try
        {
            string timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff");
            string logMessage = $"[{timestamp}] {message}\n";
            System.IO.File.AppendAllText(_debugLogPath, logMessage);
        }
        catch
        {
            // Ignore logging errors
        }
    }

    /// <summary>
    /// Create placeholder geometry when STL not available
    /// </summary>
    private GeometryModel3D CreatePlaceholderGeometry(int jointIndex)
    {
        var builder = new MeshBuilder();
        var color = GetLinkColor(jointIndex);

        switch (jointIndex)
        {
            case 0: // Base
                builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, 50), 100, 32);
                break;
            case 1: // Link 1 (rotation around Z)
                builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, 200), 60, 24);
                break;
            case 2: // Link 2 (shoulder)
                builder.AddBox(new Point3D(0, 0, 280), 60, 60, 560);
                break;
            case 3: // Link 3 (elbow)
                builder.AddBox(new Point3D(0, 0, 0), 50, 50, 50);
                break;
            case 4: // Link 4 (wrist 1)
                builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, 300), 40, 20);
                break;
            case 5: // Link 5 (wrist 2)
                builder.AddBox(new Point3D(0, 0, 0), 40, 40, 40);
                break;
            case 6: // Link 6 (flange)
                builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, 60), 30, 16);
                break;
            default:
                builder.AddBox(new Point3D(0, 0, 0), 30, 30, 30);
                break;
        }

        var mesh = builder.ToMesh();
        var material = new DiffuseMaterial(new SolidColorBrush(color));

        return new GeometryModel3D(mesh, material)
        {
            BackMaterial = material
        };
    }

    /// <summary>
    /// Get color for each link
    /// </summary>
    private Color GetLinkColor(int jointIndex)
    {
        return jointIndex switch
        {
            0 => Color.FromRgb(80, 80, 80),      // Base: Dark gray
            1 => Color.FromRgb(255, 140, 0),    // Link1: Orange
            2 => Color.FromRgb(255, 165, 0),    // Link2: Light orange
            3 => Color.FromRgb(255, 180, 0),    // Link3: Yellow-orange
            4 => Color.FromRgb(100, 149, 237),  // Link4: Cornflower blue
            5 => Color.FromRgb(65, 105, 225),   // Link5: Royal blue
            6 => Color.FromRgb(50, 50, 50),     // Link6: Dark gray (flange)
            _ => Color.FromRgb(128, 128, 128)   // Default: Gray
        };
    }

    /// <summary>
    /// Update a single joint angle (degrees)
    /// </summary>
    public void SetJointAngle(int jointIndex, double angleDegrees)
    {
        if (jointIndex < 1 || jointIndex > 6) return;
        _jointAngles[jointIndex - 1] = angleDegrees * Math.PI / 180.0;
        UpdateForwardKinematics();
    }

    /// <summary>
    /// Update all joint angles (degrees)
    /// </summary>
    public void SetAllJointAngles(double[] anglesDegrees)
    {
        if (anglesDegrees.Length < 6) return;
        for (int i = 0; i < 6; i++)
        {
            _jointAngles[i] = anglesDegrees[i] * Math.PI / 180.0;
        }
        UpdateForwardKinematics();
    }

    /// <summary>
    /// Update forward kinematics - recalculate all transforms
    /// </summary>
    public void UpdateForwardKinematics()
    {
        const double SCALE_M_TO_MM = 1000.0;
        Matrix3D parentTransform = Matrix3D.Identity;

        for (int i = 0; i < _links.Count; i++)
        {
            var link = _links[i];

            if (link.JointIndex == 0)
            {
                // Base link - just apply scale
                if (link.Geometry != null)
                {
                    var scaleTransform = new ScaleTransform3D(SCALE_M_TO_MM, SCALE_M_TO_MM, SCALE_M_TO_MM);
                    link.Geometry.Transform = scaleTransform;
                }
                parentTransform = Matrix3D.Identity;
                link.WorldTransform = Matrix3D.Identity;
            }
            else
            {
                // Joint links - use URDF origin if available, otherwise fall back to DH
                Matrix3D jointTransform;

                if (link.OriginXyz != null)
                {
                    // URDF approach: Use joint origin directly
                    jointTransform = CreateTransformFromURDF(
                        link.OriginXyz,
                        link.OriginRpy ?? new double[] { 0, 0, 0 },
                        link.Axis ?? new double[] { 0, 0, 1 },
                        _jointAngles[link.JointIndex - 1]
                    );
                }
                else
                {
                    // DH approach: Calculate from DH parameters
                    jointTransform = link.CalculateDHTransform(_jointAngles[link.JointIndex - 1]);
                }

                // Combine: Parent → Joint Transform
                var worldTransform = Matrix3D.Multiply(jointTransform, parentTransform);
                link.WorldTransform = worldTransform;

                // Apply: Scale → World Transform to geometry
                if (link.Geometry != null)
                {
                    var scaleTransform = new ScaleTransform3D(SCALE_M_TO_MM, SCALE_M_TO_MM, SCALE_M_TO_MM);
                    var transformGroup = new Transform3DGroup();
                    transformGroup.Children.Add(scaleTransform);
                    transformGroup.Children.Add(new MatrixTransform3D(worldTransform));
                    link.Geometry.Transform = transformGroup;
                }

                parentTransform = worldTransform;
            }
        }

        // Update TCP position (from last link)
        if (_links.Count > 0)
        {
            var lastLink = _links[^1];
            TcpPosition = lastLink.WorldTransform.Transform(new Point3D(0, 0, 0));
            TcpOrientation = lastLink.WorldTransform;
        }
    }

    /// <summary>
    /// Create transform from URDF joint origin
    /// </summary>
    private Matrix3D CreateTransformFromURDF(double[] xyz, double[] rpy, double[] axis, double jointAngle)
    {
        // 1. Translation from xyz
        var translation = Matrix3D.Identity;
        translation.Translate(new Vector3D(xyz[0], xyz[1], xyz[2]));

        // 2. Fixed rotation from rpy (roll-pitch-yaw)
        var rotation = CreateRotationFromRPY(rpy[0], rpy[1], rpy[2]);

        // 3. Joint rotation around axis
        var jointRotation = CreateRotationAroundAxis(axis, jointAngle);

        // Combine: Translation → Fixed Rotation → Joint Rotation
        return Matrix3D.Multiply(jointRotation, Matrix3D.Multiply(rotation, translation));
    }

    /// <summary>
    /// Create rotation matrix from Roll-Pitch-Yaw angles
    /// </summary>
    private Matrix3D CreateRotationFromRPY(double roll, double pitch, double yaw)
    {
        // Rz(yaw) * Ry(pitch) * Rx(roll)
        double cr = Math.Cos(roll);
        double sr = Math.Sin(roll);
        double cp = Math.Cos(pitch);
        double sp = Math.Sin(pitch);
        double cy = Math.Cos(yaw);
        double sy = Math.Sin(yaw);

        return new Matrix3D(
            cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr,  0,
            sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr,  0,
            -sp,      cp * sr,                 cp * cr,                 0,
            0,        0,                       0,                       1
        );
    }

    /// <summary>
    /// Create rotation matrix around arbitrary axis
    /// </summary>
    private Matrix3D CreateRotationAroundAxis(double[] axis, double angle)
    {
        // Rodrigues' rotation formula
        double x = axis[0], y = axis[1], z = axis[2];
        double len = Math.Sqrt(x * x + y * y + z * z);
        if (len < 1e-6) return Matrix3D.Identity;

        x /= len; y /= len; z /= len;

        double c = Math.Cos(angle);
        double s = Math.Sin(angle);
        double t = 1 - c;

        return new Matrix3D(
            t * x * x + c,      t * x * y - s * z,  t * x * z + s * y,  0,
            t * x * y + s * z,  t * y * y + c,      t * y * z - s * x,  0,
            t * x * z - s * y,  t * y * z + s * x,  t * z * z + c,      0,
            0,                  0,                  0,                  1
        );
    }

    /// <summary>
    /// Get TCP position as array [X, Y, Z, Rx, Ry, Rz]
    /// </summary>
    public double[] GetTcpPose()
    {
        // Extract position
        double x = TcpPosition.X;
        double y = TcpPosition.Y;
        double z = TcpPosition.Z;

        // Extract Euler angles (simplified - ZYX convention)
        double rx = 0, ry = 0, rz = 0;

        if (TcpOrientation.IsAffine)
        {
            rz = Math.Atan2(TcpOrientation.M21, TcpOrientation.M11) * 180 / Math.PI;
            ry = Math.Atan2(-TcpOrientation.M31,
                Math.Sqrt(TcpOrientation.M32 * TcpOrientation.M32 + TcpOrientation.M33 * TcpOrientation.M33)) * 180 / Math.PI;
            rx = Math.Atan2(TcpOrientation.M32, TcpOrientation.M33) * 180 / Math.PI;
        }

        return new[] { x, y, z, rx, ry, rz };
    }
}

/// <summary>
/// Robot config data from Core (matches JSON structure)
/// </summary>
public class RobotConfigData
{
    public string Name { get; set; } = "Robot";
    public string Model { get; set; } = "";
    public List<DHParameterData> DHParameters { get; set; } = new();
    public double[] HomePosition { get; set; } = new double[6];
}

public class DHParameterData
{
    public int Joint { get; set; }
    public double A { get; set; }
    public double Alpha { get; set; }
    public double D { get; set; }
    public double ThetaOffset { get; set; }
}

/// <summary>
/// Robot package data from Core (matches JSON structure from LOAD_ROBOT_PACKAGE)
/// </summary>
public class RobotPackageData
{
    public string Name { get; set; } = "";
    public string Id { get; set; } = "";
    public string Manufacturer { get; set; } = "";
    public string ModelType { get; set; } = "";
    public double PayloadKg { get; set; }
    public double ReachMm { get; set; }
    public string DhConvention { get; set; } = "modified_dh";
    public List<JointDefinitionData> Joints { get; set; } = new();
    public string BaseMesh { get; set; } = "";
    public double[] BaseOrigin { get; set; } = new double[6];
    public double[] FlangeOffset { get; set; } = new double[3];
    public double[] HomePosition { get; set; } = Array.Empty<double>();
    public string PackagePath { get; set; } = "";
}

public class JointDefinitionData
{
    public string Name { get; set; } = "";
    public string Type { get; set; } = "revolute";
    public double DhA { get; set; }
    public double DhAlpha { get; set; }
    public double DhD { get; set; }
    public double DhThetaOffset { get; set; }

    // URDF origin data (optional - for visualization)
    public double[]? OriginXyz { get; set; }
    public double[]? OriginRpy { get; set; }
    public double[]? Axis { get; set; }

    public double LimitMin { get; set; }
    public double LimitMax { get; set; }
    public double VelocityMax { get; set; }
    public double AccelerationMax { get; set; }
    public JointMeshData? Mesh { get; set; }
}

public class JointMeshData
{
    public string VisualMesh { get; set; } = "";
    public string CollisionMesh { get; set; } = "";
    public double[] Origin { get; set; } = new double[3];
}
