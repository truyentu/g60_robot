using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using RobotController.UI.Models;
using Serilog;

namespace RobotController.UI.Services;

/// <summary>
/// Interface for URDF import service
/// </summary>
public interface IUrdfImportService
{
    /// <summary>Scan folder for URDF/XACRO files</summary>
    List<UrdfFileInfo> ScanForUrdfFiles(string folderPath);

    /// <summary>Parse URDF/XACRO file</summary>
    UrdfParseResult ParseUrdf(string filePath);
}

/// <summary>
/// Info about a discovered URDF/XACRO file
/// </summary>
public class UrdfFileInfo
{
    public string FullPath { get; set; } = "";
    public string FileName { get; set; } = "";
    public string RelativePath { get; set; } = "";
    public bool IsMacro { get; set; }
    public bool IsXacro { get; set; }
}

/// <summary>
/// Service to parse URDF/XACRO files from ROS packages
/// </summary>
public class UrdfImportService : IUrdfImportService
{
    private const double MetersToMm = 1000.0;
    private const double RadToDeg = 180.0 / Math.PI;

    /// <summary>
    /// Scan folder for URDF/XACRO files
    /// </summary>
    public List<UrdfFileInfo> ScanForUrdfFiles(string folderPath)
    {
        var result = new List<UrdfFileInfo>();

        if (!Directory.Exists(folderPath))
        {
            Log.Warning("Folder does not exist: {Path}", folderPath);
            return result;
        }

        try
        {
            // Find all .urdf and .xacro files
            var urdfFiles = Directory.GetFiles(folderPath, "*.urdf", SearchOption.AllDirectories);
            var xacroFiles = Directory.GetFiles(folderPath, "*.xacro", SearchOption.AllDirectories);

            foreach (var file in urdfFiles.Concat(xacroFiles))
            {
                var fileName = Path.GetFileName(file);
                var info = new UrdfFileInfo
                {
                    FullPath = file,
                    FileName = fileName,
                    RelativePath = Path.GetRelativePath(folderPath, file),
                    IsXacro = file.EndsWith(".xacro", StringComparison.OrdinalIgnoreCase),
                    IsMacro = fileName.Contains("macro", StringComparison.OrdinalIgnoreCase)
                };
                result.Add(info);
            }

            // Sort: prioritize macro files (they usually contain full robot definition)
            result = result
                .OrderByDescending(f => f.IsMacro)
                .ThenBy(f => f.RelativePath)
                .ToList();

            Log.Information("Found {Count} URDF/XACRO files in {Path}", result.Count, folderPath);
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Error scanning for URDF files in {Path}", folderPath);
        }

        return result;
    }

    /// <summary>
    /// Parse URDF/XACRO file
    /// </summary>
    public UrdfParseResult ParseUrdf(string filePath)
    {
        var result = new UrdfParseResult();

        if (!File.Exists(filePath))
        {
            result.Success = false;
            result.ErrorMessage = $"File not found: {filePath}";
            return result;
        }

        try
        {
            var sourceFolder = Path.GetDirectoryName(filePath) ?? "";
            // Go up to package root (parent of urdf folder)
            var packageRoot = Directory.GetParent(sourceFolder)?.FullName ?? sourceFolder;

            Log.Information("Parsing URDF: {Path}", filePath);
            Log.Information("Package root: {Root}", packageRoot);

            var doc = XDocument.Load(filePath);
            var robot = new UrdfRobot
            {
                SourceFile = filePath,
                SourcePath = packageRoot
            };

            // Get robot name
            var robotElement = doc.Root;
            if (robotElement == null)
            {
                result.Success = false;
                result.ErrorMessage = "Invalid XML: no root element";
                return result;
            }

            // Handle both <robot> and xacro <robot> with macros
            if (robotElement.Name.LocalName == "robot")
            {
                robot.Name = robotElement.Attribute("name")?.Value ?? "Unknown";
            }

            // Check for xacro:include (we don't fully support this)
            var includes = doc.Descendants()
                .Where(e => e.Name.LocalName == "include" ||
                           (e.Name.NamespaceName.Contains("xacro") && e.Name.LocalName == "include"))
                .ToList();

            if (includes.Any())
            {
                result.Warnings.Add($"Found {includes.Count} xacro:include directive(s). " +
                    "Included files will not be parsed. For best results, use a fully expanded URDF or the main macro file.");
            }

            // Parse all links
            var linkElements = doc.Descendants().Where(e => e.Name.LocalName == "link").ToList();
            foreach (var linkEl in linkElements)
            {
                var link = ParseLink(linkEl, packageRoot);
                if (link != null)
                {
                    robot.Links.Add(link);

                    // Check mesh files
                    if (link.HasVisualMesh)
                    {
                        if (File.Exists(link.VisualMeshPath))
                        {
                            result.MeshFilesFound.Add(link.VisualMeshPath!);
                        }
                        else
                        {
                            result.MeshFilesMissing.Add(link.VisualMeshPath!);
                        }
                    }
                }
            }

            // Parse all joints
            var jointElements = doc.Descendants().Where(e => e.Name.LocalName == "joint").ToList();
            foreach (var jointEl in jointElements)
            {
                var joint = ParseJoint(jointEl);
                if (joint != null)
                {
                    robot.Joints.Add(joint);
                }
            }

            // Build info messages
            result.InfoMessages.Add($"Robot name: {robot.Name}");
            result.InfoMessages.Add($"Total links: {robot.Links.Count}");
            result.InfoMessages.Add($"Total joints: {robot.Joints.Count}");
            result.InfoMessages.Add($"Active joints (DOF): {robot.DOF}");
            result.InfoMessages.Add($"Meshes found: {result.MeshFilesFound.Count}");

            if (result.MeshFilesMissing.Count > 0)
            {
                result.Warnings.Add($"{result.MeshFilesMissing.Count} mesh file(s) not found");
            }

            // Validation
            if (robot.DOF == 0)
            {
                result.Warnings.Add("No movable joints found. This may be a partial URDF or include file.");
            }

            if (robot.DOF > 0 && robot.DOF != 6)
            {
                result.Warnings.Add($"Robot has {robot.DOF} DOF (expected 6 for standard industrial robot)");
            }

            result.Robot = robot;
            result.Success = true;

            Log.Information("Successfully parsed URDF: {Name} with {DOF} DOF", robot.Name, robot.DOF);
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Error parsing URDF: {Path}", filePath);
            result.Success = false;
            result.ErrorMessage = $"Parse error: {ex.Message}";
        }

        return result;
    }

    /// <summary>
    /// Parse a link element
    /// </summary>
    private UrdfLink? ParseLink(XElement linkEl, string packageRoot)
    {
        var name = linkEl.Attribute("name")?.Value;
        if (string.IsNullOrEmpty(name))
            return null;

        // Handle xacro ${prefix} in names
        if (name.Contains("${"))
        {
            name = name.Replace("${prefix}", "").Trim();
        }

        var link = new UrdfLink { Name = name };

        // Find visual element
        var visual = linkEl.Element("visual");
        if (visual != null)
        {
            // Parse origin
            var origin = visual.Element("origin");
            if (origin != null)
            {
                link.VisualOrigin = ParseXyz(origin.Attribute("xyz")?.Value, MetersToMm);
                link.VisualRpy = ParseXyz(origin.Attribute("rpy")?.Value, 1.0); // Keep radians
            }

            // Parse mesh
            var geometry = visual.Element("geometry");
            var mesh = geometry?.Element("mesh");
            var filename = mesh?.Attribute("filename")?.Value;

            if (!string.IsNullOrEmpty(filename))
            {
                link.VisualMeshPath = ResolveMeshPath(filename, packageRoot);
            }
        }

        // Find collision mesh (optional)
        var collision = linkEl.Element("collision");
        if (collision != null)
        {
            var geometry = collision.Element("geometry");
            var mesh = geometry?.Element("mesh");
            var filename = mesh?.Attribute("filename")?.Value;

            if (!string.IsNullOrEmpty(filename))
            {
                link.CollisionMeshPath = ResolveMeshPath(filename, packageRoot);
            }
        }

        return link;
    }

    /// <summary>
    /// Parse a joint element
    /// </summary>
    private UrdfJoint? ParseJoint(XElement jointEl)
    {
        var name = jointEl.Attribute("name")?.Value;
        var type = jointEl.Attribute("type")?.Value;

        if (string.IsNullOrEmpty(name))
            return null;

        // Handle xacro ${prefix}
        if (name.Contains("${"))
        {
            name = name.Replace("${prefix}", "").Trim();
        }

        var joint = new UrdfJoint
        {
            Name = name,
            Type = type ?? "fixed"
        };

        // Parse parent/child
        var parent = jointEl.Element("parent");
        var child = jointEl.Element("child");
        joint.ParentLink = parent?.Attribute("link")?.Value?.Replace("${prefix}", "") ?? "";
        joint.ChildLink = child?.Attribute("link")?.Value?.Replace("${prefix}", "") ?? "";

        // Parse origin
        var origin = jointEl.Element("origin");
        if (origin != null)
        {
            joint.OriginXyz = ParseXyz(origin.Attribute("xyz")?.Value, MetersToMm);
            joint.OriginRpy = ParseXyz(origin.Attribute("rpy")?.Value, 1.0); // Keep radians
        }

        // Parse axis
        var axis = jointEl.Element("axis");
        if (axis != null)
        {
            joint.Axis = ParseXyz(axis.Attribute("xyz")?.Value, 1.0);
        }

        // Parse limits
        var limit = jointEl.Element("limit");
        if (limit != null)
        {
            // URDF limits are in radians for revolute joints
            var lower = ParseDouble(limit.Attribute("lower")?.Value, -Math.PI);
            var upper = ParseDouble(limit.Attribute("upper")?.Value, Math.PI);
            var velocity = ParseDouble(limit.Attribute("velocity")?.Value, Math.PI);
            var effort = ParseDouble(limit.Attribute("effort")?.Value, 0);

            if (joint.Type == "revolute" || joint.Type == "continuous")
            {
                joint.LimitLower = lower * RadToDeg;
                joint.LimitUpper = upper * RadToDeg;
                joint.VelocityMax = velocity * RadToDeg;
            }
            else
            {
                // Prismatic: meters to mm
                joint.LimitLower = lower * MetersToMm;
                joint.LimitUpper = upper * MetersToMm;
                joint.VelocityMax = velocity * MetersToMm;
            }
            joint.EffortMax = effort;
        }

        // Handle xacro expressions like ${radians(180)}
        // For now, just use the raw value (xacro expressions won't be evaluated)

        return joint;
    }

    /// <summary>
    /// Resolve package:// path to absolute path
    /// </summary>
    private string ResolveMeshPath(string packagePath, string packageRoot)
    {
        // Example: "package://abb_irb7600_support/meshes/irb7600/visual/link_1.stl"
        // Should resolve to: "{packageRoot}/meshes/irb7600/visual/link_1.stl"

        if (packagePath.StartsWith("package://"))
        {
            // Remove "package://package_name/" prefix
            var parts = packagePath.Substring("package://".Length).Split('/', 2);
            if (parts.Length == 2)
            {
                var relativePath = parts[1].Replace('/', Path.DirectorySeparatorChar);
                return Path.Combine(packageRoot, relativePath);
            }
        }

        // If not a package:// path, treat as relative
        return Path.Combine(packageRoot, packagePath.Replace('/', Path.DirectorySeparatorChar));
    }

    /// <summary>
    /// Parse "x y z" string to double array
    /// </summary>
    private double[] ParseXyz(string? value, double scale)
    {
        if (string.IsNullOrWhiteSpace(value))
            return new double[] { 0, 0, 0 };

        try
        {
            var parts = value.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length >= 3)
            {
                return new double[]
                {
                    ParseDouble(parts[0], 0) * scale,
                    ParseDouble(parts[1], 0) * scale,
                    ParseDouble(parts[2], 0) * scale
                };
            }
        }
        catch (Exception ex)
        {
            Log.Warning("Failed to parse xyz value '{Value}': {Error}", value, ex.Message);
        }

        return new double[] { 0, 0, 0 };
    }

    /// <summary>
    /// Parse double value, handling xacro expressions
    /// </summary>
    private double ParseDouble(string? value, double defaultValue)
    {
        if (string.IsNullOrWhiteSpace(value))
            return defaultValue;

        // Handle xacro expressions like ${radians(180)} or -${radians(60)}
        if (value.Contains("${"))
        {
            // Try to extract number from radians() expression
            var match = System.Text.RegularExpressions.Regex.Match(value, @"radians\((-?\d+\.?\d*)\)");
            if (match.Success && double.TryParse(match.Groups[1].Value, out var degrees))
            {
                var radians = degrees * Math.PI / 180.0;
                // Check if negative
                if (value.StartsWith("-"))
                    radians = -Math.Abs(radians);
                return radians;
            }

            Log.Debug("Cannot evaluate xacro expression: {Value}, using default", value);
            return defaultValue;
        }

        if (double.TryParse(value, System.Globalization.NumberStyles.Float,
            System.Globalization.CultureInfo.InvariantCulture, out var result))
        {
            return result;
        }

        return defaultValue;
    }
}
