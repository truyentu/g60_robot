using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RobotController.UI.Models;
using Serilog;

namespace RobotController.UI.Services;

/// <summary>
/// Interface for robot package generator
/// </summary>
public interface IRobotPackageGenerator
{
    /// <summary>Generate robot package from parsed URDF</summary>
    Task<PackageGenerateResult> GeneratePackageAsync(
        UrdfRobot robot,
        string robotId,
        string manufacturer,
        string outputPath);

    /// <summary>Get default output path for new packages</summary>
    string GetDefaultOutputPath();
}

/// <summary>
/// Result of package generation
/// </summary>
public class PackageGenerateResult
{
    public bool Success { get; set; }
    public string? ErrorMessage { get; set; }
    public string? OutputPath { get; set; }
    public int MeshesCopied { get; set; }
    public List<string> Warnings { get; set; } = new();
}

/// <summary>
/// Generates robot.yaml and copies mesh files from parsed URDF
/// </summary>
public class RobotPackageGenerator : IRobotPackageGenerator
{
    /// <summary>
    /// Get default output path for robot packages
    /// Output to src/config/robots/ so both Core and UI can access after rebuild
    /// </summary>
    public string GetDefaultOutputPath()
    {
        // Navigate from bin/Debug/net8.0-windows to src/config/robots
        var basePath = AppDomain.CurrentDomain.BaseDirectory;

        // Try to find src/config/robots relative to executable
        // bin/Debug/net8.0-windows -> src/ui/RobotController.UI/bin/Debug/net8.0-windows
        // We need to go up to src/config/robots
        var srcConfigPath = Path.GetFullPath(Path.Combine(basePath, "..", "..", "..", "..", "..", "config", "robots"));

        if (Directory.Exists(Path.GetDirectoryName(srcConfigPath)))
        {
            Directory.CreateDirectory(srcConfigPath);
            return srcConfigPath;
        }

        // Fallback to local config folder
        var localPath = Path.Combine(basePath, "config", "robots");
        Directory.CreateDirectory(localPath);
        return localPath;
    }

    /// <summary>
    /// Generate robot package from parsed URDF
    /// </summary>
    public async Task<PackageGenerateResult> GeneratePackageAsync(
        UrdfRobot robot,
        string robotId,
        string manufacturer,
        string outputPath)
    {
        var result = new PackageGenerateResult();

        try
        {
            Log.Information("Generating robot package: {Id} at {Path}", robotId, outputPath);

            // Create package folder
            var packagePath = Path.Combine(outputPath, robotId);
            var meshesPath = Path.Combine(packagePath, "meshes", "visual");

            Directory.CreateDirectory(packagePath);
            Directory.CreateDirectory(meshesPath);

            // Copy mesh files and build mesh mapping
            var meshMapping = await CopyMeshFilesAsync(robot, meshesPath);
            result.MeshesCopied = meshMapping.Count;

            if (meshMapping.Count == 0)
            {
                result.Warnings.Add("No mesh files were copied. Robot will use placeholder geometry.");
            }

            // Generate robot.yaml
            var yamlContent = GenerateRobotYaml(robot, robotId, manufacturer, meshMapping);
            var yamlPath = Path.Combine(packagePath, "robot.yaml");
            await File.WriteAllTextAsync(yamlPath, yamlContent, Encoding.UTF8);

            Log.Information("Generated robot.yaml at {Path}", yamlPath);

            result.Success = true;
            result.OutputPath = packagePath;
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Error generating robot package");
            result.Success = false;
            result.ErrorMessage = ex.Message;
        }

        return result;
    }

    /// <summary>
    /// Copy mesh files from source to output
    /// </summary>
    private async Task<Dictionary<string, string>> CopyMeshFilesAsync(UrdfRobot robot, string outputPath)
    {
        var meshMapping = new Dictionary<string, string>(); // link name -> relative mesh path

        foreach (var link in robot.Links)
        {
            if (!link.HasVisualMesh || string.IsNullOrEmpty(link.VisualMeshPath))
                continue;

            if (!File.Exists(link.VisualMeshPath))
            {
                Log.Warning("Mesh file not found: {Path}", link.VisualMeshPath);
                continue;
            }

            try
            {
                var sourceFileName = Path.GetFileName(link.VisualMeshPath);
                var destPath = Path.Combine(outputPath, sourceFileName);

                // Copy file
                await Task.Run(() => File.Copy(link.VisualMeshPath, destPath, overwrite: true));

                // Store relative path for robot.yaml
                meshMapping[link.Name] = $"meshes/visual/{sourceFileName}";

                Log.Debug("Copied mesh: {Source} -> {Dest}", link.VisualMeshPath, destPath);
            }
            catch (Exception ex)
            {
                Log.Warning("Failed to copy mesh {Path}: {Error}", link.VisualMeshPath, ex.Message);
            }
        }

        return meshMapping;
    }

    /// <summary>
    /// Generate robot.yaml content
    /// </summary>
    private string GenerateRobotYaml(
        UrdfRobot robot,
        string robotId,
        string manufacturer,
        Dictionary<string, string> meshMapping)
    {
        var sb = new StringBuilder();

        // Header comment
        sb.AppendLine($"# Robot Package: {robot.Name}");
        sb.AppendLine($"# Generated from URDF: {Path.GetFileName(robot.SourceFile)}");
        sb.AppendLine($"# Date: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
        sb.AppendLine($"# Note: DH parameters are placeholders - fill in from robot datasheet for kinematics");
        sb.AppendLine();

        // Metadata
        sb.AppendLine($"name: \"{robot.Name}\"");
        sb.AppendLine($"manufacturer: \"{manufacturer}\"");
        sb.AppendLine($"type: \"6-axis-industrial\"");
        sb.AppendLine($"payload_kg: 0  # TODO: Fill from datasheet");
        sb.AppendLine($"reach_mm: 0    # TODO: Fill from datasheet");
        sb.AppendLine();

        // Kinematics section
        sb.AppendLine("kinematics:");
        sb.AppendLine("  convention: \"modified_dh\"");
        sb.AppendLine("  joints:");

        // Get active joints (revolute/prismatic) in order
        var activeJoints = robot.ActiveJoints;
        int jointIndex = 1;

        foreach (var joint in activeJoints)
        {
            sb.AppendLine($"    # Joint {jointIndex}: {joint.Name}");
            sb.AppendLine($"    - name: \"A{jointIndex}\"");
            sb.AppendLine($"      type: \"{joint.Type}\"");

            // DH parameters (placeholders)
            sb.AppendLine("      # DH parameters (for FK/IK - fill from datasheet)");
            sb.AppendLine("      dh:");
            sb.AppendLine("        a: 0");
            sb.AppendLine("        alpha: 0");
            sb.AppendLine("        d: 0");
            sb.AppendLine("        theta_offset: 0");

            // URDF origin (for visualization)
            sb.AppendLine("      # URDF origin (for visualization - from import)");
            sb.AppendLine("      origin:");
            sb.AppendLine($"        xyz: [{joint.OriginXyz[0]:F3}, {joint.OriginXyz[1]:F3}, {joint.OriginXyz[2]:F3}]");
            sb.AppendLine($"        rpy: [{joint.OriginRpy[0]:F6}, {joint.OriginRpy[1]:F6}, {joint.OriginRpy[2]:F6}]");

            // Axis
            sb.AppendLine($"      axis: [{joint.Axis[0]:F0}, {joint.Axis[1]:F0}, {joint.Axis[2]:F0}]");

            // Limits
            sb.AppendLine("      limits:");
            sb.AppendLine($"        min: {joint.LimitLower:F1}");
            sb.AppendLine($"        max: {joint.LimitUpper:F1}");
            sb.AppendLine($"        vel_max: {joint.VelocityMax:F1}");
            sb.AppendLine($"        accel_max: 720");

            // Mesh (find mesh for child link)
            var childLink = joint.ChildLink;
            if (meshMapping.TryGetValue(childLink, out var meshPath))
            {
                sb.AppendLine("      mesh:");
                sb.AppendLine($"        visual: \"{meshPath}\"");
            }
            else
            {
                sb.AppendLine("      # mesh: (not found)");
            }

            sb.AppendLine();
            jointIndex++;
        }

        // Home position
        sb.AppendLine("home_position: [0, 0, 0, 0, 0, 0]");
        sb.AppendLine();

        // Base mesh
        sb.AppendLine("base:");
        var baseLink = robot.Links.FirstOrDefault(l =>
            l.Name.Contains("base", StringComparison.OrdinalIgnoreCase));
        if (baseLink != null && meshMapping.TryGetValue(baseLink.Name, out var baseMeshPath))
        {
            sb.AppendLine($"  mesh: \"{baseMeshPath}\"");
        }
        else
        {
            sb.AppendLine("  mesh: \"\"  # TODO: Add base mesh if available");
        }
        sb.AppendLine("  origin:");
        sb.AppendLine("    x: 0");
        sb.AppendLine("    y: 0");
        sb.AppendLine("    z: 0");
        sb.AppendLine("    rx: 0");
        sb.AppendLine("    ry: 0");
        sb.AppendLine("    rz: 0");
        sb.AppendLine();

        // Flange
        sb.AppendLine("flange:");
        sb.AppendLine("  offset:");
        sb.AppendLine("    x: 0");
        sb.AppendLine("    y: 0");
        sb.AppendLine("    z: 0");

        return sb.ToString();
    }
}
