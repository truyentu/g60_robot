using System.Collections.ObjectModel;
using System.IO;
using System.Threading;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Microsoft.Win32;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using Serilog;

namespace RobotController.UI.ViewModels.Pages;

/// <summary>
/// ViewModel for URDF Import Wizard
/// Allows users to import robot packages from URDF/xacro files
/// </summary>
public partial class UrdfImportViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;

    [ObservableProperty]
    private string _urdfFilePath = "";

    [ObservableProperty]
    private string _urdfContent = "";

    [ObservableProperty]
    private string _robotName = "";

    [ObservableProperty]
    private string _manufacturer = "";

    [ObservableProperty]
    private string _outputDirectory = "";

    [ObservableProperty]
    private string _meshSourceDirectory = "";

    [ObservableProperty]
    private int _meshesCopied;

    [ObservableProperty]
    private string _statusMessage = "Select a URDF or xacro file to import";

    [ObservableProperty]
    private bool _isPackageReady;

    [ObservableProperty]
    private bool _isLoading;

    [ObservableProperty]
    private bool _isParsed;

    [ObservableProperty]
    private string _generatedYaml = "";

    [ObservableProperty]
    private ObservableCollection<UrdfJointInfo> _parsedJoints = new();

    public UrdfImportViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;

        // Default output directory
        var configDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "config", "robots");
        OutputDirectory = configDir;
    }

    [RelayCommand]
    private void BrowseUrdfFile()
    {
        var dialog = new OpenFileDialog
        {
            Title = "Select URDF or Xacro file",
            Filter = "URDF/Xacro files (*.urdf;*.xacro)|*.urdf;*.xacro|All files (*.*)|*.*",
            CheckFileExists = true
        };

        if (dialog.ShowDialog() == true)
        {
            UrdfFilePath = dialog.FileName;

            // Auto-fill robot name from filename
            var fileName = Path.GetFileNameWithoutExtension(dialog.FileName);
            if (string.IsNullOrEmpty(RobotName))
            {
                // Try to extract robot name from filename like "kr10r1420_macro.xacro"
                var nameParts = fileName.Replace("_macro", "").Replace("_", " ").ToUpper();
                RobotName = nameParts;
            }

            // Try to guess manufacturer
            if (string.IsNullOrEmpty(Manufacturer))
            {
                var lowerName = fileName.ToLower();
                if (lowerName.Contains("kuka") || lowerName.StartsWith("kr"))
                    Manufacturer = "KUKA";
                else if (lowerName.Contains("abb") || lowerName.StartsWith("irb"))
                    Manufacturer = "ABB";
                else if (lowerName.Contains("fanuc"))
                    Manufacturer = "Fanuc";
                else if (lowerName.Contains("ur") || lowerName.Contains("universal"))
                    Manufacturer = "Universal Robots";
            }

            StatusMessage = $"Selected: {Path.GetFileName(dialog.FileName)}";
        }
    }

    [RelayCommand]
    private void BrowseOutputDirectory()
    {
        var dialog = new OpenFolderDialog
        {
            Title = "Select output directory for robot package"
        };

        if (dialog.ShowDialog() == true)
        {
            OutputDirectory = dialog.FolderName;
        }
    }

    [RelayCommand]
    private void BrowseMeshDirectory()
    {
        var dialog = new OpenFolderDialog
        {
            Title = "Select folder containing STL mesh files"
        };

        // Try to start from URDF file location
        if (!string.IsNullOrEmpty(UrdfFilePath))
        {
            var urdfDir = Path.GetDirectoryName(UrdfFilePath);
            if (!string.IsNullOrEmpty(urdfDir))
            {
                // Look for common mesh folder patterns
                var meshesVisual = Path.Combine(urdfDir, "..", "meshes", "visual");
                var meshes = Path.Combine(urdfDir, "..", "meshes");

                if (Directory.Exists(meshesVisual))
                    dialog.InitialDirectory = Path.GetFullPath(meshesVisual);
                else if (Directory.Exists(meshes))
                    dialog.InitialDirectory = Path.GetFullPath(meshes);
                else
                    dialog.InitialDirectory = urdfDir;
            }
        }

        if (dialog.ShowDialog() == true)
        {
            MeshSourceDirectory = dialog.FolderName;

            // Count STL files
            var stlFiles = Directory.GetFiles(dialog.FolderName, "*.stl", SearchOption.AllDirectories);
            StatusMessage = $"Found {stlFiles.Length} STL files in mesh folder";
        }
    }

    [RelayCommand]
    private async Task ParseUrdfAsync()
    {
        if (string.IsNullOrEmpty(UrdfFilePath))
        {
            StatusMessage = "Please select a URDF file first";
            return;
        }

        try
        {
            IsLoading = true;
            StatusMessage = "Parsing URDF...";
            ParsedJoints.Clear();

            var response = await _ipcClient.ParseUrdfAsync(UrdfFilePath, isFilePath: true);

            if (response != null && response.Success && response.Model != null)
            {
                IsParsed = true;

                // Populate parsed joints
                foreach (var joint in response.Model.Joints)
                {
                    if (joint.Type == "fixed") continue; // Skip fixed joints

                    ParsedJoints.Add(new UrdfJointInfo
                    {
                        Name = joint.Name,
                        Type = joint.Type,
                        OriginX = joint.OriginXyz[0],
                        OriginY = joint.OriginXyz[1],
                        OriginZ = joint.OriginXyz[2],
                        AxisX = joint.Axis[0],
                        AxisY = joint.Axis[1],
                        AxisZ = joint.Axis[2],
                        LimitMin = joint.LimitLowerDeg,
                        LimitMax = joint.LimitUpperDeg
                    });
                }

                StatusMessage = $"Parsed successfully: {ParsedJoints.Count} joints found";
                Log.Information("URDF parsed: {Count} joints", ParsedJoints.Count);
            }
            else
            {
                IsParsed = false;
                StatusMessage = $"Parse failed: {response?.Error ?? "Unknown error"}";
                Log.Warning("URDF parse failed: {Error}", response?.Error);
            }
        }
        catch (Exception ex)
        {
            IsParsed = false;
            StatusMessage = $"Error: {ex.Message}";
            Log.Error(ex, "Failed to parse URDF");
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task GenerateYamlAsync()
    {
        if (string.IsNullOrEmpty(UrdfFilePath))
        {
            StatusMessage = "Please select a URDF file first";
            return;
        }

        if (string.IsNullOrEmpty(RobotName))
        {
            StatusMessage = "Please enter a robot name";
            return;
        }

        try
        {
            IsLoading = true;
            StatusMessage = "Generating robot.yaml...";

            // Create output directory based on robot name
            var robotId = RobotName.ToLower().Replace(" ", "_").Replace("-", "_");
            var robotDir = Path.Combine(OutputDirectory, robotId);
            var yamlPath = Path.Combine(robotDir, "robot.yaml");

            var response = await _ipcClient.GenerateRobotYamlAsync(
                UrdfFilePath,
                isFilePath: true,
                RobotName,
                Manufacturer,
                yamlPath);

            if (response != null && response.Success)
            {
                GeneratedYaml = response.YamlContent;

                // Copy STL files if mesh source directory is specified
                if (!string.IsNullOrEmpty(MeshSourceDirectory) && Directory.Exists(MeshSourceDirectory))
                {
                    StatusMessage = "Copying STL mesh files...";
                    MeshesCopied = await CopyMeshFilesAsync(robotDir);
                }

                if (!string.IsNullOrEmpty(response.SavedPath))
                {
                    var msg = $"Saved to: {response.SavedPath}";
                    if (MeshesCopied > 0)
                        msg += $" | {MeshesCopied} meshes copied";
                    StatusMessage = msg;
                    IsPackageReady = true;
                    Log.Information("robot.yaml saved to: {Path}, meshes copied: {Count}",
                        response.SavedPath, MeshesCopied);
                }
                else
                {
                    StatusMessage = "YAML generated (not saved)";
                }
            }
            else
            {
                StatusMessage = $"Generation failed: {response?.Error ?? "Unknown error"}";
                Log.Warning("YAML generation failed: {Error}", response?.Error);
            }
        }
        catch (Exception ex)
        {
            StatusMessage = $"Error: {ex.Message}";
            Log.Error(ex, "Failed to generate robot.yaml");
        }
        finally
        {
            IsLoading = false;
        }
    }

    private async Task<int> CopyMeshFilesAsync(string robotDir)
    {
        int copied = 0;

        try
        {
            // Create meshes/visual folder
            var meshOutputDir = Path.Combine(robotDir, "meshes", "visual");
            Directory.CreateDirectory(meshOutputDir);

            // Find all STL files in source directory (including subdirectories)
            var stlFiles = Directory.GetFiles(MeshSourceDirectory, "*.stl", SearchOption.AllDirectories);

            foreach (var srcFile in stlFiles)
            {
                var fileName = Path.GetFileName(srcFile);
                var destFile = Path.Combine(meshOutputDir, fileName);

                await Task.Run(() => File.Copy(srcFile, destFile, overwrite: true));
                copied++;

                Log.Debug("Copied mesh: {Source} -> {Dest}", srcFile, destFile);
            }

            Log.Information("Copied {Count} mesh files to {Dir}", copied, meshOutputDir);
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Error copying mesh files");
        }

        return copied;
    }

    [RelayCommand]
    private void CopyYamlToClipboard()
    {
        if (!string.IsNullOrEmpty(GeneratedYaml))
        {
            System.Windows.Clipboard.SetText(GeneratedYaml);
            StatusMessage = "YAML copied to clipboard";
        }
    }

    [RelayCommand]
    private async Task CopyToCoreAndRefreshAsync()
    {
        if (string.IsNullOrEmpty(RobotName))
        {
            StatusMessage = "No package to copy";
            return;
        }

        try
        {
            IsLoading = true;
            var robotId = RobotName.ToLower().Replace(" ", "_").Replace("-", "_");
            var sourceDir = Path.Combine(OutputDirectory, robotId);

            Log.Information("CopyToCoreAndRefresh: robotId={Id}, sourceDir={Source}", robotId, sourceDir);

            if (!Directory.Exists(sourceDir))
            {
                StatusMessage = "Package folder not found. Generate first.";
                Log.Warning("Source directory does not exist: {Dir}", sourceDir);
                return;
            }

            // Check source has files
            var sourceFiles = Directory.GetFiles(sourceDir, "*", SearchOption.AllDirectories);
            Log.Information("Source directory has {Count} files", sourceFiles.Length);
            if (sourceFiles.Length == 0)
            {
                StatusMessage = "Source folder is empty. Generate package first.";
                return;
            }

            var baseDir = AppDomain.CurrentDomain.BaseDirectory;
            int copiedLocations = 0;

            // === LOCATION 1: Core build folder ===
            var coreConfigPath = Path.GetFullPath(Path.Combine(
                baseDir, "..", "..", "..", "..", "..", "..", "src", "core", "build", "config", "robots"));

            StatusMessage = "Copying to Core location...";
            Log.Information("Copying package to Core: {Path}", coreConfigPath);

            try
            {
                Directory.CreateDirectory(coreConfigPath);
                var coreDestDir = Path.Combine(coreConfigPath, robotId);
                await Task.Run(() => CopyDirectoryRobust(sourceDir, coreDestDir));

                var copiedFiles = Directory.GetFiles(coreDestDir, "*", SearchOption.AllDirectories);
                if (copiedFiles.Length > 0)
                {
                    Log.Information("SUCCESS: Copied {Count} files to Core: {Dest}", copiedFiles.Length, coreDestDir);
                    copiedLocations++;
                }
                else
                {
                    Log.Warning("Copy to Core resulted in empty directory: {Dest}", coreDestDir);
                }
            }
            catch (Exception ex)
            {
                Log.Warning(ex, "Failed to copy to Core location");
            }

            // === LOCATION 2: UI mesh loading folder ===
            var uiConfigPath = Path.GetFullPath(Path.Combine(baseDir, "config", "robots"));

            StatusMessage = "Copying to UI location...";
            Log.Information("Copying package to UI: {Path}", uiConfigPath);

            try
            {
                Directory.CreateDirectory(uiConfigPath);
                var uiDestDir = Path.Combine(uiConfigPath, robotId);

                // Check if source and dest are the same (OutputDirectory is already in UI location)
                var normalizedSource = Path.GetFullPath(sourceDir).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar).ToLowerInvariant();
                var normalizedDest = Path.GetFullPath(uiDestDir).TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar).ToLowerInvariant();

                if (normalizedSource == normalizedDest)
                {
                    // Already in UI location, just verify files exist
                    var existingFiles = Directory.GetFiles(uiDestDir, "*", SearchOption.AllDirectories);
                    if (existingFiles.Length > 0)
                    {
                        Log.Information("UI location is same as source, skipping copy. Has {Count} files.", existingFiles.Length);
                        copiedLocations++;
                    }
                    else
                    {
                        Log.Warning("UI location is same as source but has no files!");
                    }
                }
                else
                {
                    await Task.Run(() => CopyDirectoryRobust(sourceDir, uiDestDir));

                    var copiedFiles = Directory.GetFiles(uiDestDir, "*", SearchOption.AllDirectories);
                    if (copiedFiles.Length > 0)
                    {
                        Log.Information("SUCCESS: Copied {Count} files to UI: {Dest}", copiedFiles.Length, uiDestDir);
                        copiedLocations++;
                    }
                    else
                    {
                        Log.Warning("Copy to UI resulted in empty directory: {Dest}", uiDestDir);
                    }
                }
            }
            catch (Exception ex)
            {
                Log.Warning(ex, "Failed to copy to UI location");
            }

            // === LOCATION 3: Also keep in OutputDirectory (already there from generate) ===
            // This ensures consistency

            if (copiedLocations == 0)
            {
                StatusMessage = "Failed to copy to any location. Check logs.";
                return;
            }

            // Reload packages via IPC (Core will re-scan folder)
            StatusMessage = "Reloading package list...";
            var response = await _ipcClient.ReloadPackagesAsync();

            if (response != null && response.Success)
            {
                var found = response.Packages.Any(p =>
                    p.Id.Equals(robotId, StringComparison.OrdinalIgnoreCase));

                if (found)
                    StatusMessage = $"Success! {robotId} is now available ({copiedLocations} locations, {response.Count} packages total)";
                else
                    StatusMessage = $"Copied to {copiedLocations} locations but not found in catalog. Restart Core.";
            }
            else
            {
                StatusMessage = $"Copied to {copiedLocations} locations. Restart Core to load new package.";
            }
        }
        catch (Exception ex)
        {
            StatusMessage = $"Error: {ex.Message}";
            Log.Error(ex, "Failed to copy package");
        }
        finally
        {
            IsLoading = false;
        }
    }

    private static void CopyDirectoryRobust(string sourceDir, string destDir)
    {
        Log.Debug("CopyDirectoryRobust: {Source} -> {Dest}", sourceDir, destDir);

        // Create destination directory
        Directory.CreateDirectory(destDir);

        // Copy all files
        foreach (var file in Directory.GetFiles(sourceDir))
        {
            var fileName = Path.GetFileName(file);
            var destFile = Path.Combine(destDir, fileName);

            try
            {
                File.Copy(file, destFile, overwrite: true);
                Log.Debug("Copied file: {File}", fileName);
            }
            catch (Exception ex)
            {
                Log.Warning(ex, "Failed to copy file {File}, retrying...", fileName);
                Thread.Sleep(100);
                try
                {
                    File.Copy(file, destFile, overwrite: true);
                }
                catch (Exception ex2)
                {
                    Log.Error(ex2, "Failed to copy file {File} after retry", fileName);
                    throw;
                }
            }
        }

        // Recursively copy subdirectories
        foreach (var dir in Directory.GetDirectories(sourceDir))
        {
            var dirName = Path.GetFileName(dir);
            var destSubDir = Path.Combine(destDir, dirName);
            CopyDirectoryRobust(dir, destSubDir);
        }
    }

    private static void CopyDirectory(string sourceDir, string destDir)
    {
        // Delete destination first to avoid locked file issues
        if (Directory.Exists(destDir))
        {
            try
            {
                Directory.Delete(destDir, recursive: true);
                Thread.Sleep(100); // Wait for filesystem to release
            }
            catch (Exception ex)
            {
                Log.Warning(ex, "Could not delete existing directory {Dir}, will try overwrite", destDir);
            }
        }

        Directory.CreateDirectory(destDir);

        // Copy files with retry for locked files
        foreach (var file in Directory.GetFiles(sourceDir))
        {
            var destFile = Path.Combine(destDir, Path.GetFileName(file));
            CopyFileWithRetry(file, destFile, maxRetries: 5, delayMs: 200);
        }

        // Copy subdirectories
        foreach (var dir in Directory.GetDirectories(sourceDir))
        {
            var destSubDir = Path.Combine(destDir, Path.GetFileName(dir));
            CopyDirectory(dir, destSubDir);
        }
    }

    private static void CopyFileWithRetry(string source, string dest, int maxRetries, int delayMs)
    {
        for (int i = 0; i < maxRetries; i++)
        {
            try
            {
                // Delete destination file first if it exists
                if (File.Exists(dest))
                {
                    File.Delete(dest);
                    Thread.Sleep(50);
                }
                File.Copy(source, dest, overwrite: true);
                return;
            }
            catch (IOException) when (i < maxRetries - 1)
            {
                Log.Debug("File locked, retry {Attempt}/{Max}: {File}", i + 1, maxRetries, dest);
                Thread.Sleep(delayMs);
            }
        }
        // Final attempt - let exception propagate
        File.Copy(source, dest, overwrite: true);
    }
}

/// <summary>
/// Display info for parsed URDF joint
/// </summary>
public class UrdfJointInfo
{
    public string Name { get; set; } = "";
    public string Type { get; set; } = "";
    public double OriginX { get; set; }
    public double OriginY { get; set; }
    public double OriginZ { get; set; }
    public double AxisX { get; set; }
    public double AxisY { get; set; }
    public double AxisZ { get; set; }
    public double LimitMin { get; set; }
    public double LimitMax { get; set; }

    public string OriginDisplay => $"[{OriginX:F1}, {OriginY:F1}, {OriginZ:F1}] mm";
    public string AxisDisplay => $"[{AxisX:F0}, {AxisY:F0}, {AxisZ:F0}]";
    public string LimitsDisplay => $"{LimitMin:F0}° to {LimitMax:F0}°";
}
