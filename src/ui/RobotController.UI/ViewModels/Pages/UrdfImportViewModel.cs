using System.Collections.ObjectModel;
using System.IO;
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
    private string _statusMessage = "Select a URDF or xacro file to import";

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

                if (!string.IsNullOrEmpty(response.SavedPath))
                {
                    StatusMessage = $"Saved to: {response.SavedPath}";
                    Log.Information("robot.yaml saved to: {Path}", response.SavedPath);
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

    [RelayCommand]
    private void CopyYamlToClipboard()
    {
        if (!string.IsNullOrEmpty(GeneratedYaml))
        {
            System.Windows.Clipboard.SetText(GeneratedYaml);
            StatusMessage = "YAML copied to clipboard";
        }
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
