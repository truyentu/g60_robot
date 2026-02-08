using System;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Microsoft.Win32;
using RobotController.UI.Models;
using RobotController.UI.Services;
using Serilog;

namespace RobotController.UI.ViewModels.Dialogs;

/// <summary>
/// ViewModel for Import Robot Dialog
/// </summary>
public partial class ImportRobotViewModel : ObservableObject
{
    private readonly IUrdfImportService _urdfService;
    private readonly IRobotPackageGenerator _generator;

    // ========================================================================
    // Step 1: Folder Selection
    // ========================================================================

    [ObservableProperty]
    private string _selectedFolder = "";

    [ObservableProperty]
    private ObservableCollection<UrdfFileInfo> _urdfFiles = new();

    // ========================================================================
    // Step 2: File Selection
    // ========================================================================

    [ObservableProperty]
    private UrdfFileInfo? _selectedUrdfFile;

    // ========================================================================
    // Step 3: Preview
    // ========================================================================

    [ObservableProperty]
    private UrdfRobot? _parsedRobot;

    [ObservableProperty]
    private UrdfParseResult? _parseResult;

    [ObservableProperty]
    private ObservableCollection<string> _previewMessages = new();

    [ObservableProperty]
    private bool _isParsed;

    [ObservableProperty]
    private bool _hasWarnings;

    // ========================================================================
    // Step 4: Import Settings
    // ========================================================================

    [ObservableProperty]
    private string _robotId = "";

    [ObservableProperty]
    private string _manufacturer = "";

    [ObservableProperty]
    private string _outputPath = "";

    // ========================================================================
    // State
    // ========================================================================

    [ObservableProperty]
    private bool _isLoading;

    [ObservableProperty]
    private string _statusMessage = "Select a ROS package folder to begin";

    [ObservableProperty]
    private bool _canImport;

    // ========================================================================
    // Events
    // ========================================================================

    /// <summary>Raised when import completes (true = success)</summary>
    public event EventHandler<bool>? ImportCompleted;

    /// <summary>Request to close dialog</summary>
    public event EventHandler? CloseRequested;

    // ========================================================================
    // Constructor
    // ========================================================================

    public ImportRobotViewModel(IUrdfImportService urdfService, IRobotPackageGenerator generator)
    {
        _urdfService = urdfService;
        _generator = generator;
        _outputPath = generator.GetDefaultOutputPath();
    }

    /// <summary>Design-time constructor</summary>
    public ImportRobotViewModel() : this(new UrdfImportService(), new RobotPackageGenerator())
    {
    }

    // ========================================================================
    // Commands
    // ========================================================================

    [RelayCommand]
    private void BrowseFolder()
    {
        var dialog = new OpenFolderDialog
        {
            Title = "Select ROS Robot Package Folder",
            Multiselect = false
        };

        if (dialog.ShowDialog() == true)
        {
            SelectedFolder = dialog.FolderName;
            ScanFolder();
        }
    }

    private void ScanFolder()
    {
        if (string.IsNullOrEmpty(SelectedFolder))
            return;

        try
        {
            IsLoading = true;
            StatusMessage = "Scanning for URDF/XACRO files...";

            UrdfFiles.Clear();
            ParsedRobot = null;
            IsParsed = false;
            CanImport = false;

            var files = _urdfService.ScanForUrdfFiles(SelectedFolder);

            foreach (var file in files)
            {
                UrdfFiles.Add(file);
            }

            if (UrdfFiles.Count > 0)
            {
                StatusMessage = $"Found {UrdfFiles.Count} URDF/XACRO file(s)";
                // Auto-select first macro file if available
                SelectedUrdfFile = UrdfFiles.FirstOrDefault(f => f.IsMacro) ?? UrdfFiles.First();
            }
            else
            {
                StatusMessage = "No URDF/XACRO files found in this folder";
            }

            // Try to detect manufacturer from folder name
            var folderName = Path.GetFileName(SelectedFolder) ?? "";
            if (folderName.StartsWith("abb", StringComparison.OrdinalIgnoreCase))
                Manufacturer = "ABB";
            else if (folderName.StartsWith("kuka", StringComparison.OrdinalIgnoreCase))
                Manufacturer = "KUKA";
            else if (folderName.Contains("motoman", StringComparison.OrdinalIgnoreCase) ||
                     folderName.StartsWith("yaskawa", StringComparison.OrdinalIgnoreCase))
                Manufacturer = "Yaskawa";
            else if (folderName.StartsWith("fanuc", StringComparison.OrdinalIgnoreCase))
                Manufacturer = "FANUC";
            else if (folderName.StartsWith("ur", StringComparison.OrdinalIgnoreCase))
                Manufacturer = "Universal Robots";
            else
                Manufacturer = "";
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Error scanning folder");
            StatusMessage = $"Error: {ex.Message}";
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task ParseSelectedFileAsync()
    {
        if (SelectedUrdfFile == null)
            return;

        try
        {
            IsLoading = true;
            StatusMessage = "Parsing URDF file...";
            PreviewMessages.Clear();
            IsParsed = false;
            CanImport = false;

            await Task.Run(() =>
            {
                ParseResult = _urdfService.ParseUrdf(SelectedUrdfFile.FullPath);
            });

            if (ParseResult == null)
            {
                StatusMessage = "Parse failed: No result returned";
                return;
            }

            // Update UI on main thread
            Application.Current.Dispatcher.Invoke(() =>
            {
                if (ParseResult.Success && ParseResult.Robot != null)
                {
                    ParsedRobot = ParseResult.Robot;
                    IsParsed = true;

                    // Add info messages
                    foreach (var msg in ParseResult.InfoMessages)
                    {
                        PreviewMessages.Add($"✓ {msg}");
                    }

                    // Add warnings
                    foreach (var msg in ParseResult.Warnings)
                    {
                        PreviewMessages.Add($"⚠ {msg}");
                    }

                    HasWarnings = ParseResult.Warnings.Count > 0;

                    // Generate default robot ID
                    RobotId = GenerateRobotId(ParsedRobot.Name);

                    // Check if can import
                    CanImport = ParsedRobot.DOF > 0;

                    if (CanImport)
                    {
                        StatusMessage = $"Ready to import: {ParsedRobot.Name} ({ParsedRobot.DOF} DOF)";
                    }
                    else
                    {
                        StatusMessage = "Cannot import: No movable joints found";
                        PreviewMessages.Add("✗ Cannot import: No movable joints found");
                    }
                }
                else
                {
                    StatusMessage = $"Parse failed: {ParseResult.ErrorMessage}";
                    PreviewMessages.Add($"✗ Error: {ParseResult.ErrorMessage}");
                }
            });
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Error parsing URDF");
            StatusMessage = $"Error: {ex.Message}";
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private async Task ImportAsync()
    {
        if (ParsedRobot == null || string.IsNullOrWhiteSpace(RobotId))
        {
            StatusMessage = "Please parse a URDF file and set Robot ID first";
            return;
        }

        try
        {
            IsLoading = true;
            StatusMessage = "Generating robot package...";

            var result = await _generator.GeneratePackageAsync(
                ParsedRobot,
                RobotId.Trim().ToLowerInvariant().Replace(" ", "_"),
                Manufacturer.Trim(),
                OutputPath);

            if (result.Success)
            {
                StatusMessage = $"Success! Package created at: {result.OutputPath}";
                Log.Information("Robot package created: {Path}", result.OutputPath);

                // Show success message
                MessageBox.Show(
                    $"Robot package created successfully!\n\n" +
                    $"Location: {result.OutputPath}\n" +
                    $"Meshes copied: {result.MeshesCopied}\n\n" +
                    "The robot will appear in Package Browser after refresh.",
                    "Import Successful",
                    MessageBoxButton.OK,
                    MessageBoxImage.Information);

                ImportCompleted?.Invoke(this, true);
                CloseRequested?.Invoke(this, EventArgs.Empty);
            }
            else
            {
                StatusMessage = $"Failed: {result.ErrorMessage}";
                MessageBox.Show(
                    $"Failed to create robot package:\n\n{result.ErrorMessage}",
                    "Import Failed",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error);
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Error importing robot");
            StatusMessage = $"Error: {ex.Message}";
            MessageBox.Show(
                $"Error during import:\n\n{ex.Message}",
                "Import Error",
                MessageBoxButton.OK,
                MessageBoxImage.Error);
        }
        finally
        {
            IsLoading = false;
        }
    }

    [RelayCommand]
    private void Cancel()
    {
        ImportCompleted?.Invoke(this, false);
        CloseRequested?.Invoke(this, EventArgs.Empty);
    }

    [RelayCommand]
    private void BrowseOutputPath()
    {
        var dialog = new OpenFolderDialog
        {
            Title = "Select Output Folder for Robot Packages",
            InitialDirectory = OutputPath
        };

        if (dialog.ShowDialog() == true)
        {
            OutputPath = dialog.FolderName;
        }
    }

    // ========================================================================
    // Helpers
    // ========================================================================

    /// <summary>
    /// Generate robot ID from name
    /// </summary>
    private string GenerateRobotId(string name)
    {
        // Convert "ABB IRB 7600-150/350" to "abb_irb_7600_150_350"
        return name
            .ToLowerInvariant()
            .Replace(" ", "_")
            .Replace("-", "_")
            .Replace("/", "_")
            .Replace(".", "_")
            .Replace("__", "_")
            .Trim('_');
    }

    // ========================================================================
    // Property Changed Handlers
    // ========================================================================

    partial void OnSelectedUrdfFileChanged(UrdfFileInfo? value)
    {
        // Auto-parse when file is selected
        if (value != null)
        {
            _ = ParseSelectedFileAsync();
        }
    }

    partial void OnRobotIdChanged(string value)
    {
        // Validate robot ID
        CanImport = IsParsed && !string.IsNullOrWhiteSpace(value) && ParsedRobot?.DOF > 0;
    }
}
