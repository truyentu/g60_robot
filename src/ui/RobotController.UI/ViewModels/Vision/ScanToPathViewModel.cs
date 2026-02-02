using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Services;
using System;
using System.Collections.ObjectModel;
using System.IO;
using System.Threading.Tasks;

namespace RobotController.UI.ViewModels.Vision;

/// <summary>
/// View model for Scan-to-Path functionality
/// </summary>
public partial class ScanToPathViewModel : ObservableObject
{
    private readonly ISensorClientService? _sensorService;
    private readonly ISeamDetectionService? _seamService;

    // ========================================================================
    // Scan Settings
    // ========================================================================

    [ObservableProperty]
    private uint _profileCount = 500;

    [ObservableProperty]
    private uint _timeoutMs = 30000;

    [ObservableProperty]
    private float _voxelSize = 1.0f;

    [ObservableProperty]
    private bool _enableDownsample = true;

    // ========================================================================
    // Scan State
    // ========================================================================

    [ObservableProperty]
    private bool _isScanning;

    [ObservableProperty]
    private double _scanProgress;

    [ObservableProperty]
    private string _scanStatus = "Ready";

    // ========================================================================
    // Results
    // ========================================================================

    [ObservableProperty]
    private SeamPathData? _scannedPath;

    [ObservableProperty]
    private bool _hasPath;

    [ObservableProperty]
    private int _pathPointCount;

    [ObservableProperty]
    private double _pathLength;

    [ObservableProperty]
    private double _avgGapWidth;

    [ObservableProperty]
    private string _detectedJointType = "";

    // ========================================================================
    // Path List
    // ========================================================================

    public ObservableCollection<SavedPathEntry> SavedPaths { get; } = new();

    [ObservableProperty]
    private SavedPathEntry? _selectedPath;

    public ScanToPathViewModel(
        ISensorClientService? sensorService = null,
        ISeamDetectionService? seamService = null)
    {
        _sensorService = sensorService;
        _seamService = seamService;

        LoadSavedPaths();
    }

    [RelayCommand]
    private async Task StartScanAsync()
    {
        if (_seamService == null) return;

        IsScanning = true;
        ScanStatus = "Scanning...";
        ScanProgress = 0;

        try
        {
            var result = await _seamService.GetSeamPathAsync();

            if (result.Success)
            {
                ScannedPath = result;
                HasPath = true;
                PathPointCount = result.PointsX.Length;
                PathLength = result.TotalLength;
                AvgGapWidth = result.AvgGapWidth;
                DetectedJointType = result.JointType.ToString();

                ScanStatus = $"Scan complete: {PathPointCount} points, {PathLength:F1}mm";
            }
            else
            {
                ScanStatus = "Scan failed";
                HasPath = false;
            }
        }
        catch (Exception ex)
        {
            ScanStatus = $"Error: {ex.Message}";
        }
        finally
        {
            IsScanning = false;
            ScanProgress = 100;
        }
    }

    [RelayCommand]
    private void StopScan()
    {
        IsScanning = false;
        ScanStatus = "Scan stopped";
    }

    [RelayCommand]
    private async Task SetAsNominalPathAsync()
    {
        if (_seamService == null || ScannedPath == null) return;

        var result = await _seamService.SetNominalPathAsync(
            ScannedPath.PointsX,
            ScannedPath.PointsY,
            ScannedPath.PointsZ);

        ScanStatus = result
            ? "Set as nominal path"
            : "Failed to set nominal path";
    }

    [RelayCommand]
    private async Task SavePathAsync()
    {
        if (ScannedPath == null) return;

        string name = $"Path_{DateTime.Now:yyyyMMdd_HHmmss}";
        string pathsDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "SeamPaths");

        Directory.CreateDirectory(pathsDir);

        string filepath = Path.Combine(pathsDir, $"{name}.json");

        // Serialize and save
        var json = System.Text.Json.JsonSerializer.Serialize(ScannedPath);
        await File.WriteAllTextAsync(filepath, json);

        var entry = new SavedPathEntry
        {
            Name = name,
            FilePath = filepath,
            PointCount = PathPointCount,
            Length = PathLength,
            CreatedAt = DateTime.Now
        };

        SavedPaths.Add(entry);
        ScanStatus = $"Saved as {name}";
    }

    [RelayCommand]
    private async Task LoadPathAsync(SavedPathEntry? entry)
    {
        if (entry == null || !File.Exists(entry.FilePath)) return;

        var json = await File.ReadAllTextAsync(entry.FilePath);
        var path = System.Text.Json.JsonSerializer.Deserialize<SeamPathData>(json);

        if (path != null)
        {
            ScannedPath = path;
            HasPath = true;
            PathPointCount = path.PointsX.Length;
            PathLength = path.TotalLength;
            AvgGapWidth = path.AvgGapWidth;
            DetectedJointType = path.JointType.ToString();

            ScanStatus = $"Loaded {entry.Name}";
        }
    }

    [RelayCommand]
    private void DeletePath(SavedPathEntry? entry)
    {
        if (entry == null) return;

        if (File.Exists(entry.FilePath))
        {
            File.Delete(entry.FilePath);
        }

        SavedPaths.Remove(entry);
    }

    private void LoadSavedPaths()
    {
        string pathsDir = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "SeamPaths");

        if (!Directory.Exists(pathsDir)) return;

        foreach (var file in Directory.GetFiles(pathsDir, "*.json"))
        {
            var info = new FileInfo(file);
            SavedPaths.Add(new SavedPathEntry
            {
                Name = Path.GetFileNameWithoutExtension(file),
                FilePath = file,
                CreatedAt = info.CreationTime
            });
        }
    }
}

public class SavedPathEntry
{
    public string Name { get; set; } = "";
    public string FilePath { get; set; } = "";
    public int PointCount { get; set; }
    public double Length { get; set; }
    public DateTime CreatedAt { get; set; }

    public string DisplayText => $"{Name} ({PointCount} pts, {Length:F0}mm)";
}
