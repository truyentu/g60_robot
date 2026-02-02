using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Services;
using System;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Threading.Tasks;

namespace RobotController.UI.ViewModels.Vision;

/// <summary>
/// View model for hand-eye calibration wizard
/// </summary>
public partial class CalibrationViewModel : ObservableObject
{
    private readonly ISensorClientService? _sensorService;

    // ========================================================================
    // Wizard State
    // ========================================================================

    [ObservableProperty]
    private int _currentStep;

    [ObservableProperty]
    private int _totalSteps = 5;

    [ObservableProperty]
    private string _stepDescription = "Move robot to first calibration position";

    [ObservableProperty]
    private bool _canGoNext;

    [ObservableProperty]
    private bool _canGoPrevious;

    [ObservableProperty]
    private bool _isCalibrating;

    // ========================================================================
    // Calibration Data
    // ========================================================================

    public ObservableCollection<CalibrationPoseEntry> CapturedPoses { get; } = new();

    [ObservableProperty]
    private int _requiredPoses = 10;

    [ObservableProperty]
    private int _capturedPoseCount;

    // Current robot pose
    [ObservableProperty]
    private double[] _currentJoints = new double[6];

    [ObservableProperty]
    private double[] _currentPosition = new double[3];

    // ========================================================================
    // Results
    // ========================================================================

    [ObservableProperty]
    private bool _calibrationComplete;

    [ObservableProperty]
    private double _reprojectionError;

    [ObservableProperty]
    private double[] _translation = new double[3];

    [ObservableProperty]
    private double[] _rotation = new double[3];

    [ObservableProperty]
    private string _calibrationStatus = "";

    // Display
    public string ProgressText => $"Pose {CapturedPoseCount} / {RequiredPoses}";
    public double ProgressPercent => (double)CapturedPoseCount / RequiredPoses * 100;

    public CalibrationViewModel(ISensorClientService? sensorService = null)
    {
        _sensorService = sensorService;

        CurrentStep = 0;
        UpdateStepDescription();
    }

    [RelayCommand]
    private async Task CapturePoseAsync()
    {
        // Get current robot pose
        var robotJoints = CurrentJoints.ToArray();

        // Get profile and detect calibration target (simplified)
        var targetPose = new double[16];
        for (int i = 0; i < 4; i++) targetPose[i * 4 + i] = 1.0;

        var entry = new CalibrationPoseEntry
        {
            Index = CapturedPoseCount + 1,
            RobotJoints = robotJoints,
            TargetPose = targetPose,
            Timestamp = DateTime.Now
        };

        CapturedPoses.Add(entry);
        CapturedPoseCount = CapturedPoses.Count;

        OnPropertyChanged(nameof(ProgressText));
        OnPropertyChanged(nameof(ProgressPercent));

        CalibrationStatus = $"Captured pose {CapturedPoseCount}";

        if (CapturedPoseCount >= RequiredPoses)
        {
            CanGoNext = true;
        }

        await Task.CompletedTask;
    }

    [RelayCommand]
    private void DeletePose(CalibrationPoseEntry? entry)
    {
        if (entry != null)
        {
            CapturedPoses.Remove(entry);
            CapturedPoseCount = CapturedPoses.Count;

            // Re-index
            for (int i = 0; i < CapturedPoses.Count; i++)
            {
                CapturedPoses[i].Index = i + 1;
            }

            OnPropertyChanged(nameof(ProgressText));
            OnPropertyChanged(nameof(ProgressPercent));
        }
    }

    [RelayCommand]
    private async Task PerformCalibrationAsync()
    {
        if (CapturedPoses.Count < 3)
        {
            CalibrationStatus = "Need at least 3 poses";
            return;
        }

        IsCalibrating = true;
        CalibrationStatus = "Calibrating...";

        // Simulate calibration (in real implementation, call C++ core)
        await Task.Delay(1000);

        IsCalibrating = false;
        CalibrationComplete = true;
        ReprojectionError = 0.15; // Simulated result
        Translation = new double[] { 50.0, 0.0, 100.0 };

        CalibrationStatus = $"Calibration complete! Error: {ReprojectionError:F3} mm";
        CurrentStep = TotalSteps - 1;
    }

    [RelayCommand]
    private async Task SaveCalibrationAsync()
    {
        string filepath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "Calibration", "hand_eye_calibration.json");

        Directory.CreateDirectory(Path.GetDirectoryName(filepath)!);

        // Serialize and save calibration data
        var calibData = new
        {
            Translation,
            Rotation,
            ReprojectionError,
            SavedAt = DateTime.Now
        };

        var json = System.Text.Json.JsonSerializer.Serialize(calibData);
        await File.WriteAllTextAsync(filepath, json);

        CalibrationStatus = $"Saved to {filepath}";
    }

    [RelayCommand]
    private async Task LoadCalibrationAsync()
    {
        string filepath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "Calibration", "hand_eye_calibration.json");

        if (!File.Exists(filepath))
        {
            CalibrationStatus = "No calibration file found";
            return;
        }

        var json = await File.ReadAllTextAsync(filepath);
        CalibrationStatus = "Calibration loaded";
        CalibrationComplete = true;
    }

    [RelayCommand]
    private void NextStep()
    {
        if (CurrentStep < TotalSteps - 1)
        {
            CurrentStep++;
            UpdateStepDescription();
        }
    }

    [RelayCommand]
    private void PreviousStep()
    {
        if (CurrentStep > 0)
        {
            CurrentStep--;
            UpdateStepDescription();
        }
    }

    [RelayCommand]
    private void Reset()
    {
        CapturedPoses.Clear();
        CapturedPoseCount = 0;
        CalibrationComplete = false;
        ReprojectionError = 0;
        CurrentStep = 0;
        CalibrationStatus = "";
        UpdateStepDescription();
    }

    private void UpdateStepDescription()
    {
        StepDescription = CurrentStep switch
        {
            0 => "Step 1: Connect sensor and verify it's working",
            1 => "Step 2: Position calibration target in sensor view",
            2 => $"Step 3: Capture {RequiredPoses} poses from different angles",
            3 => "Step 4: Run calibration algorithm",
            4 => "Step 5: Verify and save calibration",
            _ => ""
        };

        CanGoPrevious = CurrentStep > 0;
        CanGoNext = CurrentStep switch
        {
            2 => CapturedPoseCount >= RequiredPoses,
            3 => CalibrationComplete,
            _ => true
        };
    }
}

public class CalibrationPoseEntry
{
    public int Index { get; set; }
    public double[] RobotJoints { get; set; } = new double[6];
    public double[] TargetPose { get; set; } = new double[16];
    public DateTime Timestamp { get; set; }

    public string JointsDisplay =>
        string.Join(", ", RobotJoints.Select(j => $"{j * 180 / Math.PI:F1}°"));
}
