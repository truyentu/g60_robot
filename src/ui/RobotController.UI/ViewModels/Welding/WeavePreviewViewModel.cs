using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using System;
using System.Collections.ObjectModel;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;

namespace RobotController.UI.ViewModels.Welding;

/// <summary>
/// View model for weave pattern preview
/// </summary>
public partial class WeavePreviewViewModel : ObservableObject
{
    private readonly IWeaveClientService? _weaveService;

    // Pattern selection
    [ObservableProperty]
    private WeavePatternType _selectedPattern = WeavePatternType.Sinusoidal;

    [ObservableProperty]
    private string _selectedPreset = "Medium Gap";

    // Parameters
    [ObservableProperty]
    private double _amplitude = 3.0;

    [ObservableProperty]
    private double _wavelength = 10.0;

    [ObservableProperty]
    private double _frequency = 1.0;

    [ObservableProperty]
    private double _dwellLeft;

    [ObservableProperty]
    private double _dwellRight;

    [ObservableProperty]
    private double _speedAtEdge = 0.8;

    [ObservableProperty]
    private bool _useFrequency;

    [ObservableProperty]
    private bool _isEnabled;

    // Preview data
    [ObservableProperty]
    private PointCollection _previewPoints = new();

    [ObservableProperty]
    private double _previewWidth = 200;

    [ObservableProperty]
    private double _previewHeight = 80;

    // Status
    [ObservableProperty]
    private bool _isActive;

    [ObservableProperty]
    private double _currentPhase;

    [ObservableProperty]
    private int _cycleCount;

    [ObservableProperty]
    private double _pathLengthRatio = 1.0;

    // Collections
    public ObservableCollection<WeavePatternType> PatternTypes { get; } = new()
    {
        WeavePatternType.None,
        WeavePatternType.Linear,
        WeavePatternType.Triangular,
        WeavePatternType.Sinusoidal,
        WeavePatternType.Circular,
        WeavePatternType.Figure8,
        WeavePatternType.Crescent
    };

    public ObservableCollection<string> Presets { get; } = new()
    {
        "Narrow Gap",
        "Medium Gap",
        "Wide Gap",
        "Vertical Up",
        "Overhead",
        "Root Pass",
        "Cap Pass"
    };

    public WeavePreviewViewModel(IWeaveClientService? weaveService = null)
    {
        _weaveService = weaveService;
        GeneratePreviewLocal();
    }

    partial void OnSelectedPatternChanged(WeavePatternType value)
    {
        GeneratePreviewLocal();
    }

    partial void OnAmplitudeChanged(double value)
    {
        GeneratePreviewLocal();
    }

    partial void OnWavelengthChanged(double value)
    {
        GeneratePreviewLocal();
    }

    [RelayCommand]
    private void ApplyPreset(string presetName)
    {
        SelectedPreset = presetName;

        switch (presetName)
        {
            case "Narrow Gap":
                SelectedPattern = WeavePatternType.Triangular;
                Amplitude = 2.0;
                Wavelength = 6.0;
                DwellLeft = 50;
                DwellRight = 50;
                break;

            case "Medium Gap":
                SelectedPattern = WeavePatternType.Sinusoidal;
                Amplitude = 4.0;
                Wavelength = 10.0;
                DwellLeft = 100;
                DwellRight = 100;
                break;

            case "Wide Gap":
                SelectedPattern = WeavePatternType.Triangular;
                Amplitude = 6.0;
                Wavelength = 12.0;
                DwellLeft = 150;
                DwellRight = 150;
                SpeedAtEdge = 0.6;
                break;

            case "Vertical Up":
                SelectedPattern = WeavePatternType.Crescent;
                Amplitude = 5.0;
                Wavelength = 8.0;
                DwellLeft = 200;
                DwellRight = 200;
                break;

            case "Overhead":
                SelectedPattern = WeavePatternType.Linear;
                Amplitude = 3.0;
                Wavelength = 6.0;
                DwellLeft = 100;
                DwellRight = 100;
                SpeedAtEdge = 0.7;
                break;

            case "Root Pass":
                SelectedPattern = WeavePatternType.Linear;
                Amplitude = 1.5;
                Wavelength = 4.0;
                DwellLeft = 0;
                DwellRight = 0;
                break;

            case "Cap Pass":
                SelectedPattern = WeavePatternType.Sinusoidal;
                Amplitude = 5.0;
                Wavelength = 8.0;
                DwellLeft = 80;
                DwellRight = 80;
                break;
        }

        GeneratePreviewLocal();
    }

    [RelayCommand]
    private async Task ApplyToControllerAsync()
    {
        if (_weaveService == null) return;

        var paramsData = new WeaveParamsData
        {
            PatternType = (int)SelectedPattern,
            Amplitude = Amplitude,
            Wavelength = Wavelength,
            Frequency = Frequency,
            DwellLeft = DwellLeft,
            DwellRight = DwellRight,
            SpeedAtEdge = SpeedAtEdge,
            SpeedAtCenter = 1.0,
            UseFrequency = UseFrequency
        };

        await _weaveService.SetParamsAsync(paramsData);
    }

    [RelayCommand]
    private async Task EnableWeaveAsync()
    {
        if (_weaveService == null) return;
        await _weaveService.EnableAsync(true);
        IsEnabled = true;
    }

    [RelayCommand]
    private async Task DisableWeaveAsync()
    {
        if (_weaveService == null) return;
        await _weaveService.EnableAsync(false);
        IsEnabled = false;
    }

    public void UpdateStatus(WeaveStatusResponse status)
    {
        IsEnabled = status.Enabled;
        IsActive = status.Active;
        CurrentPhase = status.CurrentPhase;
        CycleCount = status.CycleCount;
    }

    private void GeneratePreviewLocal()
    {
        var points = new PointCollection();
        int numCycles = 3;
        int pointsPerCycle = 32;
        int totalPoints = numCycles * pointsPerCycle;

        double scaleX = PreviewWidth / (numCycles * Wavelength);
        double scaleY = PreviewHeight / (Amplitude * 2.5);
        double centerY = PreviewHeight / 2;

        for (int i = 0; i < totalPoints; i++)
        {
            double progress = (double)i / totalPoints;
            double phase = (progress * numCycles) % 1.0;
            double x = progress * PreviewWidth;
            double y = centerY;

            switch (SelectedPattern)
            {
                case WeavePatternType.Sinusoidal:
                    y = centerY - Amplitude * Math.Sin(2 * Math.PI * phase) * scaleY;
                    break;

                case WeavePatternType.Linear:
                case WeavePatternType.Triangular:
                    if (phase < 0.5)
                        y = centerY - Amplitude * (phase / 0.5 * 2 - 1) * scaleY;
                    else
                        y = centerY - Amplitude * (1 - (phase - 0.5) / 0.5 * 2) * scaleY;
                    break;

                case WeavePatternType.Circular:
                    y = centerY - Amplitude * Math.Cos(2 * Math.PI * phase) * scaleY;
                    break;

                case WeavePatternType.Figure8:
                    y = centerY - Amplitude * Math.Sin(2 * Math.PI * phase) * scaleY;
                    break;

                case WeavePatternType.Crescent:
                    if (phase < 0.5)
                        y = centerY - Amplitude * Math.Sin(Math.PI * phase / 0.5) * scaleY;
                    else
                        y = centerY + Amplitude * Math.Sin(Math.PI * (phase - 0.5) / 0.5) * scaleY;
                    break;
            }

            points.Add(new Point(x, y));
        }

        PreviewPoints = points;

        // Approximate path length ratio
        PathLengthRatio = SelectedPattern switch
        {
            WeavePatternType.None => 1.0,
            WeavePatternType.Linear or WeavePatternType.Triangular =>
                Math.Sqrt(1 + Math.Pow(4 * Amplitude / Wavelength, 2)),
            WeavePatternType.Sinusoidal =>
                Math.Sqrt(1 + Math.Pow(2 * Math.PI * Amplitude / Wavelength, 2) / 2),
            WeavePatternType.Circular => 2 * Math.PI * Amplitude / Wavelength,
            _ => 1.2
        };
    }
}
