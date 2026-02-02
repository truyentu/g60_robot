using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Services;
using System;
using System.Collections.ObjectModel;
using System.Threading.Tasks;
using System.Windows;

namespace RobotController.UI.ViewModels.Vision;

/// <summary>
/// View model for sensor connection and status
/// </summary>
public partial class SensorStatusViewModel : ObservableObject
{
    private readonly ISensorClientService? _sensorService;

    // ========================================================================
    // Connection
    // ========================================================================

    [ObservableProperty]
    private bool _isConnected;

    [ObservableProperty]
    private bool _isAcquiring;

    [ObservableProperty]
    private string _connectionStatus = "Disconnected";

    [ObservableProperty]
    private string _statusColor = "#808080";

    [ObservableProperty]
    private string _selectedDevice = "";

    public ObservableCollection<string> AvailableDevices { get; } = new();

    // ========================================================================
    // Sensor Info
    // ========================================================================

    [ObservableProperty]
    private string _deviceId = "";

    [ObservableProperty]
    private string _serialNumber = "";

    [ObservableProperty]
    private string _modelName = "";

    // ========================================================================
    // Performance
    // ========================================================================

    [ObservableProperty]
    private double _frameRate;

    [ObservableProperty]
    private ulong _framesReceived;

    [ObservableProperty]
    private ulong _framesDropped;

    [ObservableProperty]
    private double _dropRate;

    // ========================================================================
    // Configuration
    // ========================================================================

    [ObservableProperty]
    private double _exposureTime = 1000;

    [ObservableProperty]
    private double _gain = 1.0;

    [ObservableProperty]
    private string _triggerMode = "FreeRun";

    [ObservableProperty]
    private uint _encoderDivider = 1;

    public ObservableCollection<string> TriggerModes { get; } = new()
    {
        "FreeRun",
        "Software",
        "Encoder",
        "Hardware"
    };

    // Display strings
    public string FrameRateDisplay => $"{FrameRate:F1} Hz";
    public string FramesDisplay => $"{FramesReceived:N0}";
    public string DroppedDisplay => $"{FramesDropped:N0} ({DropRate:F2}%)";

    public SensorStatusViewModel(ISensorClientService? sensorService = null)
    {
        _sensorService = sensorService;
    }

    [RelayCommand]
    private async Task EnumerateDevicesAsync()
    {
        if (_sensorService == null) return;

        var devices = await _sensorService.EnumerateLaserProfilersAsync();

        AvailableDevices.Clear();
        foreach (var device in devices)
        {
            AvailableDevices.Add(device.DeviceId);
        }

        if (AvailableDevices.Count > 0 && string.IsNullOrEmpty(SelectedDevice))
        {
            SelectedDevice = AvailableDevices[0];
        }
    }

    [RelayCommand]
    private async Task ConnectAsync()
    {
        if (_sensorService == null || string.IsNullOrEmpty(SelectedDevice)) return;

        ConnectionStatus = "Connecting...";
        StatusColor = "#FFAA00";

        var result = await _sensorService.ConnectLaserAsync(SelectedDevice);

        if (result.Success)
        {
            IsConnected = true;
            ConnectionStatus = "Connected";
            StatusColor = "#00FF00";
            DeviceId = SelectedDevice;
        }
        else
        {
            ConnectionStatus = "Connection failed";
            StatusColor = "#FF0000";
        }
    }

    [RelayCommand]
    private async Task DisconnectAsync()
    {
        if (_sensorService == null) return;

        await _sensorService.DisconnectLaserAsync();
        IsConnected = false;
        IsAcquiring = false;
        ConnectionStatus = "Disconnected";
        StatusColor = "#808080";
    }

    [RelayCommand]
    private async Task StartAcquisitionAsync()
    {
        if (_sensorService == null) return;

        var result = await _sensorService.StartLaserAcquisitionAsync();
        if (result.Success)
        {
            IsAcquiring = true;
        }
    }

    [RelayCommand]
    private async Task StopAcquisitionAsync()
    {
        if (_sensorService == null) return;

        var result = await _sensorService.StopLaserAcquisitionAsync();
        if (result.Success)
        {
            IsAcquiring = false;
        }
    }

    [RelayCommand]
    private async Task ApplyConfigAsync()
    {
        if (_sensorService == null) return;

        var config = new LaserProfilerConfigData
        {
            ExposureTime = (float)ExposureTime,
            Gain = (float)Gain,
            TriggerMode = TriggerMode.ToLower(),
            EncoderDivider = EncoderDivider
        };
        await _sensorService.ConfigureLaserAsync(config);
    }

    public async Task RefreshStatusAsync()
    {
        if (_sensorService == null) return;

        var status = await _sensorService.GetLaserStatusAsync();

        if (status.Success)
        {
            FrameRate = status.FrameRate;
            FramesReceived = status.FramesReceived;
            FramesDropped = status.FramesDropped;

            if (status.FramesReceived > 0)
            {
                DropRate = (double)status.FramesDropped / status.FramesReceived * 100;
            }

            OnPropertyChanged(nameof(FrameRateDisplay));
            OnPropertyChanged(nameof(FramesDisplay));
            OnPropertyChanged(nameof(DroppedDisplay));
        }
    }
}
