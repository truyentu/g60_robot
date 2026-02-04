using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels.Pages;

/// <summary>
/// ViewModel for Configuration view
/// </summary>
public partial class ConfigurationViewModel : ObservableObject
{
    // Connection settings
    [ObservableProperty]
    private string _repAddress = "tcp://localhost:5555";

    [ObservableProperty]
    private string _subAddress = "tcp://localhost:5556";

    [ObservableProperty]
    private bool _autoConnect = true;

    // Serial port settings
    [ObservableProperty]
    private ObservableCollection<string> _availablePorts = new();

    [ObservableProperty]
    private string? _selectedPort;

    [ObservableProperty]
    private ObservableCollection<int> _baudRates = new() { 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000 };

    [ObservableProperty]
    private int _selectedBaudRate = 115200;

    // Robot settings
    [ObservableProperty]
    private string _robotName = "Robot6DOF";

    [ObservableProperty]
    private string _robotModel = "6-Axis Industrial";

    [ObservableProperty]
    private ObservableCollection<JointLimitItem> _jointLimits = new();

    [ObservableProperty]
    private double _tcpOffsetX = 0;

    [ObservableProperty]
    private double _tcpOffsetY = 0;

    [ObservableProperty]
    private double _tcpOffsetZ = 80;

    // Safety settings
    [ObservableProperty]
    private bool _eStopEnabled = true;

    [ObservableProperty]
    private bool _softLimitsEnabled = true;

    [ObservableProperty]
    private bool _velocityMonitorEnabled = true;

    [ObservableProperty]
    private double _t1MaxSpeed = 250;

    [ObservableProperty]
    private double _autoMaxSpeed = 2000;

    // Welding settings
    [ObservableProperty]
    private string _welderType = "MIG/MAG";

    [ObservableProperty]
    private string _wireFeeder = "Integrated";

    [ObservableProperty]
    private bool _arcSensingEnabled = false;

    [ObservableProperty]
    private bool _seamTrackingEnabled = false;

    public ConfigurationViewModel()
    {
        InitializeJointLimits();
        RefreshPorts();
    }

    private void InitializeJointLimits()
    {
        JointLimits.Add(new JointLimitItem { Name = "J1", MinLimit = -180, MaxLimit = 180 });
        JointLimits.Add(new JointLimitItem { Name = "J2", MinLimit = -135, MaxLimit = 45 });
        JointLimits.Add(new JointLimitItem { Name = "J3", MinLimit = -60, MaxLimit = 150 });
        JointLimits.Add(new JointLimitItem { Name = "J4", MinLimit = -180, MaxLimit = 180 });
        JointLimits.Add(new JointLimitItem { Name = "J5", MinLimit = -120, MaxLimit = 120 });
        JointLimits.Add(new JointLimitItem { Name = "J6", MinLimit = -360, MaxLimit = 360 });
    }

    [RelayCommand]
    private void RefreshPorts()
    {
        AvailablePorts.Clear();
        try
        {
            // Add common COM ports for Windows
            for (int i = 1; i <= 20; i++)
            {
                AvailablePorts.Add($"COM{i}");
            }
            if (AvailablePorts.Count > 0 && SelectedPort == null)
            {
                SelectedPort = AvailablePorts[0];
            }
        }
        catch
        {
            // Ignore errors
        }
    }

    [RelayCommand]
    private void ResetDefault()
    {
        RepAddress = "tcp://localhost:5555";
        SubAddress = "tcp://localhost:5556";
        AutoConnect = true;
        SelectedBaudRate = 115200;

        EStopEnabled = true;
        SoftLimitsEnabled = true;
        VelocityMonitorEnabled = true;
        T1MaxSpeed = 250;
        AutoMaxSpeed = 2000;

        TcpOffsetX = 0;
        TcpOffsetY = 0;
        TcpOffsetZ = 80;
    }

    [RelayCommand]
    private void Apply()
    {
        // Apply settings to running system
    }

    [RelayCommand]
    private void Save()
    {
        // Save settings to file
    }
}

public partial class JointLimitItem : ObservableObject
{
    [ObservableProperty]
    private string _name = "";

    [ObservableProperty]
    private double _minLimit;

    [ObservableProperty]
    private double _maxLimit;
}
