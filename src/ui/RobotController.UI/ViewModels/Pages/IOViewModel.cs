using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Services;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels.Pages;

/// <summary>
/// ViewModel for I/O monitoring view
/// </summary>
public partial class IOViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;

    public ObservableCollection<IOPoint> DigitalInputs { get; } = new();
    public ObservableCollection<IOPoint> DigitalOutputs { get; } = new();

    public IOViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;
        InitializeIOPoints();
    }

    private void InitializeIOPoints()
    {
        // Digital Inputs
        DigitalInputs.Add(new IOPoint { Address = "DI[0]", Name = "E-Stop", Value = false });
        DigitalInputs.Add(new IOPoint { Address = "DI[1]", Name = "Deadman Switch", Value = false });
        DigitalInputs.Add(new IOPoint { Address = "DI[2]", Name = "Door Sensor", Value = true });
        DigitalInputs.Add(new IOPoint { Address = "DI[3]", Name = "Light Curtain", Value = true });
        DigitalInputs.Add(new IOPoint { Address = "DI[4]", Name = "Workpiece Detect", Value = false });
        DigitalInputs.Add(new IOPoint { Address = "DI[5]", Name = "Clamp Closed", Value = false });
        DigitalInputs.Add(new IOPoint { Address = "DI[6]", Name = "Gas Flow OK", Value = true });
        DigitalInputs.Add(new IOPoint { Address = "DI[7]", Name = "Wire Feed OK", Value = true });
        DigitalInputs.Add(new IOPoint { Address = "DI[8]", Name = "Arc Established", Value = false });
        DigitalInputs.Add(new IOPoint { Address = "DI[9]", Name = "Collision Detect", Value = false });
        DigitalInputs.Add(new IOPoint { Address = "DI[10]", Name = "Home J1", Value = false });
        DigitalInputs.Add(new IOPoint { Address = "DI[11]", Name = "Home J2", Value = false });
        DigitalInputs.Add(new IOPoint { Address = "DI[12]", Name = "Home J3", Value = false });
        DigitalInputs.Add(new IOPoint { Address = "DI[13]", Name = "Home J4", Value = false });
        DigitalInputs.Add(new IOPoint { Address = "DI[14]", Name = "Home J5", Value = false });
        DigitalInputs.Add(new IOPoint { Address = "DI[15]", Name = "Home J6", Value = false });

        // Digital Outputs
        DigitalOutputs.Add(new IOPoint { Address = "DO[0]", Name = "Robot Ready", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[1]", Name = "Program Running", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[2]", Name = "Alarm Active", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[3]", Name = "Clamp Open", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[4]", Name = "Clamp Close", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[5]", Name = "Gas Valve", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[6]", Name = "Wire Feed", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[7]", Name = "Arc Start", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[8]", Name = "Torch Clean", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[9]", Name = "Signal Light Green", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[10]", Name = "Signal Light Yellow", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[11]", Name = "Signal Light Red", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[12]", Name = "Buzzer", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[13]", Name = "Spare 1", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[14]", Name = "Spare 2", Value = false });
        DigitalOutputs.Add(new IOPoint { Address = "DO[15]", Name = "Spare 3", Value = false });
    }

    [RelayCommand]
    private async Task ToggleOutput(IOPoint? ioPoint)
    {
        if (ioPoint == null) return;

        ioPoint.Value = !ioPoint.Value;

        // Send to Core
        await _ipcClient.SendCommandAsync("set_output", new
        {
            Address = ioPoint.Address,
            Value = ioPoint.Value
        });
    }

    [RelayCommand]
    private async Task RefreshIO()
    {
        // Request I/O status from Core
        await _ipcClient.SendCommandAsync("get_io_status", new { });
    }
}

public partial class IOPoint : ObservableObject
{
    [ObservableProperty]
    private string _address = "";

    [ObservableProperty]
    private string _name = "";

    [ObservableProperty]
    private bool _value;
}
