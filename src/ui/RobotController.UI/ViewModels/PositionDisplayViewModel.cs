using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using RobotController.Common.Services;
using Serilog;

namespace RobotController.UI.ViewModels;

/// <summary>
/// Position display modes (KUKA-inspired)
/// </summary>
public enum PositionDisplayMode
{
    Joint,  // J1-J6 in degrees
    World,  // X,Y,Z,Rx,Ry,Rz in World frame
    Base    // X,Y,Z,Rx,Ry,Rz in Active Base frame
}

/// <summary>
/// ViewModel for position display control
/// </summary>
public partial class PositionDisplayViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;

    [ObservableProperty]
    private PositionDisplayMode _displayMode = PositionDisplayMode.Joint;

    // Computed properties for RadioButton binding
    public bool IsJointMode => DisplayMode == PositionDisplayMode.Joint;
    public bool IsWorldMode => DisplayMode == PositionDisplayMode.World;
    public bool IsBaseMode => DisplayMode == PositionDisplayMode.Base;

    [ObservableProperty]
    private string _label1 = "J1";
    [ObservableProperty]
    private string _label2 = "J2";
    [ObservableProperty]
    private string _label3 = "J3";
    [ObservableProperty]
    private string _label4 = "J4";
    [ObservableProperty]
    private string _label5 = "J5";
    [ObservableProperty]
    private string _label6 = "J6";

    [ObservableProperty]
    private double _position1;
    [ObservableProperty]
    private double _position2;
    [ObservableProperty]
    private double _position3;
    [ObservableProperty]
    private double _position4;
    [ObservableProperty]
    private double _position5;
    [ObservableProperty]
    private double _position6;

    [ObservableProperty]
    private string _unit = "°";

    [ObservableProperty]
    private string _activeBaseId = "world";

    [ObservableProperty]
    private string _activeToolId = "tool_default";

    public PositionDisplayViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;

        _ipcClient.StatusReceived += OnStatusReceived;
    }

    partial void OnDisplayModeChanged(PositionDisplayMode value)
    {
        UpdateLabels();
        // Notify computed properties
        OnPropertyChanged(nameof(IsJointMode));
        OnPropertyChanged(nameof(IsWorldMode));
        OnPropertyChanged(nameof(IsBaseMode));
    }

    [RelayCommand]
    private void SetJointMode()
    {
        DisplayMode = PositionDisplayMode.Joint;
    }

    [RelayCommand]
    private void SetWorldMode()
    {
        DisplayMode = PositionDisplayMode.World;
    }

    [RelayCommand]
    private void SetBaseMode()
    {
        DisplayMode = PositionDisplayMode.Base;
    }

    private void UpdateLabels()
    {
        switch (DisplayMode)
        {
            case PositionDisplayMode.Joint:
                Label1 = "J1"; Label2 = "J2"; Label3 = "J3";
                Label4 = "J4"; Label5 = "J5"; Label6 = "J6";
                Unit = "°";
                break;

            case PositionDisplayMode.World:
                Label1 = "X"; Label2 = "Y"; Label3 = "Z";
                Label4 = "Rx"; Label5 = "Ry"; Label6 = "Rz";
                Unit = "mm/°";
                break;

            case PositionDisplayMode.Base:
                Label1 = "X"; Label2 = "Y"; Label3 = "Z";
                Label4 = "Rx"; Label5 = "Ry"; Label6 = "Rz";
                Unit = "mm/°";
                break;
        }
    }

    private void OnStatusReceived(object? sender, StatusPayload status)
    {
        ActiveBaseId = status.ActiveBaseId;
        ActiveToolId = status.ActiveToolId;

        switch (DisplayMode)
        {
            case PositionDisplayMode.Joint:
                UpdateFromList(status.Joints);
                break;

            case PositionDisplayMode.World:
                UpdateFromList(status.TcpPosition);
                break;

            case PositionDisplayMode.Base:
                UpdateFromList(status.TcpInBase);
                break;
        }
    }

    private void UpdateFromList(List<double> values)
    {
        if (values.Count >= 6)
        {
            Position1 = values[0];
            Position2 = values[1];
            Position3 = values[2];
            Position4 = values[3];
            Position5 = values[4];
            Position6 = values[5];
        }
    }
}
