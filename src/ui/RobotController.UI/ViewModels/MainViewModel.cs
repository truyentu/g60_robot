using CommunityToolkit.Mvvm.ComponentModel;
using System.Collections.ObjectModel;
using System.Windows.Media;

namespace RobotController.UI.ViewModels;

public partial class MainViewModel : ObservableObject
{
    [ObservableProperty]
    private string _connectionStatus = "Disconnected";

    [ObservableProperty]
    private Brush _connectionStatusColor = Brushes.Red;

    [ObservableProperty]
    private string _robotState = "IDLE";

    [ObservableProperty]
    private double _tcpX = 500.0;

    [ObservableProperty]
    private double _tcpY = 0.0;

    [ObservableProperty]
    private double _tcpZ = 600.0;

    public ObservableCollection<JointPosition> JointPositions { get; } = new()
    {
        new JointPosition { Name = "J1", Value = 0.0 },
        new JointPosition { Name = "J2", Value = -45.0 },
        new JointPosition { Name = "J3", Value = 90.0 },
        new JointPosition { Name = "J4", Value = 0.0 },
        new JointPosition { Name = "J5", Value = 45.0 },
        new JointPosition { Name = "J6", Value = 0.0 },
    };

    public MainViewModel()
    {
        // TODO: Initialize IPC client in IMPL_P1_02
        // TODO: Subscribe to status updates
    }
}

public class JointPosition
{
    public string Name { get; set; } = string.Empty;
    public double Value { get; set; }
}
