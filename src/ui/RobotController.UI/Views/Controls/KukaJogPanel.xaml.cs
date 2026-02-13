using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;
using RobotController.Common.Messages;
using RobotController.UI.ViewModels;

namespace RobotController.UI.Views.Controls;

public partial class KukaJogPanel : UserControl
{
    // Labels and value displays for each of the 6 rows
    private TextBlock[] _labels = null!;
    private TextBlock[] _values = null!;

    // Current coordinate mode
    private bool _isAxesMode = true;

    private static readonly string[] AxesLabels = { "A1", "A2", "A3", "A4", "A5", "A6" };
    private static readonly string[] CartesianLabels = { "X", "Y", "Z", "A", "B", "C" };

    public KukaJogPanel()
    {
        InitializeComponent();
        Loaded += OnLoaded;
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        _labels = new[] { Label0, Label1, Label2, Label3, Label4, Label5 };
        _values = new[] { Value0, Value1, Value2, Value3, Value4, Value5 };

        // Subscribe to status updates for live position readout
        if (DataContext is MainViewModel mainVm && mainVm.MotionControl is { } motionVm)
        {
            motionVm.PropertyChanged += (s, args) =>
            {
                if (args.PropertyName == nameof(MotionControlViewModel.TcpPosition))
                    UpdateValues();
            };

            // Also listen for joint position changes
            foreach (var joint in motionVm.Joints)
            {
                joint.PropertyChanged += (s, args) =>
                {
                    if (args.PropertyName == nameof(JointPositionViewModel.Position))
                        UpdateValues();
                };
            }
        }

        UpdateLabels();
        UpdateValues();
    }

    private MainViewModel? MainVm => DataContext as MainViewModel;
    private MotionControlViewModel? MotionVm => MainVm?.MotionControl;

    /// <summary>
    /// Coordinate system radio button checked (KUKA popup menu style)
    /// </summary>
    private void CoordRadio_Checked(object sender, RoutedEventArgs e)
    {
        if (sender is not RadioButton radio) return;

        var mode = radio.Content?.ToString() ?? "Axes";
        _isAxesMode = mode == "Axes";

        // Update MotionControlViewModel
        if (MotionVm != null)
        {
            MotionVm.JogJointMode = _isAxesMode;
            if (!_isAxesMode)
            {
                MotionVm.CoordinateSystem = mode;
            }
        }

        // Update the toggle button's icon and label via template
        UpdateToggleButtonDisplay(mode);

        // Close the popup
        CoordToggle.IsChecked = false;

        UpdateLabels();
        UpdateValues();
    }

    /// <summary>
    /// Update the icon and label shown on the toggle button
    /// </summary>
    private void UpdateToggleButtonDisplay(string mode)
    {
        // Find the named elements inside the ToggleButton's template
        if (CoordToggle.Template?.FindName("CoordIcon", CoordToggle) is TextBlock iconTb)
        {
            iconTb.Text = mode switch
            {
                "Axes" => "\u2699",
                "World" => "\u2600",
                "Base" => "\u25A6",
                "Tool" => "\u2692",
                _ => "\u2699"
            };
        }
        if (CoordToggle.Template?.FindName("CoordLabel", CoordToggle) is TextBlock labelTb)
        {
            labelTb.Text = mode;
        }
    }

    /// <summary>
    /// Update the 6 row labels based on current coordinate mode
    /// </summary>
    private void UpdateLabels()
    {
        if (_labels == null) return;

        var labels = _isAxesMode ? AxesLabels : CartesianLabels;
        for (int i = 0; i < 6; i++)
        {
            _labels[i].Text = labels[i];
        }
    }

    /// <summary>
    /// Update the 6 value displays from current robot state
    /// </summary>
    private void UpdateValues()
    {
        if (_values == null || MotionVm == null) return;

        try
        {
            Dispatcher.Invoke(() =>
            {
                if (_isAxesMode)
                {
                    // Show joint angles
                    for (int i = 0; i < 6 && i < MotionVm.Joints.Count; i++)
                    {
                        _values[i].Text = $"{MotionVm.Joints[i].Position:F2}\u00B0";
                    }
                }
                else
                {
                    // Show TCP position in current frame
                    var tcp = MotionVm.TcpPosition;
                    _values[0].Text = $"{tcp.X:F1} mm";
                    _values[1].Text = $"{tcp.Y:F1} mm";
                    _values[2].Text = $"{tcp.Z:F1} mm";
                    _values[3].Text = $"{tcp.Roll:F2}\u00B0";
                    _values[4].Text = $"{tcp.Pitch:F2}\u00B0";
                    _values[5].Text = $"{tcp.Yaw:F2}\u00B0";
                }
            });
        }
        catch { }
    }

    /// <summary>
    /// Jog minus button pressed — select this axis then start jog
    /// </summary>
    private void JogMinus_MouseDown(object sender, MouseButtonEventArgs e)
    {
        if (sender is not RepeatButton btn) return;
        if (!int.TryParse(btn.Tag?.ToString(), out int axisIndex)) return;

        SelectAxis(axisIndex);
        MotionVm?.JogStartCommand.Execute("-");
    }

    /// <summary>
    /// Jog plus button pressed — select this axis then start jog
    /// </summary>
    private void JogPlus_MouseDown(object sender, MouseButtonEventArgs e)
    {
        if (sender is not RepeatButton btn) return;
        if (!int.TryParse(btn.Tag?.ToString(), out int axisIndex)) return;

        SelectAxis(axisIndex);
        MotionVm?.JogStartCommand.Execute("+");
    }

    /// <summary>
    /// Mouse released — stop continuous jog
    /// </summary>
    private void Jog_MouseUp(object sender, MouseButtonEventArgs e)
    {
        if (MotionVm?.JogContinuous == true)
        {
            MotionVm?.JogStopCommand.Execute(null);
        }
    }

    /// <summary>
    /// Select the axis/joint for jogging based on current coordinate mode
    /// </summary>
    private void SelectAxis(int index)
    {
        if (MotionVm == null) return;

        if (_isAxesMode)
        {
            MotionVm.SelectJointCommand.Execute(index);
        }
        else
        {
            MotionVm.SelectCartesianAxisCommand.Execute(index);
        }
    }
}
