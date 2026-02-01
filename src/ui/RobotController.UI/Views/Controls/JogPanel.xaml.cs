using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using RobotController.UI.ViewModels;

namespace RobotController.UI.Views.Controls;

public partial class JogPanel : UserControl
{
    public JogPanel()
    {
        InitializeComponent();
    }

    private MotionControlViewModel? ViewModel => DataContext as MotionControlViewModel;

    private void JogMinus_MouseDown(object sender, MouseButtonEventArgs e)
    {
        ViewModel?.JogStartCommand.Execute("-");
    }

    private void JogPlus_MouseDown(object sender, MouseButtonEventArgs e)
    {
        ViewModel?.JogStartCommand.Execute("+");
    }

    private void Jog_MouseUp(object sender, MouseButtonEventArgs e)
    {
        if (ViewModel?.JogContinuous == true)
        {
            ViewModel?.JogStopCommand.Execute(null);
        }
    }
}
