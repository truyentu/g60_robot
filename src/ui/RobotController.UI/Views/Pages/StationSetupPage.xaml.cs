using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using RobotController.UI.ViewModels;

namespace RobotController.UI.Views.Pages;

public partial class StationSetupPage : UserControl
{
    public StationSetupPage()
    {
        InitializeComponent();
    }

    private void ColorPreset_Click(object sender, MouseButtonEventArgs e)
    {
        if (sender is System.Windows.Controls.Border border && border.Tag is string colorHex)
        {
            var color = (Color)ColorConverter.ConvertFromString(colorHex);
            if (DataContext is StationSetupViewModel vm)
                vm.SetColorCommand.Execute(color);
        }
    }
}
