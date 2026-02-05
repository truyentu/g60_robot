using System.Windows.Controls;

namespace RobotController.UI.Views.Pages;

/// <summary>
/// URDF Import Page - allows importing robot packages from URDF/xacro files
/// DataContext is set via XAML binding to MainViewModel.UrdfImportViewModel
/// </summary>
public partial class UrdfImportPage : UserControl
{
    public UrdfImportPage()
    {
        InitializeComponent();
    }
}
