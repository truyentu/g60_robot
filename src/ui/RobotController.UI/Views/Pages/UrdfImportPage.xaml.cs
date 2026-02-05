using System.Windows.Controls;
using RobotController.UI.ViewModels.Pages;

namespace RobotController.UI.Views.Pages;

/// <summary>
/// URDF Import Page - allows importing robot packages from URDF/xacro files
/// </summary>
public partial class UrdfImportPage : UserControl
{
    public UrdfImportPage(UrdfImportViewModel viewModel)
    {
        InitializeComponent();
        DataContext = viewModel;
    }
}
