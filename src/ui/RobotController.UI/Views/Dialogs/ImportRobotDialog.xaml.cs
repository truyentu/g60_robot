using System.Windows;
using RobotController.UI.ViewModels.Dialogs;

namespace RobotController.UI.Views.Dialogs;

/// <summary>
/// Import Robot Dialog - allows importing robots from URDF/XACRO files
/// </summary>
public partial class ImportRobotDialog : Window
{
    public ImportRobotDialog()
    {
        InitializeComponent();
    }

    public ImportRobotDialog(ImportRobotViewModel viewModel) : this()
    {
        DataContext = viewModel;

        // Subscribe to close request
        viewModel.CloseRequested += (s, e) =>
        {
            DialogResult = viewModel.ParsedRobot != null;
            Close();
        };

        viewModel.ImportCompleted += (s, success) =>
        {
            DialogResult = success;
        };
    }
}
