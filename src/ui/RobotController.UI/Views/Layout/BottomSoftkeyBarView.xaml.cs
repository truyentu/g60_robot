using System.Windows;
using System.Windows.Controls;
using RobotController.UI.ViewModels;

namespace RobotController.UI.Views.Layout;

public partial class BottomSoftkeyBarView : UserControl
{
    public BottomSoftkeyBarView()
    {
        InitializeComponent();
    }

    private MainViewModel? MainVm => DataContext as MainViewModel;

    private void EditMenuItem_Click(object sender, RoutedEventArgs e)
    {
        if (sender is not Button btn) return;
        var action = btn.Tag?.ToString() ?? "";

        EditMenuToggle.IsChecked = false;

        if (action == "Rename")
        {
            // Rename uses a dialog managed by MainViewModel
            MainVm?.ShowRenameProgramDialogCommand.Execute(null);
        }
        else
        {
            MainVm?.ProgramVm?.EditMenuAction(action);
        }
    }
}
