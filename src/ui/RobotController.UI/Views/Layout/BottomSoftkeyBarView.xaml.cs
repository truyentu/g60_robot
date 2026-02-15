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
            // Route to Navigator when on Navigator page (navIndex=9)
            if (MainVm?.SelectedNavIndex == 9)
            {
                MainVm.NavigatorVm?.RenameCommand.Execute(null);
            }
            else
            {
                MainVm?.ShowRenameProgramDialogCommand.Execute(null);
            }
        }
        else if (action == "Copy")
        {
            if (MainVm?.SelectedNavIndex == 9)
            {
                MainVm.NavigatorVm?.CopyCommand.Execute(null);
            }
            else
            {
                MainVm?.ProgramVm?.EditMenuAction(action);
            }
        }
        else if (action == "ArchiveAll")
        {
            if (MainVm?.SelectedNavIndex == 9)
                MainVm.NavigatorVm?.ArchiveAllCommand.Execute(null);
        }
        else if (action == "ArchiveSelected")
        {
            if (MainVm?.SelectedNavIndex == 9)
                MainVm.NavigatorVm?.ArchiveSelectedCommand.Execute(null);
        }
        else if (action == "Restore")
        {
            if (MainVm?.SelectedNavIndex == 9)
                MainVm.NavigatorVm?.RestoreCommand.Execute(null);
        }
        else if (action == "Compare")
        {
            if (MainVm?.SelectedNavIndex == 9)
                MainVm.NavigatorVm?.CompareArchiveCommand.Execute(null);
        }
        else
        {
            MainVm?.ProgramVm?.EditMenuAction(action);
        }
    }
}
