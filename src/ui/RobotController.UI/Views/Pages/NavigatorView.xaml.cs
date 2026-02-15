using RobotController.UI.Models;
using RobotController.UI.ViewModels.Pages;
using System.Windows.Controls;
using System.Windows.Input;

namespace RobotController.UI.Views.Pages;

public partial class NavigatorView : UserControl
{
    public NavigatorView()
    {
        InitializeComponent();
    }

    private void TreeView_SelectedItemChanged(object sender, System.Windows.RoutedPropertyChangedEventArgs<object> e)
    {
        if (DataContext is NavigatorViewModel vm && e.NewValue is DirectoryNode node)
        {
            vm.SelectedDirectory = node;
        }
    }

    private void FileList_DoubleClick(object sender, MouseButtonEventArgs e)
    {
        if (DataContext is NavigatorViewModel vm && vm.SelectedFile is FileItem item)
        {
            vm.OnFileDoubleClick(item);
        }
    }
}
