using System.Windows;
using System.Windows.Media.Media3D;
using RobotController.UI.ViewModels;
using Serilog;

namespace RobotController.UI.Views;

public partial class MainWindow : Window
{
    private MainViewModel? _viewModel;

    public MainWindow(MainViewModel viewModel)
    {
        InitializeComponent();
        DataContext = viewModel;
        _viewModel = viewModel;

        if (_viewModel != null)
        {
            _viewModel.RobotModelUpdated += OnRobotModelUpdated;
            _viewModel.PropertyChanged += OnViewModelPropertyChanged;
        }
    }

    private void OnRobotModelUpdated(object? sender, EventArgs e)
    {
        Dispatcher.Invoke(() =>
        {
            if (_viewModel?.RobotModelGroup != null)
            {
                RobotModelVisual.Content = _viewModel.RobotModelGroup;
                Log.Debug("Robot model updated in viewport");
            }

            if (_viewModel?.TcpMarkerGroup != null)
            {
                TcpMarkerVisual.Content = _viewModel.TcpMarkerGroup;
            }
        });
    }

    private void OnViewModelPropertyChanged(object? sender, System.ComponentModel.PropertyChangedEventArgs e)
    {
        // Handle property changes if needed
    }

    private void MenuItem_Exit_Click(object sender, RoutedEventArgs e)
    {
        Application.Current.Shutdown();
    }

    private void MenuItem_ResetCamera_Click(object sender, RoutedEventArgs e)
    {
        Viewport3D.Camera.Position = new Point3D(1500, 1500, 1000);
        Viewport3D.Camera.LookDirection = new Vector3D(-1, -1, -0.5);
        Viewport3D.Camera.UpDirection = new Vector3D(0, 0, 1);
    }

    private void MenuItem_TopView_Click(object sender, RoutedEventArgs e)
    {
        Viewport3D.Camera.Position = new Point3D(0, 0, 2500);
        Viewport3D.Camera.LookDirection = new Vector3D(0, 0, -1);
        Viewport3D.Camera.UpDirection = new Vector3D(0, 1, 0);
    }

    private void MenuItem_FrontView_Click(object sender, RoutedEventArgs e)
    {
        Viewport3D.Camera.Position = new Point3D(0, -2500, 500);
        Viewport3D.Camera.LookDirection = new Vector3D(0, 1, 0);
        Viewport3D.Camera.UpDirection = new Vector3D(0, 0, 1);
    }

    private void MenuItem_SideView_Click(object sender, RoutedEventArgs e)
    {
        Viewport3D.Camera.Position = new Point3D(2500, 0, 500);
        Viewport3D.Camera.LookDirection = new Vector3D(-1, 0, 0);
        Viewport3D.Camera.UpDirection = new Vector3D(0, 0, 1);
    }

    private void MenuItem_About_Click(object sender, RoutedEventArgs e)
    {
        MessageBox.Show(
            "Robot Controller v1.0.0\n\n" +
            "6-DOF Robot Controller for MIG/MAG Welding\n\n" +
            "© 2026 All Rights Reserved",
            "About Robot Controller",
            MessageBoxButton.OK,
            MessageBoxImage.Information);
    }
}
