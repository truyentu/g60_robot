using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using RobotController.UI.Services;
using RobotController.UI.ViewModels;
using Serilog;

namespace RobotController.UI.Views;

public partial class MainWindow : Window
{
    private MainViewModel? _viewModel;
    private readonly IViewportService _viewportService;

    // Object drag state
    private string? _selectedObjectId;
    private string? _previousHighlightId;
    private bool _isDraggingObject;
    private Point _dragStartMouse;
    private Point3D _dragStartWorldPoint;
    private Point3D _dragObjectStartPosition;

    public MainWindow(MainViewModel viewModel, IViewportService viewportService)
    {
        InitializeComponent();
        DataContext = viewModel;
        _viewModel = viewModel;
        _viewportService = viewportService;

        if (_viewModel != null)
        {
            _viewModel.RobotModelUpdated += OnRobotModelUpdated;
            _viewModel.PropertyChanged += OnViewModelPropertyChanged;

            // Listen for station setup object selection changes
            if (_viewModel.StationSetup != null)
            {
                _viewModel.StationSetup.SelectedObjectChanged += OnStationSetupSelectedObjectChanged;
            }
        }

        // Listen for pick mode changes to show/hide banner
        _viewportService.TcpPickModeChanged += OnTcpPickModeChanged;

        // Handle keyboard events for motion control
        PreviewKeyDown += MainWindow_PreviewKeyDown;
        PreviewKeyUp += MainWindow_PreviewKeyUp;

        // Add ViewportService's scene objects container as child of our XAML visual
        SceneObjectsVisual.Children.Add(_viewportService.SceneObjectsContainer);
    }

    private void MainWindow_PreviewKeyDown(object sender, KeyEventArgs e)
    {
        // Don't handle if typing in a text box
        if (e.OriginalSource is System.Windows.Controls.TextBox)
            return;

        _viewModel?.MotionControl?.HandleKeyDown(e.Key);
    }

    private void MainWindow_PreviewKeyUp(object sender, KeyEventArgs e)
    {
        if (e.OriginalSource is System.Windows.Controls.TextBox)
            return;

        _viewModel?.MotionControl?.HandleKeyUp(e.Key);
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

            // Scene objects are managed via SceneObjectsContainer (added in constructor)

            if (_viewModel?.BaseFrameAxesGroup != null)
            {
                BaseFrameAxesVisual.Content = _viewModel.BaseFrameAxesGroup;
            }
        });
    }

    private void OnViewModelPropertyChanged(object? sender, System.ComponentModel.PropertyChangedEventArgs e)
    {
        if (e.PropertyName == nameof(MainViewModel.SelectedNavIndex))
        {
            Dispatcher.Invoke(() =>
            {
                // Deselect when leaving Station Setup page
                if (_viewModel?.SelectedNavIndex != 8)
                {
                    DeselectObject();
                }
            });
        }
    }

    // ========================================================================
    // Mouse Handlers (TCP Pick + Station Setup Select + Drag)
    // ========================================================================

    private void Viewport3D_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        var mousePos = e.GetPosition(Viewport3D);

        // Priority 1: TCP picking mode
        if (_viewportService.IsPickingTcp)
        {
            var hits = Viewport3DHelper.FindHits(Viewport3D.Viewport, mousePos);

            if (hits.Count > 0)
            {
                var hitPoint = hits[0].Position;

                Log.Information("[MainWindow] TCP Pick hit at world ({X:F1}, {Y:F1}, {Z:F1})",
                    hitPoint.X, hitPoint.Y, hitPoint.Z);

                _viewportService.ProcessPickedPoint(hitPoint);
                e.Handled = true;
            }
            else
            {
                Log.Debug("[MainWindow] TCP Pick: no mesh hit at mouse position");
            }
            return;
        }

        // Priority 2: Station Setup object selection + drag initiation
        if (_viewModel?.SelectedNavIndex == 8)
        {
            var hits = Viewport3DHelper.FindHits(Viewport3D.Viewport, mousePos);
            foreach (var hit in hits)
            {
                var objectId = _viewportService.FindSceneObjectByVisual(hit.Visual);
                if (objectId != null)
                {
                    Log.Information("[MainWindow] Selected scene object: {Id}", objectId);
                    _viewModel.StationSetup?.SelectObjectById(objectId);

                    // Start drag
                    _isDraggingObject = true;
                    _dragStartMouse = mousePos;
                    _dragStartWorldPoint = hit.Position;

                    // Get current object position from scene data
                    var sceneObj = _viewportService.SceneObjects.Find(o => o.Id == objectId);
                    if (sceneObj != null)
                    {
                        _dragObjectStartPosition = new Point3D(sceneObj.X, sceneObj.Y, sceneObj.Z);
                    }

                    Viewport3D.CaptureMouse();
                    e.Handled = true;
                    return;
                }
            }

            // Clicked on empty space - deselect
            DeselectObject();
        }
    }

    private void Viewport3D_MouseMove(object sender, MouseEventArgs e)
    {
        if (!_isDraggingObject || _selectedObjectId == null) return;
        if (_viewModel?.SelectedNavIndex != 8) return;

        var mousePos = e.GetPosition(Viewport3D);

        var camera = Viewport3D.Camera as ProjectionCamera;
        if (camera == null) return;

        // Hold Shift = move along Z axis, otherwise move on XY plane
        bool moveZ = Keyboard.IsKeyDown(Key.LeftShift) || Keyboard.IsKeyDown(Key.RightShift);

        Vector3D planeNormal;
        if (moveZ)
        {
            // Use a vertical plane facing the camera (camera right direction cross Z)
            var lookDir = camera.LookDirection;
            var right = Vector3D.CrossProduct(lookDir, new Vector3D(0, 0, 1));
            if (right.LengthSquared < 0.0001)
                right = Vector3D.CrossProduct(lookDir, new Vector3D(0, 1, 0));
            right.Normalize();
            planeNormal = right;
        }
        else
        {
            // XY plane at object's Z height
            planeNormal = new Vector3D(0, 0, 1);
        }

        var planeOrigin = _dragStartWorldPoint;

        var currentWorldPoint = Viewport3D.Viewport.UnProject(mousePos, planeOrigin, planeNormal);
        if (currentWorldPoint == null) return;

        var startWorldPoint = Viewport3D.Viewport.UnProject(_dragStartMouse, planeOrigin, planeNormal);
        if (startWorldPoint == null) return;

        var delta = currentWorldPoint.Value - startWorldPoint.Value;

        double newX, newY, newZ;
        if (moveZ)
        {
            // Only apply Z component of delta
            newX = _dragObjectStartPosition.X;
            newY = _dragObjectStartPosition.Y;
            newZ = _dragObjectStartPosition.Z + delta.Z;
        }
        else
        {
            // Only apply XY components
            newX = _dragObjectStartPosition.X + delta.X;
            newY = _dragObjectStartPosition.Y + delta.Y;
            newZ = _dragObjectStartPosition.Z;
        }

        // Apply snap-to-grid if enabled
        if (_viewModel?.StationSetup != null)
        {
            newX = _viewModel.StationSetup.SnapValue(newX);
            newY = _viewModel.StationSetup.SnapValue(newY);
            newZ = _viewModel.StationSetup.SnapValue(newZ);
        }

        var sceneObj = _viewportService.SceneObjects.Find(o => o.Id == _selectedObjectId);
        if (sceneObj == null) return;

        _viewportService.UpdateSceneObjectTransform(
            _selectedObjectId,
            new[] { newX, newY, newZ },
            new[] { sceneObj.Rx, sceneObj.Ry, sceneObj.Rz },
            sceneObj.Scale);

        _viewModel.StationSetup?.UpdateObjectTransformFromGizmo(
            _selectedObjectId, newX, newY, newZ, sceneObj.Rx, sceneObj.Ry, sceneObj.Rz);
    }

    private void Viewport3D_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
    {
        if (_isDraggingObject)
        {
            _isDraggingObject = false;
            Viewport3D.ReleaseMouseCapture();
        }
    }

    // ========================================================================
    // Selection Lifecycle
    // ========================================================================

    private void OnStationSetupSelectedObjectChanged(object? sender, string? objectId)
    {
        Dispatcher.Invoke(() =>
        {
            if (objectId != null)
            {
                SelectObject(objectId);
            }
            else
            {
                DeselectObject();
            }
        });
    }

    private void SelectObject(string objectId)
    {
        // Unhighlight previous
        if (_previousHighlightId != null && _previousHighlightId != objectId)
        {
            _viewportService.UnhighlightSceneObject(_previousHighlightId);
        }

        var visual = _viewportService.GetSceneObjectVisual(objectId);
        if (visual == null) return;

        // Highlight selected
        _viewportService.HighlightSceneObject(objectId);
        _previousHighlightId = objectId;
        _selectedObjectId = objectId;
    }

    private void DeselectObject()
    {
        if (_previousHighlightId != null)
        {
            _viewportService.UnhighlightSceneObject(_previousHighlightId);
            _previousHighlightId = null;
        }

        _selectedObjectId = null;
        _isDraggingObject = false;
    }

    // ========================================================================
    // TCP Pick Mode
    // ========================================================================

    private void OnTcpPickModeChanged(object? sender, bool isPickMode)
    {
        Dispatcher.Invoke(() =>
        {
            TcpPickBanner.Visibility = isPickMode ? Visibility.Visible : Visibility.Collapsed;
        });
    }

    private void CancelTcpPick_Click(object sender, RoutedEventArgs e)
    {
        _viewportService.DisableTcpPicking();
    }

    // ========================================================================
    // Menu / Camera
    // ========================================================================

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
