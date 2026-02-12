using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using RobotController.Common.Services;
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

    // Gizmo drag state (3D Jogging)
    private bool _isGizmoDragging;
    private string? _gizmoDragAxis;  // "tx","ty","tz","rx","ry","rz","txy","txz","tyz","txyz"
    private Point _gizmoDragStartMouse;
    private double[] _gizmoDragStartTcp = new double[6];
    private DateTime _lastIKCall = DateTime.MinValue;
    private bool _gizmoInitialized;
    private double[]? _ghostLastJoints; // Last IK result for execute on mouse up
    private Point3D? _planeDragLastPoint; // Last ray-plane intersection for plane drag
    private bool _ikInFlight; // Prevents overlapping IK requests
    private int _ikFailCount; // Consecutive IK failures — stop after threshold

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

        // Initialize TCP Gizmo
        InitializeTcpGizmo();

        // Initialize Ghost Robot (lazy - created when robot model loads)
        InitializeGhostRobot();

        // Initialize TCP Path Trace container
        TcpTraceVisual.Children.Add(_viewportService.TcpTraceContainer);
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

            // Re-initialize ghost robot now that robot model is loaded (mesh sharing needs _robot)
            InitializeGhostRobot();
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

                // Show/hide TCP gizmo on Manual Jog page (index 0) when 3D Jog enabled
                if (_gizmoInitialized)
                {
                    bool show3DJog = _viewModel?.SelectedNavIndex == 0 && (_viewModel?.Is3DJogEnabled ?? false);
                    _viewportService.ShowTcpGizmo(show3DJog);
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

        // Priority 2: TCP Gizmo drag (Manual Jog page, index == 0)
        if (_viewModel?.SelectedNavIndex == 0 && _viewportService.IsTcpGizmoVisible)
        {
            var hits = Viewport3DHelper.FindHits(Viewport3D.Viewport, mousePos);
            foreach (var hit in hits)
            {
                var gizmoAxis = _viewportService.HitTestGizmo(hit.Visual, hit.Model as GeometryModel3D);
                if (gizmoAxis != null)
                {
                    _isGizmoDragging = true;
                    _gizmoDragAxis = gizmoAxis;
                    _gizmoDragStartMouse = mousePos;
                    _ghostLastJoints = null;
                    _planeDragLastPoint = null;
                    _ikFailCount = 0;
                    _ikInFlight = false;

                    var currentTcp = _viewportService.GetCurrentTcpPose();
                    if (currentTcp != null)
                    {
                        Array.Copy(currentTcp, _gizmoDragStartTcp, 6);

                        // For plane drag or 3D free-drag: compute initial ray-plane intersection
                        if (gizmoAxis.Length >= 3) // "txy", "txz", "tyz", "txyz"
                        {
                            var camera = Viewport3D.Camera as ProjectionCamera;
                            if (camera != null)
                            {
                                var tcpOrigin = new Point3D(currentTcp[0], currentTcp[1], currentTcp[2]);
                                Vector3D planeNormal;
                                if (gizmoAxis == "txyz")
                                {
                                    // Camera-aligned plane: normal faces camera (negate LookDirection)
                                    planeNormal = -camera.LookDirection;
                                    planeNormal.Normalize();
                                }
                                else
                                {
                                    planeNormal = GetPlaneNormal(gizmoAxis);
                                }
                                var ray = Viewport3DHelper.GetRay(Viewport3D.Viewport, mousePos);
                                if (ray != null)
                                {
                                    _planeDragLastPoint = RayPlaneIntersection(ray.Origin, ray.Direction, tcpOrigin, planeNormal);
                                }
                            }
                        }
                    }

                    // Show ghost robot for preview (not in live mode)
                    if (_viewModel?.Is3DJogLiveMode != true)
                    {
                        _viewportService.ShowGhost(true);
                        _viewportService.SetGhostReachable(true);
                    }

                    Viewport3D.CaptureMouse();
                    e.Handled = true;
                    Log.Debug("[MainWindow] Gizmo drag start: axis={Axis}", gizmoAxis);
                    return;
                }
            }
        }

        // Priority 3: Station Setup object selection + drag initiation
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
        // Gizmo drag (3D Jogging)
        if (_isGizmoDragging && _gizmoDragAxis != null)
        {
            HandleGizmoDrag(e);
            return;
        }

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
        if (_isGizmoDragging)
        {
            _isGizmoDragging = false;
            _gizmoDragAxis = null;
            Viewport3D.ReleaseMouseCapture();

            // Apply ghost joints to real robot (execute the move) — skip in live mode (already applied)
            if (_ghostLastJoints != null && _viewModel?.Is3DJogLiveMode != true)
            {
                // Update viewport visual
                _viewportService.UpdateJointAngles(_ghostLastJoints);

                // Send joints to C++ Core firmware driver
                var jointsToApply = _ghostLastJoints;
                _ = Task.Run(async () =>
                {
                    var ipc = _viewModel?.GetIpcClient();
                    if (ipc?.IsConnected == true)
                    {
                        await ipc.SetJointsAsync(jointsToApply);
                    }
                });
            }
            _ghostLastJoints = null;

            // Hide ghost
            _viewportService.ShowGhost(false);

            // Clear drag status
            if (_viewModel != null)
                _viewModel.GizmoDragStatus = "";

            Log.Debug("[MainWindow] Gizmo drag ended - applied ghost joints");
            return;
        }

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

    private void MenuItem_ClearTcpTrace_Click(object sender, RoutedEventArgs e)
    {
        _viewportService.ClearTcpTrace();
    }

    // ========================================================================
    // TCP Gizmo (3D Interactive Jogging)
    // ========================================================================

    private void InitializeTcpGizmo()
    {
        var gizmoVisual = _viewportService.CreateTcpGizmo();
        TcpGizmoVisual.Children.Add(gizmoVisual);
        _viewportService.ShowTcpGizmo(false);
        _gizmoInitialized = true;
    }

    private void HandleGizmoDrag(MouseEventArgs e)
    {
        var mousePos = e.GetPosition(Viewport3D);
        var delta = mousePos - _gizmoDragStartMouse;

        // Skip if mouse hasn't moved enough (prevents jump on initial click)
        if (Math.Abs(delta.X) < 2 && Math.Abs(delta.Y) < 2)
            return;

        // Sensitivity: pixels to mm/degrees
        double translateSensitivity = 1.0; // 1 pixel = 1 mm
        double rotateSensitivity = 0.5;    // 1 pixel = 0.5 degrees

        double[] targetPose = new double[6];
        Array.Copy(_gizmoDragStartTcp, targetPose, 6);

        // Get gizmo orientation for frame-aware delta projection
        var gizmoOri = _viewportService.GetGizmoOrientation();
        bool isToolFrame = _viewportService.CurrentGizmoFrame == GizmoFrame.Tool;

        switch (_gizmoDragAxis)
        {
            case "tx":
            {
                double d = delta.X * translateSensitivity;
                if (isToolFrame)
                {
                    // Transform local X axis to world
                    targetPose[0] += d * gizmoOri.M11;
                    targetPose[1] += d * gizmoOri.M21;
                    targetPose[2] += d * gizmoOri.M31;
                }
                else
                {
                    targetPose[0] += d;
                }
                break;
            }
            case "ty":
            {
                double d = -delta.Y * translateSensitivity;
                if (isToolFrame)
                {
                    targetPose[0] += d * gizmoOri.M12;
                    targetPose[1] += d * gizmoOri.M22;
                    targetPose[2] += d * gizmoOri.M32;
                }
                else
                {
                    targetPose[1] += d;
                }
                break;
            }
            case "tz":
            {
                double d = delta.Y * translateSensitivity;
                if (isToolFrame)
                {
                    targetPose[0] += d * gizmoOri.M13;
                    targetPose[1] += d * gizmoOri.M23;
                    targetPose[2] += d * gizmoOri.M33;
                }
                else
                {
                    targetPose[2] += d;
                }
                break;
            }
            case "rx":
                targetPose[3] += delta.X * rotateSensitivity;
                break;
            case "ry":
                targetPose[4] += delta.X * rotateSensitivity;
                break;
            case "rz":
                targetPose[5] += delta.X * rotateSensitivity;
                break;
            case "txy":
            case "txz":
            case "tyz":
            case "txyz":
            {
                // Plane drag / 3D free-drag: use ray-plane intersection for accurate 3D movement
                var camera = Viewport3D.Camera as ProjectionCamera;
                if (camera != null && _planeDragLastPoint.HasValue)
                {
                    var tcpOrigin = new Point3D(_gizmoDragStartTcp[0], _gizmoDragStartTcp[1], _gizmoDragStartTcp[2]);
                    Vector3D planeNormal;
                    if (_gizmoDragAxis == "txyz")
                    {
                        // Camera-aligned plane: normal faces camera (negate LookDirection)
                        planeNormal = -camera.LookDirection;
                        planeNormal.Normalize();
                    }
                    else
                    {
                        planeNormal = GetPlaneNormal(_gizmoDragAxis);
                    }
                    var ray = Viewport3DHelper.GetRay(Viewport3D.Viewport, mousePos);
                    if (ray != null)
                    {
                        var currentPoint = RayPlaneIntersection(ray.Origin, ray.Direction, tcpOrigin, planeNormal);
                        if (currentPoint.HasValue && _planeDragLastPoint.HasValue)
                        {
                            var startPoint = _planeDragLastPoint.Value;
                            // Delta from initial intersection
                            var worldDelta = currentPoint.Value - startPoint;
                            targetPose[0] = _gizmoDragStartTcp[0] + worldDelta.X;
                            targetPose[1] = _gizmoDragStartTcp[1] + worldDelta.Y;
                            targetPose[2] = _gizmoDragStartTcp[2] + worldDelta.Z;
                        }
                    }
                }
                break;
            }
        }

        // Update drag status text
        if (_viewModel != null && _gizmoDragAxis != null)
        {
            string axisLabel = _gizmoDragAxis switch
            {
                "tx" => "X", "ty" => "Y", "tz" => "Z",
                "rx" => "Rx", "ry" => "Ry", "rz" => "Rz",
                "txy" => "XY", "txz" => "XZ", "tyz" => "YZ",
                "txyz" => "XYZ",
                _ => ""
            };
            bool isTranslate = _gizmoDragAxis.StartsWith("t");
            double deltaVal;
            if (!isTranslate)
            {
                int idx = _gizmoDragAxis[1] == 'x' ? 3 : _gizmoDragAxis[1] == 'y' ? 4 : 5;
                deltaVal = targetPose[idx] - _gizmoDragStartTcp[idx];
            }
            else if (_gizmoDragAxis.Length >= 3)
            {
                // Multi-axis: show total distance
                deltaVal = Math.Sqrt(Math.Pow(targetPose[0] - _gizmoDragStartTcp[0], 2) +
                            Math.Pow(targetPose[1] - _gizmoDragStartTcp[1], 2) +
                            Math.Pow(targetPose[2] - _gizmoDragStartTcp[2], 2));
            }
            else
            {
                // Single axis: show signed value
                int idx = _gizmoDragAxis[1] == 'x' ? 0 : _gizmoDragAxis[1] == 'y' ? 1 : 2;
                deltaVal = targetPose[idx] - _gizmoDragStartTcp[idx];
            }
            string unit = isTranslate ? "mm" : "°";
            _viewModel.GizmoDragStatus = $"Dragging {axisLabel}: {deltaVal:+0.0;-0.0;0.0} {unit}";
        }

        // Throttle IK calls: 30Hz normally, slower (3Hz) when IK is failing
        // This prevents UI freeze from IK spam but still allows recovery when user drags back
        int throttleMs = _ikFailCount >= 3 ? 300 : 33;
        if (!_ikInFlight && (DateTime.Now - _lastIKCall).TotalMilliseconds > throttleMs)
        {
            _lastIKCall = DateTime.Now;
            _ikInFlight = true;
            _ = ComputeAndApplyIKAsync(targetPose);
        }
    }

    private async Task ComputeAndApplyIKAsync(double[] targetPose, bool apply = false)
    {
        var ipcClient = _viewModel?.GetIpcClient();
        if (ipcClient == null || !ipcClient.IsConnected) { _ikInFlight = false; return; }

        var currentJoints = _viewportService.Robot?.JointAnglesDegrees;
        if (currentJoints == null) { _ikInFlight = false; return; }

        bool shouldApply = apply || (_viewModel?.Is3DJogLiveMode == true);

        try
        {
            var result = await ipcClient.ComputeIKAsync(targetPose, currentJoints, shouldApply);
            if (result?.Success == true)
            {
                _ikFailCount = 0; // Reset on success
                _ghostLastJoints = result.Joints;
                _ = Dispatcher.BeginInvoke(() =>
                {
                    // Live mode: update real robot visual during drag
                    if (_viewModel?.Is3DJogLiveMode == true)
                    {
                        _viewportService.UpdateJointAngles(result.Joints);
                    }
                    else
                    {
                        _viewportService.UpdateGhostJoints(result.Joints);
                        _viewportService.SetGhostReachable(true);
                    }
                });
            }
            else
            {
                _ikFailCount++;
                _ = Dispatcher.BeginInvoke(() =>
                {
                    _viewportService.SetGhostReachable(false);
                });
            }
        }
        catch (Exception ex)
        {
            _ikFailCount++;
            Log.Warning("[MainWindow] IK computation failed: {Error}", ex.Message);
        }
        finally
        {
            _ikInFlight = false;
        }
    }

    private void InitializeGhostRobot()
    {
        // Clear previous ghost (re-init after robot model reload)
        GhostRobotVisual.Children.Clear();

        var ghostVisual = _viewportService.CreateGhostRobot();
        if (ghostVisual != null)
        {
            GhostRobotVisual.Children.Add(ghostVisual);
        }
    }

    // ========================================================================
    // Plane Drag Helpers (ray-plane intersection for XY/XZ/YZ plane dragging)
    // ========================================================================

    private static Vector3D GetPlaneNormal(string planeAxis)
    {
        return planeAxis switch
        {
            "txy" => new Vector3D(0, 0, 1), // XY plane: normal = Z
            "txz" => new Vector3D(0, 1, 0), // XZ plane: normal = Y
            "tyz" => new Vector3D(1, 0, 0), // YZ plane: normal = X
            _ => new Vector3D(0, 0, 1)
        };
    }

    private static Point3D? RayPlaneIntersection(Point3D rayOrigin, Vector3D rayDir, Point3D planePoint, Vector3D planeNormal)
    {
        double denom = Vector3D.DotProduct(planeNormal, rayDir);
        if (Math.Abs(denom) < 1e-6)
            return null; // Ray parallel to plane

        var diff = planePoint - rayOrigin;
        double t = Vector3D.DotProduct(planeNormal, (Vector3D)diff) / denom;
        if (t < 0)
            return null; // Intersection behind camera

        return rayOrigin + t * rayDir;
    }
}
