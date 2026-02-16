using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using CommunityToolkit.Mvvm.Messaging;
using Microsoft.Win32;
using RobotController.Common.Services;
using RobotController.UI.Helpers;
using RobotController.UI.Messages;
using RobotController.UI.Models;
using RobotController.UI.Services;
using Serilog;
using System.Collections.ObjectModel;
using System.IO;
using System.Text.RegularExpressions;
using System.Windows.Media.Media3D;

namespace RobotController.UI.ViewModels;

/// <summary>
/// A declaration parsed from a .dat file (DECL type name = value)
/// </summary>
public class DatDeclaration
{
    public string Type { get; set; } = "";   // E6POS, REAL, INT, BOOL, etc.
    public string Name { get; set; } = "";
    public string RawValue { get; set; } = ""; // The full value string ({X 100, ...} or "10.0")
}

/// <summary>
/// Variable info for the Watch panel
/// </summary>
public partial class WatchVariable : ObservableObject
{
    [ObservableProperty]
    private string _name = "";

    [ObservableProperty]
    private string _type = "";  // REAL, INT, E6POS, SYSTEM

    [ObservableProperty]
    private string _value = "";

    [ObservableProperty]
    private bool _isChanged;  // Flash highlight when value changes
}

/// <summary>
/// Point information for display
/// </summary>
public partial class PointInfo : ObservableObject
{
    [ObservableProperty]
    private string _name = "";

    [ObservableProperty]
    private double _x;

    [ObservableProperty]
    private double _y;

    [ObservableProperty]
    private double _z;

    [ObservableProperty]
    private double _rx;

    [ObservableProperty]
    private double _ry;

    [ObservableProperty]
    private double _rz;
}

/// <summary>
/// ViewModel for Program Editor - Phase 1: Touch-Up + Bi-directional Sync
/// </summary>
public partial class ProgramEditorViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;
    private readonly IViewportService _viewportService;

    [ObservableProperty]
    private string _programSource = @"; Robot Program - Welding Example
DECL E6POS pHome = {X 500.00, Y 0.00, Z 800.00, A 0.00, B 0.00, C 0.00}
DECL E6POS p1 = {X 600.00, Y 100.00, Z 500.00, A 0.00, B 90.00, C 0.00}
DECL E6POS p2 = {X 700.00, Y 100.00, Z 500.00, A 0.00, B 90.00, C 0.00}
DECL E6POS p3 = {X 700.00, Y 200.00, Z 500.00, A 0.00, B 90.00, C 0.00}

DEF WeldProgram()
    $TOOL = TOOL_DATA[1]
    $BASE = BASE_DATA[0]
    $VEL.CP = 0.5
    $APO.CDIS = 10

    ; Approach
    PTP pHome
    LIN p1

    ; Weld Seam 1
    $VEL.CP = 0.02
    ArcStart(Job_ID:=1)
    LIN p2 C_DIS
    LIN p3
    ArcEnd

    ; Retract
    $VEL.CP = 0.5
    LIN p1
    PTP pHome
END";

    [ObservableProperty]
    private string _programName = "WeldProgram";

    private string? _currentFilePath;
    private string? _currentDatPath;
    private List<DatDeclaration> _datDeclarations = new();

    [ObservableProperty]
    private int _currentLine = 0;

    [ObservableProperty]
    private string _executionState = "IDLE";

    [ObservableProperty]
    private bool _isRunning;

    [ObservableProperty]
    private bool _isPaused;

    /// <summary>When true, editor is locked for execution mode (Satzanwahl). False = editing mode.</summary>
    [ObservableProperty]
    private bool _isReadOnly;

    /// <summary>
    /// Line number of the Program Pointer (execution cursor).
    /// -1 = hidden (editing mode), >= 1 = visible at that line (execution mode).
    /// </summary>
    [ObservableProperty]
    private int _executionLine = -1;

    [ObservableProperty]
    private int _overridePercent = 100;

    [ObservableProperty]
    private ObservableCollection<string> _errors = new();

    [ObservableProperty]
    private ObservableCollection<PointInfo> _points = new();

    // ========================================================================
    // Touch-Up Properties
    // ========================================================================

    /// <summary>Caret offset in the editor (set by View)</summary>
    [ObservableProperty]
    private int _caretOffset;

    /// <summary>Name of the target variable on the current caret line (for display)</summary>
    [ObservableProperty]
    private string _caretTargetName = "";

    /// <summary>Whether the caret is on a motion instruction line</summary>
    [ObservableProperty]
    private bool _isOnMotionLine;

    /// <summary>Status message for Touch-Up result</summary>
    [ObservableProperty]
    private string _touchUpStatus = "";

    /// <summary>Whether to show the toast notification overlay</summary>
    [ObservableProperty]
    private bool _isToastVisible;

    /// <summary>Toast message content</summary>
    [ObservableProperty]
    private string _toastMessage = "";

    /// <summary>Toast type: "success" or "error"</summary>
    [ObservableProperty]
    private string _toastType = "success";

    private System.Timers.Timer? _toastTimer;

    /// <summary>Line number where caret is (1-based, set by View)</summary>
    [ObservableProperty]
    private int _caretLineNumber;

    // ========================================================================
    // Execution Simulator (local, when no IPC connection)
    // ========================================================================
    private System.Windows.Threading.DispatcherTimer? _simTimer;
    private int _simTotalLines;
    private bool _isSimulatorMode; // true when running local simulator (even if IPC connected)

    public ProgramEditorViewModel(IIpcClientService ipcClient, IViewportService viewportService)
    {
        _ipcClient = ipcClient;
        _viewportService = viewportService;

        // Subscribe to viewport -> editor messages (3D click -> scroll to line)
        WeakReferenceMessenger.Default.Register<ViewportSegmentClickedMessage>(this, (r, m) =>
        {
            ((ProgramEditorViewModel)r).OnViewportSegmentClicked(m.Value);
        });
    }

    // ========================================================================
    // Caret Tracking (called from View code-behind)
    // ========================================================================

    /// <summary>
    /// Called when caret position changes in the editor.
    /// Analyzes current line and sends messages for viewport sync.
    /// </summary>
    public void OnCaretPositionChanged(int lineNumber, string lineText)
    {
        CaretLineNumber = lineNumber;

        var targetName = RegexHelper.ExtractTargetName(lineText);

        if (targetName != null)
        {
            CaretTargetName = targetName;
            IsOnMotionLine = true;

            // Build coordinate cache and try to resolve position
            var cache = RegexHelper.BuildCoordinateCache(ProgramSource);
            var hasPos = cache.TryGetValue(targetName, out var pos);

            // Fallback: check Points collection (.dat file data)
            if (!hasPos)
            {
                var point = Points.FirstOrDefault(p =>
                    p.Name.Equals(targetName, StringComparison.OrdinalIgnoreCase));
                if (point != null)
                {
                    pos = (point.X, point.Y, point.Z);
                    hasPos = true;
                }
            }

            // Send message to viewport for 3D highlighting
            WeakReferenceMessenger.Default.Send(new EditorCaretOnMotionLineMessage(
                new EditorMotionLineInfo
                {
                    LineNumber = lineNumber,
                    TargetName = targetName,
                    X = hasPos ? pos.x : 0,
                    Y = hasPos ? pos.y : 0,
                    Z = hasPos ? pos.z : 0,
                    HasPosition = hasPos
                }));
        }
        else
        {
            CaretTargetName = "";
            IsOnMotionLine = false;

            // Clear 3D highlight
            WeakReferenceMessenger.Default.Send(new EditorCaretOffMotionLineMessage(lineNumber));
        }
    }

    // ========================================================================
    // Touch-Up Command (The Core Feature)
    // ========================================================================

    /// <summary>
    /// Modify Position (Touch-Up): Updates the definition of the target variable
    /// on the current caret line with the robot's current TCP position.
    /// </summary>
    [RelayCommand(CanExecute = nameof(CanModifyPosition))]
    private async Task ModifyPositionAsync()
    {
        try
        {
            TouchUpStatus = "";

            // 1. Get the target variable name from the current line
            if (string.IsNullOrEmpty(CaretTargetName))
            {
                TouchUpStatus = "No target variable on current line";
                return;
            }

            // 2. Get current robot TCP position from viewport service (or IPC)
            var tcpPose = _viewportService.GetCurrentTcpPose();
            if (tcpPose == null || tcpPose.Length < 6)
            {
                // Fallback: try to get from IPC status
                var status = await _ipcClient.GetStatusAsync();
                if (status?.TcpPosition == null || status.TcpPosition.Count < 6)
                {
                    TouchUpStatus = "Cannot get robot position";
                    return;
                }
                tcpPose = status.TcpPosition.ToArray();
            }

            // 3. Create new RobTarget from current pose
            var newTarget = RobTarget.FromPose(CaretTargetName, tcpPose);
            var newValue = newTarget.ToKrlString();

            // 4. Try to update in .dat declarations first (preferred after .dat split)
            var datDecl = _datDeclarations.FirstOrDefault(d =>
                d.Name.Equals(CaretTargetName, StringComparison.OrdinalIgnoreCase) && IsPointType(d.Type));

            if (datDecl != null)
            {
                // Update _datDeclarations RawValue
                datDecl.RawValue = newValue;

                // Update Points collection
                var point = Points.FirstOrDefault(p =>
                    p.Name.Equals(CaretTargetName, StringComparison.OrdinalIgnoreCase));
                if (point != null)
                {
                    point.X = tcpPose[0]; point.Y = tcpPose[1]; point.Z = tcpPose[2];
                    point.Rx = tcpPose[3]; point.Ry = tcpPose[4]; point.Rz = tcpPose[5];
                }

                SaveDatFile();
                await _ipcClient.SetPointAsync(CaretTargetName, tcpPose);
            }
            else
            {
                // Fallback: try to find in ProgramSource (legacy inline DECL)
                var definition = RegexHelper.FindVariableDefinition(ProgramSource, CaretTargetName);
                if (definition == null)
                {
                    TouchUpStatus = $"Definition for '{CaretTargetName}' not found";
                    return;
                }

                var (offset, length, oldValue) = definition.Value;
                var newSource = ProgramSource.Remove(offset, length).Insert(offset, newValue);
                ProgramSource = newSource;
            }

            // 5. Notify viewport that a point changed
            WeakReferenceMessenger.Default.Send(new PointDefinitionChangedMessage(CaretTargetName));

            var statusMsg = $"Updated {CaretTargetName}: ({tcpPose[0]:F1}, {tcpPose[1]:F1}, {tcpPose[2]:F1})";
            TouchUpStatus = statusMsg;
            ShowToast(statusMsg, "success");
            Log.Information("Touch-Up: Updated {Var} to ({X:F1}, {Y:F1}, {Z:F1})",
                CaretTargetName, tcpPose[0], tcpPose[1], tcpPose[2]);
        }
        catch (Exception ex)
        {
            TouchUpStatus = $"Touch-Up failed: {ex.Message}";
            ShowToast($"Touch-Up failed: {ex.Message}", "error", 5000);
            Log.Error(ex, "Touch-Up failed for {Var}", CaretTargetName);
        }
    }

    private bool CanModifyPosition() => IsOnMotionLine && !string.IsNullOrEmpty(CaretTargetName);

    private void ShowToast(string message, string type = "success", int durationMs = 3000)
    {
        _toastTimer?.Stop();
        _toastTimer?.Dispose();

        ToastMessage = message;
        ToastType = type;
        IsToastVisible = true;

        _toastTimer = new System.Timers.Timer(durationMs);
        _toastTimer.AutoReset = false;
        _toastTimer.Elapsed += (s, e) =>
        {
            System.Windows.Application.Current?.Dispatcher.Invoke(() =>
            {
                IsToastVisible = false;
                ToastMessage = "";
            });
        };
        _toastTimer.Start();
    }

    partial void OnIsOnMotionLineChanged(bool value) => ModifyPositionCommand.NotifyCanExecuteChanged();
    partial void OnCaretTargetNameChanged(string value) => ModifyPositionCommand.NotifyCanExecuteChanged();

    partial void OnProgramSourceChanged(string value)
    {
        // Auto-clear path preview when program source changes
        if (IsPathPreviewVisible)
        {
            _viewportService.ClearPathPreview();
            IsPathPreviewVisible = false;
        }
    }

    // ========================================================================
    // Viewport -> Editor: Scroll to line on 3D segment click
    // ========================================================================

    /// <summary>
    /// Scroll to line number requested from viewport interaction.
    /// The View code-behind subscribes to ScrollToLineRequested event.
    /// </summary>
    [ObservableProperty]
    private int _scrollToLineRequest;

    private void OnViewportSegmentClicked(int lineNumber)
    {
        if (lineNumber > 0)
        {
            ScrollToLineRequest = lineNumber;
        }
    }

    // ========================================================================
    // Existing Program Execution Commands
    // ========================================================================

    [RelayCommand]
    private async Task LoadProgramAsync()
    {
        try
        {
            // Send merged source (.dat DECL + .src) to C++ Core
            var mergedSource = GetMergedSource();
            var response = await _ipcClient.LoadProgramAsync(mergedSource);
            Errors.Clear();

            if (response == null)
            {
                Errors.Add("Failed to communicate with Core (timeout or connection lost)");
                Log.Warning("LoadProgram: response is null — IPC timeout or disconnected");
                return;
            }

            if (!response.Success)
            {
                Errors.Add(response.Error ?? "Unknown parse error");
                Log.Warning("LoadProgram failed: {Error}", response.Error);
            }
            else
            {
                ProgramName = response.ProgramName;
                Log.Information("Program loaded: {Name}", ProgramName);
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to load program");
            Errors.Add(ex.Message);
        }
    }

    [RelayCommand]
    private async Task RunProgramAsync()
    {
        // Auto-enable TCP trace: clear previous trace and start recording
        _viewportService.ClearTcpTrace();
        _viewportService.SetTcpTraceEnabled(true);
        WeakReferenceMessenger.Default.Send(new TcpTraceAutoToggleMessage(true));

        if (_ipcClient.IsConnected)
        {
            await LoadProgramAsync();
            if (Errors.Count > 0)
            {
                // Core load failed — fallback to local simulator
                Errors.Clear();
                Log.Warning("Core load failed, falling back to local simulator");
                SimulatorStart();
                return;
            }

            await _ipcClient.RunProgramAsync();
            IsRunning = true;
            IsPaused = false;
            ExecutionState = "RUNNING";
            _ = PollProgramStateAsync();
        }
        else
        {
            // Local simulation mode
            SimulatorStart();
        }
    }

    [RelayCommand]
    private async Task StepProgramAsync()
    {
        if (_ipcClient.IsConnected)
        {
            if (!IsRunning)
            {
                await LoadProgramAsync();
                if (Errors.Count > 0)
                {
                    // Core load failed — fallback to local simulator
                    Errors.Clear();
                    Log.Warning("Core load failed, falling back to local simulator for step");
                    SimulatorStep();
                    return;
                }
                IsRunning = true;
            }

            var response = await _ipcClient.StepProgramAsync();
            if (response != null)
            {
                ExecutionState = response.State;
                CurrentLine = response.CurrentLine;
                if (response.State == "COMPLETED" || response.State == "ERROR")
                    IsRunning = false;

                // Auto-refresh watch variables after step
                if (IsWatchPanelVisible)
                    _ = RefreshVariablesAsync();
            }
        }
        else
        {
            // Local simulation: single step
            SimulatorStep();
        }
    }

    [RelayCommand]
    private async Task PauseProgramAsync()
    {
        // Always stop local simulator timer (may be running via fallback)
        _simTimer?.Stop();

        if (_ipcClient.IsConnected)
        {
            await _ipcClient.PauseProgramAsync();
        }
        IsPaused = true;
        ExecutionState = "PAUSED";
        ShowToast("Program paused");
    }

    [RelayCommand]
    private async Task ResumeProgramAsync()
    {
        if (_isSimulatorMode)
        {
            _simTimer?.Start();
        }
        else if (_ipcClient.IsConnected)
        {
            await _ipcClient.RunProgramAsync();
        }
        IsPaused = false;
        ExecutionState = "RUNNING";
    }

    [RelayCommand]
    private async Task StopProgramAsync()
    {
        // Always stop local simulator timer (may be running via fallback)
        _simTimer?.Stop();

        if (_ipcClient.IsConnected)
        {
            await _ipcClient.StopProgramAsync();
        }
        IsRunning = false;
        IsPaused = false;
        _isSimulatorMode = false;
        ExecutionState = "STOPPED";
        ShowToast("Program stopped");
        Log.Information("Program stopped at line {Line}", ExecutionLine);
    }

    [RelayCommand]
    private async Task ResetProgramAsync()
    {
        // Always stop local simulator timer (may be running via fallback)
        _simTimer?.Stop();

        if (_ipcClient.IsConnected)
        {
            await _ipcClient.ResetProgramAsync();
        }
        IsRunning = false;
        IsPaused = false;
        _isSimulatorMode = false;
        ExecutionState = "IDLE";
        ExecutionLine = IsReadOnly ? FindFirstExecutableLine() : -1;
        CurrentLine = Math.Max(ExecutionLine, 0);
        ShowToast("Program reset");
        Log.Information("Program reset to line {Line}", ExecutionLine);
    }

    // ========================================================================
    // Local Execution Simulator
    // ========================================================================

    private void SimulatorStart()
    {
        if (!IsReadOnly)
        {
            ShowToast("Select a program first (Satzanwahl)", "error");
            return;
        }

        _simTotalLines = ProgramSource.Split('\n').Length;

        if (ExecutionLine <= 0)
            ExecutionLine = FindFirstExecutableLine();

        IsRunning = true;
        IsPaused = false;
        _isSimulatorMode = true;
        ExecutionState = "RUNNING";

        // Calculate interval based on override (100% = 500ms, 10% = 5000ms)
        var intervalMs = 500.0 / (OverridePercent / 100.0);

        _simTimer?.Stop();
        _simTimer = new System.Windows.Threading.DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(intervalMs)
        };
        _simTimer.Tick += SimulatorTick;
        _simTimer.Start();

        ShowToast("Program running (simulation)");
        Log.Information("Simulator started at line {Line}, override {Pct}%", ExecutionLine, OverridePercent);
    }

    private void SimulatorStep()
    {
        if (!IsReadOnly)
        {
            ShowToast("Select a program first (Satzanwahl)", "error");
            return;
        }

        _simTotalLines = ProgramSource.Split('\n').Length;

        if (ExecutionLine <= 0)
            ExecutionLine = FindFirstExecutableLine();

        if (!IsRunning)
        {
            IsRunning = true;
            ExecutionState = "STEP";
        }

        AdvanceExecutionLine();
    }

    private void SimulatorTick(object? sender, EventArgs e)
    {
        AdvanceExecutionLine();
    }

    private void AdvanceExecutionLine()
    {
        var lines = ProgramSource.Split('\n');
        var nextLine = ExecutionLine + 1;

        // Skip blank lines and comments
        while (nextLine <= lines.Length)
        {
            if (nextLine > lines.Length)
                break;

            var trimmed = lines[nextLine - 1].TrimStart();
            if (!string.IsNullOrWhiteSpace(trimmed) && !trimmed.StartsWith(";"))
                break;
            nextLine++;
        }

        // Check for END or past last line
        if (nextLine > lines.Length)
        {
            SimulatorFinish();
            return;
        }

        var currentTrimmed = lines[nextLine - 1].TrimStart();
        if (currentTrimmed.StartsWith("END", StringComparison.OrdinalIgnoreCase) &&
            !currentTrimmed.StartsWith("ENDDAT", StringComparison.OrdinalIgnoreCase) &&
            !currentTrimmed.StartsWith("ENDLOOP", StringComparison.OrdinalIgnoreCase) &&
            !currentTrimmed.StartsWith("ENDIF", StringComparison.OrdinalIgnoreCase) &&
            !currentTrimmed.StartsWith("ENDWHILE", StringComparison.OrdinalIgnoreCase) &&
            !currentTrimmed.StartsWith("ENDFOR", StringComparison.OrdinalIgnoreCase))
        {
            ExecutionLine = nextLine;
            CurrentLine = nextLine;
            SimulatorFinish();
            return;
        }

        ExecutionLine = nextLine;
        CurrentLine = nextLine;
        ExecutionState = IsRunning && !IsPaused ? "RUNNING" : "STEP";
    }

    private void SimulatorFinish()
    {
        _simTimer?.Stop();
        IsRunning = false;
        IsPaused = false;
        _isSimulatorMode = false;
        ExecutionState = "P_END";
        ShowToast("Program Finished", "success", 3000);
        Log.Information("Simulator finished at line {Line}", ExecutionLine);
    }

    [RelayCommand]
    private async Task BackwardStepAsync()
    {
        try
        {
            var response = await _ipcClient.BackwardStepAsync();
            if (response == null)
            {
                ShowToast("Failed to communicate with Core", "error");
                return;
            }

            if (response.Success)
            {
                ExecutionState = response.State;
                CurrentLine = response.CurrentLine;
                ShowToast($"Stepped back to line {response.CurrentLine}");
            }
            else
            {
                ShowToast(response.Error ?? "No steps to undo", "error");
            }
        }
        catch (Exception ex)
        {
            ShowToast($"Backward step failed: {ex.Message}", "error");
            Log.Error(ex, "Backward step failed");
        }
    }

    [RelayCommand]
    private async Task BlockSelectAsync()
    {
        try
        {
            if (CaretLineNumber <= 0)
            {
                ShowToast("Place caret on a line first", "error");
                return;
            }

            // Load program first if not running
            if (!IsRunning)
            {
                await LoadProgramAsync();
                if (Errors.Count > 0) return;
                IsRunning = true;
            }

            var response = await _ipcClient.BlockSelectAsync(CaretLineNumber);
            if (response == null)
            {
                ShowToast("Failed to communicate with Core", "error");
                return;
            }

            if (response.Success)
            {
                ExecutionState = response.State;
                CurrentLine = response.CurrentLine;
                IsPaused = response.State == "PAUSED";
                ShowToast($"Block select: line {response.CurrentLine}");
            }
            else
            {
                ShowToast(response.Error ?? "Block select failed", "error");
            }
        }
        catch (Exception ex)
        {
            ShowToast($"Block select failed: {ex.Message}", "error");
            Log.Error(ex, "Block select failed");
        }
    }

    // ========================================================================
    // File Open / Save
    // ========================================================================

    /// <summary>
    /// Open a .src file from Navigator — sets _currentFilePath so Save works.
    /// Also loads the matching .dat file path for future reference.
    /// </summary>
    public void OpenFromNavigator(string srcPath)
    {
        if (!File.Exists(srcPath)) return;

        _currentFilePath = srcPath;
        ProgramSource = File.ReadAllText(srcPath);
        ProgramName = Path.GetFileNameWithoutExtension(srcPath);
        Errors.Clear();
        LoadDatFile(srcPath);
        ShowToast($"Opened: {Path.GetFileName(srcPath)}");
        Log.Information("Opened from Navigator: {Path}", srcPath);
    }

    /// <summary>
    /// Open file for editing (Double-Click). Editable, no program pointer.
    /// </summary>
    public void OpenForEditing(string srcPath)
    {
        if (!File.Exists(srcPath)) return;

        _currentFilePath = srcPath;
        ProgramSource = File.ReadAllText(srcPath);
        ProgramName = Path.GetFileNameWithoutExtension(srcPath);
        Errors.Clear();
        LoadDatFile(srcPath);

        // Editing mode: editable, no execution pointer
        IsReadOnly = false;
        ExecutionLine = -1;

        ShowToast($"Editing: {Path.GetFileName(srcPath)}");
        Log.Information("Opened for editing: {Path}", srcPath);
    }

    /// <summary>
    /// Open file for execution (Satzanwahl/Select). Read-only with program pointer.
    /// Finds first executable line (DEF or INI) and sets pointer there.
    /// </summary>
    public void OpenForExecution(string srcPath)
    {
        if (!File.Exists(srcPath)) return;

        _currentFilePath = srcPath;
        ProgramSource = File.ReadAllText(srcPath);
        ProgramName = Path.GetFileNameWithoutExtension(srcPath);
        Errors.Clear();
        LoadDatFile(srcPath);

        // Execution mode: locked, show program pointer
        IsReadOnly = true;
        ExecutionLine = FindFirstExecutableLine();
        CurrentLine = ExecutionLine;

        ShowToast($"Selected: {Path.GetFileName(srcPath)}", "success");
        Log.Information("Opened for execution (Satzanwahl): {Path}, PC at line {Line}", srcPath, ExecutionLine);
    }

    /// <summary>
    /// Find the first executable line (DEF xxx or INI) in ProgramSource.
    /// Falls back to line 1 if not found.
    /// </summary>
    private int FindFirstExecutableLine()
    {
        var lines = ProgramSource.Split('\n');
        for (int i = 0; i < lines.Length; i++)
        {
            var trimmed = lines[i].TrimStart();
            if (trimmed.StartsWith("DEF ", StringComparison.OrdinalIgnoreCase) ||
                trimmed.StartsWith("INI", StringComparison.OrdinalIgnoreCase))
            {
                return i + 1; // 1-based line number
            }
        }
        return 1;
    }

    [RelayCommand]
    private void OpenFile()
    {
        var dlg = new OpenFileDialog
        {
            Title = "Open Robot Program",
            Filter = "KRL Source (*.src)|*.src|KRL Data (*.dat)|*.dat|All Files (*.*)|*.*",
            DefaultExt = ".src"
        };

        if (dlg.ShowDialog() == true)
        {
            try
            {
                ProgramSource = File.ReadAllText(dlg.FileName);
                _currentFilePath = dlg.FileName;
                ProgramName = Path.GetFileNameWithoutExtension(dlg.FileName);
                ShowToast($"Opened: {Path.GetFileName(dlg.FileName)}");
                Log.Information("Opened program file: {Path}", dlg.FileName);
            }
            catch (Exception ex)
            {
                ShowToast($"Open failed: {ex.Message}", "error", 5000);
                Log.Error(ex, "Failed to open file {Path}", dlg.FileName);
            }
        }
    }

    [RelayCommand]
    private void SaveFile()
    {
        if (string.IsNullOrEmpty(_currentFilePath))
        {
            SaveFileAs();
            return;
        }

        try
        {
            File.WriteAllText(_currentFilePath, ProgramSource);
            ShowToast($"Saved: {Path.GetFileName(_currentFilePath)}");
            Log.Information("Saved program file: {Path}", _currentFilePath);
        }
        catch (Exception ex)
        {
            ShowToast($"Save failed: {ex.Message}", "error", 5000);
            Log.Error(ex, "Failed to save file {Path}", _currentFilePath);
        }
    }

    [RelayCommand]
    private void SaveFileAs()
    {
        var dlg = new SaveFileDialog
        {
            Title = "Save Robot Program",
            Filter = "KRL Source (*.src)|*.src|KRL Data (*.dat)|*.dat|All Files (*.*)|*.*",
            DefaultExt = ".src",
            FileName = ProgramName
        };

        if (dlg.ShowDialog() == true)
        {
            try
            {
                File.WriteAllText(dlg.FileName, ProgramSource);
                _currentFilePath = dlg.FileName;
                ProgramName = Path.GetFileNameWithoutExtension(dlg.FileName);
                ShowToast($"Saved: {Path.GetFileName(dlg.FileName)}");
                Log.Information("Saved program file: {Path}", dlg.FileName);
            }
            catch (Exception ex)
            {
                ShowToast($"Save failed: {ex.Message}", "error", 5000);
                Log.Error(ex, "Failed to save file {Path}", dlg.FileName);
            }
        }
    }

    // ========================================================================
    // .DAT File Loading & Saving
    // ========================================================================

    /// <summary>
    /// Load and parse the .dat file matching a .src file path.
    /// Extracts DECL statements and populates the Points collection.
    /// </summary>
    private void LoadDatFile(string srcPath)
    {
        _currentDatPath = Path.ChangeExtension(srcPath, ".dat");
        _datDeclarations.Clear();
        Points.Clear();

        if (!File.Exists(_currentDatPath))
        {
            Log.Debug("No .dat file found for {Src}", srcPath);
            return;
        }

        try
        {
            var datContent = File.ReadAllText(_currentDatPath);
            ParseDatContent(datContent);
            Log.Information("Loaded .dat file: {Path} ({Count} declarations)", _currentDatPath, _datDeclarations.Count);
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to load .dat file: {Path}", _currentDatPath);
        }
    }

    /// <summary>
    /// Parse DECL statements from .dat file content.
    /// Populates _datDeclarations and Points collection.
    /// </summary>
    private void ParseDatContent(string datContent)
    {
        // Match: DECL TYPE NAME = VALUE (where VALUE can be an aggregate or scalar)
        var declRegex = new Regex(
            @"^\s*DECL\s+(\w+)\s+(\w+)\s*=\s*(.+)$",
            RegexOptions.Multiline | RegexOptions.IgnoreCase);

        foreach (Match match in declRegex.Matches(datContent))
        {
            var type = match.Groups[1].Value.ToUpperInvariant();
            var name = match.Groups[2].Value;
            var rawValue = match.Groups[3].Value.Trim();

            _datDeclarations.Add(new DatDeclaration
            {
                Type = type,
                Name = name,
                RawValue = rawValue
            });

            // Extract point coordinates for E6POS/POS/FRAME types
            if (type is "E6POS" or "POS" or "FRAME" or "E6AXIS" or "AXIS")
            {
                var point = ParsePointFromAggregate(name, type, rawValue);
                if (point != null)
                    Points.Add(point);
            }
        }
    }

    /// <summary>
    /// Parse a KRL aggregate string into a PointInfo.
    /// Handles both Cartesian {X, Y, Z, A, B, C} and Joint {A1..A6} formats.
    /// </summary>
    private static PointInfo? ParsePointFromAggregate(string name, string type, string rawValue)
    {
        var fields = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
        var fieldRegex = new Regex(@"(\w+)\s+([-\d.eE+]+)");

        foreach (Match fm in fieldRegex.Matches(rawValue))
        {
            if (double.TryParse(fm.Groups[2].Value,
                System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture,
                out var val))
            {
                fields[fm.Groups[1].Value] = val;
            }
        }

        if (fields.Count == 0) return null;

        if (type is "E6AXIS" or "AXIS")
        {
            // Joint angles: A1..A6 → map to X=A1, Y=A2, Z=A3, Rx=A4, Ry=A5, Rz=A6
            return new PointInfo
            {
                Name = name,
                X = fields.GetValueOrDefault("A1"),
                Y = fields.GetValueOrDefault("A2"),
                Z = fields.GetValueOrDefault("A3"),
                Rx = fields.GetValueOrDefault("A4"),
                Ry = fields.GetValueOrDefault("A5"),
                Rz = fields.GetValueOrDefault("A6")
            };
        }

        // Cartesian: X, Y, Z, A, B, C
        return new PointInfo
        {
            Name = name,
            X = fields.GetValueOrDefault("X"),
            Y = fields.GetValueOrDefault("Y"),
            Z = fields.GetValueOrDefault("Z"),
            Rx = fields.GetValueOrDefault("A"),
            Ry = fields.GetValueOrDefault("B"),
            Rz = fields.GetValueOrDefault("C")
        };
    }

    /// <summary>
    /// Build merged source: DECL lines from .dat prepended to .src content.
    /// Used when sending LOAD_PROGRAM to C++ Core.
    /// </summary>
    public string GetMergedSource()
    {
        if (_datDeclarations.Count == 0)
            return ProgramSource;

        var declLines = new System.Text.StringBuilder();
        foreach (var decl in _datDeclarations)
        {
            // Use CONST for point types so C++ Executor extracts coordinates into m_points
            var keyword = IsPointType(decl.Type) ? "CONST" : "DECL";
            declLines.AppendLine($"{keyword} {decl.Type} {decl.Name} = {decl.RawValue}");
        }
        declLines.AppendLine();

        return declLines.ToString() + ProgramSource;
    }

    private static bool IsPointType(string type)
    {
        return type.Equals("E6POS", StringComparison.OrdinalIgnoreCase) ||
               type.Equals("E6AXIS", StringComparison.OrdinalIgnoreCase) ||
               type.Equals("POS", StringComparison.OrdinalIgnoreCase) ||
               type.Equals("AXIS", StringComparison.OrdinalIgnoreCase) ||
               type.Equals("FRAME", StringComparison.OrdinalIgnoreCase);
    }

    /// <summary>
    /// Save the current Points collection + other DECL back to the .dat file.
    /// </summary>
    private void SaveDatFile()
    {
        if (string.IsNullOrEmpty(_currentDatPath)) return;

        try
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"DEFDAT {ProgramName}");
            sb.AppendLine("  ;FOLD EXTERNAL DECLARATIONS; %{PE}");
            sb.AppendLine("  ;ENDFOLD");
            sb.AppendLine();

            // Write point declarations (E6POS/E6AXIS) from Points collection
            foreach (var point in Points)
            {
                // Determine if this was originally an E6AXIS type
                var originalDecl = _datDeclarations.FirstOrDefault(
                    d => d.Name.Equals(point.Name, StringComparison.OrdinalIgnoreCase));
                var type = originalDecl?.Type ?? "E6POS";

                if (type is "E6AXIS" or "AXIS")
                {
                    sb.AppendLine($"  DECL {type} {point.Name}={{A1 {point.X:F1}, A2 {point.Y:F1}, A3 {point.Z:F1}, A4 {point.Rx:F1}, A5 {point.Ry:F1}, A6 {point.Rz:F1}}}");
                }
                else
                {
                    sb.AppendLine($"  DECL {type} {point.Name}={{X {point.X:F2}, Y {point.Y:F2}, Z {point.Z:F2}, A {point.Rx:F2}, B {point.Ry:F2}, C {point.Rz:F2}}}");
                }
            }

            // Write non-point declarations (REAL, INT, BOOL, etc.)
            foreach (var decl in _datDeclarations)
            {
                if (decl.Type is not ("E6POS" or "POS" or "FRAME" or "E6AXIS" or "AXIS"))
                {
                    sb.AppendLine($"  DECL {decl.Type} {decl.Name}={decl.RawValue}");
                }
            }

            sb.AppendLine();
            sb.AppendLine("ENDDAT");

            File.WriteAllText(_currentDatPath, sb.ToString());
            Log.Information("Saved .dat file: {Path}", _currentDatPath);
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to save .dat file: {Path}", _currentDatPath);
            ShowToast($"Failed to save .dat: {ex.Message}", "error", 5000);
        }
    }

    /// <summary>
    /// Ensure a .dat file exists for the current program. Creates one if missing.
    /// </summary>
    private void EnsureDatFile()
    {
        if (string.IsNullOrEmpty(_currentDatPath)) return;
        if (File.Exists(_currentDatPath)) return;

        // Create minimal .dat file
        SaveDatFile();
    }

    [RelayCommand]
    private async Task TeachPointAsync()
    {
        try
        {
            // Get current robot position
            var status = await _ipcClient.GetStatusAsync();
            if (status == null) return;

            // Generate point name
            string pointName = GenerateNextPointName();

            var point = new PointInfo
            {
                Name = pointName,
                X = status.TcpPosition.Count > 0 ? status.TcpPosition[0] : 0,
                Y = status.TcpPosition.Count > 1 ? status.TcpPosition[1] : 0,
                Z = status.TcpPosition.Count > 2 ? status.TcpPosition[2] : 0,
                Rx = status.TcpPosition.Count > 3 ? status.TcpPosition[3] : 0,
                Ry = status.TcpPosition.Count > 4 ? status.TcpPosition[4] : 0,
                Rz = status.TcpPosition.Count > 5 ? status.TcpPosition[5] : 0
            };

            // Add to local points list
            Points.Add(point);

            // Add to dat declarations
            _datDeclarations.Add(new DatDeclaration
            {
                Type = "E6POS",
                Name = pointName,
                RawValue = $"{{X {point.X:F2}, Y {point.Y:F2}, Z {point.Z:F2}, A {point.Rx:F2}, B {point.Ry:F2}, C {point.Rz:F2}}}"
            });

            // Save to .dat file
            EnsureDatFile();
            SaveDatFile();

            // Send to Core
            await _ipcClient.SetPointAsync(pointName, status.TcpPosition.ToArray());

            ShowToast($"Taught {pointName}: ({point.X:F1}, {point.Y:F1}, {point.Z:F1})");
            Log.Information("Taught point {Name} at ({X:F1}, {Y:F1}, {Z:F1})", pointName, point.X, point.Y, point.Z);
        }
        catch (Exception ex)
        {
            ShowToast($"Teach failed: {ex.Message}", "error", 5000);
            Log.Error(ex, "Failed to teach point");
        }
    }

    [RelayCommand]
    private async Task RefreshPointsAsync()
    {
        try
        {
            var response = await _ipcClient.GetPointsAsync();
            if (response != null)
            {
                Points.Clear();
                foreach (var (name, values) in response.Points)
                {
                    Points.Add(new PointInfo
                    {
                        Name = name,
                        X = values.Length > 0 ? values[0] : 0,
                        Y = values.Length > 1 ? values[1] : 0,
                        Z = values.Length > 2 ? values[2] : 0,
                        Rx = values.Length > 3 ? values[3] : 0,
                        Ry = values.Length > 4 ? values[4] : 0,
                        Rz = values.Length > 5 ? values[5] : 0
                    });
                }
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to refresh points");
        }
    }

    // ========================================================================
    // Point Management Commands (Delete, Rename, Re-teach)
    // ========================================================================

    /// <summary>Currently selected point in the Points panel.</summary>
    [ObservableProperty]
    private PointInfo? _selectedPoint;

    [RelayCommand]
    private void DeletePoint(PointInfo? point)
    {
        point ??= SelectedPoint;
        if (point == null) return;

        // Check if point is used in .src
        var usage = Regex.Match(ProgramSource, $@"\b{Regex.Escape(point.Name)}\b", RegexOptions.IgnoreCase);
        if (usage.Success)
        {
            ShowToast($"Warning: {point.Name} is used in program. Removed from .dat only.", "warning", 5000);
        }

        Points.Remove(point);
        _datDeclarations.RemoveAll(d => d.Name.Equals(point.Name, StringComparison.OrdinalIgnoreCase));
        SaveDatFile();
        ShowToast($"Deleted point {point.Name}");
        Log.Information("Deleted point {Name}", point.Name);
    }

    [RelayCommand]
    private void RenamePoint(string? newName)
    {
        if (SelectedPoint == null || string.IsNullOrWhiteSpace(newName)) return;

        var oldName = SelectedPoint.Name;
        if (oldName.Equals(newName, StringComparison.OrdinalIgnoreCase)) return;

        // Check duplicate
        if (Points.Any(p => p.Name.Equals(newName, StringComparison.OrdinalIgnoreCase)))
        {
            ShowToast($"Point name '{newName}' already exists", "error", 3000);
            return;
        }

        // Update in .src (replace all references)
        ProgramSource = Regex.Replace(ProgramSource,
            $@"\b{Regex.Escape(oldName)}\b",
            newName,
            RegexOptions.IgnoreCase);

        // Update in dat declarations
        var decl = _datDeclarations.FirstOrDefault(d => d.Name.Equals(oldName, StringComparison.OrdinalIgnoreCase));
        if (decl != null) decl.Name = newName;

        // Update PointInfo
        SelectedPoint.Name = newName;
        SaveDatFile();

        ShowToast($"Renamed {oldName} → {newName}");
        Log.Information("Renamed point {OldName} → {NewName}", oldName, newName);
    }

    [RelayCommand]
    private async Task ReteachPointAsync()
    {
        if (SelectedPoint == null) return;

        try
        {
            var tcpPose = _viewportService.GetCurrentTcpPose();
            if (tcpPose == null || tcpPose.Length < 6)
            {
                var status = await _ipcClient.GetStatusAsync();
                if (status?.TcpPosition == null || status.TcpPosition.Count < 6)
                {
                    ShowToast("Cannot get robot position", "error", 5000);
                    return;
                }
                tcpPose = status.TcpPosition.ToArray();
            }

            // Update coordinates
            SelectedPoint.X = tcpPose[0];
            SelectedPoint.Y = tcpPose[1];
            SelectedPoint.Z = tcpPose[2];
            SelectedPoint.Rx = tcpPose[3];
            SelectedPoint.Ry = tcpPose[4];
            SelectedPoint.Rz = tcpPose[5];

            // Update dat declaration
            var decl = _datDeclarations.FirstOrDefault(
                d => d.Name.Equals(SelectedPoint.Name, StringComparison.OrdinalIgnoreCase));
            if (decl != null)
            {
                // Format based on declaration type
                if (decl.Type.Equals("E6AXIS", StringComparison.OrdinalIgnoreCase) ||
                    decl.Type.Equals("AXIS", StringComparison.OrdinalIgnoreCase))
                {
                    decl.RawValue = $"{{A1 {tcpPose[0]:F2}, A2 {tcpPose[1]:F2}, A3 {tcpPose[2]:F2}, A4 {tcpPose[3]:F2}, A5 {tcpPose[4]:F2}, A6 {tcpPose[5]:F2}}}";
                }
                else
                {
                    decl.RawValue = $"{{X {tcpPose[0]:F2}, Y {tcpPose[1]:F2}, Z {tcpPose[2]:F2}, A {tcpPose[3]:F2}, B {tcpPose[4]:F2}, C {tcpPose[5]:F2}}}";
                }
            }

            SaveDatFile();

            // Also update in Core
            await _ipcClient.SetPointAsync(SelectedPoint.Name, tcpPose);

            ShowToast($"Re-taught {SelectedPoint.Name}: ({tcpPose[0]:F1}, {tcpPose[1]:F1}, {tcpPose[2]:F1})");
            Log.Information("Re-taught {Name}", SelectedPoint.Name);
        }
        catch (Exception ex)
        {
            ShowToast($"Re-teach failed: {ex.Message}", "error", 5000);
            Log.Error(ex, "Re-teach failed");
        }
    }

    [RelayCommand]
    private void GoToPointUsage(PointInfo? point)
    {
        point ??= SelectedPoint;
        if (point == null) return;

        // Find first usage in .src
        var lines = ProgramSource.Split('\n');
        for (int i = 0; i < lines.Length; i++)
        {
            if (Regex.IsMatch(lines[i], $@"\b{Regex.Escape(point.Name)}\b", RegexOptions.IgnoreCase) &&
                RegexHelper.IsMotionInstruction(lines[i]))
            {
                CaretLineNumber = i + 1;
                // Scroll editor to this line
                ScrollToLineRequest = i + 1;
                return;
            }
        }

        ShowToast($"{point.Name} not used in program");
    }

    // ========================================================================
    // Variable Watch Panel
    // ========================================================================

    [ObservableProperty]
    private bool _isWatchPanelVisible;

    public ObservableCollection<WatchVariable> WatchVariables { get; } = new();

    [RelayCommand]
    private void ToggleWatchPanel()
    {
        IsWatchPanelVisible = !IsWatchPanelVisible;
        if (IsWatchPanelVisible)
        {
            _ = RefreshVariablesAsync();
        }
    }

    [RelayCommand]
    private async Task RefreshVariablesAsync()
    {
        try
        {
            var response = await _ipcClient.GetProgramStateAsync(true);
            if (response == null) return;

            var previousValues = WatchVariables.ToDictionary(w => w.Name, w => w.Value);
            WatchVariables.Clear();

            // User variables
            if (response.Variables != null)
            {
                foreach (var (name, value) in response.Variables)
                {
                    var strValue = value.ToString("G6");
                    var isChanged = previousValues.TryGetValue(name, out var prev) && prev != strValue;
                    WatchVariables.Add(new WatchVariable { Name = name, Type = "REAL", Value = strValue, IsChanged = isChanged });
                }
            }

            // System variables
            if (response.SystemVariables != null)
            {
                foreach (var (name, value) in response.SystemVariables)
                {
                    var strValue = value.ToString("G6");
                    var isChanged = previousValues.TryGetValue(name, out var prev) && prev != strValue;
                    WatchVariables.Add(new WatchVariable { Name = name, Type = "SYSTEM", Value = strValue, IsChanged = isChanged });
                }
            }

            // Points
            if (response.Points != null)
            {
                foreach (var (name, values) in response.Points)
                {
                    var strValue = values.Length >= 3
                        ? $"({values[0]:F1}, {values[1]:F1}, {values[2]:F1})"
                        : string.Join(", ", values.Select(v => v.ToString("F1")));
                    var isChanged = previousValues.TryGetValue(name, out var prev) && prev != strValue;
                    WatchVariables.Add(new WatchVariable { Name = name, Type = "E6POS", Value = strValue, IsChanged = isChanged });
                }
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to refresh watch variables");
        }
    }

    // ========================================================================
    // Path Preview
    // ========================================================================

    [ObservableProperty]
    private bool _isPathPreviewVisible;

    [RelayCommand]
    private void PreviewPath()
    {
        if (string.IsNullOrWhiteSpace(ProgramSource))
        {
            ShowToast("No program loaded", "error", 3000);
            return;
        }

        // Parse motion statements from .src
        var motionRegex = new Regex(@"^\s*(PTP|LIN|CIRC)\s+(\w+)", RegexOptions.IgnoreCase | RegexOptions.Multiline);
        var matches = motionRegex.Matches(ProgramSource);

        if (matches.Count == 0)
        {
            ShowToast("No motion statements found", "error", 3000);
            return;
        }

        // Build lookup from Points collection
        var pointLookup = Points.ToDictionary(
            p => p.Name.ToUpperInvariant(),
            p => new Point3D(p.X, p.Y, p.Z));

        // Fallback: also include points from inline DECL/CONST in ProgramSource (legacy programs)
        var inlineCache = RegexHelper.BuildCoordinateCache(ProgramSource);
        foreach (var (name, coords) in inlineCache)
        {
            var key = name.ToUpperInvariant();
            if (!pointLookup.ContainsKey(key))
                pointLookup[key] = new Point3D(coords.x, coords.y, coords.z);
        }

        // Build segments and markers
        var segments = new List<PathSegment>();
        var markers = new List<WaypointMarker>();
        Point3D? lastPoint = null;
        var usedPoints = new HashSet<string>();

        for (int i = 0; i < matches.Count; i++)
        {
            var motionType = matches[i].Groups[1].Value.ToUpperInvariant();
            var targetName = matches[i].Groups[2].Value.ToUpperInvariant();

            if (!pointLookup.TryGetValue(targetName, out var targetPos))
                continue;

            // Add segment from last point to this target
            if (lastPoint.HasValue)
            {
                segments.Add(new PathSegment
                {
                    From = lastPoint.Value,
                    To = targetPos,
                    MotionType = motionType,
                    PointName = matches[i].Groups[2].Value
                });
            }

            // Add waypoint marker (only once per unique point)
            if (!usedPoints.Contains(targetName))
            {
                markers.Add(new WaypointMarker
                {
                    Position = targetPos,
                    Label = matches[i].Groups[2].Value,
                    IsEndpoint = i == 0 || i == matches.Count - 1
                });
                usedPoints.Add(targetName);
            }

            lastPoint = targetPos;
        }

        if (segments.Count == 0)
        {
            ShowToast("No resolvable path segments (points not found)", "error", 3000);
            return;
        }

        _viewportService.ShowPathPreview(segments);
        _viewportService.ShowWaypointMarkers(markers);
        IsPathPreviewVisible = true;
        ShowToast($"Path preview: {segments.Count} segments, {markers.Count} waypoints");
    }

    [RelayCommand]
    private void ClearPreview()
    {
        _viewportService.ClearPathPreview();
        IsPathPreviewVisible = false;
    }

    // ========================================================================
    // Teach & Insert (KRL-style teaching workflow)
    // ========================================================================

    [ObservableProperty]
    private string _teachMotionType = "LIN";

    [ObservableProperty]
    private ApproximationType _teachApproximation = ApproximationType.EXACT;

    [ObservableProperty]
    private double _teachVelocity = 0.1;  // m/s ($VEL.CP)

    public ObservableCollection<string> AvailableMotionTypes { get; } = new() { "PTP", "LIN", "CIRC" };
    public ObservableCollection<ApproximationType> AvailableApproximations { get; } = new()
    {
        ApproximationType.EXACT, ApproximationType.C_PTP, ApproximationType.C_DIS,
        ApproximationType.C_VEL, ApproximationType.C_ORI
    };

    [RelayCommand]
    private async Task TeachAndInsertAsync()
    {
        try
        {
            // 1. Get current TCP position
            var tcpPose = _viewportService.GetCurrentTcpPose();
            if (tcpPose == null || tcpPose.Length < 6)
            {
                var status = await _ipcClient.GetStatusAsync();
                if (status?.TcpPosition == null || status.TcpPosition.Count < 6)
                {
                    ShowToast("Cannot get robot position", "error", 5000);
                    return;
                }
                tcpPose = status.TcpPosition.ToArray();
            }

            // 2. Auto-generate point name
            var pointName = GenerateNextPointName();

            // 3. Add point to .dat declarations + Points collection
            var point = new PointInfo
            {
                Name = pointName,
                X = tcpPose[0], Y = tcpPose[1], Z = tcpPose[2],
                Rx = tcpPose[3], Ry = tcpPose[4], Rz = tcpPose[5]
            };
            Points.Add(point);

            var rawValue = $"{{X {tcpPose[0]:F2}, Y {tcpPose[1]:F2}, Z {tcpPose[2]:F2}, A {tcpPose[3]:F2}, B {tcpPose[4]:F2}, C {tcpPose[5]:F2}}}";
            _datDeclarations.Add(new DatDeclaration
            {
                Type = "E6POS",
                Name = pointName,
                RawValue = rawValue
            });

            // 4. Save to .dat file
            EnsureDatFile();
            SaveDatFile();

            // 5. Create KRL velocity + motion instruction (only motion line goes into .src)
            var velLine = $"    $VEL.CP = {TeachVelocity:F2}";
            var approxSuffix = TeachApproximation switch
            {
                ApproximationType.C_PTP => " C_PTP",
                ApproximationType.C_DIS => " C_DIS",
                ApproximationType.C_VEL => " C_VEL",
                ApproximationType.C_ORI => " C_ORI",
                _ => ""
            };
            var motionLine = $"    {TeachMotionType} {pointName}{approxSuffix}";

            // 6. Insert motion line into .src at caret (no DECL in .src — it's in .dat)
            InsertMotionLine(velLine + "\n" + motionLine);

            // 7. Toast
            ShowToast($"Taught {pointName}: ({tcpPose[0]:F1}, {tcpPose[1]:F1}, {tcpPose[2]:F1})");
            Log.Information("Teach & Insert: {Point} at ({X:F1}, {Y:F1}, {Z:F1})",
                pointName, tcpPose[0], tcpPose[1], tcpPose[2]);
        }
        catch (Exception ex)
        {
            ShowToast($"Teach failed: {ex.Message}", "error", 5000);
            Log.Error(ex, "Teach & Insert failed");
        }
    }

    private string GenerateNextPointName()
    {
        // Scan Points collection (from .dat file) and ProgramSource (legacy inline)
        var maxNum = 0;
        var nameRegex = new Regex(@"^p(\d+)$", RegexOptions.IgnoreCase);

        // Check Points collection first
        foreach (var p in Points)
        {
            var m = nameRegex.Match(p.Name);
            if (m.Success && int.TryParse(m.Groups[1].Value, out var num) && num > maxNum)
                maxNum = num;
        }

        // Also check _datDeclarations (covers non-point DECLs that have pN names)
        foreach (var d in _datDeclarations)
        {
            var m = nameRegex.Match(d.Name);
            if (m.Success && int.TryParse(m.Groups[1].Value, out var num) && num > maxNum)
                maxNum = num;
        }

        // Fallback: also scan ProgramSource for legacy inline programs
        var srcRegex = new Regex(@"(?:DECL|CONST)\s+(?:E6POS|POS)\s+(p\d+)", RegexOptions.IgnoreCase);
        foreach (Match sm in srcRegex.Matches(ProgramSource))
        {
            var numStr = sm.Groups[1].Value[1..];
            if (int.TryParse(numStr, out var num) && num > maxNum)
                maxNum = num;
        }

        return $"p{maxNum + 1}";
    }

    private void InsertTeachLines(string constLine, string motionLine)
    {
        var lines = ProgramSource.Split('\n').ToList();

        // Find last DECL E6POS line for inserting variable definition
        int lastConstIndex = -1;
        for (int i = 0; i < lines.Count; i++)
        {
            if (lines[i].TrimStart().StartsWith("DECL E6POS", StringComparison.OrdinalIgnoreCase) ||
                lines[i].TrimStart().StartsWith("CONST E6POS", StringComparison.OrdinalIgnoreCase))
                lastConstIndex = i;
        }

        // Insert CONST line after last CONST (or at line 0 if none found)
        int constInsertIndex = lastConstIndex >= 0 ? lastConstIndex + 1 : 0;
        lines.Insert(constInsertIndex, constLine);

        // Determine where to insert motion line
        int motionInsertIndex;
        if (CaretLineNumber > 0)
        {
            // Caret is set - insert at caret position (adjusted for the CONST line we just inserted)
            motionInsertIndex = CaretLineNumber;
            if (motionInsertIndex > constInsertIndex)
                motionInsertIndex++;
        }
        else
        {
            // Caret not set (user hasn't clicked in editor) - insert before END or at end of file
            motionInsertIndex = -1;
            for (int i = lines.Count - 1; i >= 0; i--)
            {
                if (lines[i].TrimStart().StartsWith("END", StringComparison.OrdinalIgnoreCase))
                {
                    motionInsertIndex = i;
                    break;
                }
            }
            if (motionInsertIndex < 0)
                motionInsertIndex = lines.Count; // no END found, append at end
        }
        motionInsertIndex = Math.Clamp(motionInsertIndex, 0, lines.Count);
        lines.Insert(motionInsertIndex, motionLine);

        ProgramSource = string.Join('\n', lines);
    }

    /// <summary>
    /// Insert only a motion line into .src at caret position (no DECL — that goes to .dat).
    /// </summary>
    private void InsertMotionLine(string motionLine)
    {
        var lines = ProgramSource.Split('\n').ToList();

        int motionInsertIndex;
        if (CaretLineNumber > 0)
        {
            motionInsertIndex = CaretLineNumber;
        }
        else
        {
            // Insert before END or at end
            motionInsertIndex = -1;
            for (int i = lines.Count - 1; i >= 0; i--)
            {
                if (lines[i].TrimStart().StartsWith("END", StringComparison.OrdinalIgnoreCase))
                {
                    motionInsertIndex = i;
                    break;
                }
            }
            if (motionInsertIndex < 0)
                motionInsertIndex = lines.Count;
        }
        motionInsertIndex = Math.Clamp(motionInsertIndex, 0, lines.Count);

        // Handle multi-line inserts (e.g., "$VEL.CP = 0.10\nLIN p1")
        var insertLines = motionLine.Split('\n');
        for (int i = 0; i < insertLines.Length; i++)
        {
            lines.Insert(motionInsertIndex + i, insertLines[i]);
        }

        ProgramSource = string.Join('\n', lines);
    }

    private async Task PollProgramStateAsync()
    {
        while (IsRunning && !IsPaused)
        {
            try
            {
                var state = await _ipcClient.GetProgramStateAsync();
                if (state != null)
                {
                    ExecutionState = state.State;
                    CurrentLine = state.CurrentLine;

                    // Update trace motion type from current line content
                    UpdateTraceMotionType(state.CurrentLine);

                    if (state.State == "COMPLETED" || state.State == "ERROR" || state.State == "STOPPED")
                    {
                        IsRunning = false;
                        break;
                    }
                }

                await Task.Delay(100);
            }
            catch
            {
                break;
            }
        }
    }

    private void UpdateTraceMotionType(int lineNumber)
    {
        if (lineNumber <= 0 || string.IsNullOrEmpty(ProgramSource)) return;

        var lines = ProgramSource.Split('\n');
        if (lineNumber > lines.Length) return;

        var line = lines[lineNumber - 1].TrimStart();
        if (line.StartsWith("PTP", StringComparison.OrdinalIgnoreCase))
            _viewportService.SetTraceMotionType("PTP");
        else if (line.StartsWith("LIN", StringComparison.OrdinalIgnoreCase))
            _viewportService.SetTraceMotionType("LIN");
        else if (line.StartsWith("CIRC", StringComparison.OrdinalIgnoreCase))
            _viewportService.SetTraceMotionType("CIRC");
    }
}
