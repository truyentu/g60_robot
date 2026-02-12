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

namespace RobotController.UI.ViewModels;

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
    private string _programSource = @"! Robot Program - Welding Example
! Variable Definitions
CONST robtarget pHome := [[500.00,0.00,800.00],[1.0000,0.0000,0.0000,0.0000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
CONST robtarget p1 := [[600.00,100.00,500.00],[0.7071,0.0000,0.7071,0.0000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
CONST robtarget p2 := [[700.00,100.00,500.00],[0.7071,0.0000,0.7071,0.0000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
CONST robtarget p3 := [[700.00,200.00,500.00],[0.7071,0.0000,0.7071,0.0000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

DEF WeldProgram()
    ! Approach
    MoveJ pHome, vmax, z100, tool0;
    MoveL p1, v1000, z50, tool0;

    ! Weld Seam 1
    ArcStart(Job_ID:=1);
    MoveL p2, v100, fine, tool0;
    MoveL p3, v100, fine, tool0;
    ArcEnd;

    ! Retract
    MoveL p1, v1000, z50, tool0;
    MoveJ pHome, vmax, z100, tool0;
END";

    [ObservableProperty]
    private string _programName = "WeldProgram";

    private string? _currentFilePath;

    [ObservableProperty]
    private int _currentLine = 0;

    [ObservableProperty]
    private string _executionState = "IDLE";

    [ObservableProperty]
    private bool _isRunning;

    [ObservableProperty]
    private bool _isPaused;

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
            var newValue = newTarget.ToRplString();

            // 4. Find the variable definition in the document text
            var definition = RegexHelper.FindVariableDefinition(ProgramSource, CaretTargetName);
            if (definition == null)
            {
                TouchUpStatus = $"Definition for '{CaretTargetName}' not found in document";
                return;
            }

            // 5. Replace the value in the document
            var (offset, length, oldValue) = definition.Value;
            var newSource = ProgramSource.Remove(offset, length).Insert(offset, newValue);
            ProgramSource = newSource;

            // 6. Notify viewport that a point changed
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
            var response = await _ipcClient.LoadProgramAsync(ProgramSource);
            Errors.Clear();

            if (response == null)
            {
                Errors.Add("Failed to communicate with Core");
                return;
            }

            if (!response.Success)
            {
                Errors.Add(response.Error ?? "Unknown error");
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
        await LoadProgramAsync();
        if (Errors.Count > 0) return;

        await _ipcClient.RunProgramAsync();
        IsRunning = true;
        IsPaused = false;
        ExecutionState = "RUNNING";

        // Start polling for state updates
        _ = PollProgramStateAsync();
    }

    [RelayCommand]
    private async Task StepProgramAsync()
    {
        if (!IsRunning)
        {
            await LoadProgramAsync();
            if (Errors.Count > 0) return;
            IsRunning = true;
        }

        var response = await _ipcClient.StepProgramAsync();
        if (response != null)
        {
            ExecutionState = response.State;
            CurrentLine = response.CurrentLine;

            if (response.State == "COMPLETED" || response.State == "ERROR")
            {
                IsRunning = false;
            }
        }
    }

    [RelayCommand]
    private async Task PauseProgramAsync()
    {
        await _ipcClient.PauseProgramAsync();
        IsPaused = true;
        ExecutionState = "PAUSED";
    }

    [RelayCommand]
    private async Task ResumeProgramAsync()
    {
        await _ipcClient.RunProgramAsync();
        IsPaused = false;
        ExecutionState = "RUNNING";
    }

    [RelayCommand]
    private async Task StopProgramAsync()
    {
        await _ipcClient.StopProgramAsync();
        IsRunning = false;
        IsPaused = false;
        ExecutionState = "STOPPED";
    }

    [RelayCommand]
    private async Task ResetProgramAsync()
    {
        await _ipcClient.ResetProgramAsync();
        CurrentLine = 0;
        IsRunning = false;
        IsPaused = false;
        ExecutionState = "IDLE";
    }

    // ========================================================================
    // File Open / Save
    // ========================================================================

    [RelayCommand]
    private void OpenFile()
    {
        var dlg = new OpenFileDialog
        {
            Title = "Open Robot Program",
            Filter = "RPL Files (*.rpl)|*.rpl|All Files (*.*)|*.*",
            DefaultExt = ".rpl"
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
            Filter = "RPL Files (*.rpl)|*.rpl|All Files (*.*)|*.*",
            DefaultExt = ".rpl",
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

    [RelayCommand]
    private async Task TeachPointAsync()
    {
        try
        {
            // Get current robot position
            var status = await _ipcClient.GetStatusAsync();
            if (status == null) return;

            // Generate point name
            string pointName = $"P{Points.Count + 1}";

            // Add to local points list
            Points.Add(new PointInfo
            {
                Name = pointName,
                X = status.TcpPosition.Count > 0 ? status.TcpPosition[0] : 0,
                Y = status.TcpPosition.Count > 1 ? status.TcpPosition[1] : 0,
                Z = status.TcpPosition.Count > 2 ? status.TcpPosition[2] : 0,
                Rx = status.TcpPosition.Count > 3 ? status.TcpPosition[3] : 0,
                Ry = status.TcpPosition.Count > 4 ? status.TcpPosition[4] : 0,
                Rz = status.TcpPosition.Count > 5 ? status.TcpPosition[5] : 0
            });

            // Send to Core
            await _ipcClient.SetPointAsync(pointName, status.TcpPosition.ToArray());

            Log.Information("Taught point {Name}", pointName);
        }
        catch (Exception ex)
        {
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
    // Teach & Insert (KUKA/ABB-style teaching workflow)
    // ========================================================================

    [ObservableProperty]
    private string _teachMotionType = "MoveL";

    [ObservableProperty]
    private string _teachSpeed = "v100";

    [ObservableProperty]
    private string _teachZone = "fine";

    [ObservableProperty]
    private string _teachTool = "tool0";

    public ObservableCollection<string> AvailableMotionTypes { get; } = new() { "MoveL", "MoveJ", "MoveC" };
    public ObservableCollection<string> AvailableSpeeds { get; } = new() { "v5", "v10", "v20", "v50", "v100", "v200", "v500", "v1000", "vmax" };
    public ObservableCollection<string> AvailableZones { get; } = new() { "fine", "z1", "z5", "z10", "z20", "z50", "z100", "z200" };
    public ObservableCollection<string> AvailableTools { get; } = new() { "tool0", "tool1", "tool2", "tool3" };

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

            // 3. Create CONST robtarget definition
            var target = RobTarget.FromPose(pointName, tcpPose);
            var constLine = $"CONST robtarget {pointName} := {target.ToRplString()};";

            // 4. Create motion instruction
            var motionLine = $"    {TeachMotionType} {pointName}, {TeachSpeed}, {TeachZone}, {TeachTool};";

            // 5. Insert into editor
            InsertTeachLines(constLine, motionLine);

            // 6. Update Points list
            Points.Add(new PointInfo
            {
                Name = pointName,
                X = tcpPose[0], Y = tcpPose[1], Z = tcpPose[2],
                Rx = tcpPose[3], Ry = tcpPose[4], Rz = tcpPose[5]
            });

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
        var regex = new Regex(@"CONST\s+robtarget\s+(p\d+)", RegexOptions.IgnoreCase);
        var matches = regex.Matches(ProgramSource);
        var maxNum = 0;
        foreach (Match m in matches)
        {
            var numStr = m.Groups[1].Value[1..];
            if (int.TryParse(numStr, out var num) && num > maxNum)
                maxNum = num;
        }
        return $"p{maxNum + 1}";
    }

    private void InsertTeachLines(string constLine, string motionLine)
    {
        var lines = ProgramSource.Split('\n').ToList();

        // Find last CONST robtarget line for inserting variable definition
        int lastConstIndex = -1;
        for (int i = 0; i < lines.Count; i++)
        {
            if (lines[i].TrimStart().StartsWith("CONST robtarget", StringComparison.OrdinalIgnoreCase))
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
}
