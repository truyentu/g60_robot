using System.Globalization;
using CommunityToolkit.Mvvm.ComponentModel;
using RobotController.UI.Helpers;
using RobotController.UI.Models;

namespace RobotController.UI.ViewModels;

/// <summary>
/// Type of command currently selected for inline editing.
/// </summary>
public enum InlineCommandType
{
    None,
    Motion,    // PTP / LIN / CIRC
    ArcStart,  // ArcStart(Job_ID:=N)
    Velocity,  // $VEL.CP = N
    Wait,      // WAIT SEC N / WAIT FOR condition
    Output,    // $OUT[N] = TRUE/FALSE
    Comment    // ; user comment
}

/// <summary>
/// Edit vs Insert mode for the inline form.
/// </summary>
public enum InlineFormMode
{
    Edit,   // Modify existing line in-place
    Insert  // Generate new KRL text to insert
}

/// <summary>
/// ViewModel for the KUKA-style Inline Command Editor popup.
/// Supports both EDIT mode (parse existing line, update on dropdown change)
/// and INSERT mode (fill form, generate KRL on confirm).
/// </summary>
public partial class SelectedCommandViewModel : ObservableObject
{
    private readonly Action<string> _updateLineAction;
    private bool _suppressUpdate;

    // ========================================================================
    // Core State
    // ========================================================================

    [ObservableProperty]
    private InlineCommandType _commandType = InlineCommandType.None;

    [ObservableProperty]
    private InlineFormMode _formMode = InlineFormMode.Edit;

    [ObservableProperty]
    private int _lineNumber;

    [ObservableProperty]
    private string _originalLineText = "";

    // ========================================================================
    // Visibility Booleans (driven by CommandType)
    // ========================================================================

    public bool IsMotionCommand => CommandType == InlineCommandType.Motion;
    public bool IsArcStartCommand => CommandType == InlineCommandType.ArcStart;
    public bool IsVelocityCommand => CommandType == InlineCommandType.Velocity;
    public bool IsWaitCommand => CommandType == InlineCommandType.Wait;
    public bool IsOutputCommand => CommandType == InlineCommandType.Output;
    public bool IsCommentCommand => CommandType == InlineCommandType.Comment;
    public bool IsCircMotion => CommandType == InlineCommandType.Motion && MotionType == "CIRC";
    public bool IsInsertMode => FormMode == InlineFormMode.Insert;
    public bool IsWaitTimerMode => WaitType == "SEC";
    public bool IsWaitConditionMode => WaitType == "FOR";

    partial void OnCommandTypeChanged(InlineCommandType value)
    {
        OnPropertyChanged(nameof(IsMotionCommand));
        OnPropertyChanged(nameof(IsArcStartCommand));
        OnPropertyChanged(nameof(IsVelocityCommand));
        OnPropertyChanged(nameof(IsWaitCommand));
        OnPropertyChanged(nameof(IsOutputCommand));
        OnPropertyChanged(nameof(IsCommentCommand));
        OnPropertyChanged(nameof(IsCircMotion));
    }

    partial void OnFormModeChanged(InlineFormMode value)
    {
        OnPropertyChanged(nameof(IsInsertMode));
    }

    // ========================================================================
    // Motion Parameters (PTP/LIN/CIRC)
    // ========================================================================

    [ObservableProperty]
    private string _motionType = "LIN";

    [ObservableProperty]
    private string _targetName = "";

    [ObservableProperty]
    private ApproximationType _approximation = ApproximationType.EXACT;

    // CIRC-specific
    [ObservableProperty]
    private string _auxPointName = "";

    [ObservableProperty]
    private double _caAngle = 0;

    [ObservableProperty]
    private bool _useCaAngle = false;

    // ========================================================================
    // ArcStart Parameters
    // ========================================================================

    [ObservableProperty]
    private WeldingJobInfo? _selectedJob;

    // ========================================================================
    // Velocity Parameters
    // ========================================================================

    [ObservableProperty]
    private string _velocityVariable = "$VEL.CP";

    [ObservableProperty]
    private double _velocityValue = 0.1;

    // ========================================================================
    // Wait Parameters
    // ========================================================================

    [ObservableProperty]
    private string _waitType = "SEC";

    [ObservableProperty]
    private double _waitTime = 1.0;

    [ObservableProperty]
    private string _waitCondition = "$IN[1] == TRUE";

    // ========================================================================
    // Output Parameters
    // ========================================================================

    [ObservableProperty]
    private int _outputIndex = 1;

    [ObservableProperty]
    private string _outputValueStr = "TRUE";

    // ========================================================================
    // Comment Parameters
    // ========================================================================

    [ObservableProperty]
    private string _commentText = "";

    // ========================================================================
    // Insert Mode Options
    // ========================================================================

    [ObservableProperty]
    private bool _wrapInFold = false;

    // ========================================================================
    // Available Options (for ComboBoxes)
    // ========================================================================

    public List<string> MotionTypes { get; } = ["PTP", "LIN", "CIRC"];

    public List<ApproximationType> AvailableApproximations { get; } =
    [
        ApproximationType.EXACT,
        ApproximationType.C_PTP,
        ApproximationType.C_DIS,
        ApproximationType.C_VEL,
        ApproximationType.C_ORI
    ];

    public List<WeldingJobInfo> AvailableJobs => WeldingJobInfo.DefaultJobs;

    public List<string> VelocityVariables { get; } = ["$VEL.CP", "$VEL_AXIS[1]"];
    public List<string> WaitTypes { get; } = ["SEC", "FOR"];
    public List<string> BoolValues { get; } = ["TRUE", "FALSE"];

    /// <summary>
    /// Create with a callback that updates the document line.
    /// </summary>
    public SelectedCommandViewModel(Action<string> updateLineAction)
    {
        _updateLineAction = updateLineAction;
    }

    // ========================================================================
    // EDIT Mode: Parse existing line
    // ========================================================================

    /// <summary>
    /// Parse a line of code and populate the inline form fields.
    /// Returns true if the line was recognized as an editable command.
    /// </summary>
    public bool TryParseFromLine(int lineNumber, string lineText)
    {
        _suppressUpdate = true;
        try
        {
            FormMode = InlineFormMode.Edit;
            LineNumber = lineNumber;
            OriginalLineText = lineText;

            // Try KRL motion instruction: PTP/LIN target or CIRC auxPoint, target
            var motionMatch = RegexHelper.KrlMotionRegex().Match(lineText);
            if (motionMatch.Success)
            {
                CommandType = InlineCommandType.Motion;
                MotionType = motionMatch.Groups[1].Value.ToUpperInvariant();

                // For CIRC: group[2]=auxPoint, group[3]=target; for PTP/LIN: group[2]=target
                if (motionMatch.Groups[3].Success && !string.IsNullOrEmpty(motionMatch.Groups[3].Value))
                {
                    AuxPointName = motionMatch.Groups[2].Value;
                    TargetName = motionMatch.Groups[3].Value;
                }
                else
                {
                    AuxPointName = "";
                    TargetName = motionMatch.Groups[2].Value;
                }

                // Parse CA angle (group[4])
                if (motionMatch.Groups[4].Success &&
                    double.TryParse(motionMatch.Groups[4].Value, NumberStyles.Float, CultureInfo.InvariantCulture, out var ca))
                {
                    CaAngle = ca;
                    UseCaAngle = true;
                }
                else
                {
                    CaAngle = 0;
                    UseCaAngle = false;
                }

                Approximation = RegexHelper.ExtractApproximation(lineText);
                OnPropertyChanged(nameof(IsCircMotion));
                return true;
            }

            // Try ArcStart: ArcStart(Job_ID:=N)
            var arcMatch = RegexHelper.ArcStartRegex().Match(lineText);
            if (arcMatch.Success)
            {
                CommandType = InlineCommandType.ArcStart;
                if (arcMatch.Groups[1].Success && int.TryParse(arcMatch.Groups[1].Value, out var jobId))
                {
                    SelectedJob = AvailableJobs.FirstOrDefault(j => j.Id == jobId) ?? AvailableJobs[0];
                }
                else
                {
                    SelectedJob = AvailableJobs[0];
                }
                return true;
            }

            // Try Velocity: $VEL.CP = N or $VEL_AXIS[N] = N
            var velMatch = RegexHelper.VelocityAssignRegex().Match(lineText);
            if (velMatch.Success)
            {
                CommandType = InlineCommandType.Velocity;
                VelocityVariable = velMatch.Groups[1].Value;
                if (double.TryParse(velMatch.Groups[2].Value, NumberStyles.Float, CultureInfo.InvariantCulture, out var vel))
                    VelocityValue = vel;
                return true;
            }

            // Try Wait: WAIT SEC N or WAIT FOR condition
            var waitMatch = RegexHelper.WaitRegex().Match(lineText);
            if (waitMatch.Success)
            {
                CommandType = InlineCommandType.Wait;
                if (waitMatch.Groups[2].Success && !string.IsNullOrEmpty(waitMatch.Groups[2].Value))
                {
                    WaitType = "SEC";
                    if (double.TryParse(waitMatch.Groups[2].Value, NumberStyles.Float, CultureInfo.InvariantCulture, out var t))
                        WaitTime = t;
                }
                else if (waitMatch.Groups[3].Success)
                {
                    WaitType = "FOR";
                    WaitCondition = waitMatch.Groups[3].Value.Trim();
                }
                return true;
            }

            // Try Output: $OUT[N] = TRUE/FALSE
            var outMatch = RegexHelper.OutputRegex().Match(lineText);
            if (outMatch.Success)
            {
                CommandType = InlineCommandType.Output;
                if (int.TryParse(outMatch.Groups[1].Value, out var idx))
                    OutputIndex = idx;
                OutputValueStr = outMatch.Groups[2].Value.ToUpperInvariant();
                return true;
            }

            CommandType = InlineCommandType.None;
            return false;
        }
        finally
        {
            _suppressUpdate = false;
        }
    }

    // ========================================================================
    // INSERT Mode: Initialize form with defaults
    // ========================================================================

    /// <summary>
    /// Initialize the inline form for INSERT mode with defaults for the given type.
    /// </summary>
    public void InitForInsert(InlineCommandType type, int lineNumber)
    {
        _suppressUpdate = true;
        try
        {
            FormMode = InlineFormMode.Insert;
            LineNumber = lineNumber;
            OriginalLineText = "";
            CommandType = type;

            switch (type)
            {
                case InlineCommandType.Motion:
                    MotionType = "LIN";
                    TargetName = "";
                    AuxPointName = "";
                    CaAngle = 360.0;
                    UseCaAngle = false;
                    Approximation = ApproximationType.EXACT;
                    break;
                case InlineCommandType.Velocity:
                    VelocityVariable = "$VEL.CP";
                    VelocityValue = 0.1;
                    break;
                case InlineCommandType.Wait:
                    WaitType = "SEC";
                    WaitTime = 1.0;
                    WaitCondition = "$IN[1] == TRUE";
                    break;
                case InlineCommandType.Output:
                    OutputIndex = 1;
                    OutputValueStr = "TRUE";
                    break;
                case InlineCommandType.Comment:
                    CommentText = "";
                    break;
            }

            WrapInFold = false;
            OnPropertyChanged(nameof(IsCircMotion));
        }
        finally
        {
            _suppressUpdate = false;
        }
    }

    // ========================================================================
    // Code Generation (INSERT mode)
    // ========================================================================

    /// <summary>
    /// Build the KRL text to insert at the caret position.
    /// </summary>
    public string BuildInsertText()
    {
        var indent = "  ";
        string content = CommandType switch
        {
            InlineCommandType.Motion => BuildMotionInsertText(indent),
            InlineCommandType.Velocity => $"{indent}{VelocityVariable} = {VelocityValue.ToString("F2", CultureInfo.InvariantCulture)}",
            InlineCommandType.Wait => WaitType == "SEC"
                ? $"{indent}WAIT SEC {WaitTime.ToString("F1", CultureInfo.InvariantCulture)}"
                : $"{indent}WAIT FOR {WaitCondition}",
            InlineCommandType.Output => $"{indent}$OUT[{OutputIndex}] = {OutputValueStr}",
            InlineCommandType.Comment => $"{indent}; {CommentText}",
            _ => ""
        };

        if (WrapInFold && !string.IsNullOrEmpty(content))
        {
            var summary = BuildFoldSummary();
            return $"{indent};FOLD {summary}\n{content}\n{indent};ENDFOLD";
        }
        return content;
    }

    private string BuildMotionInsertText(string indent)
    {
        var approxSuffix = BuildApproxSuffix();

        if (MotionType == "CIRC" && !string.IsNullOrEmpty(AuxPointName))
        {
            var caStr = UseCaAngle ? $", CA {CaAngle.ToString("F1", CultureInfo.InvariantCulture)}" : "";
            return $"{indent}CIRC {AuxPointName}, {TargetName}{caStr}{approxSuffix}";
        }

        return $"{indent}{MotionType} {TargetName}{approxSuffix}";
    }

    private string BuildFoldSummary()
    {
        return CommandType switch
        {
            InlineCommandType.Motion => $"{MotionType} {TargetName}",
            InlineCommandType.Velocity => $"{VelocityVariable} = {VelocityValue.ToString("F2", CultureInfo.InvariantCulture)}",
            InlineCommandType.Wait => WaitType == "SEC" ? $"WAIT SEC {WaitTime:F1}" : $"WAIT FOR ...",
            InlineCommandType.Output => $"$OUT[{OutputIndex}] = {OutputValueStr}",
            InlineCommandType.Comment => CommentText.Length > 30 ? CommentText[..30] + "..." : CommentText,
            _ => "Instruction"
        };
    }

    // ========================================================================
    // Property Changed -> Rebuild line text (EDIT mode only)
    // ========================================================================

    partial void OnMotionTypeChanged(string value)
    {
        OnPropertyChanged(nameof(IsCircMotion));
        RebuildMotionLine();
    }
    partial void OnApproximationChanged(ApproximationType value) => RebuildMotionLine();
    partial void OnSelectedJobChanged(WeldingJobInfo? value) => RebuildArcStartLine();
    partial void OnAuxPointNameChanged(string value) => RebuildMotionLine();
    partial void OnCaAngleChanged(double value) => RebuildMotionLine();
    partial void OnUseCaAngleChanged(bool value) => RebuildMotionLine();
    partial void OnVelocityVariableChanged(string value) => RebuildVelocityLine();
    partial void OnVelocityValueChanged(double value) => RebuildVelocityLine();
    partial void OnWaitTypeChanged(string value)
    {
        OnPropertyChanged(nameof(IsWaitTimerMode));
        OnPropertyChanged(nameof(IsWaitConditionMode));
        RebuildWaitLine();
    }
    partial void OnWaitTimeChanged(double value) => RebuildWaitLine();
    partial void OnWaitConditionChanged(string value) => RebuildWaitLine();
    partial void OnOutputIndexChanged(int value) => RebuildOutputLine();
    partial void OnOutputValueStrChanged(string value) => RebuildOutputLine();
    partial void OnCommentTextChanged(string value) => RebuildCommentLine();

    // ========================================================================
    // Line Rebuild Methods
    // ========================================================================

    private string GetIndent()
    {
        var indent = "";
        foreach (var ch in OriginalLineText)
        {
            if (ch == ' ' || ch == '\t') indent += ch;
            else break;
        }
        return indent;
    }

    private string BuildApproxSuffix()
    {
        return Approximation switch
        {
            ApproximationType.C_PTP => " C_PTP",
            ApproximationType.C_DIS => " C_DIS",
            ApproximationType.C_VEL => " C_VEL",
            ApproximationType.C_ORI => " C_ORI",
            _ => ""
        };
    }

    private void RebuildMotionLine()
    {
        if (_suppressUpdate || FormMode == InlineFormMode.Insert || CommandType != InlineCommandType.Motion) return;

        var indent = GetIndent();
        var approxSuffix = BuildApproxSuffix();

        string newLine;
        if (MotionType == "CIRC" && !string.IsNullOrEmpty(AuxPointName))
        {
            var caStr = UseCaAngle ? $", CA {CaAngle.ToString("F1", CultureInfo.InvariantCulture)}" : "";
            newLine = $"{indent}CIRC {AuxPointName}, {TargetName}{caStr}{approxSuffix}";
        }
        else
        {
            newLine = $"{indent}{MotionType} {TargetName}{approxSuffix}";
        }

        _updateLineAction(newLine);
    }

    private void RebuildArcStartLine()
    {
        if (_suppressUpdate || FormMode == InlineFormMode.Insert || CommandType != InlineCommandType.ArcStart || SelectedJob == null) return;

        var indent = GetIndent();
        var newLine = $"{indent}ArcStart(Job_ID:={SelectedJob.Id})";
        _updateLineAction(newLine);
    }

    private void RebuildVelocityLine()
    {
        if (_suppressUpdate || FormMode == InlineFormMode.Insert || CommandType != InlineCommandType.Velocity) return;

        var indent = GetIndent();
        var newLine = $"{indent}{VelocityVariable} = {VelocityValue.ToString("F2", CultureInfo.InvariantCulture)}";
        _updateLineAction(newLine);
    }

    private void RebuildWaitLine()
    {
        if (_suppressUpdate || FormMode == InlineFormMode.Insert || CommandType != InlineCommandType.Wait) return;

        var indent = GetIndent();
        var newLine = WaitType == "SEC"
            ? $"{indent}WAIT SEC {WaitTime.ToString("F1", CultureInfo.InvariantCulture)}"
            : $"{indent}WAIT FOR {WaitCondition}";
        _updateLineAction(newLine);
    }

    private void RebuildOutputLine()
    {
        if (_suppressUpdate || FormMode == InlineFormMode.Insert || CommandType != InlineCommandType.Output) return;

        var indent = GetIndent();
        var newLine = $"{indent}$OUT[{OutputIndex}] = {OutputValueStr}";
        _updateLineAction(newLine);
    }

    private void RebuildCommentLine()
    {
        if (_suppressUpdate || FormMode == InlineFormMode.Insert || CommandType != InlineCommandType.Comment) return;

        var indent = GetIndent();
        var newLine = $"{indent}; {CommentText}";
        _updateLineAction(newLine);
    }
}
