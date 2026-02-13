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
    ArcStart   // ArcStart(Job_ID:=N)
}

/// <summary>
/// ViewModel for the KUKA-style Inline Command Editor popup.
/// Parses the current line and provides dropdown selections for parameters.
/// Two-way: changing a dropdown immediately updates the document text.
///
/// KRL motion syntax: PTP target [C_PTP|C_DIS|C_VEL|C_ORI]
/// Velocity via system variables: $VEL.CP = 0.1 (m/s)
/// Tool/Base via system variables: $TOOL = TOOL_DATA[1]
/// </summary>
public partial class SelectedCommandViewModel : ObservableObject
{
    private readonly Action<string> _updateLineAction;
    private bool _suppressUpdate;

    [ObservableProperty]
    private InlineCommandType _commandType = InlineCommandType.None;

    public bool IsMotionCommand => CommandType == InlineCommandType.Motion;
    public bool IsArcStartCommand => CommandType == InlineCommandType.ArcStart;

    partial void OnCommandTypeChanged(InlineCommandType value)
    {
        OnPropertyChanged(nameof(IsMotionCommand));
        OnPropertyChanged(nameof(IsArcStartCommand));
    }

    [ObservableProperty]
    private int _lineNumber;

    [ObservableProperty]
    private string _originalLineText = "";

    // ========================================================================
    // Motion Parameters (PTP/LIN/CIRC) - KRL style
    // ========================================================================

    [ObservableProperty]
    private string _motionType = "LIN";

    [ObservableProperty]
    private string _targetName = "";

    [ObservableProperty]
    private ApproximationType _approximation = ApproximationType.EXACT;

    // ========================================================================
    // ArcStart Parameters
    // ========================================================================

    [ObservableProperty]
    private WeldingJobInfo? _selectedJob;

    // ========================================================================
    // Available Options
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

    /// <summary>
    /// Create with a callback that updates the document line.
    /// </summary>
    public SelectedCommandViewModel(Action<string> updateLineAction)
    {
        _updateLineAction = updateLineAction;
    }

    /// <summary>
    /// Parse a line of code and populate the inline form fields.
    /// Returns true if the line was recognized as an editable command.
    /// </summary>
    public bool TryParseFromLine(int lineNumber, string lineText)
    {
        _suppressUpdate = true;
        try
        {
            LineNumber = lineNumber;
            OriginalLineText = lineText;

            // Try KRL motion instruction: PTP/LIN target or CIRC auxPoint, target
            var motionMatch = RegexHelper.KrlMotionRegex().Match(lineText);
            if (motionMatch.Success)
            {
                CommandType = InlineCommandType.Motion;
                MotionType = motionMatch.Groups[1].Value.ToUpperInvariant();
                // For CIRC: group[2]=auxPoint, group[3]=target; for PTP/LIN: group[2]=target
                if (motionMatch.Groups[3].Success)
                    TargetName = motionMatch.Groups[3].Value;  // CIRC target
                else
                    TargetName = motionMatch.Groups[2].Value;  // PTP/LIN target
                Approximation = RegexHelper.ExtractApproximation(lineText);
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

            CommandType = InlineCommandType.None;
            return false;
        }
        finally
        {
            _suppressUpdate = false;
        }
    }

    // ========================================================================
    // Property Changed -> Rebuild line text and push to document
    // ========================================================================

    partial void OnMotionTypeChanged(string value) => RebuildMotionLine();
    partial void OnApproximationChanged(ApproximationType value) => RebuildMotionLine();
    partial void OnSelectedJobChanged(WeldingJobInfo? value) => RebuildArcStartLine();

    private void RebuildMotionLine()
    {
        if (_suppressUpdate || CommandType != InlineCommandType.Motion) return;

        // Preserve leading whitespace from original line
        var indent = "";
        foreach (var ch in OriginalLineText)
        {
            if (ch == ' ' || ch == '\t') indent += ch;
            else break;
        }

        // KRL format: PTP target [C_DIS] (no commas, no semicolons)
        var approxSuffix = Approximation switch
        {
            ApproximationType.C_PTP => " C_PTP",
            ApproximationType.C_DIS => " C_DIS",
            ApproximationType.C_VEL => " C_VEL",
            ApproximationType.C_ORI => " C_ORI",
            _ => ""
        };

        var newLine = $"{indent}{MotionType} {TargetName}{approxSuffix}";
        _updateLineAction(newLine);
    }

    private void RebuildArcStartLine()
    {
        if (_suppressUpdate || CommandType != InlineCommandType.ArcStart || SelectedJob == null) return;

        var indent = "";
        foreach (var ch in OriginalLineText)
        {
            if (ch == ' ' || ch == '\t') indent += ch;
            else break;
        }

        var newLine = $"{indent}ArcStart(Job_ID:={SelectedJob.Id})";
        _updateLineAction(newLine);
    }
}
