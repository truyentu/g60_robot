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
    Motion,    // MoveL / MoveJ / MoveC
    ArcStart   // ArcStart(Job_ID:=N)
}

/// <summary>
/// ViewModel for the KUKA-style Inline Command Editor popup.
/// Parses the current line and provides dropdown selections for parameters.
/// Two-way: changing a dropdown immediately updates the document text.
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
    // Motion Parameters (MoveL/MoveJ/MoveC)
    // ========================================================================

    [ObservableProperty]
    private string _motionType = "MoveL";

    [ObservableProperty]
    private string _targetName = "";

    [ObservableProperty]
    private SpeedData _selectedSpeed = SpeedData.V100;

    [ObservableProperty]
    private ZoneData _selectedZone = ZoneData.Fine;

    [ObservableProperty]
    private string _toolName = "tool0";

    // ========================================================================
    // ArcStart Parameters
    // ========================================================================

    [ObservableProperty]
    private WeldingJobInfo? _selectedJob;

    // ========================================================================
    // Available Options
    // ========================================================================

    public List<string> MotionTypes { get; } = ["MoveL", "MoveJ", "MoveC"];

    public List<SpeedData> AvailableSpeeds { get; } =
    [
        SpeedData.V5, SpeedData.V10, SpeedData.V50, SpeedData.V100,
        SpeedData.V200, SpeedData.V500, SpeedData.V1000, SpeedData.V2000, SpeedData.VMax
    ];

    public List<ZoneData> AvailableZones { get; } =
    [
        ZoneData.Fine, ZoneData.Z1, ZoneData.Z5, ZoneData.Z10,
        ZoneData.Z50, ZoneData.Z100, ZoneData.Z200
    ];

    public List<string> AvailableTools { get; } = ["tool0", "tool1", "tool2", "weldgun"];

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

            // Try motion instruction: MoveL target, speed, zone, tool;
            var motionMatch = RegexHelper.MotionInstructionRegex().Match(lineText);
            if (motionMatch.Success)
            {
                CommandType = InlineCommandType.Motion;
                MotionType = motionMatch.Groups[1].Value;
                TargetName = motionMatch.Groups[2].Value;

                var speedName = motionMatch.Groups[3].Value;
                SelectedSpeed = AvailableSpeeds.FirstOrDefault(
                    s => s.Name.Equals(speedName, StringComparison.OrdinalIgnoreCase)) ?? SpeedData.V100;

                var zoneName = motionMatch.Groups[4].Value;
                SelectedZone = AvailableZones.FirstOrDefault(
                    z => z.Name.Equals(zoneName, StringComparison.OrdinalIgnoreCase)) ?? ZoneData.Fine;

                ToolName = motionMatch.Groups[5].Value;
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
    partial void OnSelectedSpeedChanged(SpeedData value) => RebuildMotionLine();
    partial void OnSelectedZoneChanged(ZoneData value) => RebuildMotionLine();
    partial void OnToolNameChanged(string value) => RebuildMotionLine();
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

        var newLine = $"{indent}{MotionType} {TargetName}, {SelectedSpeed.Name}, {SelectedZone.Name}, {ToolName};";
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

        var newLine = $"{indent}ArcStart(Job_ID:={SelectedJob.Id});";
        _updateLineAction(newLine);
    }
}
