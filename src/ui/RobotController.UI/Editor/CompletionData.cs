using ICSharpCode.AvalonEdit.CodeCompletion;
using ICSharpCode.AvalonEdit.Document;
using ICSharpCode.AvalonEdit.Editing;
using RobotController.UI.Models;
using System.Windows.Media;

namespace RobotController.UI.Editor;

/// <summary>
/// Completion data for welding job parameters shown when typing ArcStart(.
/// Displays job info and inserts Job_ID:=N format.
/// </summary>
public class WeldingJobCompletionData : ICompletionData
{
    private readonly WeldingJobInfo _job;

    public WeldingJobCompletionData(WeldingJobInfo job)
    {
        _job = job;
    }

    public ImageSource? Image => null;

    public string Text => $"Job {_job.Id}: {_job.Name}";

    public object Content => Text;

    public object Description => $"Material: {_job.Material}\n" +
                                  $"Wire Feed: {_job.WireFeedSpeed:F1} m/min\n" +
                                  $"Voltage: {_job.Voltage:F1} V\n" +
                                  $"Gas: {_job.GasType}";

    public double Priority => 0;

    public void Complete(TextArea textArea, ISegment completionSegment, EventArgs insertionRequestEventArgs)
    {
        textArea.Document.Replace(completionSegment, $"Job_ID:={_job.Id}");
    }
}

/// <summary>
/// Completion data for KRL system variable templates ($VEL.CP, $APO.CDIS, $TOOL, etc.)
/// </summary>
public class SystemVarCompletionData : ICompletionData
{
    private readonly string _varName;
    private readonly string _template;
    private readonly string _desc;

    public SystemVarCompletionData(string varName, string template, string desc)
    {
        _varName = varName;
        _template = template;
        _desc = desc;
    }

    public ImageSource? Image => null;
    public string Text => _varName;
    public object Content => _varName;
    public object Description => _desc;
    public double Priority => 0;

    public void Complete(TextArea textArea, ISegment completionSegment, EventArgs insertionRequestEventArgs)
    {
        textArea.Document.Replace(completionSegment, _template);
    }

    public static SystemVarCompletionData VelCp => new("$VEL.CP",
        "$VEL.CP = 0.1", "Path velocity (m/s) for LIN/CIRC");
    public static SystemVarCompletionData AccCp => new("$ACC.CP",
        "$ACC.CP = 1.0", "Path acceleration (m/s²) for LIN/CIRC");
    public static SystemVarCompletionData ApoCdis => new("$APO.CDIS",
        "$APO.CDIS = 10", "Distance approximation (mm)");
    public static SystemVarCompletionData ApoCptp => new("$APO.CPTP",
        "$APO.CPTP = 50", "PTP approximation (%)");
    public static SystemVarCompletionData ApoCori => new("$APO.CORI",
        "$APO.CORI = 5", "Orientation approximation (deg)");
    public static SystemVarCompletionData ApoCvel => new("$APO.CVEL",
        "$APO.CVEL = 50", "Velocity approximation (%)");
    public static SystemVarCompletionData OvPro => new("$OV_PRO",
        "$OV_PRO = 100", "Override percentage (0-100%)");
    public static SystemVarCompletionData Tool => new("$TOOL",
        "$TOOL = TOOL_DATA[1]", "Select tool frame");
    public static SystemVarCompletionData Base => new("$BASE",
        "$BASE = BASE_DATA[0]", "Select base frame");
}

/// <summary>
/// Completion data for KRL approximation keywords (C_PTP, C_DIS, C_VEL, C_ORI)
/// </summary>
public class ApproximationCompletionData : ICompletionData
{
    private readonly string _keyword;
    private readonly string _desc;

    public ApproximationCompletionData(string keyword, string desc)
    {
        _keyword = keyword;
        _desc = desc;
    }

    public ImageSource? Image => null;
    public string Text => _keyword;
    public object Content => _keyword;
    public object Description => _desc;
    public double Priority => 0;

    public void Complete(TextArea textArea, ISegment completionSegment, EventArgs insertionRequestEventArgs)
    {
        textArea.Document.Replace(completionSegment, _keyword);
    }
}

/// <summary>
/// Completion data for motion instruction templates
/// </summary>
public class MotionCompletionData : ICompletionData
{
    private readonly string _instruction;
    private readonly string _template;
    private readonly string _description;

    public MotionCompletionData(string instruction, string template, string description)
    {
        _instruction = instruction;
        _template = template;
        _description = description;
    }

    public ImageSource? Image => null;
    public string Text => _instruction;
    public object Content => _instruction;
    public object Description => _description;
    public double Priority => 0;

    public void Complete(TextArea textArea, ISegment completionSegment, EventArgs insertionRequestEventArgs)
    {
        textArea.Document.Replace(completionSegment, _template);
    }

    // Predefined motion templates (KUKA KRL)
    public static MotionCompletionData LIN => new("LIN",
        "LIN p1",
        "Linear motion - TCP moves in a straight line");

    public static MotionCompletionData PTP => new("PTP",
        "PTP p1",
        "Point-to-point motion - fastest path, joint interpolation");

    public static MotionCompletionData CIRC => new("CIRC",
        "CIRC pVia, pTo",
        "Circular motion - arc through via-point to destination");
}
