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
/// Completion data for speed variables (v100, v1000, etc.)
/// </summary>
public class SpeedCompletionData : ICompletionData
{
    private readonly SpeedData _speed;

    public SpeedCompletionData(SpeedData speed) { _speed = speed; }

    public ImageSource? Image => null;
    public string Text => _speed.Name;
    public object Content => $"{_speed.Name} ({_speed.VTcp} mm/s)";
    public object Description => $"TCP Speed: {_speed.VTcp} mm/s\nOrientation: {_speed.VOrient} deg/s";
    public double Priority => 0;

    public void Complete(TextArea textArea, ISegment completionSegment, EventArgs insertionRequestEventArgs)
    {
        textArea.Document.Replace(completionSegment, _speed.Name);
    }
}

/// <summary>
/// Completion data for zone variables (fine, z1, z50, etc.)
/// </summary>
public class ZoneCompletionData : ICompletionData
{
    private readonly ZoneData _zone;

    public ZoneCompletionData(ZoneData zone) { _zone = zone; }

    public ImageSource? Image => null;
    public string Text => _zone.Name;
    public object Content => $"{_zone.Name} ({(_zone.FinePnt ? "Stop Point" : $"{_zone.PZone}mm fly-by")})";
    public object Description => _zone.FinePnt ? "Exact stop point" : $"Fly-by zone: {_zone.PZone}mm";
    public double Priority => 0;

    public void Complete(TextArea textArea, ISegment completionSegment, EventArgs insertionRequestEventArgs)
    {
        textArea.Document.Replace(completionSegment, _zone.Name);
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

    // Predefined motion templates
    public static MotionCompletionData MoveL => new("MoveL",
        "MoveL p, v100, fine, tool0;",
        "Linear motion - TCP moves in a straight line");

    public static MotionCompletionData MoveJ => new("MoveJ",
        "MoveJ p, vmax, z100, tool0;",
        "Joint motion - fastest path, non-linear TCP trajectory");

    public static MotionCompletionData MoveC => new("MoveC",
        "MoveC pVia, pTo, v100, fine, tool0;",
        "Circular motion - arc through via-point to destination");
}
