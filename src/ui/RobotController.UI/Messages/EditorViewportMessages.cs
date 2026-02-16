using CommunityToolkit.Mvvm.Messaging.Messages;

namespace RobotController.UI.Messages;

/// <summary>
/// Sent when the editor caret moves to a motion instruction line.
/// ViewportService subscribes to highlight the corresponding 3D path segment.
/// </summary>
public class EditorCaretOnMotionLineMessage : ValueChangedMessage<EditorMotionLineInfo>
{
    public EditorCaretOnMotionLineMessage(EditorMotionLineInfo value) : base(value) { }
}

/// <summary>
/// Info about which motion line the caret is on
/// </summary>
public class EditorMotionLineInfo
{
    /// <summary>Line number in the editor (1-based)</summary>
    public int LineNumber { get; init; }

    /// <summary>Target variable name (e.g., "p1", "P10")</summary>
    public string TargetName { get; init; } = "";

    /// <summary>Position from coordinate cache, if available</summary>
    public double X { get; init; }
    public double Y { get; init; }
    public double Z { get; init; }

    /// <summary>Whether position was resolved from the document</summary>
    public bool HasPosition { get; init; }
}

/// <summary>
/// Sent when the caret is NOT on a motion line (clear highlight).
/// </summary>
public class EditorCaretOffMotionLineMessage : ValueChangedMessage<int>
{
    public EditorCaretOffMotionLineMessage(int lineNumber) : base(lineNumber) { }
}

/// <summary>
/// Sent from Viewport to Editor: user clicked a 3D path segment, scroll to that line.
/// </summary>
public class ViewportSegmentClickedMessage : ValueChangedMessage<int>
{
    public ViewportSegmentClickedMessage(int lineNumber) : base(lineNumber) { }
}

/// <summary>
/// Sent when Touch-Up modifies a point definition in the editor.
/// Viewport can refresh the path display.
/// </summary>
public class PointDefinitionChangedMessage : ValueChangedMessage<string>
{
    public PointDefinitionChangedMessage(string variableName) : base(variableName) { }
}

/// <summary>
/// Sent from ViewModel to Editor View: scroll to a specific line number.
/// </summary>
public class ScrollToLineMessage : ValueChangedMessage<int>
{
    public ScrollToLineMessage(int lineNumber) : base(lineNumber) { }
}

/// <summary>
/// Sent from ProgramEditor to MainViewModel: auto-toggle TCP trace on/off.
/// </summary>
public class TcpTraceAutoToggleMessage : ValueChangedMessage<bool>
{
    public TcpTraceAutoToggleMessage(bool enabled) : base(enabled) { }
}
