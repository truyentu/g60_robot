using ICSharpCode.AvalonEdit.CodeCompletion;
using ICSharpCode.AvalonEdit.Folding;
using ICSharpCode.AvalonEdit.Highlighting;
using ICSharpCode.AvalonEdit.Highlighting.Xshd;
using ICSharpCode.AvalonEdit.Rendering;
using RobotController.UI.Editor;
using RobotController.UI.Helpers;
using RobotController.UI.Models;
using RobotController.UI.ViewModels;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Threading;
using System.Xml;

namespace RobotController.UI.Views.Controls;

/// <summary>
/// Program Editor with syntax highlighting, caret tracking, folding, and intellisense
/// </summary>
public partial class ProgramEditor : UserControl
{
    private readonly CurrentLineBackgroundRenderer _lineRenderer;
    private readonly MotionLineBackgroundRenderer _motionLineRenderer;
    private FoldingManager? _foldingManager;
    private readonly WeldSeamFoldingStrategy _foldingStrategy = new();
    private readonly DispatcherTimer _foldingUpdateTimer;
    private CompletionWindow? _completionWindow;
    private SelectedCommandViewModel? _inlineCommandVm;
    private int _lastInlineFormLine;

    public ProgramEditor()
    {
        InitializeComponent();
        LoadSyntaxHighlighting();

        // Add current execution line highlighting
        _lineRenderer = new CurrentLineBackgroundRenderer(CodeEditor);
        CodeEditor.TextArea.TextView.BackgroundRenderers.Add(_lineRenderer);

        // Add motion line highlighting (orange when caret is on a motion instruction)
        _motionLineRenderer = new MotionLineBackgroundRenderer(CodeEditor);
        CodeEditor.TextArea.TextView.BackgroundRenderers.Add(_motionLineRenderer);

        // Install folding manager
        _foldingManager = FoldingManager.Install(CodeEditor.TextArea);

        // Timer to update foldings (throttled - not on every keystroke)
        _foldingUpdateTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(500) };
        _foldingUpdateTimer.Tick += (s, e) =>
        {
            _foldingUpdateTimer.Stop();
            UpdateFoldings();
        };

        // Bind text changes to ViewModel + trigger folding update
        CodeEditor.TextChanged += (s, e) =>
        {
            if (DataContext is ProgramEditorViewModel vm)
            {
                vm.ProgramSource = CodeEditor.Text;
            }
            // Restart folding timer (debounce)
            _foldingUpdateTimer.Stop();
            _foldingUpdateTimer.Start();
        };

        // Track caret position changes for bi-directional sync
        CodeEditor.TextArea.Caret.PositionChanged += OnCaretPositionChanged;

        // Intellisense: trigger on text entering
        CodeEditor.TextArea.TextEntering += OnTextEntering;
        CodeEditor.TextArea.TextEntered += OnTextEntered;

        // Initialize editor with default program and subscribe to changes
        DataContextChanged += (s, e) =>
        {
            if (e.OldValue is ProgramEditorViewModel oldVm)
            {
                oldVm.PropertyChanged -= OnViewModelPropertyChanged;
            }

            if (DataContext is ProgramEditorViewModel vm)
            {
                CodeEditor.Text = vm.ProgramSource;
                vm.PropertyChanged += OnViewModelPropertyChanged;
                // Initial folding update
                UpdateFoldings();
            }
        };
    }

    // ========================================================================
    // Folding
    // ========================================================================

    private void UpdateFoldings()
    {
        if (_foldingManager != null && CodeEditor.Document != null)
        {
            _foldingStrategy.UpdateFoldings(_foldingManager, CodeEditor.Document);
        }
    }

    // ========================================================================
    // Intellisense / Code Completion
    // ========================================================================

    private void OnTextEntering(object sender, TextCompositionEventArgs e)
    {
        // If completion window is open and user types non-letter, complete the selection
        if (_completionWindow != null && e.Text.Length > 0 && !char.IsLetterOrDigit(e.Text[0]))
        {
            _completionWindow.CompletionList.RequestInsertion(e);
        }
    }

    private void OnTextEntered(object sender, TextCompositionEventArgs e)
    {
        // Trigger completion on "(" after ArcStart
        if (e.Text == "(")
        {
            var wordBefore = GetWordBeforeCaret();
            if (wordBefore.Equals("ArcStart", StringComparison.OrdinalIgnoreCase))
            {
                ShowWeldingJobCompletion();
            }
        }

        // Trigger approximation completion after space in motion instruction
        if (e.Text == " ")
        {
            TryShowParameterCompletion();
        }

        // Trigger system variable completion after "$"
        if (e.Text == "$")
        {
            ShowSystemVarCompletion();
        }
    }

    private void ShowWeldingJobCompletion()
    {
        _completionWindow = new CompletionWindow(CodeEditor.TextArea);
        var data = _completionWindow.CompletionList.CompletionData;

        foreach (var job in WeldingJobInfo.DefaultJobs)
        {
            data.Add(new WeldingJobCompletionData(job));
        }

        _completionWindow.Show();
        _completionWindow.Closed += (_, _) => _completionWindow = null;
    }

    private void ShowSystemVarCompletion()
    {
        _completionWindow = new CompletionWindow(CodeEditor.TextArea);
        var data = _completionWindow.CompletionList.CompletionData;

        data.Add(SystemVarCompletionData.VelCp);
        data.Add(SystemVarCompletionData.AccCp);
        data.Add(SystemVarCompletionData.ApoCdis);
        data.Add(SystemVarCompletionData.ApoCptp);
        data.Add(SystemVarCompletionData.ApoCori);
        data.Add(SystemVarCompletionData.ApoCvel);
        data.Add(SystemVarCompletionData.OvPro);
        data.Add(SystemVarCompletionData.Tool);
        data.Add(SystemVarCompletionData.Base);

        _completionWindow.Show();
        _completionWindow.Closed += (_, _) => _completionWindow = null;
    }

    private void TryShowParameterCompletion()
    {
        var lineNumber = CodeEditor.TextArea.Caret.Line;
        if (lineNumber <= 0 || lineNumber > CodeEditor.Document.LineCount) return;

        var line = CodeEditor.Document.GetLineByNumber(lineNumber);
        var lineText = CodeEditor.Document.GetText(line.Offset, line.Length);

        if (!RegexHelper.IsMotionInstruction(lineText)) return;

        // After motion target, suggest approximation keywords (C_PTP, C_DIS, etc.)
        _completionWindow = new CompletionWindow(CodeEditor.TextArea);
        var data = _completionWindow.CompletionList.CompletionData;

        data.Add(new ApproximationCompletionData("C_PTP", "PTP approximation ($APO.CPTP %)"));
        data.Add(new ApproximationCompletionData("C_DIS", "Distance approximation ($APO.CDIS mm)"));
        data.Add(new ApproximationCompletionData("C_VEL", "Velocity approximation ($APO.CVEL %)"));
        data.Add(new ApproximationCompletionData("C_ORI", "Orientation approximation ($APO.CORI deg)"));

        if (data.Count > 0)
        {
            _completionWindow.Show();
            _completionWindow.Closed += (_, _) => _completionWindow = null;
        }
        else
        {
            _completionWindow = null;
        }
    }

    private string GetWordBeforeCaret()
    {
        int offset = CodeEditor.CaretOffset;
        if (offset <= 0) return "";

        // Walk back to find word start (skip the trigger character)
        int start = offset - 1; // -1 because caret is after the "(" or " "
        while (start > 0 && char.IsLetterOrDigit(CodeEditor.Document.GetCharAt(start - 1)))
        {
            start--;
        }

        if (start >= offset - 1) return "";
        return CodeEditor.Document.GetText(start, offset - 1 - start);
    }

    // ========================================================================
    // Caret Tracking
    // ========================================================================

    private void OnCaretPositionChanged(object? sender, EventArgs e)
    {
        if (DataContext is not ProgramEditorViewModel vm) return;

        var lineNumber = CodeEditor.TextArea.Caret.Line;
        if (lineNumber <= 0 || lineNumber > CodeEditor.Document.LineCount) return;

        var line = CodeEditor.Document.GetLineByNumber(lineNumber);
        var lineText = CodeEditor.Document.GetText(line.Offset, line.Length);

        // Update motion line highlighting
        bool isMotion = RegexHelper.IsMotionInstruction(lineText);
        _motionLineRenderer.HighlightedLine = isMotion ? lineNumber : 0;
        CodeEditor.TextArea.TextView.InvalidateVisual();

        // Notify ViewModel
        vm.OnCaretPositionChanged(lineNumber, lineText);

        // ====================================================================
        // Task B: KUKA-Style Inline Command Editor
        // ====================================================================
        UpdateInlineForm(lineNumber, lineText);

        // ====================================================================
        // Task C: Floating Touch-Up Button
        // ====================================================================
        UpdateFloatingTouchUpButton(lineNumber, isMotion);
    }

    // ========================================================================
    // Task B: Inline Form Popup
    // ========================================================================

    private void UpdateInlineForm(int lineNumber, string lineText)
    {
        // Create VM on first use
        _inlineCommandVm ??= new SelectedCommandViewModel(OnInlineFormUpdateLine);

        // Only re-parse if we moved to a different line
        if (lineNumber == _lastInlineFormLine && InlineFormPopup.IsOpen) return;
        _lastInlineFormLine = lineNumber;

        bool recognized = _inlineCommandVm.TryParseFromLine(lineNumber, lineText);

        if (recognized)
        {
            InlineEditor.DataContext = _inlineCommandVm;
            PositionPopupBelowLine(InlineFormPopup, lineNumber);
            InlineFormPopup.IsOpen = true;
        }
        else
        {
            InlineFormPopup.IsOpen = false;
        }
    }

    private void OnInlineFormUpdateLine(string newLineText)
    {
        if (_inlineCommandVm == null) return;

        var lineNumber = _inlineCommandVm.LineNumber;
        if (lineNumber <= 0 || lineNumber > CodeEditor.Document.LineCount) return;

        var line = CodeEditor.Document.GetLineByNumber(lineNumber);
        CodeEditor.Document.Replace(line.Offset, line.Length, newLineText);

        // Update original line text for next rebuild
        _inlineCommandVm.OriginalLineText = newLineText;
    }

    // ========================================================================
    // Task C: Floating Touch-Up Button
    // ========================================================================

    private void UpdateFloatingTouchUpButton(int lineNumber, bool isMotion)
    {
        if (isMotion)
        {
            PositionPopupAtLineEnd(TouchUpButtonPopup, lineNumber);
            TouchUpButtonPopup.IsOpen = true;
        }
        else
        {
            TouchUpButtonPopup.IsOpen = false;
        }
    }

    // ========================================================================
    // Popup Positioning Helpers
    // ========================================================================

    private void PositionPopupBelowLine(System.Windows.Controls.Primitives.Popup popup, int lineNumber)
    {
        try
        {
            var textView = CodeEditor.TextArea.TextView;
            var visualLine = textView.GetVisualLine(lineNumber);
            if (visualLine == null) return;

            var lineTop = visualLine.VisualTop - textView.ScrollOffset.Y;
            var lineHeight = visualLine.Height;

            // Position below the line
            popup.HorizontalOffset = 40; // offset past line numbers
            popup.VerticalOffset = lineTop + lineHeight + 2;
        }
        catch
        {
            // Fallback positioning
        }
    }

    private void PositionPopupAtLineEnd(System.Windows.Controls.Primitives.Popup popup, int lineNumber)
    {
        try
        {
            var textView = CodeEditor.TextArea.TextView;
            var visualLine = textView.GetVisualLine(lineNumber);
            if (visualLine == null) return;

            var lineTop = visualLine.VisualTop - textView.ScrollOffset.Y;

            // Position at right edge of the text area
            popup.HorizontalOffset = textView.ActualWidth - 36;
            popup.VerticalOffset = lineTop;
        }
        catch
        {
            // Fallback positioning
        }
    }

    // ========================================================================
    // ViewModel Property Changed
    // ========================================================================

    private void OnViewModelPropertyChanged(object? sender, PropertyChangedEventArgs e)
    {
        if (e.PropertyName == nameof(ProgramEditorViewModel.CurrentLine))
        {
            if (DataContext is ProgramEditorViewModel vm)
            {
                _lineRenderer.CurrentLine = vm.CurrentLine;
                CodeEditor.TextArea.TextView.InvalidateVisual();

                if (vm.CurrentLine > 0 && vm.CurrentLine <= CodeEditor.Document.LineCount)
                {
                    CodeEditor.ScrollToLine(vm.CurrentLine);
                }
            }
        }
        else if (e.PropertyName == nameof(ProgramEditorViewModel.ProgramSource))
        {
            if (DataContext is ProgramEditorViewModel vm)
            {
                // Only update editor if text actually differs (avoid infinite loop)
                if (CodeEditor.Text != vm.ProgramSource)
                {
                    var caretOffset = CodeEditor.CaretOffset;
                    CodeEditor.Text = vm.ProgramSource;
                    // Restore caret position (clamp to new text length)
                    CodeEditor.CaretOffset = Math.Min(caretOffset, CodeEditor.Text.Length);
                }
            }
        }
        else if (e.PropertyName == nameof(ProgramEditorViewModel.ScrollToLineRequest))
        {
            if (DataContext is ProgramEditorViewModel vm && vm.ScrollToLineRequest > 0)
            {
                if (vm.ScrollToLineRequest <= CodeEditor.Document.LineCount)
                {
                    CodeEditor.ScrollToLine(vm.ScrollToLineRequest);
                    var targetLine = CodeEditor.Document.GetLineByNumber(vm.ScrollToLineRequest);
                    CodeEditor.Select(targetLine.Offset, targetLine.Length);
                    CodeEditor.TextArea.Caret.Line = vm.ScrollToLineRequest;
                    CodeEditor.TextArea.Caret.Column = 1;
                }
            }
        }
    }

    // ========================================================================
    // Syntax Highlighting
    // ========================================================================

    private void LoadSyntaxHighlighting()
    {
        try
        {
            using var stream = GetType().Assembly.GetManifestResourceStream(
                "RobotController.UI.Resources.RPL.xshd");

            if (stream != null)
            {
                using var reader = new XmlTextReader(stream);
                var highlighting = HighlightingLoader.Load(reader, HighlightingManager.Instance);
                CodeEditor.SyntaxHighlighting = highlighting;
            }
        }
        catch
        {
            // Fallback: no syntax highlighting
        }
    }

    /// <summary>
    /// Highlight the current execution line
    /// </summary>
    public void HighlightLine(int lineNumber)
    {
        if (lineNumber <= 0 || lineNumber > CodeEditor.Document.LineCount)
            return;

        var line = CodeEditor.Document.GetLineByNumber(lineNumber);
        CodeEditor.Select(line.Offset, line.Length);
        CodeEditor.ScrollToLine(lineNumber);
    }
}

/// <summary>
/// Background renderer for current execution line highlighting (yellow)
/// </summary>
public class CurrentLineBackgroundRenderer : IBackgroundRenderer
{
    private readonly ICSharpCode.AvalonEdit.TextEditor _editor;
    private static readonly SolidColorBrush _highlightBrush = new(Color.FromArgb(60, 255, 255, 0));

    public int CurrentLine { get; set; }

    public CurrentLineBackgroundRenderer(ICSharpCode.AvalonEdit.TextEditor editor)
    {
        _editor = editor;
    }

    public KnownLayer Layer => KnownLayer.Background;

    public void Draw(TextView textView, DrawingContext drawingContext)
    {
        if (CurrentLine <= 0 || CurrentLine > _editor.Document.LineCount)
            return;

        textView.EnsureVisualLines();

        var line = _editor.Document.GetLineByNumber(CurrentLine);
        var segment = new ICSharpCode.AvalonEdit.Document.TextSegment
        {
            StartOffset = line.Offset,
            Length = line.Length
        };

        foreach (var rect in BackgroundGeometryBuilder.GetRectsForSegment(textView, segment))
        {
            var fullWidthRect = new Rect(0, rect.Top, textView.ActualWidth, rect.Height);
            drawingContext.DrawRectangle(_highlightBrush, null, fullWidthRect);
        }
    }
}

/// <summary>
/// Background renderer for motion instruction line highlighting (orange).
/// Shows when the caret is on a PTP/LIN/CIRC line.
/// </summary>
public class MotionLineBackgroundRenderer : IBackgroundRenderer
{
    private readonly ICSharpCode.AvalonEdit.TextEditor _editor;
    private static readonly SolidColorBrush _highlightBrush = new(Color.FromArgb(40, 255, 165, 0)); // Orange

    public int HighlightedLine { get; set; }

    public MotionLineBackgroundRenderer(ICSharpCode.AvalonEdit.TextEditor editor)
    {
        _editor = editor;
    }

    public KnownLayer Layer => KnownLayer.Background;

    public void Draw(TextView textView, DrawingContext drawingContext)
    {
        if (HighlightedLine <= 0 || HighlightedLine > _editor.Document.LineCount)
            return;

        textView.EnsureVisualLines();

        var line = _editor.Document.GetLineByNumber(HighlightedLine);
        var segment = new ICSharpCode.AvalonEdit.Document.TextSegment
        {
            StartOffset = line.Offset,
            Length = line.Length
        };

        foreach (var rect in BackgroundGeometryBuilder.GetRectsForSegment(textView, segment))
        {
            var fullWidthRect = new Rect(0, rect.Top, textView.ActualWidth, rect.Height);
            drawingContext.DrawRectangle(_highlightBrush, null, fullWidthRect);
        }
    }
}
