using ICSharpCode.AvalonEdit.Highlighting;
using ICSharpCode.AvalonEdit.Highlighting.Xshd;
using ICSharpCode.AvalonEdit.Rendering;
using RobotController.UI.ViewModels;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Xml;

namespace RobotController.UI.Views.Controls;

/// <summary>
/// Program Editor with syntax highlighting for RPL language
/// </summary>
public partial class ProgramEditor : UserControl
{
    private readonly CurrentLineBackgroundRenderer _lineRenderer;

    public ProgramEditor()
    {
        InitializeComponent();
        LoadSyntaxHighlighting();

        // Add current line highlighting
        _lineRenderer = new CurrentLineBackgroundRenderer(CodeEditor);
        CodeEditor.TextArea.TextView.BackgroundRenderers.Add(_lineRenderer);

        // Bind text changes to ViewModel
        CodeEditor.TextChanged += (s, e) =>
        {
            if (DataContext is ProgramEditorViewModel vm)
            {
                vm.ProgramSource = CodeEditor.Text;
            }
        };

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
            }
        };
    }

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
    }

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
/// Background renderer for current execution line highlighting
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
