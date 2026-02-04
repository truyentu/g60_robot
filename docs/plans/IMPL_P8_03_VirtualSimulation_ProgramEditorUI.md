# IMPL_P8_03: Virtual Simulation - Program Editor UI

| Metadata | Value |
|----------|-------|
| Phase | Phase 8: Virtual Simulation |
| Plan ID | IMPL_P8_03 |
| Title | Program Editor UI with Syntax Highlighting |
| Priority | P1 |
| Status | PLANNED |
| Created | 2026-02-04 |
| Depends On | IMPL_P8_02 |

---

## 1. Overview

### 1.1 Mục tiêu

Xây dựng Program Editor UI trong WPF với:
- Syntax highlighting cho RPL
- Line numbers
- Error highlighting
- Point database panel
- Execution controls
- Current line indicator

### 1.2 Technology Choice

**Recommended: AvalonEdit** (MIT License)
- Mature WPF text editor control
- Built-in syntax highlighting
- Line numbers, folding
- Used by SharpDevelop, ILSpy

Alternative: ICSharpCode.TextEditor, FastColoredTextBox

---

## 2. Prerequisites

- [ ] IMPL_P8_02 hoàn thành (Interpreter works)
- [ ] NuGet package: AvalonEdit

---

## 3. Implementation Steps

### Step 1: Add AvalonEdit NuGet Package

**Command:**
```bash
cd src/ui/RobotController.UI
dotnet add package AvalonEdit
```

**Validation:**
- Package installed
- Build succeeds

---

### Step 2: Create RPL Syntax Highlighting Definition

**Files to create:**
- `src/ui/RobotController.UI/Resources/RPL.xshd`

```xml
<?xml version="1.0"?>
<SyntaxDefinition name="RPL" xmlns="http://icsharpcode.net/sharpdevelop/syntaxdefinition/2008">
    <Color name="Comment" foreground="#6A9955" fontStyle="italic"/>
    <Color name="String" foreground="#CE9178"/>
    <Color name="Number" foreground="#B5CEA8"/>
    <Color name="Keyword" foreground="#569CD6" fontWeight="bold"/>
    <Color name="Motion" foreground="#4EC9B0" fontWeight="bold"/>
    <Color name="Point" foreground="#DCDCAA"/>
    <Color name="SystemVar" foreground="#9CDCFE"/>
    <Color name="Operator" foreground="#D4D4D4"/>

    <RuleSet ignoreCase="true">
        <!-- Comments -->
        <Span color="Comment" begin=";"/>

        <!-- Strings -->
        <Span color="String">
            <Begin>"</Begin>
            <End>"</End>
        </Span>

        <!-- Keywords -->
        <Keywords color="Keyword">
            <Word>DEF</Word>
            <Word>END</Word>
            <Word>DECL</Word>
            <Word>REAL</Word>
            <Word>INT</Word>
            <Word>BOOL</Word>
            <Word>IF</Word>
            <Word>THEN</Word>
            <Word>ELSE</Word>
            <Word>ENDIF</Word>
            <Word>LOOP</Word>
            <Word>ENDLOOP</Word>
            <Word>WHILE</Word>
            <Word>ENDWHILE</Word>
            <Word>WAIT</Word>
            <Word>SEC</Word>
            <Word>FOR</Word>
            <Word>TRUE</Word>
            <Word>FALSE</Word>
            <Word>AND</Word>
            <Word>OR</Word>
            <Word>NOT</Word>
        </Keywords>

        <!-- Motion commands -->
        <Keywords color="Motion">
            <Word>PTP</Word>
            <Word>LIN</Word>
            <Word>CIRC</Word>
            <Word>HOME</Word>
            <Word>VEL</Word>
            <Word>ACC</Word>
            <Word>CONT</Word>
        </Keywords>

        <!-- System variables -->
        <Rule color="SystemVar">
            \$[A-Za-z_][A-Za-z0-9_]*(\[[0-9]+\])?
        </Rule>

        <!-- Points (P followed by number) -->
        <Rule color="Point">
            \bP[0-9]+\b
        </Rule>

        <!-- Numbers -->
        <Rule color="Number">
            \b[0-9]+\.?[0-9]*\b
        </Rule>
    </RuleSet>
</SyntaxDefinition>
```

**Validation:**
- File parses as valid XML

---

### Step 3: Create ProgramEditorViewModel

**Files to create:**
- `src/ui/RobotController.UI/ViewModels/ProgramEditorViewModel.cs`

```csharp
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Services;
using Serilog;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels;

public partial class ProgramEditorViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;

    [ObservableProperty]
    private string _programSource = @"DEF NewProgram()
    ; Your code here
    PTP HOME
END";

    [ObservableProperty]
    private string _programName = "NewProgram";

    [ObservableProperty]
    private int _currentLine = 0;

    [ObservableProperty]
    private string _executionState = "IDLE";

    [ObservableProperty]
    private bool _isRunning;

    [ObservableProperty]
    private bool _isPaused;

    [ObservableProperty]
    private ObservableCollection<string> _errors = new();

    [ObservableProperty]
    private ObservableCollection<PointInfo> _points = new();

    public ProgramEditorViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;
    }

    [RelayCommand]
    private async Task LoadProgramAsync()
    {
        var response = await _ipcClient.LoadProgramAsync(ProgramSource);
        if (!response.Success)
        {
            Errors.Clear();
            foreach (var error in response.Errors)
            {
                Errors.Add(error);
            }
        }
        else
        {
            Errors.Clear();
            Log.Information("Program loaded successfully");
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
    }

    [RelayCommand]
    private async Task StepProgramAsync()
    {
        if (!IsRunning)
        {
            await LoadProgramAsync();
            if (Errors.Count > 0) return;
        }

        await _ipcClient.StepProgramAsync();
        IsRunning = true;
    }

    [RelayCommand]
    private async Task PauseProgramAsync()
    {
        await _ipcClient.PauseProgramAsync();
        IsPaused = true;
    }

    [RelayCommand]
    private async Task ResumeProgramAsync()
    {
        await _ipcClient.ResumeProgramAsync();
        IsPaused = false;
    }

    [RelayCommand]
    private async Task StopProgramAsync()
    {
        await _ipcClient.StopProgramAsync();
        IsRunning = false;
        IsPaused = false;
    }

    [RelayCommand]
    private async Task ResetProgramAsync()
    {
        await _ipcClient.ResetProgramAsync();
        CurrentLine = 0;
        IsRunning = false;
        IsPaused = false;
    }

    [RelayCommand]
    private async Task TeachPointAsync()
    {
        // Get current robot position
        var status = await _ipcClient.GetStatusAsync();
        if (status == null) return;

        // Generate point name
        string pointName = $"P{Points.Count + 1}";

        // Add to points
        Points.Add(new PointInfo
        {
            Name = pointName,
            X = status.TcpPosition[0],
            Y = status.TcpPosition[1],
            Z = status.TcpPosition[2],
            Rx = status.TcpPosition[3],
            Ry = status.TcpPosition[4],
            Rz = status.TcpPosition[5]
        });

        // Send to Core
        await _ipcClient.SetPointAsync(pointName, status.TcpPosition);
    }

    [RelayCommand]
    private void InsertPoint(PointInfo point)
    {
        // Insert point reference at cursor
        // This will be handled by the View
    }
}

public class PointInfo
{
    public string Name { get; set; } = "";
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double Rx { get; set; }
    public double Ry { get; set; }
    public double Rz { get; set; }
}
```

**Validation:**
- ViewModel compiles

---

### Step 4: Create ProgramEditor View

**Files to create:**
- `src/ui/RobotController.UI/Views/Controls/ProgramEditor.xaml`
- `src/ui/RobotController.UI/Views/Controls/ProgramEditor.xaml.cs`

```xml
<UserControl x:Class="RobotController.UI.Views.Controls.ProgramEditor"
             xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
             xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
             xmlns:avalon="http://icsharpcode.net/sharpdevelop/avalonedit">

    <Grid>
        <Grid.RowDefinitions>
            <RowDefinition Height="Auto"/>
            <RowDefinition Height="*"/>
            <RowDefinition Height="Auto"/>
            <RowDefinition Height="150"/>
        </Grid.RowDefinitions>

        <Grid.ColumnDefinitions>
            <ColumnDefinition Width="*"/>
            <ColumnDefinition Width="200"/>
        </Grid.ColumnDefinitions>

        <!-- Toolbar -->
        <Border Grid.Row="0" Grid.ColumnSpan="2" Background="#333333" Padding="5">
            <StackPanel Orientation="Horizontal">
                <Button Content="▶ Run" Command="{Binding RunProgramCommand}"
                        Margin="2" Padding="10,5" Background="#4CAF50" Foreground="White"/>
                <Button Content="⏸ Pause" Command="{Binding PauseProgramCommand}"
                        Margin="2" Padding="10,5" Background="#FF9800"/>
                <Button Content="⏹ Stop" Command="{Binding StopProgramCommand}"
                        Margin="2" Padding="10,5" Background="#F44336" Foreground="White"/>
                <Button Content="⏭ Step" Command="{Binding StepProgramCommand}"
                        Margin="2" Padding="10,5"/>
                <Button Content="↩ Reset" Command="{Binding ResetProgramCommand}"
                        Margin="2" Padding="10,5"/>

                <Separator Margin="10,0"/>

                <TextBlock Text="Override:" VerticalAlignment="Center" Foreground="White"/>
                <Slider Minimum="1" Maximum="100" Value="100" Width="100"
                        VerticalAlignment="Center" Margin="5,0"/>
                <TextBlock Text="100%" VerticalAlignment="Center" Foreground="White"/>
            </StackPanel>
        </Border>

        <!-- Code Editor -->
        <avalon:TextEditor x:Name="CodeEditor"
                           Grid.Row="1" Grid.Column="0"
                           FontFamily="Consolas"
                           FontSize="14"
                           ShowLineNumbers="True"
                           Background="#1E1E1E"
                           Foreground="#D4D4D4"
                           LineNumbersForeground="#858585"/>

        <!-- Point Database -->
        <Border Grid.Row="1" Grid.Column="1" Background="#252525">
            <DockPanel>
                <StackPanel DockPanel.Dock="Top" Background="#333333" Margin="0,0,0,5">
                    <TextBlock Text="POINTS" FontWeight="Bold" Foreground="#AAAAAA" Margin="10,5"/>
                </StackPanel>

                <StackPanel DockPanel.Dock="Bottom" Margin="5">
                    <Button Content="📍 Teach Point" Command="{Binding TeachPointCommand}"
                            Margin="0,0,0,5"/>
                </StackPanel>

                <ListBox ItemsSource="{Binding Points}"
                         Background="Transparent"
                         BorderThickness="0">
                    <ListBox.ItemTemplate>
                        <DataTemplate>
                            <Border Background="#353535" CornerRadius="3" Padding="8,5" Margin="3">
                                <StackPanel>
                                    <TextBlock Text="{Binding Name}" FontWeight="Bold" Foreground="#4EC9B0"/>
                                    <TextBlock Foreground="#888888" FontSize="10">
                                        <Run Text="X:"/><Run Text="{Binding X, StringFormat=F1}"/>
                                        <Run Text=" Y:"/><Run Text="{Binding Y, StringFormat=F1}"/>
                                        <Run Text=" Z:"/><Run Text="{Binding Z, StringFormat=F1}"/>
                                    </TextBlock>
                                </StackPanel>
                            </Border>
                        </DataTemplate>
                    </ListBox.ItemTemplate>
                </ListBox>
            </DockPanel>
        </Border>

        <!-- Status Bar -->
        <Border Grid.Row="2" Grid.ColumnSpan="2" Background="#333333" Padding="5">
            <StackPanel Orientation="Horizontal">
                <TextBlock Text="State: " Foreground="Gray"/>
                <TextBlock Text="{Binding ExecutionState}" Foreground="White" FontWeight="Bold"/>
                <TextBlock Text=" | Line: " Foreground="Gray" Margin="20,0,0,0"/>
                <TextBlock Text="{Binding CurrentLine}" Foreground="White"/>
            </StackPanel>
        </Border>

        <!-- Errors Panel -->
        <Border Grid.Row="3" Grid.ColumnSpan="2" Background="#1E1E1E">
            <DockPanel>
                <TextBlock DockPanel.Dock="Top" Text="ERRORS" Foreground="#F44336"
                           FontWeight="Bold" Margin="10,5"/>
                <ListBox ItemsSource="{Binding Errors}"
                         Background="Transparent"
                         BorderThickness="0"
                         Foreground="#F44336"/>
            </DockPanel>
        </Border>
    </Grid>
</UserControl>
```

**Code-behind:**

```csharp
using ICSharpCode.AvalonEdit.Highlighting;
using ICSharpCode.AvalonEdit.Highlighting.Xshd;
using System.Windows.Controls;
using System.Xml;

namespace RobotController.UI.Views.Controls;

public partial class ProgramEditor : UserControl
{
    public ProgramEditor()
    {
        InitializeComponent();
        LoadSyntaxHighlighting();
    }

    private void LoadSyntaxHighlighting()
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
}
```

**Validation:**
- Editor displays with syntax highlighting
- Line numbers visible
- Dark theme applied

---

### Step 5: Add Current Line Highlighting

**Modify ProgramEditor.xaml.cs:**

```csharp
public void HighlightLine(int lineNumber)
{
    if (lineNumber < 1 || lineNumber > CodeEditor.Document.LineCount)
        return;

    var line = CodeEditor.Document.GetLineByNumber(lineNumber);

    // Clear previous highlighting
    // Add background marker for current line

    CodeEditor.TextArea.TextView.InvalidateLayer(
        ICSharpCode.AvalonEdit.Rendering.KnownLayer.Selection);
}
```

**Add line highlight renderer:**

```csharp
public class CurrentLineHighlighter : IBackgroundRenderer
{
    private readonly TextEditor _editor;
    private int _currentLine = 0;

    public CurrentLineHighlighter(TextEditor editor)
    {
        _editor = editor;
    }

    public int CurrentLine
    {
        get => _currentLine;
        set
        {
            _currentLine = value;
            _editor.TextArea.TextView.InvalidateLayer(Layer);
        }
    }

    public KnownLayer Layer => KnownLayer.Selection;

    public void Draw(TextView textView, DrawingContext drawingContext)
    {
        if (_currentLine < 1) return;

        textView.EnsureVisualLines();
        var line = _editor.Document.GetLineByNumber(_currentLine);
        var segment = new TextSegment { StartOffset = line.Offset, Length = line.Length };

        foreach (var rect in BackgroundGeometryBuilder.GetRectsForSegment(textView, segment))
        {
            drawingContext.DrawRectangle(
                new SolidColorBrush(Color.FromArgb(50, 255, 255, 0)),
                null,
                new Rect(0, rect.Top, textView.ActualWidth, rect.Height));
        }
    }
}
```

**Validation:**
- Current execution line highlighted in yellow

---

### Step 6: Integrate with Main Window

**Modify MainWindow.xaml to add Program Editor page:**

```xml
<!-- In PageContentOverlay, add for Index 1 (Program) -->
<pages:ProgramEditorView DataContext="{Binding ProgramEditor}">
    <!-- Visibility triggers -->
</pages:ProgramEditorView>
```

**Add to MainViewModel:**

```csharp
[ObservableProperty]
private ProgramEditorViewModel? _programEditor;

// In constructor
ProgramEditor = new ProgramEditorViewModel(_ipcClient);
```

**Validation:**
- Program page shows editor
- Can type code with syntax highlighting
- Execution controls work

---

## 4. Completion Checklist

- [ ] Step 1: AvalonEdit package installed
- [ ] Step 2: RPL.xshd syntax definition created
- [ ] Step 3: ProgramEditorViewModel implemented
- [ ] Step 4: ProgramEditor View created
- [ ] Step 5: Current line highlighting works
- [ ] Step 6: Integrated with MainWindow
- [ ] Build succeeds
- [ ] Editor shows syntax highlighting
- [ ] Can run/step programs from UI

---

## 5. UI Mockup

```
┌─────────────────────────────────────────────────────────────────────┐
│ [▶ Run] [⏸ Pause] [⏹ Stop] [⏭ Step] [↩ Reset]  Override: [===100%] │
├────────────────────────────────────────────────┬────────────────────┤
│ 1│ DEF WeldPath()                              │ POINTS             │
│ 2│   PTP HOME                                  │ ┌────────────────┐ │
│►3│   PTP P1 VEL=80%              ← Yellow BG   │ │ P1             │ │
│ 4│   LIN P2 VEL=10mm/s                         │ │ X:400 Y:100 Z: │ │
│ 5│   LIN P3 VEL=10mm/s                         │ └────────────────┘ │
│ 6│   PTP HOME                                  │ ┌────────────────┐ │
│ 7│ END                                         │ │ P2             │ │
│                                                │ │ X:400 Y:200 Z: │ │
│                                                │ └────────────────┘ │
│                                                │                    │
│                                                │ [📍 Teach Point]   │
├────────────────────────────────────────────────┴────────────────────┤
│ State: RUNNING | Line: 3                                            │
├─────────────────────────────────────────────────────────────────────┤
│ ERRORS                                                              │
│ (empty)                                                             │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 6. Next Steps

- IMPL_P8_04: Export System (save/load programs)
