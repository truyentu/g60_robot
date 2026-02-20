using System.Windows;
using System.Windows.Controls;
using RobotController.UI.ViewModels;

namespace RobotController.UI.Editor;

public partial class InlineCommandEditor : UserControl
{
    /// <summary>Raised when user confirms insertion (INSERT mode). Payload = KRL text to insert.</summary>
    public event EventHandler<string>? InsertRequested;

    /// <summary>Raised when user cancels insertion.</summary>
    public event EventHandler? CancelRequested;

    /// <summary>Raised when user clicks Teach button. Payload = "target" or "aux".</summary>
    public event EventHandler<string>? TeachPointRequested;

    public InlineCommandEditor()
    {
        InitializeComponent();
    }

    private void InsertConfirm_Click(object sender, RoutedEventArgs e)
    {
        if (DataContext is SelectedCommandViewModel vm)
        {
            var krl = vm.BuildInsertText();
            if (!string.IsNullOrWhiteSpace(krl))
                InsertRequested?.Invoke(this, krl);
        }
    }

    private void InsertCancel_Click(object sender, RoutedEventArgs e)
    {
        CancelRequested?.Invoke(this, EventArgs.Empty);
    }

    private void TeachTargetPoint_Click(object sender, RoutedEventArgs e)
    {
        TeachPointRequested?.Invoke(this, "target");
    }

    private void TeachAuxPoint_Click(object sender, RoutedEventArgs e)
    {
        TeachPointRequested?.Invoke(this, "aux");
    }
}
