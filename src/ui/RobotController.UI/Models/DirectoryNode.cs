using CommunityToolkit.Mvvm.ComponentModel;
using System.Collections.ObjectModel;

namespace RobotController.UI.Models;

public partial class DirectoryNode : ObservableObject
{
    public string Name { get; set; } = "";
    public string FullPath { get; set; } = "";
    public string DisplayPath { get; set; } = "";

    [ObservableProperty]
    private bool _isExpanded;

    [ObservableProperty]
    private bool _isSelected;

    public ObservableCollection<DirectoryNode> Children { get; set; } = new();
}
