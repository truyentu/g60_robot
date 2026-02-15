using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.UI.Models;
using RobotController.UI.Services;
using Serilog;
using System;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;

namespace RobotController.UI.ViewModels.Pages;

public partial class NavigatorViewModel : ObservableObject
{
    private readonly WorkspaceService _workspace;

    [ObservableProperty]
    private ObservableCollection<DirectoryNode> _directoryTree = new();

    [ObservableProperty]
    private ObservableCollection<FileItem> _fileList = new();

    [ObservableProperty]
    private DirectoryNode? _selectedDirectory;

    [ObservableProperty]
    private FileItem? _selectedFile;

    [ObservableProperty]
    private string _currentDisplayPath = "KRC:\\";

    [ObservableProperty]
    private string _statusText = "";

    [ObservableProperty]
    private int _selectedFilterIndex = 0;  // 0=Module, 1=Detail

    [ObservableProperty]
    private bool _isDetailMode;

    /// <summary>Event raised when user wants to open a program file in the editor</summary>
    public event Action<string>? OpenProgramRequested;

    public NavigatorViewModel(WorkspaceService workspace)
    {
        _workspace = workspace;
    }

    partial void OnSelectedFilterIndexChanged(int value)
    {
        IsDetailMode = value == 1;
        RefreshFileList();
    }

    partial void OnSelectedDirectoryChanged(DirectoryNode? value)
    {
        if (value != null)
        {
            CurrentDisplayPath = _workspace.ToDisplayPath(value.FullPath);
            RefreshFileList();
        }
    }

    partial void OnSelectedFileChanged(FileItem? value)
    {
        if (value != null)
        {
            var selectedCount = 1; // Single selection for now
            StatusText = $"Selected: {selectedCount} object    Total: {FileList.Count}";
        }
    }

    // ===== Commands =====

    [RelayCommand]
    private void NewProgram()
    {
        if (SelectedDirectory == null) return;

        var baseName = "NewProgram";
        var name = baseName;
        int suffix = 1;

        while (File.Exists(Path.Combine(SelectedDirectory.FullPath, name + ".program")))
        {
            name = $"{baseName}{suffix++}";
        }

        _workspace.CreateProgram(SelectedDirectory.FullPath, name);
        RefreshFileList();
        Log.Information("Navigator: Created program {Name}", name);
    }

    [RelayCommand]
    private void NewFolder()
    {
        if (SelectedDirectory == null) return;

        var baseName = "NewFolder";
        var name = baseName;
        int suffix = 1;

        while (Directory.Exists(Path.Combine(SelectedDirectory.FullPath, name)))
        {
            name = $"{baseName}{suffix++}";
        }

        _workspace.CreateFolder(SelectedDirectory.FullPath, name);
        RefreshTreeAndFileList();
        Log.Information("Navigator: Created folder {Name}", name);
    }

    [RelayCommand]
    private void Open()
    {
        if (SelectedFile == null) return;

        if (SelectedFile.IsDirectory)
        {
            NavigateToDirectory(SelectedFile.FullPath);
        }
        else if (SelectedFile.Type == FileItemType.Module ||
                 SelectedFile.Extension == ".program")
        {
            var programPath = SelectedFile.Type == FileItemType.Module
                ? SelectedFile.FullPath  // Already points to .program
                : SelectedFile.FullPath;
            OpenProgramRequested?.Invoke(programPath);
        }
    }

    [RelayCommand]
    private void Select()
    {
        // Select program for execution (set block pointer)
        if (SelectedFile != null && !SelectedFile.IsDirectory)
        {
            StatusText = $"Selected for execution: {SelectedFile.DisplayName}";
        }
    }

    [RelayCommand]
    private void Delete()
    {
        if (SelectedFile == null) return;

        try
        {
            _workspace.DeleteFileOrFolder(SelectedFile.FullPath);
            RefreshTreeAndFileList();
            Log.Information("Navigator: Deleted {Path}", SelectedFile.FullPath);
        }
        catch (Exception ex)
        {
            StatusText = $"Delete failed: {ex.Message}";
            Log.Error(ex, "Navigator: Delete failed for {Path}", SelectedFile.FullPath);
        }
    }

    [RelayCommand]
    private void Copy()
    {
        if (SelectedFile == null || SelectedDirectory == null) return;

        try
        {
            _workspace.CopyFileOrFolder(SelectedFile.FullPath, SelectedDirectory.FullPath);
            RefreshFileList();
            Log.Information("Navigator: Copied {Name}", SelectedFile.DisplayName);
        }
        catch (Exception ex)
        {
            StatusText = $"Copy failed: {ex.Message}";
        }
    }

    [RelayCommand]
    private void Rename()
    {
        // Rename is handled via dialog in MainViewModel
        // This command is a placeholder for the softkey binding
        StatusText = "Rename: Use Edit menu";
    }

    /// <summary>Rename selected file/folder (called from MainViewModel with new name)</summary>
    public void RenameSelected(string newName)
    {
        if (SelectedFile == null) return;

        try
        {
            _workspace.RenameFileOrFolder(SelectedFile.FullPath, newName);
            RefreshTreeAndFileList();
            Log.Information("Navigator: Renamed to {Name}", newName);
        }
        catch (Exception ex)
        {
            StatusText = $"Rename failed: {ex.Message}";
        }
    }

    // ===== Double-click =====

    public void OnFileDoubleClick(FileItem item)
    {
        if (item.IsDirectory)
        {
            NavigateToDirectory(item.FullPath);
        }
        else if (item.Type == FileItemType.Module || item.Extension == ".program")
        {
            OpenProgramRequested?.Invoke(item.FullPath);
        }
    }

    // ===== Navigation =====

    private void NavigateToDirectory(string dirPath)
    {
        var node = FindNodeByPath(DirectoryTree, dirPath);
        if (node != null)
        {
            node.IsExpanded = true;
            SelectedDirectory = node;
        }
    }

    private DirectoryNode? FindNodeByPath(ObservableCollection<DirectoryNode> nodes, string path)
    {
        foreach (var node in nodes)
        {
            var normalizedNode = Path.GetFullPath(node.FullPath).TrimEnd('\\', '/').ToLowerInvariant();
            var normalizedTarget = Path.GetFullPath(path).TrimEnd('\\', '/').ToLowerInvariant();

            if (normalizedNode == normalizedTarget)
                return node;

            var found = FindNodeByPath(node.Children, path);
            if (found != null) return found;
        }
        return null;
    }

    // ===== Init / Refresh =====

    public void Initialize()
    {
        _workspace.EnsureWorkspaceStructure();
        DirectoryTree = new ObservableCollection<DirectoryNode>(_workspace.GetDirectoryTree());

        // Auto-select R1\Programs\
        var r1Programs = FindNodeByName(DirectoryTree, "Programs");
        if (r1Programs != null)
        {
            // Expand parent nodes
            var r1 = FindNodeByName(DirectoryTree, "R1");
            if (r1 != null) r1.IsExpanded = true;

            r1Programs.IsExpanded = true;
            SelectedDirectory = r1Programs;
        }
        else if (DirectoryTree.Count > 0)
        {
            SelectedDirectory = DirectoryTree[0];
        }
    }

    private DirectoryNode? FindNodeByName(ObservableCollection<DirectoryNode> nodes, string name)
    {
        foreach (var node in nodes)
        {
            if (node.Name.Equals(name, StringComparison.OrdinalIgnoreCase))
                return node;

            var found = FindNodeByName(node.Children, name);
            if (found != null) return found;
        }
        return null;
    }

    private void RefreshFileList()
    {
        if (SelectedDirectory == null) return;

        var filter = IsDetailMode ? FilterMode.Detail : FilterMode.Module;
        var items = _workspace.GetDirectoryContents(SelectedDirectory.FullPath, filter);
        FileList = new ObservableCollection<FileItem>(items);
        StatusText = $"Total: {items.Count} objects";
    }

    private void RefreshTreeAndFileList()
    {
        var selectedPath = SelectedDirectory?.FullPath;
        DirectoryTree = new ObservableCollection<DirectoryNode>(_workspace.GetDirectoryTree());

        if (selectedPath != null)
        {
            var node = FindNodeByPath(DirectoryTree, selectedPath);
            if (node != null)
            {
                SelectedDirectory = node;
                // Expand parent chain
                ExpandToNode(DirectoryTree, selectedPath);
            }
        }

        RefreshFileList();
    }

    private bool ExpandToNode(ObservableCollection<DirectoryNode> nodes, string targetPath)
    {
        foreach (var node in nodes)
        {
            var normalizedNode = Path.GetFullPath(node.FullPath).TrimEnd('\\', '/').ToLowerInvariant();
            var normalizedTarget = Path.GetFullPath(targetPath).TrimEnd('\\', '/').ToLowerInvariant();

            if (normalizedTarget.StartsWith(normalizedNode))
            {
                node.IsExpanded = true;
                if (normalizedNode == normalizedTarget)
                    return true;
                if (ExpandToNode(node.Children, targetPath))
                    return true;
            }
        }
        return false;
    }
}
