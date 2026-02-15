using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.UI.Models;
using RobotController.UI.Services;
using Serilog;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Windows;

namespace RobotController.UI.ViewModels.Pages;

public partial class NavigatorViewModel : ObservableObject
{
    private readonly WorkspaceService _workspace;
    private readonly ArchiveService _archive;
    private FileSystemWatcher? _watcher;
    private DateTime _lastAutoRefresh = DateTime.MinValue;

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

    // Search filter
    [ObservableProperty]
    private string _searchText = "";

    // New Program Dialog state
    [ObservableProperty]
    private bool _isNewProgramDialogOpen;

    [ObservableProperty]
    private string _newProgramName = "";

    [ObservableProperty]
    private string _newProgramComment = "";

    // New Item Type chooser dialog state
    [ObservableProperty]
    private bool _isNewTypeDialogOpen;

    // Rename Dialog state
    [ObservableProperty]
    private bool _isRenameDialogOpen;

    [ObservableProperty]
    private string _renameNewName = "";

    // Delete Confirm Dialog state
    [ObservableProperty]
    private bool _isDeleteConfirmOpen;

    [ObservableProperty]
    private string _deleteTargetName = "";

    // Properties Dialog state
    [ObservableProperty]
    private bool _isPropertiesDialogOpen;

    [ObservableProperty]
    private string _propsName = "";

    [ObservableProperty]
    private string _propsType = "";

    [ObservableProperty]
    private string _propsSize = "";

    [ObservableProperty]
    private string _propsCreated = "";

    [ObservableProperty]
    private string _propsModified = "";

    [ObservableProperty]
    private string _propsComment = "";

    [ObservableProperty]
    private string _propsPath = "";

    /// <summary>Full path of the currently active (Satzanwahl) program</summary>
    [ObservableProperty]
    private string _activeProgramPath = "";

    // Restore dialog state
    [ObservableProperty] private bool _isRestoreDialogOpen;
    [ObservableProperty] private ObservableCollection<ArchiveEntry> _archiveContents = new();
    [ObservableProperty] private string _restoreStatusMessage = "";
    [ObservableProperty] private string _restoreZipPath = "";

    // Compare dialog state
    [ObservableProperty] private bool _isCompareDialogOpen;
    [ObservableProperty] private ObservableCollection<CompareResult> _compareResults = new();

    public WorkspaceService Workspace => _workspace;

    // Clipboard state for Cut/Copy → Paste
    private string? _clipboardPath;
    private bool _isCut;

    // Sort state
    private string _sortColumn = "Name";
    private bool _sortAscending = true;

    public event Action<string>? OpenProgramRequested;
    public event Action<string>? SelectProgramRequested;
    public event Action? CancelSelectRequested;

    public NavigatorViewModel(WorkspaceService workspace)
    {
        _workspace = workspace;
        _archive = new ArchiveService(workspace);
    }

    partial void OnSelectedFilterIndexChanged(int value)
    {
        IsDetailMode = value == 1;
        RefreshFileList();
    }

    partial void OnSearchTextChanged(string value)
    {
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
            StatusText = $"Selected: 1 object    Total: {FileList.Count}";
        }
    }

    // ===== Commands =====

    [RelayCommand]
    private void NewItem()
    {
        if (SelectedDirectory == null) return;
        IsNewTypeDialogOpen = true;
    }

    [RelayCommand]
    private void NewTypeModule()
    {
        IsNewTypeDialogOpen = false;
        NewProgram();
    }

    [RelayCommand]
    private void NewTypeFolder()
    {
        IsNewTypeDialogOpen = false;
        NewFolder();
    }

    [RelayCommand]
    private void CancelNewType()
    {
        IsNewTypeDialogOpen = false;
    }

    [RelayCommand]
    private void NewProgram()
    {
        if (SelectedDirectory == null) return;

        // Open dialog for name input
        NewProgramName = "";
        NewProgramComment = "";
        IsNewProgramDialogOpen = true;
    }

    [RelayCommand]
    private void ConfirmNewProgram()
    {
        if (SelectedDirectory == null || string.IsNullOrWhiteSpace(NewProgramName))
        {
            IsNewProgramDialogOpen = false;
            return;
        }

        var name = NewProgramName.Trim();

        // Validate name (no special chars, no extension)
        if (name.IndexOfAny(Path.GetInvalidFileNameChars()) >= 0)
        {
            StatusText = "Invalid program name";
            return;
        }

        // Check if already exists
        if (File.Exists(Path.Combine(SelectedDirectory.FullPath, name + ".src")))
        {
            StatusText = $"Program '{name}' already exists";
            return;
        }

        _workspace.CreateProgramFromTemplate(SelectedDirectory.FullPath, name, NewProgramComment);
        IsNewProgramDialogOpen = false;
        RefreshFileList();
        StatusText = $"Created: {name}";
        Log.Information("Navigator: Created program {Name} from template", name);
    }

    [RelayCommand]
    private void CancelNewProgram()
    {
        IsNewProgramDialogOpen = false;
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
                 SelectedFile.Type == FileItemType.SourceFile)
        {
            OpenProgramRequested?.Invoke(SelectedFile.FullPath);
        }
    }

    [RelayCommand]
    private void Select()
    {
        if (SelectedFile == null || SelectedFile.IsDirectory) return;

        // TODO: Call ExecutorService.LoadProgram(SelectedFile.FullPath) when available
        SelectProgramRequested?.Invoke(SelectedFile.FullPath);
        StatusText = $"Selected for execution: {SelectedFile.DisplayName}";
        Log.Information("Navigator: Selected for execution: {Name}", SelectedFile.DisplayName);
    }

    [RelayCommand]
    private void Delete()
    {
        if (SelectedFile == null) return;
        if (SelectedFile.IsReadOnly)
        {
            StatusText = "Cannot delete: read-only item";
            return;
        }

        DeleteTargetName = SelectedFile.DisplayName;
        IsDeleteConfirmOpen = true;
    }

    [RelayCommand]
    private void ConfirmDelete()
    {
        IsDeleteConfirmOpen = false;
        if (SelectedFile == null) return;

        var pathToDelete = SelectedFile.FullPath;
        var nameToDelete = DeleteTargetName;
        try
        {
            _workspace.DeleteFileOrFolder(pathToDelete);
            RefreshTreeAndFileList();
            StatusText = $"Deleted: {nameToDelete}";
            Log.Information("Navigator: Deleted {Path}", pathToDelete);
        }
        catch (Exception ex)
        {
            StatusText = $"Delete failed: {ex.Message}";
            Log.Error(ex, "Navigator: Delete failed for {Path}", pathToDelete);
        }
    }

    [RelayCommand]
    private void CancelDelete()
    {
        IsDeleteConfirmOpen = false;
    }

    /// <summary>Set the active program (called by MainViewModel after Satzanwahl)</summary>
    public void SetActiveProgram(string fullPath)
    {
        ActiveProgramPath = fullPath;
        // Update IsActiveProgram flag on each FileItem so icon changes
        foreach (var item in FileList)
        {
            item.IsActiveProgram = string.Equals(
                Path.GetFullPath(item.FullPath),
                Path.GetFullPath(fullPath),
                StringComparison.OrdinalIgnoreCase);
        }
    }

    /// <summary>Update program state icon for the active program (Running/Paused/Error)</summary>
    public void UpdateProgramState(ProgramState state)
    {
        foreach (var item in FileList)
        {
            if (item.IsActiveProgram)
                item.ProgramState = state;
        }
    }

    [RelayCommand]
    private void CancelSelect()
    {
        ActiveProgramPath = "";
        foreach (var item in FileList)
            item.IsActiveProgram = false;
        CancelSelectRequested?.Invoke();
        StatusText = "Program selection cancelled";
        Log.Information("Navigator: Program selection cancelled");
    }

    [RelayCommand]
    private void Properties()
    {
        if (SelectedFile == null) return;

        PropsName = SelectedFile.DisplayName;
        PropsType = SelectedFile.Type.ToString();
        PropsSize = SelectedFile.IsDirectory ? "" : SelectedFile.SizeDisplay + $" ({SelectedFile.SizeBytes:N0} bytes)";
        PropsComment = SelectedFile.Comment;
        PropsPath = _workspace.ToDisplayPath(SelectedFile.FullPath);

        try
        {
            if (SelectedFile.IsDirectory)
            {
                var di = new DirectoryInfo(SelectedFile.FullPath);
                PropsCreated = di.CreationTime.ToString("yyyy-MM-dd HH:mm:ss");
                PropsModified = di.LastWriteTime.ToString("yyyy-MM-dd HH:mm:ss");
            }
            else
            {
                var fi = new FileInfo(SelectedFile.FullPath);
                PropsCreated = fi.CreationTime.ToString("yyyy-MM-dd HH:mm:ss");
                PropsModified = fi.LastWriteTime.ToString("yyyy-MM-dd HH:mm:ss");
            }
        }
        catch
        {
            PropsCreated = "";
            PropsModified = SelectedFile.ModifiedDisplay;
        }

        IsPropertiesDialogOpen = true;
    }

    [RelayCommand]
    private void CloseProperties()
    {
        IsPropertiesDialogOpen = false;
    }

    [RelayCommand]
    private void Cut()
    {
        if (SelectedFile == null) return;
        if (SelectedFile.IsReadOnly)
        {
            StatusText = "Cannot cut: read-only item";
            return;
        }

        _clipboardPath = SelectedFile.FullPath;
        _isCut = true;
        StatusText = $"Cut: {SelectedFile.DisplayName}";
        Log.Information("Navigator: Cut {Path}", SelectedFile.FullPath);
    }

    [RelayCommand]
    private void Copy()
    {
        if (SelectedFile == null) return;

        _clipboardPath = SelectedFile.FullPath;
        _isCut = false;
        StatusText = $"Copied: {SelectedFile.DisplayName}";
        Log.Information("Navigator: Copied to clipboard {Path}", SelectedFile.FullPath);
    }

    [RelayCommand]
    private void Paste()
    {
        if (_clipboardPath == null || SelectedDirectory == null) return;

        try
        {
            if (_isCut)
            {
                _workspace.MoveFileOrFolder(_clipboardPath, SelectedDirectory.FullPath);
                Log.Information("Navigator: Moved {Path} to {Dest}", _clipboardPath, SelectedDirectory.FullPath);
                _clipboardPath = null;
            }
            else
            {
                _workspace.CopyFileOrFolder(_clipboardPath, SelectedDirectory.FullPath);
                Log.Information("Navigator: Pasted copy of {Path} to {Dest}", _clipboardPath, SelectedDirectory.FullPath);
            }
            _isCut = false;
            RefreshTreeAndFileList();
            StatusText = "Paste completed";
        }
        catch (Exception ex)
        {
            StatusText = $"Paste failed: {ex.Message}";
            Log.Error(ex, "Navigator: Paste failed");
        }
    }

    [RelayCommand]
    private void Rename()
    {
        if (SelectedFile == null) return;
        if (SelectedFile.IsReadOnly)
        {
            StatusText = "Cannot rename: read-only item";
            return;
        }

        // Pre-fill with current name (without extension for modules)
        RenameNewName = SelectedFile.Type == FileItemType.Module
            ? SelectedFile.Name
            : SelectedFile.Name;
        IsRenameDialogOpen = true;
    }

    [RelayCommand]
    private void ConfirmRename()
    {
        IsRenameDialogOpen = false;
        if (SelectedFile == null || string.IsNullOrWhiteSpace(RenameNewName)) return;

        var newName = RenameNewName.Trim();

        // Validate name
        if (newName.IndexOfAny(Path.GetInvalidFileNameChars()) >= 0)
        {
            StatusText = "Invalid name";
            return;
        }

        RenameSelected(newName);
    }

    [RelayCommand]
    private void CancelRename()
    {
        IsRenameDialogOpen = false;
    }

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

    // ===== Archive/Restore =====

    [RelayCommand]
    private void ArchiveAll()
    {
        var dlg = new Microsoft.Win32.SaveFileDialog
        {
            Title = "Archive Workspace",
            Filter = "ZIP Archive (*.zip)|*.zip",
            FileName = $"KRC_Archive_{DateTime.Now:yyyyMMdd_HHmmss}.zip"
        };
        if (dlg.ShowDialog() != true) return;

        try
        {
            _archive.ArchiveAll(dlg.FileName);
            StatusText = $"Archive saved: {Path.GetFileName(dlg.FileName)}";
            Log.Information("Navigator: Archive All to {Path}", dlg.FileName);
        }
        catch (Exception ex)
        {
            StatusText = $"Archive failed: {ex.Message}";
            Log.Error(ex, "Navigator: Archive All failed");
        }
    }

    [RelayCommand]
    private void ArchiveSelected()
    {
        if (SelectedFile == null && SelectedDirectory == null) return;

        var sourcePath = SelectedFile?.FullPath ?? SelectedDirectory?.FullPath ?? "";
        if (string.IsNullOrEmpty(sourcePath)) return;

        var sourceName = Path.GetFileNameWithoutExtension(sourcePath);
        var dlg = new Microsoft.Win32.SaveFileDialog
        {
            Title = "Archive Selected",
            Filter = "ZIP Archive (*.zip)|*.zip",
            FileName = $"{sourceName}_Archive_{DateTime.Now:yyyyMMdd_HHmmss}.zip"
        };
        if (dlg.ShowDialog() != true) return;

        try
        {
            _archive.ArchiveSelected(sourcePath, dlg.FileName);
            StatusText = $"Archive saved: {Path.GetFileName(dlg.FileName)}";
            Log.Information("Navigator: Archive Selected {Source} to {Path}", sourcePath, dlg.FileName);
        }
        catch (Exception ex)
        {
            StatusText = $"Archive failed: {ex.Message}";
            Log.Error(ex, "Navigator: Archive Selected failed");
        }
    }

    [RelayCommand]
    private void Restore()
    {
        var dlg = new Microsoft.Win32.OpenFileDialog
        {
            Title = "Restore from Archive",
            Filter = "ZIP Archive (*.zip)|*.zip"
        };
        if (dlg.ShowDialog() != true) return;

        RestoreZipPath = dlg.FileName;

        try
        {
            var entries = _archive.ListArchiveContents(dlg.FileName);
            ArchiveContents = new ObservableCollection<ArchiveEntry>(entries);
            RestoreStatusMessage = $"{entries.Count} files in archive";
            IsRestoreDialogOpen = true;
        }
        catch (Exception ex)
        {
            StatusText = $"Cannot read archive: {ex.Message}";
            Log.Error(ex, "Navigator: Restore failed to read archive");
        }
    }

    [RelayCommand]
    private void ConfirmRestore()
    {
        IsRestoreDialogOpen = false;

        try
        {
            var selectedPaths = ArchiveContents
                .Where(e => e.IsSelected)
                .Select(e => e.RelativePath)
                .ToList();

            RestoreResult result;
            if (selectedPaths.Count == ArchiveContents.Count)
            {
                result = _archive.RestoreAll(RestoreZipPath);
            }
            else
            {
                result = _archive.RestoreSelected(RestoreZipPath, selectedPaths);
            }

            StatusText = $"Restored: {result.Restored} files ({result.Overwritten} overwritten, {result.Skipped} skipped)";
            Log.Information("Navigator: Restore complete — {Restored} files", result.Restored);

            // Refresh Navigator
            RefreshTreeAndFileList();
        }
        catch (Exception ex)
        {
            StatusText = $"Restore failed: {ex.Message}";
            Log.Error(ex, "Navigator: Restore failed");
        }
    }

    [RelayCommand]
    private void CancelRestore()
    {
        IsRestoreDialogOpen = false;
    }

    [RelayCommand]
    private void CompareArchive()
    {
        var dlg = new Microsoft.Win32.OpenFileDialog
        {
            Title = "Compare with Archive",
            Filter = "ZIP Archive (*.zip)|*.zip"
        };
        if (dlg.ShowDialog() != true) return;

        try
        {
            var results = _archive.CompareWithWorkspace(dlg.FileName);
            CompareResults = new ObservableCollection<CompareResult>(results);
            IsCompareDialogOpen = true;
        }
        catch (Exception ex)
        {
            StatusText = $"Compare failed: {ex.Message}";
            Log.Error(ex, "Navigator: Compare failed");
        }
    }

    [RelayCommand]
    private void CloseCompare()
    {
        IsCompareDialogOpen = false;
    }

    // ===== Sorting =====

    public void SortByColumn(string column)
    {
        if (_sortColumn == column)
            _sortAscending = !_sortAscending;
        else
        {
            _sortColumn = column;
            _sortAscending = true;
        }
        ApplySort();
    }

    private void ApplySort()
    {
        // Separate directories (always first) from files
        var dirs = FileList.Where(f => f.IsDirectory).ToList();
        var files = FileList.Where(f => !f.IsDirectory).ToList();

        IEnumerable<FileItem> sorted = _sortColumn switch
        {
            "Name" => _sortAscending
                ? files.OrderBy(f => f.DisplayName, StringComparer.OrdinalIgnoreCase)
                : files.OrderByDescending(f => f.DisplayName, StringComparer.OrdinalIgnoreCase),
            "Type" => _sortAscending
                ? files.OrderBy(f => f.Extension, StringComparer.OrdinalIgnoreCase)
                : files.OrderByDescending(f => f.Extension, StringComparer.OrdinalIgnoreCase),
            "Size" => _sortAscending
                ? files.OrderBy(f => f.SizeBytes)
                : files.OrderByDescending(f => f.SizeBytes),
            "Changed" => _sortAscending
                ? files.OrderBy(f => f.Modified)
                : files.OrderByDescending(f => f.Modified),
            "Comment" => _sortAscending
                ? files.OrderBy(f => f.Comment, StringComparer.OrdinalIgnoreCase)
                : files.OrderByDescending(f => f.Comment, StringComparer.OrdinalIgnoreCase),
            _ => files
        };

        var result = new List<FileItem>(dirs);
        result.AddRange(sorted);
        FileList = new ObservableCollection<FileItem>(result);

        // Re-apply active program highlight
        if (!string.IsNullOrEmpty(ActiveProgramPath))
            SetActiveProgram(ActiveProgramPath);
    }

    // ===== Double-click =====

    public void OnFileDoubleClick(FileItem item)
    {
        if (item.IsDirectory)
        {
            NavigateToDirectory(item.FullPath);
        }
        else if (item.Type == FileItemType.Module || item.Type == FileItemType.SourceFile)
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

        // Auto-select R1\Program\
        var r1Program = FindNodeByName(DirectoryTree, "Program");
        if (r1Program != null)
        {
            var r1 = FindNodeByName(DirectoryTree, "R1");
            if (r1 != null) r1.IsExpanded = true;

            r1Program.IsExpanded = true;
            SelectedDirectory = r1Program;
        }
        else if (DirectoryTree.Count > 0)
        {
            SelectedDirectory = DirectoryTree[0];
        }

        SetupFileWatcher();
    }

    private void SetupFileWatcher()
    {
        _watcher?.Dispose();

        var rootDir = _workspace.WorkspaceRoot;
        if (!Directory.Exists(rootDir)) return;

        _watcher = new FileSystemWatcher(rootDir)
        {
            IncludeSubdirectories = true,
            NotifyFilter = NotifyFilters.FileName | NotifyFilters.DirectoryName | NotifyFilters.LastWrite
        };
        _watcher.Changed += OnFileSystemChanged;
        _watcher.Created += OnFileSystemChanged;
        _watcher.Deleted += OnFileSystemChanged;
        _watcher.Renamed += OnFileSystemChanged;
        _watcher.EnableRaisingEvents = true;
    }

    private void OnFileSystemChanged(object sender, FileSystemEventArgs e)
    {
        var now = DateTime.UtcNow;
        if ((now - _lastAutoRefresh).TotalMilliseconds < 500) return;
        _lastAutoRefresh = now;

        Application.Current?.Dispatcher.InvokeAsync(() =>
        {
            RefreshFileList();
        });
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

        // Apply search filter
        if (!string.IsNullOrWhiteSpace(SearchText))
        {
            items = items.Where(i => i.DisplayName.Contains(SearchText, StringComparison.OrdinalIgnoreCase)).ToList();
        }

        FileList = new ObservableCollection<FileItem>(items);
        StatusText = $"Total: {items.Count} objects";

        // Re-apply active program highlight
        if (!string.IsNullOrEmpty(ActiveProgramPath))
            SetActiveProgram(ActiveProgramPath);
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
