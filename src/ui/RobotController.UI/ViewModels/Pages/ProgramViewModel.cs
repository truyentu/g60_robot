using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using Microsoft.Win32;
using RobotController.UI.Services;
using Serilog;
using System.Collections.ObjectModel;
using System.IO;
using System.Text.Json;

namespace RobotController.UI.ViewModels.Pages;

/// <summary>
/// ViewModel for Program management view — includes Navigator softkey actions
/// </summary>
public partial class ProgramViewModel : ObservableObject
{
    private WorkspaceService? _workspace;
    private string? _currentProgramPath;

    [ObservableProperty]
    private ObservableCollection<ProgramItem> _programs = new();

    [ObservableProperty]
    private ProgramItem? _selectedProgram;

    [ObservableProperty]
    private string _programCode = "";

    [ObservableProperty]
    private int _lineCount;

    [ObservableProperty]
    private int _pointCount;

    [ObservableProperty]
    private string _programStatus = "Ready";

    [ObservableProperty]
    private bool _showModulesOnly;

    /// <summary>Set workspace service for file-based operations</summary>
    public void SetWorkspaceService(WorkspaceService workspace)
    {
        _workspace = workspace;
        LoadProgramsFromWorkspace();
    }

    /// <summary>Scan workspace/R1/Programs/ and populate the program list from disk</summary>
    public void LoadProgramsFromWorkspace()
    {
        if (_workspace == null) return;

        Programs.Clear();
        var programsDir = _workspace.ProgramsDir;
        if (!Directory.Exists(programsDir)) return;

        // Recursively find all .src files (KRL source modules)
        foreach (var programFile in Directory.EnumerateFiles(programsDir, "*.src", SearchOption.AllDirectories))
        {
            var name = Path.GetFileNameWithoutExtension(programFile);
            var comment = "";
            // Extract &COMMENT from first 10 lines
            try
            {
                using var reader = new StreamReader(programFile);
                for (int i = 0; i < 10; i++)
                {
                    var line = reader.ReadLine();
                    if (line == null) break;
                    if (line.StartsWith("&COMMENT", StringComparison.OrdinalIgnoreCase))
                    {
                        var idx = line.IndexOf(' ');
                        comment = idx >= 0 ? line.Substring(idx + 1).Trim() : "";
                        break;
                    }
                }
            }
            catch { }

            Programs.Add(new ProgramItem { Name = name, Description = comment });
        }

        Log.Information("Loaded {Count} programs from workspace", Programs.Count);
    }

    /// <summary>Open a .src file from disk into the editor</summary>
    public void OpenProgramFile(string programPath)
    {
        if (_workspace == null) return;

        _currentProgramPath = programPath;
        ProgramCode = _workspace.LoadProgramSource(programPath);

        var name = Path.GetFileNameWithoutExtension(programPath);
        ProgramStatus = $"Opened: {name}";
        Log.Information("Program opened from file: {Path}", programPath);

        UpdateCounts();
    }

    /// <summary>Save current program back to its file on disk</summary>
    public void SaveCurrentProgramToFile()
    {
        if (_workspace == null || _currentProgramPath == null) return;

        _workspace.SaveProgramSource(_currentProgramPath, ProgramCode);
        ProgramStatus = $"Saved: {Path.GetFileNameWithoutExtension(_currentProgramPath)}";
        Log.Information("Program saved to file: {Path}", _currentProgramPath);
    }

    /// <summary>Current file path of the open program (null if none)</summary>
    public string? CurrentProgramPath => _currentProgramPath;

    // Clipboard for Cut/Copy/Paste
    private ProgramItem? _clipboard;
    private bool _isCut;

    partial void OnSelectedProgramChanged(ProgramItem? value)
    {
        if (value != null)
        {
            ProgramCode = $"; Program: {value.Name}\n; {value.Description}\n\nMOVJ P1 V=50%\nMOVL P2 V=100mm/s\nARCON\nMOVL P3 V=10mm/s\nARCOFF\nMOVJ P4 V=50%\nEND";
            UpdateCounts();
        }
    }

    partial void OnProgramCodeChanged(string value)
    {
        UpdateCounts();
    }

    private void UpdateCounts()
    {
        if (string.IsNullOrEmpty(ProgramCode))
        {
            LineCount = 0;
            PointCount = 0;
            return;
        }

        var lines = ProgramCode.Split('\n');
        LineCount = lines.Length;
        PointCount = lines.Count(l =>
        {
            var trimmed = l.Trim();
            return trimmed.StartsWith("MOV", StringComparison.OrdinalIgnoreCase)
                || trimmed.StartsWith("PTP ", StringComparison.OrdinalIgnoreCase)
                || trimmed.StartsWith("LIN ", StringComparison.OrdinalIgnoreCase)
                || trimmed.StartsWith("CIRC ", StringComparison.OrdinalIgnoreCase);
        });
    }

    // ====== Navigator Softkey Actions ======

    /// <summary>Create program with a specific name (called from MainViewModel dialog)</summary>
    public void CreateProgramWithName(string name, string description)
    {
        // Create actual files on disk if workspace available
        if (_workspace != null)
        {
            _workspace.CreateProgram(_workspace.ProgramsDir, name);
            LoadProgramsFromWorkspace();
            SelectedProgram = Programs.FirstOrDefault(p => p.Name == name);
            ProgramStatus = $"Created: {name}";
            Log.Information("New program created on disk: {Name}", name);
            return;
        }

        var newProgram = new ProgramItem
        {
            Name = name,
            Description = string.IsNullOrEmpty(description) ? "New program" : description
        };
        Programs.Add(newProgram);
        SelectedProgram = newProgram;
        ProgramStatus = $"Created: {name}";
        Log.Information("New program created: {Name}", name);
    }

    /// <summary>Duplicate selected program with "_Copy" suffix</summary>
    public void DuplicateSelectedProgram()
    {
        if (SelectedProgram == null) return;

        var baseName = SelectedProgram.Name;
        var copyName = baseName + "_Copy";

        // Avoid name collision
        int suffix = 1;
        while (Programs.Any(p => p.Name == copyName))
        {
            copyName = $"{baseName}_Copy{suffix++}";
        }

        var duplicate = new ProgramItem
        {
            Name = copyName,
            Description = SelectedProgram.Description
        };
        Programs.Add(duplicate);
        SelectedProgram = duplicate;
        ProgramStatus = $"Duplicated: {copyName}";
        Log.Information("Program duplicated: {Source} -> {Copy}", baseName, copyName);
    }

    /// <summary>Archive all programs to ZIP (preferred) or JSON (fallback) file</summary>
    public async Task ArchiveProgramsAsync()
    {
        var dlg = new SaveFileDialog
        {
            Title = "Archive Programs",
            Filter = "ZIP Archive (*.zip)|*.zip|JSON Archive (*.json)|*.json",
            FileName = $"programs_archive_{DateTime.Now:yyyyMMdd_HHmmss}.zip"
        };

        if (dlg.ShowDialog() != true) return;

        try
        {
            if (dlg.FileName.EndsWith(".zip", StringComparison.OrdinalIgnoreCase))
            {
                if (_workspace != null)
                {
                    var archiveService = new ArchiveService(_workspace);
                    await Task.Run(() => archiveService.ArchiveAll(dlg.FileName));
                    ProgramStatus = $"Archived workspace to {Path.GetFileName(dlg.FileName)}";
                }
            }
            else
            {
                var archive = Programs.Select(p => new
                {
                    p.Name,
                    p.Description,
                    Code = p == SelectedProgram ? ProgramCode : ""
                }).ToList();

                var json = JsonSerializer.Serialize(archive, new JsonSerializerOptions { WriteIndented = true });
                await File.WriteAllTextAsync(dlg.FileName, json);
                ProgramStatus = $"Archived {Programs.Count} programs (JSON)";
            }

            Log.Information("Programs archived to: {Path}", dlg.FileName);
        }
        catch (Exception ex)
        {
            ProgramStatus = $"Archive failed: {ex.Message}";
            Log.Error(ex, "Failed to archive programs");
        }
    }

    /// <summary>Rename selected program</summary>
    public void RenameSelectedProgram(string newName)
    {
        if (SelectedProgram == null) return;
        var oldName = SelectedProgram.Name;
        SelectedProgram.Name = newName;

        // Force UI refresh by re-setting
        var idx = Programs.IndexOf(SelectedProgram);
        if (idx >= 0)
        {
            var item = Programs[idx];
            Programs.RemoveAt(idx);
            Programs.Insert(idx, item);
            SelectedProgram = item;
        }

        ProgramStatus = $"Renamed: {oldName} -> {newName}";
        Log.Information("Program renamed: {Old} -> {New}", oldName, newName);
    }

    // ====== Edit Submenu Actions ======

    public void EditMenuAction(string action)
    {
        switch (action)
        {
            case "Cut": CutProgram(); break;
            case "Copy": CopyProgram(); break;
            case "Paste": PasteProgram(); break;
            case "Rename": break; // Handled via MainViewModel dialog
            case "Filter": ToggleFilter(); break;
        }
    }

    private void CutProgram()
    {
        if (SelectedProgram == null) return;
        _clipboard = SelectedProgram;
        _isCut = true;
        ProgramStatus = $"Cut: {SelectedProgram.Name}";
    }

    private void CopyProgram()
    {
        if (SelectedProgram == null) return;
        _clipboard = new ProgramItem
        {
            Name = SelectedProgram.Name,
            Description = SelectedProgram.Description
        };
        _isCut = false;
        ProgramStatus = $"Copied: {SelectedProgram.Name}";
    }

    private void PasteProgram()
    {
        if (_clipboard == null)
        {
            ProgramStatus = "Nothing to paste";
            return;
        }

        if (_isCut)
        {
            // Move: remove from old location (if still exists) and add
            Programs.Remove(_clipboard);
            Programs.Add(_clipboard);
            SelectedProgram = _clipboard;
            _isCut = false;
        }
        else
        {
            // Copy: create new with "_Paste" suffix
            var pasteName = _clipboard.Name + "_Paste";
            int suffix = 1;
            while (Programs.Any(p => p.Name == pasteName))
            {
                pasteName = $"{_clipboard.Name}_Paste{suffix++}";
            }

            var pasted = new ProgramItem
            {
                Name = pasteName,
                Description = _clipboard.Description
            };
            Programs.Add(pasted);
            SelectedProgram = pasted;
        }

        ProgramStatus = $"Pasted: {SelectedProgram?.Name}";
    }

    private void ToggleFilter()
    {
        ShowModulesOnly = !ShowModulesOnly;
        ProgramStatus = ShowModulesOnly ? "Filter: Modules only" : "Filter: All files";
    }

    // ====== Existing Commands ======

    [RelayCommand]
    private void NewProgram()
    {
        var newProgram = new ProgramItem
        {
            Name = $"NewProgram{Programs.Count + 1}",
            Description = "New program"
        };
        Programs.Add(newProgram);
        SelectedProgram = newProgram;
    }

    [RelayCommand]
    public void DeleteProgram()
    {
        if (SelectedProgram == null) return;

        // Delete actual files on disk if workspace available
        if (_workspace != null)
        {
            var programPath = Path.Combine(_workspace.ProgramsDir, SelectedProgram.Name + ".src");
            // Also search subdirectories for the file
            if (!File.Exists(programPath))
            {
                var found = Directory.EnumerateFiles(_workspace.ProgramsDir, SelectedProgram.Name + ".src", SearchOption.AllDirectories).FirstOrDefault();
                if (found != null) programPath = found;
            }
            if (File.Exists(programPath))
            {
                _workspace.DeleteFileOrFolder(programPath);
                Log.Information("Program deleted from disk: {Path}", programPath);
            }
        }

        Programs.Remove(SelectedProgram);
        SelectedProgram = Programs.FirstOrDefault();
    }

    [RelayCommand]
    private void ImportProgram()
    {
        ProgramStatus = "Import not implemented";
    }

    [RelayCommand]
    private void SaveProgram()
    {
        if (_currentProgramPath != null)
            SaveCurrentProgramToFile();
        else
            ProgramStatus = "Saved";
    }

    [RelayCommand]
    private void RunProgram()
    {
        if (SelectedProgram != null)
        {
            ProgramStatus = "Running...";
        }
    }

    [RelayCommand]
    private void TeachPoint()
    {
        ProgramCode += $"\nP{PointCount + 1} = CURPOS";
        ProgramStatus = $"Taught P{PointCount + 1}";
    }

    [RelayCommand]
    private void MoveToPoint()
    {
        ProgramStatus = "Moving to point...";
    }

    [RelayCommand]
    private void AddLinear()
    {
        ProgramCode += $"\nMOVL P{PointCount + 1} V=100mm/s";
    }

    [RelayCommand]
    private void AddJoint()
    {
        ProgramCode += $"\nMOVJ P{PointCount + 1} V=50%";
    }

    [RelayCommand]
    private void AddWait()
    {
        ProgramCode += "\nWAIT 1.0";
    }

    [RelayCommand]
    private void AddIO()
    {
        ProgramCode += "\nDOUT[1] = ON";
    }
}

public class ProgramItem
{
    public string Name { get; set; } = "";
    public string Description { get; set; } = "";
}
