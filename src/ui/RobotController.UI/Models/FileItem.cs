using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace RobotController.UI.Models;

public enum FileItemType
{
    Module,         // Paired .src + .dat (shown as single entry in Module filter)
    SourceFile,     // .src file (shown in Detail filter)
    DataFile,       // .dat file
    Folder,         // Subdirectory
    Other           // Any other file type
}

public enum FilterMode
{
    Module,   // Groups .src + .dat pairs into single "Module" entry
    Detail    // Shows every file individually
}

public enum ProgramState
{
    None,       // Not selected / not active
    Selected,   // Program selected (Satzanwahl) — ▶
    Running,    // Program currently executing — ▶▶
    Paused,     // Program paused — ‖
    Error       // Program error — ✕
}

public class FileItem : INotifyPropertyChanged
{
    public string Name { get; set; } = "";
    public string DisplayName { get; set; } = "";
    public string Extension { get; set; } = "";
    public string Comment { get; set; } = "";
    public string SizeDisplay { get; set; } = "";
    public string ModifiedDisplay { get; set; } = "";
    public DateTime Modified { get; set; }
    public long SizeBytes { get; set; }
    public string FullPath { get; set; } = "";
    public bool IsDirectory { get; set; }
    public bool IsComposite { get; set; }
    public FileItemType Type { get; set; }

    private bool _isReadOnly;
    public bool IsReadOnly
    {
        get => _isReadOnly;
        set
        {
            if (_isReadOnly != value)
            {
                _isReadOnly = value;
                OnPropertyChanged();
                OnPropertyChanged(nameof(TypeIcon));
            }
        }
    }

    private bool _isActiveProgram;
    /// <summary>True when this file is the currently selected program (Satzanwahl)</summary>
    public bool IsActiveProgram
    {
        get => _isActiveProgram;
        set
        {
            if (_isActiveProgram != value)
            {
                _isActiveProgram = value;
                OnPropertyChanged();
                OnPropertyChanged(nameof(TypeIcon));
            }
        }
    }

    private ProgramState _programState = ProgramState.None;
    public ProgramState ProgramState
    {
        get => _programState;
        set
        {
            if (_programState != value)
            {
                _programState = value;
                OnPropertyChanged();
                OnPropertyChanged(nameof(TypeIcon));
            }
        }
    }

    public string TypeIcon
    {
        get
        {
            // Program state icons take priority for active programs
            if (IsActiveProgram && Type is FileItemType.Module or FileItemType.SourceFile)
            {
                return ProgramState switch
                {
                    ProgramState.Running => "\u25B6\u25B6",  // ▶▶ Running
                    ProgramState.Paused  => "\u2016",         // ‖ Paused
                    ProgramState.Error   => "\u2715",         // ✕ Error
                    _ => "\u25B6"                              // ▶ Selected
                };
            }

            // Read-only lock icon
            if (IsReadOnly && !IsDirectory)
                return "\U0001F512";  // 🔒

            return Type switch
            {
                FileItemType.Module => "\u2699",           // ⚙ Gear icon for module
                FileItemType.SourceFile => "\U0001F4C4",   // 📄 Page icon for .src
                FileItemType.DataFile => "\U0001F4CA",     // 📊 Chart icon for .dat
                FileItemType.Folder => "\U0001F4C1",       // 📁 Folder icon
                _ => "\U0001F4C4"
            };
        }
    }

    public event PropertyChangedEventHandler? PropertyChanged;

    private void OnPropertyChanged([CallerMemberName] string? name = null)
    {
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }
}
