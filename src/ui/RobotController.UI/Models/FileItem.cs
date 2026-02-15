using System;

namespace RobotController.UI.Models;

public enum FileItemType
{
    Module,         // Paired .program + .dat (shown as single entry in Module filter)
    ProgramFile,    // .program file (shown in Detail filter)
    DataFile,       // .dat file (shown in Detail filter)
    Folder,         // Subdirectory
    Other           // Any other file type
}

public enum FilterMode
{
    Module,   // Groups .program + .dat pairs into single "Module" entry
    Detail    // Shows every file individually
}

public class FileItem
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
    public FileItemType Type { get; set; }

    public string TypeIcon => Type switch
    {
        FileItemType.Module => "\U0001F4CB",       // clipboard
        FileItemType.ProgramFile => "\U0001F4C4",   // page
        FileItemType.DataFile => "\U0001F4CA",       // chart
        FileItemType.Folder => "\U0001F4C1",         // folder
        _ => "\U0001F4C4"
    };
}
