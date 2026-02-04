using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels.Pages;

/// <summary>
/// ViewModel for Program management view
/// </summary>
public partial class ProgramViewModel : ObservableObject
{
    [ObservableProperty]
    private ObservableCollection<ProgramItem> _programs = new()
    {
        new ProgramItem { Name = "WeldSeam01", Description = "Butt weld seam program" },
        new ProgramItem { Name = "WeldSeam02", Description = "Fillet weld program" },
        new ProgramItem { Name = "PickPlace01", Description = "Pick and place demo" },
    };

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

    partial void OnSelectedProgramChanged(ProgramItem? value)
    {
        if (value != null)
        {
            // Load program code
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
        PointCount = lines.Count(l => l.Trim().StartsWith("MOV", StringComparison.OrdinalIgnoreCase));
    }

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
    private void DeleteProgram()
    {
        if (SelectedProgram != null)
        {
            Programs.Remove(SelectedProgram);
            SelectedProgram = Programs.FirstOrDefault();
        }
    }

    [RelayCommand]
    private void ImportProgram()
    {
        // TODO: Open file dialog
        ProgramStatus = "Import not implemented";
    }

    [RelayCommand]
    private void SaveProgram()
    {
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
