using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
using System;
using System.Collections.ObjectModel;
using System.IO;
using System.Text.Json;
using System.Threading.Tasks;

namespace RobotController.UI.ViewModels.Welding;

/// <summary>
/// View model for welding job management
/// </summary>
public partial class WeldingJobViewModel : ObservableObject
{
    [ObservableProperty]
    private string _jobName = "Default";

    [ObservableProperty]
    private WeldingProcess _process = WeldingProcess.MigMag;

    [ObservableProperty]
    private int _transferMode;

    [ObservableProperty]
    private bool _synergicMode = true;

    [ObservableProperty]
    private int _synergicProgram = 1;

    // Timing settings
    [ObservableProperty]
    private uint _preFlowTime = 300;

    [ObservableProperty]
    private uint _postFlowTime = 500;

    [ObservableProperty]
    private uint _craterFillTime = 500;

    [ObservableProperty]
    private uint _burnBackTime = 50;

    [ObservableProperty]
    private uint _arcRetryDelay = 200;

    [ObservableProperty]
    private int _arcRetryCount = 3;

    // Gas settings
    [ObservableProperty]
    private string _gasType = "Ar/CO2 82/18";

    [ObservableProperty]
    private double _gasFlowRate = 15.0;

    // Wire settings
    [ObservableProperty]
    private string _wireMaterial = "ER70S-6";

    [ObservableProperty]
    private double _wireDiameter = 1.0;

    // Job list
    public ObservableCollection<string> SavedJobs { get; } = new()
    {
        "Default",
        "Steel_Butt_3mm",
        "Steel_Fillet_5mm",
        "Aluminum_Butt_2mm"
    };

    [ObservableProperty]
    private string? _selectedJob;

    [ObservableProperty]
    private bool _isDirty;

    // Process options
    public ObservableCollection<WeldingProcess> ProcessOptions { get; } = new()
    {
        WeldingProcess.MigMag,
        WeldingProcess.Tig,
        WeldingProcess.Spot,
        WeldingProcess.Plasma
    };

    public ObservableCollection<string> GasTypes { get; } = new()
    {
        "Ar/CO2 82/18",
        "Ar/CO2 75/25",
        "Pure Argon",
        "Pure CO2",
        "Ar/O2 98/2"
    };

    public ObservableCollection<string> WireMaterials { get; } = new()
    {
        "ER70S-6",
        "ER70S-3",
        "ER308L",
        "ER316L",
        "ER4043"
    };

    public ObservableCollection<double> WireDiameters { get; } = new()
    {
        0.8, 0.9, 1.0, 1.2, 1.6
    };

    public WeldingJobData ToJobData(
        float current,
        float voltage,
        float wireSpeed,
        float travelSpeed)
    {
        return new WeldingJobData
        {
            Name = JobName,
            Process = (int)Process,
            TransferMode = TransferMode,
            Current = current,
            Voltage = voltage,
            WireSpeed = wireSpeed,
            TravelSpeed = travelSpeed,
            PreFlowTime = PreFlowTime,
            PostFlowTime = PostFlowTime,
            CraterFillTime = CraterFillTime,
            SynergicMode = SynergicMode,
            SynergicProgram = SynergicProgram
        };
    }

    public void LoadFromJobData(WeldingJobData data)
    {
        JobName = data.Name;
        Process = (WeldingProcess)data.Process;
        TransferMode = data.TransferMode;
        PreFlowTime = data.PreFlowTime;
        PostFlowTime = data.PostFlowTime;
        CraterFillTime = data.CraterFillTime;
        SynergicMode = data.SynergicMode;
        SynergicProgram = data.SynergicProgram;
        IsDirty = false;
    }

    [RelayCommand]
    private void NewJob()
    {
        JobName = "New Job";
        PreFlowTime = 300;
        PostFlowTime = 500;
        CraterFillTime = 500;
        BurnBackTime = 50;
        SynergicMode = true;
        SynergicProgram = 1;
        IsDirty = true;
    }

    [RelayCommand]
    private async Task SaveJobAsync()
    {
        // Save to file
        string jobsPath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "WeldingJobs");

        Directory.CreateDirectory(jobsPath);

        string filePath = Path.Combine(jobsPath, $"{JobName}.json");

        var jobData = new
        {
            Name = JobName,
            Process = Process,
            TransferMode = TransferMode,
            SynergicMode = SynergicMode,
            SynergicProgram = SynergicProgram,
            PreFlowTime = PreFlowTime,
            PostFlowTime = PostFlowTime,
            CraterFillTime = CraterFillTime,
            BurnBackTime = BurnBackTime,
            ArcRetryDelay = ArcRetryDelay,
            ArcRetryCount = ArcRetryCount,
            GasType = GasType,
            GasFlowRate = GasFlowRate,
            WireMaterial = WireMaterial,
            WireDiameter = WireDiameter
        };

        string json = JsonSerializer.Serialize(jobData, new JsonSerializerOptions
        {
            WriteIndented = true
        });

        await File.WriteAllTextAsync(filePath, json);

        if (!SavedJobs.Contains(JobName))
        {
            SavedJobs.Add(JobName);
        }

        IsDirty = false;
    }

    [RelayCommand]
    private async Task LoadJobAsync(string? jobName)
    {
        if (string.IsNullOrEmpty(jobName)) return;

        string jobsPath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "WeldingJobs");

        string filePath = Path.Combine(jobsPath, $"{jobName}.json");

        if (!File.Exists(filePath)) return;

        string json = await File.ReadAllTextAsync(filePath);
        var jobData = JsonSerializer.Deserialize<JsonElement>(json);

        JobName = jobData.GetProperty("Name").GetString() ?? jobName;
        Process = (WeldingProcess)jobData.GetProperty("Process").GetInt32();
        SynergicMode = jobData.GetProperty("SynergicMode").GetBoolean();
        PreFlowTime = jobData.GetProperty("PreFlowTime").GetUInt32();
        PostFlowTime = jobData.GetProperty("PostFlowTime").GetUInt32();
        CraterFillTime = jobData.GetProperty("CraterFillTime").GetUInt32();

        if (jobData.TryGetProperty("GasType", out var gasType))
            GasType = gasType.GetString() ?? "Ar/CO2 82/18";

        if (jobData.TryGetProperty("WireMaterial", out var wireMat))
            WireMaterial = wireMat.GetString() ?? "ER70S-6";

        SelectedJob = JobName;
        IsDirty = false;
    }

    [RelayCommand]
    private void DeleteJob(string? jobName)
    {
        if (string.IsNullOrEmpty(jobName) || jobName == "Default") return;

        string jobsPath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
            "RobotController", "WeldingJobs");

        string filePath = Path.Combine(jobsPath, $"{jobName}.json");

        if (File.Exists(filePath))
        {
            File.Delete(filePath);
        }

        SavedJobs.Remove(jobName);

        if (JobName == jobName)
        {
            NewJob();
        }
    }

    // Mark as dirty when any property changes
    partial void OnPreFlowTimeChanged(uint value) => IsDirty = true;
    partial void OnPostFlowTimeChanged(uint value) => IsDirty = true;
    partial void OnCraterFillTimeChanged(uint value) => IsDirty = true;
    partial void OnSynergicModeChanged(bool value) => IsDirty = true;
}
