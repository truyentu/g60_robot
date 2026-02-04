using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Services;
using Serilog;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels;

/// <summary>
/// Point information for display
/// </summary>
public partial class PointInfo : ObservableObject
{
    [ObservableProperty]
    private string _name = "";

    [ObservableProperty]
    private double _x;

    [ObservableProperty]
    private double _y;

    [ObservableProperty]
    private double _z;

    [ObservableProperty]
    private double _rx;

    [ObservableProperty]
    private double _ry;

    [ObservableProperty]
    private double _rz;
}

/// <summary>
/// ViewModel for Program Editor
/// </summary>
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
    private int _overridePercent = 100;

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
        try
        {
            var response = await _ipcClient.LoadProgramAsync(ProgramSource);
            Errors.Clear();

            if (response == null)
            {
                Errors.Add("Failed to communicate with Core");
                return;
            }

            if (!response.Success)
            {
                Errors.Add(response.Error ?? "Unknown error");
            }
            else
            {
                ProgramName = response.ProgramName;
                Log.Information("Program loaded: {Name}", ProgramName);
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to load program");
            Errors.Add(ex.Message);
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
        ExecutionState = "RUNNING";

        // Start polling for state updates
        _ = PollProgramStateAsync();
    }

    [RelayCommand]
    private async Task StepProgramAsync()
    {
        if (!IsRunning)
        {
            await LoadProgramAsync();
            if (Errors.Count > 0) return;
            IsRunning = true;
        }

        var response = await _ipcClient.StepProgramAsync();
        if (response != null)
        {
            ExecutionState = response.State;
            CurrentLine = response.CurrentLine;

            if (response.State == "COMPLETED" || response.State == "ERROR")
            {
                IsRunning = false;
            }
        }
    }

    [RelayCommand]
    private async Task PauseProgramAsync()
    {
        await _ipcClient.PauseProgramAsync();
        IsPaused = true;
        ExecutionState = "PAUSED";
    }

    [RelayCommand]
    private async Task ResumeProgramAsync()
    {
        await _ipcClient.RunProgramAsync();
        IsPaused = false;
        ExecutionState = "RUNNING";
    }

    [RelayCommand]
    private async Task StopProgramAsync()
    {
        await _ipcClient.StopProgramAsync();
        IsRunning = false;
        IsPaused = false;
        ExecutionState = "STOPPED";
    }

    [RelayCommand]
    private async Task ResetProgramAsync()
    {
        await _ipcClient.ResetProgramAsync();
        CurrentLine = 0;
        IsRunning = false;
        IsPaused = false;
        ExecutionState = "IDLE";
    }

    [RelayCommand]
    private async Task TeachPointAsync()
    {
        try
        {
            // Get current robot position
            var status = await _ipcClient.GetStatusAsync();
            if (status == null) return;

            // Generate point name
            string pointName = $"P{Points.Count + 1}";

            // Add to local points list
            Points.Add(new PointInfo
            {
                Name = pointName,
                X = status.TcpPosition.Count > 0 ? status.TcpPosition[0] : 0,
                Y = status.TcpPosition.Count > 1 ? status.TcpPosition[1] : 0,
                Z = status.TcpPosition.Count > 2 ? status.TcpPosition[2] : 0,
                Rx = status.TcpPosition.Count > 3 ? status.TcpPosition[3] : 0,
                Ry = status.TcpPosition.Count > 4 ? status.TcpPosition[4] : 0,
                Rz = status.TcpPosition.Count > 5 ? status.TcpPosition[5] : 0
            });

            // Send to Core
            await _ipcClient.SetPointAsync(pointName, status.TcpPosition.ToArray());

            Log.Information("Taught point {Name}", pointName);
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to teach point");
        }
    }

    [RelayCommand]
    private async Task RefreshPointsAsync()
    {
        try
        {
            var response = await _ipcClient.GetPointsAsync();
            if (response != null)
            {
                Points.Clear();
                foreach (var (name, values) in response.Points)
                {
                    Points.Add(new PointInfo
                    {
                        Name = name,
                        X = values.Length > 0 ? values[0] : 0,
                        Y = values.Length > 1 ? values[1] : 0,
                        Z = values.Length > 2 ? values[2] : 0,
                        Rx = values.Length > 3 ? values[3] : 0,
                        Ry = values.Length > 4 ? values[4] : 0,
                        Rz = values.Length > 5 ? values[5] : 0
                    });
                }
            }
        }
        catch (Exception ex)
        {
            Log.Error(ex, "Failed to refresh points");
        }
    }

    private async Task PollProgramStateAsync()
    {
        while (IsRunning && !IsPaused)
        {
            try
            {
                var state = await _ipcClient.GetProgramStateAsync();
                if (state != null)
                {
                    ExecutionState = state.State;
                    CurrentLine = state.CurrentLine;

                    if (state.State == "COMPLETED" || state.State == "ERROR" || state.State == "STOPPED")
                    {
                        IsRunning = false;
                        break;
                    }
                }

                await Task.Delay(100);
            }
            catch
            {
                break;
            }
        }
    }
}
