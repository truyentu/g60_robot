using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Services;
using System.Collections.ObjectModel;

namespace RobotController.UI.ViewModels.Pages;

/// <summary>
/// ViewModel for Diagnostics view
/// </summary>
public partial class DiagnosticsViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;

    public ObservableCollection<LogEntry> LogEntries { get; } = new();

    [ObservableProperty]
    private string _logFilter = "All";

    [ObservableProperty]
    private int _errorCount;

    [ObservableProperty]
    private int _warningCount;

    // Core Status
    [ObservableProperty]
    private string _coreConnectionStatus = "Disconnected";

    [ObservableProperty]
    private bool _isConnected;

    [ObservableProperty]
    private string _coreVersion = "N/A";

    [ObservableProperty]
    private string _uptime = "00:00:00";

    [ObservableProperty]
    private string _cycleTime = "0.0 ms";

    [ObservableProperty]
    private string _cpuUsage = "0%";

    // Communication Stats
    [ObservableProperty]
    private int _messagesSent;

    [ObservableProperty]
    private int _messagesReceived;

    [ObservableProperty]
    private int _commErrors;

    [ObservableProperty]
    private string _latency = "0 ms";

    public DiagnosticsViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;
        _ipcClient.ConnectionStateChanged += OnConnectionChanged;

        // Add some initial log entries
        AddLogEntry("INFO", "Diagnostics view initialized");
        AddLogEntry("INFO", "Waiting for connection to Core...");
    }

    private void OnConnectionChanged(object? sender, bool connected)
    {
        IsConnected = connected;
        CoreConnectionStatus = connected ? "Connected" : "Disconnected";

        if (connected)
        {
            AddLogEntry("INFO", "Connected to Core");
        }
        else
        {
            AddLogEntry("WARN", "Disconnected from Core");
        }
    }

    private void AddLogEntry(string level, string message)
    {
        App.Current?.Dispatcher.Invoke(() =>
        {
            LogEntries.Add(new LogEntry
            {
                Timestamp = DateTime.Now,
                Level = level,
                Message = message
            });

            if (level == "ERROR") ErrorCount++;
            if (level == "WARN") WarningCount++;

            // Keep only last 1000 entries
            while (LogEntries.Count > 1000)
            {
                var removed = LogEntries[0];
                LogEntries.RemoveAt(0);
                if (removed.Level == "ERROR") ErrorCount--;
                if (removed.Level == "WARN") WarningCount--;
            }
        });
    }

    [RelayCommand]
    private void ClearLog()
    {
        LogEntries.Clear();
        ErrorCount = 0;
        WarningCount = 0;
        AddLogEntry("INFO", "Log cleared");
    }

    [RelayCommand]
    private void ExportLog()
    {
        AddLogEntry("INFO", "Export not implemented yet");
    }

    [RelayCommand]
    private async Task PingCore()
    {
        if (!_ipcClient.IsConnected)
        {
            AddLogEntry("ERROR", "Cannot ping: not connected");
            return;
        }

        MessagesSent++;
        var startTime = DateTime.Now;

        var pong = await _ipcClient.PingAsync();

        if (pong != null)
        {
            MessagesReceived++;
            var elapsed = (DateTime.Now - startTime).TotalMilliseconds;
            Latency = $"{elapsed:F1} ms";
            CoreVersion = pong.CoreVersion;
            AddLogEntry("INFO", $"Ping successful: {elapsed:F1}ms, Core v{pong.CoreVersion}");
        }
        else
        {
            CommErrors++;
            AddLogEntry("ERROR", "Ping failed: no response");
        }
    }

    [RelayCommand]
    private async Task RequestStatus()
    {
        if (!_ipcClient.IsConnected)
        {
            AddLogEntry("ERROR", "Cannot request status: not connected");
            return;
        }

        MessagesSent++;
        var status = await _ipcClient.GetStatusAsync();

        if (status != null)
        {
            MessagesReceived++;
            AddLogEntry("INFO", $"Status: {status.State}, Mode: {status.Mode}, Homed: {status.Homed}");
        }
        else
        {
            CommErrors++;
            AddLogEntry("ERROR", "Status request failed");
        }
    }

    [RelayCommand]
    private void ResetStats()
    {
        MessagesSent = 0;
        MessagesReceived = 0;
        CommErrors = 0;
        Latency = "0 ms";
        AddLogEntry("INFO", "Statistics reset");
    }

    [RelayCommand]
    private async Task TestEStop()
    {
        if (!_ipcClient.IsConnected)
        {
            AddLogEntry("ERROR", "Cannot test E-Stop: not connected");
            return;
        }

        AddLogEntry("WARN", "Testing E-Stop...");
        MessagesSent++;

        bool success = await _ipcClient.SendCommandAsync("test_estop", new { });

        if (success)
        {
            MessagesReceived++;
            AddLogEntry("INFO", "E-Stop test command sent");
        }
        else
        {
            CommErrors++;
            AddLogEntry("ERROR", "E-Stop test failed");
        }
    }
}

public class LogEntry
{
    public DateTime Timestamp { get; set; }
    public string Level { get; set; } = "";
    public string Message { get; set; } = "";
}
