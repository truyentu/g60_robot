using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RobotController.Common.Messages;
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

    // UDP Packet Monitor
    public ObservableCollection<UdpPacketEntry> UdpPackets { get; } = new();

    [ObservableProperty]
    private bool _isCapturing = true;

    [ObservableProperty]
    private string _packetFilter = "All";

    [ObservableProperty]
    private int _txCount;

    [ObservableProperty]
    private int _rxCount;

    private static readonly Dictionary<int, string> TxTypeNames = new()
    {
        { 0x10, "ENABLE" },
        { 0x11, "DISABLE" },
        { 0x12, "MOVE_ABS" },
        { 0x13, "MOVE_REL" },
        { 0x14, "MOVE_VEL" },
        { 0x15, "STOP" },
        { 0x16, "E_STOP" },
        { 0x17, "HOME" },
        { 0x18, "SET_PID" },
        { 0x19, "SET_IO" },
        { 0x1A, "GET_IO" },
        { 0x1B, "RESET_ALARM" },
        { 0x20, "JOG_START" },
        { 0x21, "JOG_STOP" },
        { 0xF0, "HEARTBEAT" },
        { 0xF1, "GET_STATUS" },
        { 0xFF, "RESET" },
    };

    private static readonly Dictionary<int, string> RxTypeNames = new()
    {
        { 0x80, "STATUS" },
        { 0x81, "ALARM" },
        { 0x82, "HOME_DONE" },
        { 0x83, "IO_STATE" },
        { 0x8A, "ACK" },
        { 0x8B, "NACK" },
    };

    public DiagnosticsViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;
        _ipcClient.ConnectionStateChanged += OnConnectionChanged;
        _ipcClient.FirmwarePacketLogged += OnFirmwarePacketLogged;

        // Add some initial log entries
        AddLogEntry("INFO", "Diagnostics view initialized");
        AddLogEntry("INFO", "Waiting for connection to Core...");
    }

    private void OnFirmwarePacketLogged(object? sender, FirmwarePacketLogEntry e)
    {
        if (!IsCapturing) return;

        // Apply filter
        if (PacketFilter == "TX" && e.Dir != "TX") return;
        if (PacketFilter == "RX" && e.Dir != "RX") return;

        var typeName = e.Dir == "TX"
            ? (TxTypeNames.TryGetValue(e.Type, out var txName) ? txName : $"0x{e.Type:X2}")
            : (RxTypeNames.TryGetValue(e.Type, out var rxName) ? rxName : $"0x{e.Type:X2}");

        var entry = new UdpPacketEntry
        {
            Time = DateTime.Now.ToString("HH:mm:ss.fff"),
            Dir = e.Dir,
            TypeName = typeName,
            Seq = e.Seq,
            Len = e.Len,
            Hex = e.Hex
        };

        App.Current?.Dispatcher.Invoke(() =>
        {
            UdpPackets.Add(entry);
            if (e.Dir == "TX") TxCount++;
            else RxCount++;

            // Keep max 500 entries
            while (UdpPackets.Count > 500)
            {
                UdpPackets.RemoveAt(0);
            }
        });
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
    private void ToggleCapture()
    {
        IsCapturing = !IsCapturing;
    }

    [RelayCommand]
    private void ClearPackets()
    {
        UdpPackets.Clear();
        TxCount = 0;
        RxCount = 0;
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

public class UdpPacketEntry
{
    public string Time { get; set; } = "";
    public string Dir { get; set; } = "";
    public string TypeName { get; set; } = "";
    public int Seq { get; set; }
    public int Len { get; set; }
    public string Hex { get; set; } = "";
}
