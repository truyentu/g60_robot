using System.Collections.Generic;
using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

public class FirmwareConnectRequest
{
    [JsonPropertyName("port")] public string Port { get; set; } = "";
    [JsonPropertyName("baud_rate")] public int BaudRate { get; set; } = 115200;
}

public class FirmwareConnectResponse
{
    [JsonPropertyName("success")] public bool Success { get; set; }
    [JsonPropertyName("mode")] public string Mode { get; set; } = "SIM";
    [JsonPropertyName("port")] public string Port { get; set; } = "";
    [JsonPropertyName("driver_name")] public string DriverName { get; set; } = "";
    [JsonPropertyName("error")] public string Error { get; set; } = "";
}

public class FirmwareModeResponse
{
    [JsonPropertyName("mode")] public string Mode { get; set; } = "SIM";
    [JsonPropertyName("is_connected")] public bool IsConnected { get; set; }
    [JsonPropertyName("driver_name")] public string DriverName { get; set; } = "";
    [JsonPropertyName("is_simulation")] public bool IsSimulation { get; set; } = true;
    [JsonPropertyName("port")] public string Port { get; set; } = "";
}

public class ScanPortsResponse
{
    [JsonPropertyName("ports")] public List<string> Ports { get; set; } = new();
    [JsonPropertyName("count")] public int Count { get; set; }
}

public class Stm32ConnectRequest
{
    [JsonPropertyName("ip")] public string Ip { get; set; } = "192.168.1.100";
    [JsonPropertyName("port")] public int Port { get; set; } = 5001;
}

public class Stm32ConnectResponse
{
    [JsonPropertyName("success")] public bool Success { get; set; }
    [JsonPropertyName("mode")] public string Mode { get; set; } = "SIM";
    [JsonPropertyName("ip")] public string Ip { get; set; } = "";
    [JsonPropertyName("port")] public int Port { get; set; }
    [JsonPropertyName("driver_name")] public string DriverName { get; set; } = "";
    [JsonPropertyName("error")] public string Error { get; set; } = "";
}
