using System.Text.Json.Serialization;

namespace RobotController.Common.Config;

/// <summary>
/// Viewport configuration
/// </summary>
public class ViewportConfig
{
    [JsonPropertyName("background_color")]
    public string BackgroundColor { get; set; } = "#1E1E1E";

    [JsonPropertyName("grid_visible")]
    public bool GridVisible { get; set; } = true;

    [JsonPropertyName("grid_size")]
    public int GridSize { get; set; } = 100;

    [JsonPropertyName("axis_visible")]
    public bool AxisVisible { get; set; } = true;

    [JsonPropertyName("show_tcp_marker")]
    public bool ShowTcpMarker { get; set; } = true;
}

/// <summary>
/// Panel configuration
/// </summary>
public class PanelsConfig
{
    [JsonPropertyName("navigation_width")]
    public int NavigationWidth { get; set; } = 200;

    [JsonPropertyName("properties_width")]
    public int PropertiesWidth { get; set; } = 250;

    [JsonPropertyName("show_properties")]
    public bool ShowProperties { get; set; } = true;
}

/// <summary>
/// UI appearance settings
/// </summary>
public class UiSettings
{
    [JsonPropertyName("theme")]
    public string Theme { get; set; } = "dark";

    [JsonPropertyName("language")]
    public string Language { get; set; } = "en-US";

    [JsonPropertyName("viewport")]
    public ViewportConfig Viewport { get; set; } = new();

    [JsonPropertyName("panels")]
    public PanelsConfig Panels { get; set; } = new();
}

/// <summary>
/// Connection settings
/// </summary>
public class ConnectionConfig
{
    [JsonPropertyName("core_address")]
    public string CoreAddress { get; set; } = "tcp://localhost";

    [JsonPropertyName("rep_port")]
    public int RepPort { get; set; } = 5555;

    [JsonPropertyName("pub_port")]
    public int PubPort { get; set; } = 5556;

    [JsonPropertyName("auto_connect")]
    public bool AutoConnect { get; set; } = true;

    [JsonPropertyName("reconnect_interval_ms")]
    public int ReconnectIntervalMs { get; set; } = 5000;

    [JsonPropertyName("request_timeout_ms")]
    public int RequestTimeoutMs { get; set; } = 5000;

    /// <summary>
    /// Get full REP address
    /// </summary>
    public string RepAddress => $"{CoreAddress}:{RepPort}";

    /// <summary>
    /// Get full PUB address (for SUB socket)
    /// </summary>
    public string SubAddress => $"{CoreAddress}:{PubPort}";
}

/// <summary>
/// Window state
/// </summary>
public class WindowConfig
{
    [JsonPropertyName("width")]
    public int Width { get; set; } = 1400;

    [JsonPropertyName("height")]
    public int Height { get; set; } = 800;

    [JsonPropertyName("left")]
    public int Left { get; set; } = 100;

    [JsonPropertyName("top")]
    public int Top { get; set; } = 100;

    [JsonPropertyName("maximized")]
    public bool Maximized { get; set; } = false;
}

/// <summary>
/// Complete UI configuration
/// </summary>
public class UiConfig
{
    [JsonPropertyName("ui")]
    public UiSettings Ui { get; set; } = new();

    [JsonPropertyName("connection")]
    public ConnectionConfig Connection { get; set; } = new();

    [JsonPropertyName("recent_files")]
    public List<string> RecentFiles { get; set; } = new();

    [JsonPropertyName("window")]
    public WindowConfig Window { get; set; } = new();

    [JsonPropertyName("last_active_package_id")]
    public string LastActivePackageId { get; set; } = "";
}
