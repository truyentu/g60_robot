using System.IO;
using System.Text.Json;
using Microsoft.Extensions.Logging;
using RobotController.Common.Config;

namespace RobotController.UI.Services;

/// <summary>
/// Interface for configuration service
/// </summary>
public interface IConfigService
{
    /// <summary>
    /// Current configuration
    /// </summary>
    UiConfig Config { get; }

    /// <summary>
    /// Load configuration from file
    /// </summary>
    bool Load(string? filePath = null);

    /// <summary>
    /// Save configuration to file
    /// </summary>
    bool Save(string? filePath = null);

    /// <summary>
    /// Reset to default configuration
    /// </summary>
    void Reset();

    /// <summary>
    /// Event raised when configuration changes
    /// </summary>
    event EventHandler<UiConfig>? ConfigChanged;
}

/// <summary>
/// Configuration service implementation
/// </summary>
public class ConfigService : IConfigService
{
    private readonly ILogger<ConfigService> _logger;
    private readonly string _defaultPath;
    private UiConfig _config = new();
    private readonly JsonSerializerOptions _jsonOptions;

    public UiConfig Config => _config;

    public event EventHandler<UiConfig>? ConfigChanged;

    public ConfigService(ILogger<ConfigService> logger)
    {
        _logger = logger;

        // Default path relative to executable
        _defaultPath = Path.Combine(
            AppDomain.CurrentDomain.BaseDirectory,
            "..", "..", "..", "..", "..", "config", "ui_config.json"
        );

        _jsonOptions = new JsonSerializerOptions
        {
            WriteIndented = true,
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            ReadCommentHandling = JsonCommentHandling.Skip
        };

        // Try to load on construction
        Load();
    }

    public bool Load(string? filePath = null)
    {
        string path = filePath ?? _defaultPath;
        string fullPath = Path.GetFullPath(path);

        try
        {
            if (!File.Exists(fullPath))
            {
                _logger.LogWarning("Config file not found: {Path}, using defaults", fullPath);
                _config = new UiConfig();
                return false;
            }

            _logger.LogInformation("Loading UI config from: {Path}", fullPath);

            string json = File.ReadAllText(fullPath);
            var config = JsonSerializer.Deserialize<UiConfig>(json, _jsonOptions);

            if (config != null)
            {
                _config = config;
                _logger.LogInformation("UI config loaded successfully");
                ConfigChanged?.Invoke(this, _config);
                return true;
            }
            else
            {
                _logger.LogWarning("Failed to deserialize config, using defaults");
                _config = new UiConfig();
                return false;
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error loading config from {Path}", fullPath);
            _config = new UiConfig();
            return false;
        }
    }

    public bool Save(string? filePath = null)
    {
        string path = filePath ?? _defaultPath;
        string fullPath = Path.GetFullPath(path);

        try
        {
            _logger.LogInformation("Saving UI config to: {Path}", fullPath);

            // Ensure directory exists
            string? dir = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
            {
                Directory.CreateDirectory(dir);
            }

            string json = JsonSerializer.Serialize(_config, _jsonOptions);
            File.WriteAllText(fullPath, json);

            _logger.LogInformation("UI config saved successfully");
            return true;
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error saving config to {Path}", fullPath);
            return false;
        }
    }

    public void Reset()
    {
        _logger.LogInformation("Resetting UI config to defaults");
        _config = new UiConfig();
        ConfigChanged?.Invoke(this, _config);
    }
}
