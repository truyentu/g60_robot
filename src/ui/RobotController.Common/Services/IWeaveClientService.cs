using RobotController.Common.Messages;

namespace RobotController.Common.Services;

/// <summary>
/// Client service for weave pattern control
/// </summary>
public interface IWeaveClientService
{
    // Control
    Task<WeaveControlResponse> SetParamsAsync(WeaveParamsData @params, CancellationToken ct = default);
    Task<WeaveControlResponse> StartAsync(CancellationToken ct = default);
    Task<WeaveControlResponse> StopAsync(CancellationToken ct = default);
    Task<WeaveControlResponse> EnableAsync(bool enabled, CancellationToken ct = default);

    // Real-time adjustment
    Task<WeaveControlResponse> AdjustAmplitudeAsync(double amplitude, CancellationToken ct = default);
    Task<WeaveControlResponse> AdjustFrequencyAsync(double frequency, CancellationToken ct = default);
    Task<WeaveControlResponse> AdjustDwellAsync(double leftMs, double rightMs, CancellationToken ct = default);

    // Status
    Task<WeaveStatusResponse> GetStatusAsync(CancellationToken ct = default);
    WeaveStatusResponse CachedStatus { get; }

    // Preview
    Task<WeavePreviewResponse> GetPreviewAsync(WeavePreviewRequest request, CancellationToken ct = default);

    // Presets
    WeaveParamsData GetPreset(string presetName);
    IEnumerable<string> GetPresetNames();

    // Events
    event EventHandler<WeaveStatusResponse>? StatusUpdated;
}
