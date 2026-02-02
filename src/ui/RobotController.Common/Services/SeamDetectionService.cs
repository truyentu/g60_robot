using System;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;

namespace RobotController.Common.Services;

/// <summary>
/// Implementation of seam detection service using IPC to C++ core
/// </summary>
public class SeamDetectionService : ISeamDetectionService
{
    private readonly IIpcClientService _ipc;
    private readonly ILogger<SeamDetectionService>? _logger;

    private bool _isTrackingActive;
    private float _trackingQuality;
    private TrackingStateData? _lastTrackingState;
    private SeamFeatureData? _lastFeature;

    public event EventHandler<SeamFeatureEventArgs>? FeatureDetected;
    public event EventHandler<TrackingStateEventArgs>? TrackingStateChanged;

    public bool IsTrackingActive => _isTrackingActive;
    public float TrackingQuality => _trackingQuality;

    public SeamDetectionService(IIpcClientService ipc, ILogger<SeamDetectionService>? logger = null)
    {
        _ipc = ipc ?? throw new ArgumentNullException(nameof(ipc));
        _logger = logger;
    }

    public async Task<bool> StartTrackingAsync(JointType jointType, SeamRoiConfig? roi = null)
    {
        try
        {
            roi ??= new SeamRoiConfig();

            var request = new
            {
                jointType = jointType.ToString().ToLowerInvariant(),
                roiXMin = roi.XMin,
                roiXMax = roi.XMax,
                roiZMin = roi.ZMin,
                roiZMax = roi.ZMax
            };

            var success = await _ipc.SendCommandAsync("seam.startTracking", request);

            if (success)
            {
                _isTrackingActive = true;
                _logger?.LogInformation("Seam tracking started for joint type: {JointType}", jointType);
            }

            return success;
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Exception starting seam tracking");
            return false;
        }
    }

    public async Task<bool> StopTrackingAsync()
    {
        try
        {
            var success = await _ipc.SendCommandAsync("seam.stopTracking");

            if (success)
            {
                _isTrackingActive = false;
                _trackingQuality = 0;
                _logger?.LogInformation("Seam tracking stopped");
            }

            return success;
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Exception stopping seam tracking");
            return false;
        }
    }

    public async Task<bool> SetTrackingConfigAsync(TrackingConfig config)
    {
        try
        {
            var request = new
            {
                enableMedianFilter = config.EnableMedianFilter,
                medianFilterSize = config.MedianFilterSize,
                enableOutlierRemoval = config.EnableOutlierRemoval,
                outlierThreshold = config.OutlierThreshold,
                ransacIterations = config.RansacIterations,
                ransacThreshold = config.RansacThreshold,
                systemLatency = config.SystemLatency,
                travelSpeed = config.TravelSpeed
            };

            return await _ipc.SendCommandAsync("seam.setConfig", request);
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Exception setting tracking config");
            return false;
        }
    }

    public Task<TrackingStateData> GetTrackingStateAsync()
    {
        // Return cached state - real implementation would query C++ core
        // via a separate subscription or polling mechanism
        var state = _lastTrackingState ?? new TrackingStateData
        {
            Success = _isTrackingActive,
            TrackingActive = _isTrackingActive,
            TrackingLost = false,
            TrackingQuality = _trackingQuality
        };

        return Task.FromResult(state);
    }

    public Task<SeamFeatureData> GetLatestFeatureAsync()
    {
        // Return cached feature - real implementation would query C++ core
        var feature = _lastFeature ?? new SeamFeatureData
        {
            Success = false,
            ErrorMessage = "No feature available"
        };

        return Task.FromResult(feature);
    }

    public Task<SeamPointData> GetCurrentSeamPointAsync()
    {
        // Return placeholder - real implementation would query C++ core
        return Task.FromResult(new SeamPointData
        {
            Success = false,
            ErrorMessage = "Not implemented - requires subscription"
        });
    }

    public async Task<bool> SetNominalPathAsync(float[] pointsX, float[] pointsY, float[] pointsZ)
    {
        try
        {
            if (pointsX.Length != pointsY.Length || pointsY.Length != pointsZ.Length)
            {
                _logger?.LogError("Path arrays must have equal lengths");
                return false;
            }

            var request = new
            {
                pointsX,
                pointsY,
                pointsZ
            };

            return await _ipc.SendCommandAsync("seam.setNominalPath", request);
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Exception setting nominal path");
            return false;
        }
    }

    public Task<SeamPathData> GetSeamPathAsync()
    {
        // Return placeholder - real implementation would query C++ core
        return Task.FromResult(new SeamPathData
        {
            Success = false,
            ErrorMessage = "Not implemented - requires subscription"
        });
    }

    public async Task<bool> ResetTrackerAsync()
    {
        try
        {
            var success = await _ipc.SendCommandAsync("seam.reset");

            if (success)
            {
                _isTrackingActive = false;
                _trackingQuality = 0;
                _lastTrackingState = null;
                _lastFeature = null;
                _logger?.LogInformation("Seam tracker reset");
            }

            return success;
        }
        catch (Exception ex)
        {
            _logger?.LogError(ex, "Exception resetting tracker");
            return false;
        }
    }

    /// <summary>
    /// Update internal state from subscription data (called by IPC layer)
    /// </summary>
    public void UpdateTrackingState(TrackingStateData state)
    {
        _lastTrackingState = state;
        _isTrackingActive = state.TrackingActive;
        _trackingQuality = state.TrackingQuality;
        TrackingStateChanged?.Invoke(this, new TrackingStateEventArgs(state));
    }

    /// <summary>
    /// Update internal feature from subscription data (called by IPC layer)
    /// </summary>
    public void UpdateFeature(SeamFeatureData feature)
    {
        _lastFeature = feature;
        FeatureDetected?.Invoke(this, new SeamFeatureEventArgs(feature));
    }
}
