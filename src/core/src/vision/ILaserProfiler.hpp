#pragma once

#include "VisionTypes.hpp"
#include <functional>
#include <memory>

namespace robot_controller::vision {

/// Abstract interface for laser profiler sensors
class ILaserProfiler {
public:
    virtual ~ILaserProfiler() = default;

    // ========================================================================
    // Connection
    // ========================================================================

    /// Enumerate available devices
    virtual std::vector<SensorInfo> enumerate() = 0;

    /// Connect to device
    virtual bool connect(const std::string& deviceId) = 0;

    /// Disconnect
    virtual void disconnect() = 0;

    /// Check if connected
    virtual bool isConnected() const = 0;

    /// Get sensor info
    virtual SensorInfo getInfo() const = 0;

    // ========================================================================
    // Configuration
    // ========================================================================

    /// Apply configuration
    virtual bool configure(const LaserProfilerConfig& config) = 0;

    /// Get current configuration
    virtual LaserProfilerConfig getConfig() const = 0;

    /// Set exposure time (microseconds)
    virtual bool setExposure(float exposureUs) = 0;

    /// Set gain
    virtual bool setGain(float gain) = 0;

    /// Set trigger mode
    virtual bool setTriggerMode(LaserProfilerConfig::TriggerMode mode) = 0;

    /// Set encoder divider (for hardware trigger mode)
    virtual bool setEncoderDivider(uint32_t divider) = 0;

    // ========================================================================
    // Acquisition
    // ========================================================================

    /// Start continuous acquisition
    virtual bool startAcquisition() = 0;

    /// Stop acquisition
    virtual bool stopAcquisition() = 0;

    /// Check if acquiring
    virtual bool isAcquiring() const = 0;

    /// Software trigger (for software trigger mode)
    virtual bool softwareTrigger() = 0;

    /// Get single profile (blocking)
    virtual bool getProfile(LaserProfile& profile, uint32_t timeoutMs = 1000) = 0;

    /// Get batch of profiles
    virtual bool getProfileBatch(ProfileBatch& batch,
                                  uint32_t count,
                                  uint32_t timeoutMs = 5000) = 0;

    // ========================================================================
    // Callbacks
    // ========================================================================

    /// Register profile callback (called from acquisition thread)
    virtual void setProfileCallback(ProfileCallback callback) = 0;

    /// Clear callback
    virtual void clearProfileCallback() = 0;

    // ========================================================================
    // Calibration
    // ========================================================================

    /// Load sensor calibration file
    virtual bool loadCalibration(const std::string& filepath) = 0;

    /// Set hand-eye calibration
    virtual void setHandEyeCalibration(const HandEyeCalibration& cal) = 0;

    /// Get hand-eye calibration
    virtual HandEyeCalibration getHandEyeCalibration() const = 0;

    // ========================================================================
    // Utilities
    // ========================================================================

    /// Convert profile to 3D points using robot pose
    virtual std::vector<Point3D> profileToWorldPoints(
        const LaserProfile& profile,
        const RobotPose& robotPose) const = 0;

    /// Reset statistics
    virtual void resetStats() = 0;
};

/// Factory function
std::unique_ptr<ILaserProfiler> createLaserProfiler(const std::string& type = "hikrobot");

} // namespace robot_controller::vision
