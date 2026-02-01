#pragma once

#include "VisionTypes.hpp"
#include "ILaserProfiler.hpp"
#include <memory>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include <tuple>

namespace robot_controller::vision {

/// Callback for robot pose requests (to synchronize with robot position)
using RobotPoseCallback = std::function<RobotPose()>;

/// Manages all vision sensors
class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    // Disable copy
    SensorManager(const SensorManager&) = delete;
    SensorManager& operator=(const SensorManager&) = delete;

    // ========================================================================
    // Initialization
    // ========================================================================

    /// Initialize sensor manager
    bool initialize();

    /// Shutdown all sensors
    void shutdown();

    /// Set callback to get current robot pose
    void setRobotPoseCallback(RobotPoseCallback callback);

    // ========================================================================
    // Laser Profiler
    // ========================================================================

    /// Get laser profiler instance
    ILaserProfiler* getLaserProfiler() { return laserProfiler_.get(); }

    /// Connect laser profiler by device ID or IP
    bool connectLaserProfiler(const std::string& deviceId);

    /// Disconnect laser profiler
    void disconnectLaserProfiler();

    /// Configure laser profiler
    bool configureLaserProfiler(const LaserProfilerConfig& config);

    /// Start laser scanning
    bool startLaserScanning();

    /// Stop laser scanning
    bool stopLaserScanning();

    /// Get single profile with robot pose synchronized
    bool getSynchronizedProfile(LaserProfile& profile, uint32_t timeoutMs = 1000);

    /// Collect profiles for 3D scan (scan-to-path)
    bool collectScanData(ProfileBatch& batch,
                          uint32_t profileCount,
                          uint32_t timeoutMs = 30000);

    // ========================================================================
    // Point Cloud Operations
    // ========================================================================

    /// Convert profile batch to point cloud (in world coordinates)
    PointCloud profilesToPointCloud(const ProfileBatch& batch);

    /// Downsample point cloud with voxel grid
    PointCloud downsamplePointCloud(const PointCloud& cloud, float voxelSize);

    /// Filter point cloud by bounds
    PointCloud filterByBounds(const PointCloud& cloud,
                               const Point3D& minBound,
                               const Point3D& maxBound);

    // ========================================================================
    // Calibration
    // ========================================================================

    /// Perform hand-eye calibration
    bool performHandEyeCalibration(
        const std::vector<RobotPose>& robotPoses,
        const std::vector<std::array<double, 16>>& targetPoses,
        HandEyeCalibration& result);

    /// Save hand-eye calibration
    bool saveHandEyeCalibration(const std::string& filepath);

    /// Load hand-eye calibration
    bool loadHandEyeCalibration(const std::string& filepath);

    // ========================================================================
    // Status
    // ========================================================================

    /// Get all sensor info
    struct SensorManagerStatus {
        bool laserConnected{false};
        bool laserAcquiring{false};
        SensorInfo laserInfo;
        float totalDataRate{0.0f};
    };

    SensorManagerStatus getStatus() const;

private:
    // Sensors
    std::unique_ptr<ILaserProfiler> laserProfiler_;

    // State
    std::atomic<bool> initialized_{false};

    // Robot pose callback
    RobotPoseCallback robotPoseCallback_;
    std::mutex poseCallbackMutex_;

    // Calibration
    HandEyeCalibration handEyeCalibration_;

    // Helpers
    RobotPose getCurrentRobotPose();
};

} // namespace robot_controller::vision
