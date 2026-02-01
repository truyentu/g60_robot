#pragma once

#include "ILaserProfiler.hpp"
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>

namespace robot_controller::vision {

/// Hikrobot MVS SDK implementation of laser profiler
class HikrobotLaserProfiler : public ILaserProfiler {
public:
    HikrobotLaserProfiler();
    ~HikrobotLaserProfiler() override;

    // Disable copy
    HikrobotLaserProfiler(const HikrobotLaserProfiler&) = delete;
    HikrobotLaserProfiler& operator=(const HikrobotLaserProfiler&) = delete;

    // ========================================================================
    // ILaserProfiler Implementation
    // ========================================================================

    std::vector<SensorInfo> enumerate() override;
    bool connect(const std::string& deviceId) override;
    void disconnect() override;
    bool isConnected() const override;
    SensorInfo getInfo() const override;

    bool configure(const LaserProfilerConfig& config) override;
    LaserProfilerConfig getConfig() const override;
    bool setExposure(float exposureUs) override;
    bool setGain(float gain) override;
    bool setTriggerMode(LaserProfilerConfig::TriggerMode mode) override;
    bool setEncoderDivider(uint32_t divider) override;

    bool startAcquisition() override;
    bool stopAcquisition() override;
    bool isAcquiring() const override;
    bool softwareTrigger() override;
    bool getProfile(LaserProfile& profile, uint32_t timeoutMs = 1000) override;
    bool getProfileBatch(ProfileBatch& batch,
                          uint32_t count,
                          uint32_t timeoutMs = 5000) override;

    void setProfileCallback(ProfileCallback callback) override;
    void clearProfileCallback() override;

    bool loadCalibration(const std::string& filepath) override;
    void setHandEyeCalibration(const HandEyeCalibration& cal) override;
    HandEyeCalibration getHandEyeCalibration() const override;

    std::vector<Point3D> profileToWorldPoints(
        const LaserProfile& profile,
        const RobotPose& robotPose) const override;

    void resetStats() override;

private:
    // SDK handle
    void* handle_{nullptr};

    // Configuration
    LaserProfilerConfig config_;
    HandEyeCalibration handEyeCal_;
    mutable std::mutex configMutex_;

    // State
    std::atomic<bool> connected_{false};
    std::atomic<bool> acquiring_{false};
    SensorInfo info_;

    // Acquisition thread
    std::thread acquisitionThread_;
    std::atomic<bool> stopFlag_{false};

    // Profile queue (producer-consumer pattern)
    std::queue<LaserProfile> profileQueue_;
    std::mutex queueMutex_;
    std::condition_variable queueCondition_;
    static constexpr size_t MAX_QUEUE_SIZE = 100;

    // Callback
    ProfileCallback callback_;
    std::mutex callbackMutex_;

    // Statistics
    std::atomic<uint64_t> framesReceived_{0};
    std::atomic<uint64_t> framesDropped_{0};
    std::chrono::steady_clock::time_point lastFrameTime_;
    std::atomic<float> currentFrameRate_{0.0f};

    // Private methods
    void acquisitionLoop();
    bool parseProfile(const void* rawData, size_t dataSize, LaserProfile& profile);
    void applyIntensityFilter(LaserProfile& profile);
    void applyCalibration(LaserProfile& profile);
    bool setSDKParameter(const std::string& name, int value);
    bool setSDKParameter(const std::string& name, float value);
    int getSDKParameter(const std::string& name);

    // Coordinate transform helpers
    void transformToWorld(Point3D& point,
                          const std::array<double, 16>& flangeMatrix) const;
};

} // namespace robot_controller::vision
