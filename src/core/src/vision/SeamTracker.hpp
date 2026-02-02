#pragma once

#include "SeamTypes.hpp"
#include "JointDetector.hpp"
#include <array>
#include <chrono>

namespace robot_controller::vision {

/// Simple Kalman filter for 2D seam tracking (X, Z position)
class SeamKalmanFilter {
public:
    SeamKalmanFilter();

    /// Reset filter state
    void reset();

    /// Initialize with first measurement
    void initialize(float x, float z);

    /// Predict next state
    void predict(float dt);

    /// Update with measurement
    void update(float measuredX, float measuredZ);

    /// Get current state estimate
    void getState(float& x, float& z, float& vx, float& vz) const;

    /// Get prediction for future time
    void getPrediction(float dt, float& x, float& z) const;

    /// Check if filter is initialized
    bool isInitialized() const { return initialized_; }

private:
    // State: [x, z, vx, vz]
    std::array<float, 4> state_;

    // Covariance matrix (4x4, stored as flat array)
    std::array<float, 16> P_;

    // Process noise
    float processNoise_{1.0f};

    // Measurement noise
    float measurementNoise_{0.5f};

    bool initialized_{false};
};

/// Real-time seam tracker with latency compensation
class SeamTracker {
public:
    SeamTracker();
    ~SeamTracker() = default;

    /// Set tracking configuration
    void setConfig(const SeamDetectionConfig& config);

    /// Process new profile and update tracking
    TrackingState update(const LaserProfile& profile, const RobotPose& robotPose);

    /// Get current tracking state
    TrackingState getState() const { return state_; }

    /// Reset tracker
    void reset();

    /// Enable/disable tracking
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

    /// Set nominal path for deviation calculation
    void setNominalPath(const SeamPath& path);

    /// Get predicted seam point with latency compensation
    SeamPoint getPredictedPoint(float lookaheadTime) const;

    /// Register callbacks
    void setFeatureCallback(SeamFeatureCallback callback);
    void setTrackingCallback(TrackingCallback callback);

private:
    SeamDetectionConfig config_;
    JointDetector jointDetector_;
    SeamKalmanFilter kalmanFilter_;
    TrackingState state_;

    bool enabled_{false};
    SeamPath nominalPath_;
    size_t nominalPathIndex_{0};

    // Timing
    std::chrono::steady_clock::time_point lastUpdateTime_;
    std::chrono::steady_clock::time_point startTime_;

    // Callbacks
    SeamFeatureCallback featureCallback_;
    TrackingCallback trackingCallback_;

    // Statistics
    float runningLatency_{0.0f};
    int latencySamples_{0};

    // Helpers
    SeamPoint transformToWorld(const SeamFeature& feature, const RobotPose& pose);
    float computeDeviation(const SeamPoint& detected);
    void updateStatistics(float latency, bool validDetection);
};

} // namespace robot_controller::vision
