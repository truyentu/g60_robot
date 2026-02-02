#include "SeamTracker.hpp"
#include <spdlog/spdlog.h>
#include <cmath>

namespace robot_controller::vision {

// ============================================================================
// SeamKalmanFilter Implementation
// ============================================================================

SeamKalmanFilter::SeamKalmanFilter() {
    reset();
}

void SeamKalmanFilter::reset() {
    state_ = {0, 0, 0, 0};

    // Initialize P as identity
    P_ = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };

    initialized_ = false;
}

void SeamKalmanFilter::initialize(float x, float z) {
    state_ = {x, z, 0, 0};

    // Reset covariance
    P_ = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };

    initialized_ = true;
}

void SeamKalmanFilter::predict(float dt) {
    if (!initialized_) return;

    // State transition: x += vx * dt, z += vz * dt
    state_[0] += state_[2] * dt;
    state_[1] += state_[3] * dt;

    // Add process noise to covariance
    float q = processNoise_ * dt * dt;
    P_[0] += q;
    P_[5] += q;
    P_[10] += q;
    P_[15] += q;
}

void SeamKalmanFilter::update(float measuredX, float measuredZ) {
    if (!initialized_) {
        initialize(measuredX, measuredZ);
        return;
    }

    // Simple Kalman gain calculation
    float r = measurementNoise_;

    // Innovation
    float dx = measuredX - state_[0];
    float dz = measuredZ - state_[1];

    // Kalman gain (simplified for diagonal R)
    float k0 = P_[0] / (P_[0] + r);
    float k1 = P_[5] / (P_[5] + r);

    // Update state
    state_[0] += k0 * dx;
    state_[1] += k1 * dz;

    // Update velocity estimate (simple finite difference)
    float alpha = 0.3f;  // Smoothing factor
    state_[2] = alpha * dx + (1 - alpha) * state_[2];
    state_[3] = alpha * dz + (1 - alpha) * state_[3];

    // Update covariance
    P_[0] *= (1 - k0);
    P_[5] *= (1 - k1);
}

void SeamKalmanFilter::getState(float& x, float& z, float& vx, float& vz) const {
    x = state_[0];
    z = state_[1];
    vx = state_[2];
    vz = state_[3];
}

void SeamKalmanFilter::getPrediction(float dt, float& x, float& z) const {
    x = state_[0] + state_[2] * dt;
    z = state_[1] + state_[3] * dt;
}

// ============================================================================
// SeamTracker Implementation
// ============================================================================

SeamTracker::SeamTracker() {
    reset();
}

void SeamTracker::setConfig(const SeamDetectionConfig& config) {
    config_ = config;
    jointDetector_.setConfig(config);
}

void SeamTracker::reset() {
    kalmanFilter_.reset();
    state_ = TrackingState{};
    nominalPathIndex_ = 0;
    runningLatency_ = 0;
    latencySamples_ = 0;
    startTime_ = std::chrono::steady_clock::now();
    lastUpdateTime_ = startTime_;
}

TrackingState SeamTracker::update(const LaserProfile& profile, const RobotPose& robotPose) {
    auto startProcess = std::chrono::steady_clock::now();

    // Compute dt
    float dt = std::chrono::duration<float>(startProcess - lastUpdateTime_).count();
    lastUpdateTime_ = startProcess;

    // Predict next state
    kalmanFilter_.predict(dt);

    // Detect seam feature
    SeamFeature feature = jointDetector_.detect(profile);
    state_.currentFeature = feature;

    if (feature.valid) {
        // Update Kalman filter with measurement
        kalmanFilter_.update(feature.rootPoint.x, feature.rootPoint.z);

        // Transform to world coordinates
        state_.currentSeamPoint = transformToWorld(feature, robotPose);

        // Compute deviation from nominal
        if (!nominalPath_.empty()) {
            state_.currentSeamPoint.deviation = computeDeviation(state_.currentSeamPoint);
        }

        // Update tracking quality
        state_.detectionsValid++;
        state_.trackingLost = false;
        state_.lostFrameCount = 0;
        state_.trackingQuality = 0.9f * state_.trackingQuality + 0.1f * feature.confidence;

        // Compute correction output
        if (config_.enableTracking) {
            // Get predicted position with latency compensation
            float predictedX, predictedZ;
            kalmanFilter_.getPrediction(config_.systemLatency, predictedX, predictedZ);

            state_.correction.lateralOffset = predictedX - feature.rootPoint.x;
            state_.correction.heightOffset = predictedZ - feature.rootPoint.z;
            state_.correction.valid = true;
        }

        // Invoke callback
        if (featureCallback_) {
            featureCallback_(feature);
        }
    } else {
        // No valid detection
        state_.lostFrameCount++;

        if (dt > config_.trackingTimeout) {
            state_.trackingLost = true;
            state_.trackingQuality *= 0.9f;
        }

        state_.correction.valid = false;
    }

    // Update predicted state
    kalmanFilter_.getState(
        state_.predicted.x,
        state_.predicted.z,
        state_.predicted.vx,
        state_.predicted.vz);
    state_.predicted.confidence = state_.trackingQuality;

    state_.framesProcessed++;

    // Compute processing latency
    auto endProcess = std::chrono::steady_clock::now();
    float latency = std::chrono::duration<float, std::milli>(endProcess - startProcess).count();
    updateStatistics(latency, feature.valid);

    // Invoke tracking callback
    if (trackingCallback_) {
        trackingCallback_(state_);
    }

    return state_;
}

void SeamTracker::setNominalPath(const SeamPath& path) {
    nominalPath_ = path;
    nominalPathIndex_ = 0;
}

SeamPoint SeamTracker::getPredictedPoint(float lookaheadTime) const {
    SeamPoint predicted = state_.currentSeamPoint;

    if (kalmanFilter_.isInitialized()) {
        float x, z;
        kalmanFilter_.getPrediction(lookaheadTime, x, z);

        // Apply prediction to position
        predicted.position.x += x - state_.predicted.x;
        predicted.position.z += z - state_.predicted.z;
    }

    return predicted;
}

void SeamTracker::setFeatureCallback(SeamFeatureCallback callback) {
    featureCallback_ = std::move(callback);
}

void SeamTracker::setTrackingCallback(TrackingCallback callback) {
    trackingCallback_ = std::move(callback);
}

SeamPoint SeamTracker::transformToWorld(const SeamFeature& feature, const RobotPose& pose) {
    SeamPoint seamPoint;
    seamPoint.feature = feature;
    seamPoint.robotPose = pose;
    seamPoint.timestamp = feature.timestamp;

    if (!pose.valid) {
        // No valid pose, return sensor frame coordinates
        seamPoint.position.x = feature.rootPoint.x;
        seamPoint.position.y = 0;
        seamPoint.position.z = feature.rootPoint.z;
        seamPoint.valid = feature.valid;
        return seamPoint;
    }

    // Transform using robot flange matrix
    // P_world = T_base_flange * T_flange_sensor * P_sensor

    double x = feature.rootPoint.x;
    double y = 0;  // Profile is in X-Z plane
    double z = feature.rootPoint.z;

    const auto& T = pose.flange;  // 4x4 row-major

    seamPoint.position.x = static_cast<float>(T[0]*x + T[1]*y + T[2]*z + T[3]);
    seamPoint.position.y = static_cast<float>(T[4]*x + T[5]*y + T[6]*z + T[7]);
    seamPoint.position.z = static_cast<float>(T[8]*x + T[9]*y + T[10]*z + T[11]);

    // Compute surface normal (pointing up in sensor frame = [0, 1, 0])
    seamPoint.normal = {T[4], T[5], T[6]};

    // Tangent (along X in sensor frame)
    seamPoint.tangent = {T[0], T[1], T[2]};

    // Binormal (cross product)
    seamPoint.binormal = {
        seamPoint.normal[1] * seamPoint.tangent[2] - seamPoint.normal[2] * seamPoint.tangent[1],
        seamPoint.normal[2] * seamPoint.tangent[0] - seamPoint.normal[0] * seamPoint.tangent[2],
        seamPoint.normal[0] * seamPoint.tangent[1] - seamPoint.normal[1] * seamPoint.tangent[0]
    };

    // Torch position (same as seam point for now, offset would be applied)
    seamPoint.torchPosition = {
        seamPoint.position.x,
        seamPoint.position.y,
        seamPoint.position.z
    };

    seamPoint.valid = feature.valid;
    return seamPoint;
}

float SeamTracker::computeDeviation(const SeamPoint& detected) {
    if (nominalPath_.empty()) return 0.0f;

    // Find closest point on nominal path
    float minDist = std::numeric_limits<float>::max();
    size_t closestIdx = nominalPathIndex_;

    // Search around current index
    size_t searchStart = (nominalPathIndex_ > 10) ? nominalPathIndex_ - 10 : 0;
    size_t searchEnd = std::min(nominalPathIndex_ + 50, nominalPath_.size());

    for (size_t i = searchStart; i < searchEnd; ++i) {
        float dx = detected.position.x - nominalPath_.points[i].position.x;
        float dy = detected.position.y - nominalPath_.points[i].position.y;
        float dz = detected.position.z - nominalPath_.points[i].position.z;
        float dist = dx*dx + dy*dy + dz*dz;

        if (dist < minDist) {
            minDist = dist;
            closestIdx = i;
        }
    }

    nominalPathIndex_ = closestIdx;

    // Return lateral deviation (perpendicular to path)
    // Simplified: just return X deviation
    return detected.position.x - nominalPath_.points[closestIdx].position.x;
}

void SeamTracker::updateStatistics(float latency, bool validDetection) {
    // Running average of latency
    latencySamples_++;
    runningLatency_ = runningLatency_ + (latency - runningLatency_) / latencySamples_;
    state_.avgLatency = runningLatency_;
}

} // namespace robot_controller::vision
