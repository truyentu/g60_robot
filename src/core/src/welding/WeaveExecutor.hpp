#pragma once

#include "WeavePatternGenerator.hpp"
#include <atomic>
#include <mutex>

namespace robotics {
namespace welding {

// ============================================================================
// Weave Executor
// ============================================================================

/**
 * Executes weave patterns in real-time, overlaying onto robot trajectory
 */
class WeaveExecutor {
public:
    WeaveExecutor();
    ~WeaveExecutor() = default;

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * Set weave parameters
     */
    void setParams(const WeaveParams& params);
    const WeaveParams& getParams() const { return params_; }

    /**
     * Enable/disable weaving
     */
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

    // ========================================================================
    // Execution
    // ========================================================================

    /**
     * Start weaving
     */
    void start();

    /**
     * Stop weaving
     */
    void stop();

    /**
     * Reset phase to beginning
     */
    void reset();

    /**
     * Update weave state
     * @param deltaTime Time since last update (seconds)
     * @param travelSpeed Current travel speed (mm/s)
     */
    void update(double deltaTime, double travelSpeed);

    /**
     * Get current weave offset
     * @return Current weave point
     */
    WeavePoint getCurrentOffset() const;

    /**
     * Apply weave to a trajectory point
     * @param position Original position
     * @param frame Weave coordinate frame
     * @return Modified position with weave applied
     */
    Vector3d applyWeave(
        const Vector3d& position,
        const WeaveFrame& frame) const;

    // ========================================================================
    // Status
    // ========================================================================

    bool isActive() const { return active_; }
    double getCurrentPhase() const { return phase_; }
    int getCycleCount() const { return cycleCount_; }
    double getTotalDistance() const { return totalDistance_; }

    // ========================================================================
    // Real-time Adjustment
    // ========================================================================

    /**
     * Adjust amplitude during welding
     */
    void adjustAmplitude(double amplitude);

    /**
     * Adjust frequency/wavelength during welding
     */
    void adjustFrequency(double frequency);

    /**
     * Adjust dwell times during welding
     */
    void adjustDwell(double leftMs, double rightMs);

private:
    WeaveParams params_;
    std::unique_ptr<IWeavePatternGenerator> generator_;

    std::atomic<bool> enabled_;
    std::atomic<bool> active_;

    double phase_;              // Current phase (0-1)
    double distance_;           // Distance traveled in current cycle
    int cycleCount_;            // Number of completed cycles
    double totalDistance_;      // Total distance with weave

    WeavePoint currentOffset_;

    mutable std::mutex mutex_;

    void updatePhase(double deltaDistance);
    void generateCurrentOffset();
};

// ============================================================================
// Implementation
// ============================================================================

inline WeaveExecutor::WeaveExecutor()
    : enabled_(false),
      active_(false),
      phase_(0),
      distance_(0),
      cycleCount_(0),
      totalDistance_(0) {

    generator_ = createPatternGenerator(WeavePatternType::SINUSOIDAL);
}

inline void WeaveExecutor::setParams(const WeaveParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
    generator_ = createPatternGenerator(params.type);
}

inline void WeaveExecutor::start() {
    active_ = true;
    phase_ = params_.phaseOffset / 360.0;  // Convert degrees to 0-1
    distance_ = 0;
    cycleCount_ = 0;
}

inline void WeaveExecutor::stop() {
    active_ = false;
}

inline void WeaveExecutor::reset() {
    phase_ = params_.phaseOffset / 360.0;
    distance_ = 0;
    cycleCount_ = 0;
    totalDistance_ = 0;
    currentOffset_ = WeavePoint();
}

inline void WeaveExecutor::update(double deltaTime, double travelSpeed) {
    if (!active_ || !enabled_) {
        currentOffset_ = WeavePoint();
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // Calculate distance traveled
    double deltaDistance = travelSpeed * deltaTime;
    totalDistance_ += deltaDistance;

    // Update phase based on travel distance
    updatePhase(deltaDistance);

    // Generate current offset
    generateCurrentOffset();
}

inline void WeaveExecutor::updatePhase(double deltaDistance) {
    double wavelength = params_.getWavelength(1.0);  // Speed already in deltaDistance

    if (wavelength <= 0) wavelength = 10.0;  // Default

    distance_ += deltaDistance;

    // Check for dwell at current position
    if (currentOffset_.isEdge && currentOffset_.dwellTime > 0) {
        // Implement dwell by pausing phase advancement
        // (simplified - real implementation would track dwell time)
    }

    // Advance phase
    double phaseAdvance = deltaDistance / wavelength;
    phase_ += phaseAdvance;

    // Check for cycle completion
    if (phase_ >= 1.0) {
        phase_ -= 1.0;
        cycleCount_++;
        distance_ = 0;
    }
}

inline void WeaveExecutor::generateCurrentOffset() {
    if (generator_) {
        currentOffset_ = generator_->generate(phase_, params_);
    }
}

inline WeavePoint WeaveExecutor::getCurrentOffset() const {
    if (!active_ || !enabled_) {
        return WeavePoint();
    }
    return currentOffset_;
}

inline Vector3d WeaveExecutor::applyWeave(
    const Vector3d& position,
    const WeaveFrame& frame) const {

    if (!active_ || !enabled_) {
        return position;
    }

    WeavePoint offset = getCurrentOffset();

    // Apply lateral offset
    Vector3d result = position + frame.lateral * offset.lateral;

    // Apply longitudinal offset (if any)
    result += frame.tangent * offset.longitudinal;

    // Apply vertical offset
    result += frame.normal * (offset.vertical + params_.heightOffset);

    // Apply lateral offset from centerline
    result += frame.lateral * params_.lateralOffset;

    return result;
}

inline void WeaveExecutor::adjustAmplitude(double amplitude) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_.amplitude = amplitude;
}

inline void WeaveExecutor::adjustFrequency(double frequency) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_.frequency = frequency;
    params_.useFrequency = true;
}

inline void WeaveExecutor::adjustDwell(double leftMs, double rightMs) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_.dwellLeft = leftMs;
    params_.dwellRight = rightMs;
}

} // namespace welding
} // namespace robotics
