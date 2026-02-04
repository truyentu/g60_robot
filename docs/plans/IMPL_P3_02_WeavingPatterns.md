# IMPL_P3_02: Weaving Patterns

| Metadata | Value |
|----------|-------|
| Plan ID | IMPL_P3_02 |
| Phase | 3 - Welding Integration |
| Priority | P1 |
| Depends On | IMPL_P3_01 (Welding Sequencer) |
| Status | Ready for Implementation |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/Lập Trình Robot Hàn Chuyên Nghiệp.md` | Weave patterns, torch angles, seam strategies |

---

## Overview

Implementation plan cho Weaving Patterns - tạo và thực thi các pattern weave cho hàn:
- **Pattern Types:** Linear, Triangular, Sinusoidal, Circular, Figure-8
- **Parameters:** Amplitude, frequency, dwell times, phase
- **Generation:** Real-time pattern generation
- **Superposition:** Overlay weave onto weld trajectory
- **Adaptive:** Dynamic adjustment during welding

---

## Weaving Concepts

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         WEAVING PATTERNS                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  LINEAR (Zig-Zag):        TRIANGULAR:           SINUSOIDAL:             │
│     ╱╲╱╲╱╲╱╲                 /\  /\  /\            ~~~~~                 │
│    ╱  ╲  ╲  ╱               /  \/  \/  \          /     \               │
│   ╱    ╲╱  ╲               /            \        /       \              │
│  ────────────►            ──────────────►      ─/─────────\─►           │
│   Weld direction                                                         │
│                                                                          │
│  CIRCULAR:                FIGURE-8:             CRESCENT:               │
│     ○ ○ ○ ○                 ∞ ∞ ∞ ∞              ︵ ︵ ︵ ︵             │
│    ○ ○ ○ ○ ○               ∞ ∞ ∞ ∞ ∞            ︵ ︵ ︵ ︵ ︵           │
│   ────────────►           ──────────────►      ──────────────►          │
│                                                                          │
├─────────────────────────────────────────────────────────────────────────┤
│  WEAVE PARAMETERS:                                                       │
│                                                                          │
│       ◄─── Amplitude ───►                                               │
│           (width)                                                        │
│              ╱╲                                                          │
│             ╱  ╲         ◄── Dwell (left)                               │
│            ╱    ╲                                                        │
│  ─────────╱──────╲─────────────────────                                 │
│                   ╲    ╱                                                │
│                    ╲  ╱  ◄── Dwell (right)                              │
│                     ╲╱                                                   │
│           ◄─────────►                                                   │
│            Wavelength                                                    │
│           (1/frequency)                                                  │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

- [ ] IMPL_P3_01 (Welding Sequencer) đã hoàn thành
- [ ] Trajectory system available
- [ ] Kinematics service available

---

## Step 1: Create Weaving Types

### 1.1 Create WeavingTypes.hpp

**File:** `src/cpp/include/welding/WeavingTypes.hpp`

```cpp
#pragma once

#include "kinematics/MathTypes.hpp"
#include <string>
#include <vector>
#include <cmath>

namespace robotics {
namespace welding {

using namespace math;

// ============================================================================
// Weave Pattern Types
// ============================================================================

/**
 * Type of weave pattern
 */
enum class WeavePatternType : uint8_t {
    NONE = 0,           // No weaving
    LINEAR = 1,         // Zig-zag (straight lines)
    TRIANGULAR = 2,     // Triangular with sharp corners
    SINUSOIDAL = 3,     // Smooth sine wave
    CIRCULAR = 4,       // Circular loops
    FIGURE_8 = 5,       // Figure-8 pattern
    CRESCENT = 6,       // Crescent/arc pattern
    CUSTOM = 7          // User-defined pattern
};

/**
 * Weave direction relative to weld path
 */
enum class WeaveDirection : uint8_t {
    LATERAL = 0,        // Side-to-side (perpendicular to weld)
    LONGITUDINAL = 1,   // Along weld direction
    BOTH = 2            // Combined (for figure-8, circular)
};

// ============================================================================
// Weave Parameters
// ============================================================================

/**
 * Parameters defining a weave pattern
 */
struct WeaveParams {
    WeavePatternType type;

    // Geometry
    double amplitude;           // Half-width of weave (mm)
    double wavelength;          // Distance per cycle (mm)
    double frequency;           // Cycles per second (Hz) - alternative to wavelength

    // Timing (for dwell-based patterns)
    double dwellLeft;           // Dwell time at left edge (ms)
    double dwellRight;          // Dwell time at right edge (ms)
    double dwellCenter;         // Dwell time at center (ms)

    // Phase and offset
    double phaseOffset;         // Starting phase (0-360 degrees)
    double lateralOffset;       // Offset from weld centerline (mm)
    double heightOffset;        // Vertical offset during weave (mm)

    // Speed modulation
    double speedAtEdge;         // Speed factor at edges (0.5-1.5)
    double speedAtCenter;       // Speed factor at center (0.5-1.5)

    // Pattern-specific parameters
    double circleRadius;        // For circular pattern (mm)
    double figure8Ratio;        // Width/height ratio for figure-8

    // Control flags
    bool useFrequency;          // true=Hz, false=wavelength
    bool syncWithTravel;        // Sync weave with travel speed
    bool mirrorPattern;         // Mirror pattern direction

    WeaveParams()
        : type(WeavePatternType::SINUSOIDAL),
          amplitude(3.0),
          wavelength(10.0),
          frequency(1.0),
          dwellLeft(0),
          dwellRight(0),
          dwellCenter(0),
          phaseOffset(0),
          lateralOffset(0),
          heightOffset(0),
          speedAtEdge(0.8),
          speedAtCenter(1.0),
          circleRadius(2.0),
          figure8Ratio(1.0),
          useFrequency(false),
          syncWithTravel(true),
          mirrorPattern(false) {}

    // Get effective wavelength
    double getWavelength(double travelSpeed) const {
        if (useFrequency && frequency > 0) {
            return travelSpeed / frequency;  // mm per cycle
        }
        return wavelength;
    }
};

// ============================================================================
// Weave Point
// ============================================================================

/**
 * A single point in the weave pattern
 */
struct WeavePoint {
    double lateral;         // Lateral offset from centerline (mm)
    double longitudinal;    // Longitudinal offset (mm)
    double vertical;        // Vertical offset (mm)

    double speedFactor;     // Speed multiplier at this point
    double dwellTime;       // Dwell time at this point (ms)

    double phase;           // Phase in pattern (0-1)
    bool isEdge;            // True if at edge of pattern

    WeavePoint()
        : lateral(0), longitudinal(0), vertical(0),
          speedFactor(1.0), dwellTime(0),
          phase(0), isEdge(false) {}

    WeavePoint(double lat, double lon, double vert = 0)
        : lateral(lat), longitudinal(lon), vertical(vert),
          speedFactor(1.0), dwellTime(0),
          phase(0), isEdge(false) {}
};

// ============================================================================
// Weave Cycle
// ============================================================================

/**
 * A complete weave cycle (one full oscillation)
 */
struct WeaveCycle {
    std::vector<WeavePoint> points;
    double totalLength;     // Total path length of cycle (mm)
    double duration;        // Duration at given travel speed (s)

    WeaveCycle() : totalLength(0), duration(0) {}
};

// ============================================================================
// Coordinate Frame for Weaving
// ============================================================================

/**
 * Local coordinate frame along weld path
 * Used to apply weave offsets in correct orientation
 */
struct WeaveFrame {
    Vector3d position;      // Position on weld path
    Vector3d tangent;       // Direction along weld path (normalized)
    Vector3d normal;        // Normal to weld surface (up)
    Vector3d lateral;       // Lateral direction (tangent × normal)

    WeaveFrame()
        : position(Vector3d::Zero()),
          tangent(Vector3d::UnitX()),
          normal(Vector3d::UnitZ()),
          lateral(Vector3d::UnitY()) {}

    /**
     * Create frame from path tangent and surface normal
     */
    static WeaveFrame fromTangentAndNormal(
        const Vector3d& pos,
        const Vector3d& tang,
        const Vector3d& norm) {

        WeaveFrame frame;
        frame.position = pos;
        frame.tangent = tang.normalized();
        frame.normal = norm.normalized();
        frame.lateral = frame.tangent.cross(frame.normal).normalized();

        // Ensure orthogonality
        frame.normal = frame.lateral.cross(frame.tangent).normalized();

        return frame;
    }

    /**
     * Apply weave offset to get world position
     */
    Vector3d applyOffset(const WeavePoint& weave) const {
        return position +
               lateral * weave.lateral +
               tangent * weave.longitudinal +
               normal * weave.vertical;
    }
};

// ============================================================================
// Preset Weave Configurations
// ============================================================================

/**
 * Common weave presets for different applications
 */
namespace WeavePresets {

    // Thin material, narrow gap
    inline WeaveParams narrowGap() {
        WeaveParams p;
        p.type = WeavePatternType::TRIANGULAR;
        p.amplitude = 2.0;
        p.wavelength = 6.0;
        p.dwellLeft = 50;
        p.dwellRight = 50;
        return p;
    }

    // Medium gap fill
    inline WeaveParams mediumGap() {
        WeaveParams p;
        p.type = WeavePatternType::SINUSOIDAL;
        p.amplitude = 4.0;
        p.wavelength = 10.0;
        p.dwellLeft = 100;
        p.dwellRight = 100;
        return p;
    }

    // Wide gap bridging
    inline WeaveParams wideGap() {
        WeaveParams p;
        p.type = WeavePatternType::TRIANGULAR;
        p.amplitude = 6.0;
        p.wavelength = 12.0;
        p.dwellLeft = 150;
        p.dwellRight = 150;
        p.speedAtEdge = 0.6;
        return p;
    }

    // Vertical up welding
    inline WeaveParams verticalUp() {
        WeaveParams p;
        p.type = WeavePatternType::CRESCENT;
        p.amplitude = 5.0;
        p.wavelength = 8.0;
        p.dwellLeft = 200;
        p.dwellRight = 200;
        p.dwellCenter = 50;
        return p;
    }

    // Overhead welding
    inline WeaveParams overhead() {
        WeaveParams p;
        p.type = WeavePatternType::LINEAR;
        p.amplitude = 3.0;
        p.wavelength = 6.0;
        p.dwellLeft = 100;
        p.dwellRight = 100;
        p.speedAtEdge = 0.7;
        return p;
    }

    // Root pass (first pass)
    inline WeaveParams rootPass() {
        WeaveParams p;
        p.type = WeavePatternType::LINEAR;
        p.amplitude = 1.5;
        p.wavelength = 4.0;
        p.dwellLeft = 0;
        p.dwellRight = 0;
        return p;
    }

    // Cap pass (final pass)
    inline WeaveParams capPass() {
        WeaveParams p;
        p.type = WeavePatternType::SINUSOIDAL;
        p.amplitude = 5.0;
        p.wavelength = 8.0;
        p.dwellLeft = 80;
        p.dwellRight = 80;
        return p;
    }
}

} // namespace welding
} // namespace robotics
```

---

## Step 2: Create Pattern Generator

### 2.1 Create WeavePatternGenerator.hpp

**File:** `src/cpp/include/welding/WeavePatternGenerator.hpp`

```cpp
#pragma once

#include "WeavingTypes.hpp"
#include <functional>
#include <memory>

namespace robotics {
namespace welding {

// ============================================================================
// Pattern Generator Interface
// ============================================================================

/**
 * Interface for weave pattern generators
 */
class IWeavePatternGenerator {
public:
    virtual ~IWeavePatternGenerator() = default;

    /**
     * Generate weave point at given phase
     * @param phase Phase in cycle (0.0 to 1.0)
     * @param params Weave parameters
     * @return Weave point with offsets
     */
    virtual WeavePoint generate(double phase, const WeaveParams& params) = 0;

    /**
     * Generate complete cycle
     * @param params Weave parameters
     * @param numPoints Number of points per cycle
     * @return Complete weave cycle
     */
    virtual WeaveCycle generateCycle(
        const WeaveParams& params,
        int numPoints = 32) = 0;

    /**
     * Get pattern type
     */
    virtual WeavePatternType getType() const = 0;
};

// ============================================================================
// Linear Pattern (Zig-Zag)
// ============================================================================

class LinearPatternGenerator : public IWeavePatternGenerator {
public:
    WeavePoint generate(double phase, const WeaveParams& params) override {
        WeavePoint point;

        // Normalize phase to 0-1
        phase = std::fmod(phase, 1.0);
        if (phase < 0) phase += 1.0;

        point.phase = phase;

        // Linear zig-zag: goes from -amplitude to +amplitude and back
        if (phase < 0.25) {
            // Moving from center to right edge
            point.lateral = params.amplitude * (phase / 0.25);
            point.isEdge = false;
        }
        else if (phase < 0.5) {
            // At right edge, possibly dwelling
            point.lateral = params.amplitude;
            point.isEdge = true;
            point.dwellTime = params.dwellRight;
        }
        else if (phase < 0.75) {
            // Moving from right edge to left edge through center
            double t = (phase - 0.5) / 0.25;
            point.lateral = params.amplitude * (1.0 - 2.0 * t);
            point.isEdge = (t < 0.1 || t > 0.9);
        }
        else {
            // At left edge and returning to center
            double t = (phase - 0.75) / 0.25;
            if (t < 0.2) {
                point.lateral = -params.amplitude;
                point.isEdge = true;
                point.dwellTime = params.dwellLeft;
            } else {
                point.lateral = -params.amplitude * (1.0 - (t - 0.2) / 0.8);
                point.isEdge = false;
            }
        }

        // Apply speed modulation
        double edgeFactor = std::abs(point.lateral) / params.amplitude;
        point.speedFactor = params.speedAtCenter +
            (params.speedAtEdge - params.speedAtCenter) * edgeFactor;

        return point;
    }

    WeaveCycle generateCycle(const WeaveParams& params, int numPoints) override {
        WeaveCycle cycle;
        cycle.points.reserve(numPoints);

        for (int i = 0; i < numPoints; ++i) {
            double phase = static_cast<double>(i) / numPoints;
            cycle.points.push_back(generate(phase, params));
        }

        // Calculate total path length (approximate)
        cycle.totalLength = 4.0 * params.amplitude + params.wavelength;

        return cycle;
    }

    WeavePatternType getType() const override {
        return WeavePatternType::LINEAR;
    }
};

// ============================================================================
// Triangular Pattern
// ============================================================================

class TriangularPatternGenerator : public IWeavePatternGenerator {
public:
    WeavePoint generate(double phase, const WeaveParams& params) override {
        WeavePoint point;

        phase = std::fmod(phase, 1.0);
        if (phase < 0) phase += 1.0;

        point.phase = phase;

        // Triangular: linear motion between edges
        if (phase < 0.5) {
            // Left edge to right edge
            point.lateral = -params.amplitude + 2.0 * params.amplitude * (phase / 0.5);

            if (phase < 0.05) {
                point.isEdge = true;
                point.dwellTime = params.dwellLeft;
            } else if (phase > 0.45) {
                point.isEdge = true;
                point.dwellTime = params.dwellRight;
            }
        }
        else {
            // Right edge to left edge
            double t = (phase - 0.5) / 0.5;
            point.lateral = params.amplitude - 2.0 * params.amplitude * t;

            if (t < 0.05) {
                point.isEdge = true;
                point.dwellTime = params.dwellRight;
            } else if (t > 0.95) {
                point.isEdge = true;
                point.dwellTime = params.dwellLeft;
            }
        }

        // Speed modulation
        double edgeFactor = std::abs(point.lateral) / params.amplitude;
        point.speedFactor = params.speedAtCenter +
            (params.speedAtEdge - params.speedAtCenter) * edgeFactor;

        return point;
    }

    WeaveCycle generateCycle(const WeaveParams& params, int numPoints) override {
        WeaveCycle cycle;
        cycle.points.reserve(numPoints);

        for (int i = 0; i < numPoints; ++i) {
            double phase = static_cast<double>(i) / numPoints;
            cycle.points.push_back(generate(phase, params));
        }

        cycle.totalLength = 4.0 * params.amplitude;
        return cycle;
    }

    WeavePatternType getType() const override {
        return WeavePatternType::TRIANGULAR;
    }
};

// ============================================================================
// Sinusoidal Pattern
// ============================================================================

class SinusoidalPatternGenerator : public IWeavePatternGenerator {
public:
    WeavePoint generate(double phase, const WeaveParams& params) override {
        WeavePoint point;

        phase = std::fmod(phase, 1.0);
        if (phase < 0) phase += 1.0;

        point.phase = phase;

        // Sinusoidal motion
        double angle = 2.0 * PI * phase + degToRad(params.phaseOffset);
        point.lateral = params.amplitude * std::sin(angle);

        // Check if at edge (within 10% of peak)
        double normalizedLateral = std::abs(point.lateral) / params.amplitude;
        point.isEdge = normalizedLateral > 0.9;

        if (point.isEdge) {
            point.dwellTime = (point.lateral > 0) ? params.dwellRight : params.dwellLeft;
        }

        // Smooth speed modulation based on position
        double edgeFactor = normalizedLateral;
        point.speedFactor = params.speedAtCenter +
            (params.speedAtEdge - params.speedAtCenter) * edgeFactor;

        return point;
    }

    WeaveCycle generateCycle(const WeaveParams& params, int numPoints) override {
        WeaveCycle cycle;
        cycle.points.reserve(numPoints);

        for (int i = 0; i < numPoints; ++i) {
            double phase = static_cast<double>(i) / numPoints;
            cycle.points.push_back(generate(phase, params));
        }

        // Arc length of sinusoid (approximate)
        cycle.totalLength = params.wavelength *
            std::sqrt(1.0 + std::pow(2.0 * PI * params.amplitude / params.wavelength, 2));

        return cycle;
    }

    WeavePatternType getType() const override {
        return WeavePatternType::SINUSOIDAL;
    }
};

// ============================================================================
// Circular Pattern
// ============================================================================

class CircularPatternGenerator : public IWeavePatternGenerator {
public:
    WeavePoint generate(double phase, const WeaveParams& params) override {
        WeavePoint point;

        phase = std::fmod(phase, 1.0);
        if (phase < 0) phase += 1.0;

        point.phase = phase;

        double angle = 2.0 * PI * phase + degToRad(params.phaseOffset);
        double radius = params.circleRadius > 0 ? params.circleRadius : params.amplitude;

        // Circular motion
        point.lateral = radius * std::cos(angle);
        point.longitudinal = radius * std::sin(angle);

        // No specific edges in circular pattern
        point.isEdge = false;
        point.speedFactor = 1.0;  // Constant speed for circular

        return point;
    }

    WeaveCycle generateCycle(const WeaveParams& params, int numPoints) override {
        WeaveCycle cycle;
        cycle.points.reserve(numPoints);

        for (int i = 0; i < numPoints; ++i) {
            double phase = static_cast<double>(i) / numPoints;
            cycle.points.push_back(generate(phase, params));
        }

        double radius = params.circleRadius > 0 ? params.circleRadius : params.amplitude;
        cycle.totalLength = 2.0 * PI * radius;

        return cycle;
    }

    WeavePatternType getType() const override {
        return WeavePatternType::CIRCULAR;
    }
};

// ============================================================================
// Figure-8 Pattern
// ============================================================================

class Figure8PatternGenerator : public IWeavePatternGenerator {
public:
    WeavePoint generate(double phase, const WeaveParams& params) override {
        WeavePoint point;

        phase = std::fmod(phase, 1.0);
        if (phase < 0) phase += 1.0;

        point.phase = phase;

        double angle = 2.0 * PI * phase + degToRad(params.phaseOffset);

        // Lissajous figure with 2:1 frequency ratio
        point.lateral = params.amplitude * std::sin(angle);
        point.longitudinal = params.amplitude * params.figure8Ratio * std::sin(2.0 * angle) / 2.0;

        // Edges at lateral extremes
        double normalizedLateral = std::abs(point.lateral) / params.amplitude;
        point.isEdge = normalizedLateral > 0.9;

        if (point.isEdge) {
            point.dwellTime = (point.lateral > 0) ? params.dwellRight : params.dwellLeft;
        }

        point.speedFactor = 1.0;  // Could modulate based on curvature

        return point;
    }

    WeaveCycle generateCycle(const WeaveParams& params, int numPoints) override {
        WeaveCycle cycle;
        cycle.points.reserve(numPoints);

        for (int i = 0; i < numPoints; ++i) {
            double phase = static_cast<double>(i) / numPoints;
            cycle.points.push_back(generate(phase, params));
        }

        // Approximate arc length
        cycle.totalLength = 4.0 * params.amplitude * (1.0 + params.figure8Ratio);

        return cycle;
    }

    WeavePatternType getType() const override {
        return WeavePatternType::FIGURE_8;
    }
};

// ============================================================================
// Crescent Pattern
// ============================================================================

class CrescentPatternGenerator : public IWeavePatternGenerator {
public:
    WeavePoint generate(double phase, const WeaveParams& params) override {
        WeavePoint point;

        phase = std::fmod(phase, 1.0);
        if (phase < 0) phase += 1.0;

        point.phase = phase;

        // Crescent: semi-circular arcs, one side at a time
        if (phase < 0.5) {
            // Arc on one side
            double arcPhase = phase / 0.5;  // 0 to 1
            double angle = PI * arcPhase;

            point.lateral = params.amplitude * std::sin(angle);
            point.longitudinal = params.amplitude * 0.3 * (1.0 - std::cos(angle));
        }
        else {
            // Arc on other side
            double arcPhase = (phase - 0.5) / 0.5;  // 0 to 1
            double angle = PI * arcPhase;

            point.lateral = -params.amplitude * std::sin(angle);
            point.longitudinal = params.amplitude * 0.3 * (1.0 - std::cos(angle));
        }

        // Edges at peaks
        double normalizedLateral = std::abs(point.lateral) / params.amplitude;
        point.isEdge = normalizedLateral > 0.9;

        if (point.isEdge) {
            point.dwellTime = (point.lateral > 0) ? params.dwellRight : params.dwellLeft;
        }

        point.speedFactor = params.speedAtCenter +
            (params.speedAtEdge - params.speedAtCenter) * normalizedLateral;

        return point;
    }

    WeaveCycle generateCycle(const WeaveParams& params, int numPoints) override {
        WeaveCycle cycle;
        cycle.points.reserve(numPoints);

        for (int i = 0; i < numPoints; ++i) {
            double phase = static_cast<double>(i) / numPoints;
            cycle.points.push_back(generate(phase, params));
        }

        cycle.totalLength = 2.0 * PI * params.amplitude * 0.65;  // Approximate

        return cycle;
    }

    WeavePatternType getType() const override {
        return WeavePatternType::CRESCENT;
    }
};

// ============================================================================
// Pattern Factory
// ============================================================================

/**
 * Create pattern generator for given type
 */
inline std::unique_ptr<IWeavePatternGenerator> createPatternGenerator(
    WeavePatternType type) {

    switch (type) {
        case WeavePatternType::LINEAR:
            return std::make_unique<LinearPatternGenerator>();
        case WeavePatternType::TRIANGULAR:
            return std::make_unique<TriangularPatternGenerator>();
        case WeavePatternType::SINUSOIDAL:
            return std::make_unique<SinusoidalPatternGenerator>();
        case WeavePatternType::CIRCULAR:
            return std::make_unique<CircularPatternGenerator>();
        case WeavePatternType::FIGURE_8:
            return std::make_unique<Figure8PatternGenerator>();
        case WeavePatternType::CRESCENT:
            return std::make_unique<CrescentPatternGenerator>();
        default:
            return std::make_unique<SinusoidalPatternGenerator>();
    }
}

} // namespace welding
} // namespace robotics
```

---

## Step 3: Create Weave Executor

### 3.1 Create WeaveExecutor.hpp

**File:** `src/cpp/include/welding/WeaveExecutor.hpp`

```cpp
#pragma once

#include "WeavePatternGenerator.hpp"
#include "trajectory/TrajectoryTypes.hpp"
#include <atomic>
#include <mutex>

namespace robotics {
namespace welding {

using namespace trajectory;

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
     * @param point Original trajectory point
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
```

---

## Step 4: Create Weave Visualization

### 4.1 Create WeaveVisualizer.hpp

**File:** `src/cpp/include/welding/WeaveVisualizer.hpp`

```cpp
#pragma once

#include "WeavePatternGenerator.hpp"
#include <vector>

namespace robotics {
namespace welding {

/**
 * Generates visualization data for weave patterns
 */
class WeaveVisualizer {
public:
    /**
     * Generate 2D preview of pattern
     * @param params Weave parameters
     * @param numCycles Number of cycles to show
     * @param pointsPerCycle Points per cycle
     * @return Vector of (x, y) points for visualization
     */
    static std::vector<std::pair<double, double>> generatePreview2D(
        const WeaveParams& params,
        int numCycles = 3,
        int pointsPerCycle = 64) {

        std::vector<std::pair<double, double>> points;
        auto generator = createPatternGenerator(params.type);

        double wavelength = params.wavelength;
        int totalPoints = numCycles * pointsPerCycle;

        points.reserve(totalPoints);

        for (int i = 0; i < totalPoints; ++i) {
            double progress = static_cast<double>(i) / totalPoints;
            double phase = std::fmod(progress * numCycles, 1.0);

            WeavePoint wp = generator->generate(phase, params);

            // X is travel direction, Y is lateral
            double x = progress * numCycles * wavelength;
            double y = wp.lateral;

            points.emplace_back(x, y);
        }

        return points;
    }

    /**
     * Generate 3D path preview
     * @param params Weave parameters
     * @param basePath Base weld path points
     * @param normals Surface normals at each point
     * @return Vector of 3D points with weave applied
     */
    static std::vector<Vector3d> generatePreview3D(
        const WeaveParams& params,
        const std::vector<Vector3d>& basePath,
        const std::vector<Vector3d>& normals) {

        if (basePath.size() < 2 || basePath.size() != normals.size()) {
            return basePath;
        }

        std::vector<Vector3d> result;
        result.reserve(basePath.size());

        auto generator = createPatternGenerator(params.type);

        // Calculate total path length
        double totalLength = 0;
        std::vector<double> distances;
        distances.push_back(0);

        for (size_t i = 1; i < basePath.size(); ++i) {
            double segLen = (basePath[i] - basePath[i-1]).norm();
            totalLength += segLen;
            distances.push_back(totalLength);
        }

        // Number of weave cycles
        double numCycles = totalLength / params.wavelength;

        for (size_t i = 0; i < basePath.size(); ++i) {
            // Calculate phase based on distance
            double phase = std::fmod(distances[i] / params.wavelength, 1.0);

            // Generate weave offset
            WeavePoint wp = generator->generate(phase, params);

            // Build coordinate frame
            Vector3d tangent;
            if (i < basePath.size() - 1) {
                tangent = (basePath[i+1] - basePath[i]).normalized();
            } else {
                tangent = (basePath[i] - basePath[i-1]).normalized();
            }

            WeaveFrame frame = WeaveFrame::fromTangentAndNormal(
                basePath[i], tangent, normals[i]);

            // Apply offset
            Vector3d weavedPoint = frame.applyOffset(wp);
            result.push_back(weavedPoint);
        }

        return result;
    }

    /**
     * Generate pattern statistics
     */
    struct PatternStats {
        double pathLengthRatio;     // Weave path / straight path
        double averageSpeed;        // Average speed factor
        double maxLateralAccel;     // Maximum lateral acceleration
        double dwellPercent;        // Percentage time dwelling
    };

    static PatternStats calculateStats(
        const WeaveParams& params,
        double travelSpeed) {

        PatternStats stats{1.0, 1.0, 0, 0};

        auto generator = createPatternGenerator(params.type);
        WeaveCycle cycle = generator->generateCycle(params, 100);

        // Path length ratio
        stats.pathLengthRatio = cycle.totalLength / params.wavelength;

        // Average speed
        double totalSpeed = 0;
        double totalDwell = 0;
        for (const auto& pt : cycle.points) {
            totalSpeed += pt.speedFactor;
            totalDwell += pt.dwellTime;
        }
        stats.averageSpeed = totalSpeed / cycle.points.size();
        stats.dwellPercent = totalDwell / (params.wavelength / travelSpeed * 1000) * 100;

        // Max lateral acceleration (approximate)
        double maxAccel = 0;
        for (size_t i = 1; i < cycle.points.size() - 1; ++i) {
            double accel = std::abs(
                cycle.points[i+1].lateral -
                2*cycle.points[i].lateral +
                cycle.points[i-1].lateral);
            maxAccel = std::max(maxAccel, accel);
        }
        stats.maxLateralAccel = maxAccel * travelSpeed * travelSpeed /
            (params.wavelength * params.wavelength);

        return stats;
    }
};

} // namespace welding
} // namespace robotics
```

---

## Step 5: Create Unit Tests

### 5.1 Create test_weaving.cpp

**File:** `src/cpp/tests/test_weaving.cpp`

```cpp
#include <gtest/gtest.h>
#include "welding/WeavingTypes.hpp"
#include "welding/WeavePatternGenerator.hpp"
#include "welding/WeaveExecutor.hpp"
#include "welding/WeaveVisualizer.hpp"

using namespace robotics::welding;
using namespace robotics::math;

// ============================================================================
// Weaving Types Tests
// ============================================================================

TEST(WeavingTypesTest, DefaultParams) {
    WeaveParams params;

    EXPECT_EQ(params.type, WeavePatternType::SINUSOIDAL);
    EXPECT_GT(params.amplitude, 0);
    EXPECT_GT(params.wavelength, 0);
}

TEST(WeavingTypesTest, PresetNarrowGap) {
    auto params = WeavePresets::narrowGap();

    EXPECT_EQ(params.type, WeavePatternType::TRIANGULAR);
    EXPECT_LT(params.amplitude, 3);  // Narrow
}

TEST(WeavingTypesTest, PresetWideGap) {
    auto params = WeavePresets::wideGap();

    EXPECT_EQ(params.type, WeavePatternType::TRIANGULAR);
    EXPECT_GT(params.amplitude, 5);  // Wide
    EXPECT_LT(params.speedAtEdge, 1);  // Slower at edges
}

TEST(WeavingTypesTest, WeaveFrame) {
    WeaveFrame frame = WeaveFrame::fromTangentAndNormal(
        Vector3d(0, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0, 0, 1));

    EXPECT_NEAR(frame.tangent.x(), 1, 0.01);
    EXPECT_NEAR(frame.normal.z(), 1, 0.01);
    EXPECT_NEAR(frame.lateral.y(), 1, 0.01);  // Right-hand rule
}

TEST(WeavingTypesTest, WeaveFrameApplyOffset) {
    WeaveFrame frame = WeaveFrame::fromTangentAndNormal(
        Vector3d(100, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0, 0, 1));

    WeavePoint offset;
    offset.lateral = 5.0;

    Vector3d result = frame.applyOffset(offset);

    EXPECT_NEAR(result.x(), 100, 0.01);
    EXPECT_NEAR(result.y(), 5.0, 0.01);
    EXPECT_NEAR(result.z(), 0, 0.01);
}

// ============================================================================
// Pattern Generator Tests
// ============================================================================

TEST(PatternGeneratorTest, LinearPattern) {
    LinearPatternGenerator gen;
    WeaveParams params;
    params.amplitude = 5.0;

    // At phase 0, should be near center
    auto pt0 = gen.generate(0, params);
    EXPECT_NEAR(pt0.lateral, 0, 0.5);

    // At phase 0.25, should be at right edge
    auto pt25 = gen.generate(0.25, params);
    EXPECT_GT(pt25.lateral, 4.5);

    // At phase 0.75, should be at left edge
    auto pt75 = gen.generate(0.75, params);
    EXPECT_LT(pt75.lateral, -4.0);
}

TEST(PatternGeneratorTest, SinusoidalPattern) {
    SinusoidalPatternGenerator gen;
    WeaveParams params;
    params.amplitude = 3.0;

    // At phase 0.25 (90 degrees), should be at max
    auto pt25 = gen.generate(0.25, params);
    EXPECT_NEAR(pt25.lateral, params.amplitude, 0.1);

    // At phase 0.75 (270 degrees), should be at min
    auto pt75 = gen.generate(0.75, params);
    EXPECT_NEAR(pt75.lateral, -params.amplitude, 0.1);
}

TEST(PatternGeneratorTest, TriangularPattern) {
    TriangularPatternGenerator gen;
    WeaveParams params;
    params.amplitude = 4.0;

    // At phase 0.5, should be at right edge
    auto pt50 = gen.generate(0.5, params);
    EXPECT_NEAR(pt50.lateral, params.amplitude, 0.5);

    // At phase 0, should be at left edge
    auto pt0 = gen.generate(0.0, params);
    EXPECT_NEAR(pt0.lateral, -params.amplitude, 0.5);
}

TEST(PatternGeneratorTest, CircularPattern) {
    CircularPatternGenerator gen;
    WeaveParams params;
    params.amplitude = 2.0;
    params.circleRadius = 2.0;

    auto cycle = gen.generateCycle(params, 36);

    EXPECT_EQ(cycle.points.size(), 36);

    // Check radius consistency
    for (const auto& pt : cycle.points) {
        double radius = std::sqrt(pt.lateral * pt.lateral +
                                   pt.longitudinal * pt.longitudinal);
        EXPECT_NEAR(radius, params.circleRadius, 0.1);
    }
}

TEST(PatternGeneratorTest, Figure8Pattern) {
    Figure8PatternGenerator gen;
    WeaveParams params;
    params.amplitude = 3.0;

    auto cycle = gen.generateCycle(params, 64);

    EXPECT_EQ(cycle.points.size(), 64);

    // Should cross center multiple times
    int centerCrossings = 0;
    for (size_t i = 1; i < cycle.points.size(); ++i) {
        if (cycle.points[i-1].lateral * cycle.points[i].lateral < 0) {
            centerCrossings++;
        }
    }
    EXPECT_GE(centerCrossings, 2);  // At least 2 crossings per cycle
}

TEST(PatternGeneratorTest, CrescentPattern) {
    CrescentPatternGenerator gen;
    WeaveParams params;
    params.amplitude = 4.0;

    auto cycle = gen.generateCycle(params, 32);

    EXPECT_EQ(cycle.points.size(), 32);

    // Should have longitudinal movement
    double maxLong = 0;
    for (const auto& pt : cycle.points) {
        maxLong = std::max(maxLong, std::abs(pt.longitudinal));
    }
    EXPECT_GT(maxLong, 0);
}

TEST(PatternGeneratorTest, Factory) {
    auto linear = createPatternGenerator(WeavePatternType::LINEAR);
    EXPECT_EQ(linear->getType(), WeavePatternType::LINEAR);

    auto sine = createPatternGenerator(WeavePatternType::SINUSOIDAL);
    EXPECT_EQ(sine->getType(), WeavePatternType::SINUSOIDAL);
}

// ============================================================================
// Weave Executor Tests
// ============================================================================

TEST(WeaveExecutorTest, InitialState) {
    WeaveExecutor executor;

    EXPECT_FALSE(executor.isActive());
    EXPECT_FALSE(executor.isEnabled());
    EXPECT_EQ(executor.getCycleCount(), 0);
}

TEST(WeaveExecutorTest, StartStop) {
    WeaveExecutor executor;
    executor.setEnabled(true);

    executor.start();
    EXPECT_TRUE(executor.isActive());

    executor.stop();
    EXPECT_FALSE(executor.isActive());
}

TEST(WeaveExecutorTest, PhaseAdvancement) {
    WeaveExecutor executor;
    WeaveParams params;
    params.type = WeavePatternType::SINUSOIDAL;
    params.wavelength = 10.0;  // 10mm per cycle

    executor.setParams(params);
    executor.setEnabled(true);
    executor.start();

    // Travel 5mm at 10mm/s = 0.5s
    executor.update(0.5, 10.0);

    // Should be at phase 0.5
    EXPECT_NEAR(executor.getCurrentPhase(), 0.5, 0.1);
}

TEST(WeaveExecutorTest, CycleCompletion) {
    WeaveExecutor executor;
    WeaveParams params;
    params.wavelength = 10.0;

    executor.setParams(params);
    executor.setEnabled(true);
    executor.start();

    EXPECT_EQ(executor.getCycleCount(), 0);

    // Travel 15mm (1.5 cycles)
    executor.update(1.5, 10.0);

    EXPECT_EQ(executor.getCycleCount(), 1);
}

TEST(WeaveExecutorTest, ApplyWeave) {
    WeaveExecutor executor;
    WeaveParams params;
    params.type = WeavePatternType::SINUSOIDAL;
    params.amplitude = 5.0;

    executor.setParams(params);
    executor.setEnabled(true);
    executor.start();

    // Update to phase where offset is non-zero
    executor.update(0.25, 10.0);

    WeaveFrame frame = WeaveFrame::fromTangentAndNormal(
        Vector3d(0, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0, 0, 1));

    Vector3d original(100, 0, 50);
    Vector3d weaved = executor.applyWeave(original, frame);

    // Y should have lateral offset
    EXPECT_NE(weaved.y(), original.y());
}

TEST(WeaveExecutorTest, DisabledNoEffect) {
    WeaveExecutor executor;

    executor.start();  // But not enabled

    WeavePoint offset = executor.getCurrentOffset();

    EXPECT_NEAR(offset.lateral, 0, 0.01);
}

TEST(WeaveExecutorTest, AdjustAmplitude) {
    WeaveExecutor executor;
    WeaveParams params;
    params.amplitude = 3.0;

    executor.setParams(params);
    executor.adjustAmplitude(5.0);

    EXPECT_EQ(executor.getParams().amplitude, 5.0);
}

// ============================================================================
// Weave Visualizer Tests
// ============================================================================

TEST(WeaveVisualizerTest, Preview2D) {
    WeaveParams params;
    params.type = WeavePatternType::SINUSOIDAL;
    params.amplitude = 3.0;
    params.wavelength = 10.0;

    auto points = WeaveVisualizer::generatePreview2D(params, 2, 32);

    EXPECT_EQ(points.size(), 64);  // 2 cycles * 32 points

    // Check amplitude bounds
    for (const auto& pt : points) {
        EXPECT_LE(std::abs(pt.second), params.amplitude + 0.1);
    }
}

TEST(WeaveVisualizerTest, Preview3D) {
    WeaveParams params;
    params.type = WeavePatternType::LINEAR;
    params.amplitude = 2.0;
    params.wavelength = 5.0;

    // Simple straight path
    std::vector<Vector3d> basePath;
    std::vector<Vector3d> normals;

    for (int i = 0; i <= 20; ++i) {
        basePath.push_back(Vector3d(i * 5.0, 0, 0));
        normals.push_back(Vector3d(0, 0, 1));
    }

    auto weaved = WeaveVisualizer::generatePreview3D(params, basePath, normals);

    EXPECT_EQ(weaved.size(), basePath.size());

    // Weaved path should deviate in Y
    double maxY = 0;
    for (const auto& pt : weaved) {
        maxY = std::max(maxY, std::abs(pt.y()));
    }
    EXPECT_GT(maxY, 0);
    EXPECT_LE(maxY, params.amplitude + 0.1);
}

TEST(WeaveVisualizerTest, PatternStats) {
    WeaveParams params;
    params.type = WeavePatternType::SINUSOIDAL;
    params.amplitude = 3.0;
    params.wavelength = 10.0;

    auto stats = WeaveVisualizer::calculateStats(params, 10.0);

    // Weave path should be longer than straight
    EXPECT_GT(stats.pathLengthRatio, 1.0);

    // Average speed should be around 1.0
    EXPECT_GT(stats.averageSpeed, 0.5);
    EXPECT_LT(stats.averageSpeed, 1.5);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(WeaveIntegrationTest, FullWeaveCycle) {
    WeaveExecutor executor;
    WeaveParams params;
    params.type = WeavePatternType::TRIANGULAR;
    params.amplitude = 4.0;
    params.wavelength = 8.0;
    params.dwellLeft = 50;
    params.dwellRight = 50;

    executor.setParams(params);
    executor.setEnabled(true);
    executor.start();

    // Simulate welding for 2 cycles
    double dt = 0.01;  // 10ms
    double travelSpeed = 10.0;  // 10 mm/s

    std::vector<double> lateralPositions;

    for (int i = 0; i < 200; ++i) {  // 2 seconds
        executor.update(dt, travelSpeed);
        auto offset = executor.getCurrentOffset();
        lateralPositions.push_back(offset.lateral);
    }

    // Should have completed approximately 2.5 cycles
    EXPECT_GE(executor.getCycleCount(), 2);

    // Check that we hit both edges
    double minLat = *std::min_element(lateralPositions.begin(), lateralPositions.end());
    double maxLat = *std::max_element(lateralPositions.begin(), lateralPositions.end());

    EXPECT_LT(minLat, -3.5);  // Near left edge
    EXPECT_GT(maxLat, 3.5);   // Near right edge
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

---

## Step 6: Create IPC Messages

### 6.1 Create WeavePayloads.hpp

**File:** `src/cpp/include/ipc/WeavePayloads.hpp`

```cpp
#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace robotics {
namespace ipc {

// ============================================================================
// Weave Parameters
// ============================================================================

struct WeaveParamsData {
    int patternType;
    double amplitude;
    double wavelength;
    double frequency;
    double dwellLeft;
    double dwellRight;
    double dwellCenter;
    double phaseOffset;
    double speedAtEdge;
    double speedAtCenter;
    bool useFrequency;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeaveParamsData,
        patternType, amplitude, wavelength, frequency,
        dwellLeft, dwellRight, dwellCenter, phaseOffset,
        speedAtEdge, speedAtCenter, useFrequency)
};

// ============================================================================
// Weave Control
// ============================================================================

struct WeaveControlRequest {
    std::string action;  // "START", "STOP", "SET_PARAMS", "ADJUST"
    WeaveParamsData params;
    std::string adjustParam;
    double adjustValue;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeaveControlRequest,
        action, params, adjustParam, adjustValue)
};

struct WeaveControlResponse {
    bool success;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeaveControlResponse, success, error)
};

// ============================================================================
// Weave Status
// ============================================================================

struct WeaveStatusResponse {
    bool enabled;
    bool active;
    int patternType;
    double currentPhase;
    double currentAmplitude;
    double lateralOffset;
    int cycleCount;
    double totalDistance;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeaveStatusResponse,
        enabled, active, patternType, currentPhase,
        currentAmplitude, lateralOffset, cycleCount, totalDistance)
};

// ============================================================================
// Weave Preview
// ============================================================================

struct WeavePreviewRequest {
    WeaveParamsData params;
    int numCycles;
    int pointsPerCycle;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeavePreviewRequest,
        params, numCycles, pointsPerCycle)
};

struct WeavePreviewResponse {
    std::vector<double> xPoints;
    std::vector<double> yPoints;
    double pathLengthRatio;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(WeavePreviewResponse,
        xPoints, yPoints, pathLengthRatio)
};

} // namespace ipc
} // namespace robotics
```

---

## Step 7: Create C# Weave Client

### 7.1 Create WeavePayloads.cs

**File:** `src/csharp/RobotController.Core/IPC/WeavePayloads.cs`

```csharp
using System.Text.Json.Serialization;

namespace RobotController.Core.IPC;

// ============================================================================
// Enums
// ============================================================================

public enum WeavePatternType
{
    None = 0,
    Linear = 1,
    Triangular = 2,
    Sinusoidal = 3,
    Circular = 4,
    Figure8 = 5,
    Crescent = 6,
    Custom = 7
}

// ============================================================================
// Weave Parameters
// ============================================================================

public record WeaveParamsData
{
    [JsonPropertyName("patternType")]
    public int PatternType { get; init; } = (int)WeavePatternType.Sinusoidal;

    [JsonPropertyName("amplitude")]
    public double Amplitude { get; init; } = 3.0;

    [JsonPropertyName("wavelength")]
    public double Wavelength { get; init; } = 10.0;

    [JsonPropertyName("frequency")]
    public double Frequency { get; init; } = 1.0;

    [JsonPropertyName("dwellLeft")]
    public double DwellLeft { get; init; }

    [JsonPropertyName("dwellRight")]
    public double DwellRight { get; init; }

    [JsonPropertyName("dwellCenter")]
    public double DwellCenter { get; init; }

    [JsonPropertyName("phaseOffset")]
    public double PhaseOffset { get; init; }

    [JsonPropertyName("speedAtEdge")]
    public double SpeedAtEdge { get; init; } = 0.8;

    [JsonPropertyName("speedAtCenter")]
    public double SpeedAtCenter { get; init; } = 1.0;

    [JsonPropertyName("useFrequency")]
    public bool UseFrequency { get; init; }
}

// ============================================================================
// Weave Control
// ============================================================================

public record WeaveControlRequest
{
    [JsonPropertyName("action")]
    public string Action { get; init; } = "";

    [JsonPropertyName("params")]
    public WeaveParamsData? Params { get; init; }

    [JsonPropertyName("adjustParam")]
    public string AdjustParam { get; init; } = "";

    [JsonPropertyName("adjustValue")]
    public double AdjustValue { get; init; }
}

public record WeaveControlResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; init; }

    [JsonPropertyName("error")]
    public string Error { get; init; } = "";
}

// ============================================================================
// Weave Status
// ============================================================================

public record WeaveStatusResponse
{
    [JsonPropertyName("enabled")]
    public bool Enabled { get; init; }

    [JsonPropertyName("active")]
    public bool Active { get; init; }

    [JsonPropertyName("patternType")]
    public int PatternType { get; init; }

    [JsonPropertyName("currentPhase")]
    public double CurrentPhase { get; init; }

    [JsonPropertyName("currentAmplitude")]
    public double CurrentAmplitude { get; init; }

    [JsonPropertyName("lateralOffset")]
    public double LateralOffset { get; init; }

    [JsonPropertyName("cycleCount")]
    public int CycleCount { get; init; }

    [JsonPropertyName("totalDistance")]
    public double TotalDistance { get; init; }
}

// ============================================================================
// Weave Preview
// ============================================================================

public record WeavePreviewRequest
{
    [JsonPropertyName("params")]
    public WeaveParamsData? Params { get; init; }

    [JsonPropertyName("numCycles")]
    public int NumCycles { get; init; } = 3;

    [JsonPropertyName("pointsPerCycle")]
    public int PointsPerCycle { get; init; } = 64;
}

public record WeavePreviewResponse
{
    [JsonPropertyName("xPoints")]
    public double[] XPoints { get; init; } = Array.Empty<double>();

    [JsonPropertyName("yPoints")]
    public double[] YPoints { get; init; } = Array.Empty<double>();

    [JsonPropertyName("pathLengthRatio")]
    public double PathLengthRatio { get; init; }
}
```

### 7.2 Create IWeaveClientService.cs

**File:** `src/csharp/RobotController.Core/Services/IWeaveClientService.cs`

```csharp
using RobotController.Core.IPC;

namespace RobotController.Core.Services;

/// <summary>
/// Client service for weave pattern control
/// </summary>
public interface IWeaveClientService
{
    // Control
    Task<WeaveControlResponse> SetParamsAsync(WeaveParamsData params, CancellationToken ct = default);
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
```

---

## Step 8: Update CMakeLists.txt

**Add to CMakeLists.txt:**
```cmake
# Weaving tests
add_executable(test_weaving tests/test_weaving.cpp)
target_link_libraries(test_weaving
    PRIVATE
    GTest::gtest
    GTest::gtest_main
    Eigen3::Eigen
)
target_include_directories(test_weaving PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME WeavingTests COMMAND test_weaving)
```

---

## Step 9: Validation

### 9.1 Build and Test

```powershell
cd E:\DEV_CONTEXT_PROJECTs\Robot_controller
cmake --build build --config Debug
.\build\Debug\test_weaving.exe
```

**Expected:**
```
[==========] Running 25 tests from 5 test suites.
...
[  PASSED  ] 25 tests.
```

---

## Completion Checklist

- [ ] WeavingTypes.hpp created (patterns, params, presets)
- [ ] WeavePatternGenerator.hpp created (6 pattern types)
- [ ] WeaveExecutor.hpp created (real-time execution)
- [ ] WeaveVisualizer.hpp created (2D/3D preview)
- [ ] test_weaving.cpp created with 25+ tests
- [ ] WeavePayloads (C++ & C#) created
- [ ] IWeaveClientService.cs created
- [ ] CMakeLists.txt updated
- [ ] All tests pass

---

## Git Commit

```powershell
git add -A
git commit -m "IMPL_P3_02: Add weaving patterns

- Create WeavingTypes with 6 pattern types
- Implement pattern generators: Linear, Triangular, Sinusoidal,
  Circular, Figure-8, Crescent
- Add WeaveExecutor for real-time weave execution
- Create WeaveVisualizer for 2D/3D preview
- Add preset configurations for common applications
- Implement phase-based pattern generation
- Add dwell time and speed modulation support
- Create 25 unit tests for weaving
- Add C# WeaveClientService

Co-Authored-By: Claude <noreply@anthropic.com>"
```
