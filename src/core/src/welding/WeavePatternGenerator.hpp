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
