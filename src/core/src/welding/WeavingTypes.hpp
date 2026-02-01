#pragma once

#include "../kinematics/MathTypes.hpp"
#include <string>
#include <vector>
#include <cmath>

namespace robotics {
namespace welding {

using namespace robot_controller::kinematics;

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
