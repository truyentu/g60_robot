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
        (void)numCycles; // Unused but kept for reference

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
