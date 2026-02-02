#pragma once

#include "VisionTypes.hpp"
#include "SeamTypes.hpp"
#include <vector>

namespace robot_controller::vision {

/// Processes raw laser profiles for seam detection
class ProfileProcessor {
public:
    ProfileProcessor();
    ~ProfileProcessor() = default;

    /// Set processing configuration
    void setConfig(const SeamDetectionConfig& config);

    /// Process raw profile and extract features
    SeamFeature processProfile(const LaserProfile& profile);

    /// Apply median filter to profile
    void medianFilter(std::vector<Point2D>& points, int kernelSize);

    /// Remove outliers using statistical analysis
    void removeOutliers(std::vector<Point2D>& points, float threshold);

    /// Extract centerline using Steger algorithm (sub-pixel accuracy)
    std::vector<Point2D> extractCenterline(
        const std::vector<Point2D>& points,
        float sigma);

    /// Segment profile into regions
    struct ProfileSegment {
        std::vector<Point2D> points;
        float startX{0.0f};
        float endX{0.0f};
        float avgZ{0.0f};
        float slope{0.0f};
        bool isFlat{false};
        bool isSlope{false};
    };
    std::vector<ProfileSegment> segmentProfile(const std::vector<Point2D>& points);

private:
    SeamDetectionConfig config_;

    // Helpers
    float computeMedian(std::vector<float>& values);
    float computeStdDev(const std::vector<float>& values, float mean);
};

} // namespace robot_controller::vision
