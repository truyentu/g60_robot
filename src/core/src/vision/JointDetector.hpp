#pragma once

#include "SeamTypes.hpp"
#include "ProfileProcessor.hpp"
#include <memory>
#include <optional>
#include <random>

namespace robot_controller::vision {

/// RANSAC result for line fitting
struct RansacResult {
    float slope{0.0f};
    float intercept{0.0f};
    float r2{0.0f};
    std::vector<int> inlierIndices;
    bool valid{false};
};

/// Detects weld joint type and extracts features
class JointDetector {
public:
    JointDetector();
    ~JointDetector() = default;

    /// Set detection configuration
    void setConfig(const SeamDetectionConfig& config);

    /// Detect joint type from profile
    SeamFeature detect(const LaserProfile& profile);

    /// Detect V-groove joint
    SeamFeature detectVGroove(const std::vector<Point2D>& points);

    /// Detect lap joint
    SeamFeature detectLapJoint(const std::vector<Point2D>& points);

    /// Detect fillet joint
    SeamFeature detectFilletJoint(const std::vector<Point2D>& points);

    /// Auto-detect joint type
    JointType classifyJointType(const std::vector<Point2D>& points);

    /// RANSAC line fitting
    RansacResult ransacLineFit(
        const std::vector<Point2D>& points,
        int iterations,
        float threshold,
        int minInliers);

    /// Least squares line fitting
    RansacResult leastSquaresLineFit(const std::vector<Point2D>& points);

    /// Find intersection of two lines
    std::optional<Point2D> findLineIntersection(
        const RansacResult& line1,
        const RansacResult& line2);

private:
    SeamDetectionConfig config_;
    ProfileProcessor profileProcessor_;
    std::mt19937 rng_;

    // Helper methods
    std::vector<Point2D> filterValidPoints(const std::vector<Point2D>& points);
    float computeAngle(float slope);
    float lineDistance(const Point2D& point, float slope, float intercept);
};

} // namespace robot_controller::vision
