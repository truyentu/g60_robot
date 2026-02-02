#include "JointDetector.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace robot_controller::vision {

JointDetector::JointDetector()
    : rng_(std::random_device{}()) {
    spdlog::debug("JointDetector created");
}

void JointDetector::setConfig(const SeamDetectionConfig& config) {
    config_ = config;
    profileProcessor_.setConfig(config);
}

SeamFeature JointDetector::detect(const LaserProfile& profile) {
    SeamFeature feature;
    feature.timestamp = profile.timestamp;
    feature.frameId = profile.frameId;

    // Get valid points
    auto points = filterValidPoints(profile.points);

    if (points.size() < 20) {
        spdlog::debug("Insufficient points for detection: {}", points.size());
        return feature;
    }

    // Determine joint type
    JointType type = config_.expectedJointType;
    if (type == JointType::Unknown) {
        type = classifyJointType(points);
    }

    // Detect based on joint type
    switch (type) {
        case JointType::VGroove:
            feature = detectVGroove(points);
            break;
        case JointType::LapJoint:
            feature = detectLapJoint(points);
            break;
        case JointType::FilletJoint:
            feature = detectFilletJoint(points);
            break;
        default:
            feature = detectVGroove(points);  // Default to V-groove
            break;
    }

    feature.timestamp = profile.timestamp;
    feature.frameId = profile.frameId;
    return feature;
}

SeamFeature JointDetector::detectVGroove(const std::vector<Point2D>& points) {
    SeamFeature feature;
    feature.jointType = JointType::VGroove;

    if (points.size() < 20) return feature;

    // Find the minimum Z point (bottom of groove)
    size_t minZIndex = 0;
    float minZ = points[0].z;
    for (size_t i = 1; i < points.size(); ++i) {
        if (points[i].z < minZ) {
            minZ = points[i].z;
            minZIndex = i;
        }
    }

    // Split points into left and right of minimum
    std::vector<Point2D> leftPoints, rightPoints;
    for (size_t i = 0; i < minZIndex; ++i) {
        leftPoints.push_back(points[i]);
    }
    for (size_t i = minZIndex + 1; i < points.size(); ++i) {
        rightPoints.push_back(points[i]);
    }

    if (leftPoints.size() < 5 || rightPoints.size() < 5) {
        return feature;
    }

    // Fit lines to left and right sides
    auto leftFit = ransacLineFit(leftPoints, config_.ransacIterations,
                                  config_.ransacThreshold, config_.ransacMinInliers);
    auto rightFit = ransacLineFit(rightPoints, config_.ransacIterations,
                                   config_.ransacThreshold, config_.ransacMinInliers);

    if (!leftFit.valid || !rightFit.valid) {
        return feature;
    }

    // Find intersection (root point)
    auto intersection = findLineIntersection(leftFit, rightFit);
    if (!intersection) {
        // Use the minimum point
        feature.rootPoint = points[minZIndex];
    } else {
        feature.rootPoint = *intersection;
    }

    // Calculate angles
    feature.leftAngle = computeAngle(leftFit.slope);
    feature.rightAngle = computeAngle(rightFit.slope);

    // Store line fits
    feature.leftLine.slope = leftFit.slope;
    feature.leftLine.intercept = leftFit.intercept;
    feature.leftLine.r2 = leftFit.r2;
    feature.leftLine.inlierCount = static_cast<int>(leftFit.inlierIndices.size());

    feature.rightLine.slope = rightFit.slope;
    feature.rightLine.intercept = rightFit.intercept;
    feature.rightLine.r2 = rightFit.r2;
    feature.rightLine.inlierCount = static_cast<int>(rightFit.inlierIndices.size());

    // Calculate gap width (horizontal distance at top of groove)
    if (!leftPoints.empty() && !rightPoints.empty()) {
        float leftX = leftPoints.front().x;
        float rightX = rightPoints.back().x;
        feature.gapWidth = std::abs(rightX - leftX);
    }

    // Calculate depth
    float maxZ = std::max(leftPoints.front().z, rightPoints.back().z);
    feature.depth = maxZ - feature.rootPoint.z;

    // Plate heights
    feature.leftPlateHeight = leftPoints.front().z;
    feature.rightPlateHeight = rightPoints.back().z;

    // Joint center (same as root for V-groove)
    feature.jointCenter = feature.rootPoint;

    // Confidence based on fit quality
    feature.confidence = (leftFit.r2 + rightFit.r2) / 2.0f;
    feature.valid = feature.confidence > 0.3f;

    return feature;
}

SeamFeature JointDetector::detectLapJoint(const std::vector<Point2D>& points) {
    SeamFeature feature;
    feature.jointType = JointType::LapJoint;

    if (points.size() < 20) return feature;

    // Find step (sudden Z change)
    float maxStep = 0.0f;
    size_t stepIndex = 0;

    for (size_t i = 5; i < points.size() - 5; ++i) {
        float step = std::abs(points[i+1].z - points[i].z);
        if (step > maxStep) {
            maxStep = step;
            stepIndex = i;
        }
    }

    if (maxStep < config_.minStepHeight) {
        return feature;  // No significant step found
    }

    // Root point is at the step
    feature.rootPoint.x = (points[stepIndex].x + points[stepIndex+1].x) / 2.0f;
    feature.rootPoint.z = std::min(points[stepIndex].z, points[stepIndex+1].z);

    // Joint center slightly offset from step
    feature.jointCenter = feature.rootPoint;

    // Plate heights
    feature.leftPlateHeight = points.front().z;
    feature.rightPlateHeight = points.back().z;
    feature.plateThickness = maxStep;

    feature.depth = maxStep;
    feature.confidence = std::min(1.0f, maxStep / config_.maxStepHeight);
    feature.valid = true;

    return feature;
}

SeamFeature JointDetector::detectFilletJoint(const std::vector<Point2D>& points) {
    SeamFeature feature;
    feature.jointType = JointType::FilletJoint;

    if (points.size() < 20) return feature;

    // Find corner (intersection of horizontal and vertical surfaces)
    // Look for maximum curvature point

    float maxCurvature = 0.0f;
    size_t cornerIndex = 0;

    for (size_t i = 5; i < points.size() - 5; ++i) {
        // Compute second derivative (curvature approximation)
        float dx1 = points[i].x - points[i-3].x;
        float dz1 = points[i].z - points[i-3].z;
        float dx2 = points[i+3].x - points[i].x;
        float dz2 = points[i+3].z - points[i].z;

        float angle1 = std::atan2(dz1, dx1);
        float angle2 = std::atan2(dz2, dx2);
        float curvature = std::abs(angle2 - angle1);

        if (curvature > maxCurvature) {
            maxCurvature = curvature;
            cornerIndex = i;
        }
    }

    if (cornerIndex == 0) {
        return feature;
    }

    // Root point at corner
    feature.rootPoint = points[cornerIndex];
    feature.jointCenter = feature.rootPoint;

    // Fit lines before and after corner
    std::vector<Point2D> beforeCorner(points.begin(), points.begin() + cornerIndex);
    std::vector<Point2D> afterCorner(points.begin() + cornerIndex, points.end());

    if (beforeCorner.size() >= 5 && afterCorner.size() >= 5) {
        auto leftFit = leastSquaresLineFit(beforeCorner);
        auto rightFit = leastSquaresLineFit(afterCorner);

        feature.leftAngle = computeAngle(leftFit.slope);
        feature.rightAngle = computeAngle(rightFit.slope);
    }

    feature.leftPlateHeight = points.front().z;
    feature.rightPlateHeight = points.back().z;

    feature.confidence = std::min(1.0f, maxCurvature / 1.5f);
    feature.valid = maxCurvature > 0.3f;

    return feature;
}

JointType JointDetector::classifyJointType(const std::vector<Point2D>& points) {
    if (points.size() < 20) return JointType::Unknown;

    // Analyze profile shape to determine joint type

    // Find min Z and check if it's a clear valley (V-groove)
    size_t minIdx = 0;
    float minZ = points[0].z;
    for (size_t i = 1; i < points.size(); ++i) {
        if (points[i].z < minZ) {
            minZ = points[i].z;
            minIdx = i;
        }
    }

    // Check if min is in the middle (V-groove) or edge (lap/fillet)
    float position = static_cast<float>(minIdx) / points.size();

    if (position > 0.3f && position < 0.7f) {
        // Minimum in the middle - likely V-groove
        float leftZ = points[0].z;
        float rightZ = points.back().z;
        float depth = (leftZ + rightZ) / 2.0f - minZ;

        if (depth > config_.minGapWidth) {
            return JointType::VGroove;
        }
    }

    // Check for step (lap joint)
    float maxStep = 0.0f;
    for (size_t i = 1; i < points.size(); ++i) {
        float step = std::abs(points[i].z - points[i-1].z);
        maxStep = std::max(maxStep, step);
    }

    if (maxStep > config_.minStepHeight) {
        return JointType::LapJoint;
    }

    // Check for corner (fillet)
    // If neither V nor lap, assume fillet
    return JointType::FilletJoint;
}

RansacResult JointDetector::ransacLineFit(
    const std::vector<Point2D>& points,
    int iterations,
    float threshold,
    int minInliers) {

    RansacResult bestResult;

    if (points.size() < 2) return bestResult;

    std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

    for (int iter = 0; iter < iterations; ++iter) {
        // Randomly select 2 points
        size_t idx1 = dist(rng_);
        size_t idx2 = dist(rng_);
        if (idx1 == idx2) continue;

        const auto& p1 = points[idx1];
        const auto& p2 = points[idx2];

        float dx = p2.x - p1.x;
        if (std::abs(dx) < 0.001f) continue;

        float slope = (p2.z - p1.z) / dx;
        float intercept = p1.z - slope * p1.x;

        // Count inliers
        std::vector<int> inliers;
        for (size_t i = 0; i < points.size(); ++i) {
            float dist = lineDistance(points[i], slope, intercept);
            if (dist < threshold) {
                inliers.push_back(static_cast<int>(i));
            }
        }

        if (static_cast<int>(inliers.size()) >= minInliers &&
            inliers.size() > bestResult.inlierIndices.size()) {
            bestResult.slope = slope;
            bestResult.intercept = intercept;
            bestResult.inlierIndices = inliers;
            bestResult.valid = true;
        }
    }

    // Refit using all inliers
    if (bestResult.valid && !bestResult.inlierIndices.empty()) {
        std::vector<Point2D> inlierPoints;
        for (int idx : bestResult.inlierIndices) {
            inlierPoints.push_back(points[idx]);
        }
        auto refined = leastSquaresLineFit(inlierPoints);
        bestResult.slope = refined.slope;
        bestResult.intercept = refined.intercept;
        bestResult.r2 = refined.r2;
    }

    return bestResult;
}

RansacResult JointDetector::leastSquaresLineFit(const std::vector<Point2D>& points) {
    RansacResult result;

    if (points.size() < 2) return result;

    // Compute means
    float sumX = 0, sumZ = 0;
    for (const auto& p : points) {
        sumX += p.x;
        sumZ += p.z;
    }
    float meanX = sumX / points.size();
    float meanZ = sumZ / points.size();

    // Compute slope and intercept
    float numerator = 0, denominator = 0;
    for (const auto& p : points) {
        float dx = p.x - meanX;
        numerator += dx * (p.z - meanZ);
        denominator += dx * dx;
    }

    if (std::abs(denominator) < 0.0001f) {
        result.slope = 0;
    } else {
        result.slope = numerator / denominator;
    }
    result.intercept = meanZ - result.slope * meanX;

    // Compute R-squared
    float ssTot = 0, ssRes = 0;
    for (const auto& p : points) {
        float predicted = result.slope * p.x + result.intercept;
        ssRes += (p.z - predicted) * (p.z - predicted);
        ssTot += (p.z - meanZ) * (p.z - meanZ);
    }

    result.r2 = (ssTot > 0.0001f) ? 1.0f - (ssRes / ssTot) : 0.0f;
    result.valid = true;

    return result;
}

std::optional<Point2D> JointDetector::findLineIntersection(
    const RansacResult& line1,
    const RansacResult& line2) {

    // Solve: slope1 * x + intercept1 = slope2 * x + intercept2
    float slopeDiff = line1.slope - line2.slope;

    if (std::abs(slopeDiff) < 0.001f) {
        return std::nullopt;  // Parallel lines
    }

    float x = (line2.intercept - line1.intercept) / slopeDiff;
    float z = line1.slope * x + line1.intercept;

    return Point2D(x, z);
}

std::vector<Point2D> JointDetector::filterValidPoints(const std::vector<Point2D>& points) {
    std::vector<Point2D> valid;
    valid.reserve(points.size());

    for (const auto& p : points) {
        if (p.valid &&
            p.x >= config_.roiXMin && p.x <= config_.roiXMax &&
            p.z >= config_.roiZMin && p.z <= config_.roiZMax) {
            valid.push_back(p);
        }
    }

    return valid;
}

float JointDetector::computeAngle(float slope) {
    return std::atan(slope) * 180.0f / 3.14159265f;
}

float JointDetector::lineDistance(const Point2D& point, float slope, float intercept) {
    // Distance from point to line: |ax + bz + c| / sqrt(a^2 + b^2)
    // For line z = slope * x + intercept: -slope * x + z - intercept = 0
    float a = -slope;
    float b = 1.0f;
    float c = -intercept;

    return std::abs(a * point.x + b * point.z + c) / std::sqrt(a*a + b*b);
}

} // namespace robot_controller::vision
