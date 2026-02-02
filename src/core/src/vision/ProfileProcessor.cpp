#include "ProfileProcessor.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace robot_controller::vision {

ProfileProcessor::ProfileProcessor() {
    spdlog::debug("ProfileProcessor created");
}

void ProfileProcessor::setConfig(const SeamDetectionConfig& config) {
    config_ = config;
}

SeamFeature ProfileProcessor::processProfile(const LaserProfile& profile) {
    SeamFeature feature;
    feature.timestamp = profile.timestamp;
    feature.frameId = profile.frameId;

    if (profile.points.empty()) {
        return feature;
    }

    // Copy valid points only
    std::vector<Point2D> points;
    points.reserve(profile.points.size());

    for (const auto& p : profile.points) {
        if (p.valid &&
            p.x >= config_.roiXMin && p.x <= config_.roiXMax &&
            p.z >= config_.roiZMin && p.z <= config_.roiZMax) {
            points.push_back(p);
        }
    }

    if (points.size() < 20) {
        spdlog::warn("Too few valid points in profile: {}", points.size());
        return feature;
    }

    // Apply preprocessing
    if (config_.enableMedianFilter) {
        medianFilter(points, config_.medianFilterSize);
    }

    if (config_.enableOutlierRemoval) {
        removeOutliers(points, config_.outlierThreshold);
    }

    // Segment the profile
    auto segments = segmentProfile(points);

    if (segments.size() < 2) {
        return feature;
    }

    // Basic feature extraction (placeholder - will be enhanced in JointDetector)
    feature.valid = true;
    feature.confidence = 0.5f;

    return feature;
}

void ProfileProcessor::medianFilter(std::vector<Point2D>& points, int kernelSize) {
    if (points.size() < static_cast<size_t>(kernelSize)) return;

    int halfKernel = kernelSize / 2;
    std::vector<Point2D> filtered = points;

    for (size_t i = halfKernel; i < points.size() - halfKernel; ++i) {
        std::vector<float> zValues;
        zValues.reserve(kernelSize);

        for (int k = -halfKernel; k <= halfKernel; ++k) {
            if (points[i + k].valid) {
                zValues.push_back(points[i + k].z);
            }
        }

        if (!zValues.empty()) {
            filtered[i].z = computeMedian(zValues);
        }
    }

    points = std::move(filtered);
}

void ProfileProcessor::removeOutliers(std::vector<Point2D>& points, float threshold) {
    if (points.size() < 10) return;

    // Compute mean and std dev of Z values
    std::vector<float> zValues;
    zValues.reserve(points.size());

    for (const auto& p : points) {
        if (p.valid) {
            zValues.push_back(p.z);
        }
    }

    if (zValues.empty()) return;

    float mean = std::accumulate(zValues.begin(), zValues.end(), 0.0f) / zValues.size();
    float stdDev = computeStdDev(zValues, mean);

    if (stdDev < 0.001f) return;  // All values are same

    // Mark outliers as invalid
    for (auto& p : points) {
        if (p.valid) {
            float deviation = std::abs(p.z - mean) / stdDev;
            if (deviation > threshold) {
                p.valid = false;
            }
        }
    }
}

std::vector<Point2D> ProfileProcessor::extractCenterline(
    const std::vector<Point2D>& points,
    float sigma) {

    // Steger algorithm for sub-pixel centerline extraction
    // This is a simplified version; full implementation would use Hessian matrix

    std::vector<Point2D> centerline;
    centerline.reserve(points.size());

    int kernelSize = static_cast<int>(std::ceil(sigma * 3)) * 2 + 1;
    int halfKernel = kernelSize / 2;

    for (size_t i = halfKernel; i < points.size() - halfKernel; ++i) {
        if (!points[i].valid) continue;

        // Compute weighted centroid (simplified Steger)
        float sumWeight = 0.0f;
        float sumWZ = 0.0f;

        for (int k = -halfKernel; k <= halfKernel; ++k) {
            const auto& p = points[i + k];
            if (p.valid) {
                float distance = static_cast<float>(k);
                float weight = std::exp(-(distance * distance) / (2 * sigma * sigma));
                weight *= p.intensity;  // Weight by intensity

                sumWeight += weight;
                sumWZ += weight * p.z;
            }
        }

        if (sumWeight > 0) {
            Point2D center = points[i];
            center.z = sumWZ / sumWeight;
            centerline.push_back(center);
        }
    }

    return centerline;
}

std::vector<ProfileProcessor::ProfileSegment> ProfileProcessor::segmentProfile(
    const std::vector<Point2D>& points) {

    std::vector<ProfileSegment> segments;

    if (points.size() < 10) return segments;

    // Simple segmentation based on gradient changes
    const float slopeThreshold = 0.5f;  // Threshold for detecting slope change
    const int windowSize = 10;

    ProfileSegment currentSegment;
    float lastSlope = 0.0f;
    bool inSegment = false;

    for (size_t i = windowSize; i < points.size() - windowSize; ++i) {
        if (!points[i].valid) continue;

        // Compute local slope
        float dx = points[i + windowSize/2].x - points[i - windowSize/2].x;
        float dz = points[i + windowSize/2].z - points[i - windowSize/2].z;
        float slope = (std::abs(dx) > 0.001f) ? dz / dx : 0.0f;

        if (!inSegment) {
            // Start new segment
            currentSegment = ProfileSegment();
            currentSegment.startX = points[i].x;
            inSegment = true;
        }

        currentSegment.points.push_back(points[i]);

        // Check for segment boundary (significant slope change)
        if (std::abs(slope - lastSlope) > slopeThreshold && currentSegment.points.size() > 5) {
            currentSegment.endX = points[i].x;
            currentSegment.slope = lastSlope;
            currentSegment.isFlat = std::abs(lastSlope) < 0.1f;
            currentSegment.isSlope = !currentSegment.isFlat;

            // Compute average Z
            float sumZ = 0.0f;
            for (const auto& p : currentSegment.points) {
                sumZ += p.z;
            }
            currentSegment.avgZ = sumZ / currentSegment.points.size();

            segments.push_back(currentSegment);
            inSegment = false;
        }

        lastSlope = slope;
    }

    // Add final segment
    if (inSegment && !currentSegment.points.empty()) {
        currentSegment.endX = currentSegment.points.back().x;
        currentSegment.slope = lastSlope;
        currentSegment.isFlat = std::abs(lastSlope) < 0.1f;

        float sumZ = 0.0f;
        for (const auto& p : currentSegment.points) {
            sumZ += p.z;
        }
        currentSegment.avgZ = sumZ / currentSegment.points.size();

        segments.push_back(currentSegment);
    }

    return segments;
}

float ProfileProcessor::computeMedian(std::vector<float>& values) {
    if (values.empty()) return 0.0f;

    size_t n = values.size();
    std::nth_element(values.begin(), values.begin() + n/2, values.end());

    if (n % 2 == 0) {
        float mid1 = values[n/2];
        std::nth_element(values.begin(), values.begin() + n/2 - 1, values.end());
        float mid2 = values[n/2 - 1];
        return (mid1 + mid2) / 2.0f;
    }

    return values[n/2];
}

float ProfileProcessor::computeStdDev(const std::vector<float>& values, float mean) {
    if (values.size() < 2) return 0.0f;

    float sumSq = 0.0f;
    for (float v : values) {
        float diff = v - mean;
        sumSq += diff * diff;
    }

    return std::sqrt(sumSq / (values.size() - 1));
}

} // namespace robot_controller::vision
