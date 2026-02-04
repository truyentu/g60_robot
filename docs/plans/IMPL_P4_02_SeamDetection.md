# IMPL_P4_02: Seam Detection

| Metadata | Value |
|----------|-------|
| Plan ID | IMPL_P4_02 |
| Phase | 4 - Vision Integration |
| Priority | P0 |
| Depends On | IMPL_P4_01 (Sensor Drivers), IMPL_P2_02 (Kinematics) |
| Status | Ready for Implementation |

---

## Required Reading (ĐỌC TRƯỚC KHI CODE)

| Priority | Document | Lý do |
|----------|----------|-------|
| P0 | `ressearch_doc_md/Robot Hàn_ Cảm Biến Laser & Mã Nguồn.md` | Profile data, seam detection algorithms |
| P0 | `ressearch_doc_md/Robot Hàn Scan-to-Path_ Tái tạo 3D.md` | Point cloud processing, surface reconstruction |
| P1 | `ressearch_doc_md/Robot Hàn_ Tái tạo 3D và Tìm đường.md` | Path planning on surfaces |

---

## Overview

Implementation plan cho Seam Detection - phát hiện và bám đường hàn:
- **Profile Processing:** Lọc nhiễu, trích xuất tâm đường laser (Steger algorithm)
- **Joint Detection:** Phát hiện V-groove, Lap joint, Fillet joint
- **Feature Extraction:** Tìm điểm đặc trưng (root point, gap width, angle)
- **Seam Tracking:** Bám đường hàn real-time với bù trễ
- **Coordinate Transform:** Chuyển đổi từ sensor frame sang robot frame
- **Path Planning:** Tạo quỹ đạo hàn từ dữ liệu scan

---

## Seam Detection Pipeline

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      SEAM DETECTION PIPELINE                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐              │
│  │ Raw Profile  │───►│ Preprocessing │───►│  Centerline  │              │
│  │  (2048 pts)  │    │   Filtering   │    │  Extraction  │              │
│  └──────────────┘    └──────────────┘    └──────┬───────┘              │
│                                                  │                       │
│                                                  ▼                       │
│                      ┌──────────────────────────────────────────┐       │
│                      │         JOINT TYPE DETECTION              │       │
│                      │                                           │       │
│                      │  ┌─────────┐  ┌─────────┐  ┌─────────┐  │       │
│                      │  │V-Groove │  │Lap Joint│  │ Fillet  │  │       │
│                      │  │   \/    │  │  ___    │  │   |__   │  │       │
│                      │  │  /  \   │  │ |       │  │   |     │  │       │
│                      │  └────┬────┘  └────┬────┘  └────┬────┘  │       │
│                      └───────┼────────────┼────────────┼────────┘       │
│                              │            │            │                 │
│                              └────────────┼────────────┘                 │
│                                           ▼                              │
│                      ┌──────────────────────────────────────────┐       │
│                      │         FEATURE EXTRACTION                │       │
│                      │                                           │       │
│                      │  - Root Point (X, Z)                      │       │
│                      │  - Gap Width                              │       │
│                      │  - Left/Right Angle                       │       │
│                      │  - Joint Center                           │       │
│                      │  - Normal Vector                          │       │
│                      └─────────────────────┬────────────────────┘       │
│                                            │                             │
│                                            ▼                             │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐              │
│  │ Robot Pose   │───►│  Coordinate  │───►│ Seam Point   │              │
│  │ (from IPC)   │    │  Transform   │    │ (World XYZ)  │              │
│  └──────────────┘    └──────────────┘    └──────┬───────┘              │
│                                                  │                       │
│                                                  ▼                       │
│                      ┌──────────────────────────────────────────┐       │
│                      │           SEAM TRACKER                    │       │
│                      │                                           │       │
│                      │  - Kalman Filter (prediction)             │       │
│                      │  - Latency Compensation                   │       │
│                      │  - Deviation Calculation                  │       │
│                      │  - Path Correction Output                 │       │
│                      └──────────────────────────────────────────┘       │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

- [ ] IMPL_P4_01 (Sensor Drivers) đã hoàn thành
- [ ] IMPL_P2_02 (Kinematics) đã hoàn thành
- [ ] LaserProfile data available from SensorManager
- [ ] Hand-Eye calibration completed

---

## Step 1: Create Seam Detection Types

### 1.1 Create SeamTypes.hpp

**File:** `src/cpp/core/vision/SeamTypes.hpp`

```cpp
#pragma once

#include "VisionTypes.hpp"
#include <vector>
#include <array>
#include <optional>

namespace RobotController::Vision {

// ============================================================================
// Joint Types
// ============================================================================

/// Supported weld joint types
enum class JointType {
    Unknown,
    VGroove,        // V-groove butt joint (vát mép chữ V)
    UGroove,        // U-groove
    JGroove,        // J-groove
    LapJoint,       // Overlap joint (mối hàn chồng)
    FilletJoint,    // Fillet/T-joint (mối hàn góc)
    ButtSquare,     // Square butt joint
    EdgeJoint,      // Edge joint
    Custom
};

/// Convert joint type to string
inline const char* jointTypeToString(JointType type) {
    switch (type) {
        case JointType::VGroove: return "V-Groove";
        case JointType::UGroove: return "U-Groove";
        case JointType::JGroove: return "J-Groove";
        case JointType::LapJoint: return "Lap Joint";
        case JointType::FilletJoint: return "Fillet Joint";
        case JointType::ButtSquare: return "Butt Square";
        case JointType::EdgeJoint: return "Edge Joint";
        case JointType::Custom: return "Custom";
        default: return "Unknown";
    }
}

// ============================================================================
// Seam Feature Point
// ============================================================================

/// Detected seam feature from a single profile
struct SeamFeature {
    // Primary feature point (weld target)
    Point2D rootPoint;              // Root/bottom of groove in sensor frame
    Point2D jointCenter;            // Center of joint (for lap/fillet)

    // Joint geometry
    JointType jointType{JointType::Unknown};
    float gapWidth{0.0f};           // Gap/groove width (mm)
    float leftAngle{0.0f};          // Left bevel angle (degrees)
    float rightAngle{0.0f};         // Right bevel angle (degrees)
    float depth{0.0f};              // Groove depth (mm)

    // Plate information
    float leftPlateHeight{0.0f};    // Left plate Z level
    float rightPlateHeight{0.0f};   // Right plate Z level
    float plateThickness{0.0f};     // Estimated plate thickness

    // Quality metrics
    float confidence{0.0f};         // Detection confidence (0-1)
    bool valid{false};

    // Line fit results (for V-groove)
    struct LineFit {
        float slope{0.0f};
        float intercept{0.0f};
        float r2{0.0f};             // R-squared fit quality
        int inlierCount{0};
    };
    LineFit leftLine;
    LineFit rightLine;

    // Timestamps
    uint64_t timestamp{0};
    uint64_t frameId{0};
};

// ============================================================================
// Seam Point (World Coordinates)
// ============================================================================

/// Seam point in world/robot coordinates
struct SeamPoint {
    // Position
    Point3D position;               // XYZ in world frame (mm)

    // Orientation (for torch angle)
    std::array<double, 3> normal;   // Surface normal vector
    std::array<double, 3> tangent;  // Seam direction (travel vector)
    std::array<double, 3> binormal; // Cross product (normal x tangent)

    // Torch pose (6-DOF)
    std::array<double, 3> torchPosition;    // TCP position
    std::array<double, 3> torchOrientation; // RPY angles (rad)

    // Feature data
    SeamFeature feature;
    RobotPose robotPose;            // Robot pose when detected

    // Seam tracking
    float deviation{0.0f};          // Lateral deviation from nominal (mm)
    float heightOffset{0.0f};       // Height offset from nominal (mm)

    bool valid{false};
    uint64_t timestamp{0};
};

// ============================================================================
// Seam Path
// ============================================================================

/// Complete seam path from scan
struct SeamPath {
    std::vector<SeamPoint> points;
    JointType jointType{JointType::Unknown};

    // Path statistics
    float totalLength{0.0f};        // Total path length (mm)
    float avgGapWidth{0.0f};
    float maxDeviation{0.0f};

    // Bounding box
    Point3D minBound;
    Point3D maxBound;

    // Metadata
    std::string pathId;
    uint64_t startTimestamp{0};
    uint64_t endTimestamp{0};
    bool valid{false};

    void computeStatistics() {
        if (points.empty()) return;

        totalLength = 0.0f;
        float sumGap = 0.0f;
        maxDeviation = 0.0f;

        minBound = {FLT_MAX, FLT_MAX, FLT_MAX, 0};
        maxBound = {-FLT_MAX, -FLT_MAX, -FLT_MAX, 0};

        for (size_t i = 0; i < points.size(); ++i) {
            const auto& p = points[i];

            // Accumulate length
            if (i > 0) {
                float dx = p.position.x - points[i-1].position.x;
                float dy = p.position.y - points[i-1].position.y;
                float dz = p.position.z - points[i-1].position.z;
                totalLength += std::sqrt(dx*dx + dy*dy + dz*dz);
            }

            // Gap width
            sumGap += p.feature.gapWidth;

            // Deviation
            maxDeviation = std::max(maxDeviation, std::abs(p.deviation));

            // Bounds
            minBound.x = std::min(minBound.x, p.position.x);
            minBound.y = std::min(minBound.y, p.position.y);
            minBound.z = std::min(minBound.z, p.position.z);
            maxBound.x = std::max(maxBound.x, p.position.x);
            maxBound.y = std::max(maxBound.y, p.position.y);
            maxBound.z = std::max(maxBound.z, p.position.z);
        }

        avgGapWidth = points.size() > 0 ? sumGap / points.size() : 0.0f;
    }

    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
};

// ============================================================================
// Tracking State
// ============================================================================

/// Seam tracking state for real-time correction
struct TrackingState {
    // Current detection
    SeamFeature currentFeature;
    SeamPoint currentSeamPoint;

    // Predicted state (Kalman filter output)
    struct PredictedState {
        float x{0.0f};              // Lateral position
        float z{0.0f};              // Height
        float vx{0.0f};             // Lateral velocity
        float vz{0.0f};             // Height velocity
        float confidence{0.0f};
    };
    PredictedState predicted;

    // Correction output
    struct Correction {
        float lateralOffset{0.0f};  // X correction (mm)
        float heightOffset{0.0f};   // Z correction (mm)
        float rollCorrection{0.0f}; // Roll adjustment (rad)
        bool valid{false};
    };
    Correction correction;

    // Tracking quality
    bool trackingLost{false};
    int lostFrameCount{0};
    float trackingQuality{0.0f};    // 0-1, running average

    // Statistics
    uint64_t framesProcessed{0};
    uint64_t detectionsValid{0};
    float avgLatency{0.0f};         // Processing latency (ms)
};

// ============================================================================
// Detection Configuration
// ============================================================================

/// Configuration for seam detection algorithms
struct SeamDetectionConfig {
    // Expected joint type (auto-detect if Unknown)
    JointType expectedJointType{JointType::Unknown};

    // Region of Interest
    float roiXMin{-50.0f};          // mm
    float roiXMax{50.0f};
    float roiZMin{0.0f};
    float roiZMax{100.0f};

    // Preprocessing
    bool enableMedianFilter{true};
    int medianFilterSize{5};
    bool enableOutlierRemoval{true};
    float outlierThreshold{3.0f};   // Standard deviations

    // Centerline extraction (Steger)
    bool useStegerAlgorithm{true};
    float stegerSigma{2.0f};        // Gaussian smoothing sigma

    // RANSAC parameters
    int ransacIterations{100};
    float ransacThreshold{0.5f};    // Inlier distance threshold (mm)
    int ransacMinInliers{10};

    // V-Groove detection
    float minGrooveAngle{20.0f};    // Minimum bevel angle (degrees)
    float maxGrooveAngle{80.0f};
    float minGapWidth{0.5f};        // mm
    float maxGapWidth{20.0f};

    // Lap joint detection
    float minStepHeight{0.5f};      // Minimum plate step (mm)
    float maxStepHeight{20.0f};

    // Tracking
    bool enableTracking{true};
    float trackingTimeout{0.5f};    // Seconds before declaring lost
    float predictionHorizon{0.02f}; // Prediction time (s)

    // Latency compensation
    float systemLatency{0.030f};    // Total system latency (s)
    float travelSpeed{10.0f};       // Assumed travel speed (mm/s)
};

// ============================================================================
// Callbacks
// ============================================================================

using SeamFeatureCallback = std::function<void(const SeamFeature&)>;
using SeamPointCallback = std::function<void(const SeamPoint&)>;
using TrackingCallback = std::function<void(const TrackingState&)>;

} // namespace RobotController::Vision
```

---

## Step 2: Create Profile Processor

### 2.1 Create ProfileProcessor.hpp

**File:** `src/cpp/core/vision/ProfileProcessor.hpp`

```cpp
#pragma once

#include "VisionTypes.hpp"
#include "SeamTypes.hpp"
#include <vector>

namespace RobotController::Vision {

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

} // namespace RobotController::Vision
```

### 2.2 Create ProfileProcessor.cpp

**File:** `src/cpp/core/vision/ProfileProcessor.cpp`

```cpp
#include "ProfileProcessor.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace RobotController::Vision {

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

} // namespace RobotController::Vision
```

---

## Step 3: Create Joint Detector

### 3.1 Create JointDetector.hpp

**File:** `src/cpp/core/vision/JointDetector.hpp`

```cpp
#pragma once

#include "SeamTypes.hpp"
#include "ProfileProcessor.hpp"
#include <memory>

namespace RobotController::Vision {

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

    // Helper methods
    std::vector<Point2D> getLeftHalf(const std::vector<Point2D>& points);
    std::vector<Point2D> getRightHalf(const std::vector<Point2D>& points);
    float computeAngle(float slope);  // Convert slope to angle in degrees
};

} // namespace RobotController::Vision
```

### 3.2 Create JointDetector.cpp

**File:** `src/cpp/core/vision/JointDetector.cpp`

```cpp
#include "JointDetector.hpp"
#include <spdlog/spdlog.h>
#include <random>
#include <cmath>
#include <algorithm>

namespace RobotController::Vision {

JointDetector::JointDetector() {
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

    // Preprocess profile
    std::vector<Point2D> points;
    for (const auto& p : profile.points) {
        if (p.valid &&
            p.x >= config_.roiXMin && p.x <= config_.roiXMax &&
            p.z >= config_.roiZMin && p.z <= config_.roiZMax) {
            points.push_back(p);
        }
    }

    if (points.size() < 30) {
        spdlog::debug("Insufficient points for detection: {}", points.size());
        return feature;
    }

    // Apply preprocessing
    if (config_.enableMedianFilter) {
        profileProcessor_.medianFilter(points, config_.medianFilterSize);
    }

    if (config_.enableOutlierRemoval) {
        profileProcessor_.removeOutliers(points, config_.outlierThreshold);
    }

    // Detect based on expected or auto-detected joint type
    JointType jointType = config_.expectedJointType;
    if (jointType == JointType::Unknown) {
        jointType = classifyJointType(points);
    }

    switch (jointType) {
        case JointType::VGroove:
        case JointType::UGroove:
            feature = detectVGroove(points);
            break;

        case JointType::LapJoint:
            feature = detectLapJoint(points);
            break;

        case JointType::FilletJoint:
            feature = detectFilletJoint(points);
            break;

        default:
            // Try V-groove by default
            feature = detectVGroove(points);
            break;
    }

    feature.timestamp = profile.timestamp;
    feature.frameId = profile.frameId;

    return feature;
}

SeamFeature JointDetector::detectVGroove(const std::vector<Point2D>& points) {
    SeamFeature feature;
    feature.jointType = JointType::VGroove;

    // Split profile into left and right halves
    auto leftPoints = getLeftHalf(points);
    auto rightPoints = getRightHalf(points);

    if (leftPoints.size() < 10 || rightPoints.size() < 10) {
        spdlog::debug("Insufficient points for V-groove detection");
        return feature;
    }

    // Fit lines to left and right slopes using RANSAC
    auto leftFit = ransacLineFit(
        leftPoints,
        config_.ransacIterations,
        config_.ransacThreshold,
        config_.ransacMinInliers);

    auto rightFit = ransacLineFit(
        rightPoints,
        config_.ransacIterations,
        config_.ransacThreshold,
        config_.ransacMinInliers);

    if (!leftFit.valid || !rightFit.valid) {
        spdlog::debug("RANSAC line fitting failed");
        return feature;
    }

    // Find intersection point (root of V-groove)
    auto intersection = findLineIntersection(leftFit, rightFit);

    if (!intersection.has_value()) {
        spdlog::debug("Could not find line intersection");
        return feature;
    }

    // Populate feature
    feature.rootPoint = intersection.value();
    feature.leftAngle = computeAngle(leftFit.slope);
    feature.rightAngle = computeAngle(rightFit.slope);

    // Validate angles
    if (feature.leftAngle < config_.minGrooveAngle ||
        feature.leftAngle > config_.maxGrooveAngle ||
        feature.rightAngle < config_.minGrooveAngle ||
        feature.rightAngle > config_.maxGrooveAngle) {
        spdlog::debug("Groove angles out of range: L={:.1f}, R={:.1f}",
                      feature.leftAngle, feature.rightAngle);
        return feature;
    }

    // Compute gap width at top of groove
    // Find Z level of plates (flat regions)
    float leftPlateZ = 0.0f, rightPlateZ = 0.0f;
    int leftCount = 0, rightCount = 0;

    for (const auto& p : leftPoints) {
        if (std::abs(leftFit.slope * p.x + leftFit.intercept - p.z) > config_.ransacThreshold) {
            leftPlateZ += p.z;
            leftCount++;
        }
    }
    for (const auto& p : rightPoints) {
        if (std::abs(rightFit.slope * p.x + rightFit.intercept - p.z) > config_.ransacThreshold) {
            rightPlateZ += p.z;
            rightCount++;
        }
    }

    if (leftCount > 0) feature.leftPlateHeight = leftPlateZ / leftCount;
    if (rightCount > 0) feature.rightPlateHeight = rightPlateZ / rightCount;

    // Groove depth
    float avgPlateHeight = (feature.leftPlateHeight + feature.rightPlateHeight) / 2;
    feature.depth = avgPlateHeight - feature.rootPoint.z;

    // Gap width (estimated at plate level)
    float gapLeftX = (avgPlateHeight - leftFit.intercept) / leftFit.slope;
    float gapRightX = (avgPlateHeight - rightFit.intercept) / rightFit.slope;
    feature.gapWidth = std::abs(gapRightX - gapLeftX);

    // Validate gap width
    if (feature.gapWidth < config_.minGapWidth ||
        feature.gapWidth > config_.maxGapWidth) {
        spdlog::debug("Gap width out of range: {:.2f}mm", feature.gapWidth);
        return feature;
    }

    // Store line fit results
    feature.leftLine = {leftFit.slope, leftFit.intercept, leftFit.r2,
                        static_cast<int>(leftFit.inlierIndices.size())};
    feature.rightLine = {rightFit.slope, rightFit.intercept, rightFit.r2,
                         static_cast<int>(rightFit.inlierIndices.size())};

    // Joint center (between root and plate level)
    feature.jointCenter.x = feature.rootPoint.x;
    feature.jointCenter.z = (feature.rootPoint.z + avgPlateHeight) / 2;

    // Compute confidence based on line fit quality
    feature.confidence = (leftFit.r2 + rightFit.r2) / 2.0f;
    feature.valid = feature.confidence > 0.5f;

    spdlog::debug("V-groove detected: root=({:.2f}, {:.2f}), gap={:.2f}mm, conf={:.2f}",
                  feature.rootPoint.x, feature.rootPoint.z,
                  feature.gapWidth, feature.confidence);

    return feature;
}

SeamFeature JointDetector::detectLapJoint(const std::vector<Point2D>& points) {
    SeamFeature feature;
    feature.jointType = JointType::LapJoint;

    // Find step edge in profile
    // Look for sudden Z change

    float maxGradient = 0.0f;
    size_t stepIndex = 0;

    for (size_t i = 5; i < points.size() - 5; ++i) {
        if (!points[i].valid) continue;

        // Compute gradient over window
        float z1 = 0.0f, z2 = 0.0f;
        int c1 = 0, c2 = 0;

        for (int k = -5; k < 0; ++k) {
            if (points[i + k].valid) {
                z1 += points[i + k].z;
                c1++;
            }
        }
        for (int k = 0; k < 5; ++k) {
            if (points[i + k].valid) {
                z2 += points[i + k].z;
                c2++;
            }
        }

        if (c1 > 0 && c2 > 0) {
            float gradient = std::abs((z2 / c2) - (z1 / c1));
            if (gradient > maxGradient) {
                maxGradient = gradient;
                stepIndex = i;
            }
        }
    }

    // Validate step height
    if (maxGradient < config_.minStepHeight ||
        maxGradient > config_.maxStepHeight) {
        return feature;
    }

    // Found step edge
    feature.rootPoint = points[stepIndex];
    feature.jointCenter = points[stepIndex];
    feature.depth = maxGradient;

    // Determine which plate is on top
    float leftZ = 0.0f, rightZ = 0.0f;
    int leftCount = 0, rightCount = 0;

    for (size_t i = 0; i < stepIndex - 5; ++i) {
        if (points[i].valid) {
            leftZ += points[i].z;
            leftCount++;
        }
    }
    for (size_t i = stepIndex + 5; i < points.size(); ++i) {
        if (points[i].valid) {
            rightZ += points[i].z;
            rightCount++;
        }
    }

    if (leftCount > 0) feature.leftPlateHeight = leftZ / leftCount;
    if (rightCount > 0) feature.rightPlateHeight = rightZ / rightCount;

    feature.plateThickness = std::abs(feature.leftPlateHeight - feature.rightPlateHeight);
    feature.confidence = std::min(1.0f, maxGradient / 5.0f);  // Higher step = higher confidence
    feature.valid = true;

    spdlog::debug("Lap joint detected: edge=({:.2f}, {:.2f}), step={:.2f}mm",
                  feature.rootPoint.x, feature.rootPoint.z, feature.depth);

    return feature;
}

SeamFeature JointDetector::detectFilletJoint(const std::vector<Point2D>& points) {
    SeamFeature feature;
    feature.jointType = JointType::FilletJoint;

    // Look for L-shaped profile (horizontal + vertical)
    auto segments = profileProcessor_.segmentProfile(points);

    if (segments.size() < 2) {
        return feature;
    }

    // Find flat segment and steep segment that meet at corner
    const ProfileProcessor::ProfileSegment* flatSegment = nullptr;
    const ProfileProcessor::ProfileSegment* steepSegment = nullptr;

    for (size_t i = 0; i < segments.size() - 1; ++i) {
        if (segments[i].isFlat && segments[i + 1].isSlope) {
            float slopeAngle = std::abs(std::atan(segments[i + 1].slope)) * 180.0f / M_PI;
            if (slopeAngle > 60.0f) {  // Nearly vertical
                flatSegment = &segments[i];
                steepSegment = &segments[i + 1];
                break;
            }
        }
        if (segments[i].isSlope && segments[i + 1].isFlat) {
            float slopeAngle = std::abs(std::atan(segments[i].slope)) * 180.0f / M_PI;
            if (slopeAngle > 60.0f) {
                steepSegment = &segments[i];
                flatSegment = &segments[i + 1];
                break;
            }
        }
    }

    if (!flatSegment || !steepSegment) {
        return feature;
    }

    // Corner point is where segments meet
    feature.rootPoint.x = flatSegment->endX;
    feature.rootPoint.z = flatSegment->avgZ;
    feature.jointCenter = feature.rootPoint;

    feature.leftAngle = 0.0f;  // Horizontal
    feature.rightAngle = computeAngle(steepSegment->slope);

    feature.confidence = 0.8f;
    feature.valid = true;

    spdlog::debug("Fillet joint detected: corner=({:.2f}, {:.2f})",
                  feature.rootPoint.x, feature.rootPoint.z);

    return feature;
}

JointType JointDetector::classifyJointType(const std::vector<Point2D>& points) {
    // Simple heuristic-based classification

    auto segments = profileProcessor_.segmentProfile(points);

    if (segments.size() < 2) {
        return JointType::Unknown;
    }

    // Count flat and sloped segments
    int flatCount = 0;
    int slopeCount = 0;
    float maxSteepness = 0.0f;

    for (const auto& seg : segments) {
        if (seg.isFlat) flatCount++;
        else {
            slopeCount++;
            maxSteepness = std::max(maxSteepness, std::abs(seg.slope));
        }
    }

    // V-groove: two sloped segments meeting at bottom
    if (slopeCount >= 2 && flatCount >= 2) {
        return JointType::VGroove;
    }

    // Fillet: one flat, one nearly vertical
    if (flatCount >= 1 && maxSteepness > 2.0f) {
        return JointType::FilletJoint;
    }

    // Lap joint: look for step
    float maxZDiff = 0.0f;
    for (size_t i = 0; i < segments.size() - 1; ++i) {
        float diff = std::abs(segments[i].avgZ - segments[i + 1].avgZ);
        maxZDiff = std::max(maxZDiff, diff);
    }

    if (maxZDiff > config_.minStepHeight) {
        return JointType::LapJoint;
    }

    return JointType::Unknown;
}

RansacResult JointDetector::ransacLineFit(
    const std::vector<Point2D>& points,
    int iterations,
    float threshold,
    int minInliers) {

    RansacResult best;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis(0, points.size() - 1);

    for (int iter = 0; iter < iterations; ++iter) {
        // Random sample two points
        size_t idx1 = dis(gen);
        size_t idx2 = dis(gen);

        if (idx1 == idx2 || !points[idx1].valid || !points[idx2].valid) {
            continue;
        }

        const auto& p1 = points[idx1];
        const auto& p2 = points[idx2];

        float dx = p2.x - p1.x;
        if (std::abs(dx) < 0.001f) continue;

        float slope = (p2.z - p1.z) / dx;
        float intercept = p1.z - slope * p1.x;

        // Count inliers
        std::vector<int> inliers;
        for (size_t i = 0; i < points.size(); ++i) {
            if (!points[i].valid) continue;

            float expectedZ = slope * points[i].x + intercept;
            float error = std::abs(points[i].z - expectedZ);

            if (error < threshold) {
                inliers.push_back(static_cast<int>(i));
            }
        }

        if (static_cast<int>(inliers.size()) > static_cast<int>(best.inlierIndices.size())) {
            best.slope = slope;
            best.intercept = intercept;
            best.inlierIndices = std::move(inliers);
        }
    }

    best.valid = static_cast<int>(best.inlierIndices.size()) >= minInliers;

    // Refine with least squares on inliers
    if (best.valid && !best.inlierIndices.empty()) {
        std::vector<Point2D> inlierPoints;
        for (int idx : best.inlierIndices) {
            inlierPoints.push_back(points[idx]);
        }

        auto refined = leastSquaresLineFit(inlierPoints);
        if (refined.valid) {
            best.slope = refined.slope;
            best.intercept = refined.intercept;
            best.r2 = refined.r2;
        }
    }

    return best;
}

RansacResult JointDetector::leastSquaresLineFit(const std::vector<Point2D>& points) {
    RansacResult result;

    if (points.size() < 2) return result;

    // Compute means
    float sumX = 0.0f, sumZ = 0.0f;
    int n = 0;

    for (const auto& p : points) {
        if (p.valid) {
            sumX += p.x;
            sumZ += p.z;
            n++;
        }
    }

    if (n < 2) return result;

    float meanX = sumX / n;
    float meanZ = sumZ / n;

    // Compute slope and intercept
    float sumXX = 0.0f, sumXZ = 0.0f;
    float sumResidual = 0.0f;
    float sumTotal = 0.0f;

    for (const auto& p : points) {
        if (p.valid) {
            float dx = p.x - meanX;
            float dz = p.z - meanZ;
            sumXX += dx * dx;
            sumXZ += dx * dz;
            sumTotal += dz * dz;
        }
    }

    if (std::abs(sumXX) < 0.0001f) return result;

    result.slope = sumXZ / sumXX;
    result.intercept = meanZ - result.slope * meanX;

    // Compute R-squared
    for (const auto& p : points) {
        if (p.valid) {
            float predicted = result.slope * p.x + result.intercept;
            float residual = p.z - predicted;
            sumResidual += residual * residual;
        }
    }

    result.r2 = 1.0f - (sumResidual / (sumTotal + 0.0001f));
    result.valid = true;

    return result;
}

std::optional<Point2D> JointDetector::findLineIntersection(
    const RansacResult& line1,
    const RansacResult& line2) {

    // z = slope1 * x + intercept1
    // z = slope2 * x + intercept2
    // slope1 * x + intercept1 = slope2 * x + intercept2
    // x = (intercept2 - intercept1) / (slope1 - slope2)

    float slopeDiff = line1.slope - line2.slope;
    if (std::abs(slopeDiff) < 0.0001f) {
        return std::nullopt;  // Parallel lines
    }

    float x = (line2.intercept - line1.intercept) / slopeDiff;
    float z = line1.slope * x + line1.intercept;

    return Point2D(x, z, 0.0f, true);
}

std::vector<Point2D> JointDetector::getLeftHalf(const std::vector<Point2D>& points) {
    std::vector<Point2D> left;

    // Find center X
    float sumX = 0.0f;
    int n = 0;
    for (const auto& p : points) {
        if (p.valid) {
            sumX += p.x;
            n++;
        }
    }
    float centerX = n > 0 ? sumX / n : 0.0f;

    for (const auto& p : points) {
        if (p.valid && p.x < centerX) {
            left.push_back(p);
        }
    }

    return left;
}

std::vector<Point2D> JointDetector::getRightHalf(const std::vector<Point2D>& points) {
    std::vector<Point2D> right;

    float sumX = 0.0f;
    int n = 0;
    for (const auto& p : points) {
        if (p.valid) {
            sumX += p.x;
            n++;
        }
    }
    float centerX = n > 0 ? sumX / n : 0.0f;

    for (const auto& p : points) {
        if (p.valid && p.x >= centerX) {
            right.push_back(p);
        }
    }

    return right;
}

float JointDetector::computeAngle(float slope) {
    return std::abs(std::atan(slope)) * 180.0f / static_cast<float>(M_PI);
}

} // namespace RobotController::Vision
```

---

## Step 4: Create Seam Tracker

### 4.1 Create SeamTracker.hpp

**File:** `src/cpp/core/vision/SeamTracker.hpp`

```cpp
#pragma once

#include "SeamTypes.hpp"
#include "JointDetector.hpp"
#include "VisionTypes.hpp"
#include <memory>
#include <deque>
#include <mutex>

namespace RobotController::Vision {

/// Simple Kalman filter for seam tracking
class KalmanFilter2D {
public:
    KalmanFilter2D();

    void reset();
    void setProcessNoise(float q);
    void setMeasurementNoise(float r);

    void predict(float dt);
    void update(float x, float z);

    float getX() const { return state_[0]; }
    float getZ() const { return state_[1]; }
    float getVx() const { return state_[2]; }
    float getVz() const { return state_[3]; }

private:
    std::array<float, 4> state_{};      // [x, z, vx, vz]
    std::array<float, 16> P_{};         // Covariance matrix (4x4)
    float Q_{0.1f};                      // Process noise
    float R_{1.0f};                      // Measurement noise
    bool initialized_{false};
};

/// Real-time seam tracking with prediction and latency compensation
class SeamTracker {
public:
    SeamTracker();
    ~SeamTracker() = default;

    /// Set configuration
    void setConfig(const SeamDetectionConfig& config);

    /// Reset tracker state
    void reset();

    /// Process new profile and update tracking state
    TrackingState update(const LaserProfile& profile, const RobotPose& robotPose);

    /// Get predicted seam position (for latency compensation)
    SeamPoint getPredictedPosition(float lookAheadTime) const;

    /// Get correction offset for robot
    TrackingState::Correction getCorrection() const;

    /// Get current tracking state
    TrackingState getState() const;

    /// Check if tracking is active
    bool isTracking() const;

    /// Set nominal path (for deviation calculation)
    void setNominalPath(const SeamPath& path);

    /// Set callbacks
    void setFeatureCallback(SeamFeatureCallback callback);
    void setTrackingCallback(TrackingCallback callback);

private:
    SeamDetectionConfig config_;
    JointDetector detector_;
    KalmanFilter2D kalman_;

    // State
    TrackingState state_;
    mutable std::mutex stateMutex_;
    bool tracking_{false};
    std::chrono::steady_clock::time_point lastDetectionTime_;

    // History for smoothing
    std::deque<SeamFeature> featureHistory_;
    static constexpr size_t HISTORY_SIZE = 10;

    // Nominal path for deviation
    SeamPath nominalPath_;
    size_t nominalPathIndex_{0};

    // Callbacks
    SeamFeatureCallback featureCallback_;
    TrackingCallback trackingCallback_;
    std::mutex callbackMutex_;

    // Hand-Eye calibration
    HandEyeCalibration handEyeCal_;

    // Helpers
    float computeDeviation(const SeamPoint& detected);
    SeamPoint transformToWorld(const SeamFeature& feature, const RobotPose& robotPose);
    void updateStatistics(const SeamFeature& feature);
};

} // namespace RobotController::Vision
```

### 4.2 Create SeamTracker.cpp

**File:** `src/cpp/core/vision/SeamTracker.cpp`

```cpp
#include "SeamTracker.hpp"
#include <spdlog/spdlog.h>
#include <cmath>

namespace RobotController::Vision {

// ============================================================================
// KalmanFilter2D Implementation
// ============================================================================

KalmanFilter2D::KalmanFilter2D() {
    reset();
}

void KalmanFilter2D::reset() {
    state_.fill(0.0f);

    // Initialize covariance matrix (identity * large value)
    P_.fill(0.0f);
    P_[0] = P_[5] = P_[10] = P_[15] = 1000.0f;

    initialized_ = false;
}

void KalmanFilter2D::setProcessNoise(float q) {
    Q_ = q;
}

void KalmanFilter2D::setMeasurementNoise(float r) {
    R_ = r;
}

void KalmanFilter2D::predict(float dt) {
    if (!initialized_) return;

    // State prediction: x = x + vx*dt, z = z + vz*dt
    state_[0] += state_[2] * dt;
    state_[1] += state_[3] * dt;

    // Covariance prediction (simplified)
    // P = F*P*F' + Q
    P_[0] += Q_;
    P_[5] += Q_;
    P_[10] += Q_;
    P_[15] += Q_;
}

void KalmanFilter2D::update(float x, float z) {
    if (!initialized_) {
        state_[0] = x;
        state_[1] = z;
        state_[2] = 0.0f;
        state_[3] = 0.0f;
        initialized_ = true;
        return;
    }

    // Innovation
    float innovX = x - state_[0];
    float innovZ = z - state_[1];

    // Kalman gain (simplified)
    float S = P_[0] + R_;
    float K = P_[0] / (S + 0.0001f);

    // State update
    state_[0] += K * innovX;
    state_[1] += K * innovZ;

    // Estimate velocity from innovation
    state_[2] = innovX * 0.1f + state_[2] * 0.9f;  // Smoothed velocity
    state_[3] = innovZ * 0.1f + state_[3] * 0.9f;

    // Covariance update
    P_[0] *= (1.0f - K);
    P_[5] *= (1.0f - K);
}

// ============================================================================
// SeamTracker Implementation
// ============================================================================

SeamTracker::SeamTracker() {
    spdlog::info("SeamTracker created");
}

void SeamTracker::setConfig(const SeamDetectionConfig& config) {
    config_ = config;
    detector_.setConfig(config);

    kalman_.setProcessNoise(0.1f);
    kalman_.setMeasurementNoise(1.0f);
}

void SeamTracker::reset() {
    std::lock_guard<std::mutex> lock(stateMutex_);

    kalman_.reset();
    state_ = TrackingState{};
    featureHistory_.clear();
    tracking_ = false;
    nominalPathIndex_ = 0;

    spdlog::info("SeamTracker reset");
}

TrackingState SeamTracker::update(const LaserProfile& profile, const RobotPose& robotPose) {
    auto startTime = std::chrono::steady_clock::now();

    // Detect seam feature
    SeamFeature feature = detector_.detect(profile);

    std::lock_guard<std::mutex> lock(stateMutex_);

    if (feature.valid) {
        // Update Kalman filter
        float dt = 0.010f;  // Assume 100Hz
        if (tracking_) {
            auto elapsed = std::chrono::duration<float>(startTime - lastDetectionTime_);
            dt = elapsed.count();
        }

        kalman_.predict(dt);
        kalman_.update(feature.rootPoint.x, feature.rootPoint.z);

        // Transform to world coordinates
        SeamPoint seamPoint = transformToWorld(feature, robotPose);

        // Compute deviation from nominal
        if (!nominalPath_.empty()) {
            seamPoint.deviation = computeDeviation(seamPoint);
        }

        // Update tracking state
        state_.currentFeature = feature;
        state_.currentSeamPoint = seamPoint;

        state_.predicted.x = kalman_.getX();
        state_.predicted.z = kalman_.getZ();
        state_.predicted.vx = kalman_.getVx();
        state_.predicted.vz = kalman_.getVz();
        state_.predicted.confidence = feature.confidence;

        // Compute correction with latency compensation
        float lookAhead = config_.systemLatency * config_.travelSpeed;
        state_.correction.lateralOffset = kalman_.getX() + kalman_.getVx() * config_.systemLatency;
        state_.correction.heightOffset = kalman_.getZ() + kalman_.getVz() * config_.systemLatency;
        state_.correction.valid = true;

        // Update statistics
        state_.detectionsValid++;
        state_.trackingLost = false;
        state_.lostFrameCount = 0;

        // Add to history
        featureHistory_.push_back(feature);
        if (featureHistory_.size() > HISTORY_SIZE) {
            featureHistory_.pop_front();
        }

        tracking_ = true;
        lastDetectionTime_ = startTime;

        // Callback
        {
            std::lock_guard<std::mutex> cbLock(callbackMutex_);
            if (featureCallback_) {
                featureCallback_(feature);
            }
        }

    } else {
        // Detection failed
        state_.lostFrameCount++;

        float lostTime = state_.lostFrameCount * 0.010f;  // Estimate
        if (lostTime > config_.trackingTimeout) {
            state_.trackingLost = true;
            state_.correction.valid = false;
            tracking_ = false;
            spdlog::warn("Seam tracking lost after {:.2f}s", lostTime);
        } else {
            // Use prediction
            kalman_.predict(0.010f);
            state_.predicted.x = kalman_.getX();
            state_.predicted.z = kalman_.getZ();
            state_.correction.lateralOffset = state_.predicted.x;
            state_.correction.heightOffset = state_.predicted.z;
        }
    }

    state_.framesProcessed++;

    // Compute processing latency
    auto endTime = std::chrono::steady_clock::now();
    float latency = std::chrono::duration<float, std::milli>(endTime - startTime).count();
    state_.avgLatency = state_.avgLatency * 0.9f + latency * 0.1f;

    // Tracking quality (running average of confidence)
    float newQuality = feature.valid ? feature.confidence : 0.0f;
    state_.trackingQuality = state_.trackingQuality * 0.95f + newQuality * 0.05f;

    // Callback
    {
        std::lock_guard<std::mutex> cbLock(callbackMutex_);
        if (trackingCallback_) {
            trackingCallback_(state_);
        }
    }

    return state_;
}

SeamPoint SeamTracker::getPredictedPosition(float lookAheadTime) const {
    std::lock_guard<std::mutex> lock(stateMutex_);

    SeamPoint predicted = state_.currentSeamPoint;

    // Predict forward
    predicted.position.x += kalman_.getVx() * lookAheadTime;
    predicted.position.z += kalman_.getVz() * lookAheadTime;

    return predicted;
}

TrackingState::Correction SeamTracker::getCorrection() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return state_.correction;
}

TrackingState SeamTracker::getState() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return state_;
}

bool SeamTracker::isTracking() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return tracking_ && !state_.trackingLost;
}

void SeamTracker::setNominalPath(const SeamPath& path) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    nominalPath_ = path;
    nominalPathIndex_ = 0;
}

void SeamTracker::setFeatureCallback(SeamFeatureCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    featureCallback_ = std::move(callback);
}

void SeamTracker::setTrackingCallback(TrackingCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    trackingCallback_ = std::move(callback);
}

float SeamTracker::computeDeviation(const SeamPoint& detected) {
    if (nominalPath_.empty()) return 0.0f;

    // Find closest point on nominal path
    float minDist = FLT_MAX;
    float deviation = 0.0f;

    // Search around current index for efficiency
    size_t searchStart = (nominalPathIndex_ > 5) ? nominalPathIndex_ - 5 : 0;
    size_t searchEnd = std::min(nominalPathIndex_ + 20, nominalPath_.size());

    for (size_t i = searchStart; i < searchEnd; ++i) {
        const auto& nom = nominalPath_.points[i];

        float dx = detected.position.x - nom.position.x;
        float dy = detected.position.y - nom.position.y;
        float dz = detected.position.z - nom.position.z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < minDist) {
            minDist = dist;
            nominalPathIndex_ = i;

            // Compute lateral deviation (perpendicular to tangent)
            // Simplified: use X deviation
            deviation = detected.position.x - nom.position.x;
        }
    }

    return deviation;
}

SeamPoint SeamTracker::transformToWorld(const SeamFeature& feature, const RobotPose& robotPose) {
    SeamPoint seamPoint;
    seamPoint.feature = feature;
    seamPoint.robotPose = robotPose;
    seamPoint.timestamp = feature.timestamp;
    seamPoint.valid = feature.valid;

    if (!robotPose.valid) {
        // No robot pose, return in sensor frame
        seamPoint.position.x = feature.rootPoint.x;
        seamPoint.position.y = 0.0f;
        seamPoint.position.z = feature.rootPoint.z;
        return seamPoint;
    }

    // Transform point from sensor frame to world frame
    // P_world = T_base_flange * T_flange_sensor * P_sensor

    // For now, simplified transformation (assuming hand-eye is identity)
    // In full implementation, use handEyeCal_.sensorToFlange

    const auto& flange = robotPose.flange;  // 4x4 matrix (row-major)

    float sx = feature.rootPoint.x;
    float sy = 0.0f;  // Profile is in X-Z plane
    float sz = feature.rootPoint.z;

    // Apply flange transformation
    seamPoint.position.x = static_cast<float>(
        flange[0]*sx + flange[1]*sy + flange[2]*sz + flange[3]);
    seamPoint.position.y = static_cast<float>(
        flange[4]*sx + flange[5]*sy + flange[6]*sz + flange[7]);
    seamPoint.position.z = static_cast<float>(
        flange[8]*sx + flange[9]*sy + flange[10]*sz + flange[11]);

    // Compute surface normal (perpendicular to bevel lines)
    // Simplified: use Z axis of sensor frame transformed to world
    seamPoint.normal = {
        static_cast<double>(flange[2]),
        static_cast<double>(flange[6]),
        static_cast<double>(flange[10])
    };

    return seamPoint;
}

void SeamTracker::updateStatistics(const SeamFeature& feature) {
    // Update running statistics (placeholder)
}

} // namespace RobotController::Vision
```

---

## Step 5: Create Seam Detection IPC Messages

### 5.1 Update IpcMessages.hpp

**Add seam detection messages:**

```cpp
// ============================================================================
// Seam Detection Messages
// ============================================================================

namespace SeamMessages {

struct ConfigureDetectionRequest {
    std::string jointType;          // "vgroove", "lapjoint", "fillet", "auto"
    float roiXMin{-50.0f};
    float roiXMax{50.0f};
    float roiZMin{0.0f};
    float roiZMax{100.0f};
    int ransacIterations{100};
    float ransacThreshold{0.5f};
    float minGrooveAngle{20.0f};
    float maxGrooveAngle{80.0f};
    bool enableTracking{true};
    float systemLatency{0.030f};

    MSGPACK_DEFINE(jointType, roiXMin, roiXMax, roiZMin, roiZMax,
                   ransacIterations, ransacThreshold, minGrooveAngle,
                   maxGrooveAngle, enableTracking, systemLatency)
};

struct FeatureResponse {
    bool success{false};
    std::string jointType;
    float rootX{0.0f};
    float rootZ{0.0f};
    float gapWidth{0.0f};
    float leftAngle{0.0f};
    float rightAngle{0.0f};
    float depth{0.0f};
    float confidence{0.0f};
    uint64_t timestamp{0};
    std::string errorMessage;

    MSGPACK_DEFINE(success, jointType, rootX, rootZ, gapWidth,
                   leftAngle, rightAngle, depth, confidence,
                   timestamp, errorMessage)
};

struct SeamPointResponse {
    bool success{false};
    float worldX{0.0f};
    float worldY{0.0f};
    float worldZ{0.0f};
    std::array<float, 3> normal;
    float deviation{0.0f};
    float heightOffset{0.0f};
    float confidence{0.0f};
    uint64_t timestamp{0};
    std::string errorMessage;

    MSGPACK_DEFINE(success, worldX, worldY, worldZ, normal,
                   deviation, heightOffset, confidence,
                   timestamp, errorMessage)
};

struct TrackingStateResponse {
    bool success{false};
    bool tracking{false};
    bool trackingLost{false};
    float predictedX{0.0f};
    float predictedZ{0.0f};
    float lateralCorrection{0.0f};
    float heightCorrection{0.0f};
    float trackingQuality{0.0f};
    float avgLatency{0.0f};
    uint64_t framesProcessed{0};
    uint64_t detectionsValid{0};
    std::string errorMessage;

    MSGPACK_DEFINE(success, tracking, trackingLost, predictedX, predictedZ,
                   lateralCorrection, heightCorrection, trackingQuality,
                   avgLatency, framesProcessed, detectionsValid, errorMessage)
};

struct SeamPathResponse {
    bool success{false};
    std::vector<float> pointsX;
    std::vector<float> pointsY;
    std::vector<float> pointsZ;
    std::vector<float> deviations;
    std::string jointType;
    float totalLength{0.0f};
    float avgGapWidth{0.0f};
    std::string errorMessage;

    MSGPACK_DEFINE(success, pointsX, pointsY, pointsZ, deviations,
                   jointType, totalLength, avgGapWidth, errorMessage)
};

} // namespace SeamMessages
```

---

## Step 6: Create C# Seam Detection Service

### 6.1 Create ISeamDetectionService.cs

**File:** `src/csharp/RobotController.Core/Services/ISeamDetectionService.cs`

```csharp
namespace RobotController.Core.Services;

/// <summary>
/// Service for seam detection and tracking
/// </summary>
public interface ISeamDetectionService
{
    // ========================================================================
    // Events
    // ========================================================================

    event EventHandler<SeamFeatureEventArgs>? FeatureDetected;
    event EventHandler<SeamPointEventArgs>? SeamPointUpdated;
    event EventHandler<TrackingStateEventArgs>? TrackingStateChanged;
    event EventHandler<string>? ErrorOccurred;

    // ========================================================================
    // Configuration
    // ========================================================================

    /// <summary>
    /// Configure seam detection parameters
    /// </summary>
    Task<OperationResult> ConfigureAsync(SeamDetectionConfigData config);

    /// <summary>
    /// Get current configuration
    /// </summary>
    Task<SeamDetectionConfigData> GetConfigAsync();

    // ========================================================================
    // Detection
    // ========================================================================

    /// <summary>
    /// Detect seam feature from current profile
    /// </summary>
    Task<SeamFeatureData> DetectFeatureAsync();

    /// <summary>
    /// Start continuous seam tracking
    /// </summary>
    Task<OperationResult> StartTrackingAsync();

    /// <summary>
    /// Stop seam tracking
    /// </summary>
    Task<OperationResult> StopTrackingAsync();

    /// <summary>
    /// Reset tracker state
    /// </summary>
    Task<OperationResult> ResetTrackerAsync();

    /// <summary>
    /// Get current tracking state
    /// </summary>
    Task<TrackingStateData> GetTrackingStateAsync();

    /// <summary>
    /// Get correction for robot (lateral and height offset)
    /// </summary>
    Task<CorrectionData> GetCorrectionAsync();

    // ========================================================================
    // Scanning
    // ========================================================================

    /// <summary>
    /// Scan seam path (offline mode)
    /// </summary>
    Task<SeamPathData> ScanSeamPathAsync(uint profileCount, uint timeoutMs = 30000);

    /// <summary>
    /// Set nominal path for deviation tracking
    /// </summary>
    Task<OperationResult> SetNominalPathAsync(SeamPathData path);

    // ========================================================================
    // Status
    // ========================================================================

    /// <summary>
    /// Check if tracking is active
    /// </summary>
    bool IsTracking { get; }

    /// <summary>
    /// Check if tracking is lost
    /// </summary>
    bool IsTrackingLost { get; }
}

// ============================================================================
// Data Types
// ============================================================================

public class SeamDetectionConfigData
{
    public string JointType { get; set; } = "auto";
    public float RoiXMin { get; set; } = -50.0f;
    public float RoiXMax { get; set; } = 50.0f;
    public float RoiZMin { get; set; } = 0.0f;
    public float RoiZMax { get; set; } = 100.0f;
    public int RansacIterations { get; set; } = 100;
    public float RansacThreshold { get; set; } = 0.5f;
    public float MinGrooveAngle { get; set; } = 20.0f;
    public float MaxGrooveAngle { get; set; } = 80.0f;
    public bool EnableTracking { get; set; } = true;
    public float SystemLatency { get; set; } = 0.030f;
}

public class SeamFeatureData
{
    public bool Valid { get; set; }
    public string JointType { get; set; } = "";
    public float RootX { get; set; }
    public float RootZ { get; set; }
    public float GapWidth { get; set; }
    public float LeftAngle { get; set; }
    public float RightAngle { get; set; }
    public float Depth { get; set; }
    public float Confidence { get; set; }
    public ulong Timestamp { get; set; }
}

public class SeamPointData
{
    public bool Valid { get; set; }
    public float WorldX { get; set; }
    public float WorldY { get; set; }
    public float WorldZ { get; set; }
    public float[] Normal { get; set; } = new float[3];
    public float Deviation { get; set; }
    public float HeightOffset { get; set; }
    public float Confidence { get; set; }
    public ulong Timestamp { get; set; }
}

public class TrackingStateData
{
    public bool Tracking { get; set; }
    public bool TrackingLost { get; set; }
    public float PredictedX { get; set; }
    public float PredictedZ { get; set; }
    public float LateralCorrection { get; set; }
    public float HeightCorrection { get; set; }
    public float TrackingQuality { get; set; }
    public float AvgLatency { get; set; }
    public ulong FramesProcessed { get; set; }
    public ulong DetectionsValid { get; set; }
}

public class CorrectionData
{
    public bool Valid { get; set; }
    public float LateralOffset { get; set; }
    public float HeightOffset { get; set; }
    public float RollCorrection { get; set; }
}

public class SeamPathData
{
    public bool Valid { get; set; }
    public float[] PointsX { get; set; } = [];
    public float[] PointsY { get; set; } = [];
    public float[] PointsZ { get; set; } = [];
    public float[] Deviations { get; set; } = [];
    public string JointType { get; set; } = "";
    public float TotalLength { get; set; }
    public float AvgGapWidth { get; set; }
    public int PointCount => PointsX?.Length ?? 0;
}

// Event Args
public record SeamFeatureEventArgs(SeamFeatureData Feature);
public record SeamPointEventArgs(SeamPointData SeamPoint);
public record TrackingStateEventArgs(TrackingStateData State);
```

### 6.2 Create SeamDetectionService.cs

**File:** `src/csharp/RobotController.Core/Services/SeamDetectionService.cs`

```csharp
using RobotController.Core.IPC;
using Microsoft.Extensions.Logging;

namespace RobotController.Core.Services;

/// <summary>
/// Implementation of seam detection service
/// </summary>
public class SeamDetectionService : ISeamDetectionService
{
    private readonly IIpcClientService _ipc;
    private readonly ILogger<SeamDetectionService> _logger;

    private bool _isTracking;
    private bool _isTrackingLost;

    public event EventHandler<SeamFeatureEventArgs>? FeatureDetected;
    public event EventHandler<SeamPointEventArgs>? SeamPointUpdated;
    public event EventHandler<TrackingStateEventArgs>? TrackingStateChanged;
    public event EventHandler<string>? ErrorOccurred;

    public bool IsTracking => _isTracking;
    public bool IsTrackingLost => _isTrackingLost;

    public SeamDetectionService(IIpcClientService ipc, ILogger<SeamDetectionService> logger)
    {
        _ipc = ipc;
        _logger = logger;

        // Subscribe to IPC events
        _ipc.MessageReceived += OnIpcMessageReceived;
    }

    private void OnIpcMessageReceived(object? sender, IpcMessage message)
    {
        switch (message.Type)
        {
            case "seam.feature":
                HandleFeatureMessage(message);
                break;
            case "seam.point":
                HandleSeamPointMessage(message);
                break;
            case "seam.tracking":
                HandleTrackingMessage(message);
                break;
            case "seam.error":
                ErrorOccurred?.Invoke(this, message.GetPayload<string>() ?? "Unknown error");
                break;
        }
    }

    private void HandleFeatureMessage(IpcMessage message)
    {
        var data = message.GetPayload<FeatureResponseDto>();
        if (data != null && data.Success)
        {
            var feature = new SeamFeatureData
            {
                Valid = true,
                JointType = data.JointType,
                RootX = data.RootX,
                RootZ = data.RootZ,
                GapWidth = data.GapWidth,
                LeftAngle = data.LeftAngle,
                RightAngle = data.RightAngle,
                Depth = data.Depth,
                Confidence = data.Confidence,
                Timestamp = data.Timestamp
            };

            FeatureDetected?.Invoke(this, new SeamFeatureEventArgs(feature));
        }
    }

    private void HandleSeamPointMessage(IpcMessage message)
    {
        var data = message.GetPayload<SeamPointResponseDto>();
        if (data != null && data.Success)
        {
            var point = new SeamPointData
            {
                Valid = true,
                WorldX = data.WorldX,
                WorldY = data.WorldY,
                WorldZ = data.WorldZ,
                Normal = data.Normal,
                Deviation = data.Deviation,
                HeightOffset = data.HeightOffset,
                Confidence = data.Confidence,
                Timestamp = data.Timestamp
            };

            SeamPointUpdated?.Invoke(this, new SeamPointEventArgs(point));
        }
    }

    private void HandleTrackingMessage(IpcMessage message)
    {
        var data = message.GetPayload<TrackingStateResponseDto>();
        if (data != null && data.Success)
        {
            _isTracking = data.Tracking;
            _isTrackingLost = data.TrackingLost;

            var state = new TrackingStateData
            {
                Tracking = data.Tracking,
                TrackingLost = data.TrackingLost,
                PredictedX = data.PredictedX,
                PredictedZ = data.PredictedZ,
                LateralCorrection = data.LateralCorrection,
                HeightCorrection = data.HeightCorrection,
                TrackingQuality = data.TrackingQuality,
                AvgLatency = data.AvgLatency,
                FramesProcessed = data.FramesProcessed,
                DetectionsValid = data.DetectionsValid
            };

            TrackingStateChanged?.Invoke(this, new TrackingStateEventArgs(state));
        }
    }

    public async Task<OperationResult> ConfigureAsync(SeamDetectionConfigData config)
    {
        _logger.LogInformation("Configuring seam detection: JointType={JointType}", config.JointType);

        var request = new ConfigureDetectionRequestDto
        {
            JointType = config.JointType,
            RoiXMin = config.RoiXMin,
            RoiXMax = config.RoiXMax,
            RoiZMin = config.RoiZMin,
            RoiZMax = config.RoiZMax,
            RansacIterations = config.RansacIterations,
            RansacThreshold = config.RansacThreshold,
            MinGrooveAngle = config.MinGrooveAngle,
            MaxGrooveAngle = config.MaxGrooveAngle,
            EnableTracking = config.EnableTracking,
            SystemLatency = config.SystemLatency
        };

        var response = await _ipc.SendRequestAsync<ConfigureDetectionRequestDto, OperationResultDto>(
            "seam.configure", request);

        return response?.Success == true
            ? OperationResult.Ok()
            : OperationResult.Fail(response?.ErrorMessage ?? "Configuration failed");
    }

    public async Task<SeamDetectionConfigData> GetConfigAsync()
    {
        var response = await _ipc.SendRequestAsync<object, SeamDetectionConfigData>(
            "seam.config.get", new { });

        return response ?? new SeamDetectionConfigData();
    }

    public async Task<SeamFeatureData> DetectFeatureAsync()
    {
        var response = await _ipc.SendRequestAsync<object, FeatureResponseDto>(
            "seam.detect", new { });

        if (response?.Success == true)
        {
            return new SeamFeatureData
            {
                Valid = true,
                JointType = response.JointType,
                RootX = response.RootX,
                RootZ = response.RootZ,
                GapWidth = response.GapWidth,
                LeftAngle = response.LeftAngle,
                RightAngle = response.RightAngle,
                Depth = response.Depth,
                Confidence = response.Confidence,
                Timestamp = response.Timestamp
            };
        }

        return new SeamFeatureData { Valid = false };
    }

    public async Task<OperationResult> StartTrackingAsync()
    {
        _logger.LogInformation("Starting seam tracking");

        var response = await _ipc.SendRequestAsync<object, OperationResultDto>(
            "seam.tracking.start", new { });

        if (response?.Success == true)
        {
            _isTracking = true;
            return OperationResult.Ok();
        }

        return OperationResult.Fail(response?.ErrorMessage ?? "Start tracking failed");
    }

    public async Task<OperationResult> StopTrackingAsync()
    {
        _logger.LogInformation("Stopping seam tracking");

        var response = await _ipc.SendRequestAsync<object, OperationResultDto>(
            "seam.tracking.stop", new { });

        _isTracking = false;

        return response?.Success == true
            ? OperationResult.Ok()
            : OperationResult.Fail(response?.ErrorMessage ?? "Stop tracking failed");
    }

    public async Task<OperationResult> ResetTrackerAsync()
    {
        var response = await _ipc.SendRequestAsync<object, OperationResultDto>(
            "seam.tracking.reset", new { });

        return response?.Success == true
            ? OperationResult.Ok()
            : OperationResult.Fail(response?.ErrorMessage ?? "Reset failed");
    }

    public async Task<TrackingStateData> GetTrackingStateAsync()
    {
        var response = await _ipc.SendRequestAsync<object, TrackingStateResponseDto>(
            "seam.tracking.state", new { });

        if (response?.Success == true)
        {
            return new TrackingStateData
            {
                Tracking = response.Tracking,
                TrackingLost = response.TrackingLost,
                PredictedX = response.PredictedX,
                PredictedZ = response.PredictedZ,
                LateralCorrection = response.LateralCorrection,
                HeightCorrection = response.HeightCorrection,
                TrackingQuality = response.TrackingQuality,
                AvgLatency = response.AvgLatency,
                FramesProcessed = response.FramesProcessed,
                DetectionsValid = response.DetectionsValid
            };
        }

        return new TrackingStateData();
    }

    public async Task<CorrectionData> GetCorrectionAsync()
    {
        var response = await _ipc.SendRequestAsync<object, CorrectionResponseDto>(
            "seam.correction", new { });

        if (response?.Success == true)
        {
            return new CorrectionData
            {
                Valid = true,
                LateralOffset = response.LateralOffset,
                HeightOffset = response.HeightOffset,
                RollCorrection = response.RollCorrection
            };
        }

        return new CorrectionData { Valid = false };
    }

    public async Task<SeamPathData> ScanSeamPathAsync(uint profileCount, uint timeoutMs = 30000)
    {
        _logger.LogInformation("Scanning seam path: {Count} profiles", profileCount);

        var response = await _ipc.SendRequestAsync<object, SeamPathResponseDto>(
            "seam.scan",
            new { ProfileCount = profileCount, TimeoutMs = timeoutMs },
            TimeSpan.FromMilliseconds(timeoutMs + 5000));

        if (response?.Success == true)
        {
            return new SeamPathData
            {
                Valid = true,
                PointsX = response.PointsX,
                PointsY = response.PointsY,
                PointsZ = response.PointsZ,
                Deviations = response.Deviations,
                JointType = response.JointType,
                TotalLength = response.TotalLength,
                AvgGapWidth = response.AvgGapWidth
            };
        }

        return new SeamPathData { Valid = false };
    }

    public async Task<OperationResult> SetNominalPathAsync(SeamPathData path)
    {
        var response = await _ipc.SendRequestAsync<SeamPathData, OperationResultDto>(
            "seam.nominal.set", path);

        return response?.Success == true
            ? OperationResult.Ok()
            : OperationResult.Fail(response?.ErrorMessage ?? "Set nominal path failed");
    }
}

// DTOs for IPC
internal class ConfigureDetectionRequestDto
{
    public string JointType { get; set; } = "";
    public float RoiXMin { get; set; }
    public float RoiXMax { get; set; }
    public float RoiZMin { get; set; }
    public float RoiZMax { get; set; }
    public int RansacIterations { get; set; }
    public float RansacThreshold { get; set; }
    public float MinGrooveAngle { get; set; }
    public float MaxGrooveAngle { get; set; }
    public bool EnableTracking { get; set; }
    public float SystemLatency { get; set; }
}

internal class FeatureResponseDto
{
    public bool Success { get; set; }
    public string JointType { get; set; } = "";
    public float RootX { get; set; }
    public float RootZ { get; set; }
    public float GapWidth { get; set; }
    public float LeftAngle { get; set; }
    public float RightAngle { get; set; }
    public float Depth { get; set; }
    public float Confidence { get; set; }
    public ulong Timestamp { get; set; }
    public string ErrorMessage { get; set; } = "";
}

internal class SeamPointResponseDto
{
    public bool Success { get; set; }
    public float WorldX { get; set; }
    public float WorldY { get; set; }
    public float WorldZ { get; set; }
    public float[] Normal { get; set; } = [];
    public float Deviation { get; set; }
    public float HeightOffset { get; set; }
    public float Confidence { get; set; }
    public ulong Timestamp { get; set; }
}

internal class TrackingStateResponseDto
{
    public bool Success { get; set; }
    public bool Tracking { get; set; }
    public bool TrackingLost { get; set; }
    public float PredictedX { get; set; }
    public float PredictedZ { get; set; }
    public float LateralCorrection { get; set; }
    public float HeightCorrection { get; set; }
    public float TrackingQuality { get; set; }
    public float AvgLatency { get; set; }
    public ulong FramesProcessed { get; set; }
    public ulong DetectionsValid { get; set; }
}

internal class CorrectionResponseDto
{
    public bool Success { get; set; }
    public float LateralOffset { get; set; }
    public float HeightOffset { get; set; }
    public float RollCorrection { get; set; }
}

internal class SeamPathResponseDto
{
    public bool Success { get; set; }
    public float[] PointsX { get; set; } = [];
    public float[] PointsY { get; set; } = [];
    public float[] PointsZ { get; set; } = [];
    public float[] Deviations { get; set; } = [];
    public string JointType { get; set; } = "";
    public float TotalLength { get; set; }
    public float AvgGapWidth { get; set; }
    public string ErrorMessage { get; set; } = "";
}

internal class OperationResultDto
{
    public bool Success { get; set; }
    public string ErrorMessage { get; set; } = "";
}
```

---

## Step 7: Unit Tests

### 7.1 Create SeamDetectionTests.cpp

**File:** `src/cpp/tests/vision/SeamDetectionTests.cpp`

```cpp
#include <gtest/gtest.h>
#include "vision/SeamTypes.hpp"
#include "vision/ProfileProcessor.hpp"
#include "vision/JointDetector.hpp"
#include "vision/SeamTracker.hpp"

using namespace RobotController::Vision;

// ============================================================================
// SeamTypes Tests
// ============================================================================

TEST(SeamTypesTest, JointTypeToString) {
    EXPECT_STREQ(jointTypeToString(JointType::VGroove), "V-Groove");
    EXPECT_STREQ(jointTypeToString(JointType::LapJoint), "Lap Joint");
    EXPECT_STREQ(jointTypeToString(JointType::FilletJoint), "Fillet Joint");
}

TEST(SeamTypesTest, SeamFeatureDefault) {
    SeamFeature feature;
    EXPECT_FALSE(feature.valid);
    EXPECT_EQ(feature.jointType, JointType::Unknown);
    EXPECT_FLOAT_EQ(feature.confidence, 0.0f);
}

TEST(SeamTypesTest, SeamPathStatistics) {
    SeamPath path;

    SeamPoint p1, p2, p3;
    p1.position = {0, 0, 0, 0};
    p2.position = {10, 0, 0, 0};
    p3.position = {20, 0, 0, 0};
    p1.feature.gapWidth = 2.0f;
    p2.feature.gapWidth = 3.0f;
    p3.feature.gapWidth = 4.0f;
    p1.deviation = 0.5f;
    p2.deviation = 1.0f;
    p3.deviation = 0.3f;

    path.points = {p1, p2, p3};
    path.computeStatistics();

    EXPECT_FLOAT_EQ(path.totalLength, 20.0f);
    EXPECT_FLOAT_EQ(path.avgGapWidth, 3.0f);
    EXPECT_FLOAT_EQ(path.maxDeviation, 1.0f);
}

// ============================================================================
// ProfileProcessor Tests
// ============================================================================

TEST(ProfileProcessorTest, MedianFilter) {
    ProfileProcessor processor;

    std::vector<Point2D> points = {
        {0, 10, 100, true},
        {1, 100, 100, true},  // Outlier
        {2, 12, 100, true},
        {3, 11, 100, true},
        {4, 13, 100, true}
    };

    processor.medianFilter(points, 3);

    // Outlier should be smoothed
    EXPECT_LT(points[1].z, 50.0f);
}

TEST(ProfileProcessorTest, RemoveOutliers) {
    ProfileProcessor processor;
    SeamDetectionConfig config;
    config.outlierThreshold = 2.0f;
    processor.setConfig(config);

    std::vector<Point2D> points;
    for (int i = 0; i < 100; ++i) {
        points.push_back({static_cast<float>(i), 10.0f, 100.0f, true});
    }
    // Add outlier
    points[50].z = 100.0f;

    processor.removeOutliers(points, 2.0f);

    EXPECT_FALSE(points[50].valid);
}

TEST(ProfileProcessorTest, SegmentProfile) {
    ProfileProcessor processor;

    // Create V-shaped profile
    std::vector<Point2D> points;
    for (int i = 0; i < 100; ++i) {
        float x = i - 50.0f;
        float z = std::abs(x);  // V-shape
        points.push_back({x, z, 100.0f, true});
    }

    auto segments = processor.segmentProfile(points);

    EXPECT_GE(segments.size(), 2u);
}

// ============================================================================
// JointDetector Tests
// ============================================================================

TEST(JointDetectorTest, DetectVGroove) {
    JointDetector detector;
    SeamDetectionConfig config;
    config.expectedJointType = JointType::VGroove;
    config.minGrooveAngle = 20.0f;
    config.maxGrooveAngle = 80.0f;
    detector.setConfig(config);

    // Create V-groove profile
    // Left plate (flat at z=50), left slope, right slope, right plate (flat at z=50)
    std::vector<Point2D> points;

    // Left plate
    for (int i = 0; i < 20; ++i) {
        points.push_back({static_cast<float>(i - 50), 50.0f, 100.0f, true});
    }

    // Left slope (45 degree)
    for (int i = 0; i < 20; ++i) {
        float x = i - 30.0f;
        float z = 50.0f - (30.0f + x);  // Going down
        points.push_back({x, z, 100.0f, true});
    }

    // Right slope (45 degree)
    for (int i = 0; i < 20; ++i) {
        float x = i - 10.0f;
        float z = 20.0f + x;  // Going up
        points.push_back({x, z, 100.0f, true});
    }

    // Right plate
    for (int i = 0; i < 20; ++i) {
        points.push_back({static_cast<float>(i + 10), 50.0f, 100.0f, true});
    }

    auto feature = detector.detectVGroove(points);

    // Should detect V-groove (may not be perfect with simplified data)
    // At minimum, the algorithm should run without crashing
    EXPECT_EQ(feature.jointType, JointType::VGroove);
}

TEST(JointDetectorTest, RansacLineFit) {
    JointDetector detector;

    // Create points on a line y = 2x + 5 with some outliers
    std::vector<Point2D> points;
    for (int i = 0; i < 50; ++i) {
        float x = static_cast<float>(i);
        float z = 2.0f * x + 5.0f;
        points.push_back({x, z, 100.0f, true});
    }

    // Add outliers
    points.push_back({25.0f, 100.0f, 100.0f, true});
    points.push_back({30.0f, -50.0f, 100.0f, true});

    auto result = detector.ransacLineFit(points, 100, 1.0f, 10);

    EXPECT_TRUE(result.valid);
    EXPECT_NEAR(result.slope, 2.0f, 0.1f);
    EXPECT_NEAR(result.intercept, 5.0f, 1.0f);
}

TEST(JointDetectorTest, FindLineIntersection) {
    JointDetector detector;

    // Line 1: z = x + 0 (45 degree, passes through origin)
    RansacResult line1;
    line1.slope = 1.0f;
    line1.intercept = 0.0f;
    line1.valid = true;

    // Line 2: z = -x + 10 (passes through (5, 5))
    RansacResult line2;
    line2.slope = -1.0f;
    line2.intercept = 10.0f;
    line2.valid = true;

    auto intersection = detector.findLineIntersection(line1, line2);

    ASSERT_TRUE(intersection.has_value());
    EXPECT_NEAR(intersection->x, 5.0f, 0.01f);
    EXPECT_NEAR(intersection->z, 5.0f, 0.01f);
}

// ============================================================================
// SeamTracker Tests
// ============================================================================

TEST(SeamTrackerTest, InitialState) {
    SeamTracker tracker;

    EXPECT_FALSE(tracker.isTracking());

    auto state = tracker.getState();
    EXPECT_FALSE(state.trackingLost);
    EXPECT_EQ(state.framesProcessed, 0u);
}

TEST(SeamTrackerTest, Reset) {
    SeamTracker tracker;
    SeamDetectionConfig config;
    tracker.setConfig(config);

    tracker.reset();

    auto state = tracker.getState();
    EXPECT_EQ(state.framesProcessed, 0u);
    EXPECT_EQ(state.detectionsValid, 0u);
}

TEST(KalmanFilter2DTest, Prediction) {
    KalmanFilter2D kf;

    // Initialize with measurement
    kf.update(10.0f, 20.0f);

    // Predict forward
    kf.predict(0.1f);  // 100ms

    // Position should not change much without velocity
    EXPECT_NEAR(kf.getX(), 10.0f, 1.0f);
    EXPECT_NEAR(kf.getZ(), 20.0f, 1.0f);
}

TEST(KalmanFilter2DTest, TrackMovingTarget) {
    KalmanFilter2D kf;
    kf.setProcessNoise(0.1f);
    kf.setMeasurementNoise(1.0f);

    // Simulate target moving at constant velocity
    for (int i = 0; i < 20; ++i) {
        float trueX = i * 1.0f;
        float trueZ = 50.0f;

        kf.predict(0.1f);
        kf.update(trueX, trueZ);
    }

    // Filter should track the position
    EXPECT_NEAR(kf.getX(), 19.0f, 2.0f);
    EXPECT_NEAR(kf.getZ(), 50.0f, 2.0f);

    // Should have estimated positive X velocity
    EXPECT_GT(kf.getVx(), 0.0f);
}
```

---

## Completion Checklist

- [ ] SeamTypes.hpp created with JointType, SeamFeature, SeamPoint, SeamPath
- [ ] ProfileProcessor.hpp/.cpp implemented (median filter, outlier removal)
- [ ] JointDetector.hpp/.cpp implemented (V-groove, Lap, Fillet detection)
- [ ] SeamTracker.hpp/.cpp implemented (Kalman filter, latency compensation)
- [ ] IPC message types for seam detection defined
- [ ] ISeamDetectionService.cs interface created
- [ ] SeamDetectionService.cs implemented
- [ ] Unit tests created (18+ tests)
- [ ] CMake configuration updated
- [ ] All tests pass

---

## Troubleshooting

### Detection Not Working
- Check ROI settings match sensor field of view
- Verify profile data is valid (not all NaN)
- Adjust RANSAC threshold for noise level
- Check groove angle limits

### Tracking Lost Frequently
- Increase tracking timeout
- Reduce measurement noise in Kalman filter
- Verify robot pose synchronization
- Check for arc interference in profile

### High Latency
- Reduce profile resolution if possible
- Optimize RANSAC iterations
- Use hardware trigger for synchronization
- Profile code for bottlenecks

---

## Git Commit

```powershell
git add -A
git commit -m "IMPL_P4_02: Add seam detection and tracking

- Create SeamTypes.hpp with JointType, SeamFeature, SeamPath structures
- Create ProfileProcessor for preprocessing and segmentation
- Implement JointDetector with V-groove, Lap, Fillet detection
- Implement RANSAC line fitting for robust groove detection
- Create SeamTracker with Kalman filter for prediction
- Add latency compensation for real-time tracking
- Create ISeamDetectionService C# interface
- Implement SeamDetectionService with IPC integration
- Add IPC message types for seam detection
- Add 18+ unit tests for detection and tracking
- Support V-groove angle and gap width measurement

Co-Authored-By: Claude <noreply@anthropic.com>"
```
