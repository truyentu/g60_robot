#pragma once

#include "VisionTypes.hpp"
#include <vector>
#include <array>
#include <optional>
#include <functional>
#include <cmath>
#include <cfloat>

namespace robot_controller::vision {

// ============================================================================
// Joint Types
// ============================================================================

/// Supported weld joint types
enum class JointType {
    Unknown,
    VGroove,        // V-groove butt joint
    UGroove,        // U-groove
    JGroove,        // J-groove
    LapJoint,       // Overlap joint
    FilletJoint,    // Fillet/T-joint
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

} // namespace robot_controller::vision
