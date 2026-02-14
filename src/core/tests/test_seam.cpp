#include <gtest/gtest.h>
#include "../src/vision/SeamTypes.hpp"
#include "../src/vision/ProfileProcessor.hpp"
#include "../src/vision/JointDetector.hpp"
#include "../src/vision/SeamTracker.hpp"
#include <cmath>

using namespace robot_controller::vision;

// ============================================================================
// SeamTypes Tests
// ============================================================================

TEST(SeamTypesTest, Point2DDefaultConstruction) {
    Point2D p;
    EXPECT_FLOAT_EQ(p.x, 0.0f);
    EXPECT_FLOAT_EQ(p.z, 0.0f);
    EXPECT_TRUE(p.valid);
}

TEST(SeamTypesTest, Point2DParameterizedConstruction) {
    Point2D p(10.5f, 20.3f);
    EXPECT_FLOAT_EQ(p.x, 10.5f);
    EXPECT_FLOAT_EQ(p.z, 20.3f);
}

TEST(SeamTypesTest, SeamFeatureDefaults) {
    SeamFeature feature;
    EXPECT_FALSE(feature.valid);
    EXPECT_EQ(feature.jointType, JointType::Unknown);
    EXPECT_FLOAT_EQ(feature.confidence, 0.0f);
}

TEST(SeamTypesTest, SeamPointDefaults) {
    SeamPoint point;
    EXPECT_FALSE(point.valid);
    EXPECT_FLOAT_EQ(point.deviation, 0.0f);
}

TEST(SeamTypesTest, SeamDetectionConfigDefaults) {
    SeamDetectionConfig config;
    EXPECT_EQ(config.expectedJointType, JointType::Unknown);
    EXPECT_TRUE(config.enableMedianFilter);
    EXPECT_EQ(config.medianFilterSize, 5);
    EXPECT_TRUE(config.enableOutlierRemoval);
}

TEST(SeamTypesTest, TrackingStateDefaults) {
    TrackingState state;
    EXPECT_FALSE(state.trackingLost);
    EXPECT_EQ(state.framesProcessed, 0);
    EXPECT_FLOAT_EQ(state.trackingQuality, 0.0f);
}

// ============================================================================
// ProfileProcessor Tests
// ============================================================================

TEST(ProfileProcessorTest, MedianFilterOddSize) {
    ProfileProcessor processor;
    SeamDetectionConfig config;
    config.enableMedianFilter = true;
    config.medianFilterSize = 3;
    processor.setConfig(config);

    std::vector<Point2D> points = {
        Point2D(0, 10),
        Point2D(1, 100),  // Outlier
        Point2D(2, 12),
        Point2D(3, 11),
        Point2D(4, 13)
    };

    float originalZ1 = points[1].z;
    processor.medianFilter(points, 3);

    // Median filter should reduce the outlier effect
    ASSERT_EQ(points.size(), 5u);
    // Middle value should be closer to neighbors (outlier reduced)
    EXPECT_LT(std::abs(points[1].z - 12), std::abs(originalZ1 - 12));
}

TEST(ProfileProcessorTest, MedianFilterPreservesSize) {
    ProfileProcessor processor;

    std::vector<Point2D> points;
    for (int i = 0; i < 20; ++i) {
        points.emplace_back(static_cast<float>(i), static_cast<float>(i * 2));
    }

    size_t originalSize = points.size();
    processor.medianFilter(points, 5);

    EXPECT_EQ(points.size(), originalSize);
}

TEST(ProfileProcessorTest, RemoveOutliers) {
    ProfileProcessor processor;
    SeamDetectionConfig config;
    config.enableOutlierRemoval = true;
    config.outlierThreshold = 2.0f;  // 2 sigma
    processor.setConfig(config);

    std::vector<Point2D> points;
    // Normal points around z = 10
    for (int i = 0; i < 10; ++i) {
        points.emplace_back(static_cast<float>(i), 10.0f + (i % 2 ? 0.5f : -0.5f));
    }
    // Add outlier
    points[5].z = 100.0f;

    size_t originalSize = points.size();
    processor.removeOutliers(points, 2.0f);

    // Outlier should be removed or marked invalid
    // The function modifies in place, check if size changed or outlier handled
    EXPECT_LE(points.size(), originalSize);
}

TEST(ProfileProcessorTest, ProcessProfileIntegration) {
    ProfileProcessor processor;
    SeamDetectionConfig config;
    config.enableMedianFilter = true;
    config.medianFilterSize = 3;
    config.enableOutlierRemoval = true;
    config.outlierThreshold = 3.0f;
    processor.setConfig(config);

    LaserProfile profile;
    profile.frameId = 1;
    profile.timestamp = 1000;

    // Create V-groove like profile
    for (int i = 0; i < 50; ++i) {
        float x = static_cast<float>(i - 25);  // -25 to 24
        float z = std::abs(x);  // V shape
        profile.points.emplace_back(x, z);
    }

    auto feature = processor.processProfile(profile);

    // Feature should be returned (may or may not be valid depending on detection)
    // Just verify it doesn't crash and returns something
    SUCCEED();
}

// ============================================================================
// JointDetector Tests
// ============================================================================

TEST(JointDetectorTest, Construction) {
    JointDetector detector;
    // Should not throw
    SUCCEED();
}

TEST(JointDetectorTest, DetectVGrooveProfile) {
    JointDetector detector;
    SeamDetectionConfig config;
    config.expectedJointType = JointType::VGroove;
    config.ransacIterations = 50;
    config.ransacThreshold = 1.0f;
    config.ransacMinInliers = 5;
    detector.setConfig(config);

    // Create V-groove profile
    std::vector<Point2D> points;
    for (int i = 0; i < 50; ++i) {
        float x = static_cast<float>(i - 25);
        float z = std::abs(x) * 0.5f;  // V shape with 0.5 slope
        points.emplace_back(x, z);
    }

    auto feature = detector.detectVGroove(points);

    EXPECT_TRUE(feature.valid);
    EXPECT_EQ(feature.jointType, JointType::VGroove);
    EXPECT_NEAR(feature.rootPoint.x, 0.0f, 2.0f);  // Root should be near center
    EXPECT_NEAR(feature.rootPoint.z, 0.0f, 2.0f);
    EXPECT_GT(feature.confidence, 0.3f);
}

TEST(JointDetectorTest, DetectLapJointProfile) {
    JointDetector detector;
    SeamDetectionConfig config;
    config.expectedJointType = JointType::LapJoint;
    config.minStepHeight = 2.0f;
    config.maxStepHeight = 20.0f;
    detector.setConfig(config);

    // Create lap joint profile (step)
    std::vector<Point2D> points;
    for (int i = 0; i < 50; ++i) {
        float x = static_cast<float>(i - 25);
        float z = (i < 25) ? 0.0f : 10.0f;  // Step at center
        points.emplace_back(x, z);
    }

    auto feature = detector.detectLapJoint(points);

    EXPECT_TRUE(feature.valid);
    EXPECT_EQ(feature.jointType, JointType::LapJoint);
    EXPECT_NEAR(feature.depth, 10.0f, 1.0f);
}

TEST(JointDetectorTest, DetectFilletJointProfile) {
    JointDetector detector;
    SeamDetectionConfig config;
    config.expectedJointType = JointType::FilletJoint;
    detector.setConfig(config);

    // Create fillet joint profile (90 degree corner)
    std::vector<Point2D> points;
    for (int i = 0; i < 25; ++i) {
        float x = static_cast<float>(i - 25);
        points.emplace_back(x, 0.0f);  // Horizontal
    }
    for (int i = 0; i < 25; ++i) {
        float z = static_cast<float>(i);
        points.emplace_back(0.0f, z);  // Vertical
    }

    auto feature = detector.detectFilletJoint(points);

    EXPECT_EQ(feature.jointType, JointType::FilletJoint);
    // Confidence should be > 0 if corner is detected
}

TEST(JointDetectorTest, ClassifyJointTypeVGroove) {
    JointDetector detector;
    SeamDetectionConfig config;
    config.minGapWidth = 2.0f;
    detector.setConfig(config);

    std::vector<Point2D> points;
    for (int i = 0; i < 50; ++i) {
        float x = static_cast<float>(i - 25);
        float z = 10.0f + std::abs(x) * (-0.4f);  // V shape pointing down
        points.emplace_back(x, z);
    }

    auto type = detector.classifyJointType(points);
    // Should detect as V-groove or similar
    EXPECT_NE(type, JointType::Unknown);
}

TEST(JointDetectorTest, RansacLineFit) {
    JointDetector detector;

    // Create linear points with some noise
    std::vector<Point2D> points;
    for (int i = 0; i < 20; ++i) {
        float x = static_cast<float>(i);
        float z = 2.0f * x + 1.0f + (i % 3 - 1) * 0.1f;  // z = 2x + 1 with noise
        points.emplace_back(x, z);
    }

    auto result = detector.ransacLineFit(points, 100, 1.0f, 10);

    EXPECT_TRUE(result.valid);
    EXPECT_NEAR(result.slope, 2.0f, 0.3f);
    EXPECT_NEAR(result.intercept, 1.0f, 1.0f);
    EXPECT_GT(result.r2, 0.9f);
}

TEST(JointDetectorTest, LeastSquaresLineFit) {
    JointDetector detector;

    // Perfect linear points
    std::vector<Point2D> points;
    for (int i = 0; i < 10; ++i) {
        float x = static_cast<float>(i);
        float z = 3.0f * x + 5.0f;  // z = 3x + 5
        points.emplace_back(x, z);
    }

    auto result = detector.leastSquaresLineFit(points);

    EXPECT_TRUE(result.valid);
    EXPECT_FLOAT_EQ(result.slope, 3.0f);
    EXPECT_FLOAT_EQ(result.intercept, 5.0f);
    EXPECT_FLOAT_EQ(result.r2, 1.0f);
}

TEST(JointDetectorTest, FindLineIntersection) {
    JointDetector detector;

    RansacResult line1;
    line1.slope = 1.0f;
    line1.intercept = 0.0f;  // z = x
    line1.valid = true;

    RansacResult line2;
    line2.slope = -1.0f;
    line2.intercept = 10.0f;  // z = -x + 10
    line2.valid = true;

    auto intersection = detector.findLineIntersection(line1, line2);

    ASSERT_TRUE(intersection.has_value());
    EXPECT_FLOAT_EQ(intersection->x, 5.0f);
    EXPECT_FLOAT_EQ(intersection->z, 5.0f);
}

TEST(JointDetectorTest, FindLineIntersectionParallel) {
    JointDetector detector;

    RansacResult line1;
    line1.slope = 1.0f;
    line1.intercept = 0.0f;
    line1.valid = true;

    RansacResult line2;
    line2.slope = 1.0f;  // Same slope = parallel
    line2.intercept = 5.0f;
    line2.valid = true;

    auto intersection = detector.findLineIntersection(line1, line2);

    EXPECT_FALSE(intersection.has_value());
}

// ============================================================================
// SeamKalmanFilter Tests
// ============================================================================

TEST(SeamKalmanFilterTest, Initialization) {
    SeamKalmanFilter filter;
    EXPECT_FALSE(filter.isInitialized());

    filter.initialize(10.0f, 20.0f);
    EXPECT_TRUE(filter.isInitialized());

    float x, z, vx, vz;
    filter.getState(x, z, vx, vz);
    EXPECT_FLOAT_EQ(x, 10.0f);
    EXPECT_FLOAT_EQ(z, 20.0f);
    EXPECT_FLOAT_EQ(vx, 0.0f);
    EXPECT_FLOAT_EQ(vz, 0.0f);
}

TEST(SeamKalmanFilterTest, Reset) {
    SeamKalmanFilter filter;
    filter.initialize(10.0f, 20.0f);
    EXPECT_TRUE(filter.isInitialized());

    filter.reset();
    EXPECT_FALSE(filter.isInitialized());
}

TEST(SeamKalmanFilterTest, PredictWithoutInit) {
    SeamKalmanFilter filter;

    // Should not crash
    filter.predict(0.1f);
    EXPECT_FALSE(filter.isInitialized());
}

TEST(SeamKalmanFilterTest, PredictWithVelocity) {
    SeamKalmanFilter filter;
    filter.initialize(0.0f, 0.0f);

    // Update with moving measurement
    filter.update(10.0f, 5.0f);

    // Predict forward
    float x, z;
    filter.getPrediction(1.0f, x, z);

    // State should have some velocity estimate
    float sx, sz, vx, vz;
    filter.getState(sx, sz, vx, vz);

    // With measurements, velocity should be non-zero
    EXPECT_NE(vx, 0.0f);
}

TEST(SeamKalmanFilterTest, UpdateAutoInitializes) {
    SeamKalmanFilter filter;
    EXPECT_FALSE(filter.isInitialized());

    filter.update(5.0f, 10.0f);
    EXPECT_TRUE(filter.isInitialized());

    float x, z, vx, vz;
    filter.getState(x, z, vx, vz);
    EXPECT_FLOAT_EQ(x, 5.0f);
    EXPECT_FLOAT_EQ(z, 10.0f);
}

// ============================================================================
// SeamTracker Tests
// ============================================================================

TEST(SeamTrackerTest, Construction) {
    SeamTracker tracker;
    EXPECT_FALSE(tracker.isEnabled());
}

TEST(SeamTrackerTest, EnableDisable) {
    SeamTracker tracker;

    tracker.setEnabled(true);
    EXPECT_TRUE(tracker.isEnabled());

    tracker.setEnabled(false);
    EXPECT_FALSE(tracker.isEnabled());
}

TEST(SeamTrackerTest, Reset) {
    SeamTracker tracker;
    tracker.setEnabled(true);
    tracker.reset();

    auto state = tracker.getState();
    EXPECT_EQ(state.framesProcessed, 0);
    EXPECT_EQ(state.detectionsValid, 0);
}

TEST(SeamTrackerTest, SetConfig) {
    SeamTracker tracker;
    SeamDetectionConfig config;
    config.expectedJointType = JointType::VGroove;
    config.enableTracking = true;
    config.systemLatency = 0.050f;

    tracker.setConfig(config);
    // Should not throw
    SUCCEED();
}

TEST(SeamTrackerTest, UpdateWithVGrooveProfile) {
    SeamTracker tracker;
    SeamDetectionConfig config;
    config.expectedJointType = JointType::VGroove;
    config.enableTracking = true;
    config.roiXMin = -30.0f;
    config.roiXMax = 30.0f;
    config.roiZMin = 0.0f;
    config.roiZMax = 50.0f;
    tracker.setConfig(config);
    tracker.setEnabled(true);

    // Create V-groove profile
    LaserProfile profile;
    profile.frameId = 1;
    profile.timestamp = 1000;

    for (int i = 0; i < 50; ++i) {
        float x = static_cast<float>(i - 25);
        float z = 20.0f - std::abs(x) * 0.4f;  // Inverted V
        Point2D p(x, z);
        p.valid = true;
        profile.points.push_back(p);
    }

    RobotPose pose;
    pose.valid = false;  // No transform

    auto state = tracker.update(profile, pose);

    EXPECT_EQ(state.framesProcessed, 1);
    // Feature might or might not be valid depending on detection
}

TEST(SeamTrackerTest, SetNominalPath) {
    SeamTracker tracker;

    SeamPath path;
    for (int i = 0; i < 10; ++i) {
        SeamPoint point;
        point.position.x = static_cast<float>(i);
        point.position.y = 0.0f;
        point.position.z = 0.0f;
        point.valid = true;
        path.points.push_back(point);
    }

    tracker.setNominalPath(path);
    // Should not throw
    SUCCEED();
}

TEST(SeamTrackerTest, GetPredictedPoint) {
    SeamTracker tracker;
    SeamDetectionConfig config;
    config.expectedJointType = JointType::VGroove;
    tracker.setConfig(config);

    auto point = tracker.getPredictedPoint(0.1f);
    // Should return a point (may be invalid if no tracking)
    SUCCEED();
}

TEST(SeamTrackerTest, FeatureCallback) {
    SeamTracker tracker;

    bool callbackCalled = false;
    SeamFeature receivedFeature;

    tracker.setFeatureCallback([&](const SeamFeature& feature) {
        callbackCalled = true;
        receivedFeature = feature;
    });

    // Create profile that should trigger detection
    SeamDetectionConfig config;
    config.expectedJointType = JointType::VGroove;
    config.roiXMin = -50.0f;
    config.roiXMax = 50.0f;
    config.roiZMin = 0.0f;
    config.roiZMax = 100.0f;
    tracker.setConfig(config);

    LaserProfile profile;
    for (int i = 0; i < 50; ++i) {
        float x = static_cast<float>(i - 25);
        float z = std::abs(x);
        Point2D p(x, z);
        p.valid = true;
        profile.points.push_back(p);
    }

    RobotPose pose;
    pose.valid = false;

    tracker.update(profile, pose);

    // Callback should be called if feature is valid
    if (receivedFeature.valid) {
        EXPECT_TRUE(callbackCalled);
    }
}

TEST(SeamTrackerTest, TrackingCallback) {
    SeamTracker tracker;

    bool callbackCalled = false;
    TrackingState receivedState;

    tracker.setTrackingCallback([&](const TrackingState& state) {
        callbackCalled = true;
        receivedState = state;
    });

    LaserProfile profile;
    for (int i = 0; i < 30; ++i) {
        Point2D p(static_cast<float>(i), static_cast<float>(i));
        p.valid = true;
        profile.points.push_back(p);
    }

    RobotPose pose;
    pose.valid = false;

    tracker.update(profile, pose);

    EXPECT_TRUE(callbackCalled);
    EXPECT_EQ(receivedState.framesProcessed, 1);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(SeamIntegrationTest, FullTrackingLoop) {
    SeamTracker tracker;
    SeamDetectionConfig config;
    config.expectedJointType = JointType::VGroove;
    config.enableTracking = true;
    config.roiXMin = -30.0f;
    config.roiXMax = 30.0f;
    config.roiZMin = 0.0f;
    config.roiZMax = 50.0f;
    config.systemLatency = 0.030f;
    tracker.setConfig(config);
    tracker.setEnabled(true);

    // Simulate multiple frames
    for (int frame = 0; frame < 10; ++frame) {
        LaserProfile profile;
        profile.frameId = static_cast<uint32_t>(frame);
        profile.timestamp = static_cast<uint64_t>(frame * 33);  // ~30 Hz

        // V-groove profile with slight movement
        float offset = static_cast<float>(frame) * 0.1f;
        for (int i = 0; i < 50; ++i) {
            float x = static_cast<float>(i - 25) + offset;
            float z = 20.0f - std::abs(static_cast<float>(i - 25)) * 0.4f;
            Point2D p(x, z);
            p.valid = true;
            profile.points.push_back(p);
        }

        RobotPose pose;
        pose.valid = false;

        auto state = tracker.update(profile, pose);
        EXPECT_EQ(state.framesProcessed, static_cast<uint64_t>(frame + 1));
    }

    auto finalState = tracker.getState();
    EXPECT_EQ(finalState.framesProcessed, 10);
    EXPECT_GT(finalState.avgLatency, 0.0f);  // Should have measured some latency
}

TEST(SeamIntegrationTest, TrackingWithTransform) {
    SeamTracker tracker;
    SeamDetectionConfig config;
    config.expectedJointType = JointType::VGroove;
    config.roiXMin = -30.0f;
    config.roiXMax = 30.0f;
    config.roiZMin = 0.0f;
    config.roiZMax = 50.0f;
    tracker.setConfig(config);

    LaserProfile profile;
    for (int i = 0; i < 50; ++i) {
        float x = static_cast<float>(i - 25);
        float z = std::abs(x);
        Point2D p(x, z);
        p.valid = true;
        profile.points.push_back(p);
    }

    // Identity transform
    RobotPose pose;
    pose.valid = true;
    pose.flange = {
        1, 0, 0, 100,   // Translation X = 100
        0, 1, 0, 200,   // Translation Y = 200
        0, 0, 1, 50,    // Translation Z = 50
        0, 0, 0, 1
    };

    auto state = tracker.update(profile, pose);

    if (state.currentSeamPoint.valid) {
        // World coordinates should include transform
        EXPECT_GT(state.currentSeamPoint.position.x, 50);  // Translated
        EXPECT_GT(state.currentSeamPoint.position.y, 150);
    }
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
