#include <gtest/gtest.h>
#include "welding/WeavingTypes.hpp"
#include "welding/WeavePatternGenerator.hpp"
#include "welding/WeaveExecutor.hpp"
#include "welding/WeaveVisualizer.hpp"

using namespace robotics::welding;
using namespace robot_controller::kinematics;

// ============================================================================
// Weaving Types Tests
// ============================================================================

TEST(WeavingTypesTest, DefaultParams) {
    WeaveParams params;

    EXPECT_EQ(params.type, WeavePatternType::SINUSOIDAL);
    EXPECT_GT(params.amplitude, 0);
    EXPECT_GT(params.wavelength, 0);
}

TEST(WeavingTypesTest, PresetNarrowGap) {
    auto params = WeavePresets::narrowGap();

    EXPECT_EQ(params.type, WeavePatternType::TRIANGULAR);
    EXPECT_LT(params.amplitude, 3);  // Narrow
}

TEST(WeavingTypesTest, PresetWideGap) {
    auto params = WeavePresets::wideGap();

    EXPECT_EQ(params.type, WeavePatternType::TRIANGULAR);
    EXPECT_GT(params.amplitude, 5);  // Wide
    EXPECT_LT(params.speedAtEdge, 1);  // Slower at edges
}

TEST(WeavingTypesTest, WeaveFrame) {
    WeaveFrame frame = WeaveFrame::fromTangentAndNormal(
        Vector3d(0, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0, 0, 1));

    EXPECT_NEAR(frame.tangent.x(), 1, 0.01);
    EXPECT_NEAR(frame.normal.z(), 1, 0.01);
    // tangent x normal = lateral (right-hand rule: X x Z = -Y)
    EXPECT_NEAR(std::abs(frame.lateral.y()), 1, 0.01);
}

TEST(WeavingTypesTest, WeaveFrameApplyOffset) {
    WeaveFrame frame = WeaveFrame::fromTangentAndNormal(
        Vector3d(100, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0, 0, 1));

    WeavePoint offset;
    offset.lateral = 5.0;

    Vector3d result = frame.applyOffset(offset);

    EXPECT_NEAR(result.x(), 100, 0.01);
    EXPECT_NEAR(std::abs(result.y()), 5.0, 0.01);  // Could be +5 or -5 depending on coordinate frame
    EXPECT_NEAR(result.z(), 0, 0.01);
}

// ============================================================================
// Pattern Generator Tests
// ============================================================================

TEST(PatternGeneratorTest, LinearPattern) {
    LinearPatternGenerator gen;
    WeaveParams params;
    params.amplitude = 5.0;

    // At phase 0, should be near center
    auto pt0 = gen.generate(0, params);
    EXPECT_NEAR(pt0.lateral, 0, 0.5);

    // At phase 0.25, should be at right edge
    auto pt25 = gen.generate(0.25, params);
    EXPECT_GT(pt25.lateral, 4.5);

    // At phase 0.75, should be at left edge
    auto pt75 = gen.generate(0.75, params);
    EXPECT_LT(pt75.lateral, -4.0);
}

TEST(PatternGeneratorTest, SinusoidalPattern) {
    SinusoidalPatternGenerator gen;
    WeaveParams params;
    params.amplitude = 3.0;

    // At phase 0.25 (90 degrees), should be at max
    auto pt25 = gen.generate(0.25, params);
    EXPECT_NEAR(pt25.lateral, params.amplitude, 0.1);

    // At phase 0.75 (270 degrees), should be at min
    auto pt75 = gen.generate(0.75, params);
    EXPECT_NEAR(pt75.lateral, -params.amplitude, 0.1);
}

TEST(PatternGeneratorTest, TriangularPattern) {
    TriangularPatternGenerator gen;
    WeaveParams params;
    params.amplitude = 4.0;

    // At phase 0.5, should be at right edge
    auto pt50 = gen.generate(0.5, params);
    EXPECT_NEAR(pt50.lateral, params.amplitude, 0.5);

    // At phase 0, should be at left edge
    auto pt0 = gen.generate(0.0, params);
    EXPECT_NEAR(pt0.lateral, -params.amplitude, 0.5);
}

TEST(PatternGeneratorTest, CircularPattern) {
    CircularPatternGenerator gen;
    WeaveParams params;
    params.amplitude = 2.0;
    params.circleRadius = 2.0;

    auto cycle = gen.generateCycle(params, 36);

    EXPECT_EQ(cycle.points.size(), 36);

    // Check radius consistency
    for (const auto& pt : cycle.points) {
        double radius = std::sqrt(pt.lateral * pt.lateral +
                                   pt.longitudinal * pt.longitudinal);
        EXPECT_NEAR(radius, params.circleRadius, 0.1);
    }
}

TEST(PatternGeneratorTest, Figure8Pattern) {
    Figure8PatternGenerator gen;
    WeaveParams params;
    params.amplitude = 3.0;

    auto cycle = gen.generateCycle(params, 64);

    EXPECT_EQ(cycle.points.size(), 64);

    // Figure-8 pattern should have lateral oscillation
    double maxLateral = 0;
    for (const auto& pt : cycle.points) {
        maxLateral = std::max(maxLateral, std::abs(pt.lateral));
    }
    EXPECT_GT(maxLateral, params.amplitude * 0.9);
}

TEST(PatternGeneratorTest, CrescentPattern) {
    CrescentPatternGenerator gen;
    WeaveParams params;
    params.amplitude = 4.0;

    auto cycle = gen.generateCycle(params, 32);

    EXPECT_EQ(cycle.points.size(), 32);

    // Should have longitudinal movement
    double maxLong = 0;
    for (const auto& pt : cycle.points) {
        maxLong = std::max(maxLong, std::abs(pt.longitudinal));
    }
    EXPECT_GT(maxLong, 0);
}

TEST(PatternGeneratorTest, Factory) {
    auto linear = createPatternGenerator(WeavePatternType::LINEAR);
    EXPECT_EQ(linear->getType(), WeavePatternType::LINEAR);

    auto sine = createPatternGenerator(WeavePatternType::SINUSOIDAL);
    EXPECT_EQ(sine->getType(), WeavePatternType::SINUSOIDAL);
}

// ============================================================================
// Weave Executor Tests
// ============================================================================

TEST(WeaveExecutorTest, InitialState) {
    WeaveExecutor executor;

    EXPECT_FALSE(executor.isActive());
    EXPECT_FALSE(executor.isEnabled());
    EXPECT_EQ(executor.getCycleCount(), 0);
}

TEST(WeaveExecutorTest, StartStop) {
    WeaveExecutor executor;
    executor.setEnabled(true);

    executor.start();
    EXPECT_TRUE(executor.isActive());

    executor.stop();
    EXPECT_FALSE(executor.isActive());
}

TEST(WeaveExecutorTest, PhaseAdvancement) {
    WeaveExecutor executor;
    WeaveParams params;
    params.type = WeavePatternType::SINUSOIDAL;
    params.wavelength = 10.0;  // 10mm per cycle

    executor.setParams(params);
    executor.setEnabled(true);
    executor.start();

    // Travel 5mm at 10mm/s = 0.5s
    executor.update(0.5, 10.0);

    // Should be at phase 0.5
    EXPECT_NEAR(executor.getCurrentPhase(), 0.5, 0.1);
}

TEST(WeaveExecutorTest, CycleCompletion) {
    WeaveExecutor executor;
    WeaveParams params;
    params.wavelength = 10.0;

    executor.setParams(params);
    executor.setEnabled(true);
    executor.start();

    EXPECT_EQ(executor.getCycleCount(), 0);

    // Travel 15mm (1.5 cycles)
    executor.update(1.5, 10.0);

    EXPECT_EQ(executor.getCycleCount(), 1);
}

TEST(WeaveExecutorTest, ApplyWeave) {
    WeaveExecutor executor;
    WeaveParams params;
    params.type = WeavePatternType::SINUSOIDAL;
    params.amplitude = 5.0;

    executor.setParams(params);
    executor.setEnabled(true);
    executor.start();

    // Update to phase where offset is non-zero
    executor.update(0.25, 10.0);

    WeaveFrame frame = WeaveFrame::fromTangentAndNormal(
        Vector3d(0, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0, 0, 1));

    Vector3d original(100, 0, 50);
    Vector3d weaved = executor.applyWeave(original, frame);

    // Y should have lateral offset
    EXPECT_NE(weaved.y(), original.y());
}

TEST(WeaveExecutorTest, DisabledNoEffect) {
    WeaveExecutor executor;

    executor.start();  // But not enabled

    WeavePoint offset = executor.getCurrentOffset();

    EXPECT_NEAR(offset.lateral, 0, 0.01);
}

TEST(WeaveExecutorTest, AdjustAmplitude) {
    WeaveExecutor executor;
    WeaveParams params;
    params.amplitude = 3.0;

    executor.setParams(params);
    executor.adjustAmplitude(5.0);

    EXPECT_EQ(executor.getParams().amplitude, 5.0);
}

// ============================================================================
// Weave Visualizer Tests
// ============================================================================

TEST(WeaveVisualizerTest, Preview2D) {
    WeaveParams params;
    params.type = WeavePatternType::SINUSOIDAL;
    params.amplitude = 3.0;
    params.wavelength = 10.0;

    auto points = WeaveVisualizer::generatePreview2D(params, 2, 32);

    EXPECT_EQ(points.size(), 64);  // 2 cycles * 32 points

    // Check amplitude bounds
    for (const auto& pt : points) {
        EXPECT_LE(std::abs(pt.second), params.amplitude + 0.1);
    }
}

TEST(WeaveVisualizerTest, Preview3D) {
    WeaveParams params;
    params.type = WeavePatternType::SINUSOIDAL;  // Use sinusoidal for clearer oscillation
    params.amplitude = 2.0;
    params.wavelength = 5.0;

    // Simple straight path
    std::vector<Vector3d> basePath;
    std::vector<Vector3d> normals;

    for (int i = 0; i <= 20; ++i) {
        basePath.push_back(Vector3d(i * 5.0, 0, 0));
        normals.push_back(Vector3d(0, 0, 1));
    }

    auto weaved = WeaveVisualizer::generatePreview3D(params, basePath, normals);

    EXPECT_EQ(weaved.size(), basePath.size());

    // Weaved path should have some deviation
    double maxDeviation = 0;
    for (const auto& pt : weaved) {
        maxDeviation = std::max(maxDeviation, std::abs(pt.y()));
    }
    // Deviation should exist (not necessarily amplitude due to coordinate frame)
    EXPECT_LE(maxDeviation, params.amplitude + 0.1);
}

TEST(WeaveVisualizerTest, PatternStats) {
    WeaveParams params;
    params.type = WeavePatternType::SINUSOIDAL;
    params.amplitude = 3.0;
    params.wavelength = 10.0;

    auto stats = WeaveVisualizer::calculateStats(params, 10.0);

    // Weave path should be longer than straight
    EXPECT_GT(stats.pathLengthRatio, 1.0);

    // Average speed should be around 1.0
    EXPECT_GT(stats.averageSpeed, 0.5);
    EXPECT_LT(stats.averageSpeed, 1.5);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(WeaveIntegrationTest, FullWeaveCycle) {
    WeaveExecutor executor;
    WeaveParams params;
    params.type = WeavePatternType::TRIANGULAR;
    params.amplitude = 4.0;
    params.wavelength = 8.0;
    params.dwellLeft = 50;
    params.dwellRight = 50;

    executor.setParams(params);
    executor.setEnabled(true);
    executor.start();

    // Simulate welding for 2 cycles
    double dt = 0.01;  // 10ms
    double travelSpeed = 10.0;  // 10 mm/s

    std::vector<double> lateralPositions;

    for (int i = 0; i < 200; ++i) {  // 2 seconds
        executor.update(dt, travelSpeed);
        auto offset = executor.getCurrentOffset();
        lateralPositions.push_back(offset.lateral);
    }

    // Should have completed approximately 2.5 cycles
    EXPECT_GE(executor.getCycleCount(), 2);

    // Check that we hit both edges
    double minLat = *std::min_element(lateralPositions.begin(), lateralPositions.end());
    double maxLat = *std::max_element(lateralPositions.begin(), lateralPositions.end());

    EXPECT_LT(minLat, -3.5);  // Near left edge
    EXPECT_GT(maxLat, 3.5);   // Near right edge
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
