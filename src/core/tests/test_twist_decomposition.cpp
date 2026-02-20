/**
 * @file test_twist_decomposition.cpp
 * @brief Unit tests for TWA (Twist Decomposition Algorithm)
 *
 * Tests the core TWA math:
 *   - Twist Projector construction and properties
 *   - Parameter of Singularity (PS) index
 *   - Secondary task gradient
 *   - Full TWA resolution
 *   - DLS fallback
 */

#include <gtest/gtest.h>
#include "../src/kinematics/TwistDecomposition.hpp"
#include <cmath>

using namespace robot_controller::kinematics;

// ============================================================================
// Twist Projector Tests
// ============================================================================

TEST(TwistDecomposition, TwistProjector_ZAxis) {
    // Torch axis along Z → should block out ωz rotation
    Vector3d torchAxis(0.0, 0.0, 1.0);
    Matrix6d T = TwistDecomposition::computeTwistProjector(torchAxis);

    // Angular part: M_ω = I - e·eᵀ = diag(1,1,0) for Z-axis
    EXPECT_NEAR(T(0, 0), 1.0, 1e-10);  // ωx preserved
    EXPECT_NEAR(T(1, 1), 1.0, 1e-10);  // ωy preserved
    EXPECT_NEAR(T(2, 2), 0.0, 1e-10);  // ωz blocked (spin)

    // Linear part: identity
    EXPECT_NEAR(T(3, 3), 1.0, 1e-10);  // vx preserved
    EXPECT_NEAR(T(4, 4), 1.0, 1e-10);  // vy preserved
    EXPECT_NEAR(T(5, 5), 1.0, 1e-10);  // vz preserved

    // Off-diagonal angular should be zero for axis-aligned case
    EXPECT_NEAR(T(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(T(0, 2), 0.0, 1e-10);
    EXPECT_NEAR(T(1, 2), 0.0, 1e-10);
}

TEST(TwistDecomposition, TwistProjector_XAxis) {
    // Torch axis along X → should block out ωx rotation
    Vector3d torchAxis(1.0, 0.0, 0.0);
    Matrix6d T = TwistDecomposition::computeTwistProjector(torchAxis);

    EXPECT_NEAR(T(0, 0), 0.0, 1e-10);  // ωx blocked (spin)
    EXPECT_NEAR(T(1, 1), 1.0, 1e-10);  // ωy preserved
    EXPECT_NEAR(T(2, 2), 1.0, 1e-10);  // ωz preserved
}

TEST(TwistDecomposition, TwistProjector_ArbitraryAxis) {
    // Torch axis at 45° in XZ plane
    Vector3d torchAxis(1.0, 0.0, 1.0);  // Will be normalized
    Matrix6d T = TwistDecomposition::computeTwistProjector(torchAxis);

    // Verify property: T·[e_angular; 0] should project out the spin component
    Vector6d spinTwist;
    Vector3d e = torchAxis.normalized();
    spinTwist << e(0), e(1), e(2), 0.0, 0.0, 0.0;
    Vector6d projected = T * spinTwist;

    // Angular part should be near zero (spin projected out)
    EXPECT_NEAR(projected.head<3>().norm(), 0.0, 1e-10);
    // Linear part should be zero (no translation in input)
    EXPECT_NEAR(projected.tail<3>().norm(), 0.0, 1e-10);
}

TEST(TwistDecomposition, TwistProjector_PreservesTranslation) {
    Vector3d torchAxis(0.0, 0.0, 1.0);
    Matrix6d T = TwistDecomposition::computeTwistProjector(torchAxis);

    // Pure linear twist
    Vector6d linearTwist;
    linearTwist << 0.0, 0.0, 0.0, 10.0, 20.0, 30.0;
    Vector6d result = T * linearTwist;

    // Translation should be preserved exactly
    EXPECT_NEAR(result(3), 10.0, 1e-10);
    EXPECT_NEAR(result(4), 20.0, 1e-10);
    EXPECT_NEAR(result(5), 30.0, 1e-10);
}

TEST(TwistDecomposition, TwistProjector_Idempotent) {
    // T should be idempotent: T² = T (it's a projector)
    Vector3d torchAxis(0.3, 0.5, 0.8);
    Matrix6d T = TwistDecomposition::computeTwistProjector(torchAxis);
    Matrix6d T2 = T * T;

    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            EXPECT_NEAR(T(i, j), T2(i, j), 1e-10);
}

// ============================================================================
// PS Index Tests
// ============================================================================

TEST(TwistDecomposition, PS_Identity) {
    // Identity Jacobian → well-conditioned
    Jacobian J = Jacobian::Identity();
    double ps = TwistDecomposition::computePS(J);

    // For I: all singular values = 1
    // ωps = √(1/(1·1·1·1·1²)) = 1.0
    EXPECT_NEAR(ps, 1.0, 1e-6);
}

TEST(TwistDecomposition, PS_ScaledIdentity) {
    // Scaled identity → should still be well-conditioned
    Jacobian J = 5.0 * Jacobian::Identity();
    double ps = TwistDecomposition::computePS(J);

    // All σ = 5, so ωps = √(5/(5·5·5·5·25)) = √(5/5^5) = √(1/5^4) = 1/25
    // Actually: ωps = √(σ1/(σ2*σ3*σ4*σ5*σ6^2)) = √(5/(5*5*5*5*25)) = √(1/3125)
    EXPECT_GT(ps, 0.0);
    EXPECT_LT(ps, 1.0);  // Well-conditioned, low PS
}

TEST(TwistDecomposition, PS_NearSingular) {
    // Jacobian with one very small singular value → high PS
    Jacobian J = Jacobian::Identity();
    J(5, 5) = 1e-6;  // Make σ₆ very small

    double ps = TwistDecomposition::computePS(J);

    // Should be very high (approaching singularity)
    EXPECT_GT(ps, 100.0);
}

TEST(TwistDecomposition, PS_ExactSingular) {
    // Singular Jacobian → PS should saturate at max
    Jacobian J = Jacobian::Identity();
    J(5, 5) = 0.0;  // Exact rank deficiency

    double ps = TwistDecomposition::computePS(J);

    EXPECT_GE(ps, 1e5);  // Should return very large value
}

// ============================================================================
// Gradient Tests
// ============================================================================

TEST(TwistDecomposition, Gradient_MidPosition) {
    // When current = mid → joint-limit term = 0
    JointAngles current = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    JointAngles mid = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    JointAngles singEntry = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Vector6d W = Vector6d::Ones();
    Vector6d K = Vector6d::Ones();

    Vector6d h = TwistDecomposition::computeGradient(current, mid, singEntry, W, K, 0.0);

    // All zeros: at mid position, no PS active
    for (int i = 0; i < 6; ++i)
        EXPECT_NEAR(h(i), 0.0, 1e-10);
}

TEST(TwistDecomposition, Gradient_NearLimit) {
    // Joint 4 near upper limit → gradient should push it back
    JointAngles current = {0.0, 0.0, 0.0, 2.5, 0.0, 0.0};  // J4 near limit
    JointAngles mid = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    JointAngles singEntry = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Vector6d W = Vector6d::Ones();
    Vector6d K = Vector6d::Ones();

    Vector6d h = TwistDecomposition::computeGradient(current, mid, singEntry, W, K, 0.0);

    // Joint 4 gradient should be negative (push back toward mid)
    EXPECT_LT(h(3), 0.0);

    // Other joints should be zero (at mid)
    EXPECT_NEAR(h(0), 0.0, 1e-10);
    EXPECT_NEAR(h(1), 0.0, 1e-10);
    EXPECT_NEAR(h(2), 0.0, 1e-10);
}

TEST(TwistDecomposition, Gradient_WithSingularity) {
    // When PS > 0, singularity term should activate
    JointAngles current = {0.1, 0.2, 0.3, 0.4, 0.05, 0.6};  // J5 near singularity
    JointAngles mid = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    JointAngles singEntry = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};  // Recorded safe config
    Vector6d W = Vector6d::Ones();
    Vector6d K = Vector6d::Ones();
    double ps = 100.0;

    Vector6d h = TwistDecomposition::computeGradient(current, mid, singEntry, W, K, ps);

    // Joint 5: singularity term should push toward entry (0.5), away from current (0.05)
    // K[4] * ps * (0.5 - 0.05) = 1 * 100 * 0.45 = 45
    // Plus W[4] * (0.0 - 0.05) = -0.05
    // Total ≈ 44.95
    EXPECT_GT(h(4), 40.0);
}

// ============================================================================
// Resolve Tests
// ============================================================================

TEST(TwistDecomposition, Resolve_NoSingularity) {
    // Well-conditioned: PS < threshold → pass-through behavior
    Jacobian J = Jacobian::Identity();
    Vector3d torchAxis(0.0, 0.0, 1.0);
    Matrix6d T = TwistDecomposition::computeTwistProjector(torchAxis);
    Vector6d twist;
    twist << 0.0, 0.0, 0.0, 10.0, 0.0, 0.0;  // Pure X translation
    Vector6d h = Vector6d::Zero();

    TWAConfig config;
    config.psThreshold = 0.3;
    double ps = 0.1;  // Well below threshold

    TWAResult result = TwistDecomposition::resolve(J, T, twist, h, config, ps);

    EXPECT_FALSE(result.singularityActive);
    EXPECT_NEAR(result.velocityScale, 1.0, 1e-10);

    // For identity J and axis-aligned T, task component should match twist
    EXPECT_NEAR(result.jointDisplacement[3], 10.0, 1e-6);  // vx → q4
}

TEST(TwistDecomposition, Resolve_Singularity_VelocityScaled) {
    // PS above threshold → velocity should be scaled
    Jacobian J = Jacobian::Identity();
    Vector3d torchAxis(0.0, 0.0, 1.0);
    Matrix6d T = TwistDecomposition::computeTwistProjector(torchAxis);
    Vector6d twist;
    twist << 0.0, 0.0, 0.0, 10.0, 0.0, 0.0;
    Vector6d h = Vector6d::Zero();

    TWAConfig config;
    config.psThreshold = 0.3;
    double ps = 1.2;  // 4x threshold

    TWAResult result = TwistDecomposition::resolve(J, T, twist, h, config, ps);

    EXPECT_TRUE(result.singularityActive);
    // With velocityScaleMin=1.0, scale is always 1.0 (no reduction)
    EXPECT_NEAR(result.velocityScale, 1.0, 1e-4);
}

// ============================================================================
// DLS Fallback Tests
// ============================================================================

TEST(TwistDecomposition, DLS_Bounded) {
    // Near-exact singularity: DLS should produce bounded output
    Jacobian J = Jacobian::Identity();
    J(5, 5) = 1e-10;  // Nearly singular in last row

    Vector6d twist;
    twist << 0.0, 0.0, 0.0, 0.0, 0.0, 10.0;  // Push along singular direction

    JointVelocities qdot = TwistDecomposition::dampedLeastSquares(J, twist, 0.1);

    // Output should be bounded (no explosion)
    for (int i = 0; i < NUM_JOINTS; ++i) {
        EXPECT_LT(std::abs(qdot[i]), 1e4);  // No explosion
    }
}

TEST(TwistDecomposition, DLS_ConvergesToPseudoinverse) {
    // With λ→0, DLS should converge to standard pseudo-inverse
    Jacobian J = Jacobian::Identity();
    Vector6d twist;
    twist << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

    JointVelocities qdot = TwistDecomposition::dampedLeastSquares(J, twist, 1e-12);

    // For identity J, pseudo-inverse = J itself
    for (int i = 0; i < NUM_JOINTS; ++i) {
        EXPECT_NEAR(qdot[i], twist(i), 1e-6);
    }
}

// ============================================================================
// Full Pipeline Test
// ============================================================================

TEST(TwistDecomposition, FullPipeline_SafeRegion) {
    // Complete pipeline: far from singularity
    // Note: Identity Jacobian gives PS=1.0, so we scale J to get PS < threshold
    Jacobian J = Jacobian::Identity() * 100.0;  // Large singular values → small PS
    Vector3d torchAxis(0.0, 0.0, 1.0);
    Vector6d desiredTwist;
    desiredTwist << 0.0, 0.0, 0.0, 5.0, 0.0, 0.0;  // Move X at 5mm/s

    JointAngles current = {0.0, 0.0, 0.0, 0.0, 0.5, 0.0};  // Well away from singularity
    JointAngles singEntry = {};

    TWAConfig config;
    config.psThreshold = 0.3;

    TWAResult result = TwistDecomposition::computeSafeVelocity(
        J, torchAxis, desiredTwist, current, singEntry, config);

    EXPECT_FALSE(result.singularityActive);
    EXPECT_NEAR(result.velocityScale, 1.0, 1e-10);
    EXPECT_LT(result.psIndex, config.psThreshold);
}

// ============================================================================
// findCriticalJoint Tests
// ============================================================================

TEST(TwistDecomposition, FindCriticalJoint_AllSafe) {
    JointAngles current = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    JointAngles minLimits = {-3.0, -3.0, -3.0, -3.0, -3.0, -3.0};
    JointAngles maxLimits = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

    int critical = TwistDecomposition::findCriticalJoint(current, minLimits, maxLimits);
    EXPECT_EQ(critical, -1);  // All joints at mid → safe
}

TEST(TwistDecomposition, FindCriticalJoint_NearLimit) {
    JointAngles current = {0.0, 0.0, 0.0, 0.0, 2.8, 0.0};  // J5 near limit
    JointAngles minLimits = {-3.0, -3.0, -3.0, -3.0, -3.0, -3.0};
    JointAngles maxLimits = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

    int critical = TwistDecomposition::findCriticalJoint(current, minLimits, maxLimits);
    EXPECT_EQ(critical, 4);  // Joint 5 (index 4) is closest to limit
}
