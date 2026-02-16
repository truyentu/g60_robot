/**
 * @file AnalyticalKinematics.cpp
 * @brief Analytical (closed-form) FK/IK implementation for Yaskawa MA2010
 *
 * DH Convention: Standard DH
 *   T_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
 *
 * MA2010 DH Parameters (verified against URDF FK):
 *   | Joint | a(mm) | α(deg) | d(mm) | θ = sign*q + offset |
 *   |-------|-------|--------|-------|---------------------|
 *   | 1     | 150   | -90    | 505   | +q1                 |
 *   | 2     | 760   |   0    |   0   | +q2 - 90°           |
 *   | 3     | 200   | -90    |   0   | -q3                 |
 *   | 4     |   0   |  90    | 1082  | -q4                 |
 *   | 5     |   0   | -90    |   0   | -q5                 |
 *   | 6     |   0   |   0    | 100   | -q6                 |
 *
 * IK uses Position/Orientation Decoupling (Pieper's method):
 *   Step 1: Wrist Center = P_target - d6 * R * [0,0,1]^T
 *   Step 2: Solve θ1, θ2, θ3 from wrist center (geometric)
 *   Step 3: R_36 = R_03^T * R_06
 *   Step 4: Solve θ4, θ5, θ6 from ZYZ Euler extraction of R_36
 */

#include "AnalyticalKinematics.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_controller {
namespace kinematics {
namespace analytical {

// ============================================================================
// Constructor — Hard-code MA2010 DH Parameters
// ============================================================================

AnalyticalKinematics::AnalyticalKinematics() {
    // MA2010 Standard DH: a, alpha, d, theta_offset, sign, q_min, q_max
    //
    // Joint limits from URDF (radians):
    //   J1: [-3.1416, +3.1416]  = ±180°
    //   J2: [-1.8326, +2.7052]  = [-105°, +155°]
    //   J3: [-1.5009, +2.7925]  = [-86°, +160°]
    //   J4: [-2.6180, +2.6180]  = ±150°
    //   J5: [-2.3562, +1.5708]  = [-135°, +90°]
    //   J6: [-3.6652, +3.6652]  = ±210°

    dh_[0] = { 150.0, -90.0 * DEG2RAD, 505.0,   0.0,          +1.0, -3.1416,  3.1416 };
    dh_[1] = { 760.0,   0.0,             0.0,  -90.0 * DEG2RAD, +1.0, -1.8326,  2.7052 };
    dh_[2] = { 200.0, -90.0 * DEG2RAD,   0.0,   0.0,          -1.0, -1.5009,  2.7925 };
    dh_[3] = {   0.0,  90.0 * DEG2RAD, 1082.0,   0.0,          -1.0, -2.6180,  2.6180 };
    dh_[4] = {   0.0, -90.0 * DEG2RAD,   0.0,   0.0,          -1.0, -2.3562,  1.5708 };
    dh_[5] = {   0.0,   0.0,           100.0,   0.0,          -1.0, -3.6652,  3.6652 };

    // Tool frame correction: DH frame 6 → URDF tool frame
    // At home, DH frame 6 has Z pointing in world X, X in world Z, Y in world -Y
    // URDF tool frame at home is identity
    // Correction: rotate to align
    // T_tool = Ry(+90°) * Rz(180°) maps DH_frame6 to URDF_tool
    // Actually, we embed d6 in the DH chain (as d6=100), so no separate tool transform needed.
    // The tool correction is only for orientation mapping if user provides pose in URDF convention.
    //
    // For now, FK output is in DH frame convention.
    // IK input must match this convention.
    T_tool_correction_ = Eigen::Matrix4d::Identity();
}

// ============================================================================
// DH Transformation Matrix
// ============================================================================

Eigen::Matrix4d AnalyticalKinematics::dhTransform(
    double theta, double d, double a, double alpha)
{
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    Eigen::Matrix4d T;
    T << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
          0,       sa,       ca,      d,
          0,        0,        0,      1;
    return T;
}

// ============================================================================
// URDF ↔ DH Angle Conversion
// ============================================================================

std::array<double, NUM_JOINTS> AnalyticalKinematics::urdfToDH(
    const JointAngles& q) const
{
    std::array<double, NUM_JOINTS> dh_theta;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        dh_theta[i] = dh_[i].sign * q[i] + dh_[i].theta_offset;
    }
    return dh_theta;
}

JointAngles AnalyticalKinematics::dhToUrdf(
    const std::array<double, NUM_JOINTS>& dh_theta) const
{
    JointAngles q;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        // dh_theta = sign * q + offset  →  q = (dh_theta - offset) / sign
        q[i] = (dh_theta[i] - dh_[i].theta_offset) / dh_[i].sign;
    }
    return q;
}

// ============================================================================
// Normalize angle to [-PI, PI]
// ============================================================================

double AnalyticalKinematics::normalizeAngle(double a) {
    while (a > PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

// ============================================================================
// Joint Limits Check
// ============================================================================

bool AnalyticalKinematics::isWithinJointLimits(const JointAngles& q) const {
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (q[i] < dh_[i].q_min || q[i] > dh_[i].q_max) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// Forward Kinematics
// ============================================================================

std::array<Eigen::Matrix4d, 7> AnalyticalKinematics::computeAllTransforms(
    const JointAngles& q) const
{
    auto dh_theta = urdfToDH(q);

    std::array<Eigen::Matrix4d, 7> T;

    // T[0] = T_01
    T[0] = dhTransform(dh_theta[0], dh_[0].d, dh_[0].a, dh_[0].alpha);

    // T[1] = T_02 = T_01 * T_12
    T[1] = T[0] * dhTransform(dh_theta[1], dh_[1].d, dh_[1].a, dh_[1].alpha);

    // T[2] = T_03
    T[2] = T[1] * dhTransform(dh_theta[2], dh_[2].d, dh_[2].a, dh_[2].alpha);

    // T[3] = T_04
    T[3] = T[2] * dhTransform(dh_theta[3], dh_[3].d, dh_[3].a, dh_[3].alpha);

    // T[4] = T_05
    T[4] = T[3] * dhTransform(dh_theta[4], dh_[4].d, dh_[4].a, dh_[4].alpha);

    // T[5] = T_06 (includes d6=100, so this is the tool tip in DH convention)
    T[5] = T[4] * dhTransform(dh_theta[5], dh_[5].d, dh_[5].a, dh_[5].alpha);

    // T[6] = T_tcp (with any additional tool correction)
    T[6] = T[5] * T_tool_correction_;

    return T;
}

Pose AnalyticalKinematics::computeFK(const JointAngles& q) const {
    auto transforms = computeAllTransforms(q);
    return Pose::fromTransform(transforms[6]);
}

// ============================================================================
// Inverse Kinematics — Position Sub-Problem (θ1, θ2, θ3)
//
// Method: HP20 paper (Fei & Huang 2016) adapted for MA2010.
//
// Given wrist center (wc), solve for DH θ1, θ2, θ3.
//
// Key equations (projected into arm plane after solving θ1):
//   c2·h1 + s2·pz = h3    where h1 = r, pz = s (arm plane coords)
//  -s2·h1 + c2·pz = h4    h3 = a3·c3 + d4·s3 + a2 (MA2010 specific)
//                          h4 = a3·s3 - d4·c3
//
// NOTE: For MA2010, alpha1 = -90° (negative), so the z-axis of frame 1
// points along [-sin(θ1), cos(θ1), 0] in world frame.
// Positive θ2 rotates the arm from +r toward -s in the arm plane.
// The arm plane equations are therefore:
//   -s2·h1 + c2·pz = h3   (note the sign swap vs HP20's alpha=+90°)
//    c2·h1 + s2·pz = h4
// ============================================================================

std::vector<std::array<double, 3>> AnalyticalKinematics::solvePosition(
    const Eigen::Vector3d& wc) const
{
    // DH parameters for arm
    const double a1 = dh_[0].a;    // 150
    const double d1 = dh_[0].d;    // 505
    const double a2 = dh_[1].a;    // 760
    const double a3 = dh_[2].a;    // 200
    const double d4 = dh_[3].d;    // 1082

    std::vector<std::array<double, 3>> results;

    // ---- Solve θ1 (two solutions: shoulder left / right) ----
    double theta1_options[2];
    int n_theta1 = 0;

    double wc_x = wc.x();
    double wc_y = wc.y();
    double r_xy = std::sqrt(wc_x * wc_x + wc_y * wc_y);

    if (r_xy < 1e-6) {
        theta1_options[0] = 0.0;
        n_theta1 = 1;
    } else {
        theta1_options[0] = std::atan2(wc_y, wc_x);
        theta1_options[1] = normalizeAngle(theta1_options[0] + PI);
        n_theta1 = 2;
    }

    for (int i1 = 0; i1 < n_theta1; ++i1) {
        double th1 = theta1_options[i1];
        double c1 = std::cos(th1);
        double s1 = std::sin(th1);

        // ---- Project wrist center into arm plane ----
        // h1 = horizontal distance in arm plane, minus shoulder offset a1
        // pz = vertical distance from base
        double h1 = c1 * wc_x + s1 * wc_y - a1;
        double pz = wc.z() - d1;

        // ---- Solve θ3 (cosine law) ----
        // Distance squared from J2 to WC in the arm plane
        double D_sq = h1 * h1 + pz * pz;

        // Effective forearm length
        double L_forearm = std::sqrt(a3 * a3 + d4 * d4);  // √(200² + 1082²) ≈ 1100.3

        // Cosine law: D² = a2² + L² - 2·a2·L·cos(π - elbow_angle)
        // or equivalently: cos(elbow_angle) = (a2² + L² - D²) / (2·a2·L)
        double cos_elbow = (a2 * a2 + L_forearm * L_forearm - D_sq) / (2.0 * a2 * L_forearm);

        if (std::abs(cos_elbow) > 1.0 + 1e-8) continue;
        cos_elbow = std::clamp(cos_elbow, -1.0, 1.0);

        double elbow_angle = std::acos(cos_elbow);  // internal angle at elbow

        // The offset angle between the a3 direction and the total forearm vector
        double zeta = std::atan2(d4, a3);  // atan2(1082, 200) ≈ 79.52°

        // θ3 is the DH rotation at joint 3.
        // At home (θ3=0), the forearm makes angle zeta with the a3-axis direction.
        // The elbow angle relates to θ3 through: elbow_angle = π - (θ3 + zeta)
        // → θ3 = π - elbow_angle - zeta  OR  θ3 = -(π - elbow_angle) - zeta + 2π
        //
        // Two elbow solutions:
        double theta3_options[2];
        theta3_options[0] = PI - elbow_angle - zeta;         // elbow up
        theta3_options[1] = -(PI - elbow_angle) - zeta;      // elbow down = elbow_angle + zeta - π

        for (int i3 = 0; i3 < 2; ++i3) {
            double th3 = theta3_options[i3];
            double c3 = std::cos(th3);
            double s3 = std::sin(th3);

            // ---- Solve θ2 from simultaneous equations ----
            // For Standard DH with alpha1 = -90°, the arm-plane equations are:
            //
            // From the FK chain: position of WC relative to J2 origin in arm plane:
            //   wc_r = a2·c2 + a3·c23 + d4·s23    (along r-axis)
            //   wc_s = a2·s2 + a3·s23 - d4·c23    (along s-axis... wait, need to derive properly)
            //
            // Actually, let's use the HP20 approach directly:
            //   h3 = a3·c3 + d4·s3 + a2   = position contribution along the arm-extended direction
            //   h4 = a3·s3 - d4·c3         = position contribution perpendicular to arm
            //
            // But for alpha1=-90° (MA2010), the z1 axis points in [-sin(th1), cos(th1), 0]
            // which means positive θ2 rotation takes the arm from +r toward -s.
            //
            // This gives (deriving from T01*T02*T03 position):
            //   h1 =  c2·(a3·c3 + d4·s3 + a2) + s2·(a3·s3 - d4·c3)
            //   pz = -s2·(a3·c3 + d4·s3 + a2) + c2·(a3·s3 - d4·c3)
            //
            // Wait, that's wrong too. Let me derive correctly.
            //
            // From FK: T02 position in arm plane:
            //   r_J3 = a2·cos(th2_dh)
            //   s_J3 = a2·sin(th2_dh)  (but sign depends on alpha1!)
            //
            // For alpha1=-90°, frame 1 has z1 pointing "into" the arm plane
            // such that positive theta2 goes +r → -s.
            // So if we define the arm-plane angle phi_arm measured from +r CCW:
            //   phi_arm = -th2_dh
            //   r_J3 = a2·cos(-th2_dh) = a2·cos(th2_dh)
            //   s_J3 = a2·sin(-th2_dh) = -a2·sin(th2_dh)
            //
            // Combined with link 3 contribution (at angle th2+th3 from frame 2):
            //   r_wc = a2·c2 + a3·c(2+3) + d4·s(2+3) ... but with the -s sign...
            //
            // OK, let me use a cleaner approach.
            //
            // Define auxiliary variables (HP20 paper style):
            double h3 = a2 + a3 * c3 - d4 * s3;
            double h4 = -(a3 * s3 + d4 * c3);

            // The arm plane equations for alpha1 = -90° are:
            //   h1 =  c2 * h3 + s2 * h4    ... (A)
            //   pz = -s2 * h3 + c2 * h4    ... (B)
            //
            // (Derived from T01*T12*T23 position projection into arm plane,
            //  with alpha1=-90° causing the sign pattern)
            //
            // Solving for c2, s2:
            //   From (A): h1 = c2·h3 + s2·h4
            //   From (B): pz = -s2·h3 + c2·h4
            //   Determinant = h3·h4 - h4·(-h3) = h3² + h4² (= D_sq, never zero for reachable)
            //   c2 = (h1·h3 + pz·h4) / (h3² + h4²)
            //   s2 = (h1·h4 - pz·h3) / (h3² + h4²)
            double denom = h3 * h3 + h4 * h4;
            if (denom < 1e-10) continue;  // degenerate (shouldn't happen)

            double th2 = std::atan2(h1 * h4 - pz * h3, h1 * h3 + pz * h4);

            results.push_back({th1, th2, th3});
        }
    }

    return results;
}

// ============================================================================
// Inverse Kinematics — Orientation Sub-Problem (θ4, θ5, θ6)
// ============================================================================

std::vector<std::array<double, 3>> AnalyticalKinematics::solveOrientation(
    const Eigen::Matrix3d& R_36,
    std::optional<double> th4_ref) const
{
    // DH chain: R_36 = Rz(θ4) * Rx(+π/2) * Rz(θ5) * Rx(-π/2) * Rz(θ6)
    //         = Rz(θ4) * Ry(-θ5) * Rz(θ6)
    //
    // This is ZYZ Euler with t5_eff = -θ5 (sign flip from Rx conjugation).
    // Extract as ZYZ: get t4, t5_eff, t6, then negate t5_eff to get θ5_dh.
    //
    // Matrix elements (with t5_eff = -θ5):
    //   R_36(0,2) = -c4·s5      R_36(1,2) = -s4·s5      R_36(2,2) = c5
    //   R_36(2,0) = s5·c6       R_36(2,1) = -s5·s6
    //
    // Extraction: θ4 = atan2(r23, r13) [correct: -s5 cancels]
    //             θ6 = atan2(r32, -r31) [correct: s5_eff cancels]
    //             θ5_dh = -atan2(|s5_eff|, c5)  or flip

    std::vector<std::array<double, 3>> results;

    double r13 = R_36(0, 2);
    double r23 = R_36(1, 2);
    double r33 = R_36(2, 2);
    double r31 = R_36(2, 0);
    double r32 = R_36(2, 1);

    // Solve θ5
    double s5_sq = r13 * r13 + r23 * r23;
    double s5 = std::sqrt(s5_sq);

    if (s5 < 1e-6) {
        // ---- Wrist Singularity: θ5 ≈ 0 or π ----
        // Only (θ4 + θ6) or (θ4 - θ6) is determinable.
        // Instead of forcing θ4 = 0 (which causes joint discontinuity),
        // generate multiple candidate solutions so computeIKNearest
        // can pick the one closest to current configuration.

        double th5;
        double total;  // the determinable combined angle

        if (r33 > 0) {
            // t5_eff ≈ 0, so θ5_dh ≈ 0 → R_36 ≈ Rz(θ4 + θ6)
            th5 = 0.0;
            total = std::atan2(R_36(1, 0), R_36(0, 0));  // θ4 + θ6
        } else {
            // t5_eff ≈ π, so θ5_dh ≈ -π
            th5 = -PI;
            total = std::atan2(-R_36(1, 0), -R_36(0, 0));  // θ4 - θ6
        }

        if (th4_ref.has_value()) {
            // Use reference θ4 for continuity: keep θ4 = ref, solve θ6
            double th4 = th4_ref.value();
            double th6 = (r33 > 0) ? (total - th4) : (total - th4);
            // For th5=-π case: total = th4 - th6, so th6 = th4 - total
            if (r33 <= 0) {
                th6 = th4 - total;
            }
            results.push_back({th4, th5, th6});
        } else {
            // No reference: generate candidate solutions at multiple θ4 values
            // so computeIKNearest can choose the nearest one
            double candidates[] = {0.0, PI / 2, -PI / 2, PI, -PI};
            for (double th4 : candidates) {
                double th6;
                if (r33 > 0) {
                    th6 = total - th4;
                } else {
                    th6 = th4 - total;
                }
                results.push_back({th4, th5, th6});
            }
        }

        return results;
    }

    // ---- Normal case: sin(θ5) ≠ 0 ----
    // Two solutions: θ5 positive and θ5 negative (wrist flip / no-flip)

    for (int flip = 0; flip < 2; ++flip) {
        double th5, th4, th6;

        if (flip == 0) {
            // s5_eff > 0 → θ5_dh < 0
            double t5_eff = std::atan2(s5, r33);
            th5 = -t5_eff;  // Negate: actual DH θ5 = -t5_eff
            th4 = std::atan2(r23, r13);
            th6 = std::atan2(r32, -r31);
        } else {
            // s5_eff < 0 → θ5_dh > 0
            double t5_eff = std::atan2(-s5, r33);
            th5 = -t5_eff;  // Negate: actual DH θ5 = -t5_eff
            th4 = std::atan2(-r23, -r13);
            th6 = std::atan2(-r32, r31);
        }

        results.push_back({th4, th5, th6});
    }

    return results;
}

// ============================================================================
// Full Inverse Kinematics
// ============================================================================

IKSolution AnalyticalKinematics::computeIK(const Pose& target) const {
    return computeIKInternal(target, nullptr);
}

IKSolution AnalyticalKinematics::computeIKInternal(
    const Pose& target, const JointAngles* q_ref) const {
    IKSolution result;

    const double d6 = dh_[5].d;  // 100mm (tool flange)

    // ================================================================
    // Step 1: Compute Wrist Center
    // ================================================================
    // p_wc = p_target - d6 * R_target * [0, 0, 1]^T
    //
    // The approach vector is the Z-column of the target rotation matrix
    // (in DH frame convention, tool Z points along the tool axis).
    Eigen::Matrix3d R_target = target.rotationMatrix();
    Eigen::Vector3d approach = R_target.col(2);  // Z column
    Eigen::Vector3d wc = target.position - d6 * approach;

    // ================================================================
    // Step 2: Solve Position (θ1, θ2, θ3) — up to 4 solutions
    // ================================================================
    auto pos_solutions = solvePosition(wc);

    if (pos_solutions.empty()) {
        return result;  // Unreachable
    }

    // ================================================================
    // Step 3 & 4: For each position solution, solve orientation
    // ================================================================
    for (const auto& pos_sol : pos_solutions) {
        double th1_dh = pos_sol[0];
        double th2_dh = pos_sol[1];
        double th3_dh = pos_sol[2];

        // Compute R_03 from DH transforms
        Eigen::Matrix4d T01 = dhTransform(th1_dh, dh_[0].d, dh_[0].a, dh_[0].alpha);
        Eigen::Matrix4d T12 = dhTransform(th2_dh, dh_[1].d, dh_[1].a, dh_[1].alpha);
        Eigen::Matrix4d T23 = dhTransform(th3_dh, dh_[2].d, dh_[2].a, dh_[2].alpha);
        Eigen::Matrix3d R_03 = (T01 * T12 * T23).block<3, 3>(0, 0);

        // R_36 = R_03^T * R_06
        // R_06 = target rotation (in DH convention)
        Eigen::Matrix3d R_36 = R_03.transpose() * R_target;

        // Solve orientation — pass th4 reference for singularity continuity
        std::optional<double> th4_ref_dh = std::nullopt;
        if (q_ref) {
            // Convert q_ref (URDF) to DH for joint 4
            auto dh_ref = urdfToDH(*q_ref);
            th4_ref_dh = dh_ref[3];
        }
        auto ori_solutions = solveOrientation(R_36, th4_ref_dh);

        for (const auto& ori_sol : ori_solutions) {
            double th4_dh = ori_sol[0];
            double th5_dh = ori_sol[1];
            double th6_dh = ori_sol[2];

            // Convert DH angles back to URDF convention
            std::array<double, NUM_JOINTS> dh_angles = {
                th1_dh, th2_dh, th3_dh, th4_dh, th5_dh, th6_dh
            };
            JointAngles q_urdf = dhToUrdf(dh_angles);

            // Normalize all angles to [-π, π]
            for (int i = 0; i < NUM_JOINTS; ++i) {
                q_urdf[i] = normalizeAngle(q_urdf[i]);
            }

            // Filter: REJECT if any joint is out of limits (NO clamping!)
            if (!isWithinJointLimits(q_urdf)) {
                continue;
            }

            result.solutions.push_back(q_urdf);
        }
    }

    return result;
}

// ============================================================================
// IK Nearest to Reference
// ============================================================================

std::optional<JointAngles> AnalyticalKinematics::computeIKNearest(
    const Pose& target, const JointAngles& q_ref) const
{
    auto ik = computeIKInternal(target, &q_ref);
    if (!ik.hasSolution()) {
        return std::nullopt;
    }

    // Find solution with minimum weighted joint travel
    double best_cost = std::numeric_limits<double>::max();
    const JointAngles* best = nullptr;

    for (const auto& sol : ik.solutions) {
        double cost = 0.0;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            double diff = normalizeAngle(sol[i] - q_ref[i]);
            // Weight: larger joints (1-3) have more inertia → higher cost
            double w = (i < 3) ? 1.0 : 0.5;
            cost += w * diff * diff;
        }
        if (cost < best_cost) {
            best_cost = cost;
            best = &sol;
        }
    }

    return best ? std::optional<JointAngles>(*best) : std::nullopt;
}

// ============================================================================
// Joint Limit Cost Function
// ============================================================================

double AnalyticalKinematics::jointLimitCost(const JointAngles& q) const {
    double cost = 0.0;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        double q_mid = 0.5 * (dh_[i].q_min + dh_[i].q_max);
        double q_range = dh_[i].q_max - dh_[i].q_min;
        if (q_range < 1e-10) continue;
        double normalized = (q[i] - q_mid) / q_range;
        cost += normalized * normalized;
    }
    return cost;
}

// ============================================================================
// Welding IK — Redundancy Resolution via Tool Z Rotation
// ============================================================================

WeldingIKResult AnalyticalKinematics::computeWeldingIK(
    const Pose& target, const JointAngles& currentJoints) const
{
    WeldingIKResult best;
    best.valid = false;
    best.jointLimitCost = std::numeric_limits<double>::max();

    // Coarse sampling: 13 values of ψ (rotation around tool Z)
    // 0°, ±30°, ±60°, ±90°, ±120°, ±150°, 180°
    constexpr int NUM_PSI_SAMPLES = 13;
    constexpr double PSI_SAMPLES[NUM_PSI_SAMPLES] = {
        0.0,
         30.0 * DEG2RAD,  -30.0 * DEG2RAD,
         60.0 * DEG2RAD,  -60.0 * DEG2RAD,
         90.0 * DEG2RAD,  -90.0 * DEG2RAD,
        120.0 * DEG2RAD, -120.0 * DEG2RAD,
        150.0 * DEG2RAD, -150.0 * DEG2RAD,
        180.0 * DEG2RAD, -180.0 * DEG2RAD
    };

    Eigen::Matrix3d R_target = target.rotationMatrix();

    for (int i = 0; i < NUM_PSI_SAMPLES; ++i) {
        double psi = PSI_SAMPLES[i];

        // Rotate target around its own Z axis by ψ
        // R_rotated = R_target * Rz(ψ)
        double cp = std::cos(psi);
        double sp = std::sin(psi);
        Eigen::Matrix3d Rz_psi;
        Rz_psi << cp, -sp, 0,
                  sp,  cp, 0,
                   0,   0, 1;

        Pose rotatedTarget;
        rotatedTarget.position = target.position;
        rotatedTarget.orientation = Eigen::Quaterniond(R_target * Rz_psi);
        rotatedTarget.orientation.normalize();

        // Solve IK for this rotated target
        auto ik = computeIK(rotatedTarget);
        if (!ik.hasSolution()) continue;

        // Find nearest solution to current joints
        for (const auto& sol : ik.solutions) {
            double jlCost = jointLimitCost(sol);

            // Also factor in joint travel from current position (small weight)
            double travelCost = 0.0;
            for (int j = 0; j < NUM_JOINTS; ++j) {
                double diff = normalizeAngle(sol[j] - currentJoints[j]);
                travelCost += diff * diff;
            }

            // Combined cost: joint limit avoidance (primary) + travel (secondary)
            double totalCost = jlCost + 0.1 * travelCost;

            if (totalCost < best.jointLimitCost) {
                best.joints = sol;
                best.psi = psi;
                best.jointLimitCost = totalCost;
                best.manipulability = manipulabilityMeasure(sol);
                best.valid = true;
            }
        }
    }

    return best;
}

// ============================================================================
// Analytical Jacobian
// ============================================================================

Eigen::Matrix<double, 6, 6> AnalyticalKinematics::computeJacobian(
    const JointAngles& q) const
{
    auto transforms = computeAllTransforms(q);

    Eigen::Matrix<double, 6, 6> J;
    J.setZero();

    // TCP position
    Eigen::Vector3d p_tcp = transforms[6].block<3, 1>(0, 3);

    // Base frame (identity for joint 1)
    // For each joint i, the Jacobian column is:
    //   J_v_i = z_i × (p_tcp - p_i)   (linear velocity)
    //   J_w_i = z_i                     (angular velocity)
    //
    // z_i = Z-axis of frame i-1 (the rotation axis of joint i)
    // p_i = origin of frame i-1

    // Frame 0 = base
    Eigen::Vector3d z_prev = Eigen::Vector3d::UnitZ();  // z0 = world Z
    Eigen::Vector3d p_prev = Eigen::Vector3d::Zero();    // p0 = world origin

    // However, DH joints may have sign inversions and offsets.
    // We need z-axis of frame i-1 and origin of frame i-1.
    // The transforms array gives T_0i, so:
    //   z_{i-1} = T_{0,i-1}.col(2).head(3)  (Z column of frame i-1)
    //   p_{i-1} = T_{0,i-1}.col(3).head(3)  (origin of frame i-1)
    //
    // But we also need to account for the DH sign convention.
    // The actual joint rotation axis in world frame incorporates the sign.

    for (int i = 0; i < NUM_JOINTS; ++i) {
        Eigen::Vector3d z_i, p_i;

        if (i == 0) {
            z_i = Eigen::Vector3d::UnitZ();  // base Z
            p_i = Eigen::Vector3d::Zero();    // base origin
        } else {
            // Frame i-1 (DH convention)
            z_i = transforms[i - 1].block<3, 1>(0, 2);  // Z-column of frame i-1
            p_i = transforms[i - 1].block<3, 1>(0, 3);  // origin of frame i-1
        }

        // DH joint sign: the actual rotation rate is dh_sign * dq
        // So the effective z-axis for the Jacobian includes the sign
        Eigen::Vector3d z_effective = dh_[i].sign * z_i;

        // Linear velocity: z × (p_tcp - p)
        J.block<3, 1>(0, i) = z_effective.cross(p_tcp - p_i);

        // Angular velocity: z
        J.block<3, 1>(3, i) = z_effective;
    }

    return J;
}

// ============================================================================
// Singularity Detection
// ============================================================================

bool AnalyticalKinematics::isSingularity(const JointAngles& q) const {
    // Primary check: wrist singularity (fastest)
    // sin(θ5_urdf) → in DH: th5_dh = -q5, so sin(th5_dh) = -sin(q5)
    // |sin(θ5)| < threshold
    if (std::abs(std::sin(q[4])) < 0.02) {  // ~1.15° from singularity
        return true;
    }

    // Secondary check: determinant of Jacobian
    double m = manipulabilityMeasure(q);
    return m < 1e-3;
}

double AnalyticalKinematics::manipulabilityMeasure(const JointAngles& q) const {
    auto J = computeJacobian(q);
    // Yoshikawa manipulability: sqrt(det(J * J^T))
    Eigen::Matrix<double, 6, 6> JJT = J * J.transpose();
    double det = JJT.determinant();
    return std::sqrt(std::abs(det));
}

} // namespace analytical
} // namespace kinematics
} // namespace robot_controller
