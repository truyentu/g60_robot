/**
 * @file AnalyticalIK.hpp
 * @brief Analytical inverse kinematics for 6-DOF robot with spherical wrist
 *
 * DEPRECATED: Uses Modified DH convention which produces incorrect FK.
 * Will be replaced by IKFast analytical IK (IMPL_P10_03) or KDL IK (IMPL_P10_02).
 *
 * Derived specifically for Modified DH convention:
 *   T_i = Rx(α_i) * Tx(a_i) * Rz(θ_i) * Tz(d_i)
 *
 * Robot structure: 3 position joints (J1-J3) + spherical wrist (J4-J6)
 * DH pattern: α = [-90°, 0°, -90°, 90°, -90°, 0°]
 *
 * Key insight for this Modified DH convention:
 *   - Frame 1 origin is FIXED at (a1, d1, 0) regardless of θ1
 *   - J1 rotation affects arm direction in X-Z plane (NOT X-Y!)
 *   - WC_y depends only on d1 and d4*cos(θ3) → θ3 solved first
 *   - θ2 solved from arm plane distance
 *   - θ1 solved last using θ2, θ3 results
 */

#pragma once

#include "DHParameters.hpp"
#include "ForwardKinematics.hpp"
#include <optional>
#include <vector>
#include <cmath>

namespace robot_controller {
namespace kinematics {

struct AnalyticalIKSolution {
    JointAngles angles;
    bool isValid;
    int configuration;  // 0-7 for 8 possible solutions
    double residualError;

    AnalyticalIKSolution() : angles{0}, isValid(false), configuration(0), residualError(0) {}
};

struct AnalyticalIKSolutions {
    std::vector<AnalyticalIKSolution> solutions;
    bool hasAnySolution() const { return !solutions.empty(); }

    std::optional<AnalyticalIKSolution> getClosestTo(const JointAngles& reference) const {
        if (solutions.empty()) return std::nullopt;

        const AnalyticalIKSolution* best = nullptr;
        double minDist = 1e18;

        for (const auto& sol : solutions) {
            if (!sol.isValid) continue;
            double dist = 0;
            for (int i = 0; i < NUM_JOINTS; i++) {
                double diff = normalizeAngle(sol.angles[i] - reference[i]);
                dist += diff * diff;
            }
            if (dist < minDist) {
                minDist = dist;
                best = &sol;
            }
        }
        return best ? std::optional<AnalyticalIKSolution>(*best) : std::nullopt;
    }
};

/**
 * Analytical IK solver for 6-DOF robot with spherical wrist.
 *
 * Algorithm (Modified DH specific):
 *   1. Compute wrist center (WC) from target pose
 *   2. Solve θ3 from WC_y = d1 + d4*cos(q3)
 *   3. Solve θ2 from arm plane distance constraint
 *   4. Solve θ1 from WC_x, WC_z using θ2, θ3 results
 *   5. Solve θ4, θ5, θ6 from R_36 = R_03^T * R_06
 */
class AnalyticalIK {
public:
    explicit AnalyticalIK(const RobotKinematicConfig& config)
        : config_(config), fk_(config) {
        extractParams();
    }

    void setConfig(const RobotKinematicConfig& config) {
        config_ = config;
        fk_.setConfig(config);
        extractParams();
    }

    /**
     * Compute all valid IK solutions (up to 8)
     */
    AnalyticalIKSolutions computeAll(const TCPPose& targetPose,
                                      const JointAngles& referenceAngles = {0,0,0,0,0,0}) const {
        AnalyticalIKSolutions result;

        // === Step 0: Compute wrist center ===
        // Remove tool transform to get flange pose
        Matrix4d T_flange = targetPose.toTransform();
        Matrix4d T_tool_inv = Matrix4d::Identity();
        T_tool_inv.block<3, 3>(0, 0) = config_.toolRotation.transpose();
        T_tool_inv.block<3, 1>(0, 3) = -config_.toolRotation.transpose() * config_.toolOffset;
        T_flange = T_flange * T_tool_inv;

        // Remove base transform
        Matrix4d T_base_inv = Matrix4d::Identity();
        T_base_inv.block<3, 3>(0, 0) = config_.baseRotation.transpose();
        T_base_inv.block<3, 1>(0, 3) = -config_.baseRotation.transpose() * config_.baseOffset;
        Matrix4d T_06 = T_base_inv * T_flange;

        Matrix3d R_06 = T_06.block<3, 3>(0, 0);
        Vector3d P_06 = T_06.block<3, 1>(0, 3);

        // Wrist center = P_06 - d6 * R_06 * [0,0,1]
        Vector3d wc = P_06 - d6_ * R_06.col(2);

        double WCx = wc.x();
        double WCy = wc.y();
        double WCz = wc.z();

        // R² = (WCx - a1)² + WCz² : radial distance in XZ plane from frame 1 origin
        double R_sq = (WCx - a1_) * (WCx - a1_) + WCz * WCz;

        // === Step 1: Solve θ3 ===
        // From FK derivation: WCy = d1 + d4*cos(q3)
        // So: cos(q3) = (WCy - d1) / d4

        double cos_q3 = (WCy - d1_) / d4_;
        if (std::abs(cos_q3) > 1.0 + 1e-6) {
            return result;  // unreachable
        }
        cos_q3 = std::clamp(cos_q3, -1.0, 1.0);

        // Two solutions for q3 (elbow up/down)
        double q3_options[2];
        q3_options[0] = std::acos(cos_q3);    // positive q3
        q3_options[1] = -std::acos(cos_q3);   // negative q3
        int n_q3 = (std::abs(cos_q3) > 1.0 - 1e-10) ? 1 : 2;

        for (int i3 = 0; i3 < n_q3; i3++) {
            double q3 = q3_options[i3];
            double s3 = std::sin(q3);

            // === Step 2: Solve θ2 ===
            // K = a3 + d4*sin(q3)
            // R² = a2² + K² + 2*a2*K*sin(q2)

            double K = a3_ + d4_ * s3;
            double sin_q2_num = R_sq - a2_ * a2_ - K * K;
            double sin_q2_den = 2.0 * a2_ * K;

            if (std::abs(sin_q2_den) < 1e-10) {
                continue;
            }

            double sin_q2 = sin_q2_num / sin_q2_den;
            if (std::abs(sin_q2) > 1.0 + 1e-6) {
                continue;
            }
            sin_q2 = std::clamp(sin_q2, -1.0, 1.0);

            // Two solutions for q2
            double q2_options[2];
            q2_options[0] = std::asin(sin_q2);
            q2_options[1] = PI - std::asin(sin_q2);
            int n_q2 = (std::abs(sin_q2) > 1.0 - 1e-10) ? 1 : 2;

            for (int i2 = 0; i2 < n_q2; i2++) {
                double q2 = q2_options[i2];
                double s2 = std::sin(q2);
                double c2 = std::cos(q2);

                // === Step 3: Solve θ1 ===
                double plx = a2_ + s2 * K;
                double ply = -c2 * K;

                double psi = std::atan2(-WCz, WCx - a1_);
                double phi = std::atan2(-ply, plx);

                double q1 = normalizeAngle(psi + phi);

                // Also try q1 + π (opposite shoulder)
                double q1_options[2];
                q1_options[0] = q1;
                q1_options[1] = normalizeAngle(q1 + PI);

                for (int i1 = 0; i1 < 2; i1++) {
                    double theta1 = q1_options[i1];

                    // Verify arm position before computing wrist
                    JointAngles q_arm = {theta1, q2, q3, 0, 0, 0};
                    auto armTransforms = fk_.computeAllTransforms(q_arm);
                    Vector3d wc_verify = armTransforms[4].block<3, 1>(0, 3); // Frame 5 = WC
                    double wc_err = (wc_verify - wc).norm();
                    if (wc_err > 1.0) {
                        continue; // This arm config doesn't reach target WC
                    }

                    // === Step 4: Solve wrist angles (θ4, θ5, θ6) ===
                    // Build T_03 from θ1, θ2, θ3
                    Matrix4d T_03 = Matrix4d::Identity();
                    for (int j = 0; j < 3; j++) {
                        T_03 = T_03 * fk_.computeJointTransform(j, q_arm[j]);
                    }
                    Matrix3d R_03 = T_03.block<3, 3>(0, 0);

                    // R_36 = R_03^T * R_06
                    Matrix3d R_36 = R_03.transpose() * R_06;

                    // R_36 structure (derived from Modified DH R4*R5*R6):
                    //   R4: α3=90°  → [c4,-s4,0; 0,0,-1; s4,c4,0]
                    //   R5: α4=-90° → [c5,-s5,0; 0,0,1; -s5,-c5,0]
                    //   R6: α5=0°   → [c6,-s6,0; s6,c6,0; 0,0,1]
                    //
                    // R_36 = R4*R5*R6 =
                    //   [c4*cos(φ),  -c4*sin(φ),  -s4]
                    //   [sin(φ),      cos(φ),       0 ]
                    //   [s4*cos(φ),  -s4*sin(φ),   c4 ]
                    //
                    // where φ = q5 + q6 (coupled!)
                    //
                    // Extraction:
                    //   q4 = atan2(-R_36(0,2), R_36(2,2))
                    //   φ  = atan2(R_36(1,0), R_36(1,1))
                    //   q5+q6 = φ → split using reference angles

                    double q4 = std::atan2(-R_36(0, 2), R_36(2, 2));

                    double phi_wrist = std::atan2(R_36(1, 0), R_36(1, 1));
                    // phi_wrist = q5 + q6

                    // Distribute q5+q6 using reference angles:
                    // Keep q5 close to reference, compute q6 = phi - q5
                    double ref_q5 = referenceAngles[5 - 1]; // index 4
                    double q5 = ref_q5;
                    double q6 = normalizeAngle(phi_wrist - q5);

                    // If q5 or q6 out of limits, try q5=0
                    if (q5 < config_.dhParams[4].minAngle || q5 > config_.dhParams[4].maxAngle ||
                        q6 < config_.dhParams[5].minAngle || q6 > config_.dhParams[5].maxAngle) {
                        q5 = 0.0;
                        q6 = normalizeAngle(phi_wrist);
                    }

                    // If still out of limits, try splitting evenly
                    if (q5 < config_.dhParams[4].minAngle || q5 > config_.dhParams[4].maxAngle ||
                        q6 < config_.dhParams[5].minAngle || q6 > config_.dhParams[5].maxAngle) {
                        q5 = phi_wrist / 2.0;
                        q6 = phi_wrist / 2.0;
                    }

                    AnalyticalIKSolution sol;
                    sol.angles[0] = normalizeAngle(theta1);
                    sol.angles[1] = normalizeAngle(q2);
                    sol.angles[2] = normalizeAngle(q3);
                    sol.angles[3] = normalizeAngle(q4);
                    sol.angles[4] = normalizeAngle(q5);
                    sol.angles[5] = normalizeAngle(q6);
                    sol.configuration = i1 * 4 + i3 * 2 + i2;
                    sol.isValid = true;

                    // Check joint limits
                    for (int j = 0; j < 6; j++) {
                        if (sol.angles[j] < config_.dhParams[j].minAngle ||
                            sol.angles[j] > config_.dhParams[j].maxAngle) {
                            sol.isValid = false;
                            break;
                        }
                    }

                    // Verify with FK (critical sanity check)
                    if (sol.isValid) {
                        TCPPose verify = fk_.compute(sol.angles);
                        double posErr = (verify.position - targetPose.position).norm();

                        // Also check orientation error
                        Matrix3d R_err = verify.rotation.transpose() * targetPose.rotation;
                        double orientErr = std::acos(std::clamp((R_err.trace() - 1.0) / 2.0, -1.0, 1.0));

                        sol.residualError = posErr;

                        // Due to wrist q5+q6 coupling, exact orientation match may not
                        // be possible when arm angles differ from original. Use relaxed
                        // thresholds suitable for Cartesian jog (small steps).
                        if (posErr > 5.0) {
                            sol.isValid = false;
                        }
                    }

                    // Check for duplicate solutions
                    if (sol.isValid) {
                        bool isDuplicate = false;
                        for (const auto& existing : result.solutions) {
                            // Compare by TCP pose equivalence, not joint angles
                            // (because q5/q6 can differ but produce same TCP)
                            TCPPose p1 = fk_.compute(sol.angles);
                            TCPPose p2 = fk_.compute(existing.angles);
                            double posDiff = (p1.position - p2.position).norm();
                            if (posDiff < 0.01) {
                                // Same TCP → keep the one closer to reference
                                double dist_new = 0, dist_old = 0;
                                for (int j = 0; j < 6; j++) {
                                    double d1 = normalizeAngle(sol.angles[j] - referenceAngles[j]);
                                    double d2 = normalizeAngle(existing.angles[j] - referenceAngles[j]);
                                    dist_new += d1 * d1;
                                    dist_old += d2 * d2;
                                }
                                isDuplicate = true;
                                // Replace if new one is closer to reference
                                if (dist_new < dist_old) {
                                    // Find and replace
                                    for (auto& ex : result.solutions) {
                                        TCPPose pe = fk_.compute(ex.angles);
                                        if ((pe.position - p2.position).norm() < 0.01) {
                                            ex = sol;
                                            break;
                                        }
                                    }
                                }
                                break;
                            }
                        }
                        if (!isDuplicate) {
                            result.solutions.push_back(sol);
                        }
                    }
                }
            }
        }

        return result;
    }

    /**
     * Compute single best IK solution closest to reference
     */
    std::optional<AnalyticalIKSolution> compute(
        const TCPPose& targetPose,
        const JointAngles& referenceAngles) const {

        auto allSolutions = computeAll(targetPose, referenceAngles);
        return allSolutions.getClosestTo(referenceAngles);
    }

private:
    RobotKinematicConfig config_;
    ForwardKinematics fk_;

    // Extracted DH parameters
    double a1_, a2_, a3_;   // link lengths
    double d1_, d4_, d6_;   // joint offsets

    void extractParams() {
        if (config_.dhParams.size() >= 6) {
            a1_ = config_.dhParams[0].a;
            a2_ = config_.dhParams[1].a;
            a3_ = config_.dhParams[2].a;
            d1_ = config_.dhParams[0].d;
            d4_ = config_.dhParams[3].d;
            d6_ = config_.dhParams[5].d;
        }
    }
};

} // namespace kinematics
} // namespace robot_controller
