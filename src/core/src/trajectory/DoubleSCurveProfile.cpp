/**
 * @file DoubleSCurveProfile.cpp
 * @brief Implementation of 7-Segment Double S-Curve velocity profile
 *
 * Algorithm: Biagiotti-Melchiorri, Chapter 3, Algorithm 3.1
 *
 * Phase structure:
 *   Phase 1: jerk = +J_max   (acceleration ramps up)
 *   Phase 2: jerk = 0        (constant acceleration = A_max)
 *   Phase 3: jerk = -J_max   (acceleration ramps down to 0)
 *   Phase 4: jerk = 0        (constant velocity = V_lim, zero accel)
 *   Phase 5: jerk = -J_max   (deceleration ramps up)
 *   Phase 6: jerk = 0        (constant deceleration = -A_max)
 *   Phase 7: jerk = +J_max   (deceleration ramps down to 0)
 */

#include "DoubleSCurveProfile.hpp"
#include <algorithm>
#include <cmath>

namespace robot_controller {
namespace trajectory {

// ============================================================================
// Constructor
// ============================================================================

DoubleSCurveProfile::DoubleSCurveProfile()
    : qStart_(0), qEnd_(0), vStart_(0), vEnd_(0)
    , vMax_(0), aMax_(0), jMax_(0), sign_(1)
    , T_{}, tEnd_{}, phaseStart_{}, vLim_(0)
    , jerk_{}, accelConst_{}
    , totalDuration_(0), valid_(false)
{
}

// ============================================================================
// Plan
// ============================================================================

bool DoubleSCurveProfile::plan(
    double q_start, double q_end,
    double v_start, double v_end,
    double v_max, double a_max, double j_max)
{
    valid_ = false;

    // Validate inputs
    if (v_max <= 0 || a_max <= 0 || j_max <= 0) return false;
    if (v_start < 0 || v_end < 0) return false;
    if (v_start > v_max || v_end > v_max) return false;

    qStart_ = q_start;
    qEnd_ = q_end;
    vStart_ = v_start;
    vEnd_ = v_end;
    vMax_ = v_max;
    aMax_ = a_max;
    jMax_ = j_max;

    double h = q_end - q_start;

    // Handle direction
    if (h >= 0) {
        sign_ = 1;
    } else {
        sign_ = -1;
        h = -h;
    }

    // Handle zero distance
    if (h < 1e-12) {
        T_.fill(0);
        totalDuration_ = 0;
        vLim_ = 0;
        computeCumulativeTimes();
        computePhaseStartStates(qStart_, 0);
        valid_ = true;
        return true;
    }

    // Compute phase durations (working in absolute values)
    if (!computePhaseDurations(h, v_start, v_end, v_max, a_max, j_max)) {
        return false;
    }

    // Compute cumulative times and phase start states
    computeCumulativeTimes();
    computePhaseStartStates(0.0, v_start);

    valid_ = true;
    return true;
}

// ============================================================================
// Compute Phase Durations (Biagiotti-Melchiorri Algorithm 3.1)
// ============================================================================

bool DoubleSCurveProfile::computePhaseDurations(
    double h, double vs, double ve,
    double vm, double am, double jm)
{
    // ---- Step 1: Compute acceleration phase (T1, T2, T3) ----
    double Tj1, Ta;
    double aLimA;  // actual peak acceleration reached

    if ((vm - vs) * jm < am * am) {
        // A_max NOT reachable → triangular accel (T2 = 0)
        Tj1 = std::sqrt((vm - vs) / jm);
        Ta = 2.0 * Tj1;
        aLimA = jm * Tj1;
    } else {
        // A_max reachable → trapezoidal accel
        Tj1 = am / jm;
        Ta = Tj1 + (vm - vs) / am;
        aLimA = am;
    }

    // ---- Step 2: Compute deceleration phase (T5, T6, T7) ----
    double Tj2, Td;
    double aLimD;  // actual peak deceleration reached

    if ((vm - ve) * jm < am * am) {
        // A_max NOT reachable → triangular decel (T6 = 0)
        Tj2 = std::sqrt((vm - ve) / jm);
        Td = 2.0 * Tj2;
        aLimD = jm * Tj2;
    } else {
        // A_max reachable → trapezoidal decel
        Tj2 = am / jm;
        Td = Tj2 + (vm - ve) / am;
        aLimD = am;
    }

    // ---- Step 3: Compute cruise phase T4 ----
    double T4;
    if (Ta > 1e-12 && vm > 1e-12) {
        T4 = h / vm - (Ta / 2.0) * (1.0 + vs / vm) - (Td / 2.0) * (1.0 + ve / vm);
    } else {
        T4 = h / vm - Ta - Td;
    }

    // ---- Step 4: Handle degenerate cases ----
    double vLim = vm;

    if (T4 < 0) {
        // Distance too short to reach v_max
        // Set T4 = 0 and solve for reduced V_lim
        T4 = 0;

        // Try with a_max still reachable
        // Quadratic in V_lim: displacement = accel_dist + decel_dist
        // From Biagiotti eq:
        //   h = (V_lim + vs)/2 * Ta + (V_lim + ve)/2 * Td
        // where Ta = am/jm + (V_lim - vs)/am, Td = am/jm + (V_lim - ve)/am
        //
        // Expanding:
        //   h = (V_lim^2 - vs^2)/(2*am) + am*(V_lim+vs)/(2*jm)
        //     + (V_lim^2 - ve^2)/(2*am) + am*(V_lim+ve)/(2*jm)
        //
        // Collecting: (1/am)*V_lim^2 + (am/jm)*V_lim + C = 0

        double A = 1.0 / am;
        double B = am / jm;
        double C = -(h + vs * vs / (2.0 * am) + ve * ve / (2.0 * am)
                      + am * vs / (2.0 * jm) + am * ve / (2.0 * jm));

        double disc = B * B - 4.0 * A * C;
        if (disc < 0) {
            // Even more constrained: a_max not reachable either
            // Try triangular accel AND decel (T2=T6=0)
            // h = Tj1^2 * jm + vs * 2*Tj1 + Tj2^2 * jm + ve * 2*Tj2
            // where V_lim = vs + jm*Tj1^2 = ve + jm*Tj2^2
            // This is complex; use iterative reduction

            // Simple approach: binary search for V_lim
            double vLo = std::max(vs, ve);
            double vHi = vm;

            for (int iter = 0; iter < 100; ++iter) {
                vLim = 0.5 * (vLo + vHi);

                // Recompute accel phase
                double tj1_t, ta_t;
                if ((vLim - vs) * jm < am * am) {
                    tj1_t = std::sqrt(std::max(0.0, (vLim - vs) / jm));
                    ta_t = 2.0 * tj1_t;
                } else {
                    tj1_t = am / jm;
                    ta_t = tj1_t + (vLim - vs) / am;
                }

                // Recompute decel phase
                double tj2_t, td_t;
                if ((vLim - ve) * jm < am * am) {
                    tj2_t = std::sqrt(std::max(0.0, (vLim - ve) / jm));
                    td_t = 2.0 * tj2_t;
                } else {
                    tj2_t = am / jm;
                    td_t = tj2_t + (vLim - ve) / am;
                }

                // Compute distance at this V_lim
                double dist;
                if (vLim > 1e-12) {
                    dist = (ta_t / 2.0) * (vLim + vs) + (td_t / 2.0) * (vLim + ve);
                } else {
                    dist = 0;
                }

                if (std::abs(dist - h) < 1e-10) break;

                if (dist > h) {
                    vHi = vLim;
                } else {
                    vLo = vLim;
                }
            }

            // Recompute final durations with found V_lim
            if ((vLim - vs) * jm < am * am) {
                Tj1 = std::sqrt(std::max(0.0, (vLim - vs) / jm));
                Ta = 2.0 * Tj1;
                aLimA = jm * Tj1;
            } else {
                Tj1 = am / jm;
                Ta = Tj1 + (vLim - vs) / am;
                aLimA = am;
            }

            if ((vLim - ve) * jm < am * am) {
                Tj2 = std::sqrt(std::max(0.0, (vLim - ve) / jm));
                Td = 2.0 * Tj2;
                aLimD = jm * Tj2;
            } else {
                Tj2 = am / jm;
                Td = Tj2 + (vLim - ve) / am;
                aLimD = am;
            }
        } else {
            vLim = (-B + std::sqrt(disc)) / (2.0 * A);
            vLim = std::max(vLim, std::max(vs, ve));  // V_lim >= max(vs, ve)

            // Recompute accel/decel with reduced V_lim
            if ((vLim - vs) * jm < am * am) {
                Tj1 = std::sqrt(std::max(0.0, (vLim - vs) / jm));
                Ta = 2.0 * Tj1;
                aLimA = jm * Tj1;
            } else {
                Tj1 = am / jm;
                Ta = Tj1 + (vLim - vs) / am;
                aLimA = am;
            }

            if ((vLim - ve) * jm < am * am) {
                Tj2 = std::sqrt(std::max(0.0, (vLim - ve) / jm));
                Td = 2.0 * Tj2;
                aLimD = jm * Tj2;
            } else {
                Tj2 = am / jm;
                Td = Tj2 + (vLim - ve) / am;
                aLimD = am;
            }
        }
    }

    vLim_ = vLim;

    // ---- Set phase durations ----
    T_[0] = Tj1;                         // T1: jerk+
    T_[1] = Ta - 2.0 * Tj1;             // T2: const accel
    T_[2] = Tj1;                         // T3: jerk-
    T_[3] = T4;                          // T4: cruise
    T_[4] = Tj2;                         // T5: jerk-
    T_[5] = Td - 2.0 * Tj2;             // T6: const decel
    T_[6] = Tj2;                         // T7: jerk+

    // Clamp negative durations from numerical errors
    for (int i = 0; i < 7; ++i) {
        if (T_[i] < 0) T_[i] = 0;
    }

    // ---- Set jerk values for each phase ----
    jerk_[0] = jm;      // Phase 1: positive jerk
    jerk_[1] = 0;        // Phase 2: zero jerk
    jerk_[2] = -jm;     // Phase 3: negative jerk
    jerk_[3] = 0;        // Phase 4: zero jerk
    jerk_[4] = -jm;     // Phase 5: negative jerk
    jerk_[5] = 0;        // Phase 6: zero jerk
    jerk_[6] = jm;      // Phase 7: positive jerk

    totalDuration_ = 0;
    for (int i = 0; i < 7; ++i) {
        totalDuration_ += T_[i];
    }

    return true;
}

// ============================================================================
// Compute Cumulative Times
// ============================================================================

void DoubleSCurveProfile::computeCumulativeTimes() {
    tEnd_[0] = 0;
    for (int i = 0; i < 7; ++i) {
        tEnd_[i + 1] = tEnd_[i] + T_[i];
    }
}

// ============================================================================
// Compute Phase Start States
// ============================================================================

void DoubleSCurveProfile::computePhaseStartStates(double q0, double vs) {
    // Phase 0 start: initial conditions
    phaseStart_[0] = {q0, vs, 0.0, jerk_[0]};

    // Propagate through each phase
    for (int i = 0; i < 7; ++i) {
        double dt = T_[i];
        double j = jerk_[i];
        double a0 = phaseStart_[i].acceleration;
        double v0 = phaseStart_[i].velocity;
        double p0 = phaseStart_[i].position;

        // End state of phase i = start state of phase i+1
        double a1 = a0 + j * dt;
        double v1 = v0 + a0 * dt + 0.5 * j * dt * dt;
        double p1 = p0 + v0 * dt + 0.5 * a0 * dt * dt + (1.0 / 6.0) * j * dt * dt * dt;

        phaseStart_[i + 1] = {p1, v1, a1, (i < 6) ? jerk_[i + 1] : 0.0};
    }
}

// ============================================================================
// Evaluate Phase
// ============================================================================

SCurveState DoubleSCurveProfile::evaluatePhase(int phase, double tau) const {
    double j = jerk_[phase];
    double a0 = phaseStart_[phase].acceleration;
    double v0 = phaseStart_[phase].velocity;
    double p0 = phaseStart_[phase].position;

    SCurveState s;
    s.jerk = j;
    s.acceleration = a0 + j * tau;
    s.velocity = v0 + a0 * tau + 0.5 * j * tau * tau;
    s.position = p0 + v0 * tau + 0.5 * a0 * tau * tau + (1.0 / 6.0) * j * tau * tau * tau;

    return s;
}

// ============================================================================
// Evaluate
// ============================================================================

SCurveState DoubleSCurveProfile::evaluate(double t) const {
    if (!valid_) {
        return {qStart_, 0, 0, 0};
    }

    // Before start
    if (t <= 0) {
        return {qStart_, vStart_ * sign_, 0, 0};
    }

    // After end
    if (t >= totalDuration_) {
        return {qEnd_, vEnd_ * sign_, 0, 0};
    }

    // Find which phase we're in
    int phase = 6;  // default to last phase
    for (int i = 0; i < 7; ++i) {
        if (t < tEnd_[i + 1]) {
            phase = i;
            break;
        }
    }

    double tau = t - tEnd_[phase];

    SCurveState raw = evaluatePhase(phase, tau);

    // Apply direction sign and offset
    SCurveState result;
    result.position = qStart_ + sign_ * raw.position;
    result.velocity = sign_ * raw.velocity;
    result.acceleration = sign_ * raw.acceleration;
    result.jerk = sign_ * raw.jerk;

    return result;
}

} // namespace trajectory
} // namespace robot_controller
