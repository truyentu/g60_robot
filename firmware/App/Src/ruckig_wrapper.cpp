/**
 * @file ruckig_wrapper.cpp
 * @brief C++ implementation of ruckig_wrapper.h
 *
 * Thin wrapper around Ruckig<6> for embedded use.
 * Compiled as C++17, linked with C code via extern "C".
 *
 * Memory: all Ruckig objects are statically allocated (no heap).
 */

#include "ruckig_wrapper.h"
#include <ruckig/ruckig.hpp>
#include <cstring>

static constexpr size_t DOF = RUCKIG_NUM_AXES;

/* Static allocation — no heap needed */
static ruckig::InputParameter<DOF>  s_input;
static ruckig::OutputParameter<DOF> s_output;
static ruckig::Ruckig<DOF>*         s_otg = nullptr;
static ruckig::Result               s_last_result = ruckig::Result::Finished;
static bool                         s_active = false;

/* Storage for Ruckig instance (placement new, no heap) */
static alignas(ruckig::Ruckig<DOF>) uint8_t s_otg_storage[sizeof(ruckig::Ruckig<DOF>)];

extern "C" {

void ruckig_init(float cycle_time_s)
{
    /* Placement new — construct Ruckig in static storage */
    s_otg = new (s_otg_storage) ruckig::Ruckig<DOF>(
        static_cast<double>(cycle_time_s)
    );
    s_active = false;
    s_last_result = ruckig::Result::Finished;
}

RuckigResult ruckig_plan(
    const float cur_pos[RUCKIG_NUM_AXES],
    const float cur_vel[RUCKIG_NUM_AXES],
    const float cur_acc[RUCKIG_NUM_AXES],
    const float target_pos[RUCKIG_NUM_AXES],
    const float max_vel[RUCKIG_NUM_AXES],
    const float max_acc[RUCKIG_NUM_AXES],
    const float max_jerk[RUCKIG_NUM_AXES])
{
    if (!s_otg) return RUCKIG_ERROR;

    for (size_t i = 0; i < DOF; i++) {
        s_input.current_position[i]     = static_cast<double>(cur_pos[i]);
        s_input.current_velocity[i]     = cur_vel ? static_cast<double>(cur_vel[i]) : 0.0;
        s_input.current_acceleration[i] = cur_acc ? static_cast<double>(cur_acc[i]) : 0.0;

        s_input.target_position[i]      = static_cast<double>(target_pos[i]);
        s_input.target_velocity[i]      = 0.0;  /* Stop at target */
        s_input.target_acceleration[i]  = 0.0;

        s_input.max_velocity[i]         = static_cast<double>(max_vel[i]);
        s_input.max_acceleration[i]     = static_cast<double>(max_acc[i]);
        s_input.max_jerk[i]             = static_cast<double>(max_jerk[i]);
    }

    /* Validate by doing first update */
    s_last_result = s_otg->update(s_input, s_output);

    if (s_last_result == ruckig::Result::Working ||
        s_last_result == ruckig::Result::Finished) {
        s_active = (s_last_result == ruckig::Result::Working);
        /* Feed output back as input for next cycle */
        s_output.pass_to_input(s_input);
        return s_active ? RUCKIG_WORKING : RUCKIG_FINISHED;
    }

    s_active = false;
    return RUCKIG_ERROR;
}

RuckigResult ruckig_update(
    float pos_out[RUCKIG_NUM_AXES],
    float vel_out[RUCKIG_NUM_AXES],
    float acc_out[RUCKIG_NUM_AXES])
{
    if (!s_otg || !s_active) {
        return RUCKIG_FINISHED;
    }

    s_last_result = s_otg->update(s_input, s_output);

    /* Copy outputs */
    for (size_t i = 0; i < DOF; i++) {
        if (pos_out) pos_out[i] = static_cast<float>(s_output.new_position[i]);
        if (vel_out) vel_out[i] = static_cast<float>(s_output.new_velocity[i]);
        if (acc_out) acc_out[i] = static_cast<float>(s_output.new_acceleration[i]);
    }

    if (s_last_result == ruckig::Result::Working) {
        s_output.pass_to_input(s_input);
        return RUCKIG_WORKING;
    }

    if (s_last_result == ruckig::Result::Finished) {
        s_active = false;
        /* Ensure final position is exact target */
        for (size_t i = 0; i < DOF; i++) {
            if (pos_out) pos_out[i] = static_cast<float>(s_output.new_position[i]);
            if (vel_out) vel_out[i] = 0.0f;
            if (acc_out) acc_out[i] = 0.0f;
        }
        return RUCKIG_FINISHED;
    }

    /* Error */
    s_active = false;
    return RUCKIG_ERROR;
}

void ruckig_abort(void)
{
    s_active = false;
    s_last_result = ruckig::Result::Finished;
}

int ruckig_is_done(void)
{
    return s_active ? 0 : 1;
}

float ruckig_get_duration(void)
{
    if (!s_active) return 0.0f;
    return static_cast<float>(s_output.trajectory.get_duration());
}

} /* extern "C" */
