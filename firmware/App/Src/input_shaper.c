/**
 * @file input_shaper.c
 * @brief Zero Vibration (ZV) input shaper implementation
 *
 * ZV filter splits each step command into two impulses:
 *   output(t)   += A1 * input(t)
 *   output(t+D) += A2 * input(t)
 *
 * where D = half-period of resonant frequency.
 * This cancels vibration at the resonant frequency.
 */

#include "input_shaper.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void InputShaper_Init(InputShaper *is, float freq_hz, float damping, float tick_ms)
{
    memset(is, 0, sizeof(InputShaper));

    if (freq_hz <= 0 || damping < 0 || damping >= 1.0f) {
        is->enabled = 0;
        return;
    }

    /* K = exp(-π * ζ / sqrt(1 - ζ²)) */
    float zeta = damping;
    is->K = expf(-M_PI * zeta / sqrtf(1.0f - zeta * zeta));

    /* Amplitudes */
    float denom = 1.0f + is->K;
    is->A1 = 1.0f / denom;
    is->A2 = is->K / denom;

    /* Delay = half-period of resonant frequency, in ticks */
    float half_period_ms = 500.0f / freq_hz;  /* T/2 in ms */
    is->delay_ticks = (uint32_t)(half_period_ms / tick_ms + 0.5f);

    /* Clamp to buffer size */
    if (is->delay_ticks >= INPUT_SHAPER_BUF_SIZE) {
        is->delay_ticks = INPUT_SHAPER_BUF_SIZE - 1;
    }
    if (is->delay_ticks < 1) {
        is->delay_ticks = 1;
    }

    is->buf_idx = 0;
    is->enabled = 1;
}

int32_t InputShaper_Apply(InputShaper *is, int32_t input_steps)
{
    if (!is->enabled) {
        return input_steps;
    }

    /* Current output = A1 * input + A2 * delayed_input */
    uint8_t delayed_idx = (is->buf_idx + INPUT_SHAPER_BUF_SIZE - is->delay_ticks)
                          % INPUT_SHAPER_BUF_SIZE;
    int32_t delayed = is->delay_buf[delayed_idx];

    int32_t output = (int32_t)(is->A1 * (float)input_steps + is->A2 * (float)delayed);

    /* Store current input in circular buffer */
    is->delay_buf[is->buf_idx] = input_steps;
    is->buf_idx = (is->buf_idx + 1) % INPUT_SHAPER_BUF_SIZE;

    return output;
}

void InputShaper_Enable(InputShaper *is, uint8_t enable)
{
    is->enabled = enable;
    if (!enable) {
        InputShaper_Reset(is);
    }
}

void InputShaper_Reset(InputShaper *is)
{
    memset(is->delay_buf, 0, sizeof(is->delay_buf));
    is->buf_idx = 0;
}
