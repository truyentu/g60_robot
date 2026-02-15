/**
 * @file input_shaper.h
 * @brief Zero Vibration (ZV) input shaper for vibration suppression
 *
 * ZV filter: 2 impulses at t=0 and t=T/2
 * Amplitudes: A1 = 1/(1+K), A2 = K/(1+K)
 * where K = exp(-π * damping / sqrt(1 - damping²))
 */

#ifndef INPUT_SHAPER_H
#define INPUT_SHAPER_H

#include <stdint.h>

/* Maximum delay buffer size (ticks) */
#define INPUT_SHAPER_BUF_SIZE  64

typedef struct {
    float   K;                                /* Shaper gain coefficient      */
    float   A1;                               /* Amplitude of impulse 1       */
    float   A2;                               /* Amplitude of impulse 2       */
    uint32_t delay_ticks;                     /* Delay in motion ticks        */
    int32_t delay_buf[INPUT_SHAPER_BUF_SIZE]; /* Circular delay buffer        */
    uint8_t buf_idx;                          /* Current write index          */
    uint8_t enabled;                          /* 0 = bypass, 1 = active       */
} InputShaper;

/**
 * Initialize a ZV input shaper.
 *
 * @param is       Shaper instance
 * @param freq_hz  Resonant frequency (Hz) to suppress
 * @param damping  Damping ratio (0.0 - 1.0, typically 0.05-0.15)
 * @param tick_ms  Motion loop period (ms)
 */
void InputShaper_Init(InputShaper *is, float freq_hz, float damping, float tick_ms);

/**
 * Apply ZV filter to input step command.
 *
 * @param is           Shaper instance
 * @param input_steps  Raw step count for this tick
 * @return             Shaped step count
 */
int32_t InputShaper_Apply(InputShaper *is, int32_t input_steps);

/**
 * Enable/disable the shaper (bypass when disabled).
 */
void InputShaper_Enable(InputShaper *is, uint8_t enable);

/**
 * Reset the shaper buffer.
 */
void InputShaper_Reset(InputShaper *is);

#endif /* INPUT_SHAPER_H */
