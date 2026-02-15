/**
 * @file board_timers.c
 * @brief Hardware timer initialization for step pulse generation
 *
 * Only compiled when SIMULATION_MODE is NOT defined.
 * Configures TIM1 (axes 1,2,5), TIM3 (axes 3,4), TIM4 (axis 6)
 * in Output Compare Toggle mode for step pulse generation.
 */

#include "board_config.h"

#ifndef SIMULATION_MODE

#include "board_timers.h"
#include "protocol_defs.h"
#include <string.h>

/* ========================================================================= */
/*  Timer Handles                                                            */
/* ========================================================================= */

static TIM_HandleTypeDef htim1;  /* Axes 1, 2, 5 */
static TIM_HandleTypeDef htim3;  /* Axes 3, 4 */
static TIM_HandleTypeDef htim4;  /* Axis 6 */

/* Per-axis timer handle + channel lookup table */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
} AxisTimerConfig;

static const AxisTimerConfig s_axis_cfg[PROTO_NUM_AXES] = {
    { &htim1, TIM_CHANNEL_1, MOTOR1_DIR_PORT, MOTOR1_DIR_PIN },
    { &htim1, TIM_CHANNEL_2, MOTOR2_DIR_PORT, MOTOR2_DIR_PIN },
    { &htim3, TIM_CHANNEL_1, MOTOR3_DIR_PORT, MOTOR3_DIR_PIN },
    { &htim3, TIM_CHANNEL_2, MOTOR4_DIR_PORT, MOTOR4_DIR_PIN },
    { &htim1, TIM_CHANNEL_3, MOTOR5_DIR_PORT, MOTOR5_DIR_PIN },
    { &htim4, TIM_CHANNEL_1, MOTOR6_DIR_PORT, MOTOR6_DIR_PIN },
};

/* ========================================================================= */
/*  Internal: Configure one timer                                            */
/* ========================================================================= */

static void init_timer(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
    memset(htim, 0, sizeof(*htim));
    htim->Instance               = instance;
    htim->Init.Prescaler         = 0;        /* No prescaler: full 240 MHz */
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.Period            = 0xFFFF;   /* Max period (stopped state) */
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_OC_Init(htim);
}

static void init_oc_channel(TIM_HandleTypeDef *htim, uint32_t channel)
{
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_TOGGLE;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_OC_ConfigChannel(htim, &oc, channel);
}

/* ========================================================================= */
/*  Internal: Configure pulse GPIO pins as timer AF outputs                  */
/* ========================================================================= */

static void init_pulse_gpio(GPIO_TypeDef *port, uint16_t pin,
                            uint8_t af, void (*clk_enable)(void))
{
    clk_enable();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = pin;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = af;
    HAL_GPIO_Init(port, &gpio);
}

static void init_dir_gpio(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &gpio);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void Board_InitTimers(void)
{
    /* Enable timer clocks */
    MOTOR1_TIM_CLK_ENABLE();
    MOTOR3_TIM_CLK_ENABLE();
    MOTOR6_TIM_CLK_ENABLE();

    /* Initialize timers */
    init_timer(&htim1, TIM1);
    init_timer(&htim3, TIM3);
    init_timer(&htim4, TIM4);

    /* Configure OC channels */
    /* TIM1: CH1 (Axis1), CH2 (Axis2), CH3 (Axis5) */
    init_oc_channel(&htim1, TIM_CHANNEL_1);
    init_oc_channel(&htim1, TIM_CHANNEL_2);
    init_oc_channel(&htim1, TIM_CHANNEL_3);

    /* TIM3: CH1 (Axis3), CH2 (Axis4) */
    init_oc_channel(&htim3, TIM_CHANNEL_1);
    init_oc_channel(&htim3, TIM_CHANNEL_2);

    /* TIM4: CH1 (Axis6) */
    init_oc_channel(&htim4, TIM_CHANNEL_1);

    /* Configure pulse GPIO pins as timer AF outputs */
    init_pulse_gpio(MOTOR1_PULSE_PORT, MOTOR1_PULSE_PIN, MOTOR1_TIM_AF,
                    MOTOR1_PULSE_CLK_ENABLE);
    init_pulse_gpio(MOTOR2_PULSE_PORT, MOTOR2_PULSE_PIN, MOTOR2_TIM_AF,
                    MOTOR2_PULSE_CLK_ENABLE);
    init_pulse_gpio(MOTOR3_PULSE_PORT, MOTOR3_PULSE_PIN, MOTOR3_TIM_AF,
                    MOTOR3_PULSE_CLK_ENABLE);
    init_pulse_gpio(MOTOR4_PULSE_PORT, MOTOR4_PULSE_PIN, MOTOR4_TIM_AF,
                    MOTOR4_PULSE_CLK_ENABLE);
    init_pulse_gpio(MOTOR5_PULSE_PORT, MOTOR5_PULSE_PIN, MOTOR5_TIM_AF,
                    MOTOR5_PULSE_CLK_ENABLE);
    init_pulse_gpio(MOTOR6_PULSE_PORT, MOTOR6_PULSE_PIN, MOTOR6_TIM_AF,
                    MOTOR6_PULSE_CLK_ENABLE);

    /* Configure direction GPIO pins */
    init_dir_gpio(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN);
    init_dir_gpio(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN);
    init_dir_gpio(MOTOR3_DIR_PORT, MOTOR3_DIR_PIN);
    init_dir_gpio(MOTOR4_DIR_PORT, MOTOR4_DIR_PIN);
    init_dir_gpio(MOTOR5_DIR_PORT, MOTOR5_DIR_PIN);
    init_dir_gpio(MOTOR6_DIR_PORT, MOTOR6_DIR_PIN);
}

void StepTimer_SetFrequency(uint8_t axis, uint32_t freq_hz)
{
    if (axis >= PROTO_NUM_AXES) return;

    const AxisTimerConfig *cfg = &s_axis_cfg[axis];

    if (freq_hz == 0) {
        /* Stop pulse output on this channel */
        HAL_TIM_OC_Stop(cfg->htim, cfg->channel);
        return;
    }

    /* Clamp to max pulse frequency */
    if (freq_hz > DEFAULT_MAX_PULSE_FREQ) {
        freq_hz = DEFAULT_MAX_PULSE_FREQ;
    }

    uint32_t arr = StepTimer_FreqToARR(freq_hz);
    __HAL_TIM_SET_AUTORELOAD(cfg->htim, arr);
    __HAL_TIM_SET_COMPARE(cfg->htim, cfg->channel, arr / 2);

    /* Start OC output if not already running */
    HAL_TIM_OC_Start(cfg->htim, cfg->channel);
}

void StepTimer_SetDirection(uint8_t axis, int8_t direction)
{
    if (axis >= PROTO_NUM_AXES) return;

    const AxisTimerConfig *cfg = &s_axis_cfg[axis];
    HAL_GPIO_WritePin(cfg->dir_port, cfg->dir_pin,
                      (direction >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void StepTimer_StopAll(void)
{
    for (int i = 0; i < PROTO_NUM_AXES; i++) {
        HAL_TIM_OC_Stop(s_axis_cfg[i].htim, s_axis_cfg[i].channel);
    }
}

#endif /* !SIMULATION_MODE */
