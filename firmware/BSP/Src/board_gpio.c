/**
 * @file board_gpio.c
 * @brief GPIO driver implementation for drive signals, digital I/O, safety
 *
 * Uses lookup tables mapping axis index to port/pin for clean array-indexed
 * access. Pattern follows AxisTimerConfig in board_timers.c.
 */

#include "board_gpio.h"
#include <string.h>

/* ========================================================================= */
/*  Internal Types                                                          */
/* ========================================================================= */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} GpioPin;

/* ========================================================================= */
/*  Pin Lookup Tables                                                       */
/* ========================================================================= */

static const GpioPin s_enable_pins[PROTO_NUM_AXES] = {
    { MOTOR1_ENABLE_PORT, MOTOR1_ENABLE_PIN },
    { MOTOR2_ENABLE_PORT, MOTOR2_ENABLE_PIN },
    { MOTOR3_ENABLE_PORT, MOTOR3_ENABLE_PIN },
    { MOTOR4_ENABLE_PORT, MOTOR4_ENABLE_PIN },
    { MOTOR5_ENABLE_PORT, MOTOR5_ENABLE_PIN },
    { MOTOR6_ENABLE_PORT, MOTOR6_ENABLE_PIN },
};

static const GpioPin s_alarm_pins[PROTO_NUM_AXES] = {
    { MOTOR1_ALARM_PORT, MOTOR1_ALARM_PIN },
    { MOTOR2_ALARM_PORT, MOTOR2_ALARM_PIN },
    { MOTOR3_ALARM_PORT, MOTOR3_ALARM_PIN },
    { MOTOR4_ALARM_PORT, MOTOR4_ALARM_PIN },
    { MOTOR5_ALARM_PORT, MOTOR5_ALARM_PIN },
    { MOTOR6_ALARM_PORT, MOTOR6_ALARM_PIN },
};

static const GpioPin s_ready_pins[PROTO_NUM_AXES] = {
    { MOTOR1_READY_PORT, MOTOR1_READY_PIN },
    { MOTOR2_READY_PORT, MOTOR2_READY_PIN },
    { MOTOR3_READY_PORT, MOTOR3_READY_PIN },
    { MOTOR4_READY_PORT, MOTOR4_READY_PIN },
    { MOTOR5_READY_PORT, MOTOR5_READY_PIN },
    { MOTOR6_READY_PORT, MOTOR6_READY_PIN },
};

static const GpioPin s_brake_pins[PROTO_NUM_AXES] = {
    { MOTOR1_BRAKE_PORT, MOTOR1_BRAKE_PIN },
    { MOTOR2_BRAKE_PORT, MOTOR2_BRAKE_PIN },
    { MOTOR3_BRAKE_PORT, MOTOR3_BRAKE_PIN },
    { MOTOR4_BRAKE_PORT, MOTOR4_BRAKE_PIN },
    { MOTOR5_BRAKE_PORT, MOTOR5_BRAKE_PIN },
    { MOTOR6_BRAKE_PORT, MOTOR6_BRAKE_PIN },
};

static const GpioPin s_home_pins[PROTO_NUM_AXES] = {
    { HOME1_PORT, HOME1_PIN },
    { HOME2_PORT, HOME2_PIN },
    { HOME3_PORT, HOME3_PIN },
    { HOME4_PORT, HOME4_PIN },
    { HOME5_PORT, HOME5_PIN },
    { HOME6_PORT, HOME6_PIN },
};

/* Track the current digital output state for readback */
static uint16_t s_dout_state = 0;

/* Track enabled state for readback */
static uint8_t s_enabled_mask = 0;

/* ========================================================================= */
/*  Helper: Init a batch of GPIO pins                                       */
/* ========================================================================= */

static void init_output_pins(const GpioPin *pins, uint8_t count, GPIO_PinState initial)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint8_t i = 0; i < count; i++) {
        gpio.Pin = pins[i].pin;
        HAL_GPIO_Init(pins[i].port, &gpio);
        HAL_GPIO_WritePin(pins[i].port, pins[i].pin, initial);
    }
}

static void init_input_pins(const GpioPin *pins, uint8_t count, uint32_t pull)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = pull;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint8_t i = 0; i < count; i++) {
        gpio.Pin = pins[i].pin;
        HAL_GPIO_Init(pins[i].port, &gpio);
    }
}

/* ========================================================================= */
/*  Initialization                                                          */
/* ========================================================================= */

void Board_InitGPIO(void)
{
    /* Enable all GPIO clocks used */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    DIN_LO_CLK_ENABLE();
    DIN_HI_CLK_ENABLE();
    DOUT_LO_CLK_ENABLE();
    DOUT_HI_CLK_ENABLE();
    DEADMAN_CLK_ENABLE();

    /* Drive enable outputs — initially disabled */
    init_output_pins(s_enable_pins, PROTO_NUM_AXES, GPIO_PIN_RESET);

    /* Brake outputs — initially engaged (GPIO_PIN_RESET = brake ON) */
    init_output_pins(s_brake_pins, PROTO_NUM_AXES, GPIO_PIN_RESET);

    /* Drive alarm inputs — pull-up (alarm is active low typically) */
    init_input_pins(s_alarm_pins, PROTO_NUM_AXES, GPIO_PULLUP);

    /* Drive ready inputs — pull-down */
    init_input_pins(s_ready_pins, PROTO_NUM_AXES, GPIO_PULLDOWN);

    /* Home switch inputs — pull-up (NC contacts) */
    init_input_pins(s_home_pins, PROTO_NUM_AXES, GPIO_PULLUP);

    /* Digital inputs (16 channels) */
    {
        GPIO_InitTypeDef gpio = {0};
        gpio.Mode  = GPIO_MODE_INPUT;
        gpio.Pull  = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;

        gpio.Pin = DIN_LO_PINS;
        HAL_GPIO_Init(DIN_LO_PORT, &gpio);

        gpio.Pin = DIN_HI_PINS;
        HAL_GPIO_Init(DIN_HI_PORT, &gpio);
    }

    /* Digital outputs (16 channels) — initially off */
    {
        GPIO_InitTypeDef gpio = {0};
        gpio.Mode  = GPIO_MODE_OUTPUT_PP;
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;

        gpio.Pin = DOUT_LO_PINS;
        HAL_GPIO_Init(DOUT_LO_PORT, &gpio);
        HAL_GPIO_WritePin(DOUT_LO_PORT, DOUT_LO_PINS, GPIO_PIN_RESET);

        gpio.Pin = DOUT_HI_PINS;
        HAL_GPIO_Init(DOUT_HI_PORT, &gpio);
        HAL_GPIO_WritePin(DOUT_HI_PORT, DOUT_HI_PINS, GPIO_PIN_RESET);
    }

    /* E-stop input — pull-up (NC contact) */
    {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = ESTOP_PIN;
        gpio.Mode  = GPIO_MODE_INPUT;
        gpio.Pull  = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(ESTOP_PORT, &gpio);
    }

    /* Safety signals */
    {
        GPIO_InitTypeDef gpio = {0};
        gpio.Mode  = GPIO_MODE_INPUT;
        gpio.Pull  = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;

        /* Deadman switch */
        gpio.Pin = DEADMAN_PIN;
        HAL_GPIO_Init(DEADMAN_PORT, &gpio);

        /* Safety fence interlock */
        gpio.Pin = SAFETY_FENCE_PIN;
        HAL_GPIO_Init(SAFETY_FENCE_PORT, &gpio);

        /* Operating mode selector */
        gpio.Pull = GPIO_PULLDOWN;
        gpio.Pin = MODE_SEL_PIN0;
        HAL_GPIO_Init(MODE_SEL_PORT, &gpio);
        gpio.Pin = MODE_SEL_PIN1;
        HAL_GPIO_Init(MODE_SEL_PORT, &gpio);
    }

    s_dout_state = 0;
    s_enabled_mask = 0;
}

/* ========================================================================= */
/*  E-Stop EXTI Init                                                        */
/* ========================================================================= */

void Board_InitEstopEXTI(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = ESTOP_PIN;
    gpio.Mode  = GPIO_MODE_IT_FALLING;  /* NC contact: falling edge = activated */
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ESTOP_PORT, &gpio);

    /* NVIC priority 0 = above FreeRTOS (configMAX_SYSCALL_INTERRUPT_PRIORITY) */
    HAL_NVIC_SetPriority(ESTOP_EXTI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ESTOP_EXTI_IRQn);
}

/* ========================================================================= */
/*  Servo Drive Signals                                                     */
/* ========================================================================= */

void GPIO_DriveEnable(uint8_t axis, uint8_t enable)
{
    if (axis >= PROTO_NUM_AXES) return;
    HAL_GPIO_WritePin(s_enable_pins[axis].port, s_enable_pins[axis].pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (enable)
        s_enabled_mask |= (1U << axis);
    else
        s_enabled_mask &= ~(1U << axis);
}

uint8_t GPIO_DriveGetReady(uint8_t axis)
{
    if (axis >= PROTO_NUM_AXES) return 0;
    return HAL_GPIO_ReadPin(s_ready_pins[axis].port, s_ready_pins[axis].pin)
           == GPIO_PIN_SET ? 1 : 0;
}

uint8_t GPIO_DriveGetAlarm(uint8_t axis)
{
    if (axis >= PROTO_NUM_AXES) return 0;
    /* Alarm is typically active LOW on servo drivers */
    return HAL_GPIO_ReadPin(s_alarm_pins[axis].port, s_alarm_pins[axis].pin)
           == GPIO_PIN_RESET ? 1 : 0;
}

void GPIO_BrakeRelease(uint8_t axis, uint8_t release)
{
    if (axis >= PROTO_NUM_AXES) return;
    /* Power-to-release: SET = released, RESET = engaged */
    HAL_GPIO_WritePin(s_brake_pins[axis].port, s_brake_pins[axis].pin,
                      release ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t GPIO_DriveGetReadyMask(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (GPIO_DriveGetReady(i))
            mask |= (1U << i);
    }
    return mask;
}

uint8_t GPIO_DriveGetAlarmMask(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < PROTO_NUM_AXES; i++) {
        if (GPIO_DriveGetAlarm(i))
            mask |= (1U << i);
    }
    return mask;
}

uint8_t GPIO_DriveGetEnabledMask(void)
{
    return s_enabled_mask;
}

/* ========================================================================= */
/*  Home Switches                                                           */
/* ========================================================================= */

uint8_t GPIO_HomeGetRaw(uint8_t axis)
{
    if (axis >= PROTO_NUM_AXES) return 0;
    /* NC contact with pull-up: LOW = switch triggered */
    return HAL_GPIO_ReadPin(s_home_pins[axis].port, s_home_pins[axis].pin)
           == GPIO_PIN_RESET ? 1 : 0;
}

/* ========================================================================= */
/*  E-Stop                                                                  */
/* ========================================================================= */

uint8_t GPIO_EstopIsActive(void)
{
#if ESTOP_ACTIVE_LOW
    return HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_RESET ? 1 : 0;
#else
    return HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_SET ? 1 : 0;
#endif
}

/* ========================================================================= */
/*  Digital I/O                                                             */
/* ========================================================================= */

uint16_t GPIO_ReadDigitalInputs(void)
{
    uint16_t result = 0;

    /* Read low 8 bits from GPIOH[0-7] */
    uint16_t port_val = (uint16_t)(DIN_LO_PORT->IDR & 0xFF);
    result |= port_val;

    /* Read high 8 bits from GPIOI[0-7] */
    port_val = (uint16_t)(DIN_HI_PORT->IDR & 0xFF);
    result |= (port_val << 8);

    return result;
}

uint16_t GPIO_ReadDigitalOutputs(void)
{
    return s_dout_state;
}

void GPIO_WriteDigitalOutput(uint8_t index, uint8_t value)
{
    if (index >= 16) return;

    if (index < 8) {
        uint16_t pin = (1U << index);
        HAL_GPIO_WritePin(DOUT_LO_PORT, pin,
                          value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        uint16_t pin = (1U << (index - 8));
        HAL_GPIO_WritePin(DOUT_HI_PORT, pin,
                          value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    if (value)
        s_dout_state |= (1U << index);
    else
        s_dout_state &= ~(1U << index);
}

void GPIO_WriteDigitalOutputs(uint16_t mask, uint16_t values)
{
    for (uint8_t i = 0; i < 16; i++) {
        if (mask & (1U << i)) {
            GPIO_WriteDigitalOutput(i, (values >> i) & 1);
        }
    }
}

/* ========================================================================= */
/*  Safety Signals                                                          */
/* ========================================================================= */

uint8_t GPIO_DeadmanIsPressed(void)
{
    /* Deadman: active when pressed (pull-up, press = LOW) */
    return HAL_GPIO_ReadPin(DEADMAN_PORT, DEADMAN_PIN) == GPIO_PIN_RESET ? 1 : 0;
}

uint8_t GPIO_SafetyFenceIsClosed(void)
{
    /* NC contact: closed = LOW with pull-up */
    return HAL_GPIO_ReadPin(SAFETY_FENCE_PORT, SAFETY_FENCE_PIN) == GPIO_PIN_RESET ? 1 : 0;
}

uint8_t GPIO_ReadOperatingMode(void)
{
    uint8_t bit0 = HAL_GPIO_ReadPin(MODE_SEL_PORT, MODE_SEL_PIN0) == GPIO_PIN_SET ? 1 : 0;
    uint8_t bit1 = HAL_GPIO_ReadPin(MODE_SEL_PORT, MODE_SEL_PIN1) == GPIO_PIN_SET ? 1 : 0;
    /* 00 = T1 (0), 01 = T2 (1), 10 = AUT (2) */
    return (bit1 << 1) | bit0;
}
