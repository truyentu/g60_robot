/**
 * @file board_config.h
 * @brief Board Support Package — Hardware pin definitions for custom STM32H743 board
 *
 * ALL hardware-specific definitions are centralized here.
 * Adjust pin mappings to match your schematic.
 *
 * Convention: Pins marked "TODO: Set from schematic" need to be verified.
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32h7xx_hal.h"

/* ========================================================================= */
/*  SIMULATION MODE                                                          */
/* ========================================================================= */
/* Comment this out to enable Real Hardware Timers for step pulse generation. */
/* When defined: MotionTask uses printf simulation, timers are NOT initialized. */
/* When undefined: MotionTask drives real TIM hardware for step/dir output.    */
#define SIMULATION_MODE 1

/* ========================================================================= */
/*  SYSTEM CLOCK                                                             */
/* ========================================================================= */

#define BOARD_HSE_VALUE             25000000U   /* TODO: External crystal frequency (Hz) */

/* ========================================================================= */
/*  STATUS LED                                                               */
/* ========================================================================= */

#define LED_STATUS_PORT             GPIOB       /* TODO: Set from schematic */
#define LED_STATUS_PIN              GPIO_PIN_0
#define LED_STATUS_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()

/* ========================================================================= */
/*  DEBUG UART                                                               */
/* ========================================================================= */

#define DEBUG_UART_INSTANCE         USART3      /* TODO: Set from schematic */
#define DEBUG_UART_BAUDRATE         115200U
#define DEBUG_UART_TX_PORT          GPIOD       /* TODO: Set from schematic */
#define DEBUG_UART_TX_PIN           GPIO_PIN_8
#define DEBUG_UART_RX_PORT          GPIOD       /* TODO: Set from schematic */
#define DEBUG_UART_RX_PIN           GPIO_PIN_9
#define DEBUG_UART_AF               GPIO_AF7_USART3
#define DEBUG_UART_TX_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define DEBUG_UART_CLK_ENABLE()     __HAL_RCC_USART3_CLK_ENABLE()

/* ========================================================================= */
/*  ETHERNET RMII                                                            */
/* ========================================================================= */
/* Standard RMII pinout — adjust for your board layout.                      */
/* PHY: LAN8742 or compatible (DP83848, KSZ8081, etc.)                       */

#define ETH_RMII_REF_CLK_PORT      GPIOA       /* TODO: Set from schematic */
#define ETH_RMII_REF_CLK_PIN       GPIO_PIN_1
#define ETH_RMII_MDIO_PORT         GPIOA       /* TODO: Set from schematic */
#define ETH_RMII_MDIO_PIN          GPIO_PIN_2
#define ETH_RMII_MDC_PORT          GPIOC       /* TODO: Set from schematic */
#define ETH_RMII_MDC_PIN           GPIO_PIN_1
#define ETH_RMII_CRS_DV_PORT       GPIOA       /* TODO: Set from schematic */
#define ETH_RMII_CRS_DV_PIN        GPIO_PIN_7
#define ETH_RMII_RXD0_PORT         GPIOC       /* TODO: Set from schematic */
#define ETH_RMII_RXD0_PIN          GPIO_PIN_4
#define ETH_RMII_RXD1_PORT         GPIOC       /* TODO: Set from schematic */
#define ETH_RMII_RXD1_PIN          GPIO_PIN_5
#define ETH_RMII_TX_EN_PORT        GPIOG       /* TODO: Set from schematic */
#define ETH_RMII_TX_EN_PIN         GPIO_PIN_11
#define ETH_RMII_TXD0_PORT         GPIOG       /* TODO: Set from schematic */
#define ETH_RMII_TXD0_PIN          GPIO_PIN_13
#define ETH_RMII_TXD1_PORT         GPIOB       /* TODO: Set from schematic */
#define ETH_RMII_TXD1_PIN          GPIO_PIN_13

/* === PHY CONFIG === */
#define ETH_PHY_ADDRESS             0U          /* TODO: LAN8742 default address */
#define ETH_PHY_RESET_PORT          GPIOA       /* TODO: PHY reset pin (optional) */
#define ETH_PHY_RESET_PIN           GPIO_PIN_3

/* ========================================================================= */
/*  MOTOR PULSE / DIRECTION — 6 Axes (AC Servo)                              */
/* ========================================================================= */
/* Pulse output: Timer Output Compare or GPIO toggle                         */
/* Direction: GPIO output                                                    */
/* Min pulse width: >= 2.5us, direction setup: >= 5us before first pulse     */

/* Axis 1 (J1) */
#define MOTOR1_PULSE_PORT           GPIOA       /* TODO: Set from schematic */
#define MOTOR1_PULSE_PIN            GPIO_PIN_0
#define MOTOR1_DIR_PORT             GPIOA       /* TODO: Set from schematic */
#define MOTOR1_DIR_PIN              GPIO_PIN_1

/* Axis 2 (J2) */
#define MOTOR2_PULSE_PORT           GPIOA       /* TODO: Set from schematic */
#define MOTOR2_PULSE_PIN            GPIO_PIN_2
#define MOTOR2_DIR_PORT             GPIOA       /* TODO: Set from schematic */
#define MOTOR2_DIR_PIN              GPIO_PIN_3

/* Axis 3 (J3) */
#define MOTOR3_PULSE_PORT           GPIOB       /* TODO: Set from schematic */
#define MOTOR3_PULSE_PIN            GPIO_PIN_4
#define MOTOR3_DIR_PORT             GPIOB       /* TODO: Set from schematic */
#define MOTOR3_DIR_PIN              GPIO_PIN_5

/* Axis 4 (J4) */
#define MOTOR4_PULSE_PORT           GPIOB       /* TODO: Set from schematic */
#define MOTOR4_PULSE_PIN            GPIO_PIN_6
#define MOTOR4_DIR_PORT             GPIOB       /* TODO: Set from schematic */
#define MOTOR4_DIR_PIN              GPIO_PIN_7

/* Axis 5 (J5) */
#define MOTOR5_PULSE_PORT           GPIOE       /* TODO: Set from schematic */
#define MOTOR5_PULSE_PIN            GPIO_PIN_9
#define MOTOR5_DIR_PORT             GPIOE       /* TODO: Set from schematic */
#define MOTOR5_DIR_PIN              GPIO_PIN_10

/* Axis 6 (J6) */
#define MOTOR6_PULSE_PORT           GPIOE       /* TODO: Set from schematic */
#define MOTOR6_PULSE_PIN            GPIO_PIN_11
#define MOTOR6_DIR_PORT             GPIOE       /* TODO: Set from schematic */
#define MOTOR6_DIR_PIN              GPIO_PIN_12

/* ========================================================================= */
/*  SERVO DRIVE SIGNALS — 6 Axes                                             */
/* ========================================================================= */
/* ENABLE: output to servo driver (active high/low — configurable)           */
/* ALARM:  input from servo driver (fault indicator)                         */
/* READY:  input from servo driver (drive ready)                             */
/* BRAKE:  output to brake solenoid (power-to-release, 50-200ms delay)      */

/* Axis 1 */
#define MOTOR1_ENABLE_PORT          GPIOC       /* TODO: Set from schematic */
#define MOTOR1_ENABLE_PIN           GPIO_PIN_0
#define MOTOR1_ALARM_PORT           GPIOC       /* TODO: Set from schematic */
#define MOTOR1_ALARM_PIN            GPIO_PIN_1
#define MOTOR1_READY_PORT           GPIOC       /* TODO: Set from schematic */
#define MOTOR1_READY_PIN            GPIO_PIN_2
#define MOTOR1_BRAKE_PORT           GPIOC       /* TODO: Set from schematic */
#define MOTOR1_BRAKE_PIN            GPIO_PIN_3

/* Axis 2 */
#define MOTOR2_ENABLE_PORT          GPIOC       /* TODO: Set from schematic */
#define MOTOR2_ENABLE_PIN           GPIO_PIN_4
#define MOTOR2_ALARM_PORT           GPIOC       /* TODO: Set from schematic */
#define MOTOR2_ALARM_PIN            GPIO_PIN_5
#define MOTOR2_READY_PORT           GPIOD       /* TODO: Set from schematic */
#define MOTOR2_READY_PIN            GPIO_PIN_0
#define MOTOR2_BRAKE_PORT           GPIOD       /* TODO: Set from schematic */
#define MOTOR2_BRAKE_PIN            GPIO_PIN_1

/* Axis 3 */
#define MOTOR3_ENABLE_PORT          GPIOD       /* TODO: Set from schematic */
#define MOTOR3_ENABLE_PIN           GPIO_PIN_2
#define MOTOR3_ALARM_PORT           GPIOD       /* TODO: Set from schematic */
#define MOTOR3_ALARM_PIN            GPIO_PIN_3
#define MOTOR3_READY_PORT           GPIOD       /* TODO: Set from schematic */
#define MOTOR3_READY_PIN            GPIO_PIN_4
#define MOTOR3_BRAKE_PORT           GPIOD       /* TODO: Set from schematic */
#define MOTOR3_BRAKE_PIN            GPIO_PIN_5

/* Axis 4 */
#define MOTOR4_ENABLE_PORT          GPIOD       /* TODO: Set from schematic */
#define MOTOR4_ENABLE_PIN           GPIO_PIN_6
#define MOTOR4_ALARM_PORT           GPIOD       /* TODO: Set from schematic */
#define MOTOR4_ALARM_PIN            GPIO_PIN_7
#define MOTOR4_READY_PORT           GPIOF       /* TODO: Set from schematic */
#define MOTOR4_READY_PIN            GPIO_PIN_0
#define MOTOR4_BRAKE_PORT           GPIOF       /* TODO: Set from schematic */
#define MOTOR4_BRAKE_PIN            GPIO_PIN_1

/* Axis 5 */
#define MOTOR5_ENABLE_PORT          GPIOF       /* TODO: Set from schematic */
#define MOTOR5_ENABLE_PIN           GPIO_PIN_2
#define MOTOR5_ALARM_PORT           GPIOF       /* TODO: Set from schematic */
#define MOTOR5_ALARM_PIN            GPIO_PIN_3
#define MOTOR5_READY_PORT           GPIOF       /* TODO: Set from schematic */
#define MOTOR5_READY_PIN            GPIO_PIN_4
#define MOTOR5_BRAKE_PORT           GPIOF       /* TODO: Set from schematic */
#define MOTOR5_BRAKE_PIN            GPIO_PIN_5

/* Axis 6 */
#define MOTOR6_ENABLE_PORT          GPIOF       /* TODO: Set from schematic */
#define MOTOR6_ENABLE_PIN           GPIO_PIN_6
#define MOTOR6_ALARM_PORT           GPIOF       /* TODO: Set from schematic */
#define MOTOR6_ALARM_PIN            GPIO_PIN_7
#define MOTOR6_READY_PORT           GPIOF       /* TODO: Set from schematic */
#define MOTOR6_READY_PIN            GPIO_PIN_8
#define MOTOR6_BRAKE_PORT           GPIOF       /* TODO: Set from schematic */
#define MOTOR6_BRAKE_PIN            GPIO_PIN_9

/* ========================================================================= */
/*  HOME SWITCHES — 6 Axes                                                   */
/* ========================================================================= */

#define HOME1_PORT                  GPIOG       /* TODO: Set from schematic */
#define HOME1_PIN                   GPIO_PIN_0
#define HOME2_PORT                  GPIOG       /* TODO: Set from schematic */
#define HOME2_PIN                   GPIO_PIN_1
#define HOME3_PORT                  GPIOG       /* TODO: Set from schematic */
#define HOME3_PIN                   GPIO_PIN_2
#define HOME4_PORT                  GPIOG       /* TODO: Set from schematic */
#define HOME4_PIN                   GPIO_PIN_3
#define HOME5_PORT                  GPIOG       /* TODO: Set from schematic */
#define HOME5_PIN                   GPIO_PIN_4
#define HOME6_PORT                  GPIOG       /* TODO: Set from schematic */
#define HOME6_PIN                   GPIO_PIN_5

/* ========================================================================= */
/*  E-STOP                                                                   */
/* ========================================================================= */

#define ESTOP_PORT                  GPIOE       /* TODO: Set from schematic */
#define ESTOP_PIN                   GPIO_PIN_0
#define ESTOP_EXTI_IRQn             EXTI0_IRQn
#define ESTOP_ACTIVE_LOW            1           /* 1 = NC contact (active low) */

/* ========================================================================= */
/*  NETWORK CONFIGURATION                                                    */
/* ========================================================================= */

#define NET_IP_ADDR0                192
#define NET_IP_ADDR1                168
#define NET_IP_ADDR2                1
#define NET_IP_ADDR3                100

#define NET_NETMASK0                255
#define NET_NETMASK1                255
#define NET_NETMASK2                255
#define NET_NETMASK3                0

#define NET_GW_ADDR0                192
#define NET_GW_ADDR1                168
#define NET_GW_ADDR2                1
#define NET_GW_ADDR3                1

#define NET_UDP_CMD_PORT            5001        /* PC -> STM32 commands (must match Core STM32EthernetDriver) */
#define NET_UDP_STATUS_PORT         5002        /* STM32 -> PC status  (Core listens on cmd_port + 1)       */

/* ========================================================================= */
/*  FREERTOS TASK PRIORITIES                                                 */
/* ========================================================================= */
/* Higher number = higher priority. Max = configMAX_PRIORITIES - 1 = 7       */

#define TASK_PRIORITY_MOTION        7           /* Real-time: step generation */
#define TASK_PRIORITY_SAFETY        6           /* Safety monitoring */
#define TASK_PRIORITY_NET           4           /* Network rx/tx */
#define TASK_PRIORITY_IO            3           /* Digital I/O scan */
#define TASK_PRIORITY_STATUS        2           /* Status reporting */
#define TASK_PRIORITY_HOUSEKEEP     1           /* Diagnostics, watchdog */

/* ========================================================================= */
/*  FREERTOS TASK STACK SIZES (in words, 1 word = 4 bytes)                   */
/* ========================================================================= */

#define TASK_STACK_MOTION           512         /* 2KB */
#define TASK_STACK_NET              1024        /* 4KB (LwIP needs more) */
#define TASK_STACK_STATUS           512         /* 2KB */

/* ========================================================================= */
/*  SERVO PARAMETERS (defaults, overridden at runtime via protocol)          */
/* ========================================================================= */

#define ENCODER_PPR                 2500        /* Encoder pulses per revolution */
#define ENCODER_CPR                 10000       /* Counts per rev (4x quadrature) */
#define DEFAULT_MAX_PULSE_FREQ      200000      /* 200 kHz max pulse rate */
#define PULSE_MIN_WIDTH_US          3           /* Minimum pulse width (us) */
#define DIR_SETUP_TIME_US           5           /* Direction setup time (us) */

/* ========================================================================= */
/*  DIGITAL INPUTS — 16 channels (general purpose)                          */
/* ========================================================================= */

#define DIN_LO_PORT                 GPIOH       /* DIN0-7 on GPIOH */
#define DIN_LO_PIN0                 GPIO_PIN_0
#define DIN_LO_PIN1                 GPIO_PIN_1
#define DIN_LO_PIN2                 GPIO_PIN_2
#define DIN_LO_PIN3                 GPIO_PIN_3
#define DIN_LO_PIN4                 GPIO_PIN_4
#define DIN_LO_PIN5                 GPIO_PIN_5
#define DIN_LO_PIN6                 GPIO_PIN_6
#define DIN_LO_PIN7                 GPIO_PIN_7
#define DIN_LO_PINS                 (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | \
                                     GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define DIN_LO_CLK_ENABLE()         __HAL_RCC_GPIOH_CLK_ENABLE()

#define DIN_HI_PORT                 GPIOI       /* DIN8-15 on GPIOI */
#define DIN_HI_PIN0                 GPIO_PIN_0
#define DIN_HI_PIN1                 GPIO_PIN_1
#define DIN_HI_PIN2                 GPIO_PIN_2
#define DIN_HI_PIN3                 GPIO_PIN_3
#define DIN_HI_PIN4                 GPIO_PIN_4
#define DIN_HI_PIN5                 GPIO_PIN_5
#define DIN_HI_PIN6                 GPIO_PIN_6
#define DIN_HI_PIN7                 GPIO_PIN_7
#define DIN_HI_PINS                 (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | \
                                     GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define DIN_HI_CLK_ENABLE()         __HAL_RCC_GPIOI_CLK_ENABLE()

/* ========================================================================= */
/*  DIGITAL OUTPUTS — 16 channels (general purpose)                         */
/* ========================================================================= */

#define DOUT_LO_PORT                GPIOJ       /* DOUT0-7 on GPIOJ */
#define DOUT_LO_PINS                (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | \
                                     GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define DOUT_LO_CLK_ENABLE()        __HAL_RCC_GPIOJ_CLK_ENABLE()

#define DOUT_HI_PORT                GPIOK       /* DOUT8-15 on GPIOK */
#define DOUT_HI_PINS                (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | \
                                     GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define DOUT_HI_CLK_ENABLE()        __HAL_RCC_GPIOK_CLK_ENABLE()

/* ========================================================================= */
/*  SAFETY SIGNALS                                                          */
/* ========================================================================= */

#define DEADMAN_PORT                GPIOE       /* 3-position deadman switch */
#define DEADMAN_PIN                 GPIO_PIN_1
#define DEADMAN_CLK_ENABLE()        __HAL_RCC_GPIOE_CLK_ENABLE()

#define SAFETY_FENCE_PORT           GPIOE       /* Safety fence interlock (NC) */
#define SAFETY_FENCE_PIN            GPIO_PIN_2

#define MODE_SEL_PORT               GPIOE       /* Operating mode selector */
#define MODE_SEL_PIN0               GPIO_PIN_3  /* bit 0 */
#define MODE_SEL_PIN1               GPIO_PIN_4  /* bit 1 */
/* Mode encoding: 00=T1, 01=T2, 10=AUT */

/* ========================================================================= */
/*  COMMUNICATION WATCHDOG                                                  */
/* ========================================================================= */

#define COMM_TIMEOUT_MS             500         /* Heartbeat timeout (ms) */
#define COMM_TIMEOUT_COUNT          3           /* Consecutive misses before alarm */

/* ========================================================================= */
/*  HOMING DEFAULTS                                                         */
/* ========================================================================= */

#define HOME_DEFAULT_FAST_SPEED     5000        /* steps/ms */
#define HOME_DEFAULT_SLOW_SPEED     500         /* steps/ms */
#define HOME_DEFAULT_BACKOFF        1000        /* steps */
#define HOME_DEBOUNCE_MS            10          /* switch debounce (ms) */

/* ========================================================================= */
/*  BRAKE TIMING                                                            */
/* ========================================================================= */

#define BRAKE_ENGAGE_DELAY_MS       100         /* Time for brake to engage */
#define BRAKE_RELEASE_DELAY_MS      200         /* Time for brake to open */

/* ========================================================================= */
/*  WATCHDOG (IWDG)                                                         */
/* ========================================================================= */

#define IWDG_TIMEOUT_MS             1000        /* Independent watchdog timeout */

/* ========================================================================= */
/*  MOTION CONTROL CONSTANTS                                                */
/* ========================================================================= */

#define MOTION_LOOP_PERIOD_MS       4           /* Motion task period (ms) */
#define MOTION_LOOP_PERIOD_US       4000        /* Motion task period (us) */

#endif /* BOARD_CONFIG_H */
