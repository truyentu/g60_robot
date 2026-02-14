cmake_minimum_required(VERSION 3.22)

# ===========================================================================
# Project
# ===========================================================================
project(robot_firmware C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ===========================================================================
# MCU & Compiler Flags
# ===========================================================================
set(MCU_FLAGS
    -mcpu=cortex-m7
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -mthumb
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter -fdata-sections -ffunction-sections")
set(CMAKE_C_FLAGS_DEBUG "-Og -g3 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")

# Linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H743ZITx_FLASH.ld)

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} \
    -T${LINKER_SCRIPT} \
    ${MCU_FLAGS} \
    --specs=nano.specs \
    --specs=nosys.specs \
    -Wl,--gc-sections \
    -Wl,--print-memory-usage \
    -Wl,-Map=${PROJECT_NAME}.map"
)

# ===========================================================================
# Definitions
# ===========================================================================
add_compile_definitions(
    STM32H743xx
    USE_HAL_DRIVER
)

add_compile_options(${MCU_FLAGS})

# ===========================================================================
# Include Paths
# ===========================================================================

# Application includes
set(APP_INCLUDES
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/BSP/Inc
    ${CMAKE_SOURCE_DIR}/App/Inc
    ${CMAKE_SOURCE_DIR}/Protocol/Inc
)

# Driver includes (populated by user — see Drivers/README.md)
set(DRIVER_INCLUDES
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/FreeRTOS/include
    ${CMAKE_SOURCE_DIR}/Drivers/FreeRTOS/portable/GCC/ARM_CM7/r0p1
    ${CMAKE_SOURCE_DIR}/Drivers/LwIP/src/include
    ${CMAKE_SOURCE_DIR}/Drivers/LwIP/system
)

include_directories(${APP_INCLUDES} ${DRIVER_INCLUDES})

# ===========================================================================
# Source Files
# ===========================================================================

# Application sources
file(GLOB_RECURSE CORE_SOURCES    "Core/Src/*.c")
file(GLOB_RECURSE BSP_SOURCES     "BSP/Src/*.c")
file(GLOB_RECURSE APP_SOURCES     "App/Src/*.c")
file(GLOB_RECURSE PROTO_SOURCES   "Protocol/Src/*.c")

# Startup assembly (user must place in Drivers/CMSIS/)
set(STARTUP_SOURCE
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h743xx.s
)

# HAL sources (user populates Drivers/)
file(GLOB HAL_SOURCES "Drivers/STM32H7xx_HAL_Driver/Src/*.c")

# FreeRTOS sources
set(FREERTOS_SOURCES
    Drivers/FreeRTOS/tasks.c
    Drivers/FreeRTOS/queue.c
    Drivers/FreeRTOS/list.c
    Drivers/FreeRTOS/timers.c
    Drivers/FreeRTOS/event_groups.c
    Drivers/FreeRTOS/stream_buffer.c
    Drivers/FreeRTOS/portable/GCC/ARM_CM7/r0p1/port.c
    Drivers/FreeRTOS/portable/MemMang/heap_4.c
)

# LwIP sources (core + API + netif)
file(GLOB_RECURSE LWIP_CORE_SOURCES   "Drivers/LwIP/src/core/*.c")
file(GLOB_RECURSE LWIP_API_SOURCES    "Drivers/LwIP/src/api/*.c")
file(GLOB_RECURSE LWIP_NETIF_SOURCES  "Drivers/LwIP/src/netif/*.c")
set(LWIP_SOURCES ${LWIP_CORE_SOURCES} ${LWIP_API_SOURCES} ${LWIP_NETIF_SOURCES})

# ===========================================================================
# Executable
# ===========================================================================
add_executable(${PROJECT_NAME}
    ${CORE_SOURCES}
    ${BSP_SOURCES}
    ${APP_SOURCES}
    ${PROTO_SOURCES}
    ${STARTUP_SOURCE}
    ${HAL_SOURCES}
    ${FREERTOS_SOURCES}
    ${LWIP_SOURCES}
)

# ===========================================================================
# Post-build: generate .bin and .hex
# ===========================================================================
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE}    $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating .bin and .hex, printing size..."
)
