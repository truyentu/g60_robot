cmake_minimum_required(VERSION 3.22)

# ===========================================================================
# Project
# ===========================================================================
project(robot_firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===========================================================================
# FetchContent — Auto-download Dependencies
# ===========================================================================
include(FetchContent)

set(FETCHCONTENT_QUIET OFF)

# --- CMSIS Core (ARM) — core_cm7.h, cmsis_gcc.h, etc. ---
FetchContent_Declare(cmsis_core
    GIT_REPOSITORY https://github.com/ARM-software/CMSIS_5.git
    GIT_TAG        5.9.0
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

# --- CMSIS Device H7 (ST) — stm32h7xx.h, stm32h743xx.h, startup_stm32h743xx.s ---
FetchContent_Declare(cmsis_device_h7
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_device_h7.git
    GIT_TAG        v1.10.4
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

# --- STM32H7 HAL Driver (ST) — standalone, much lighter than full CubeH7 ---
FetchContent_Declare(stm32h7_hal
    GIT_REPOSITORY https://github.com/STMicroelectronics/stm32h7xx_hal_driver.git
    GIT_TAG        v1.11.3
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

# --- FreeRTOS Kernel ---
FetchContent_Declare(freertos_kernel
    GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
    GIT_TAG        V11.1.0
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

# --- LwIP ---
FetchContent_Declare(lwip
    GIT_REPOSITORY https://github.com/lwip-tcpip/lwip.git
    GIT_TAG        STABLE-2_2_0_RELEASE
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

# --- Ruckig (MIT) — Time-optimal jerk-limited trajectory generation ---
FetchContent_Declare(ruckig
    GIT_REPOSITORY https://github.com/pantor/ruckig.git
    GIT_TAG        v0.14.0
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

# Download all — we manage sources manually, so use Populate (not MakeAvailable)
# to avoid sub-projects trying to build themselves
FetchContent_GetProperties(cmsis_core)
if(NOT cmsis_core_POPULATED)
    FetchContent_Populate(cmsis_core)
endif()

FetchContent_GetProperties(cmsis_device_h7)
if(NOT cmsis_device_h7_POPULATED)
    FetchContent_Populate(cmsis_device_h7)
endif()

FetchContent_GetProperties(stm32h7_hal)
if(NOT stm32h7_hal_POPULATED)
    FetchContent_Populate(stm32h7_hal)
endif()

FetchContent_GetProperties(freertos_kernel)
if(NOT freertos_kernel_POPULATED)
    FetchContent_Populate(freertos_kernel)
endif()

FetchContent_GetProperties(lwip)
if(NOT lwip_POPULATED)
    FetchContent_Populate(lwip)
endif()

FetchContent_GetProperties(ruckig)
if(NOT ruckig_POPULATED)
    FetchContent_Populate(ruckig)
endif()

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

# C++ flags: no exceptions/RTTI for embedded (saves ~40KB flash)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# Linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H743ZITx_FLASH.ld)

# Convert MCU_FLAGS list to space-separated string for linker
string(REPLACE ";" " " MCU_FLAGS_STR "${MCU_FLAGS}")

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} \
    -T${LINKER_SCRIPT} \
    ${MCU_FLAGS_STR} \
    --specs=nano.specs \
    --specs=nosys.specs \
    -lstdc++ \
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
    RUCKIG_NO_ONLINE_TRAJECTORY_GENERATION=OFF
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

# FetchContent populates *_SOURCE_DIR variables
set(DRIVER_INCLUDES
    # CMSIS Core (ARM headers)
    ${cmsis_core_SOURCE_DIR}/CMSIS/Core/Include
    # CMSIS Device H7 (ST device headers)
    ${cmsis_device_h7_SOURCE_DIR}/Include
    # STM32H7 HAL Driver
    ${stm32h7_hal_SOURCE_DIR}/Inc
    # FreeRTOS Kernel
    ${freertos_kernel_SOURCE_DIR}/include
    ${freertos_kernel_SOURCE_DIR}/portable/GCC/ARM_CM7/r0p1
    # LwIP
    ${lwip_SOURCE_DIR}/src/include
    # LwIP FreeRTOS port (our own sys_arch)
    ${CMAKE_SOURCE_DIR}/LwIP_Port/Inc
    # Ruckig — jerk-limited trajectory generation
    ${ruckig_SOURCE_DIR}/include
)

include_directories(${APP_INCLUDES} ${DRIVER_INCLUDES})

# ===========================================================================
# Source Files
# ===========================================================================

# --- Application sources ---
file(GLOB_RECURSE CORE_SOURCES    "Core/Src/*.c")
file(GLOB_RECURSE BSP_SOURCES     "BSP/Src/*.c")
file(GLOB_RECURSE APP_SOURCES     "App/Src/*.c")
file(GLOB_RECURSE PROTO_SOURCES   "Protocol/Src/*.c")

# --- Startup assembly ---
set(STARTUP_SOURCE
    ${cmsis_device_h7_SOURCE_DIR}/Source/Templates/gcc/startup_stm32h743xx.s
)

# --- CMSIS Device system file ---
# NOTE: We use our own Core/Src/system_stm32h7xx.c (included via CORE_SOURCES glob)
# instead of the template from cmsis_device_h7 to avoid duplicate SystemInit symbols.

# --- STM32H7 HAL sources ---
# Include all HAL .c files; linker --gc-sections removes unused ones
file(GLOB HAL_SOURCES "${stm32h7_hal_SOURCE_DIR}/Src/*.c")
# Remove HAL template files (they conflict with our implementations)
list(FILTER HAL_SOURCES EXCLUDE REGEX ".*_template\\.c$")

# --- FreeRTOS sources ---
set(FREERTOS_SOURCES
    ${freertos_kernel_SOURCE_DIR}/tasks.c
    ${freertos_kernel_SOURCE_DIR}/queue.c
    ${freertos_kernel_SOURCE_DIR}/list.c
    ${freertos_kernel_SOURCE_DIR}/timers.c
    ${freertos_kernel_SOURCE_DIR}/event_groups.c
    ${freertos_kernel_SOURCE_DIR}/stream_buffer.c
    ${freertos_kernel_SOURCE_DIR}/portable/GCC/ARM_CM7/r0p1/port.c
    ${freertos_kernel_SOURCE_DIR}/portable/MemMang/heap_4.c
)

# --- LwIP sources (core + API + netif) ---
file(GLOB LWIP_CORE_SOURCES
    "${lwip_SOURCE_DIR}/src/core/*.c"
    "${lwip_SOURCE_DIR}/src/core/ipv4/*.c"
)
file(GLOB LWIP_API_SOURCES  "${lwip_SOURCE_DIR}/src/api/*.c")
file(GLOB LWIP_NETIF_SOURCES "${lwip_SOURCE_DIR}/src/netif/*.c")
# Remove IPv6 (not needed)
list(FILTER LWIP_NETIF_SOURCES EXCLUDE REGEX ".*lowpan6.*")

# LwIP FreeRTOS port (our sys_arch.c)
set(LWIP_PORT_SOURCES
    ${CMAKE_SOURCE_DIR}/LwIP_Port/Src/sys_arch.c
)

set(LWIP_SOURCES ${LWIP_CORE_SOURCES} ${LWIP_API_SOURCES} ${LWIP_NETIF_SOURCES} ${LWIP_PORT_SOURCES})

# --- Ruckig sources (C++17) ---
file(GLOB RUCKIG_SOURCES "${ruckig_SOURCE_DIR}/src/*.cpp")

# --- Application C++ sources (Ruckig wrapper) ---
file(GLOB APP_CXX_SOURCES "App/Src/*.cpp")

# ===========================================================================
# Executable
# ===========================================================================
add_executable(${PROJECT_NAME}
    ${CORE_SOURCES}
    ${BSP_SOURCES}
    ${APP_SOURCES}
    ${APP_CXX_SOURCES}
    ${PROTO_SOURCES}
    ${STARTUP_SOURCE}
    ${HAL_SOURCES}
    ${FREERTOS_SOURCES}
    ${LWIP_SOURCES}
    ${RUCKIG_SOURCES}
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
