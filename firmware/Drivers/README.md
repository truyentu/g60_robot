# Drivers — External Dependencies

This folder contains external libraries required by the firmware.
They are NOT included in the repository — you must download and place them here.

## Required Components

### 1. STM32CubeH7 HAL Driver

**Source**: https://github.com/STMicroelectronics/STM32CubeH7

Copy these folders from the STM32CubeH7 repository:

```
Drivers/
├── STM32H7xx_HAL_Driver/
│   ├── Inc/           ← from STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Inc/
│   └── Src/           ← from STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/
│
└── CMSIS/
    ├── Device/ST/STM32H7xx/
    │   ├── Include/   ← stm32h7xx.h, stm32h743xx.h, system_stm32h7xx.h
    │   └── Source/Templates/gcc/
    │       └── startup_stm32h743xx.s   ← Startup assembly file
    │
    └── Include/       ← core_cm7.h, cmsis_gcc.h, etc.
```

### 2. FreeRTOS Kernel

**Source**: https://github.com/FreeRTOS/FreeRTOS-Kernel

```
Drivers/
└── FreeRTOS/
    ├── include/       ← FreeRTOS.h, task.h, queue.h, etc.
    ├── tasks.c
    ├── queue.c
    ├── list.c
    ├── timers.c
    ├── event_groups.c
    ├── stream_buffer.c
    └── portable/
        ├── GCC/ARM_CM7/r0p1/
        │   ├── port.c
        │   └── portmacro.h
        └── MemMang/
            └── heap_4.c
```

### 3. LwIP Stack

**Source**: https://github.com/lwip-tcpip/lwip (or from STM32CubeH7 Middlewares)

```
Drivers/
└── LwIP/
    ├── src/
    │   ├── include/   ← lwip/*.h, netif/*.h
    │   ├── core/      ← tcp.c, udp.c, ip.c, etc.
    │   ├── api/       ← netconn.c, sockets.c, etc.
    │   └── netif/     ← etharp.c, ethernet.c
    └── system/
        └── OS/        ← sys_arch.c/h for FreeRTOS integration
```

**Note**: The LwIP FreeRTOS port (`sys_arch.c`) is typically found in:
`STM32CubeH7/Middlewares/Third_Party/LwIP/system/OS/sys_arch.c`

## Quick Setup

```bash
# Clone STM32CubeH7 (large repo, use shallow clone)
git clone --depth 1 https://github.com/STMicroelectronics/STM32CubeH7.git /tmp/cube

# Copy HAL
cp -r /tmp/cube/Drivers/STM32H7xx_HAL_Driver firmware/Drivers/
cp -r /tmp/cube/Drivers/CMSIS firmware/Drivers/

# Copy LwIP
cp -r /tmp/cube/Middlewares/Third_Party/LwIP firmware/Drivers/

# Clone FreeRTOS Kernel
git clone --depth 1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git firmware/Drivers/FreeRTOS
```

## Build

```bash
cd firmware
cmake -B build -S . --toolchain arm-none-eabi-gcc.cmake
cmake --build build
```

Output: `build/robot_firmware.elf`, `.bin`, `.hex`
