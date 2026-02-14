/**
 * Linker script for STM32H743ZITx
 * Flash: 2MB, Multi-domain RAM architecture
 *
 * Memory layout:
 *   DTCM  (128KB) — Zero-wait-state, stack + critical data
 *   AXI   (512KB) — Main heap, general data
 *   SRAM1 (128KB) — D2 domain, DMA-capable (Ethernet DMA descriptors + buffers)
 *   SRAM2 (128KB) — D2 domain, DMA-capable
 *   SRAM3 (32KB)  — D2 domain
 *   SRAM4 (64KB)  — D3 domain, available in low-power
 */

/* Entry point */
ENTRY(Reset_Handler)

/* Stack & Heap sizes */
_Min_Heap_Size  = 0x4000;  /* 16KB minimum heap */
_Min_Stack_Size = 0x2000;  /* 8KB minimum stack */

MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
    DTCM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K    /* Fast, zero-wait-state */
    AXI_SRAM (rwx) : ORIGIN = 0x24000000, LENGTH = 512K  /* Main heap */
    SRAM1  (rwx) : ORIGIN = 0x30000000, LENGTH = 128K    /* D2 — ETH DMA */
    SRAM2  (rwx) : ORIGIN = 0x30020000, LENGTH = 128K    /* D2 */
    SRAM3  (rwx) : ORIGIN = 0x30040000, LENGTH = 32K     /* D2 */
    SRAM4  (rwx) : ORIGIN = 0x38000000, LENGTH = 64K     /* D3 */
}

SECTIONS
{
    /* ------------------------------------------------------------------ */
    /* Interrupt vector table — must be at start of Flash                  */
    /* ------------------------------------------------------------------ */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* ------------------------------------------------------------------ */
    /* Code and read-only data                                             */
    /* ------------------------------------------------------------------ */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH

    /* ------------------------------------------------------------------ */
    /* Initialized data — loaded from Flash, runs in AXI SRAM              */
    /* ------------------------------------------------------------------ */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        *(.RamFunc)
        *(.RamFunc*)
        . = ALIGN(4);
        _edata = .;
    } >AXI_SRAM AT> FLASH

    /* ------------------------------------------------------------------ */
    /* BSS (zero-initialized) — AXI SRAM                                   */
    /* ------------------------------------------------------------------ */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >AXI_SRAM

    /* ------------------------------------------------------------------ */
    /* Heap — AXI SRAM (after BSS)                                         */
    /* ------------------------------------------------------------------ */
    ._user_heap :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = ALIGN(8);
    } >AXI_SRAM

    /* ------------------------------------------------------------------ */
    /* Stack — DTCM (fast, zero-wait-state)                                */
    /* ------------------------------------------------------------------ */
    ._user_stack :
    {
        . = ALIGN(8);
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >DTCM

    _estack = ORIGIN(DTCM) + LENGTH(DTCM);  /* Top of DTCM = initial SP */

    /* ------------------------------------------------------------------ */
    /* Ethernet DMA — MUST be in D2 SRAM (SRAM1)                           */
    /* DMA descriptors and buffers cannot be in AXI SRAM on H7!            */
    /* ------------------------------------------------------------------ */
    .lwip_sec (NOLOAD) :
    {
        . = ALIGN(4);
        *(.RxDecripSection)         /* ETH Rx DMA descriptors */
        *(.TxDecripSection)         /* ETH Tx DMA descriptors */
        . = ALIGN(4);
        *(.RxArraySection)          /* ETH Rx buffers */
        *(.TxArraySection)          /* ETH Tx buffers */
        . = ALIGN(4);
    } >SRAM1

    /* ------------------------------------------------------------------ */
    /* D3 SRAM4 — available for low-power or special use                   */
    /* ------------------------------------------------------------------ */
    .sram4 (NOLOAD) :
    {
        . = ALIGN(4);
        *(.sram4)
        *(.sram4*)
        . = ALIGN(4);
    } >SRAM4

    /* Discard debug sections to reduce binary size */
    /DISCARD/ :
    {
        libc.a(*)
        libm.a(*)
        libgcc.a(*)
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
