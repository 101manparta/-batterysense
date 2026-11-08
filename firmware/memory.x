/* Linker script for STM32G474RE — dual-bank flash layout for A/B OTA */
MEMORY
{
    /* Flash Bank 1 (Active) — 256 KB */
    FLASH_BANK1 (rx) : ORIGIN = 0x08000000, LENGTH = 256K
    /* Flash Bank 2 (Staging) — 256 KB */
    FLASH_BANK2 (rx) : ORIGIN = 0x08040000, LENGTH = 256K
    /* Configuration sector — 8 KB (shared) */
    CONFIG_FLASH (rw) : ORIGIN = 0x08080000, LENGTH = 8K
    /* Calibration data — 8 KB */
    CALIB_FLASH (rw) : ORIGIN = 0x08082000, LENGTH = 8K
    /* OTA manifest — 4 KB */
    OTA_MANIFEST (rw) : ORIGIN = 0x08084000, LENGTH = 4K
    /* SRAM1 — 128 KB */
    RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
    /* SRAM2 — 32 KB (parity protected, used for safety-critical data) */
    RAM_SAFETY (xrw) : ORIGIN = 0x20020000, LENGTH = 32K
    /* CCM RAM — 32 KB (tightly coupled, zero wait) */
    CCM_RAM (xrw) : ORIGIN = 0x10000000, LENGTH = 32K
}

/* Stack/heap configuration */
_estack = ORIGIN(RAM) + LENGTH(RAM);
_min_stack_size = 4K;
_min_heap_size = 2K;

/* Section placement */
SECTIONS
{
    .text :
    {
        KEEP(*(.vector_table))
        *(.text*)
        *(.rodata*)
        *(.eh_frame*)
    } > FLASH_BANK1

    .text_bank2 :
    {
        *(.text_bank2*)
    } > FLASH_BANK2

    .config : ALIGN(4K)
    {
        *(.config*)
    } > CONFIG_FLASH

    .calibration : ALIGN(4K)
    {
        *(.calibration*)
    } > CALIB_FLASH

    .ota_manifest : ALIGN(4K)
    {
        *(.ota_manifest*)
    } > OTA_MANIFEST

    .data : ALIGN(4)
    {
        _sidata = LOADADDR(.data);
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM AT > FLASH_BANK1

    .bss : ALIGN(4)
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM

    .safety_data : ALIGN(4)
    {
        _ssafety = .;
        *(.safety_data*)
        _esafety = .;
    } > RAM_SAFETY

    .ccm_data : ALIGN(4)
    {
        _sccm = .;
        *(.ccm_data*)
        _eccm = .;
    } > CCM_RAM

    .stack (NOLOAD) : ALIGN(8)
    {
        _sstack = .;
        . += _min_stack_size;
        _estack = .;
    } > RAM

    .heap (NOLOAD) : ALIGN(8)
    {
        _sheap = .;
        . += _min_heap_size;
        _eheap = .;
    } > RAM

    /DISCARD/ :
    {
        *(.ARM.exidx*)
        *(.note.gnu.property*)
    }
}