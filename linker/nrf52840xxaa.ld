/*
 * Linker script for Nordic Semiconductor nRF52840 (xxAA)
 * 1 MB Flash (0x00000000 - 0x00100000), 256 KB RAM (0x20000000 - 0x20040000)
 *
 * Flash layout (with SoftDevice S140 v7.3.0 & wear leveling):
 *   0x00000000 .. SD_FLASH_SIZE   : MBR + SoftDevice S140 (0x27000 when BLE enabled)
 *   SD_FLASH_SIZE .. WL_BASE_ADDRESS : Application firmware
 *   WL_BASE_ADDRESS .. FLASH_SIZE : Wear-leveling persistent configuration
 *   FLASH_SIZE .. 0x00100000      : Reserved for optional Nordic DFU bootloader / MBR params
 *
 * RAM layout:
 *   0x20000000 .. 0x20000000 + SD_RAM_SIZE : SoftDevice S140 RAM (0x4000 when BLE enabled)
 *   __ram_start__ .. 0x20040000            : Application RAM
 */

ENTRY(Reset_Handler)

BASE_ADDR = 0x00000000;
RAM_BASE  = 0x20000000;
RAM_TOTAL = 256K;

MEMORY
{
  FLASH    (rx)  : ORIGIN = BASE_ADDR + SD_FLASH_SIZE, LENGTH = WL_BASE_ADDRESS - SD_FLASH_SIZE
  FLASH_WL (r)   : ORIGIN = BASE_ADDR + WL_BASE_ADDRESS, LENGTH = FLASH_SIZE - WL_BASE_ADDRESS
  RAM      (xrw) : ORIGIN = RAM_BASE + SD_RAM_SIZE, LENGTH = RAM_TOTAL - SD_RAM_SIZE
}

__ram_start__ = ORIGIN(RAM);
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

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
    __etext = .;
  } > FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  .ARM.extab :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } > FLASH

  .ARM :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } > FLASH

  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
    . = ALIGN(4);
  } > FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
    . = ALIGN(4);
  } > FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
    . = ALIGN(4);
  } > FLASH

  /* Symbols required by Nordic nrfx gcc_startup_nrf52840.S */
  _sidata = LOADADDR(.data);
  __data_load_start = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    __data_start = .;
    __data_start__ = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
    __data_end = .;
    __data_end__ = .;
  } > RAM AT > FLASH

  __sdata_load_start = LOADADDR(.sdata);
  .sdata :
  {
    . = ALIGN(4);
    __sdata_start = .;
    *(.sdata*)
    . = ALIGN(4);
    __sdata_end = .;
  } > RAM AT > FLASH

  __tdata_load_start = LOADADDR(.tdata);
  .tdata :
  {
    . = ALIGN(4);
    __tdata_start = .;
    *(.tdata*)
    . = ALIGN(4);
    __tdata_end = .;
  } > RAM AT > FLASH

  __fast_load_start = LOADADDR(.fast);
  .fast :
  {
    . = ALIGN(4);
    __fast_start = .;
    *(.fast*)
    . = ALIGN(4);
    __fast_end = .;
  } > RAM AT > FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = .;
  } > RAM

  .tbss (NOLOAD) :
  {
    . = ALIGN(4);
    __tbss_start__ = .;
    *(.tbss*)
    . = ALIGN(4);
    __tbss_end__ = .;
  } > RAM

  .sbss (NOLOAD) :
  {
    . = ALIGN(4);
    __sbss_start__ = .;
    *(.sbss*)
    . = ALIGN(4);
    __sbss_end__ = .;
  } > RAM

  .heap (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    KEEP(*(.heap*))
    . = ALIGN(8);
  } > RAM

  .stack (NOLOAD) :
  {
    . = ALIGN(8);
    KEEP(*(.stack*))
    . = ALIGN(8);
  } > RAM

  ASSERT(__StackTop <= ORIGIN(RAM) + LENGTH(RAM), "RAM overflowed with stack")

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
