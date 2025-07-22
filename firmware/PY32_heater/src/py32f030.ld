/*
******************************************************************************
**
**  File        : py32f030.ld
**
**  Abstract    : Linker script for PY32F030 series
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM); /* end of RAM */
_Min_Heap_Size = 0x80;    /* 128 bytes for heap */
_Min_Stack_Size = 0x200; /* 512 bytes for stack */

/* Specify the memory areas */
MEMORY
{
  RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 4K
  FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 32K
}

/* Define output sections */
SECTIONS
{
  /* Vector table in Flash */
  .vectors :
  {
    . = ALIGN(4);
    KEEP(*(.vectors)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* Program code and constants */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;        /* define a global symbol at end of code */
  } >FLASH

  /* Constant data in Flash */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* Initialized data (.data) */
  .data : ALIGN(4)
  {
    __data_start = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    __data_end = .;
  } >RAM AT> FLASH
  __data_load_start = LOADADDR(.data);

  /* Uninitialized data (.bss) */
  .bss : ALIGN(4)
  {
    __bss_start = .;
    PROVIDE(__bss_start__ = __bss_start); /* For crt0.o */
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end = .;
    PROVIDE(__bss_end__ = __bss_end); /* For crt0.o */
  } >RAM

  /* Heap */
  .heap : ALIGN(8)
  {
    __heap_base = .;
    . = . + _Min_Heap_Size; /* 128 bytes for heap */
    __heap_limit = .;
  } >RAM

  /* Stack */
  .stack : ALIGN(8)
  {
    . = . + _Min_Stack_Size; /* 256 bytes for stack */
    __initial_sp = .;
    PROVIDE(_estack = .); /* For compatibility */
  } >RAM

  /* Check that stack and heap fit in RAM */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = ALIGN(8);
  } >RAM

  /* Remove unnecessary library sections */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}