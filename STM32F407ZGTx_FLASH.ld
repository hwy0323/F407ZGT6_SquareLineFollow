ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);
MEMORY { FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K }
SECTIONS {
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) _etext = .; } > FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
  _sidata = LOADADDR(.data);
  .data : { _sdata = .; *(.data*) _edata = .; } > RAM AT > FLASH
  .bss (NOLOAD) : { _sbss = .; *(.bss*) *(COMMON) _ebss = .; } > RAM
  ._user_heap_stack : { . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .); . = . + 0x400; . = . + 0x800; . = ALIGN(8); } > RAM
}
