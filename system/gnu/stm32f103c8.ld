MEMORY
{
  /* Define each memory region */
  Flash (rx) : ORIGIN = 0x08000000, LENGTH = 0x00010000
  Ram  (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00005000
}
  
/* Define a symbol for the top of each memory region */
__top_Flash = 0x08000000 + 0x00010000;
__top_Ram   = 0x20000000 + 0x00005000;


ENTRY(reset_handler) 

GROUP
(
  libgcc.a
  libc_nano.a
  libsupc++_nano.a
  libm.a
  crti.o
  crtn.o
  crtbegin.o
  crtend.o
)

SECTIONS
{
  /* MAIN TEXT SECTION */ 
  .text : ALIGN(4)
    {
    FILL(0xff)
    KEEP(*(.vectors))
    
    /* Global Section Table */
    . = ALIGN(4) ;
    __section_table_start = .;
    __data_section_table = .;
    LONG(LOADADDR(.data));
    LONG(    ADDR(.data)) ;
    LONG(  SIZEOF(.data));
    __data_section_table_end = .;
    __bss_section_table = .;
    LONG(    ADDR(.bss));
    LONG(  SIZEOF(.bss));
    __bss_section_table_end = .;
    __section_table_end = . ;
    /* End of Global Section Table */
    

    *(.after_vectors*)
    
    *(.text*)
    *(.rodata .rodata.*)
    . = ALIGN(4);
    
    /* C++ constructors etc */
    . = ALIGN(4);
    KEEP(*(.init))
    
    . = ALIGN(4);
    __preinit_array_start = .;
    KEEP (*(.preinit_array))
    __preinit_array_end = .;
    
    . = ALIGN(4);
    __init_array_start = .;
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    __init_array_end = .;
    
    KEEP(*(.fini));
    
    . = ALIGN(0x4);
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*crtend.o(.ctors))
    
    . = ALIGN(0x4);
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*crtend.o(.dtors))
    /* End C++ */
    } > Flash

  /*
   * for exception handling/unwind - some Newlib functions (in common
   * with C++ and STDC++) use this. 
   * Use KEEP so not discarded with --gc-sections
   */
  .ARM.extab : ALIGN(4)
    {
    KEEP(*(.ARM.extab* .gnu.linkonce.armextab.*))
    } > Flash
  __exidx_start = .;
  
  .ARM.exidx : ALIGN(4)
    {
    KEEP(*(.ARM.exidx* .gnu.linkonce.armexidx.*))
    } > Flash
  __exidx_end = .;
  
  _etext = .;
    
  
  /* MAIN DATA SECTION */

  .uninit_RESERVED : ALIGN(4)
    {
    KEEP(*(.bss.$RESERVED*))
    . = ALIGN(4) ;
    _end_uninit_RESERVED = .;
    } > Ram

  .data : ALIGN(4)
    {
    FILL(0xff)
    _data = .;
    *(vtable)
    *(.data*)
    . = ALIGN(4) ;
    _edata = .;
    } > Ram AT>Flash


  /* MAIN BSS SECTION */
  .bss : ALIGN(4)
    {
    _bss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4) ;
    _ebss = .;
    PROVIDE(end = .);
    } > Ram
  
  /* PROVIDE(_pvHeapStart = .); */
  PROVIDE(StackTop = __top_Ram - 0x100);
}
