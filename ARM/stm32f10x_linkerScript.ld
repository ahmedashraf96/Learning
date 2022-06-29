ENTRY(Reset_Handler)

MEMORY
{
    FLASH(rx) : ORIGIN = 0x08000000, LENGTH = 64K
    SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS
{
    .vect :
    {
        *(.isr_vectors)
        . = ALIGN(4);
    }> FLASH
    
    .text :
    {
        *(.ARM.*)
        *(.text.startup)
        *(.text)
        *(.rodata)
        . = ALIGN(4);
        _etext = .;
    }> FLASH

    .data :
    {
        _sdata = .;
        *(.data)
        _edata = .;
        . = ALIGN(4);
    }> SRAM AT> FLASH 

    .bss :
    {
        _sbss = .;
        *(.bss)
        _ebss = .;
        . = ALIGN(4);
    }> SRAM
}
