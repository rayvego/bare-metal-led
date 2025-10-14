ENTRY(_start)

SECTIONS
{
    . = 0x80000;
    
    .text : {
        *(.text.boot)
        *(.text*)
    }
    
    .rodata : {
        *(.rodata*)
    }
    
    .data : {
        *(.data*)
    }
    
    .bss : {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    }
    
    /* Reserve space for page tables right after BSS */
    . = ALIGN(4096);
    __page_tables_start = .;
    . += 0x4000; /* 16KB for L0, L1, L2 tables */
    __page_tables_end = .;
    
    /* Stack grows downward from _start (0x80000) */
    
    /DISCARD/ : {
        *(.comment)
        *(.gnu*)
        *(.note*)
        *(.eh_frame*)
    }
}