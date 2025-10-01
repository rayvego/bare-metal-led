/* Define the program's entry point. This must match the function name in our Rust code. */
ENTRY(_start)

SECTIONS
{
    /* Start placing code at the physical address 0x80000. */
    . = 0x80000;

    /* Place the code section first. */
    .text : {
        *(.text*)
    }

    /* Place other sections after the code. */
    .rodata : { *(.rodata*) }
    .data : { *(.data*) }
    .bss : { *(.bss*) }
}