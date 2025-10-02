#![no_std]
#![no_main]

use core::arch::global_asm;
use core::panic::PanicInfo;
use core::ptr::{read_volatile, write_volatile};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

global_asm!(
    ".section .text.boot",
    ".global _start",
    "_start:",
    // --- Step 1: Read CPU ID to check if we are the primary core (Core 0) ---
    "   mrs  x0, mpidr_el1",
    "   and  x0, x0, #3", 
    "   cbz  x0, .L_core0_main", 
    // --- If not Core 0, put this core into a holding pattern ---
    ".L_secondary_core_sleep:",
    "   wfe",
    "   b    .L_secondary_core_sleep",

    ".L_core0_main:",
    // --- Step 2: Set up a temporary stack right below the kernel load address ---
    "   ldr  x0, =_start",
    "   mov  sp, x0",

    // --- Step 3: Clean and Invalidate the Data Cache (D-Cache) ---
    "   mrs  x0, clidr_el1",      
    "   and  x0, x0, #0x7000000", 
    "   lsr  x0, x0, #23",        
    "   cbz  x0, .L_finished_cache_cleaning", 
    "   mov  x10, x0",            

    "   mov  x1, #0",             
    ".L_cache_level_loop:",
    "   add  x2, x1, x1, lsr #1", 
    "   lsl  x0, x1, #1",     // x0 = level * 2 (for CSSELR shift)
    "   msr  csselr_el1, x0",     
    "   isb",                    
    "   mrs  x0, ccsidr_el1",     
    "   and  x2, x0, #7",         
    "   add  x2, x2, #4",         
    "   ubfx x3, x0, #3, #10",    
    "   ubfx x4, x0, #13, #15",   
    
    ".L_cache_set_loop:",
    "   mov  x5, x3",
    ".L_cache_way_loop:",
    "   lsl  x6, x5, #30",
    "   lsl  x7, x1, #1",
    "   lsl  x9, x4, x2",
    "   orr  x6, x6, x7",
    "   orr  x6, x6, x9",
    "   dc   cisw, x6",
    "   subs x5, x5, #1",
    "   bge  .L_cache_way_loop",
    "   subs x4, x4, #1",
    "   bge  .L_cache_set_loop",

    "   add  x1, x1, #1",
    "   cmp  x1, x10",
    "   b.lt .L_cache_level_loop",

    ".L_finished_cache_cleaning:",
    "   dsb  ish",

    // --- Step 4: Disable MMU, Caches, and Invalidate TLBs ---
    "   ic   iallu",
    "   tlbi vmalle1is",
    "   dsb  ish",
    "   isb",

    "   mrs  x0, sctlr_el1",
    "   bic  x0, x0, #(1 << 0)",
    "   bic  x0, x0, #(1 << 2)",
    "   bic  x0, x0, #(1 << 12)",
    "   msr  sctlr_el1, x0",
    "   isb",

    // --- Step 5: Jump to Rust code after clearing BSS ---
    "   ldr  x0, =__bss_start",
    "   ldr  x1, =__bss_end",
    "   sub  x1, x1, x0",
    "   bl   memset_zero",
    "   bl   rust_main",
    
    // --- Loop forever if rust_main returns ---
    "loop:",
    "   wfe",
    "   b    loop",

    // Helper function to zero memory for the BSS section.
    "memset_zero:",
    "   cbz  x1, .L_memset_zero_end",
    "   str  xzr, [x0], #8",
    "   sub  x1, x1, #8",
    "   b    memset_zero",
    ".L_memset_zero_end:",
    "   ret",
);


/// The main Rust function, called after assembly setup.
#[no_mangle]
pub extern "C" fn rust_main() -> ! {
    const BCM2712_PCIE_BASE: u64 = 0x1f00000000;
    const IO_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xd0000;
    const PADS_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xf0000;

    const GPIO14_CTRL_ADDR: u64 = IO_BANK0_BASE + 0x074;
    const PADS14_CTRL_ADDR: u64 = PADS_BANK0_BASE + 0x03c;
    
    const BIT_PADS_OD: u32 = 1 << 7; // For bit 7 of the Pad Controller
    const BIT_PADS_IE: u32 = 1 << 6; // For bit 6 of the Pad Controller
    const BIT_IO_OEOVER: u32 = 3 << 14; // For bits 15:14 of the IO Controller
    const BIT_IO_OUTOVER: u32 = 3 << 12; // For bits 13:12 of the IO Controller

    // unsafe raw pointer in rust
    let gpio_ctrl = GPIO14_CTRL_ADDR as *mut u32;
    let pad_ctrl = PADS14_CTRL_ADDR as *mut u32;

    unsafe {
        // 1. Configure the Physical Pad
        let mut pdctrl = read_volatile(pad_ctrl);
        pdctrl &= !BIT_PADS_OD; // Enable the output driver
        pdctrl |= BIT_PADS_IE;
        write_volatile(pad_ctrl, pdctrl);

        // 2. Configure the Logical IO
        let mut ioctrl = read_volatile(gpio_ctrl);
        ioctrl |= BIT_IO_OEOVER | BIT_IO_OUTOVER; // Force high output
        write_volatile(gpio_ctrl, ioctrl);
    }

    loop {}
}