#![no_std]
#![no_main]

use core::arch::global_asm;
use core::panic::PanicInfo;
use core::ptr::{read_volatile, write_volatile};

mod mmu;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

global_asm!(
    ".section .text.boot",
    ".global _start",
    "_start:",
    
    // --- Step 1: Check for Primary Core ---
    "   mrs  x0, mpidr_el1",
    "   and  x0, x0, #3",
    "   cbz  x0, .L_core0_main",
    
    ".L_secondary_core_sleep:",
    "   wfe",
    "   b    .L_secondary_core_sleep",

    ".L_core0_main:",
    
    // --- Step 2: Check Current Exception Level ---
    "   mrs  x0, CurrentEL",
    "   and  x0, x0, #0xC",
    "   lsr  x0, x0, #2",
    
    // If we're in EL2, drop to EL1
    "   cmp  x0, #2",
    "   b.ne .L_skip_el2_to_el1",
    
    // Configure EL1 entry
    "   mov  x0, #0x3c5",           // EL1h (use SP_EL1), AArch64
    "   msr  spsr_el2, x0",
    "   adr  x0, .L_skip_el2_to_el1",
    "   msr  elr_el2, x0",
    
    // Disable EL2 traps
    "   mov  x0, #0x33ff",
    "   msr  cptr_el2, x0",         // Don't trap FP/SIMD
    "   msr  hstr_el2, xzr",        // Don't trap system registers
    "   mov  x0, #(3 << 20)",
    "   msr  cpacr_el1, x0",        // Enable FP/SIMD at EL1
    
    // Enable physical timer for EL1
    "   mrs  x0, cnthctl_el2",
    "   orr  x0, x0, #3",
    "   msr  cnthctl_el2, x0",
    "   msr  cntvoff_el2, xzr",
    
    // Disable MMU/caches at EL2 before dropping
    "   mrs  x0, sctlr_el2",
    "   bic  x0, x0, #(1 << 0)",    // Clear M bit
    "   bic  x0, x0, #(1 << 2)",    // Clear C bit
    "   bic  x0, x0, #(1 << 12)",   // Clear I bit
    "   msr  sctlr_el2, x0",
    
    // Set EL1 execution state
    "   mov  x0, #(1 << 31)",       // AArch64
    "   orr  x0, x0, #(1 << 1)",    // SWIO hardwired
    "   msr  hcr_el2, x0",
    "   isb",
    
    "   eret",                       // Drop to EL1
    
    ".L_skip_el2_to_el1:",
    
    // --- Step 3: Set Stack Pointer ---
    "   ldr  x0, =_start",
    "   mov  sp, x0",

    // --- Step 4: Clean and Invalidate Data Cache ---
    "   mrs  x0, clidr_el1",
    "   and  x0, x0, #0x7000000",
    "   lsr  x0, x0, #23",
    "   cbz  x0, .L_finished_cache_cleaning",
    "   mov  x10, x0",

    "   mov  x1, #0",
    ".L_cache_level_loop:",
    "   add  x2, x1, x1, lsr #1",
    "   lsl  x0, x1, #1",
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
    "   dsb  sy",

    // --- Step 5: Clear BSS ---
    "   ldr  x0, =__bss_start",
    "   ldr  x1, =__bss_end",
    "   sub  x1, x1, x0",
    "   bl   memset_zero",

    // --- Step 6: Setup Page Tables ---
    "   bl   setup_mmu",

// --- Step 7: Prepare to enable MMU ---
    "   ic   iallu",                 // Invalidate instruction cache
    "   dsb  sy",                    // Full system barrier to ensure all prior ops complete
    "   isb",                        // Synchronize context, flush pipeline

    // --- Step 8: Enable the MMU ---
    "   mrs  x0, sctlr_el1",
    "   orr  x0, x0, #(1 << 0)",    // M bit: MMU enable
    "   orr  x0, x0, #(1 << 2)",    // C bit: Data cache enable
    "   orr  x0, x0, #(1 << 12)",   // I bit: Instruction cache enable
    "   msr  sctlr_el1, x0",
    "   isb",                        // Critical: ensure MMU enable is seen by next instruction

    // --- Step 9: Jump to Rust ---
    "   bl   rust_main",
    
    "loop:",
    "   wfe",
    "   b    loop",

    "memset_zero:",
    "   cbz  x1, .L_memset_zero_end",
    "   str  xzr, [x0], #8",
    "   subs x1, x1, #8",
    "   b.gt memset_zero",
    ".L_memset_zero_end:",
    "   ret",
);

/// Simple delay function for LED blinking
fn delay(count: u32) {
    for _ in 0..count {
        unsafe {
            core::arch::asm!("nop");
        }
    }
}

#[no_mangle]
pub extern "C" fn rust_main() -> ! {
    const BCM2712_PCIE_BASE: u64 = 0x1f00000000;
    const IO_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xd0000;
    const PADS_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xf0000;

    const GPIO14_CTRL_ADDR: u64 = IO_BANK0_BASE + 0x074;
    const PADS14_CTRL_ADDR: u64 = PADS_BANK0_BASE + 0x03c;
    
    const BIT_PADS_OD: u32 = 1 << 7;
    const BIT_PADS_IE: u32 = 1 << 6;
    const BIT_IO_OEOVER: u32 = 3 << 14;
    const BIT_IO_OUTOVER: u32 = 3 << 12;

    let gpio_ctrl = GPIO14_CTRL_ADDR as *mut u32;
    let pad_ctrl = PADS14_CTRL_ADDR as *mut u32;

    unsafe {
        // Configure the Physical Pad
        let mut pdctrl = read_volatile(pad_ctrl);
        pdctrl &= !BIT_PADS_OD;
        pdctrl |= BIT_PADS_IE;
        write_volatile(pad_ctrl, pdctrl);

        // Configure the Logical IO - turn LED on
        let mut ioctrl = read_volatile(gpio_ctrl);
        ioctrl |= BIT_IO_OEOVER | BIT_IO_OUTOVER;
        write_volatile(gpio_ctrl, ioctrl);
    }

    // Blink to verify MMU is working
    loop {
    }
}