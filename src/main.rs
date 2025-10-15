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

// global_asm! block remains the same as the previous working version...
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


// Physical addresses calculated from BAR + Offset
const UART0_BASE: u64 = 0x1F00030000;

// PL011 Register Offsets (from bcm2711-peripherals.pdf, Table 172)
const UART_DR_OFFSET:   u64 = 0x00;
const UART_FR_OFFSET:   u64 = 0x18;
const UART_IBRD_OFFSET: u64 = 0x24;
const UART_FBRD_OFFSET: u64 = 0x28;
const UART_LCRH_OFFSET: u64 = 0x2c;
const UART_CR_OFFSET:   u64 = 0x30;
const UART_ICR_OFFSET:  u64 = 0x44;

// Flag Register (FR) bits
const UART_FR_TXFF: u32 = 1 << 5; // Transmit FIFO Full

// Line Control Register (LCRH) bits
const UART_LCRH_WLEN8: u32 = 0b11 << 5; // 8-bit word length
const UART_LCRH_FEN:   u32 = 1 << 4;    // Enable FIFOs

// Control Register (CR) bits
const UART_CR_UARTEN: u32 = 1 << 0; // UART Enable
const UART_CR_TXE:    u32 = 1 << 8; // Transmit Enable
const UART_CR_RXE:    u32 = 1 << 9; // Receive Enable

/// Initializes UART0 for 115200 baud communication.
fn uart_init() {
    unsafe {
        // --- Step 1: Disable the UART to configure it ---
        write_volatile((UART0_BASE + UART_CR_OFFSET) as *mut u32, 0);

        // --- Step 2: Set the Baud Rate ---
        // Formula: BAUDDIV = UARTCLK / (16 * Baud Rate)
        // We assume UARTCLK is 48MHz from the DTS.
        // BAUDDIV = 48,000,000 / (16 * 115200) = 26.0416...
        // IBRD = 26
        // FBRD = integer((0.0416 * 64) + 0.5) = 3
        write_volatile((UART0_BASE + UART_IBRD_OFFSET) as *mut u32, 26);
        write_volatile((UART0_BASE + UART_FBRD_OFFSET) as *mut u32, 3);

        // --- Step 3: Configure the Line Control ---
        // Enable FIFOs and set 8-bit word length, no parity, 1 stop bit.
        write_volatile((UART0_BASE + UART_LCRH_OFFSET) as *mut u32, UART_LCRH_FEN | UART_LCRH_WLEN8);

        // --- Step 4: Clear all interrupts ---
        write_volatile((UART0_BASE + UART_ICR_OFFSET) as *mut u32, 0x7FF);

        // --- Step 5: Enable the UART, TX, and RX ---
        write_volatile((UART0_BASE + UART_CR_OFFSET) as *mut u32, UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE);
    }
}

/// Transmits a single character over UART0.
fn uart_putc(c: char) {
    unsafe {
        // Loop until the transmit FIFO is no longer full.
        while read_volatile((UART0_BASE + UART_FR_OFFSET) as *mut u32) & UART_FR_TXFF != 0 {}
        // Write the character to the data register.
        write_volatile((UART0_BASE + UART_DR_OFFSET) as *mut u32, c as u32);
    }
}

/// Transmits a string slice over UART0.
fn uart_puts(s: &str) {
    for c in s.chars() {
        uart_putc(c);
    }
}

/// **ADD THIS FUNCTION:** Helper to print a u64 in hexadecimal.
fn uart_print_hex(n: u64) {
    uart_puts("0x");
    for i in (0..16).rev() {
        let nibble = (n >> (i * 4)) & 0xF;
        let c = match nibble {
            0..=9 => (b'0' + nibble as u8) as char,
            _ => (b'A' + (nibble - 10) as u8) as char,
        };
        uart_putc(c);
    }
}

#[no_mangle]
pub extern "C" fn rust_main() -> ! {
    // --- GPIO Address and Bitmask Constants (from previous code) ---
    const BCM2712_PCIE_BASE: u64 = 0x1f00000000;
    const IO_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xd0000;
    const PADS_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xf0000;
    
    // GPIO 14 (TXD0)
    const GPIO14_CTRL_ADDR: u64 = IO_BANK0_BASE + 0x074;
    const PADS14_CTRL_ADDR: u64 = PADS_BANK0_BASE + 0x03c;
    // GPIO 15 (RXD0)
    const GPIO15_CTRL_ADDR: u64 = IO_BANK0_BASE + 0x07c;
    const PADS15_CTRL_ADDR: u64 = PADS_BANK0_BASE + 0x040;

    // --- Pad Control Bits ---
    const BIT_PADS_OD: u32 = 1 << 7;  // Output Disable
    const BIT_PADS_IE: u32 = 1 << 6;  // Input Enable
    const BIT_PADS_PUE: u32 = 1 << 3; // Pull-Up Enable

    // --- IO Control Fields ---
    const FUNCSEL_FIELD_LSB: u32 = 0;
    const FUNCSEL_FIELD_MASK: u32 = 0x1F << FUNCSEL_FIELD_LSB;
    const FUNC_UART0: u32 = 4;

    unsafe {
        // --- Step 1: Configure GPIO Pins (The Two Locks) ---
        
        // Configure GPIO 14 as UART0 TX
        let mut pads14_val = read_volatile(PADS14_CTRL_ADDR as *mut u32);
        pads14_val &= !BIT_PADS_OD; // LOCK 1: Enable output driver
        write_volatile(PADS14_CTRL_ADDR as *mut u32, pads14_val);

        let mut io14_val = read_volatile(GPIO14_CTRL_ADDR as *mut u32);
        io14_val &= !FUNCSEL_FIELD_MASK; // Clear the function field
        io14_val |= FUNC_UART0 << FUNCSEL_FIELD_LSB; // LOCK 2: Set function to UART0
        write_volatile(GPIO14_CTRL_ADDR as *mut u32, io14_val);

        // Configure GPIO 15 as UART0 RX
        let mut pads15_val = read_volatile(PADS15_CTRL_ADDR as *mut u32);
        pads15_val |= BIT_PADS_IE; // LOCK 1: Enable input buffer
        pads15_val |= BIT_PADS_PUE; // Enable pull-up as per DTS
        write_volatile(PADS15_CTRL_ADDR as *mut u32, pads15_val);

        let mut io15_val = read_volatile(GPIO15_CTRL_ADDR as *mut u32);
        io15_val &= !FUNCSEL_FIELD_MASK; // Clear the function field
        io15_val |= FUNC_UART0 << FUNCSEL_FIELD_LSB; // LOCK 2: Set function to UART0
        write_volatile(GPIO15_CTRL_ADDR as *mut u32, io15_val);
        
        // --- Step 2: Initialize and use the UART ---
        uart_init();
        uart_puts("Hello from Rust with the MMU enabled!\r\n");

        // --- ADD THIS BLOCK FOR VERIFICATION ---
        uart_puts("\r\n--- MMU VERIFICATION ---\r\n");
        let sctlr: u64;
        core::arch::asm!("mrs {}, sctlr_el1", out(reg) sctlr);

        uart_puts("SCTLR_EL1 = ");
        uart_print_hex(sctlr);
        uart_puts("\r\n");

        if (sctlr & 1) == 1 {
            uart_puts("Result: MMU is ON (Bit 0 is 1).\r\n");
        } else {
            uart_puts("Result: MMU is OFF (Bit 0 is 0).\r\n");
        }
        uart_puts("------------------------\r\n");

        // --- THE ULTIMATE PROOF ---
        uart_puts("Now, attempting to read from an unmapped address...\r\n");
        uart_puts("If the kernel hangs here, the MMU is working and protecting memory.\r\n");

        // This address is guaranteed not to be in our page tables.
        let invalid_address = 0xDEADBEEF_CAFEBABE as *const u64;
        let _ = read_volatile(invalid_address);

        // This line should NEVER be printed.
        uart_puts("!!! TEST FAILED: Invalid memory access succeeded. MMU might be OFF. !!!\r\n");
        // --- END OF VERIFICATION BLOCK ---
    }

    loop {}
}