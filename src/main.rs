#![no_std]
#![no_main]

use core::ptr;

// Base addresses (physical, identity-mapped)
const BCM2712_PCIE_BASE: u64 = 0x1f00000000;
const IO_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xd0000;
const PADS_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xf0000;

// GPIO14 registers (offsets from base)
const GPIO14_CTRL: u64 = IO_BANK0_BASE + 0x074; // GPIO14 control
const PADS14_CTRL: u64 = PADS_BANK0_BASE + 0x03c; // PAD14 control

// Bit definitions
const BIT_PADS_OD: u32 = 1 << 7;   // Output Disable (0 = enabled)
const BIT_PADS_IE: u32 = 1 << 6;   // Input Enable
const BIT_IO_OEOVER: u32 = 0b11 << 14; // Output enable override: 0b11 = drive output
const BIT_IO_OUTOVER: u32 = 0b11 << 12; // Output value override: 0b11 = drive high

#[unsafe(no_mangle)]
pub extern "C" fn _start() -> ! {
    // Safety: We assume these physical addresses are accessible
    unsafe {
        // Configure PAD14: enable output, enable input (for read-back if needed)
        let pad_reg = PADS14_CTRL as *mut u32;
        let mut val = ptr::read_volatile(pad_reg);
        val &= !BIT_PADS_OD; // Enable output
        val |= BIT_PADS_IE;  // Enable input
        ptr::write_volatile(pad_reg, val);

        // Configure GPIO14: override OE and OUT to drive HIGH
        let gpio_reg = GPIO14_CTRL as *mut u32;
        let val = ptr::read_volatile(gpio_reg);
        let new_val = val | BIT_IO_OEOVER | BIT_IO_OUTOVER;
        ptr::write_volatile(gpio_reg, new_val);
    }

    // LED should now be ON (if connected to GPIO14)
    // Halt or spin forever
    loop {
        // Optionally add a delay or WFI
    }
}

// Panic handler (required by Rust)
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}