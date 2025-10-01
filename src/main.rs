// 1. Tell Rust we are not using the standard library.
#![no_std]
// 2. Tell Rust we are not using the standard main function entry point.
#![no_main]

use core::panic::PanicInfo;
use core::ptr::{read_volatile, write_volatile};

// 3. Provide a panic handler. This function is called if the program panics.
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // A real program would handle this, but we just loop forever.
    loop {}
}

// Memory addresses for GPIO14 on the Raspberry Pi 5.
const BCM2712_PCIE_BASE: u64 = 0x1f00000000;
const IO_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xd0000;
const PADS_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xf0000;

const GPIO14_CTRL: *mut u32 = (IO_BANK0_BASE + 0x074) as *mut u32;
const PADS14_CTRL: *mut u32 = (PADS_BANK0_BASE + 0x03c) as *mut u32;

// Bitmasks for register control.
const BIT_PADS_OD: u32 = 1 << 7; // Output Disable
const BIT_PADS_IE: u32 = 1 << 6; // Input Enable
const BIT_IO_OEOVER: u32 = 3 << 14; // Output Enable Override
const BIT_IO_OUTOVER: u32 = 3 << 12; // Output Value Override

// 4. Define our own entry point.
// `#[no_mangle]` ensures the function name isn't changed by the compiler.
// `extern "C"` uses the C calling convention, making it easy for the linker to find.
#[unsafe(no_mangle)]
pub extern "C" fn _start() -> ! {
    // In Rust, we use read_volatile and write_volatile for memory-mapped I/O.
    // This is the safe equivalent of using `volatile` pointers in C.
    unsafe {
        // Configure the pad control register
        let mut pdctrl_val = read_volatile(PADS14_CTRL);
        pdctrl_val &= !BIT_PADS_OD; // Enable output
        pdctrl_val |= BIT_PADS_IE;  // Enable input
        write_volatile(PADS14_CTRL, pdctrl_val);

        // Configure the GPIO control register
        let mut ioctrl_val = read_volatile(GPIO14_CTRL);
        ioctrl_val |= BIT_IO_OEOVER | BIT_IO_OUTOVER; // Force output high
        write_volatile(GPIO14_CTRL, ioctrl_val);
    }

    // Bare-metal programs don't exit.
    loop {}
}