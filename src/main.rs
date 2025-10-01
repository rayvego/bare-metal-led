#![no_std]
#![no_main]

use core::panic::PanicInfo;
use core::ptr::{read_volatile, write_volatile};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

const BCM2712_PCIE_BASE: u64 = 0x1f00000000;
const IO_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xd0000;
const PADS_BANK0_BASE: u64 = BCM2712_PCIE_BASE + 0xf0000;

const GPIO14_CTRL: *mut u32 = (IO_BANK0_BASE + 0x074) as *mut u32;
const PADS14_CTRL: *mut u32 = (PADS_BANK0_BASE + 0x03c) as *mut u32;

const BIT_PADS_OD: u32 = 1 << 7;
const BIT_PADS_IE: u32 = 1 << 6;
const BIT_IO_OEOVER: u32 = 3 << 14;
const BIT_IO_OUTOVER: u32 = 3 << 12;

#[no_mangle]  // ← NO unsafe, NO feature flags needed
pub extern "C" fn _start() -> ! {
    unsafe {
        let mut val = read_volatile(PADS14_CTRL);
        val &= !BIT_PADS_OD;
        val |= BIT_PADS_IE;
        write_volatile(PADS14_CTRL, val);

        let val = read_volatile(GPIO14_CTRL);
        write_volatile(GPIO14_CTRL, val | BIT_IO_OEOVER | BIT_IO_OUTOVER);
    }

    loop {}
}