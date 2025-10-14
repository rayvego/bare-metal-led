use core::ptr::write_volatile;

const PT_ENTRIES: usize = 512;

#[repr(C, align(4096))]
struct PageTable {
    entries: [u64; PT_ENTRIES],
}

// Allocate page tables in BSS (will be zeroed before use)
static mut L0_TABLE: PageTable = PageTable { entries: [0; PT_ENTRIES] };
static mut L1_TABLE: PageTable = PageTable { entries: [0; PT_ENTRIES] };
static mut L2_TABLE_LOW: PageTable = PageTable { entries: [0; PT_ENTRIES] };
static mut L2_TABLE_PCIE: PageTable = PageTable { entries: [0; PT_ENTRIES] };

// Page table descriptor bits (ARMv8-A Architecture Reference Manual)
const PT_VALID: u64 = 1 << 0;                    // Valid entry
const PT_TABLE: u64 = 1 << 1;                    // Table descriptor
const PT_BLOCK: u64 = 0 << 1;                    // Block descriptor

// Lower attributes [11:2]
const PT_ATTR_IDX_SHIFT: u64 = 2;                // AttrIndx[2:0] at bits [4:2]
const PT_NS: u64 = 1 << 5;                       // Non-secure bit
const PT_AP_RW_EL1: u64 = 0 << 6;               // Read-Write, EL1 only
const PT_AP_RW_ALL: u64 = 1 << 6;               // Read-Write, EL0 and EL1
const PT_AP_RO_EL1: u64 = 2 << 6;               // Read-Only, EL1 only
const PT_AP_RO_ALL: u64 = 3 << 6;               // Read-Only, EL0 and EL1
const PT_SH_INNER: u64 = 3 << 8;                 // Inner shareable
const PT_AF: u64 = 1 << 10;                      // Access flag (must be 1)

// Upper attributes [63:51]
const PT_UXN: u64 = 1 << 54;                     // Unprivileged Execute Never
const PT_PXN: u64 = 1 << 53;                     // Privileged Execute Never

// Memory attribute indices (for MAIR_EL1)
const ATTR_IDX_NORMAL: u64 = 0;
const ATTR_IDX_DEVICE: u64 = 1;

#[no_mangle]
pub unsafe fn setup_mmu() {
    // --- Configure MAIR_EL1 (Memory Attribute Indirection Register) ---
    // Attr0: Normal memory, Inner/Outer Write-Back Cacheable, Read-Allocate, Write-Allocate
    //        0xFF = 0b11111111 = Outer WB R/W-Allocate, Inner WB R/W-Allocate
    // Attr1: Device-nGnRnE (non-Gathering, non-Reordering, no Early write acknowledgement)
    //        0x00 = Device-nGnRnE
    const MAIR_NORMAL: u64 = 0xFF;
    const MAIR_DEVICE: u64 = 0x00;
    let mair_val = (MAIR_NORMAL << (8 * ATTR_IDX_NORMAL)) | 
                   (MAIR_DEVICE << (8 * ATTR_IDX_DEVICE));

    // --- Configure TCR_EL1 (Translation Control Register) ---
    // See ARMv8-A ARM Figure D5-15 and Table D5-8
    const TCR_T0SZ: u64 = 16;              // 48-bit address space for TTBR0 (64-16=48)
    const TCR_IRGN0_WBWA: u64 = 1;         // Inner Write-Back Write-Allocate (bits 9:8)
    const TCR_ORGN0_WBWA: u64 = 1;         // Outer Write-Back Write-Allocate (bits 11:10)
    const TCR_SH0_INNER: u64 = 3;          // Inner shareable (bits 13:12)
    const TCR_TG0_4K: u64 = 0;             // 4KB granule for TTBR0 (bits 15:14)
    const TCR_IPS_48BIT: u64 = 5;          // 48-bit Intermediate Physical Address (bits 34:32)
    
    // For TTBR1 (not used, but must configure to avoid faults on high addresses)
    const TCR_T1SZ: u64 = 16;              // 48-bit address space for TTBR1
    const TCR_TG1_4K: u64 = 2;             // 4KB granule for TTBR1 (bits 31:30, value 2 = 4KB)
    const TCR_IRGN1_WBWA: u64 = 1;         // Inner Write-Back Write-Allocate (bits 25:24)
    const TCR_ORGN1_WBWA: u64 = 1;         // Outer Write-Back Write-Allocate (bits 27:26)
    const TCR_SH1_INNER: u64 = 3;          // Inner shareable (bits 29:28)
    
    let tcr_val = (TCR_T0SZ << 0) |
                  (TCR_IRGN0_WBWA << 8) |
                  (TCR_ORGN0_WBWA << 10) |
                  (TCR_SH0_INNER << 12) |
                  (TCR_TG0_4K << 14) |
                  (TCR_T1SZ << 16) |
                  (TCR_IRGN1_WBWA << 24) |
                  (TCR_ORGN1_WBWA << 26) |
                  (TCR_SH1_INNER << 28) |
                  (TCR_TG1_4K << 30) |
                  (TCR_IPS_48BIT << 32);

    // --- Build Page Tables ---
    let l0_ptr = L0_TABLE.entries.as_mut_ptr();
    let l1_ptr = L1_TABLE.entries.as_mut_ptr();
    let l2_low_ptr = L2_TABLE_LOW.entries.as_mut_ptr();
    let l2_pcie_ptr = L2_TABLE_PCIE.entries.as_mut_ptr();

    // L0[0] -> L1 Table (covers 0x0000_0000_0000_0000 to 0x0000_007F_FFFF_FFFF, 512GB)
    write_volatile(
        l0_ptr.offset(0),
        (l1_ptr as u64) | PT_VALID | PT_TABLE
    );

    // L1[0] -> L2_LOW Table (covers 0x0 to 0x3FFF_FFFF, 1GB)
    write_volatile(
        l1_ptr.offset(0),
        (l2_low_ptr as u64) | PT_VALID | PT_TABLE
    );

    // Map first 4MB of RAM as Normal memory (covers kernel at 0x80000)
    // Use 2MB blocks, Read-Write at EL1 only, executable
    // L2_LOW[0] -> 2MB block at PA 0x0
    // L2_LOW[1] -> 2MB block at PA 0x200000
    for i in 0..2 {
        let pa = (i as u64) * 0x20_0000; // 2MB blocks
        write_volatile(
            l2_low_ptr.offset(i),
            pa | PT_VALID | PT_BLOCK | PT_AF | PT_SH_INNER | 
            PT_AP_RW_EL1 |                              // Read-Write, EL1 only
            (ATTR_IDX_NORMAL << PT_ATTR_IDX_SHIFT) |
            PT_UXN                                       // User Execute Never (kernel code only)
        );
    }

    // L1[124] -> L2_PCIE Table (covers 0x1F_0000_0000 to 0x1F_3FFF_FFFF, 1GB)
    // Index calculation: 0x1F_0000_0000 >> 30 = 0x7C = 124 decimal
    write_volatile(
        l1_ptr.offset(124),
        (l2_pcie_ptr as u64) | PT_VALID | PT_TABLE
    );

    // Map the entire 1GB PCIe space as Device memory using 2MB blocks
    // This covers GPIO at 0x1f000d0000
    // Device memory must be non-executable, Read-Write at EL1
    for i in 0..512 {
        let pa = 0x1f00000000u64 + ((i as u64) * 0x20_0000); // 2MB blocks
        write_volatile(
            l2_pcie_ptr.offset(i),
            pa | PT_VALID | PT_BLOCK | PT_AF | PT_SH_INNER |
            PT_AP_RW_EL1 |                              // Read-Write, EL1 only
            (ATTR_IDX_DEVICE << PT_ATTR_IDX_SHIFT) |
            PT_UXN | PT_PXN                             // Execute Never at all levels
        );
    }

    // --- Load MMU Configuration Registers ---
    // This follows the sequence from ARMv8-A ARM section D5.10.2
    core::arch::asm!(
        "msr mair_el1, {mair}",
        "msr tcr_el1, {tcr}",
        "msr ttbr0_el1, {ttbr0}",
        "msr ttbr1_el1, xzr",        // Not using TTBR1, set to 0
        "isb",                        // Ensure config is visible
        mair = in(reg) mair_val,
        tcr = in(reg) tcr_val,
        ttbr0 = in(reg) l0_ptr as u64,
    );
}