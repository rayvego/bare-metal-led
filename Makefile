# The name of your Rust project from Cargo.toml
PROJECT_NAME = bare_metal_led

# The target triple for Raspberry Pi 5
TARGET_TRIPLE = aarch64-unknown-none

# --- Executables ---
GDB = aarch64-elf-gdb
OPENOCD = openocd
OBJCOPY = rust-objcopy

# --- File Paths ---
# Path to the UN-STRIPPED debug ELF file (for GDB)
KERNEL_ELF_DEBUG = target/$(TARGET_TRIPLE)/debug/$(PROJECT_NAME)

# Path to the STRIPPED release binary (for the SD card)
KERNEL_IMG = kernel8.img
KERNEL_ELF_RELEASE = target/$(TARGET_TRIPLE)/release/$(PROJECT_NAME)

# --- OpenOCD Config ---
# Assumes these files are in the directory where you run 'make'
# or in OpenOCD's default search path.
OPENOCD_INTERFACE_CFG = interface/cmsis-dap.cfg
OPENOCD_TARGET_CFG = target/raspberrypi5.cfg

.PHONY: all build build-release image openocd-pi5 gdb-pi5 clean

##
## Build Targets
##

# Default target: build for debug
all: build

# Build the debug ELF file with symbols.
build:
	@echo "--- Building for Debug ---"
	@cargo build

# Build the release ELF file, optimized and without symbols.
build-release:
	@echo "--- Building for Release ---"
	@cargo build --release

# Create the bootable kernel8.img from the release build.
image: build-release
	@echo "--- Creating Bootable Image ($(KERNEL_IMG)) ---"
	@$(OBJCOPY) --strip-all -O binary $(KERNEL_ELF_RELEASE) $(KERNEL_IMG)

##
## Debugging Targets
##

# Start the OpenOCD server.
openocd-pi5:
	@$(OPENOCD) -f $(OPENOCD_INTERFACE_CFG) -f $(OPENOCD_TARGET_CFG)

# Build for debug, then launch GDB and run the init script.
gdb-pi5: build
	@echo "--- Launching GDB ---"
	@$(GDB) -x gdb-init.txt $(KERNEL_ELF_DEBUG)

##
## Cleanup
##

# Remove all build artifacts.
clean:
	@cargo clean
	@rm -f $(KERNEL_IMG)