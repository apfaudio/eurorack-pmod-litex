#!/bin/bash
set -e

if [[ -z "$BOARD" ]]; then
    echo "Must provide BOARD (e.g. $ BOARD='colorlight_i5' ./build.sh)" 1>&2
    echo "Existing builds are" $(ls `pwd`/../build)
    exit 1
fi

export BUILD_DIR=`pwd`/../build/${BOARD}
echo "Using BUILD_DIR=$BUILD_DIR"

# DEFMT logging level is fixed at firmware compile time.
# You can use this to switch logging off completely.
export DEFMT_LOG=debug

FW_ROOT=`pwd`

OBJCOPY=${OBJCOPY:=riscv-none-elf-objcopy}

# Generate the Rust CSR bindings spat out by the LiteX SOC generated.
cd $FW_ROOT/deps/generated-litex-pac/src
svd2rust -i $BUILD_DIR/csr.svd --target riscv

# Build the firmware .elf file
cd $FW_ROOT/polyvec
cargo clippy --target=riscv32imac-unknown-none-elf
cargo build --target=riscv32imac-unknown-none-elf --release

# Copy it into a binary that litex_term can upload.
${OBJCOPY} target/riscv32imac-unknown-none-elf/release/polyvec_main -O binary $BUILD_DIR/rust-fw.bin

#cd $FW_ROOT/polyboot
#cargo build --target=riscv32imac-unknown-none-elf -vv --release
#${OBJCOPY} target/riscv32imac-unknown-none-elf/release/polyboot -O binary $BUILD_DIR/bootloader.bin
