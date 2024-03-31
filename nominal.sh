#!/bin/bash
set -e

# SVD only
echo "Generate SVD"
./sim.py --with-sdram --cpu-type vexriscv_smp --cpu-variant standard --csr-svd build/sim/csr.svd --no-compile-gateware --timer-uptime
# FW only
echo "Build FW"
cd firmware
OBJCOPY=riscv64-linux-gnu-objcopy BOARD=sim ./build.sh
cd ..
# Boot firmware after BIOS
echo "Simulate GW+FW"
./sim.py --with-sdram --sdram-init build/sim/rust-fw.bin --cpu-type vexriscv_smp --cpu-variant standard --timer-uptime --gtkwave-savefile --trace --trace-fst
