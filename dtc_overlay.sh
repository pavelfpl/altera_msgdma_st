#!/bin/bash
rm -i altera_msgdma_rbf.dtbo
# System install - use recent version (1.4.0 is obsolete  - use DTC1.4.7 !!!) ...
# -------------------------------------------------------------------------------
dtc -O dtb -o altera_msgdma_rbf.dtbo -b 0 -@ altera_msgdma_rbf.dts
# Local install ... 
# -----------------
# /usr/src/linux-headers-4.20.0-socfpga-r0/scripts/dtc/dtc -O dtb -o altera_msgdma_rbf.dtbo -b 0 -@ altera_msgdma_rbf.dts
