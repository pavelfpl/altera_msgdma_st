# Altera MSGDMA linux driver (only for architecture ARM)
# ------------------------------------------------------
# Pavel Fiala 2018 / 2019 / 2020
# ------------------------------------------------------

ARCH := arm

ifeq ($(shell uname -m | sed -e s/arm.*/arm/),arm)
  KERNEL_SRC_DIR  ?= /lib/modules/$(shell uname -r)/build
	CROSS_COMPILE  ?= arm-linux-gnueabihf-
else
  KERNEL_SRC_DIR ?= $(HOME)/build_sockit_arm/socfpga-kernel-dev-3/KERNEL
	# For cross compilation is necessary to export CC variable / gcc for cross compilation ...
	# ----------------------------------------------------------------------------------------
	# Linux Kernel 4.17 ...
	# ---------------------
	# export CC=`pwd`/socfpga-kernel-dev-2/dl/gcc-linaro-7.2.1-2017.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
	# or Linux Kernel 4.20 ...
	# ------------------------
	# export CC=`pwd`/socfpga-kernel-dev-3/dl/gcc-linaro-7.2.1-2017.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
	# Test with ${CC}gcc --version
  CROSS_COMPILE = ${CC}
endif

obj-m := altera_msgdma_st.o

all:
	make -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules
clean:
	make -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean
