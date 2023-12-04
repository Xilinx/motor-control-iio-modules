# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2023 Advanced Micro Devices, Inc.

obj-m += xilinx-adc-hub.o
obj-m += hls-foc-periodic.o
obj-m += hls-qei-axi.o
obj-m += hls-pwm-gen.o
obj-m += hls-svpwm-duty.o

SRC := $(shell pwd)

EXTRA_CFLAGS := -I$(KERNEL_SRC)/drivers/media/platform/
EXTRA_CFLAGS += -I$(KERNEL_SRC)/include/linux
EXTRA_CFLAGS += -I$(KERNEL_SRC)/arch/arm64/include/
EXTRA_CFLAGS += -I$(KERNEL_SRC)/arch/arm64/include/generated/
EXTRA_CFLAGS += -I$(KERNEL_SRC)/arch/arm64/include/generated/uapi/
EXTRA_CFLAGS += -I$(KERNEL_SRC)/arch/arm64/include/uapi/

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -f */modules.order */modules.builtin
	rm -rf .tmp_versions Modules.symvers
