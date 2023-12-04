# motor-control-iio-modules

This repository contains the Kernel drivers for the Kd240 Motor Control
Application, leveraging the Industrial IO (IIO) framework. The drivers
are built on target using DKMS (Dynamic Kernel Module Support) framework.
 They are packaged for debian installation as shown in the next section

# Steps to build and install dkms drivers on target

```
  $ sudo apt install dkms
  $ sudo rm -r /usr/src/motor-control-iio-modules-0.1
  $ sudo mkdir -p /usr/src/motor-control-iio-modules-0.1
  $ git clone  https://github.com/Xilinx/motor-control-iio-modules
  $ sudo cp motor-control-iio-modules/src/* /usr/src/motor-control-iio-modules-0.1/
  $ sudo cp motor-control-iio-modules/debian/motor-control-iio-modules.dkms
/usr/src/motor-control-iio-modules-0.1/dkms.conf
  $ sudo dkms add -m motor-control-iio-modules -v 0.1
  $ sudo dkms build -m motor-control-iio-modules -v 0.1
  $ sudo dkms install -m motor-control-iio-modules -v 0.1
```
* `xmutil loadapp kd240-motor-ctrl-qei` will load the dkms installed drivers on target

# Verify module installation

```
$ modinfo hls-pwm-gen
filename:       /lib/modules/5.15.0-9002-xilinx-zynqmp/updates/dkms/hls-pwm-gen.ko
author:         Vivekananda Dayananda <vivekananda.dayananda@amd.com>
license:        GPL
srcversion:     623691B9B83CA4AB91F8695
alias:          of:N*T*Cxlnx,hls-pwm-gen-1.0C*
alias:          of:N*T*Cxlnx,hls-pwm-gen-1.0
depends:
name:           hls_pwm_gen
vermagic:       5.15.0-9002-xilinx-zynqmp SMP mod_unload modversions aarch64
```

# License

(C) Copyright 2022 - 2023 Advanced Micro Devices, Inc.\
SPDX-License-Identifier: GPL-2.0
