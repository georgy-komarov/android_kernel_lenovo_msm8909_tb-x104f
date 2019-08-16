#!/bin/bash

workdir=`pwd`
export PATH=$PATH:$workdir/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.9/bin
make ARCH=arm CROSS_COMPILE=arm-linux-androideabi- O=out hq_msm8909-perf_defconfig
make ARCH=arm CROSS_COMPILE=arm-linux-androideabi- O=out -j32


