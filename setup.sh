!#/bin/bash
export ARCH=arm
export CROSS_COMPILE=../prebuilts/gcc/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi-
export UTS_RELEASE="3.0.36+"
make amplified_defconfig
