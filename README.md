This is the Kernel Source for the Newsmy NU3001 CarPadIIp. Kernel version 3.0.36+

To compile a booting kernel for the NU3001 the following commands are required:

__________These Commands can be executed by running setup.sh__________________

export ARCH=arm

export CROSS_COMPILE=../prebuilts/gcc/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi-

export UTS_RELEASE="3.0.36+"

make amplified_defconfig

_______________________________________________________________________________

make kernel.img

make modules

sudo make modules_install


You may now procede to building android

