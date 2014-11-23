To compile a booting kernel for the NU3001 the following commands are required:


export ARCH=arm

export CROSS_COMPILE=../prebuilts/gcc/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi-

export UTS_RELEASE="3.0.36+"

make amplified_defconfig

make kernel.img

make modules

sudo make modules_install


You may now procede to building android

