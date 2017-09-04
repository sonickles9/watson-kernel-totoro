#!/usr/bin/sh

# automation script for a MTD-compatible Touchwiz kernel

export ARCH=arm
# put here the path for your cross-compiler: I recommend Linaro or older (because those are the ones I worked with)
export CROSS_COMPILE=~/Android/kernel/toolchains/arm-eabi-linaro-4.6.2/bin/arm-eabi-

cp .config.sec .config

# if your processor has more cores, then -j$(number of cores+1)
# This kernel (probably) works without GE, but I kept it in for safety
make CONFIG_GE=m -j3

if [ $? == 0 ]
then

mv arch/arm/boot/zImage ../watson-kernel-bcm21553-img-gb/
cp ../watson-kernel-bcm21553-img-gb/zip/boot.img ../AIK-Linux/gb.img
cd ../AIK-Linux/
sh unpackimg.sh gb.img > /dev/null
rm -rf ramdisk/*
cp -R ../watson-kernel-bcm21553-img-gb/ramdisk/. ramdisk/
cp ../watson-kernel-bcm21553-img-gb/zImage ../AIK-Linux/split_img/gb.img-zImage
cd ../watson-kernel-bcm21553-src/
find ./ -name *.ko -exec cp {} ../watson-kernel-bcm21553-img-gb/zip/system/lib/modules/ \;

cd ../AIK-Linux/
sh repackimg.sh > /dev/null
mv boot.img ../watson-kernel-bcm21553-img-gb/zip/
cd ../watson-kernel-bcm21553-img-gb/zip/
sh automate.sh

fi
