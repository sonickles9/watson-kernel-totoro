#!/usr/bin/sh

# automation script for an ICS kernel

export ARCH=arm
# put here the path for your cross-compiler: I recommend Linaro or older (because those are the ones I worked with)
export CROSS_COMPILE=~/Android/kernel/toolchains/arm-eabi-linaro-4.6.2/bin/arm-eabi-

cp .config.mtd .config

# if your processor has more cores, then -j$(number of cores+1)
make -j3

if [ $? == 0 ]
then

mv arch/arm/boot/zImage ../watson-kernel-bcm21553-img-ics/
cp ../watson-kernel-bcm21553-img-ics/zip/boot.img ../AIK-Linux/ics.img
cd ../AIK-Linux/
sh unpackimg.sh ics.img > /dev/null
rm -rf ramdisk/*
cp -R ../watson-kernel-bcm21553-img-ics/ramdisk/. ramdisk/
cp ../watson-kernel-bcm21553-img-ics/zImage ../AIK-Linux/split_img/ics.img-zImage
cd ../watson-kernel-bcm21553-src/
find ./ -name *.ko -exec cp {} ../watson-kernel-bcm21553-img-ics/zip/system/lib/modules/ \;

cd ../AIK-Linux/
sh repackimg.sh > /dev/null
mv boot.img ../watson-kernel-bcm21553-img-ics/zip/
cd ../watson-kernel-bcm21553-img-ics/zip/
sh automate.sh

fi
