#!/usr/bin/sh

# automation script for a KitKat kernel

# the file hierarchy necessary for correct operation of this script MUST be as specified:

#./ (the folder where the folder for the source is located
#./watson-kernel-bcm21553-img (the folder where the KitKat boot.img, update.zip structure and ramdisk will be
#./watson-kernel-bcm21553-src (the folder for the source)
#./AIK-Linux (dsixda's kitchen)

export ARCH=arm
# put here the path for your cross-compiler: I recommend Linaro or older (because those are the ones I worked with)
export CROSS_COMPILE=~/Android/kernel/toolchains/arm-eabi-linaro-4.6.2/bin/arm-eabi-

cp .config.mtd .config

# if your processor has more cores, then -j$(number of cores+1)
make -j3

if [ $? == 0 ]
then

mv arch/arm/boot/zImage ../watson-kernel-bcm21553-img/
cp ../watson-kernel-bcm21553-img/zip/boot.img ../AIK-Linux/Watson-old.img
cd ../AIK-Linux/
sh unpackimg.sh Watson-old.img > /dev/null
rm -rf ramdisk/*
cp -R ../watson-kernel-bcm21553-img/ramdisk/. ramdisk/
cp ../watson-kernel-bcm21553-img/zImage ../AIK-Linux/split_img/Watson-old.img-zImage
cd ../watson-kernel-bcm21553-src/
find ./ -name *.ko -exec cp {} ../watson-kernel-bcm21553-img/zip/system/lib/modules/ \;

cd ../AIK-Linux/
sh repackimg.sh > /dev/null
mv boot.img ../watson-kernel-bcm21553-img/zip/
cd ../watson-kernel-bcm21553-img/zip/
sh automate.sh

fi
