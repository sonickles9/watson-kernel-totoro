#!/usr/bin/sh

rm Watson-gb.zip

mv system/lib/modules/bcmdhd.ko system/lib/modules/dhd.ko
mv system/lib/modules/bcm4330.ko system/lib/modules/dhd.ko

zip -r Watson-gb.zip META-INF system boot.img
tar -cf Watson-gb.tar boot.img

adb push Watson-gb.zip /sdcard/
adb push Watson-gb.zip /cache/
adb push boot.img /sdcard/Watson-gb.img
adb root
adb wait-for-device
adb shell mkdir /cache/recovery
adb shell touch /cache/recovery/command
adb shell "echo boot-recovery | tee /cache/recovery/command"
adb shell "echo "--update-package=/cache/Watson-gb.zip" | tee -a /cache/recovery/command"
adb reboot recovery
