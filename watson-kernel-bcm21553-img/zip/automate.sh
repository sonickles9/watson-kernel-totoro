#!/usr/bin/sh

rm Watson.zip

zip -r Watson.zip META-INF system boot.img
tar -cf Watson.tar boot.img

adb push Watson.zip /sdcard/
adb push Watson.zip /cache/
adb push boot.img /sdcard/Watson.img
adb root
adb wait-for-device
adb shell mkdir /cache/recovery
adb shell touch /cache/recovery/command
adb shell "echo boot-recovery | tee /cache/recovery/command"
adb shell "echo "--update-package=/cache/Watson.zip" | tee -a /cache/recovery/command"
adb reboot recovery
