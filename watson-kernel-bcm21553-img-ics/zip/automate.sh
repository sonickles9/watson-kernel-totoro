#!/usr/bin/sh

rm Watson.zip

zip -r Watson-ics.zip META-INF system boot.img

adb push Watson-ics.zip /sdcard/
adb root 
sleep 5
adb shell mkdir /cache/recovery
adb shell touch /cache/recovery/command 
adb shell exec sh /sdcard/automate-ics.sh
