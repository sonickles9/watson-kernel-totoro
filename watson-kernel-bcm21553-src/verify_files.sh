#!/usr/bin/sh

if [ ! -e $1 ]
then
exit 1
elif [ -z $1 ]
then
exit 1
fi

for file in $(find ./../*/$1)
do
md5sum $file
done
