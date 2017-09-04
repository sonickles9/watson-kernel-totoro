#!/usr/bin/sh

# this script will search in the source tree for matching $1 words inside the source files
# only C, Header and Assembly files are supported (for now)

if [ -z "$1" ];
then
echo "Missing argument"
exit 1
fi

echo "Searching in C source files..."
find ./ -name *.c -exec grep -inH "$1" {} \;
echo "Searching in C header files..."
find ./ -name *.h -exec grep -inH "$1" {} \;
echo "Searching in ASM source files..."
find ./ -name *.S -exec grep -inH "$1" {} \;
