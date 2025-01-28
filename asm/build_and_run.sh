#!/bin/bash
ASM_FILENAME=$1
BASE_NAME=$(basename "$ASM_FILENAME" .asm)

nasm -f elf64 $ASM_FILENAME -o "$BASE_NAME.o"
x86_64-elf-ld "$BASE_NAME.o" -o "$BASE_NAME"
docker build -t x86-runner --build-arg EXEC_NAME="$BASE_NAME" .
docker run --rm x86-runner

rm "$BASE_NAME.o" "$BASE_NAME"
