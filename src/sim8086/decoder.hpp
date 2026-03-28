#pragma once

#include <cstdint>
#include <vector>

enum Op : uint8_t {
    mov,
};

enum Reg : uint8_t {
    al, // 000 W=0
    ax, // 000 W=1
    cl, // 001 W=0
    cx, // 001 W=1
    dl, // 010 W=0
    dx, // 010 W=1
    bl, // 011 W=0
    bx, // 011 W=1
    ah, // 100 W=0
    sp, // 100 W=1
    ch, // 101 W=0
    bp, // 101 W=1
    dh, // 110 W=0
    si, // 110 W=1
    bh, // 111 W=0
    di, // 111 W=1
};

// Instruction decoding for the 8086 instruction set.
// Please see ./doc/8086.txt for the reference.
struct Instruction {
    Op op;
    Reg src;
    Reg dst;
};

std::vector<Instruction> decode(const std::vector<uint8_t>&);
