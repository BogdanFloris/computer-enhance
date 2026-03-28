#pragma once

#include <cstdint>
#include <vector>

enum Op : uint8_t {
    mov,
};

enum Reg : uint8_t {
    ax,
    bx,
};

// Instruction decoding for the 8086 instruction set.
// Please see ./doc/8086.txt for the reference.
struct Instruction {
    Op op;
    Reg src;
    Reg dst;
};

Instruction decode(const std::vector<uint8_t>&);
