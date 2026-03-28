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

struct Instruction {
    Op op;
    Reg src;
    Reg dst;
};

Instruction decode(const std::vector<uint8_t>&);
