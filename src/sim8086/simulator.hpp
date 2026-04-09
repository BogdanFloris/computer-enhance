#pragma once
/*
 * Instruction simulation for the 8086 instruction set.
 */

#include "decoder.hpp"

#include <vector>

using Registers = std::array<uint16_t, 8>;

struct Simulator {
  public:
    Simulator() = delete;
    Simulator(std::vector<Instruction>&& instructions)
        : mInstructions(std::move(instructions)), mRegisters{} {}

    void exec();
    void exec(const Instruction& instr);
    [[nodiscard]] Registers registers() const { return mRegisters; }

  private:

    std::vector<Instruction> mInstructions;
    Registers mRegisters;
};
