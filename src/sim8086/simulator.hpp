#pragma once
/*
 * Instruction simulation for the 8086 instruction set.
 */

#include "decoder.hpp"

#include <cstdint>
#include <vector>

inline size_t reg_index(Reg r) {
    return r / 2;
}
using Registers = std::array<uint16_t, 8>;

struct Simulator {
  public:
    Simulator() = delete;
    Simulator(std::vector<Instruction>&& instructions, bool debug = false)
        : mInstructions(std::move(instructions)), mRegisters{}, mDebug(debug) {}

    void exec();
    void exec(const Instruction& instr);
    [[nodiscard]] Registers registers() const { return mRegisters; }

  private:
    uint16_t read_operand(const Operand& op);
    void write_operand(const Operand& op, uint16_t val);
    bool mDebug;
    std::vector<Instruction> mInstructions;
    Registers mRegisters;
};
