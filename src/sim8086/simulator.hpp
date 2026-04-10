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

// constexpr uint16_t most = 0x8000;
constexpr uint16_t zero_mask = 0x40;
constexpr uint16_t sign_mask = 0x80;

struct Simulator {
  public:
    Simulator() = delete;
    Simulator(std::vector<Instruction>&& instructions, bool debug = false)
        : mInstructions(std::move(instructions)), mRegisters{}, mDebug(debug), mFlags(0) {}

    void exec();
    void exec(const Instruction& instr);
    [[nodiscard]] Registers registers() const { return mRegisters; }
    [[nodiscard]] uint16_t flags() const { return mFlags; }

  private:
    uint16_t read_operand(const Operand& op);
    void write_operand(const Operand& op, uint16_t val);
    void set_flags(uint16_t result);
    bool mDebug;
    uint16_t mFlags;
    std::vector<Instruction> mInstructions;
    Registers mRegisters;
};
