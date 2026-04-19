#pragma once
/*
 * Instruction simulation for the 8086 instruction set.
 */

#include "decoder.hpp"

#include <cstdint>
#include <span>

inline size_t reg_index(Reg r) {
    return r / 2;
}
using Registers = std::array<uint16_t, 8>;

constexpr uint16_t zero_mask = 0x40;
constexpr uint16_t sign_mask = 0x80;

struct Simulator {
  public:
    Simulator(bool debug = false) : mDebug{debug} {}

    void exec(std::span<const uint8_t>& bytes);
    void exec(const Instruction& instr);
    [[nodiscard]] Registers registers() const { return mRegisters; }
    [[nodiscard]] uint16_t flags() const { return mFlags; }
    [[nodiscard]] uint16_t ip() const { return mIp; }

  private:
    uint16_t read_operand(const Operand& op, bool wide);
    void write_operand(const Operand& op, uint16_t val, bool wide);
    void set_flags(uint16_t result);
    void jump_if(const Instruction& instr, bool condition);
    size_t mem_addr(const Memory& mem);
    bool mDebug;
    uint16_t mFlags{};
    uint16_t mIp{};
    Registers mRegisters{};
    std::array<uint8_t, 1024L * 1024L> mMemory{};
};
