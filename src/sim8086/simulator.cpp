#include "decoder.hpp"

#include <cstdint>
#include <simulator.hpp>
#include <variant>

void Simulator::exec() {
    for (auto& instr : mInstructions) {
        exec(instr);
    }
}

uint16_t Simulator::read_operand(const Operand& op) {
    if (const auto* r = std::get_if<Reg>(&op)) {
        return mRegisters.at(reg_index(*r));
    }
    if (const auto* imm = std::get_if<Immediate>(&op)) {
        return imm->value;
    }
    throw std::runtime_error("unexpected operand type");
}

void Simulator::write_operand(const Operand& op, uint16_t val) {
    if (const auto* r = std::get_if<Reg>(&op)) {
        mRegisters.at(reg_index(*r)) = val;
        return;
    }
    throw std::runtime_error("unexpected operand type");
}

void Simulator::exec(const Instruction& instr) {
    switch (instr.op()) {
    case mov: {
        write_operand(instr.dst(), read_operand(instr.src()));
        break;
    }
    case add:
    case sub:
    case cmp:
    case jnz:
    case je:
    case jl:
    case jle:
    case jb:
    case jbe:
    case jp:
    case jo:
    case js:
    case jne:
    case jnl:
    case jg:
    case jnb:
    case ja:
    case jnp:
    case jno:
    case jns:
    case loop:
    case loopz:
    case loopnz:
    case jcxz:
        break;
    }
}
