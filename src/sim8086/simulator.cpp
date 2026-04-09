#include "decoder.hpp"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <simulator.hpp>
#include <variant>

void Simulator::exec() {
    for (auto& instr : mInstructions) {
        exec(instr);
    }

    if (mDebug) {
        static constexpr std::array names = {
            "ax", "cx", "dx", "bx", "sp", "bp", "si", "di",
        };
        std::cout << "\nFinal registers:\n";
        for (int i = 0; i < mRegisters.size(); ++i) {
            std::cout << names.at(i) << ": 0x" << std::hex << std::setfill('0') << std::setw(4)
                      << mRegisters.at(i) << " (" << std::dec << mRegisters.at(i) << ")\n";
        }
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
        if (mDebug) {
            std::cout << *r << ": 0x" << std::hex << std::setfill('0') << std::setw(4)
                      << mRegisters.at(reg_index(*r)) << " -> 0x" << std::setw(4) << val;
        }
        mRegisters.at(reg_index(*r)) = val;
        return;
    }
    throw std::runtime_error("unexpected operand type");
}

void Simulator::exec(const Instruction& instr) {
    if (mDebug) {
        std::cout << instr << " ; ";
    }
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

    if (mDebug) {
        std::cout << "\n";
    }
}
