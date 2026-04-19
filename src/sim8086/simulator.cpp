#include "decoder.hpp"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <simulator.hpp>
#include <span>
#include <variant>

namespace {
uint16_t compute(Op op, uint16_t dst_val, uint16_t src_val) {
    switch (op) {
    case add:
        return dst_val + src_val;
    case sub:
    case cmp:
        return dst_val - src_val;
    default:
        throw std::runtime_error("unexpected op");
    }
}
void print_flags(uint16_t f) {
    if (f != 0) {
        if ((f & zero_mask) != 0) {
            std::cout << "Z";
        }
        if ((f & sign_mask) != 0) {
            std::cout << "S";
        }
    }
}

} // namespace

void Simulator::exec(std::span<const uint8_t>& bytes) {
    while (mIp < bytes.size()) {
        auto subspan = bytes.subspan(mIp);
        auto instr = Instruction::decode(subspan);
        exec(instr);
        mIp += instr.offset();
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
        std::cout << "ip" << ": 0x" << std::hex << std::setfill('0') << std::setw(4) << mIp << " ("
                  << std::dec << mIp << ")\n";
        std::cout << "flags: ";
        print_flags(mFlags);
        std::cout << "\n";
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
                      << mRegisters.at(reg_index(*r)) << " -> 0x" << std::setw(4) << val << " ";
        }
        mRegisters.at(reg_index(*r)) = val;
        return;
    }
    throw std::runtime_error("unexpected operand type");
}

void Simulator::set_flags(uint16_t result) {
    uint16_t old_flags = mFlags;
    mFlags = 0;
    if (result == 0) {
        mFlags |= zero_mask;
    }
    if ((result & 0x8000) != 0) {
        mFlags |= sign_mask;
    }
    if (mDebug) {
        std::cout << "flags:";
        print_flags(old_flags);
        std::cout << "->";
        print_flags(mFlags);
    }
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
    case cmp: {
        auto dst_val = read_operand(instr.dst());
        auto src_val = read_operand(instr.src());
        auto result = compute(instr.op(), dst_val, src_val);
        if (instr.op() != cmp) {
            write_operand(instr.dst(), result);
        }
        set_flags(result);
        break;
    }
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
