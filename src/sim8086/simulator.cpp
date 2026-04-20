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

std::ostream& hex16(std::ostream& os, uint16_t val) {
    return os << "0x" << std::hex << std::setfill('0') << std::setw(4) << val << std::dec;
}

void print_flags(std::ostream& os, uint16_t f) {
    if ((f & zero_mask) != 0) {
        os << "Z";
    }
    if ((f & sign_mask) != 0) {
        os << "S";
    }
}

} // namespace

void Simulator::exec(std::span<const uint8_t>& bytes) {
    while (mIp < bytes.size()) {
        auto subspan = bytes.subspan(mIp);
        auto instr = Instruction::decode(subspan);
        exec(instr);
        if (mDebug) {
            std::cout << "ip: ";
            hex16(std::cout, mIp) << " -> ";
            hex16(std::cout, mIp + instr.offset()) << "\n";
        }

        mIp += instr.offset();
    }

    if (mDebug) {
        static constexpr std::array names = {"ax", "cx", "dx", "bx", "sp", "bp", "si", "di"};
        std::cout << "\nFinal registers:\n";
        for (int i = 0; i < mRegisters.size(); ++i) {
            std::cout << names.at(i) << ": ";
            hex16(std::cout, mRegisters.at(i)) << " (" << mRegisters.at(i) << ")\n";
        }
        std::cout << "ip: ";
        hex16(std::cout, mIp) << " (" << mIp << ")\n";
        std::cout << "flags: ";
        print_flags(std::cout, mFlags);
        std::cout << "\n";
    }
}

size_t Simulator::mem_addr(const Memory& mem) {
    size_t addr = 0;
    if (mem.base) {
        auto reg = mem.base.value();
        addr += mRegisters.at(reg_index(reg));
    }
    if (mem.index) {
        auto reg = mem.index.value();
        addr += mRegisters.at(reg_index(reg));
    }
    addr += mem.disp;
    return addr;
}

uint16_t Simulator::read_operand(const Operand& op, bool wide) {
    if (const auto* r = std::get_if<Reg>(&op)) {
        return mRegisters.at(reg_index(*r));
    }
    if (const auto* imm = std::get_if<Immediate>(&op)) {
        return imm->value;
    }
    if (const auto* m = std::get_if<Memory>(&op)) {
        auto idx = mem_addr(*m);
        if (wide) {
            return (mMemory.at(idx + 1) << 8) | mMemory.at(idx);
        }
        return mMemory.at(idx);
    }
    throw std::runtime_error("unexpected operand type");
}

void Simulator::write_operand(const Operand& op, uint16_t val, bool wide) {
    if (const auto* r = std::get_if<Reg>(&op)) {
        if (mDebug) {
            std::cout << *r << ": ";
            hex16(std::cout, mRegisters.at(reg_index(*r))) << " -> ";
            hex16(std::cout, val) << " ";
        }
        mRegisters.at(reg_index(*r)) = val;
        return;
    }
    if (const auto* m = std::get_if<Memory>(&op)) {
        auto idx = mem_addr(*m);
        if (wide) {
            mMemory.at(idx + 1) = val >> 8;
        }
        mMemory.at(idx) = val;
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
        print_flags(std::cout, old_flags);
        std::cout << "->";
        print_flags(std::cout, mFlags);
        std::cout << " ";
    }
}

void Simulator::jump_if(const Instruction& instr, bool condition) {
    if (condition) {
        if (const auto* jo = std::get_if<JumpOffset>(&instr.dst())) {
            mIp += *jo;
        }
    }
}

void Simulator::exec(const Instruction& instr) {
    if (mDebug) {
        std::cout << instr << " ; ";
    }
    switch (instr.op()) {
    case mov: {
        write_operand(instr.dst(), read_operand(instr.src(), instr.wide()), instr.wide());
        break;
    }
    case add:
    case sub:
    case cmp: {
        auto dst_val = read_operand(instr.dst(), instr.wide());
        auto src_val = read_operand(instr.src(), instr.wide());
        auto result = compute(instr.op(), dst_val, src_val);
        if (instr.op() != cmp) {
            write_operand(instr.dst(), result, instr.wide());
        }
        set_flags(result);
        break;
    }
    case je:
        jump_if(instr, (mFlags & zero_mask) != 0);
        break;
    case jne:
    case jnz:
        jump_if(instr, (mFlags & zero_mask) == 0);
        break;
    case js:
        jump_if(instr, (mFlags & sign_mask) != 0);
        break;
    case jns:
        jump_if(instr, (mFlags & sign_mask) == 0);
        break;
    case jl:
        jump_if(instr, (mFlags & sign_mask) != 0);
        break;
    case jnl:
        jump_if(instr, (mFlags & sign_mask) == 0);
        break;
    case jle:
        jump_if(instr, (mFlags & (zero_mask | sign_mask)) != 0);
        break;
    case jg:
        jump_if(instr, (mFlags & (zero_mask | sign_mask)) == 0);
        break;
    case jb:
    case jbe:
    case jp:
    case jo:
    case jnb:
    case ja:
    case jnp:
    case jno:
    case loop:
    case loopz:
    case loopnz:
    case jcxz:
        break;
    }
}
