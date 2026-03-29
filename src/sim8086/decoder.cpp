#include <cstdint>
#include <decoder.hpp>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <variant>

std::vector<Instruction> Instruction::decode_bytes(const std::vector<uint8_t>& bytes) {
    std::vector<Instruction> instructions{};
    instructions.reserve(bytes.size() / 2);
    std::span<const uint8_t> remaining{bytes};
    while (!remaining.empty()) {
        if (remaining.size() < 2) {
            throw std::runtime_error("truncated instruction stream");
        }
        instructions.push_back(Instruction::decode(remaining));
    }

    return instructions;
}

Instruction Instruction::decode(std::span<const uint8_t>& bytes) {
    auto op = decode_op(bytes[0]);
    uint8_t w = bytes[0] & 0x01;
    uint8_t d = (bytes[0] >> 1) & 0x01;
    uint8_t mod = (bytes[1] >> 6) & 0x3;
    auto reg = decode_reg((bytes[1] >> 3) & 0x7, w);
    auto rm = decode_reg(bytes[1] & 0x7, w);
    auto src = (d != 0U) ? rm : reg;
    auto dst = (d != 0U) ? reg : rm;
    bytes = bytes.subspan(2);

    return {op, Operand{dst}, Operand{src}, w != 0U};
}

Op decode_op(uint8_t byte) {
    switch (byte) {
    case 0x88:
    case 0x89:
    case 0x8A:
    case 0x8B:
        return mov;
    default:
        throw std::runtime_error("unknown opcode");
    }
}

Reg decode_reg(uint8_t regByte, uint8_t wByte) {
    return static_cast<Reg>((regByte << 1) | wByte);
}

std::ostream& operator<<(std::ostream& os, const Op& op) {
    static constexpr std::array ops = {"mov"};
    if (auto i = static_cast<std::size_t>(op); i < ops.size()) {
        return os << ops.at(i);
    }
    return os << "unknown";
}

std::ostream& operator<<(std::ostream& os, const Reg& reg) {
    static constexpr std::array names = {
        "al", "ax", "cl", "cx", "dl", "dx", "bl", "bx",
        "ah", "sp", "ch", "bp", "dh", "si", "bh", "di",
    };
    if (auto i = static_cast<std::size_t>(reg); i < names.size()) {
        return os << names.at(i);
    }
    return os << "unknown";
}

std::ostream& operator<<(std::ostream& os, const Memory& memory) {
    os << "[";
    bool needs_sep = false;

    if (memory.base) {
        os << memory.base.value();
        needs_sep = true;
    }
    if (memory.index) {
        if (needs_sep) {
            os << " + ";
        }
        os << memory.index.value();
        needs_sep = true;
    }
    if (memory.disp != 0 || !needs_sep) {
        if (needs_sep) {
            if (memory.disp >= 0) {
                os << " + " << memory.disp;
            } else {
                os << " - " << -memory.disp;
            }
        } else {
            os << memory.disp;
        }
    }

    os << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Immediate& imm) {
    os << imm.value;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Operand& operand) {
    std::visit([&os](const auto& v) { os << v; }, operand);
    return os;
}

std::ostream& operator<<(std::ostream& os, const Instruction& inst) {
    return os << inst.op() << " " << inst.dst() << "," << inst.src();
}
