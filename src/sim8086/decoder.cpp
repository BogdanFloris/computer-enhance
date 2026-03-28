#include <decoder.hpp>
#include <stdexcept>

std::vector<Instruction> Instruction::decode(const std::vector<uint8_t>& bytes) {
    return {Instruction(mov, DstReg{cx}, SrcReg{bx})};
}

Op decode_op(uint8_t byte) {
    switch (byte) {
    case 0x88:
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

std::ostream& operator<<(std::ostream& os, const Instruction& inst) {
    return os << inst.op() << " " << inst.dst() << "," << inst.src();
}
