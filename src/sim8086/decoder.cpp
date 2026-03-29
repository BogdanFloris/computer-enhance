#include <cstdint>
#include <decoder.hpp>
#include <span>
#include <stdexcept>
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
    auto info = opcode_table.at(bytes[0]);
    uint8_t w = ((bytes[0] & info.w_mask) != 0) ? 1 : 0;

    Operand reg_operand;
    Operand rm_operand;
    size_t offset = 1;

    uint8_t d = (bytes[0] >> 1) & 0x01;

    if (info.has_modrm) {
        reg_operand = decode_reg((bytes[1] >> 3) & 0x7, w);
        rm_operand = decode_rm(bytes[1] & 0x7, (bytes[1] >> 6) & 0x3, w);
        offset = 2;
        if (auto* v = std::get_if<Memory>(&rm_operand)) {
            switch (v->disp_type) {
            case no_disp:
                break;
            case disp_lo:
                v->disp = static_cast<int16_t>(static_cast<int8_t>(bytes[2])); // NOLINT
                offset = 3;
                break;
            case disp_hi:
                v->disp =
                    static_cast<int16_t>(static_cast<uint16_t>(bytes[3]) << 8) | bytes[2]; // NOLINT
                offset = 4;
                break;
            }
        }
    }

    Operand dst = resolve_operand(info.dst, bytes[0], w, reg_operand, rm_operand, bytes, offset);
    Operand src = resolve_operand(info.src, bytes[0], w, reg_operand, rm_operand, bytes, offset);
    bytes = bytes.subspan(offset);
    return {info.op, Operand{dst}, Operand{src}, w != 0U};
}

Operand resolve_operand(OpSource source, uint8_t opcode, uint8_t w,
                        const Operand& reg_operand, // pre-decoded from REG field (if has_modrm)
                        const Operand& rm_operand,  // pre-decoded from R/M field (if has_modrm)
                        std::span<const uint8_t>& bytes, size_t& offset) {
    switch (source) {
    case OpSource::reg:
        return reg_operand;
    case OpSource::rm:
        return rm_operand;
    case OpSource::opcode_reg:
        return decode_reg(opcode & 0x07, w);
    case OpSource::imm: { /* read immediate, advance offset */
        Immediate imm{};
        imm.wide = (w != 0);
        if (imm.wide) {
            imm.value = static_cast<uint16_t>(bytes[offset]) |
                        (static_cast<uint16_t>(bytes[offset + 1]) << 8);
            offset += 2;
        } else {
            imm.value = bytes[offset];
            offset += 1;
        }
        return imm;
    }
    case OpSource::acc:
        return decode_reg(0, w); // AL or AX
    case OpSource::addr:
        int16_t addr = static_cast<int16_t>(bytes[offset]) |           // NOLINT
                       (static_cast<int16_t>(bytes[offset + 1]) << 8); // NOLINT
        offset += 2;
        return Memory{
            .base = std::nullopt, .index = std::nullopt, .disp = addr, .disp_type = disp_hi};
    }
}

Operand decode_reg(uint8_t regByte, uint8_t wByte) {
    return Operand{static_cast<Reg>((regByte << 1) | wByte)};
}

Operand decode_rm(uint8_t rmByte, uint8_t mod, uint8_t wByte) {
    if (mod == 0b11) {
        return decode_reg(rmByte, wByte);
    }

    if (mod == 0b00 && rmByte == 0b110) {
        return Memory{.base = std::nullopt,
                      .index = std::nullopt,
                      .disp = 0,
                      .disp_type = disp_hi}; // direct address
    }

    constexpr std::array<DispType, 3> disp_types = {no_disp, disp_lo, disp_hi};

    const auto& ea = ea_table.at(rmByte);
    auto mem = Memory{
        .base = ea.base,
        .index = ea.index,
    };
    mem.disp_type = disp_types.at(mod);
    return Operand{mem};
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
    if (imm.wide) {
        os << static_cast<int16_t>(imm.value);
    } else {
        // Cast to int8_t first to sign-extend, then to int for printing
        os << static_cast<int>(static_cast<int8_t>(imm.value));
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const Operand& operand) {
    std::visit([&os](const auto& v) { os << v; }, operand);
    return os;
}

std::ostream& operator<<(std::ostream& os, const Instruction& inst) {
    os << inst.op() << " " << inst.dst() << ", ";

    // Need byte/word prefix when moving immediate to memory
    const auto& src = inst.src();
    if (std::holds_alternative<Memory>(inst.dst()) &&
        std::holds_alternative<Immediate>(src)) {
        const auto& imm = std::get<Immediate>(src);
        os << (imm.wide ? "word " : "byte ");
    }

    os << inst.src();
    return os;
}
