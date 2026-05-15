#include "table.hpp"

#include <cstdint>
#include <decoder.hpp>
#include <span>
#include <variant>

// Group 1 opcodes: REG field determines operation
constexpr std::array<Op, 8> group1_ops = {add, add, add, add, add, sub, add, cmp};

namespace {
enum class OperandShape : uint8_t {
    reg_reg,
    reg_imm,
    reg_mem,
    mem_reg,
    mem_imm,
    unsupported,
};

size_t ea_clocks(const Memory& mem) {
    const bool has_base = mem.base.has_value();
    const bool has_index = mem.index.has_value();
    const bool has_disp = mem.disp != 0 || (!has_base && !has_index);

    if (!has_base && !has_index) {
        return 6;
    }

    if (has_base && has_index) {
        const bool is_bx_si = mem.base == bx && mem.index == si;
        const bool is_bp_di = mem.base == bp && mem.index == di;
        return (is_bx_si || is_bp_di ? 7 : 8) + (has_disp ? 4 : 0);
    }

    return 5 + (has_disp ? 4 : 0);
}

OperandShape operand_shape(const Operand& dst, const Operand& src) {
    if (std::holds_alternative<Reg>(dst) && std::holds_alternative<Reg>(src)) {
        return OperandShape::reg_reg;
    }
    if (std::holds_alternative<Reg>(dst) && std::holds_alternative<Immediate>(src)) {
        return OperandShape::reg_imm;
    }
    if (std::holds_alternative<Reg>(dst) && std::holds_alternative<Memory>(src)) {
        return OperandShape::reg_mem;
    }
    if (std::holds_alternative<Memory>(dst) && std::holds_alternative<Reg>(src)) {
        return OperandShape::mem_reg;
    }
    if (std::holds_alternative<Memory>(dst) && std::holds_alternative<Immediate>(src)) {
        return OperandShape::mem_imm;
    }
    return OperandShape::unsupported;
}

size_t operand_ea_clocks(const Operand& dst, const Operand& src) {
    if (const auto* mem = std::get_if<Memory>(&dst)) {
        return ea_clocks(*mem);
    }
    if (const auto* mem = std::get_if<Memory>(&src)) {
        return ea_clocks(*mem);
    }
    return 0;
}

size_t mov_clocks(OperandShape shape, size_t ea) {
    switch (shape) {
    case OperandShape::reg_reg:
        return 2;
    case OperandShape::reg_imm:
        return 4;
    case OperandShape::reg_mem:
        return 8 + ea;
    case OperandShape::mem_reg:
        return 9 + ea;
    case OperandShape::mem_imm:
        return 10 + ea;
    default:
        return 0;
    }
}

size_t alu_clocks(OperandShape shape, size_t ea, size_t mem_reg_base, size_t mem_imm_base) {
    switch (shape) {
    case OperandShape::reg_reg:
        return 3;
    case OperandShape::reg_imm:
        return 4;
    case OperandShape::reg_mem:
        return 9 + ea;
    case OperandShape::mem_reg:
        return mem_reg_base + ea;
    case OperandShape::mem_imm:
        return mem_imm_base + ea;
    default:
        return 0;
    }
}

size_t jump_clocks(ClockKind kind) {
    switch (kind) {
    case ClockKind::conditional_jump:
        return 4;
    case ClockKind::loop_jump:
        return 5;
    default:
        return 0;
    }
}

size_t estimate_clocks(ClockKind kind, const Operand& dst, const Operand& src) {
    const auto shape = operand_shape(dst, src);
    const auto ea = operand_ea_clocks(dst, src);

    switch (kind) {
    case ClockKind::mov:
        return mov_clocks(shape, ea);
    case ClockKind::alu:
        return alu_clocks(shape, ea, 16, 17);
    case ClockKind::cmp:
        return alu_clocks(shape, ea, 9, 10);
    case ClockKind::conditional_jump:
    case ClockKind::loop_jump:
        return jump_clocks(kind);
    case ClockKind::none:
        break;
    }
    return 0;
}
} // namespace

Instruction Instruction::decode(std::span<const uint8_t>& bytes) {
    auto info = opcode_table.at(bytes[0]);
    uint8_t w = ((bytes[0] & info.w_mask) != 0) ? 1 : 0;
    uint8_t imm_w = w; // width for immediate operand

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

    // Handle group 1 opcodes (0x80, 0x81, 0x83)
    Op op = info.op;
    uint8_t opcode = bytes[0];
    if (opcode == 0x80 || opcode == 0x81 || opcode == 0x83) {
        uint8_t reg_field = (bytes[1] >> 3) & 0x7;
        op = group1_ops.at(reg_field);
        // 0x83: sign-extended 8-bit immediate to 16-bit
        if (opcode == 0x83) {
            imm_w = 0; // read 8-bit immediate
        }
    }

    Operand dst = resolve_operand(info.dst, bytes[0], w, reg_operand, rm_operand, bytes, offset);
    Operand src =
        resolve_operand(info.src, bytes[0], imm_w, reg_operand, rm_operand, bytes, offset);
    auto clock_kind = info.clock_kind;
    if (opcode == 0x80 || opcode == 0x81 || opcode == 0x83) {
        clock_kind = (op == cmp) ? ClockKind::cmp : ClockKind::alu;
    }
    const auto clocks = estimate_clocks(clock_kind, dst, src);
    bytes = bytes.subspan(offset);
    return {op, Operand{dst}, Operand{src}, w != 0U, offset, clocks};
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
    case OpSource::addr: {
        int16_t addr = static_cast<int16_t>(bytes[offset]) |           // NOLINT
                       (static_cast<int16_t>(bytes[offset + 1]) << 8); // NOLINT
        offset += 2;
        return Memory{
            .base = std::nullopt, .index = std::nullopt, .disp = addr, .disp_type = disp_hi};
    }
    case OpSource::rel8: {
        auto jump_offset = static_cast<int8_t>(bytes[offset]);
        offset += 1;
        return jump_offset;
    }
    case OpSource::none:
        return std::monostate{};
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
        return Memory{
            .base = std::nullopt,
            .index = std::nullopt,
            .disp = 0,
            .disp_type = disp_hi,
        }; // direct address
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
    std::visit(
        [&os](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, JumpOffset>) {
                int offset = 2 + static_cast<int>(v);
                os << "$" << std::showpos << offset << std::noshowpos;
            } else if constexpr (!std::is_same_v<T, std::monostate>) {
                os << v;
            }
        },
        operand);
    return os;
}

std::ostream& operator<<(std::ostream& os, const Instruction& inst) {
    os << inst.op() << " " << inst.dst();
    if (!std::holds_alternative<std::monostate>(inst.src())) {
        os << ", ";
    }

    // Need byte/word prefix when operating on immediate to memory
    const auto& src = inst.src();
    if (std::holds_alternative<Memory>(inst.dst()) && std::holds_alternative<Immediate>(src)) {
        os << (inst.wide() ? "word " : "byte ");
    }

    os << inst.src();
    return os;
}
