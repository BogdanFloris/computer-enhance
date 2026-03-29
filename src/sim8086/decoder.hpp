#pragma once
/*
 * Instruction decoding for the 8086 instruction set.
 * Please see ./doc/8086.txt for the reference.
 */

#include <cstdint>
#include <optional>
#include <ostream>
#include <span>
#include <variant>
#include <vector>

enum Op : uint8_t {
    mov,
};

Op decode_op(uint8_t byte);
std::ostream& operator<<(std::ostream& os, const Op& op);

enum Reg : uint8_t {
    al, // 000 W=0
    ax, // 000 W=1
    cl, // 001 W=0
    cx, // 001 W=1
    dl, // 010 W=0
    dx, // 010 W=1
    bl, // 011 W=0
    bx, // 011 W=1
    ah, // 100 W=0
    sp, // 100 W=1
    ch, // 101 W=0
    bp, // 101 W=1
    dh, // 110 W=0
    si, // 110 W=1
    bh, // 111 W=0
    di, // 111 W=1
};

std::ostream& operator<<(std::ostream& os, const Reg& reg);

enum DispType : uint8_t {
    no_disp,
    disp_lo,
    disp_hi,
};

struct Memory {
    std::optional<Reg> base;  // bx, bp, si, di, or none (direct address)
    std::optional<Reg> index; // si, di, or none
    int16_t disp = 0;         // signed displacement
    DispType disp_type;       // type of displacement
};
std::ostream& operator<<(std::ostream& os, const Memory& memory);
inline bool operator==(const Memory& lhs, const Memory& rhs) {
    return lhs.base == rhs.base && lhs.index == rhs.index && lhs.disp == rhs.index &&
           lhs.disp_type == rhs.disp_type;
}

struct Immediate {
    uint16_t value;
    bool wide;
};
std::ostream& operator<<(std::ostream& os, const Immediate& imm);
inline bool operator==(const Immediate& lhs, const Immediate& rhs) {
    return lhs.value == rhs.value && lhs.wide == rhs.wide;
}

using Operand = std::variant<Reg, Memory, Immediate>;
std::ostream& operator<<(std::ostream& os, const Operand& operand);

inline bool operator==(const Operand& op, Reg r) {
    return std::holds_alternative<Reg>(op) && std::get<Reg>(op) == r;
}
inline bool operator==(const Operand& op, const Memory& m) {
    return std::holds_alternative<Memory>(op) && std::get<Memory>(op) == m;
}
inline bool operator==(const Operand& op, const Immediate& i) {
    return std::holds_alternative<Immediate>(op) && std::get<Immediate>(op) == i;
}

Operand decode_reg(uint8_t regByte, uint8_t wByte);
Operand decode_rm(uint8_t rmByte, uint8_t mod, uint8_t wByte);

class Instruction {
  public:
    Instruction() = delete;
    Instruction(Op op, Operand dst, Operand src, bool wide)
        : mOp(op), mDst(dst), mSrc(src), mWide(wide) {}

    static std::vector<Instruction> decode_bytes(const std::vector<uint8_t>& bytes);

    [[nodiscard]] Op op() const { return mOp; }
    [[nodiscard]] Operand dst() const { return mDst; }
    [[nodiscard]] Operand src() const { return mSrc; }

  private:
    Op mOp;
    Operand mDst;
    Operand mSrc;
    bool mWide;

    static Instruction decode(std::span<const uint8_t>& bytes);
};

std::ostream& operator<<(std::ostream& os, const Instruction& inst);
