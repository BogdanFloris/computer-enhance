#pragma once
/*
 * Instruction decoding for the 8086 instruction set.
 * Please see ./doc/8086.txt for the reference.
 */

#include <cstdint>
#include <ostream>
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

Reg decode_reg(uint8_t regByte, uint8_t wByte);
std::ostream& operator<<(std::ostream& os, const Reg& reg);

struct DstReg {
    Reg reg;
};

struct SrcReg {
    Reg reg;
};

class Instruction {
  public:
    Instruction() = delete;
    Instruction(Op op, DstReg dst, SrcReg src) : mOp(op), mDst(dst.reg), mSrc(src.reg) {}

    static std::vector<Instruction> decode_bytes(const std::vector<uint8_t>& bytes);

    [[nodiscard]] Op op() const { return mOp; }
    [[nodiscard]] Reg dst() const { return mDst; }
    [[nodiscard]] Reg src() const { return mSrc; }

  private:
    Op mOp;
    Reg mDst;
    Reg mSrc;

    static Instruction decode(const std::array<uint8_t, 2>& bytes);
};

std::ostream& operator<<(std::ostream& os, const Instruction& inst);
