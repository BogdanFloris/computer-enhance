#pragma once

#include <cstdint>
#include <ostream>

enum Op : uint8_t {
    mov,
    add,
    sub,
    cmp,
    jnz,
    je,
    jl,
    jle,
    jb,
    jbe,
    jp,
    jo,
    js,
    jne,
    jnl,
    jg,
    jnb,
    ja,
    jnp,
    jno,
    jns,
    loop,
    loopz,
    loopnz,
    jcxz,
};

std::ostream& operator<<(std::ostream& os, const Op& op);

enum class OpSource : uint8_t {
    reg,        // from REG field of MOD/REG/RM byte
    rm,         // from R/M field (register or memory based on MOD)
    opcode_reg, // from low 3 bits of opcode itself
    imm,        // immediate follows
    acc,        // fixed AL/AX based on W
    addr,       // direct address follows (2 bytes)
    rel8,       // relative offset
    none,       // no operand
};

struct OpcodeInfo {
    Op op;
    OpSource dst;
    OpSource src;
    bool has_modrm; // is there a MOD/REG/RM byte?
    uint8_t w_mask; // which bit is W? (0x01 or 0x08)
};

// d = 0: The REG field in the instruction specifies the source operand,
//        and the R/M field specifies the destination.
//        So the operation goes from REG to R/M.
// d = 1: The REG field specifies the destination, and the R/M field
//        specifies the source.
//        So the operation goes from R/M to REG.
constexpr std::array<OpcodeInfo, 256> make_opcode_table() {
    std::array<OpcodeInfo, 256> t{};

    // 0x88-0x8B: mov r/m, reg (or reg, r/m based on D bit)
    t[0x88] = {
        .op = mov, .dst = OpSource::rm, .src = OpSource::reg, .has_modrm = true, .w_mask = 0x01};
    t[0x89] = {
        .op = mov, .dst = OpSource::rm, .src = OpSource::reg, .has_modrm = true, .w_mask = 0x01};
    t[0x8A] = {
        .op = mov, .dst = OpSource::reg, .src = OpSource::rm, .has_modrm = true, .w_mask = 0x01};
    t[0x8B] = {
        .op = mov, .dst = OpSource::reg, .src = OpSource::rm, .has_modrm = true, .w_mask = 0x01};

    // 0xB0-0xBF: mov reg, imm (reg in opcode)
    for (uint8_t i = 0xB0; i <= 0xBF; ++i) {
        t.at(i) = {.op = mov,
                   .dst = OpSource::opcode_reg,
                   .src = OpSource::imm,
                   .has_modrm = false,
                   .w_mask = 0x08};
    }

    // Accumulator forms (0xA0-0xA3)
    t[0xA0] = {.op = mov,
               .dst = OpSource::acc,
               .src = OpSource::addr,
               .has_modrm = false,
               .w_mask = 0x01}; // mov al, [addr]
    t[0xA1] = {.op = mov,
               .dst = OpSource::acc,
               .src = OpSource::addr,
               .has_modrm = false,
               .w_mask = 0x01}; // mov ax, [addr]
    t[0xA2] = {.op = mov,
               .dst = OpSource::addr,
               .src = OpSource::acc,
               .has_modrm = false,
               .w_mask = 0x01}; // mov [addr], al
    t[0xA3] = {.op = mov,
               .dst = OpSource::addr,
               .src = OpSource::acc,
               .has_modrm = false,
               .w_mask = 0x01}; // mov [addr], ax

    // Immediate to r/m (0xC6, 0xC7)
    for (uint8_t i = 0xC6; i <= 0xC7; ++i) {
        t.at(i) = {.op = mov,
                   .dst = OpSource::rm,
                   .src = OpSource::imm,
                   .has_modrm = true,
                   .w_mask = 0x01};
    }

    // ADD 0x00-0x05
    t[0x00] = {
        .op = add, .dst = OpSource::rm, .src = OpSource::reg, .has_modrm = true, .w_mask = 0x01};
    t[0x01] = {
        .op = add, .dst = OpSource::rm, .src = OpSource::reg, .has_modrm = true, .w_mask = 0x01};
    t[0x02] = {
        .op = add, .dst = OpSource::reg, .src = OpSource::rm, .has_modrm = true, .w_mask = 0x01};
    t[0x03] = {
        .op = add, .dst = OpSource::reg, .src = OpSource::rm, .has_modrm = true, .w_mask = 0x01};
    t[0x04] = {
        .op = add, .dst = OpSource::acc, .src = OpSource::imm, .has_modrm = false, .w_mask = 0x01};
    t[0x05] = {
        .op = add, .dst = OpSource::acc, .src = OpSource::imm, .has_modrm = false, .w_mask = 0x01};

    // SUB 0x28-0x2D
    t[0x28] = {
        .op = sub, .dst = OpSource::rm, .src = OpSource::reg, .has_modrm = true, .w_mask = 0x01};
    t[0x29] = {
        .op = sub, .dst = OpSource::rm, .src = OpSource::reg, .has_modrm = true, .w_mask = 0x01};
    t[0x2A] = {
        .op = sub, .dst = OpSource::reg, .src = OpSource::rm, .has_modrm = true, .w_mask = 0x01};
    t[0x2B] = {
        .op = sub, .dst = OpSource::reg, .src = OpSource::rm, .has_modrm = true, .w_mask = 0x01};
    t[0x2C] = {
        .op = sub, .dst = OpSource::acc, .src = OpSource::imm, .has_modrm = false, .w_mask = 0x01};
    t[0x2D] = {
        .op = sub, .dst = OpSource::acc, .src = OpSource::imm, .has_modrm = false, .w_mask = 0x01};

    // CMP 0x38-0x3D
    t[0x38] = {
        .op = cmp, .dst = OpSource::rm, .src = OpSource::reg, .has_modrm = true, .w_mask = 0x01};
    t[0x39] = {
        .op = cmp, .dst = OpSource::rm, .src = OpSource::reg, .has_modrm = true, .w_mask = 0x01};
    t[0x3A] = {
        .op = cmp, .dst = OpSource::reg, .src = OpSource::rm, .has_modrm = true, .w_mask = 0x01};
    t[0x3B] = {
        .op = cmp, .dst = OpSource::reg, .src = OpSource::rm, .has_modrm = true, .w_mask = 0x01};
    t[0x3C] = {
        .op = cmp, .dst = OpSource::acc, .src = OpSource::imm, .has_modrm = false, .w_mask = 0x01};
    t[0x3D] = {
        .op = cmp, .dst = OpSource::acc, .src = OpSource::imm, .has_modrm = false, .w_mask = 0x01};

    // Group 1: 0x80/0x81/0x83 - op determined by REG field (handled in decoder)
    // REG=0: ADD, REG=5: SUB, REG=7: CMP
    t[0x80] = {
        .op = add, .dst = OpSource::rm, .src = OpSource::imm, .has_modrm = true, .w_mask = 0x01};
    t[0x81] = {
        .op = add, .dst = OpSource::rm, .src = OpSource::imm, .has_modrm = true, .w_mask = 0x01};
    t[0x83] = {
        .op = add, .dst = OpSource::rm, .src = OpSource::imm, .has_modrm = true, .w_mask = 0x01};

    // Conditional jumps 0x70-0x7F
    t[0x70] = {
        .op = jo, .dst = OpSource::rel8, .src = OpSource::none, .has_modrm = false, .w_mask = 0x00};
    t[0x71] = {.op = jno,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};
    t[0x72] = {
        .op = jb, .dst = OpSource::rel8, .src = OpSource::none, .has_modrm = false, .w_mask = 0x00};
    t[0x73] = {.op = jnb,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};
    t[0x74] = {
        .op = je, .dst = OpSource::rel8, .src = OpSource::none, .has_modrm = false, .w_mask = 0x00};
    t[0x75] = {.op = jnz,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};
    t[0x76] = {.op = jbe,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};
    t[0x77] = {
        .op = ja, .dst = OpSource::rel8, .src = OpSource::none, .has_modrm = false, .w_mask = 0x00};
    t[0x78] = {
        .op = js, .dst = OpSource::rel8, .src = OpSource::none, .has_modrm = false, .w_mask = 0x00};
    t[0x79] = {.op = jns,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};
    t[0x7A] = {
        .op = jp, .dst = OpSource::rel8, .src = OpSource::none, .has_modrm = false, .w_mask = 0x00};
    t[0x7B] = {.op = jnp,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};
    t[0x7C] = {
        .op = jl, .dst = OpSource::rel8, .src = OpSource::none, .has_modrm = false, .w_mask = 0x00};
    t[0x7D] = {.op = jnl,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};
    t[0x7E] = {.op = jle,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};
    t[0x7F] = {
        .op = jg, .dst = OpSource::rel8, .src = OpSource::none, .has_modrm = false, .w_mask = 0x00};

    // Loop instructions
    t[0xE0] = {.op = loopnz,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};
    t[0xE1] = {.op = loopz,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};
    t[0xE2] = {.op = loop,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};
    t[0xE3] = {.op = jcxz,
               .dst = OpSource::rel8,
               .src = OpSource::none,
               .has_modrm = false,
               .w_mask = 0x00};

    return t;
}

constexpr auto opcode_table = make_opcode_table();
