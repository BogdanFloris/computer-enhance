#pragma once

#include <cstdint>
#include <ostream>

enum Op : uint8_t {
    mov,
};

std::ostream& operator<<(std::ostream& os, const Op& op);

enum class OpSource : uint8_t {
    reg,        // from REG field of MOD/REG/RM byte
    rm,         // from R/M field (register or memory based on MOD)
    opcode_reg, // from low 3 bits of opcode itself
    imm,        // immediate follows
    acc,        // fixed AL/AX based on W
    addr,       // direct address follows (2 bytes)
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

    return t;
}

constexpr auto opcode_table = make_opcode_table();
