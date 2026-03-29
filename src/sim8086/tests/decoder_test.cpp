#include "decoder.hpp"

#include <gtest/gtest.h>
#include <vector>

struct Expected {
    Op op;
    Operand dst;
    Operand src;
};

TEST(Decoder, Listing37) {
    std::vector<uint8_t> bytes = {0x89, 0xd9}; // mov cx,bx
    auto instructions = Instruction::decode_bytes(bytes);
    EXPECT_EQ(instructions[0].op(), mov);
    EXPECT_EQ(instructions[0].dst(), cx);
    EXPECT_EQ(instructions[0].src(), bx);
}

TEST(Decoder, Listing38) {
    std::vector<uint8_t> bytes = {
        0x89, 0xD9, 0x88, 0xE5, 0x89, 0xDA, 0x89, 0xDE, 0x89, 0xFB, 0x88,
        0xC8, 0x88, 0xED, 0x89, 0xC3, 0x89, 0xF3, 0x89, 0xFC, 0x89, 0xC5,
    };

    auto instructions = Instruction::decode_bytes(bytes);

    std::array<Expected, 11> expected = {{
        {.op = mov, .dst = cx, .src = bx},
        {.op = mov, .dst = ch, .src = ah},
        {.op = mov, .dst = dx, .src = bx},
        {.op = mov, .dst = si, .src = bx},
        {.op = mov, .dst = bx, .src = di},
        {.op = mov, .dst = al, .src = cl},
        {.op = mov, .dst = ch, .src = ch},
        {.op = mov, .dst = bx, .src = ax},
        {.op = mov, .dst = bx, .src = si},
        {.op = mov, .dst = sp, .src = di},
        {.op = mov, .dst = bp, .src = ax},
    }};

    ASSERT_EQ(instructions.size(), std::size(expected));
    for (size_t i = 0; i < std::size(expected); ++i) {
        EXPECT_EQ(instructions[i].op(), expected.at(i).op) << "instruction " << i;
        EXPECT_EQ(instructions[i].dst(), expected.at(i).dst) << "instruction " << i;
        EXPECT_EQ(instructions[i].src(), expected.at(i).src) << "instruction " << i;
    }
}
