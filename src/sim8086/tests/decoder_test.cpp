#include "decoder.hpp"

#include <gtest/gtest.h>
#include <vector>

struct Expected {
    Op op;
    Reg dst;
    Reg src;
};

TEST(Decoder, Listing37) {
    std::vector<uint8_t> bytes = {0x89, 0xd9}; // mov cx,bx
    auto instructions = decode(bytes);
    EXPECT_EQ(instructions[0].op, Op::mov);
    EXPECT_EQ(instructions[0].src, Reg::bx);
    EXPECT_EQ(instructions[0].dst, Reg::cx);
}

TEST(Decoder, Listing38) {
    std::vector<uint8_t> bytes = {
        0x89, 0xD9, 0x88, 0xE5, 0x89, 0xDA, 0x89, 0xDE, 0x89, 0xFB, 0x88,
        0xC8, 0x88, 0xED, 0x89, 0xC3, 0x89, 0xF3, 0x89, 0xFC, 0x89, 0xC5,
    };

    auto instructions = decode(bytes);

    std::array<Expected, 11> expected = {{
        {Op::mov, Reg::cx, Reg::bx},
        {Op::mov, Reg::ch, Reg::ah},
        {Op::mov, Reg::dx, Reg::bx},
        {Op::mov, Reg::si, Reg::bx},
        {Op::mov, Reg::bx, Reg::di},
        {Op::mov, Reg::al, Reg::cl},
        {Op::mov, Reg::ch, Reg::ch},
        {Op::mov, Reg::bx, Reg::ax},
        {Op::mov, Reg::bx, Reg::si},
        {Op::mov, Reg::sp, Reg::di},
        {Op::mov, Reg::bp, Reg::ax},
    }};

    ASSERT_EQ(instructions.size(), std::size(expected));
    for (size_t i = 0; i < std::size(expected); ++i) {
        EXPECT_EQ(instructions[i].op, expected.at(i).op) << "instruction " << i;
        EXPECT_EQ(instructions[i].dst, expected.at(i).dst) << "instruction " << i;
        EXPECT_EQ(instructions[i].src, expected.at(i).src) << "instruction " << i;
    }
}
