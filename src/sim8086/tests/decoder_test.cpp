#include <gtest/gtest.h>
#include <vector>

#include "decoder.hpp"

TEST(Decoder, MovRegToReg) {
    std::vector<uint8_t> bytes = {0x89, 0xd8};
    auto instr = decode(bytes);
    EXPECT_EQ(instr.op, Op::mov);
    EXPECT_EQ(instr.src, Reg::bx);
    EXPECT_EQ(instr.dst, Reg::ax);
}
