#include "decoder.hpp"

#include <gtest/gtest.h>
#include <sstream>
#include <vector>

TEST(Decoder, Listing37) {
    std::vector<uint8_t> bytes = {0x89, 0xd9}; // mov cx,bx
    auto instructions = Instruction::decode_bytes(bytes);
    std::ostringstream ss;
    ss << instructions[0];
    EXPECT_EQ(ss.str(), "mov cx, bx");
}

TEST(Decoder, Listing38) {
    std::vector<uint8_t> bytes = {
        0x89, 0xD9, 0x88, 0xE5, 0x89, 0xDA, 0x89, 0xDE, 0x89, 0xFB, 0x88,
        0xC8, 0x88, 0xED, 0x89, 0xC3, 0x89, 0xF3, 0x89, 0xFC, 0x89, 0xC5,
    };

    auto instructions = Instruction::decode_bytes(bytes);

    std::array<std::string, 11> expected = {{
        "mov cx, bx",
        "mov ch, ah",
        "mov dx, bx",
        "mov si, bx",
        "mov bx, di",
        "mov al, cl",
        "mov ch, ch",
        "mov bx, ax",
        "mov bx, si",
        "mov sp, di",
        "mov bp, ax",
    }};

    for (size_t i = 0; i < std::size(expected); ++i) {
        std::ostringstream ss;
        ss << instructions[i];
        EXPECT_EQ(ss.str(), expected.at(i)) << "instruction " << i;
    }
}
