#include <iostream>

#include "decoder.hpp"

int main() {
    std::cout << "sim8086: 8086 decoder\n";
    std::vector<uint8_t> bytes = {0x70, 0xFE}; // jo 0x0
    auto instructions = Instruction::decode_bytes(bytes);

    for (auto inst: instructions) {
        std::cout << inst << "\n";
    }
    return 0;
}
