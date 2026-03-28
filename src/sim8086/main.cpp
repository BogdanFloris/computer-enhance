#include <iostream>

#include "decoder.hpp"

int main() {
    std::cout << "sim8086: 8086 decoder\n";
    std::vector<uint8_t> bytes = {0x89, 0xd9}; // mov cx,bx
    auto instructions = Instruction::decode(bytes);

    for (auto inst: instructions) {
        std::cout << inst << "\n";
    }
    return 0;
}
