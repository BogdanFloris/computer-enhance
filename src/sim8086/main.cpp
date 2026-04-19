#include "decoder.hpp"
#include "simulator.hpp"

#include <iostream>

int main() {
    std::cout << "sim8086\n";
    std::cout << "===================================\n";
    std::vector<uint8_t> bytes = {
        0xB9, 0xC8, 0x00, 0x89, 0xCB, 0x81, 0xC1, 0xE8, 0x03, 0xBB, 0xD0, 0x07, 0x29, 0xD9,
    };
    auto instructions = Instruction::decode_bytes(bytes);
    Simulator sim(std::move(instructions), true /* debug */);
    sim.exec();
    return 0;
}
