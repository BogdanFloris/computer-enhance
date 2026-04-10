#include "decoder.hpp"
#include "simulator.hpp"

#include <iostream>

int main() {
    std::cout << "sim8086\n";
    std::cout << "===================================\n";
    std::vector<uint8_t> bytes = {
        0xBB, 0x03, 0xF0, 0xB9, 0x01, 0x0F, 0x29, 0xCB, 0xBC, 0xE6, 0x03, 0xBD,
        0xE7, 0x03, 0x39, 0xE5, 0x81, 0xC5, 0x03, 0x04, 0x81, 0xED, 0xEA, 0x07,
    };
    auto instructions = Instruction::decode_bytes(bytes);
    Simulator sim(std::move(instructions), true /* debug */);
    sim.exec();
    return 0;
}
