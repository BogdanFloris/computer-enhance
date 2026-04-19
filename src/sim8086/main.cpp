#include "simulator.hpp"

#include <iostream>

int main() {
    std::cout << "sim8086\n";
    std::cout << "===================================\n";
    const std::vector<uint8_t> bytes = {
        0xB9, 0x03, 0x00, 0xBB, 0xE8, 0x03, 0x83, 0xC3, 0x0A, 0x83, 0xE9, 0x01, 0x75, 0xF8,
    };
    Simulator sim(true /* debug */);
    auto span = std::span{bytes};
    sim.exec(span);
    return 0;
}
