#include "simulator.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>

int main() {
    std::cout << "sim8086\n";
    std::cout << "===================================\n";
    const std::vector<uint8_t> bytes = {
        0xBA, 0x00, 0x00, 0xB9, 0x00, 0x00, 0x89, 0x4E, 0x00, 0x89, 0x56, 0x02,
        0xC6, 0x46, 0x03, 0xFF, 0x83, 0xC5, 0x04, 0x83, 0xC1, 0x01, 0x83, 0xF9,
        0x40, 0x75, 0xEB, 0x83, 0xC2, 0x01, 0x83, 0xFA, 0x40, 0x75, 0xE0,
    };
    Simulator sim(true /* debug */);
    auto span = std::span{bytes};
    sim.exec(span);
    {
        // write image to file
        std::span full_memory{sim.memory()};
        auto data_to_write = full_memory.subspan(256, 64L * 64 * 4);
        std::ofstream f{"output.data", std::ios::binary};
        if (f) {
            std::ranges::copy(data_to_write, std::ostreambuf_iterator<char>(f));
        }
    }
    return 0;
}
