#include "decoder.hpp"

#include <simulator.hpp>

void Simulator::exec(const Instruction& instr) {}

void Simulator::exec() {
    for (auto& instr : mInstructions) {
        exec(instr);
    }
}
