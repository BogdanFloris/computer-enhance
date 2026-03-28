#include <decoder.hpp>

std::vector<Instruction> decode(const std::vector<uint8_t>& bytes) {
    return {Instruction{
        .op = Op::mov,
        .src = Reg::bx,
        .dst = Reg::cx,
    }};
}
