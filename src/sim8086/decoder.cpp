#include <decoder.hpp>

Instruction decode(const std::vector<uint8_t>& bytes) {
    return Instruction{
        .op = Op::mov,
        .src = Reg::ax,
        .dst = Reg::bx,
    };
}
