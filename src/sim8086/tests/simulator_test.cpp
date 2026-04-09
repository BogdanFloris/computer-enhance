#include "decoder.hpp"
#include "simulator.hpp"

#include "gtest/gtest.h"
#include <cstddef>
#include <string>
#include <vector>

struct SimulatorTestCase {
    std::string name;
    std::vector<uint8_t> bytes;
    Registers expectedRegs;
};

class SimulatorTest : public testing::TestWithParam<SimulatorTestCase> {};

TEST_P(SimulatorTest, SimulatesCorrectly) {
    const auto& [name, bytes, expectedRegs] = GetParam();
    auto instructions = Instruction::decode_bytes(bytes);
    Simulator sim(std::move(instructions));
    sim.exec();

    const auto actual_registers = sim.registers();
    for (size_t i = 0; i < actual_registers.size(); ++i) {
        EXPECT_EQ(actual_registers.at(i), expectedRegs.at(i)) << "register " << i;
    }
}

INSTANTIATE_TEST_SUITE_P(
    Listings, SimulatorTest,
    testing::Values(SimulatorTestCase{"Listing43",
                                      {
                                          0xB8, 0x01, 0x00, 0xBB, 0x02, 0x00, 0xB9, 0x03,
                                          0x00, 0xBA, 0x04, 0x00, 0xBC, 0x05, 0x00, 0xBD,
                                          0x06, 0x00, 0xBE, 0x07, 0x00, 0xBF, 0x08, 0x00,
                                      },
                                      {
                                          0x0001,
                                          0x0003,
                                          0x0004,
                                          0x0002,
                                          0x0005,
                                          0x0006,
                                          0x0007,
                                          0x0008,
                                      }},
                    SimulatorTestCase{"Listing44",
                                      {
                                          0xB8, 0x01, 0x00, 0xBB, 0x02, 0x00, 0xB9,
                                          0x03, 0x00, 0xBA, 0x04, 0x00, 0x89, 0xC4,
                                          0x89, 0xDD, 0x89, 0xCE, 0x89, 0xD7, 0x89,
                                          0xE2, 0x89, 0xE9, 0x89, 0xF3, 0x89, 0xF8,

                                      },
                                      {
                                          0x0004,
                                          0x0002,
                                          0x0001,
                                          0x0003,
                                          0x0001,
                                          0x0002,
                                          0x0003,
                                          0x0004,

                                      }}),
    [](const auto& info) { return info.param.name; });
