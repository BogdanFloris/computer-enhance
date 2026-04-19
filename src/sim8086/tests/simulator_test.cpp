#include "decoder.hpp"
#include "simulator.hpp"

#include "gtest/gtest.h"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct SimulatorTestCase {
    std::string name;
    std::vector<uint8_t> bytes;
    Registers expectedRegs;
    uint16_t expectedFlags;
    uint16_t expectedIp;
};

class SimulatorTest : public testing::TestWithParam<SimulatorTestCase> {};

TEST_P(SimulatorTest, SimulatesCorrectly) {
    const auto& [name, bytes, expectedRegs, expectedFlags, expectedIp] = GetParam();
    std::vector<Instruction> instructions{};
    Simulator sim{};
    auto bytes_span = std::span{bytes};
    sim.exec(bytes_span);

    const auto actual_registers = sim.registers();
    for (size_t i = 0; i < actual_registers.size(); ++i) {
        EXPECT_EQ(actual_registers.at(i), expectedRegs.at(i)) << "register " << i;
    }
    EXPECT_EQ(sim.flags(), expectedFlags);
    EXPECT_EQ(sim.ip(), expectedIp);
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
                                      },
                                      0,
                                      0x18},
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
                                      },
                                      0,
                                      0x1C},
                    SimulatorTestCase{"Listing46",
                                      {
                                          0xBB, 0x03, 0xF0, 0xB9, 0x01, 0x0F, 0x29, 0xCB,
                                          0xBC, 0xE6, 0x03, 0xBD, 0xE7, 0x03, 0x39, 0xE5,
                                          0x81, 0xC5, 0x03, 0x04, 0x81, 0xED, 0xEA, 0x07,
                                      },
                                      {
                                          0x0000,
                                          0x0f01,
                                          0x0000,
                                          0xe102,
                                          0x03e6,
                                          0x0000,
                                          0x0000,
                                          0x0000,

                                      },
                                      0x40,
                                      0x18},
                    SimulatorTestCase{"Listing48",
                                      {
                                          0xB9,
                                          0xC8,
                                          0x00,
                                          0x89,
                                          0xCB,
                                          0x81,
                                          0xC1,
                                          0xE8,
                                          0x03,
                                          0xBB,
                                          0xD0,
                                          0x07,
                                          0x29,
                                          0xD9,
                                      },
                                      {
                                          0x0000,
                                          0xfce0,
                                          0x0000,
                                          0x07d0,
                                          0x0000,
                                          0x0000,
                                          0x0000,
                                          0x0000,
                                      },
                                      0x80,
                                      0x000e},
                    SimulatorTestCase{"Listing49",
                                      {
                                          0xB9,
                                          0x03,
                                          0x00,
                                          0xBB,
                                          0xE8,
                                          0x03,
                                          0x83,
                                          0xC3,
                                          0x0A,
                                          0x83,
                                          0xE9,
                                          0x01,
                                          0x75,
                                          0xF8,
                                      },
                                      {
                                          0x0000,
                                          0x0000,
                                          0x0000,
                                          0x0406,
                                          0x0000,
                                          0x0000,
                                          0x0000,
                                          0x0000,
                                      },
                                      0x40,
                                      0x000e}),
    [](const auto& info) { return info.param.name; });
