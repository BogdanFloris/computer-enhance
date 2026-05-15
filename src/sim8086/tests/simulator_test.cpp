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
    size_t expectedClocks;
};

class SimulatorTest : public testing::TestWithParam<SimulatorTestCase> {};

TEST_P(SimulatorTest, SimulatesCorrectly) {
    const auto& [name, bytes, expectedRegs, expectedFlags, expectedIp, expectedClocks] = GetParam();
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
    EXPECT_EQ(sim.clocks(), expectedClocks);
}

TEST(SimulatorClockTest, PrintsClockTracePerInstruction) {
    const std::vector<uint8_t> bytes = {
        0xB8, 0x01, 0x00, 0xBB, 0x02, 0x00, 0x01, 0xD8,
    };
    Simulator sim(true);
    auto bytes_span = std::span{bytes};

    testing::internal::CaptureStdout();
    sim.exec(bytes_span);
    const auto output = testing::internal::GetCapturedStdout();

    EXPECT_NE(output.find("mov ax, 1 ; ax: 0x0000 -> 0x0001 Clocks: +4 = 4"), std::string::npos);
    EXPECT_NE(output.find("mov bx, 2 ; bx: 0x0000 -> 0x0002 Clocks: +4 = 8"), std::string::npos);
    EXPECT_NE(output.find("add ax, bx ; ax: 0x0001 -> 0x0003 flags:-> Clocks: +3 = 11"),
              std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(
    Listings, SimulatorTest,
    testing::Values(
        SimulatorTestCase{"Listing43",
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
                          0x18,
                          32},
        SimulatorTestCase{"Listing44",
                          {
                              0xB8, 0x01, 0x00, 0xBB, 0x02, 0x00, 0xB9, 0x03, 0x00, 0xBA,
                              0x04, 0x00, 0x89, 0xC4, 0x89, 0xDD, 0x89, 0xCE, 0x89, 0xD7,
                              0x89, 0xE2, 0x89, 0xE9, 0x89, 0xF3, 0x89, 0xF8,

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
                          0x1C,
                          32},
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
                          0x18,
                          30},
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
                          0x000e,
                          17},
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
                          0x000e,
                          68},
        SimulatorTestCase{"Listing51",
                          {
                              0xC7, 0x06, 0xE8, 0x03, 0x01, 0x00, 0xC7, 0x06, 0xEA, 0x03,
                              0x02, 0x00, 0xC7, 0x06, 0xEC, 0x03, 0x03, 0x00, 0xC7, 0x06,
                              0xEE, 0x03, 0x04, 0x00, 0xBB, 0xE8, 0x03, 0xC7, 0x47, 0x04,
                              0x0A, 0x00, 0x8B, 0x1E, 0xE8, 0x03, 0x8B, 0x0E, 0xEA, 0x03,
                              0x8B, 0x16, 0xEC, 0x03, 0x8B, 0x2E, 0xEE, 0x03,
                          },
                          {
                              0x0000,
                              0x0002,
                              0x000a,
                              0x0001,
                              0x0000,
                              0x0004,
                              0x0000,
                              0x0000,
                          },
                          0x00,
                          0x0030,
                          143},
        SimulatorTestCase{"Listing52",
                          {
                              0xBA, 0x06, 0x00, 0xBD, 0xE8, 0x03, 0xBE, 0x00, 0x00,
                              0x89, 0x32, 0x83, 0xC6, 0x02, 0x39, 0xD6, 0x75, 0xF7,
                              0xBB, 0x00, 0x00, 0xBE, 0x00, 0x00, 0x8B, 0x0A, 0x01,
                              0xCB, 0x83, 0xC6, 0x02, 0x39, 0xD6, 0x75, 0xF5,
                          },
                          {
                              0x0000,
                              0x0004,
                              0x0006,
                              0x0006,
                              0x0000,
                              0x03e8,
                              0x0006,
                              0x0000,
                          },
                          0x40,
                          0x0023,
                          242},
        SimulatorTestCase{"Listing56",
                          {
                              0xBB, 0xE8, 0x03, 0xBD, 0xD0, 0x07, 0xBE, 0xB8, 0x0B, 0xBF, 0xA0,
                              0x0F, 0x89, 0xD9, 0xBA, 0x0C, 0x00, 0x8B, 0x16, 0xE8, 0x03, 0x8B,
                              0x0F, 0x8B, 0x4E, 0x00, 0x89, 0x0C, 0x89, 0x0D, 0x8B, 0x8F, 0xE8,
                              0x03, 0x8B, 0x8E, 0xE8, 0x03, 0x89, 0x8C, 0xE8, 0x03, 0x89, 0x8D,
                              0xE8, 0x03, 0x01, 0xD1, 0x01, 0x8D, 0xE8, 0x03, 0x83, 0xC2, 0x32,
                          },
                          {
                              0x0000,
                              0x0000,
                              0x0032,
                              0x03e8,
                              0x0000,
                              0x07d0,
                              0x0bb8,
                              0x0fa0,
                          },
                          0x00,
                          0x0037,
                          192}),
    [](const auto& info) { return info.param.name; });
