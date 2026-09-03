#include "profiler.hpp"
#include "repetition_tester.hpp"

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <span>

constexpr uint64_t repetition_count = 100;
constexpr uint64_t seconds_without_improvement = 10;

int main(int argc, char* argv[]) {
    std::span<char*> args{argv, static_cast<size_t>(argc)};

    if (args.size() < 2) {
        std::cerr << "error: file not found\n";
        return 1;
    }
    std::error_code size_ec;
    uintmax_t byte_count = std::filesystem::file_size(args[1], size_ec);
    profiler::Buffer buffer = profiler::allocate_buffer(byte_count);
    std::FILE* f = std::fopen(args[1], "rb");
    if (f == nullptr) {
        std::cerr << "error: could not open " << args[1] << "\n";
        return 1;
    }

    const uint64_t cpu_timer_freq = profiler::estimate_cpu_timer_freq();
    const uint64_t timeout = seconds_without_improvement * cpu_timer_freq;
    uint64_t minimum_found_at = profiler::read_cpu_timer();
    profiler::RepTesterResult result{};

    while (profiler::read_cpu_timer() - minimum_found_at < timeout) {
        if (std::fseek(f, 0, SEEK_SET) != 0) {
            std::cerr << "error: seek failed\n";
            return 1;
        }

        uint64_t start = profiler::read_cpu_timer();
        size_t bytes_read = std::fread(buffer.data, 1, buffer.size, f);
        uint64_t elapsed = profiler::read_cpu_timer() - start;

        if (bytes_read != buffer.size) {
            std::cerr << "error: mismatched sizes " << args[1] << "\n";
            return 1;
        }

        if (profiler::record(result, elapsed)) {
            minimum_found_at = profiler::read_cpu_timer();
        }
    }

    profiler::print_result(result, "fread", cpu_timer_freq, byte_count);

    return 0;
}
