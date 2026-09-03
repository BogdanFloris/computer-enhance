#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#ifndef PROFILER
#define PROFILER 1
#endif

namespace profiler {

struct Buffer {
    uint8_t* data;
    size_t size;
};

static Buffer allocate_buffer(size_t size) {
    Buffer res{};
    res.data = (uint8_t*)malloc(size);
    if (res.data != nullptr) {
        res.size = size;
    } else {
        std::cerr << "error: unable to allocate " << size << " bytes.\n";
    }
    return res;
}

struct RepTesterResult {
    uint64_t test_count = 0;
    uint64_t total_time = 0;
    uint64_t max_time = 0;
    uint64_t min_time = UINT64_MAX;
    uint64_t avg_time = 0;
};

inline bool record(RepTesterResult& result, uint64_t elapsed) {
    const bool new_minimum = elapsed < result.min_time;

    result.test_count++;
    result.total_time += elapsed;
    result.min_time = std::min(elapsed, result.min_time);
    result.max_time = std::max(elapsed, result.max_time);

    return new_minimum;
}

inline void print_stat(uint64_t elapsed, uint64_t cpu_timer_freq, uint64_t byte_count,
                       const char* tag) {
    const double seconds = ((double)elapsed / (double)cpu_timer_freq);
    const double gibibytes = static_cast<double>(byte_count) / (1024.0 * 1024.0 * 1024.0);
    const double gbps = gibibytes / seconds;
    std::cout << tag << ": " << elapsed << " (" << seconds * 1000 << "ms) " << gbps << "gb/s\n";
}

inline void print_result(const RepTesterResult& result, const char* fname, uint64_t cpu_timer_freq,
                         uint64_t byte_count) {
    std::cout << "--- " << fname << " ---\n";
    print_stat(result.min_time, cpu_timer_freq, byte_count, "Min");
    print_stat(result.max_time, cpu_timer_freq, byte_count, "Max");
    print_stat(result.total_time / result.test_count, cpu_timer_freq, byte_count, "Avg");
}

} // namespace profiler
