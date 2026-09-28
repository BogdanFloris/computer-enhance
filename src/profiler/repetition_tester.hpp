#pragma once

#include "profiler.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <sys/resource.h>

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
    uint64_t total_minor_faults = 0;
    uint64_t total_major_faults = 0;
    uint64_t min_minor_faults = 0;
    uint64_t min_major_faults = 0;
    uint64_t max_minor_faults = 0;
    uint64_t max_major_faults = 0;
};

struct RepSample {
    uint64_t elapsed = 0;
    uint64_t minor_faults = 0;
    uint64_t major_faults = 0;
};

inline std::optional<rusage> read_usage() {
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
        std::perror("getrusage");
        return std::nullopt;
    }
    return usage;
}

inline RepSample make_sample(uint64_t elapsed, const rusage& before, const rusage& after) {
    return {elapsed, static_cast<uint64_t>(after.ru_minflt - before.ru_minflt),
            static_cast<uint64_t>(after.ru_majflt - before.ru_majflt)};
}

inline bool record(RepTesterResult& result, RepSample sample) {
    const bool new_minimum = sample.elapsed < result.min_time;
    const bool new_maximum = sample.elapsed > result.max_time || result.test_count == 0;

    result.test_count++;
    result.total_time += sample.elapsed;
    result.total_minor_faults += sample.minor_faults;
    result.total_major_faults += sample.major_faults;
    if (new_minimum) {
        result.min_time = sample.elapsed;
        result.min_minor_faults = sample.minor_faults;
        result.min_major_faults = sample.major_faults;
    }
    if (new_maximum) {
        result.max_time = sample.elapsed;
        result.max_minor_faults = sample.minor_faults;
        result.max_major_faults = sample.major_faults;
    }

    return new_minimum;
}

template <typename Test>
bool run_until_stable(RepTesterResult& result, uint64_t timeout, Test&& test) {
    uint64_t minimum_found_at = read_cpu_timer();

    while (read_cpu_timer() - minimum_found_at < timeout) {
        std::optional<RepSample> sample = test();
        if (!sample) {
            return false;
        }

        if (record(result, *sample)) {
            minimum_found_at = read_cpu_timer();
        }
    }

    return true;
}

inline void print_stat(uint64_t elapsed, uint64_t cpu_timer_freq, uint64_t byte_count,
                       double minor_faults, double major_faults, const char* tag) {
    const double seconds = ((double)elapsed / (double)cpu_timer_freq);
    const double gibibytes = static_cast<double>(byte_count) / (1024.0 * 1024.0 * 1024.0);
    const double gbps = gibibytes / seconds;
    std::cout << tag << ": " << elapsed << " (" << seconds * 1000 << "ms) " << gbps
              << "gb/s faults (minor/major): " << minor_faults << "/" << major_faults << "\n";
}

inline void print_result(const RepTesterResult& result, const std::string& fname,
                         uint64_t cpu_timer_freq, uint64_t byte_count) {
    std::cout << "--- " << fname << " ---\n";
    print_stat(result.min_time, cpu_timer_freq, byte_count, result.min_minor_faults,
               result.min_major_faults, "Min");
    print_stat(result.max_time, cpu_timer_freq, byte_count, result.max_minor_faults,
               result.max_major_faults, "Max");
    print_stat(result.total_time / result.test_count, cpu_timer_freq, byte_count,
               static_cast<double>(result.total_minor_faults) / result.test_count,
               static_cast<double>(result.total_major_faults) / result.test_count, "Avg");
}

} // namespace profiler
