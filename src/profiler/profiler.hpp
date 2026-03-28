#pragma once

#include <cstdint>
#include <chrono>

namespace profiler {

inline uint64_t read_cpu_timer() {
#if defined(__x86_64__) || defined(_M_X64)
    return __builtin_ia32_rdtsc();
#elif defined(__aarch64__) || defined(_M_ARM64)
    uint64_t val;
    asm volatile("mrs %0, cntvct_el0" : "=r"(val));
    return val;
#else
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
#endif
}

} // namespace profiler
