#pragma once

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/time.h>
#include <utility>

namespace profiler {

constexpr uint64_t os_timer_freq = 1000000;

inline uint64_t read_os_timer() {
    timeval val{};
    gettimeofday(&val, nullptr);
    return (os_timer_freq * val.tv_sec) + val.tv_usec;
}

inline uint64_t read_cpu_timer() {
#if defined(__x86_64__) || defined(_M_X64)
    return __builtin_ia32_rdtsc();
#elif defined(__aarch64__) || defined(_M_ARM64)
    uint64_t val{};
    asm volatile("mrs %0, cntvct_el0" : "=r"(val));
    return val;
#else
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
#endif
}

inline uint64_t estimate_cpu_timer_freq() {
    constexpr uint64_t millis_to_wait = 100;
    uint64_t cpu_start = read_cpu_timer();
    uint64_t os_start = read_os_timer();
    uint64_t os_end = 0;
    uint64_t os_elapsed = 0;
    uint64_t wait_time = os_timer_freq * millis_to_wait / 1000;
    while (os_elapsed < wait_time) {
        os_end = read_os_timer();
        os_elapsed = os_end - os_start;
    }

    uint64_t cpu_end = read_cpu_timer();
    uint64_t cpu_elapsed = cpu_end - cpu_start;
    uint64_t cpu_freq = 0;
    if (os_elapsed != 0) {
        cpu_freq = os_timer_freq * cpu_elapsed / os_elapsed;
    }
    return cpu_freq;
}

#define PROF_CONCAT_IMPL(x, y) x##y
#define PROF_CONCAT(x, y) PROF_CONCAT_IMPL(x, y)
#define BEGIN_PROF_TAG(tag) profiler::Profiler PROF_CONCAT(profiler_, __LINE__)(tag)
#define BEGIN_PROF() BEGIN_PROF_TAG(__func__)

class Profiler {
  public:
    Profiler(const Profiler&) = default;
    Profiler(Profiler&&) = delete;
    Profiler& operator=(const Profiler&) = delete;
    Profiler& operator=(Profiler&&) = delete;
    Profiler(std::string label) : mLabel(std::move(label)), mBegin(read_cpu_timer()) {}
    ~Profiler() {
        uint64_t end = read_cpu_timer();
        uint64_t elapsed = end - mBegin;
        float percent = 0;
        std::cout << "  " << mLabel << ": " << elapsed << " (" << std::fixed << std::setprecision(2)
                  << percent << "%)\n";
    }

  private:
    std::string mLabel;
    uint64_t mBegin;
};

} // namespace profiler
