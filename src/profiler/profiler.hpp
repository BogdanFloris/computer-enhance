#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sys/time.h>
#include <vector>

namespace profiler {

constexpr uint64_t os_timer_freq = 1000000;
constexpr size_t max_profile_zones = 4096;
constexpr size_t max_profile_threads = 64;

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
    return (os_elapsed != 0) ? (os_timer_freq * cpu_elapsed / os_elapsed) : 0;
}

struct ProfileAnchor {
    uint64_t total_tsc = 0; // Inclusive time
    int64_t self_tsc = 0;   // Exclusive time
    uint64_t hit_count = 0;
    uint32_t active_count = 0; // Used to prevent overcounting total time during recursion
};

struct ThreadProfileData {
    std::array<ProfileAnchor, max_profile_zones> anchors{};
    uint32_t parent_index = 0;
};

inline std::array<const char*, max_profile_zones> g_zone_labels = {"Root"};
inline std::atomic<uint32_t> g_zone_count{1};
inline std::array<ThreadProfileData, max_profile_threads> g_thread_data;
inline std::atomic<uint32_t> g_thread_count{0};

inline uint32_t allocate_zone(const char* label) {
    uint32_t index = g_zone_count.fetch_add(1, std::memory_order_relaxed);
    assert(index < max_profile_zones && "Exceeded MAX_PROFILE_ZONES!");
    g_zone_labels.at(index) = label;
    return index;
}

inline ThreadProfileData* get_thread_data() {
    static thread_local ThreadProfileData* td = []() {
        uint32_t index = g_thread_count.fetch_add(1, std::memory_order_relaxed);
        assert(index < max_profile_threads && "Exceeded MAX_PROFILE_THREADS!");
        return &g_thread_data.at(index);
    }();
    return td;
}

class ScopedProfile {
  public:
    ScopedProfile(ScopedProfile&&) = delete;
    ScopedProfile& operator=(ScopedProfile&&) = delete;
    ScopedProfile(const ScopedProfile&) = delete;
    ScopedProfile& operator=(const ScopedProfile&) = delete;

    explicit ScopedProfile(uint32_t zone_index)
        : m_thread_data(get_thread_data()), m_zone_index(zone_index),
          m_parent_index(m_thread_data->parent_index), m_start_tsc(read_cpu_timer()) {
        m_thread_data->parent_index = zone_index;
        m_thread_data->anchors.at(m_zone_index).active_count++;
    }

    ~ScopedProfile() { end(); }

    void end() {
        if (m_ended) {
            return;
        }
        uint64_t elapsed = read_cpu_timer() - m_start_tsc;

        ProfileAnchor& zone = m_thread_data->anchors.at(m_zone_index);
        zone.self_tsc += (int64_t)elapsed;
        zone.hit_count++;
        zone.active_count--;

        // Only add to total if we are the outermost call of this specific zone (handles recursion)
        if (zone.active_count == 0) {
            zone.total_tsc += elapsed;
        }

        if (m_parent_index != 0) {
            m_thread_data->anchors.at(m_parent_index).self_tsc -= (int64_t)elapsed;
        }

        m_thread_data->parent_index = m_parent_index;
        m_ended = true;
    }

  private:
    ThreadProfileData* m_thread_data;
    uint32_t m_zone_index;
    uint32_t m_parent_index;
    uint64_t m_start_tsc;
    bool m_ended = false;
};

// ----------------------------------------------------------------------------
// Macros
// ----------------------------------------------------------------------------
#define PROF_CONCAT_IMPL(x, y) x##y
#define PROF_CONCAT(x, y) PROF_CONCAT_IMPL(x, y)
// The static index guarantees allocate_zone() is only called once per block of code.
// No strings are hashed, no maps are checked during the hot path.
#define BEGIN_PROF_NAMED(identifier, label)                                                        \
    static uint32_t PROF_CONCAT(prof_zone_, identifier) = 0;                                       \
    if (PROF_CONCAT(prof_zone_, identifier) == 0) {                                                \
        PROF_CONCAT(prof_zone_, identifier) = profiler::allocate_zone(label);                      \
    }                                                                                              \
    profiler::ScopedProfile PROF_CONCAT(profiler_, identifier)(PROF_CONCAT(prof_zone_, identifier))

#define BEGIN_PROF_TAG(tag) BEGIN_PROF_NAMED(__LINE__, tag)
#define BEGIN_PROF() BEGIN_PROF_NAMED(__LINE__, __func__)
#define END_PROF(identifier) PROF_CONCAT(profiler_, identifier).end()

inline void print_profile_report(uint64_t cpu_timer_freq = estimate_cpu_timer_freq()) {
    if (cpu_timer_freq == 0) {
        return;
    }
    struct AggregatedZone {
        const char* label;
        uint64_t total_tsc;
        int64_t self_tsc;
        uint64_t hit_count;
    };

    uint32_t num_zones = g_zone_count.load(std::memory_order_relaxed);
    uint32_t num_threads = g_thread_count.load(std::memory_order_relaxed);
    std::vector<AggregatedZone> results;
    results.reserve(num_zones);
    double ms_per_cycle = 1000.0 / static_cast<double>(cpu_timer_freq);
    uint64_t global_total_tsc = 0;

    // Aggregate across all threads
    for (uint32_t z = 1; z < num_zones; ++z) {
        AggregatedZone agg = {
            .label = g_zone_labels.at(z), .total_tsc = 0, .self_tsc = 0, .hit_count = 0};
        for (uint32_t t = 0; t < num_threads; ++t) {
            const ProfileAnchor& a = g_thread_data.at(t).anchors.at(z);
            agg.total_tsc += a.total_tsc;
            agg.self_tsc += a.self_tsc;
            agg.hit_count += a.hit_count;
        }
        if (agg.hit_count > 0) {
            results.push_back(agg);
            global_total_tsc = std::max(agg.total_tsc, global_total_tsc);
        }
    }

    std::ranges::sort(results, [](const AggregatedZone& a, const AggregatedZone& b) {
        return a.self_tsc > b.self_tsc;
    });

    std::cout << "\n=== Profiler Report ===\n";
    std::cout << std::left << std::setw(30) << "Zone Name" << std::right << std::setw(15)
              << "Self Time" << std::setw(15) << "Total Time" << std::setw(12) << "Calls" << "\n";
    std::cout << std::string(72, '-') << "\n";

    for (const auto& r : results) {
        double self_ms = static_cast<double>(r.self_tsc) * ms_per_cycle;
        double total_ms = static_cast<double>(r.total_tsc) * ms_per_cycle;

        std::cout << std::left << std::setw(30) << r.label << std::right << std::setw(11)
                  << std::fixed << std::setprecision(3) << self_ms << " ms" << std::setw(11)
                  << total_ms << " ms" << std::setw(12) << r.hit_count << "\n";
    }
    std::cout << std::string(72, '-') << "\n";
}

} // namespace profiler
