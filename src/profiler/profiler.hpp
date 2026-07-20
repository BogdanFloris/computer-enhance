#pragma once

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <ranges>
#include <string>
#include <string_view>
#include <sys/time.h>
#include <utility>
#include <vector>

namespace profiler {

constexpr uint64_t os_timer_freq = 1000000;
constexpr size_t max_profile_depth = 64;

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
#define BEGIN_PROF_TAG(tag) BEGIN_PROF_TAG_IMPL(tag, __LINE__)
#define BEGIN_PROF_TAG_IMPL(tag, line)                                                             \
    static thread_local profiler::ProfileNodeCache PROF_CONCAT(profile_node_cache_, line);         \
    profiler::Profiler PROF_CONCAT(profiler_, line)(tag, PROF_CONCAT(profile_node_cache_, line))
#define BEGIN_PROF() BEGIN_PROF_TAG(__func__)
#define BEGIN_PROF_NAMED(identifier, tag)                                                          \
    static thread_local profiler::ProfileNodeCache PROF_CONCAT(profile_node_cache_, identifier);   \
    profiler::Profiler PROF_CONCAT(profiler_,                                                      \
                                   identifier)(tag, PROF_CONCAT(profile_node_cache_, identifier))
#define END_PROF(identifier) PROF_CONCAT(profiler_, identifier).end()

struct ProfileStats {
    std::atomic<uint64_t> total_cycles = 0;
    std::atomic<uint64_t> child_cycles = 0;
    std::atomic<uint64_t> call_count = 0;
    std::atomic<uint64_t> recursive_scopes_elided = 0;
    std::atomic<uint64_t> depth_scopes_elided = 0;
};

struct ProfileNode {
    std::string_view label;
    ProfileStats stats;
    std::map<std::string, ProfileNode, std::less<>> children;
};

struct ProfileNodeCache {
    ProfileNode* parent = nullptr;
    ProfileNode* node = nullptr;
};

class ProfileRegistry {
  public:
    ProfileNode* enter(ProfileNode* parent, std::string_view label) {
        std::lock_guard lock(mMutex);
        auto& nodes = parent != nullptr ? parent->children : mRoots;
        auto it = nodes.find(label);
        if (it == nodes.end()) {
            it = nodes.try_emplace(std::string(label)).first;
            it->second.label = it->first;
        }
        return &it->second;
    }

    static void record(ProfileNode* node, ProfileNode* parent, uint64_t elapsed_cycles) {
        node->stats.total_cycles.fetch_add(elapsed_cycles, std::memory_order_relaxed);
        node->stats.call_count.fetch_add(1, std::memory_order_relaxed);
        if (parent != nullptr) {
            parent->stats.child_cycles.fetch_add(elapsed_cycles, std::memory_order_relaxed);
        }
    }

    static void record_elided(ProfileNode* node, bool recursive) {
        auto& count =
            recursive ? node->stats.recursive_scopes_elided : node->stats.depth_scopes_elided;
        count.fetch_add(1, std::memory_order_relaxed);
    }

    void print_report() const { print_report(estimate_cpu_timer_freq()); }

    void print_report(uint64_t cpu_timer_freq) const {
        if (cpu_timer_freq == 0) {
            return;
        }

        std::lock_guard lock(mMutex);
        bool printed_header = false;
        for (const auto& [label, root] : mRoots) {
            const uint64_t total_cycles = root.stats.total_cycles.load(std::memory_order_relaxed);
            if (total_cycles == 0) {
                continue;
            }

            if (!printed_header) {
                std::cout << "Profile\n";
                printed_header = true;
            }
            print_node(root, total_cycles, 1, 1000.0 / static_cast<double>(cpu_timer_freq));
        }
    }

  private:
    static void print_node(const ProfileNode& root, uint64_t report_total, int root_depth,
                           double milliseconds_per_cycle) {
        std::vector<std::pair<const ProfileNode*, int>> pending{{&root, root_depth}};
        while (!pending.empty()) {
            const auto current = pending.back();
            pending.pop_back();

            const ProfileNode& node = *current.first;
            const uint64_t total_cycles = node.stats.total_cycles.load(std::memory_order_relaxed);
            const uint64_t child_cycles = node.stats.child_cycles.load(std::memory_order_relaxed);
            const uint64_t self_cycles =
                total_cycles >= child_cycles ? total_cycles - child_cycles : 0;
            const uint64_t call_count = node.stats.call_count.load(std::memory_order_relaxed);
            const uint64_t recursive_scopes_elided =
                node.stats.recursive_scopes_elided.load(std::memory_order_relaxed);
            const uint64_t depth_scopes_elided =
                node.stats.depth_scopes_elided.load(std::memory_order_relaxed);
            const double percent =
                100.0 * static_cast<double>(total_cycles) / static_cast<double>(report_total);
            const double total_milliseconds = (double)total_cycles * milliseconds_per_cycle;
            const double self_milliseconds = (double)self_cycles * milliseconds_per_cycle;

            std::cout << std::string(static_cast<size_t>(current.second) * 2, ' ') << node.label
                      << ": " << std::fixed << std::setprecision(3) << total_milliseconds << " ms ("
                      << std::setprecision(2) << percent << "% total, ";
            if (recursive_scopes_elided != 0 || depth_scopes_elided != 0) {
                std::cout << "self unavailable, " << call_count << " calls";
                if (recursive_scopes_elided != 0) {
                    std::cout << ", " << recursive_scopes_elided << " recursive scopes elided";
                }
                if (depth_scopes_elided != 0) {
                    std::cout << ", " << depth_scopes_elided << " depth-limit scopes elided";
                }
                std::cout << ")\n";
            } else {
                std::cout << std::setprecision(3) << self_milliseconds << " ms self, " << call_count
                          << " calls)\n";
            }

            for (const auto& it : std::ranges::reverse_view(node.children)) {
                pending.emplace_back(&it.second, current.second + 1);
            }
        }
    }

    mutable std::mutex mMutex;
    std::map<std::string, ProfileNode, std::less<>> mRoots;
};

inline ProfileRegistry& profile_registry() {
    static ProfileRegistry registry;
    return registry;
}

inline void print_profile_report() {
    profile_registry().print_report();
}

class Profiler {
  public:
    Profiler(const Profiler&) = delete;
    Profiler(Profiler&&) = delete;
    Profiler& operator=(const Profiler&) = delete;
    Profiler& operator=(Profiler&&) = delete;
    explicit Profiler(std::string_view label) { begin(label, nullptr); }
    Profiler(std::string_view label, ProfileNodeCache& cache) { begin(label, &cache); }
    ~Profiler() { end(); }

    void end() {
        if (mEnded) {
            return;
        }
        assert(!active_profilers.empty() && active_profilers.back() == this &&
               "profilers must end in reverse construction order");
        uint64_t end = read_cpu_timer();
        uint64_t elapsed = end - mBegin;
        if (mElisionNode != nullptr) {
            profiler::ProfileRegistry::record_elided(mElisionNode, mElidedByRecursion);
        } else {
            profiler::ProfileRegistry::record(mNode, mParent != nullptr ? mParent->mNode : nullptr,
                                              elapsed);
        }
        active_profilers.pop_back();
        mEnded = true;
    }

  private:
    void begin(std::string_view label, ProfileNodeCache* cache) {
        mParent = active_profilers.empty() ? nullptr : active_profilers.back();
        if (mParent != nullptr && mParent->mElisionNode != nullptr) {
            mNode = mParent->mNode;
            mElisionNode = mParent->mElisionNode;
            mElidedByRecursion = mParent->mElidedByRecursion;
        } else if (active_profilers.size() >= max_profile_depth) {
            mNode = mParent != nullptr ? mParent->mNode : nullptr;
            mElisionNode = mNode;
        } else if (ProfileNode* boundary = recursive_boundary(label); boundary != nullptr) {
            mNode = mParent != nullptr ? mParent->mNode : nullptr;
            mElisionNode = boundary;
            mElidedByRecursion = true;
        } else if (has_active_label(label)) {
            initialize_node(label, cache);
            mRecursionBoundary = true;
        } else {
            initialize_node(label, cache);
        }
        mBegin = read_cpu_timer();
        active_profilers.push_back(this);
    }

    void initialize_node(std::string_view label, ProfileNodeCache* cache) {
        ProfileNode* parent_node = mParent != nullptr ? mParent->mNode : nullptr;
        if (cache != nullptr && cache->parent == parent_node && cache->node != nullptr) {
            mNode = cache->node;
        } else {
            mNode = profile_registry().enter(parent_node, label);
            if (cache != nullptr) {
                cache->parent = parent_node;
                cache->node = mNode;
            }
        }
    }

    static bool has_active_label(std::string_view label) {
        return std::ranges::any_of(active_profilers, [label](const Profiler* profiler) {
            return profiler->mElisionNode == nullptr && profiler->mNode->label == label;
        });
    }

    static ProfileNode* recursive_boundary(std::string_view label) {
        for (const Profiler* profiler : active_profilers) {
            if (profiler->mRecursionBoundary && profiler->mNode->label == label) {
                return profiler->mNode;
            }
        }
        return nullptr;
    }

    Profiler* mParent = nullptr;
    ProfileNode* mNode = nullptr;
    ProfileNode* mElisionNode = nullptr;
    uint64_t mBegin = 0;
    bool mEnded = false;
    bool mElidedByRecursion = false;
    bool mRecursionBoundary = false;
    inline static thread_local std::vector<Profiler*> active_profilers = [] {
        std::vector<Profiler*> profilers;
        profilers.reserve(16);
        return profilers;
    }();
};

} // namespace profiler
