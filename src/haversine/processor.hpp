#pragma once

#include "haversine.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace haversine {

std::optional<std::vector<f64>> read_answers(const std::string& path);

struct ComputeResult {
    f64 sum = 0.0;
    uint64_t mismatches = 0;
};

ComputeResult evaluate_pairs(const std::vector<Pair>& pairs, const std::vector<f64>* answers);

struct ValidationResult {
    f64 computed_sum = 0.0;
    f64 reference_sum = 0.0;
    uint64_t mismatches = 0;
    uint64_t expected_values = 0;
    uint64_t actual_values = 0;
    bool size_ok = false;
};

[[nodiscard]] inline bool passed(const ValidationResult& result) {
    return result.size_ok && result.mismatches == 0 && result.computed_sum == result.reference_sum;
}

ValidationResult validate(const std::vector<Pair>& pairs, const std::vector<f64>& answers);

} // namespace haversine
