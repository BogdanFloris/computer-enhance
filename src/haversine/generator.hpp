#pragma once
/*
 * Haversine input generator.
 *
 * Produces a JSON file of random coordinate pairs together with a binary
 * "answers" file holding the reference Haversine distance for each pair plus
 * the expected sum. The answers file is used later to validate a parser /
 * processor against this known-good input.
 */

#include "reference_haversine.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace haversine {

enum class Method : uint8_t {
    uniform,
    cluster,
};

std::optional<Method> method_from_string(std::string_view name);

struct Pair {
    f64 x0; // longitude  [-180, 180]
    f64 y0; // latitude   [-90, 90]
    f64 x1;
    f64 y1;
};

struct GenerateParams {
    Method method;
    uint64_t seed;
    uint64_t pair_count;
};

struct GenerateResult {
    uint64_t pair_count;
    f64 expected_sum;
};

GenerateResult generate(const GenerateParams& params, const std::string& json_path,
                        const std::string& answers_path);

} // namespace haversine
