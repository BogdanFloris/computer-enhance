#include "processor.hpp"

#include "reference_haversine.hpp"

#include <array>
#include <bit>
#include <fstream>
#include <ios>

namespace haversine {

std::optional<std::vector<f64>> read_answers(const std::string& path) {
    std::ifstream in{path, std::ios::binary};
    if (!in.is_open()) {
        return std::nullopt;
    }
    std::vector<f64> values;
    std::array<char, sizeof(f64)> bytes{};
    while (in.read(bytes.data(), bytes.size())) {
        values.push_back(std::bit_cast<f64>(bytes));
    }
    return values;
}

ComputeResult evaluate_pairs(const std::vector<Pair>& pairs, const std::vector<f64>* answers) {
    ComputeResult result;
    f64 coeff = 1.0 / static_cast<f64>(pairs.size());
    for (size_t i = 0; i < pairs.size(); ++i) {
        const auto& p = pairs[i];
        f64 distance = reference_haversine(p.x0, p.y0, p.x1, p.y1);
        result.sum += coeff * distance;
        if (answers != nullptr && distance != (*answers)[i]) {
            ++result.mismatches;
        }
    }
    return result;
}

ValidationResult validate(const std::vector<Pair>& pairs, const std::vector<f64>& answers) {
    ValidationResult result;
    result.expected_values = pairs.size() + 1;
    result.actual_values = answers.size();
    result.size_ok = (answers.size() == result.expected_values);
    if (!result.size_ok) {
        result.computed_sum = evaluate_pairs(pairs, nullptr).sum;
        return result;
    }

    ComputeResult computed = evaluate_pairs(pairs, &answers);
    result.computed_sum = computed.sum;
    result.mismatches = computed.mismatches;
    result.reference_sum = answers.back();
    return result;
}

} // namespace haversine
