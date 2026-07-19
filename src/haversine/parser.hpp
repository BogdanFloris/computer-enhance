#pragma once

#include "haversine.hpp"
#include "profiler.hpp"

#include <cstdint>
#include <vector>

namespace haversine {

namespace {

bool extract_value(const std::string& object_str, const std::string& key, double& out_value) {
    std::string search_key = "\"" + key + "\"";
    size_t key_pos = object_str.find(search_key);
    if (key_pos == std::string::npos) {
        return false;
    }

    size_t colon_pos = object_str.find(':', key_pos);
    if (colon_pos == std::string::npos) {
        return false;
    }

    try {
        out_value = std::stod(object_str.substr(colon_pos + 1));
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace

enum class ParseStatus : uint8_t { ok, no_pairs_key, malformed };

inline ParseStatus parse_input(const std::string& input, std::vector<Pair>& pairs) {
    BEGIN_PROF();
    size_t array_pos = input.find("\"pairs\"");
    if (array_pos == std::string::npos) {
        return ParseStatus::no_pairs_key;
    }

    size_t array_start = input.find('[', array_pos);
    if (array_start == std::string::npos) {
        return ParseStatus::malformed;
    }
    size_t array_end = input.find(']', array_start);
    if (array_end == std::string::npos) {
        return ParseStatus::malformed;
    }

    size_t obj_start = input.find('{', array_start);
    while (obj_start != std::string::npos && obj_start < array_end) {
        size_t obj_end = input.find('}', obj_start);
        if (obj_end == std::string::npos || obj_end > array_end) {
            return ParseStatus::malformed;
        }

        std::string obj_str = input.substr(obj_start, obj_end - obj_start);
        Pair p{};
        bool valid = true;
        valid &= extract_value(obj_str, "x0", p.x0);
        valid &= extract_value(obj_str, "y0", p.y0);
        valid &= extract_value(obj_str, "x1", p.x1);
        valid &= extract_value(obj_str, "y1", p.y1);
        if (!valid) {
            return ParseStatus::malformed;
        }

        pairs.push_back(p);
        obj_start = input.find('{', obj_end);
    }

    return ParseStatus::ok;
}

}; // namespace haversine
