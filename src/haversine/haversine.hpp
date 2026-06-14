#pragma once

#include <ostream>

namespace haversine {

using f64 = double;

struct Pair {
    f64 x0; // longitude  [-180, 180]
    f64 y0; // latitude   [-90, 90]
    f64 x1;
    f64 y1;

    friend std::ostream& operator<<(std::ostream& os, const Pair& p) {
        return os << "{x0: " << p.x0 << ", y0: " << p.y0 << ", x1: " << p.x1 << ", y1: " << p.y1
                  << "}";
    }
};

} // namespace haversine
