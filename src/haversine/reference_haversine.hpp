#pragma once
/*
 * Reference Haversine distance calculation.
 *
 * NOTE: This is not meant to be a "good" way to calculate the Haversine
 * distance. Instead, it attempts to follow, as closely as possible, the
 * formula used in the real-world question on which these exercises are
 * loosely based.
 */

#include <cmath>

namespace haversine {

using f64 = double;

// EarthRadius is generally expected to be 6372.8
constexpr f64 earth_radius = 6372.8;

inline f64 square(f64 a) {
    f64 result = a * a;
    return result;
}

inline f64 radians_from_degrees(f64 degrees) {
    f64 result = 0.01745329251994329577 * degrees;
    return result;
}

[[nodiscard]] inline f64 reference_haversine(f64 x0, f64 y0, f64 x1, f64 y1,
                                             f64 earth_radius = haversine::earth_radius) {
    f64 lat1 = y0;
    f64 lat2 = y1;
    f64 lon1 = x0;
    f64 lon2 = x1;

    f64 d_lat = radians_from_degrees(lat2 - lat1);
    f64 d_lon = radians_from_degrees(lon2 - lon1);
    lat1 = radians_from_degrees(lat1);
    lat2 = radians_from_degrees(lat2);

    f64 a = square(std::sin(d_lat / 2.0)) +
            (std::cos(lat1) * std::cos(lat2) * square(std::sin(d_lon / 2.0)));
    f64 c = 2.0 * std::asin(std::sqrt(a));

    f64 result = earth_radius * c;
    return result;
}

} // namespace haversine
