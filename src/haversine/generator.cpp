#include "generator.hpp"

#include "haversine.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <fstream>
#include <ios>
#include <random>
#include <stdexcept>
#include <vector>

namespace haversine {

namespace {

constexpr uint64_t cluster_count = 64;
constexpr f64 max_x_radius = 30.0;
constexpr f64 max_y_radius = 15.0;

struct Cluster {
    f64 x_center;
    f64 y_center;
    f64 x_radius;
    f64 y_radius;
};

void write_f64(std::ostream& out, f64 value) {
    auto bytes = std::bit_cast<std::array<char, sizeof(f64)>>(value);
    out.write(bytes.data(), bytes.size());
}

// Draw one coordinate in [center - radius, center + radius], clamped to the
// axis limit (180 for longitude, 90 for latitude).
f64 draw_coord(std::mt19937_64& rng, f64 center, f64 radius, f64 limit) {
    std::uniform_real_distribution<f64> dist(center - radius, center + radius);
    return std::clamp(dist(rng), -limit, limit);
}

Pair uniform_pair(std::mt19937_64& rng) {
    std::uniform_real_distribution<f64> lon_dist(-180.0, 180.0);
    std::uniform_real_distribution<f64> lat_dist(-90.0, 90.0);
    return Pair{
        .x0 = lon_dist(rng),
        .y0 = lat_dist(rng),
        .x1 = lon_dist(rng),
        .y1 = lat_dist(rng),
    };
}

Pair cluster_pair(std::mt19937_64& rng, const Cluster& c) {
    return Pair{
        .x0 = draw_coord(rng, c.x_center, c.x_radius, 180.0),
        .y0 = draw_coord(rng, c.y_center, c.y_radius, 90.0),
        .x1 = draw_coord(rng, c.x_center, c.x_radius, 180.0),
        .y1 = draw_coord(rng, c.y_center, c.y_radius, 90.0),
    };
}

std::vector<Cluster> make_clusters(std::mt19937_64& rng) {
    std::uniform_real_distribution<f64> lon_dist(-180.0, 180.0);
    std::uniform_real_distribution<f64> lat_dist(-90.0, 90.0);
    std::uniform_real_distribution<f64> x_radius_dist(0.0, max_x_radius);
    std::uniform_real_distribution<f64> y_radius_dist(0.0, max_y_radius);

    std::vector<Cluster> clusters;
    clusters.reserve(cluster_count);
    for (uint64_t i = 0; i < cluster_count; ++i) {
        clusters.push_back(Cluster{
            .x_center = lon_dist(rng),
            .y_center = lat_dist(rng),
            .x_radius = x_radius_dist(rng),
            .y_radius = y_radius_dist(rng),
        });
    }
    return clusters;
}

} // namespace

std::optional<Method> method_from_string(std::string_view name) {
    if (name == "uniform") {
        return Method::uniform;
    }
    if (name == "cluster") {
        return Method::cluster;
    }
    return std::nullopt;
}

GenerateResult generate(const GenerateParams& params, const std::string& json_path,
                        const std::string& answers_path) {
    std::ofstream json{json_path};
    if (!json) {
        throw std::runtime_error("failed to open JSON output file: " + json_path);
    }
    std::ofstream answers{answers_path, std::ios::binary};
    if (!answers) {
        throw std::runtime_error("failed to open answers output file: " + answers_path);
    }
    json.precision(17);

    std::mt19937_64 rng{params.seed};
    f64 sum = 0.0;
    f64 sum_coefficient =
        (params.pair_count > 0) ? (1.0 / static_cast<f64>(params.pair_count)) : 0.0;

    // In cluster mode, pre-generate the clusters and assign pairs to them in
    // contiguous blocks (every pair_count / cluster_count pairs picks the next
    // cluster). Drawing both centers and radii here keeps the run reproducible
    // from the seed.
    std::vector<Cluster> clusters;
    uint64_t block_size = 1;
    if (params.method == Method::cluster) {
        clusters = make_clusters(rng);
        block_size = std::max<uint64_t>(1, params.pair_count / cluster_count);
    }

    json << "{\"pairs\":[\n";
    for (uint64_t i = 0; i < params.pair_count; ++i) {
        Pair pair{};
        if (params.method == Method::cluster) {
            const Cluster& c = clusters[std::min(i / block_size, cluster_count - 1)];
            pair = cluster_pair(rng, c);
        } else {
            pair = uniform_pair(rng);
        }

        json << "    {\"x0\":" << pair.x0 << ", \"y0\":" << pair.y0 << ", \"x1\":" << pair.x1
             << ", \"y1\":" << pair.y1 << '}';
        if (i + 1 < params.pair_count) {
            json << ',';
        }
        json << '\n';

        f64 distance = reference_haversine(pair.x0, pair.y0, pair.x1, pair.y1);
        sum += sum_coefficient * distance;
        write_f64(answers, distance);
    }
    json << "]}\n";

    write_f64(answers, sum);
    return GenerateResult{.pair_count = params.pair_count, .expected_sum = sum};
}

} // namespace haversine
