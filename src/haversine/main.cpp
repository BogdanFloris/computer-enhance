#include "generator.hpp"

#include <charconv>
#include <iomanip>
#include <iostream>
#include <span>
#include <string>
#include <string_view>

namespace {

std::optional<uint64_t> parse_u64(std::string_view text) {
    uint64_t value = 0;
    auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (ec != std::errc{} || ptr != text.data() + text.size()) {
        return std::nullopt;
    }
    return value;
}

void print_usage(std::string_view program) {
    std::cerr << "usage: " << program << " [uniform/cluster] [seed] [pair count]\n";
}

} // namespace

int main(int argc, char* argv[]) {
    std::span<char*> args{argv, static_cast<size_t>(argc)};
    std::string_view program = !args.empty() ? args[0] : "haversine";

    if (args.size() != 4) {
        print_usage(program);
        return 1;
    }

    std::string_view method_name = args[1];
    auto method = haversine::method_from_string(method_name);
    if (!method) {
        std::cerr << "error: unknown method '" << method_name << "'\n";
        print_usage(program);
        return 1;
    }

    auto seed = parse_u64(args[2]);
    if (!seed) {
        std::cerr << "error: invalid seed '" << args[2] << "'\n";
        print_usage(program);
        return 1;
    }

    auto pair_count = parse_u64(args[3]);
    if (!pair_count) {
        std::cerr << "error: invalid pair count '" << args[3] << "'\n";
        print_usage(program);
        return 1;
    }

    haversine::GenerateParams params{
        .method = *method,
        .seed = *seed,
        .pair_count = *pair_count,
    };

    std::string stem = "samples/haversine_" + std::string(method_name) + "_" + std::to_string(*pair_count);
    std::string json_path = stem + ".json";
    std::string answers_path = stem + ".answers.f64";

    try {
        auto result = haversine::generate(params, json_path, answers_path);
        std::cout << "Method: " << method_name << '\n';
        std::cout << "Seed: " << *seed << '\n';
        std::cout << "Pair count: " << result.pair_count << '\n';
        std::cout << "Expected sum: " << std::setprecision(17) << result.expected_sum << '\n';
        std::cout << "Wrote " << json_path << " and " << answers_path << '\n';
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
