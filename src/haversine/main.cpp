#include "generator.hpp"
#include "parser.hpp"

#include <charconv>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <span>
#include <sstream>
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

void print_generate_usage(std::string_view program) {
    std::cerr << "usage: " << program << " generate [uniform/cluster] [seed] [pair count]\n";
}

void print_compute_usage(std::string_view program) {
    std::cerr << "usage: " << program << " compute [input path]\n";
}

int generate_input(std::span<char*> args, const std::string_view& program) {
    if (args.size() < 3) {
        print_generate_usage(program);
        return 1;
    }

    std::string_view method_name = args[0];
    auto method = haversine::method_from_string(method_name);
    if (!method) {
        std::cerr << "error: unknown method '" << method_name << "'\n";
        print_generate_usage(program);
        return 1;
    }

    auto seed = parse_u64(args[1]);
    if (!seed) {
        std::cerr << "error: invalid seed '" << args[1] << "'\n";
        print_generate_usage(program);
        return 1;
    }

    auto pair_count = parse_u64(args[2]);
    if (!pair_count) {
        std::cerr << "error: invalid pair count '" << args[2] << "'\n";
        print_generate_usage(program);
        return 1;
    }

    haversine::GenerateParams params{
        .method = *method,
        .seed = *seed,
        .pair_count = *pair_count,
    };

    std::string stem =
        "samples/haversine_" + std::string(method_name) + "_" + std::to_string(*pair_count);
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

int compute_distances(std::span<char*> args, const std::string_view& program) {
    if (args.size() < 1) {
        print_compute_usage(program);
        return 1;
    }

    std::ifstream input{args[0]};
    if (!input.is_open()) {
        std::cerr << "error: could not open " << args[0] << "\n";
        return 1;
    }
    std::stringstream buf;
    buf << input.rdbuf();
    std::vector<haversine::Pair> pairs;
    input.close();
    auto res = haversine::parse_input(buf.str(), pairs);
    switch (res) {
    case haversine::ParseStatus::ok: {
        std::cout << "Read " << pairs.size() << "\n";
        std::cout << std::setprecision(17);
        for (auto& pair : pairs) {
            std::cout << pair << "\n";
        }
        return 0;
    }
    case haversine::ParseStatus::no_pairs_key:
        std::cerr << "error: no pairs key in provided input\n";
        return 1;
    case haversine::ParseStatus::malformed:
        std::cerr << "error: malformed json\n";
        return 1;
    }
}

} // namespace

int main(int argc, char* argv[]) {
    std::span<char*> args{argv, static_cast<size_t>(argc)};
    std::string_view program = !args.empty() ? args[0] : "haversine";

    if (args.size() < 2) {
        std::cerr << "error: command not found\n";
        return 1;
    }

    std::string_view command = args[1];
    std::span<char*> command_args = args.subspan(2);
    if (command == "compute") {
        return compute_distances(command_args, program);
    }
    if (command == "generate") {
        return generate_input(command_args, program);
    }
    std::cerr << "error: invalid command '" << command << "'\n";
    return 1;
}
