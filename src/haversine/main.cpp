#include "generator.hpp"
#include "parser.hpp"
#include "processor.hpp"
#include "profiler.hpp"

#include <charconv>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

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
    std::cerr << "usage: " << program << " compute [input path] [answers_path?]\n";
}

const char* parse_error_message(haversine::ParseStatus status) {
    switch (status) {
    case haversine::ParseStatus::ok:
        return nullptr;
    case haversine::ParseStatus::no_pairs_key:
        return "no pairs key in provided input";
    case haversine::ParseStatus::malformed:
        return "malformed json";
    }
    return "unknown parse error";
}

int print_validation(uint64_t pair_count, const haversine::ValidationResult& result) {
    std::cout << std::setprecision(17);
    std::cout << "Pair count: " << pair_count << "\n";
    std::cout << "Sum: " << result.computed_sum << "\n";

    if (!result.size_ok) {
        std::cerr << "error: answers file has " << result.actual_values << " values, expected "
                  << result.expected_values << " (" << pair_count << " pairs + sum)\n";
        return 1;
    }

    std::cout << "Reference sum: " << result.reference_sum << "\n";
    std::cout << "Difference: " << (result.computed_sum - result.reference_sum) << "\n";
    if (result.mismatches != 0) {
        std::cout << "Distance mismatches: " << result.mismatches << " / " << pair_count << "\n";
    }
    std::cout << "Validation: " << (haversine::passed(result) ? "PASS" : "FAIL") << "\n";
    return haversine::passed(result) ? 0 : 1;
}

void print_time_elapsed(char const* label, uint64_t total_tsce_elapsed, uint64_t begin,
                        uint64_t end) {
    auto elapsed = end - begin;
    double percent = 100.0 * ((double)elapsed / (double)total_tsce_elapsed);
    std::cout << "  " << label << ": " << elapsed << " (" << std::fixed << std::setprecision(2)
              << percent << "%)\n";
}

int cmd_generate(std::span<char*> args, std::string_view program) {
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

int cmd_compute(std::span<char*> args, std::string_view program) {
    BEGIN_PROF();
    if (args.empty()) {
        print_compute_usage(program);
        return 1;
    }

    BEGIN_PROF_NAMED(read_file, "read_file");
    std::ifstream input{args[0]};
    if (!input.is_open()) {
        std::cerr << "error: could not open " << args[0] << "\n";
        return 1;
    }
    std::stringstream buf;
    buf << input.rdbuf();
    END_PROF(read_file);

    std::vector<haversine::Pair> pairs;
    const char* parse_error = parse_error_message(haversine::parse_input(buf.str(), pairs));
    if (parse_error != nullptr) {
        std::cerr << "error: " << parse_error << "\n";
        return 1;
    }

    if (args.size() < 2) {
        std::cout << std::setprecision(17);
        std::cout << "Pair count: " << pairs.size() << "\n";
        std::cout << "Sum: " << haversine::evaluate_pairs(pairs, nullptr).sum << "\n";
        return 0;
    }

    auto answers = haversine::read_answers(args[1]);
    if (!answers) {
        std::cerr << "error: could not read answers file " << args[1] << "\n";
        return 1;
    }

    auto rv = print_validation(pairs.size(), haversine::validate(pairs, *answers));
    return rv;
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
        const int result = cmd_compute(command_args, program);
        profiler::print_profile_report("cmd_compute");
        return result;
    }
    if (command == "generate") {
        return cmd_generate(command_args, program);
    }
    std::cerr << "error: invalid command '" << command << "'\n";
    return 1;
}
