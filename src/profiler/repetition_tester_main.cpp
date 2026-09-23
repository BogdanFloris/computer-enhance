#include "profiler.hpp"
#include "repetition_tester.hpp"

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

constexpr uint64_t seconds_without_improvement = 10;

namespace {

enum AllocType : uint8_t { none, malloc };

const char* alloc_to_str(AllocType alloc_type) {
    switch (alloc_type) {
    case none:
        return "";
    case malloc:
        return " + malloc";
    }
    return "";
}

bool handle_allocation(AllocType alloc_type, profiler::Buffer& buffer, size_t byte_count) {
    switch (alloc_type) {
    case none:
        return true;
    case malloc:
        buffer = profiler::allocate_buffer(byte_count);
        return buffer.data != nullptr;
    }
    return false;
}

void handle_deallocation(AllocType alloc_type, profiler::Buffer buffer) {
    switch (alloc_type) {
    case none:
        return;
    case malloc:
        std::free(buffer.data);
        return;
    }
}

bool test_fread(const char* path, profiler::Buffer buffer, uint64_t timeout,
                uint64_t cpu_timer_freq, AllocType alloc_type) {
    std::FILE* file = std::fopen(path, "rb");
    if (file == nullptr) {
        std::cerr << "error: could not open " << path << "\n";
        return false;
    }

    profiler::RepTesterResult result{};
    bool succeeded = profiler::run_until_stable(result, timeout, [&]() -> std::optional<uint64_t> {
        if (std::fseek(file, 0, SEEK_SET) != 0) {
            std::cerr << "error: fread seek failed\n";
            return std::nullopt;
        }
        if (!handle_allocation(alloc_type, buffer, buffer.size)) {
            return std::nullopt;
        }

        uint64_t start = profiler::read_cpu_timer();
        size_t bytes_read = std::fread(buffer.data, 1, buffer.size, file);
        uint64_t elapsed = profiler::read_cpu_timer() - start;
        handle_deallocation(alloc_type, buffer);

        if (bytes_read != buffer.size) {
            std::cerr << "error: fread returned " << bytes_read << " of " << buffer.size
                      << " bytes\n";
            return std::nullopt;
        }
        return elapsed;
    });

    std::fclose(file);
    if (succeeded) {
        profiler::print_result(result, std::format("fread{}", alloc_to_str(alloc_type)),
                               cpu_timer_freq, buffer.size);
    }
    return succeeded;
}

bool test_read(const char* path, profiler::Buffer buffer, uint64_t timeout, uint64_t cpu_timer_freq,
               AllocType alloc_type) {
    int file = open(path, O_RDONLY);
    if (file == -1) {
        std::cerr << "error: could not open " << path << "\n";
        return false;
    }

    profiler::RepTesterResult result{};
    bool succeeded = profiler::run_until_stable(result, timeout, [&]() -> std::optional<uint64_t> {
        if (lseek(file, 0, SEEK_SET) == -1) {
            std::cerr << "error: read seek failed\n";
            return std::nullopt;
        }
        if (!handle_allocation(alloc_type, buffer, buffer.size)) {
            return std::nullopt;
        }

        uint64_t start = profiler::read_cpu_timer();
        size_t bytes_read = 0;
        while (bytes_read < buffer.size) {
            ssize_t read_size = read(file, buffer.data + bytes_read, buffer.size - bytes_read);
            if (read_size <= 0) {
                break;
            }
            bytes_read += static_cast<size_t>(read_size);
        }
        uint64_t elapsed = profiler::read_cpu_timer() - start;
        handle_deallocation(alloc_type, buffer);

        if (bytes_read != buffer.size) {
            std::cerr << "error: read returned " << bytes_read << " of " << buffer.size
                      << " bytes\n";
            return std::nullopt;
        }
        return elapsed;
    });

    close(file);
    if (succeeded) {
        profiler::print_result(result, std::format("read{}", alloc_to_str(alloc_type)),
                               cpu_timer_freq, buffer.size);
    }
    return succeeded;
}

bool test_ifstream_read(const char* path, profiler::Buffer buffer, uint64_t timeout,
                        uint64_t cpu_timer_freq, AllocType alloc_type) {
    std::ifstream input{path, std::ios::binary};
    if (!input.is_open()) {
        std::cerr << "error: could not open " << path << "\n";
        return false;
    }

    profiler::RepTesterResult result{};
    bool succeeded = profiler::run_until_stable(result, timeout, [&]() -> std::optional<uint64_t> {
        input.clear();
        input.seekg(0, std::ios::beg);
        if (!input) {
            std::cerr << "error: ifstream seek failed\n";
            return std::nullopt;
        }
        if (!handle_allocation(alloc_type, buffer, buffer.size)) {
            return std::nullopt;
        }

        uint64_t start = profiler::read_cpu_timer();
        input.read(reinterpret_cast<char*>(buffer.data), static_cast<std::streamsize>(buffer.size));
        uint64_t elapsed = profiler::read_cpu_timer() - start;
        handle_deallocation(alloc_type, buffer);

        if (static_cast<size_t>(input.gcount()) != buffer.size) {
            std::cerr << "error: ifstream returned " << input.gcount() << " of " << buffer.size
                      << " bytes\n";
            return std::nullopt;
        }
        return elapsed;
    });

    if (succeeded) {
        profiler::print_result(result, std::format("ifstream::read{}", alloc_to_str(alloc_type)),
                               cpu_timer_freq, buffer.size);
    }
    return succeeded;
}

bool test_mmap_copy(const char* path, profiler::Buffer buffer, uint64_t timeout,
                    uint64_t cpu_timer_freq, AllocType alloc_type) {
    int file = open(path, O_RDONLY);
    if (file == -1) {
        std::cerr << "error: could not open " << path << "\n";
        return false;
    }

    const size_t byte_count = buffer.size;
    void* mapped = mmap(nullptr, byte_count, PROT_READ, MAP_PRIVATE, file, 0);
    if (mapped == MAP_FAILED) {
        std::cerr << "error: could not mmap " << path << "\n";
        close(file);
        return false;
    }

    profiler::RepTesterResult result{};
    bool succeeded = profiler::run_until_stable(result, timeout, [&]() -> std::optional<uint64_t> {
        if (!handle_allocation(alloc_type, buffer, byte_count)) {
            return std::nullopt;
        }

        uint64_t start = profiler::read_cpu_timer();
        std::memcpy(buffer.data, mapped, buffer.size);
        // Keep the optimizer from discarding a copy whose result is only benchmark output.
        asm volatile("" : : "r"(buffer.data) : "memory");
        uint64_t elapsed = profiler::read_cpu_timer() - start;
        handle_deallocation(alloc_type, buffer);
        return elapsed;
    });

    munmap(mapped, byte_count);
    close(file);
    if (succeeded) {
        profiler::print_result(result, std::format("mmap + memcpy{}", alloc_to_str(alloc_type)),
                               cpu_timer_freq, byte_count);
    }
    return succeeded;
}

bool test_rdbuf_str(const char* path, size_t byte_count, uint64_t timeout,
                    uint64_t cpu_timer_freq) {
    std::ifstream input{path, std::ios::binary};
    if (!input.is_open()) {
        std::cerr << "error: could not open " << path << "\n";
        return false;
    }

    profiler::RepTesterResult result{};
    bool succeeded = profiler::run_until_stable(result, timeout, [&]() -> std::optional<uint64_t> {
        input.clear();
        input.seekg(0, std::ios::beg);
        if (!input) {
            std::cerr << "error: rdbuf seek failed\n";
            return std::nullopt;
        }

        std::stringstream buffer;
        uint64_t start = profiler::read_cpu_timer();
        buffer << input.rdbuf();
        std::string contents = buffer.str();
        uint64_t elapsed = profiler::read_cpu_timer() - start;

        if (contents.size() != byte_count) {
            std::cerr << "error: rdbuf returned " << contents.size() << " of " << byte_count
                      << " bytes\n";
            return std::nullopt;
        }
        return elapsed;
    });

    if (succeeded) {
        profiler::print_result(result, "rdbuf + str", cpu_timer_freq, byte_count);
    }
    return succeeded;
}

} // namespace

int main(int argc, char* argv[]) {
    std::span<char*> args{argv, static_cast<size_t>(argc)};

    if (args.size() < 2) {
        std::cerr << "error: file not found\n";
        return 1;
    }
    std::error_code size_ec;
    uintmax_t byte_count = std::filesystem::file_size(args[1], size_ec);
    if (size_ec) {
        std::cerr << "error: could not get size of " << args[1] << ": " << size_ec.message()
                  << "\n";
        return 1;
    }

    profiler::Buffer buffer = profiler::allocate_buffer(byte_count);
    if (buffer.data == nullptr) {
        return 1;
    }

    const uint64_t cpu_timer_freq = profiler::estimate_cpu_timer_freq();
    const uint64_t timeout = seconds_without_improvement * cpu_timer_freq;
    bool succeeded = test_fread(args[1], buffer, timeout, cpu_timer_freq, AllocType::none);
    succeeded =
        test_fread(args[1], buffer, timeout, cpu_timer_freq, AllocType::malloc) && succeeded;
    succeeded = test_read(args[1], buffer, timeout, cpu_timer_freq, AllocType::none) && succeeded;
    succeeded = test_read(args[1], buffer, timeout, cpu_timer_freq, AllocType::malloc) && succeeded;
    succeeded =
        test_ifstream_read(args[1], buffer, timeout, cpu_timer_freq, AllocType::none) && succeeded;
    succeeded = test_ifstream_read(args[1], buffer, timeout, cpu_timer_freq, AllocType::malloc) &&
                succeeded;
    succeeded =
        test_mmap_copy(args[1], buffer, timeout, cpu_timer_freq, AllocType::none) && succeeded;
    succeeded =
        test_mmap_copy(args[1], buffer, timeout, cpu_timer_freq, AllocType::malloc) && succeeded;
    succeeded = test_rdbuf_str(args[1], byte_count, timeout, cpu_timer_freq) && succeeded;

    std::free(buffer.data);
    return succeeded ? 0 : 1;
}
