#include <table.hpp>

std::ostream& operator<<(std::ostream& os, const Op& op) {
    static constexpr std::array ops = {"mov"};
    if (auto i = static_cast<std::size_t>(op); i < ops.size()) {
        return os << ops.at(i);
    }
    return os << "unknown";
}

