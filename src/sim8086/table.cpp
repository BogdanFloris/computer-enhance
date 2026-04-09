#include <table.hpp>

std::ostream& operator<<(std::ostream& os, const Op& op) {
    static constexpr std::array ops = {
        "mov", "add", "sub", "cmp",  "jnz",   "je",     "jl",   "jle", "jb",
        "jbe", "jp",  "jo",  "js",   "jne",   "jnl",    "jg",   "jnb", "ja",
        "jnp", "jno", "jns", "loop", "loopz", "loopnz", "jcxz",
    };
    if (auto i = static_cast<std::size_t>(op); i < ops.size()) {
        return os << ops.at(i);
    }
    return os << "unknown";
}
