#pragma once

// Path-aware, strict reader over one YAML mapping. Every read records the key as
// consumed; finish() rejects whatever was not consumed, which gives "unknown key"
// errors and also rejects keys that do not apply to the selected variant.

#include "fcstub/config.hpp"
#include "fcstub/config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace fcstub::detail {

std::string join_path(const std::string& base, const std::string& key);
std::string index_path(const std::string& base, std::size_t index);

// Shortest readable form of a bound: "250" rather than "250.000000"; unary + keeps
// uint8_t from printing as a character.
template <typename T>
std::string format_bound(T value) {
    std::ostringstream os;
    os << +value;
    return os.str();
}

template <typename E>
using EnumNames = std::vector<std::pair<const char*, E>>;

class Section {
public:
    Section(YAML::Node node, std::string path);

    const std::string& path() const noexcept { return path_; }
    bool has(const char* key) const;
    void require(const char* key) const;

    template <typename T>
    void number(const char* key, T& out, T lo, T hi);
    void boolean(const char* key, bool& out);
    void text(const char* key, std::string& out);
    void ipv4(const char* key, std::string& out);
    void vec3(const char* key, Vec3& out, double lo, double hi);
    template <typename E>
    void enumeration(const char* key, E& out, const EnumNames<E>& names);

    // Nested mapping; an absent key yields an empty section.
    Section child(const char* key);
    // Sequence elements; an absent key yields no elements.
    std::vector<YAML::Node> sequence(const char* key, std::size_t max_len);

    void finish() const;

private:
    YAML::Node take(const char* key);
    [[noreturn]] void fail(const char* key, const std::string& reason) const;

    YAML::Node node_;
    std::string path_;
    std::set<std::string> consumed_;
};

template <typename T>
void Section::number(const char* key, T& out, T lo, T hi) {
    const YAML::Node n = take(key);
    if (!n) {
        return;
    }
    using Wide = std::conditional_t<std::is_floating_point_v<T>, double,
                                    std::conditional_t<std::is_signed_v<T>, long long,
                                                       unsigned long long>>;
    Wide value{};
    try {
        value = n.as<Wide>();
    } catch (const YAML::Exception&) {
        fail(key, std::is_integral_v<T> ? "expected an integer" : "expected a number");
    }
    if constexpr (std::is_floating_point_v<T>) {
        if (!std::isfinite(value)) {
            fail(key, "must be finite");
        }
    }
    if (value < static_cast<Wide>(lo) || value > static_cast<Wide>(hi)) {
        fail(key, "out of range [" + format_bound(lo) + ", " + format_bound(hi) + "]");
    }
    out = static_cast<T>(value);
}

template <typename E>
void Section::enumeration(const char* key, E& out, const EnumNames<E>& names) {
    std::string value;
    text(key, value);
    if (!has(key)) {
        return;
    }
    for (const auto& [name, e] : names) {
        if (value == name) {
            out = e;
            return;
        }
    }
    std::string allowed;
    for (const auto& entry : names) {
        allowed += allowed.empty() ? entry.first : std::string("|") + entry.first;
    }
    fail(key, "unknown value '" + value + "', expected " + allowed);
}

}  // namespace fcstub::detail
