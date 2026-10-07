#include "yaml_section.hpp"

#include <arpa/inet.h>

namespace fcstub::detail {

std::string join_path(const std::string& base, const std::string& key) {
    return base.empty() ? key : base + "." + key;
}

std::string index_path(const std::string& base, std::size_t index) {
    return base + "[" + std::to_string(index) + "]";
}

Section::Section(YAML::Node node, std::string path)
    : node_(std::move(node)), path_(std::move(path)) {
    if (node_ && !node_.IsNull() && !node_.IsMap()) {
        throw ConfigError(path_, "expected a mapping");
    }
}

bool Section::has(const char* key) const {
    return node_ && node_.IsMap() && node_[key].IsDefined();
}

void Section::require(const char* key) const {
    if (!has(key)) {
        fail(key, "required key is missing");
    }
}

YAML::Node Section::take(const char* key) {
    consumed_.insert(key);
    if (!has(key)) {
        return YAML::Node(YAML::NodeType::Undefined);
    }
    return node_[key];
}

void Section::fail(const char* key, const std::string& reason) const {
    throw ConfigError(join_path(path_, key), reason);
}

void Section::boolean(const char* key, bool& out) {
    const YAML::Node n = take(key);
    if (!n) {
        return;
    }
    try {
        out = n.as<bool>();
    } catch (const YAML::Exception&) {
        fail(key, "expected true or false");
    }
}

void Section::text(const char* key, std::string& out) {
    const YAML::Node n = take(key);
    if (!n) {
        return;
    }
    if (!n.IsScalar()) {
        fail(key, "expected a string");
    }
    out = n.Scalar();
}

void Section::ipv4(const char* key, std::string& out) {
    std::string value = out;
    text(key, value);
    in_addr addr{};
    if (inet_pton(AF_INET, value.c_str(), &addr) != 1) {
        fail(key, "expected an IPv4 address, got '" + value + "'");
    }
    out = value;
}

void Section::vec3(const char* key, Vec3& out, double lo, double hi) {
    const YAML::Node n = take(key);
    if (!n) {
        return;
    }
    if (!n.IsSequence() || n.size() != 3) {
        fail(key, "expected a list of 3 numbers [n, e, d]");
    }
    Vec3 value{};
    for (std::size_t i = 0; i < 3; ++i) {
        try {
            value[i] = n[i].as<double>();
        } catch (const YAML::Exception&) {
            fail(key, "expected a list of 3 numbers [n, e, d]");
        }
        if (!std::isfinite(value[i]) || value[i] < lo || value[i] > hi) {
            fail(key, "element out of range [" + format_bound(lo) + ", " + format_bound(hi) + "]");
        }
    }
    out = value;
}

Section Section::child(const char* key) { return Section(take(key), join_path(path_, key)); }

std::vector<YAML::Node> Section::sequence(const char* key, std::size_t max_len) {
    const YAML::Node n = take(key);
    std::vector<YAML::Node> items;
    if (!n || n.IsNull()) {
        return items;
    }
    if (!n.IsSequence()) {
        fail(key, "expected a list");
    }
    if (n.size() > max_len) {
        fail(key, "at most " + std::to_string(max_len) + " entries allowed");
    }
    for (const auto& item : n) {
        items.push_back(item);
    }
    return items;
}

void Section::finish() const {
    if (!node_ || !node_.IsMap()) {
        return;
    }
    for (const auto& kv : node_) {
        const std::string key = kv.first.as<std::string>();
        if (consumed_.count(key) == 0) {
            throw ConfigError(join_path(path_, key), "unknown key (or not applicable here)");
        }
    }
}

}  // namespace fcstub::detail
