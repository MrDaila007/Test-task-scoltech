#pragma once

// Strict YAML loader for Config (schema version 1).
//
// This is the only part of the program that throws: it runs once at start-up.
// Every rejection carries the YAML key path, e.g. "faults[2].delay_ms".

#include "fcstub/config.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>

namespace fcstub {

class ConfigError : public std::runtime_error {
public:
    ConfigError(std::string key_path, const std::string& reason);

    // Dotted path of the offending key; empty for document-level problems.
    const std::string& key_path() const noexcept { return key_path_; }

private:
    std::string key_path_;
};

Config load_config(const std::string& path);
Config load_config_from_string(const std::string& yaml);

// Cross-field checks (sim needs a duration, fault windows of one type must not
// overlap). Called by the loaders; call again after CLI overrides.
void validate_config(const Config& cfg);

// Number of scalar leaves in the document, counting every list element.
std::size_t count_leaf_params(const std::string& path);
std::size_t count_leaf_params_in_string(const std::string& yaml);

}  // namespace fcstub
