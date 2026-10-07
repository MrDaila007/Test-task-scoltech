#pragma once

#include "fcstub/config.hpp"
#include "yaml_section.hpp"

#include <vector>

namespace fcstub::detail {

std::vector<FaultSpec> parse_faults(Section& root);
std::vector<ClientAction> parse_client(Section& root);

}  // namespace fcstub::detail
