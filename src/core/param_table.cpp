#include "fcstub/param_table.hpp"

#include <cmath>
#include <cstring>

namespace fcstub {

namespace {

constexpr std::int32_t kOblActionHold = 5;  // COM_OBL_RC_ACT: Hold

struct ParamInfo {
    const char* name;
    std::uint8_t type;
};

constexpr std::array<ParamInfo, kParamCount> kParams = {{
    {"COM_OF_LOSS_T", kParamTypeReal32},
    {"COM_OBL_RC_ACT", kParamTypeInt32},
    {"MPC_XY_VEL_MAX", kParamTypeReal32},
    {"BAT1_CAPACITY", kParamTypeReal32},
    {"BAT1_N_CELLS", kParamTypeInt32},
    {"MAV_SYS_ID", kParamTypeInt32},
}};

const ParamInfo& info(Param p) noexcept { return kParams[static_cast<std::size_t>(p)]; }

float int_on_wire(std::int32_t value) noexcept {
    float wire = 0.0F;
    std::memcpy(&wire, &value, sizeof(wire));
    return wire;
}

}  // namespace

const char* param_name(Param p) noexcept { return info(p).name; }

std::uint8_t param_type(Param p) noexcept { return info(p).type; }

std::optional<Param> find_param(const ParamName& id) noexcept {
    for (std::size_t i = 0; i < kParamCount; ++i) {
        if (std::strncmp(kParams[i].name, id.data(), id.size()) == 0) {
            return static_cast<Param>(i);
        }
    }
    return std::nullopt;
}

std::optional<Param> param_at(int index) noexcept {
    if (index < 0 || index >= static_cast<int>(kParamCount)) {
        return std::nullopt;
    }
    return static_cast<Param>(index);
}

float param_wire_value(Param p, const Config& cfg) noexcept {
    switch (p) {
        case Param::ComOfLossT:
            return static_cast<float>(cfg.modes.offboard_timeout_ms / 1000.0);
        case Param::ComOblRcAct:
            return int_on_wire(kOblActionHold);
        case Param::MpcXyVelMax:
            return static_cast<float>(cfg.vehicle.v_max_mps);
        case Param::Bat1Capacity:
            return static_cast<float>(cfg.battery.capacity_mah);
        case Param::Bat1NCells:
            return int_on_wire(cfg.battery.cells);
        case Param::MavSysId:
            return int_on_wire(cfg.identity.system_id);
    }
    return 0.0F;
}

bool param_set(Param p, float wire_value, std::uint8_t type, Config& cfg) noexcept {
    // Only the offboard-loss timeout is writable: it is the knob an onboard
    // computer legitimately sets before flight. Range as modes.offboard_timeout_ms.
    if (p != Param::ComOfLossT || type != kParamTypeReal32 || !std::isfinite(wire_value) ||
        wire_value < 0.05F || wire_value > 10.0F) {
        return false;
    }
    cfg.modes.offboard_timeout_ms = static_cast<int>(std::lround(wire_value * 1000.0));
    return true;
}

}  // namespace fcstub
