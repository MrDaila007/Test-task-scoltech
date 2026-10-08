#pragma once

// The PX4 parameters the stub exposes over the MAVLink parameter protocol. Each
// one is a view of a configuration value, so `PARAM_SET` on a writable one
// changes the run (COM_OF_LOSS_T -> modes.offboard_timeout_ms). INT32 values go
// on the wire bytewise inside the float field, as PX4 sends them.

#include "fcstub/config.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace fcstub {

enum class Param : std::uint8_t {
    ComOfLossT,    // offboard-loss timeout, s (writable)
    ComOblRcAct,   // offboard-loss action: 5 = Hold (the only one the stub flies)
    MpcXyVelMax,   // speed limit, m/s
    Bat1Capacity,  // pack capacity, mAh
    Bat1NCells,    // cells in series
    MavSysId,      // MAVLink system id
};
inline constexpr std::size_t kParamCount = 6;

inline constexpr std::uint8_t kParamTypeInt32 = 6;   // MAV_PARAM_TYPE_INT32
inline constexpr std::uint8_t kParamTypeReal32 = 9;  // MAV_PARAM_TYPE_REAL32

using ParamName = std::array<char, 16>;  // MAVLink param_id: not NUL-terminated at 16

const char* param_name(Param p) noexcept;
std::uint8_t param_type(Param p) noexcept;

// Lookup by MAVLink id (up to 16 characters) or by index.
std::optional<Param> find_param(const ParamName& id) noexcept;
std::optional<Param> param_at(int index) noexcept;

// The value as it goes on the wire (float field; INT32 bytewise).
float param_wire_value(Param p, const Config& cfg) noexcept;

// Applies a PARAM_SET. False when the parameter is read-only, the type does not
// match or the value is out of range; `cfg` is then unchanged.
bool param_set(Param p, float wire_value, std::uint8_t type, Config& cfg) noexcept;

}  // namespace fcstub
