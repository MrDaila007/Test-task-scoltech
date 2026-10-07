#pragma once

// Scripted onboard computer for sim mode: turns the `client` section of the
// scenario into MAVLink traffic at exact model times (commands, setpoint streams,
// TIMESYNC requests). It identifies itself like MAVSDK (sysid 255, compid 190)
// and goes through the uplink like any real client, so uplink faults apply.

#include "fcstub/config.hpp"
#include "fcstub/mav_codec.hpp"
#include "fcstub/time.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace fcstub {

class ClientSink {
public:
    virtual void send(TimeNs t, const std::uint8_t* data, std::size_t len) noexcept = 0;

protected:
    ~ClientSink() = default;
};

class VirtualClient {
public:
    static constexpr TimeNs kNever = std::numeric_limits<TimeNs>::max();
    static constexpr std::uint8_t kSystemId = 255;
    static constexpr std::uint8_t kComponentId = 190;

    explicit VirtualClient(const std::vector<ClientAction>& actions,
                           std::uint8_t target_system = 1, std::uint8_t target_component = 1);

    TimeNs next_event() const noexcept;

    // Sends everything due at exactly `t`, in configuration order.
    void emit(TimeNs t, ClientSink& sink) noexcept;

private:
    struct Track {
        ClientAction action;
        TimeNs next;
        TimeNs end;     // exclusive; commands fire once
        TimeNs period;  // 0 for commands
        std::uint64_t count = 0;
    };

    FrameBuf encode(const ClientAction& a, TimeNs t) noexcept;

    std::vector<Track> tracks_;
    MavClientEncoder encoder_;
};

}  // namespace fcstub
