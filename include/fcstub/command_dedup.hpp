#pragma once

// COMMAND_LONG retransmission handling. A client that misses the COMMAND_ACK
// resends the command with confirmation > 0; such a resend within one second of
// the first must get the original result again and must not run the command a
// second time.

#include "fcstub/mode_machine.hpp"
#include "fcstub/time.hpp"

#include <cstdint>
#include <optional>

namespace fcstub {

struct CommandKey {
    std::uint8_t source_system;
    std::uint8_t source_component;
    std::uint16_t command;

    bool operator==(const CommandKey& other) const noexcept {
        return source_system == other.source_system && source_component == other.source_component &&
               command == other.command;
    }
};

class CommandDedup {
public:
    static constexpr TimeNs kWindow = kNsPerS;

    // Cached result if this is a retransmission of the remembered command.
    std::optional<AckResult> lookup(const CommandKey& key, std::uint8_t confirmation,
                                    TimeNs now) const noexcept;

    void remember(const CommandKey& key, TimeNs now, AckResult result) noexcept;

private:
    bool valid_ = false;
    CommandKey key_{};
    TimeNs first_seen_ = 0;
    AckResult result_ = AckResult::Accepted;
};

}  // namespace fcstub
