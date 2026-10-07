#include "fcstub/command_dedup.hpp"

namespace fcstub {

std::optional<AckResult> CommandDedup::lookup(const CommandKey& key, std::uint8_t confirmation,
                                              TimeNs now) const noexcept {
    if (!valid_ || confirmation == 0 || !(key == key_) || now - first_seen_ > kWindow) {
        return std::nullopt;
    }
    return result_;
}

void CommandDedup::remember(const CommandKey& key, TimeNs now, AckResult result) noexcept {
    valid_ = true;
    key_ = key;
    first_seen_ = now;
    result_ = result;
}

}  // namespace fcstub
