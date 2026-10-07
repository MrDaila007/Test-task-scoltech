#pragma once

// One direction of the autopilot's serial/network link, with the link fault (F4).
//
// Without a fault every packet is released at the time it was pushed. With a
// fault each packet goes through, in this fixed order of random draws:
//   1. Gilbert-Elliott state change (good -> bad with p_good_to_bad, bad -> good
//      with p_bad_to_good), then loss with loss_good / loss_bad: bursty losses;
//   2. delay_ms + uniform(-jitter_ms, +jitter_ms), clamped at 0; overlapping
//      delays reorder packets, as a congested USB/UART bridge does;
//   3. independent bit flips per byte with byte_error_rate, applied after the
//      CRC was computed, so the receiver sees CRC errors.
// The queue has a fixed capacity; when full, the packet due first is dropped
// (a full buffer loses its oldest content). No allocation after construction.

#include "fcstub/config.hpp"
#include "fcstub/rng.hpp"
#include "fcstub/time.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace fcstub {

inline constexpr std::size_t kMaxLinkPacket = 2048;

struct LinkPacket {
    TimeNs release = 0;
    std::uint16_t len = 0;
    std::array<std::uint8_t, kMaxLinkPacket> data{};
};

struct LinkStats {
    std::uint64_t pushed = 0;
    std::uint64_t delivered = 0;
    std::uint64_t lost = 0;       // dropped by the loss model
    std::uint64_t overflow = 0;   // dropped because the queue was full
    std::uint64_t oversize = 0;   // longer than the link accepts
    std::uint64_t corrupted = 0;  // delivered with at least one flipped bit
};

class LinkModel {
public:
    static constexpr TimeNs kNever = std::numeric_limits<TimeNs>::max();

    // Precondition: 0 < capacity <= 65535, max_len <= kMaxLinkPacket.
    LinkModel(std::size_t capacity, std::size_t max_len);

    // `fault` and `rng` may be null for a clean link; `rng` must be the fault's stream.
    void push(TimeNs now, const std::uint8_t* data, std::size_t len, const LinkFaultParams* fault,
              Rng* rng) noexcept;

    // Next packet released at or before `now`, in (release time, push order).
    bool pop_due(TimeNs now, LinkPacket& out) noexcept;

    TimeNs next_release() const noexcept;
    std::size_t queued() const noexcept { return heap_.size(); }
    const LinkStats& stats() const noexcept { return stats_; }

private:
    struct Entry {
        TimeNs release;
        std::uint64_t order;
        std::uint16_t slot;
    };

    bool apply_fault(TimeNs now, const LinkFaultParams& f, Rng& rng, TimeNs& release) noexcept;
    std::uint16_t acquire_slot() noexcept;
    void drop_first() noexcept;
    std::uint8_t* slot_data(std::uint16_t slot) noexcept { return &storage_[slot * max_len_]; }

    std::size_t capacity_;
    std::size_t max_len_;
    std::vector<std::uint8_t> storage_;
    std::vector<std::uint16_t> slot_len_;
    std::vector<std::uint16_t> free_slots_;
    std::vector<Entry> heap_;
    std::uint64_t next_order_ = 0;
    bool bad_state_ = false;
    LinkStats stats_{};
};

}  // namespace fcstub
