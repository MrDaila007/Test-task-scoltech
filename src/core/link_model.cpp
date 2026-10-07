#include "fcstub/link_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace fcstub {

namespace {

// Min-heap on (release, order): std heap algorithms build a max-heap, so invert.
struct Later {
    template <typename E>
    bool operator()(const E& a, const E& b) const noexcept {
        return a.release != b.release ? a.release > b.release : a.order > b.order;
    }
};

}  // namespace

LinkModel::LinkModel(std::size_t capacity, std::size_t max_len)
    : capacity_(std::clamp<std::size_t>(capacity, 1, 65535)),
      max_len_(std::min(max_len, kMaxLinkPacket)),
      storage_(capacity_ * max_len_),
      slot_len_(capacity_) {
    free_slots_.reserve(capacity_);
    for (std::size_t i = capacity_; i > 0; --i) {
        free_slots_.push_back(static_cast<std::uint16_t>(i - 1));
    }
    heap_.reserve(capacity_);
}

bool LinkModel::apply_fault(TimeNs now, const LinkFaultParams& f, Rng& rng,
                            TimeNs& release) noexcept {
    if (bad_state_) {
        bad_state_ = !rng.bernoulli(f.p_bad_to_good);
    } else {
        bad_state_ = rng.bernoulli(f.p_good_to_bad);
    }
    if (rng.bernoulli(bad_state_ ? f.loss_bad : f.loss_good)) {
        ++stats_.lost;
        return false;
    }
    const double delay_ms = std::max(0.0, f.delay_ms + (2.0 * rng.uniform01() - 1.0) * f.jitter_ms);
    release = now + static_cast<TimeNs>(std::llround(delay_ms * static_cast<double>(kNsPerMs)));
    return true;
}

void LinkModel::drop_first() noexcept {
    std::pop_heap(heap_.begin(), heap_.end(), Later{});
    free_slots_.push_back(heap_.back().slot);
    heap_.pop_back();
    ++stats_.overflow;
}

std::uint16_t LinkModel::acquire_slot() noexcept {
    if (free_slots_.empty()) {
        drop_first();
    }
    const std::uint16_t slot = free_slots_.back();
    free_slots_.pop_back();
    return slot;
}

void LinkModel::push(TimeNs now, const std::uint8_t* data, std::size_t len,
                     const LinkFaultParams* fault, Rng* rng) noexcept {
    if (len > max_len_) {
        ++stats_.oversize;
        return;
    }
    ++stats_.pushed;
    TimeNs release = now;
    if (fault == nullptr || rng == nullptr) {
        bad_state_ = false;
    } else if (!apply_fault(now, *fault, *rng, release)) {
        return;
    }
    const std::uint16_t slot = acquire_slot();
    std::uint8_t* dst = slot_data(slot);
    std::memcpy(dst, data, len);
    slot_len_[slot] = static_cast<std::uint16_t>(len);
    if (fault != nullptr && rng != nullptr && fault->byte_error_rate > 0.0) {
        bool flipped = false;
        for (std::size_t i = 0; i < len; ++i) {
            if (rng->bernoulli(fault->byte_error_rate)) {
                dst[i] = static_cast<std::uint8_t>(dst[i] ^ (1U << (rng->next_u64() % 8U)));
                flipped = true;
            }
        }
        stats_.corrupted += flipped ? 1U : 0U;
    }
    heap_.push_back(Entry{release, next_order_++, slot});
    std::push_heap(heap_.begin(), heap_.end(), Later{});
}

bool LinkModel::pop_due(TimeNs now, LinkPacket& out) noexcept {
    if (heap_.empty() || heap_.front().release > now) {
        return false;
    }
    std::pop_heap(heap_.begin(), heap_.end(), Later{});
    const Entry e = heap_.back();
    heap_.pop_back();
    out.release = e.release;
    out.len = slot_len_[e.slot];
    std::memcpy(out.data.data(), slot_data(e.slot), out.len);
    std::memset(out.data.data() + out.len, 0, max_len_ - out.len);
    free_slots_.push_back(e.slot);
    ++stats_.delivered;
    return true;
}

TimeNs LinkModel::next_release() const noexcept {
    return heap_.empty() ? kNever : heap_.front().release;
}

}  // namespace fcstub
