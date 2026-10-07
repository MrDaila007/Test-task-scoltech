#pragma once

// Streaming SHA-256 (FIPS 180-4) for the reproducibility hash.
// Self-contained to avoid a crypto dependency; verified against NIST vectors.

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace fcstub {

class Sha256 {
public:
    using Digest = std::array<std::uint8_t, 32>;

    Sha256() noexcept;

    void update(const std::uint8_t* data, std::size_t len) noexcept;

    // Finalizes and returns the digest; the object must not be updated afterwards.
    Digest finish() noexcept;

    static std::string to_hex(const Digest& digest);

private:
    void compress(const std::uint8_t* block) noexcept;

    std::array<std::uint32_t, 8> state_;
    std::array<std::uint8_t, 64> buffer_{};
    std::size_t buffered_ = 0;
    std::uint64_t total_bytes_ = 0;
};

}  // namespace fcstub
