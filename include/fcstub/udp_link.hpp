#pragma once

// Non-blocking UDP socket bound to the local address, sending every datagram to
// one configured remote address (like PX4 SITL's offboard link). RAII, move-only.

#include "fcstub/config.hpp"

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace fcstub {

class UdpLink {
public:
    // Opens and binds; on failure returns an invalid link and fills `error`.
    static UdpLink open(const LinkConfig& cfg, std::string& error);

    UdpLink(UdpLink&& other) noexcept;
    UdpLink& operator=(UdpLink&& other) noexcept;
    UdpLink(const UdpLink&) = delete;
    UdpLink& operator=(const UdpLink&) = delete;
    ~UdpLink();

    bool valid() const noexcept { return fd_ >= 0; }
    int fd() const noexcept { return fd_; }

    // False on any error (EAGAIN/ENOBUFS included); the caller counts it.
    bool send(const std::uint8_t* data, std::size_t len) noexcept;

    // Bytes received, 0 if nothing is pending, -1 on error.
    long receive(std::uint8_t* buf, std::size_t cap) noexcept;

private:
    UdpLink() = default;

    int fd_ = -1;
    sockaddr_in remote_{};
};

}  // namespace fcstub
