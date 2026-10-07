#include "fcstub/udp_link.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace fcstub {

namespace {

bool make_addr(const std::string& ip, std::uint16_t port, sockaddr_in& out) {
    out = sockaddr_in{};
    out.sin_family = AF_INET;
    out.sin_port = htons(port);
    return inet_pton(AF_INET, ip.c_str(), &out.sin_addr) == 1;
}

}  // namespace

UdpLink UdpLink::open(const LinkConfig& cfg, std::string& error) {
    UdpLink link;
    sockaddr_in local{};
    if (!make_addr(cfg.bind_addr, cfg.bind_port, local) ||
        !make_addr(cfg.remote_addr, cfg.remote_port, link.remote_)) {
        error = "invalid address";
        return link;
    }
    const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        error = std::string("socket: ") + std::strerror(errno);
        return link;
    }
    if (::bind(fd, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0) {
        error = "bind " + cfg.bind_addr + ":" + std::to_string(cfg.bind_port) + ": " +
                std::strerror(errno);
        ::close(fd);
        return link;
    }
    link.fd_ = fd;
    return link;
}

UdpLink::UdpLink(UdpLink&& other) noexcept
    : fd_(std::exchange(other.fd_, -1)), remote_(other.remote_) {}

UdpLink& UdpLink::operator=(UdpLink&& other) noexcept {
    if (this != &other) {
        if (fd_ >= 0) {
            ::close(fd_);
        }
        fd_ = std::exchange(other.fd_, -1);
        remote_ = other.remote_;
    }
    return *this;
}

UdpLink::~UdpLink() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

bool UdpLink::send(const std::uint8_t* data, std::size_t len) noexcept {
    const ssize_t n =
        ::sendto(fd_, data, len, 0, reinterpret_cast<const sockaddr*>(&remote_), sizeof(remote_));
    return n == static_cast<ssize_t>(len);
}

long UdpLink::receive(std::uint8_t* buf, std::size_t cap) noexcept {
    const ssize_t n = ::recv(fd_, buf, cap, MSG_TRUNC);
    if (n < 0) {
        return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
    }
    // MSG_TRUNC reports the real datagram size; anything above cap was cut off and
    // must not reach the parser as if it were complete.
    return static_cast<std::size_t>(n) > cap ? static_cast<long>(cap) + 1 : static_cast<long>(n);
}

}  // namespace fcstub
