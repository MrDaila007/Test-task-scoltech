// End-to-end over real UDP and wall-clock time: a test client talks to the
// fc_stub process exactly like an onboard computer would.

#include "process.hpp"

#include <common/mavlink.h>
#include <gtest/gtest.h>
#include <poll.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <thread>

namespace {

using namespace fcstub::itest;
using Clock = std::chrono::steady_clock;

class UdpClient {
public:
    explicit UdpClient(std::uint16_t fc_port) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        socklen_t len = sizeof(local);
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) == 0 &&
            ::getsockname(fd_, reinterpret_cast<sockaddr*>(&local), &len) == 0) {
            port_ = ntohs(local.sin_port);
        }
        fc_.sin_family = AF_INET;
        fc_.sin_port = htons(fc_port);
        fc_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    }
    ~UdpClient() { ::close(fd_); }

    std::uint16_t port() const { return port_; }

    void send(const mavlink_message_t& msg) {
        std::uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const std::uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
        ::sendto(fd_, buf, n, 0, reinterpret_cast<const sockaddr*>(&fc_), sizeof(fc_));
    }

    void command(std::uint16_t cmd, float p1, float p2 = 0, float p3 = 0) {
        mavlink_message_t m{};
        mavlink_msg_command_long_pack_status(255, 190, &tx_, &m, 1, 1, cmd, 0, p1, p2, p3, 0, 0, 0,
                                             0);
        send(m);
    }

    void velocity(float vn) {
        mavlink_message_t m{};
        mavlink_msg_set_position_target_local_ned_pack_status(255, 190, &tx_, &m, 0, 1, 1,
                                                              MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0,
                                                              vn, 0, -1, 0, 0, 0, 0, 0);
        send(m);
    }

    // Receives until `pred` accepts a message or `timeout` passes.
    bool wait_for(std::chrono::milliseconds timeout,
                  const std::function<bool(const mavlink_message_t&)>& pred) {
        const auto until = Clock::now() + timeout;
        while (Clock::now() < until) {
            mavlink_message_t m{};
            if (receive(until, m) && pred(m)) {
                return true;
            }
        }
        return false;
    }

    // Counts messages of `msgid` over `window`.
    int count(std::uint32_t msgid, std::chrono::milliseconds window) {
        int n = 0;
        const auto until = Clock::now() + window;
        while (Clock::now() < until) {
            mavlink_message_t m{};
            if (receive(until, m) && m.msgid == msgid) {
                ++n;
            }
        }
        return n;
    }

private:
    bool receive(Clock::time_point until, mavlink_message_t& out) {
        const auto left =
            std::chrono::duration_cast<std::chrono::milliseconds>(until - Clock::now()).count();
        pollfd p{fd_, POLLIN, 0};
        if (left <= 0 || ::poll(&p, 1, static_cast<int>(left)) <= 0) {
            return false;
        }
        std::uint8_t buf[2048];
        const ssize_t n = ::recv(fd_, buf, sizeof(buf), 0);
        mavlink_message_t rx{};
        mavlink_status_t st{};
        mavlink_status_t out_st{};
        for (ssize_t i = 0; i < n; ++i) {
            if (mavlink_frame_char_buffer(&rx, &st, buf[i], &out, &out_st) == MAVLINK_FRAMING_OK) {
                return true;
            }
        }
        return false;
    }

    int fd_ = -1;
    std::uint16_t port_ = 0;
    sockaddr_in fc_{};
    mavlink_status_t tx_{};
};

bool is_ack(const mavlink_message_t& m, std::uint16_t cmd, std::uint8_t result) {
    return m.msgid == MAVLINK_MSG_ID_COMMAND_ACK && mavlink_msg_command_ack_get_command(&m) == cmd &&
           mavlink_msg_command_ack_get_result(&m) == result;
}

bool is_text(const mavlink_message_t& m, const char* needle) {
    if (m.msgid != MAVLINK_MSG_ID_STATUSTEXT) {
        return false;
    }
    char text[51] = {};
    mavlink_msg_statustext_get_text(&m, text);
    return std::strstr(text, needle) != nullptr;
}

}  // namespace

TEST(UdpE2E, ArmOffboardAndHoldOnSetpointLoss) {
    const std::uint16_t fc_port = free_udp_port();
    ASSERT_NE(fc_port, 0);
    UdpClient client(fc_port);
    ASSERT_NE(client.port(), 0);
    const auto cfg = temp_file(
        "e2e.yaml", "schema_version: 1\nrun: {duration_s: 15}\nlink: {bind_addr: 127.0.0.1, "
                    "bind_port: " + std::to_string(fc_port) + ", remote_port: " +
                        std::to_string(client.port()) + "}\nrealtime: {sched_fifo: false}\n");
    Child fc({"--config", cfg.string()});

    // Telemetry rate on the receiving side over 3 s.
    ASSERT_TRUE(client.wait_for(std::chrono::seconds(3), [](const auto& m) {
        return m.msgid == MAVLINK_MSG_ID_HEARTBEAT;
    }));
    const int attitude = client.count(MAVLINK_MSG_ID_ATTITUDE, std::chrono::seconds(3));
    EXPECT_NEAR(attitude, 150, 3);

    // Ready by now (2 s after boot): arm.
    client.command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
    ASSERT_TRUE(client.wait_for(std::chrono::seconds(1), [](const auto& m) {
        return is_ack(m, MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED);
    }));

    // Stream setpoints, switch to OFFBOARD, keep streaming for a second.
    for (int i = 0; i < 5; ++i) {
        client.velocity(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    client.command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6);
    bool offboard_acked = false;
    for (int i = 0; i < 20; ++i) {
        client.velocity(1);
        offboard_acked = offboard_acked ||
                         client.wait_for(std::chrono::milliseconds(50), [](const auto& m) {
                             return is_ack(m, MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED);
                         });
    }
    ASSERT_TRUE(offboard_acked);

    // Stop the stream: HOLD must follow just after 500 ms.
    client.velocity(1);
    const auto last_setpoint = Clock::now();
    ASSERT_TRUE(client.wait_for(std::chrono::seconds(2), [](const auto& m) {
        return is_text(m, "Offboard lost >500ms: HOLD");
    }));
    const auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - last_setpoint);
    EXPECT_GE(delay.count(), 500);
    EXPECT_LE(delay.count(), 650);
    EXPECT_TRUE(client.wait_for(std::chrono::seconds(2), [](const auto& m) {
        return m.msgid == MAVLINK_MSG_ID_HEARTBEAT &&
               mavlink_msg_heartbeat_get_custom_mode(&m) == ((4U << 16U) | (3U << 24U));
    }));
    EXPECT_EQ(fc.terminate(), 0);
}
