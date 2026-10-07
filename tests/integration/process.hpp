#pragma once

// Helpers for running the real fc_stub binary from integration tests.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fcstub::itest {

inline const std::string kBinary = FCSTUB_BIN;

struct RunResult {
    int exit_code = -1;
    std::string out;
};

// Runs fc_stub with `args` (shell-quoted by the caller), capturing stdout.
inline RunResult run(const std::string& args) {
    RunResult r;
    FILE* p = ::popen((kBinary + " " + args + " 2>/dev/null").c_str(), "r");
    if (p == nullptr) {
        return r;
    }
    std::array<char, 256> buf{};
    while (std::fgets(buf.data(), static_cast<int>(buf.size()), p) != nullptr) {
        r.out += buf.data();
    }
    const int status = ::pclose(p);
    r.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    return r;
}

// Background fc_stub process; killed on destruction if still running.
class Child {
public:
    explicit Child(const std::vector<std::string>& args) {
        std::vector<char*> argv;
        argv.push_back(const_cast<char*>(kBinary.c_str()));
        for (const auto& a : args) {
            argv.push_back(const_cast<char*>(a.c_str()));
        }
        argv.push_back(nullptr);
        pid_ = ::fork();
        if (pid_ == 0) {
            ::execv(kBinary.c_str(), argv.data());
            ::_exit(127);
        }
    }
    Child(const Child&) = delete;
    Child& operator=(const Child&) = delete;
    ~Child() {
        if (pid_ > 0) {
            ::kill(pid_, SIGKILL);
            ::waitpid(pid_, nullptr, 0);
        }
    }

    // Sends SIGTERM and returns the exit code (-1 if killed by a signal).
    int terminate() {
        ::kill(pid_, SIGTERM);
        return wait();
    }

    int wait() {
        int status = 0;
        ::waitpid(pid_, &status, 0);
        pid_ = -1;
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    }

private:
    pid_t pid_ = -1;
};

// A free UDP port on 127.0.0.1 (bound and released; small race, fine for tests).
inline std::uint16_t free_udp_port() {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t len = sizeof(a);
    const bool ok = ::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0 &&
                    ::getsockname(fd, reinterpret_cast<sockaddr*>(&a), &len) == 0;
    ::close(fd);
    return ok ? ntohs(a.sin_port) : 0;  // 0 makes the caller's assertion fail
}

inline std::filesystem::path temp_file(const std::string& name, const std::string& content) {
    const auto path = std::filesystem::temp_directory_path() / ("fcstub_it_" + name);
    std::ofstream(path) << content;
    return path;
}

}  // namespace fcstub::itest
