#include "uart_link.hpp"

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace
{
float normalizedControl(float value)
{
    if (!std::isfinite(value)) {
        return 0.0f;
    }

    return std::clamp(value, -1.0f, 1.0f);
}
}

UartLink::UartLink(std::string device)
    : device_(std::move(device))
{
}

UartLink::~UartLink()
{
    if (fd_ >= 0) {
        close(fd_);
    }
}

bool UartLink::openPort()
{
    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "UART open failed: " << device_ << ": "
                  << std::strerror(errno) << "\n";
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        std::cerr << "UART tcgetattr failed: " << std::strerror(errno) << "\n";
        close(fd_);
        fd_ = -1;
        return false;
    }

    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);

    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::cerr << "UART tcsetattr failed: " << std::strerror(errno) << "\n";
        close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

UartReadResult UartLink::readPacket(UartPacket& packet)
{
    if (fd_ < 0) {
        return UartReadResult::Error;
    }

    uint8_t byte = 0;

    while (true) {
        const ssize_t n = read(fd_, &byte, 1);

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }

            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return UartReadResult::WouldBlock;
            }

            std::cerr << "UART read failed: " << std::strerror(errno) << "\n";
            return UartReadResult::Error;
        }

        if (n == 0) {
            return UartReadResult::Closed;
        }

        uint8_t type = 0;
        uint8_t len = 0;
        uint8_t payload[260]{};

        if (parser_.feed(byte, type, payload, len)) {
            packet.type = type;
            packet.len = len;
            std::memcpy(packet.payload, payload, len);
            return UartReadResult::Packet;
        }
    }
}

bool UartLink::writeAll(const uint8_t* data, std::size_t size)
{
    using Clock = std::chrono::steady_clock;
    constexpr auto WRITE_TIMEOUT = std::chrono::milliseconds(20);

    const auto deadline = Clock::now() + WRITE_TIMEOUT;
    std::size_t offset = 0;

    while (offset < size) {
        if (Clock::now() >= deadline) {
            std::cerr << "UART write timed out\n";
            return false;
        }

        const ssize_t written = write(fd_, data + offset, size - offset);

        if (written > 0) {
            offset += static_cast<std::size_t>(written);
            continue;
        }

        if (written == 0) {
            std::cerr << "UART write returned zero bytes\n";
            return false;
        }

        if (errno == EINTR) {
            continue;
        }

        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "UART write failed: " << std::strerror(errno) << "\n";
            return false;
        }

        const auto now = Clock::now();
        if (now >= deadline) {
            std::cerr << "UART write timed out\n";
            return false;
        }

        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now);
        int timeoutMs = static_cast<int>(remaining.count());
        if (timeoutMs < 1) {
            timeoutMs = 1;
        }

        pollfd descriptor{};
        descriptor.fd = fd_;
        descriptor.events = POLLOUT;

        const int pollResult = poll(&descriptor, 1, timeoutMs);

        if (pollResult == 0) {
            std::cerr << "UART write timed out\n";
            return false;
        }

        if (pollResult < 0) {
            if (errno == EINTR) {
                continue;
            }

            std::cerr << "UART poll failed: " << std::strerror(errno) << "\n";
            return false;
        }

        if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            std::cerr << "UART became unavailable while writing\n";
            return false;
        }
    }

    return true;
}

bool UartLink::sendControl(float accel, float turnRate)
{
    if (fd_ < 0) {
        return false;
    }

    dlink::Control control{
        normalizedControl(accel),
        normalizedControl(turnRate)};
    uint8_t encoded[64]{};

    const size_t len = dlink::encode(
        dlink::PKT_CONTROL,
        &control,
        static_cast<uint8_t>(sizeof(control)),
        encoded);

    return writeAll(encoded, len);
}
