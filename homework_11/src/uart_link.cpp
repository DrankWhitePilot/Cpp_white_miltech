#include "uart_link.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <utility>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace
{
float clampControl(float value)
{
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
        std::perror("open UART");
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        std::perror("tcgetattr");
        return false;
    }

    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::perror("tcsetattr");
        return false;
    }

    return true;
}

bool UartLink::poll()
{
    if (fd_ < 0) {
        return false;
    }

    bool gotAny = false;

    for (;;) {
        uint8_t buf[256]{};
        const ssize_t n = read(fd_, buf, sizeof(buf));

        if (n > 0) {
            gotAny = true;

            uint8_t type = 0;
            uint8_t len = 0;
            uint8_t payload[260]{};

            for (ssize_t i = 0; i < n; ++i) {
                if (parser_.feed(buf[i], type, payload, len)) {
                    UartPacket packet{};
                    packet.type = type;
                    packet.len = len;
                    std::memcpy(packet.payload, payload, len);
                    pending_.push_back(packet);
                }
            }

            continue;
        }

        if (n == 0) {
            return gotAny;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return gotAny;
        }

        std::perror("read UART");
        return false;
    }
}

bool UartLink::readPacket(UartPacket& packet)
{
    if (pending_.empty()) {
        poll();
    }

    if (pending_.empty()) {
        return false;
    }

    packet = pending_.front();
    pending_.pop_front();
    return true;
}

bool UartLink::sendControl(float accel, float turnRate)
{
    if (fd_ < 0) {
        return false;
    }

    dlink::Control control{clampControl(accel), clampControl(turnRate)};
    uint8_t out[64]{};
    const size_t size = dlink::encode(
        dlink::PKT_CONTROL,
        &control,
        static_cast<uint8_t>(sizeof(control)),
        out);

    size_t written = 0;
    while (written < size) {
        const ssize_t n = write(fd_, out + written, size - written);
        if (n > 0) {
            written += static_cast<size_t>(n);
            continue;
        }

        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            usleep(1000);
            continue;
        }

        std::perror("write CONTROL");
        return false;
    }

    return true;
}
