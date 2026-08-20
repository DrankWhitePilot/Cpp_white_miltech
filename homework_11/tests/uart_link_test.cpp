#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include "drone_link.h"
#include "uart_link.hpp"

namespace
{
bool writeAll(int fd, const uint8_t* data, std::size_t size)
{
    std::size_t offset = 0;
    while (offset < size) {
        const ssize_t written = write(fd, data + offset, size - offset);
        if (written > 0) {
            offset += static_cast<std::size_t>(written);
            continue;
        }
        if (written < 0 && errno == EINTR) {
            continue;
        }
        return false;
    }
    return true;
}

bool readExact(int fd, uint8_t* data, std::size_t size)
{
    std::size_t offset = 0;
    while (offset < size) {
        pollfd descriptor{};
        descriptor.fd = fd;
        descriptor.events = POLLIN;

        const int pollResult = poll(&descriptor, 1, 200);
        if (pollResult <= 0) {
            return false;
        }

        const ssize_t count = read(fd, data + offset, size - offset);
        if (count > 0) {
            offset += static_cast<std::size_t>(count);
            continue;
        }
        if (count < 0 && errno == EINTR) {
            continue;
        }
        return false;
    }
    return true;
}

bool near(float lhs, float rhs)
{
    return std::fabs(lhs - rhs) <= 1e-6f;
}

int fail(const char* message)
{
    std::cerr << "UART_LINK_TEST_FAIL: " << message << '\n';
    return 1;
}
}

int main()
{
    int masterFd = -1;
    int temporarySlaveFd = -1;
    char slaveName[256]{};

    if (openpty(
            &masterFd,
            &temporarySlaveFd,
            slaveName,
            nullptr,
            nullptr) != 0)
    {
        return fail("openpty");
    }
    close(temporarySlaveFd);

    UartLink link(slaveName);
    if (!link.openPort()) {
        close(masterFd);
        return fail("openPort");
    }

    UartPacket packet{};
    if (link.readPacket(packet) != UartReadResult::WouldBlock) {
        close(masterFd);
        return fail("initial WouldBlock");
    }

    dlink::TargetPos target{3, 12.5f, -7.25f};
    uint8_t targetFrame[64]{};
    const std::size_t targetFrameSize = dlink::encode(
        dlink::PKT_TARGET,
        &target,
        static_cast<uint8_t>(sizeof(target)),
        targetFrame);

    if (!writeAll(masterFd, targetFrame, 3)) {
        close(masterFd);
        return fail("fragment write");
    }
    if (link.readPacket(packet) != UartReadResult::WouldBlock) {
        close(masterFd);
        return fail("fragment parser state");
    }
    if (!writeAll(
            masterFd,
            targetFrame + 3,
            targetFrameSize - 3))
    {
        close(masterFd);
        return fail("remaining frame write");
    }
    if (link.readPacket(packet) != UartReadResult::Packet ||
        packet.type != dlink::PKT_TARGET ||
        packet.len != sizeof(target) ||
        std::memcmp(packet.payload, &target, sizeof(target)) != 0)
    {
        close(masterFd);
        return fail("fragmented packet decode");
    }

    uint8_t badFrame[64]{};
    std::memcpy(badFrame, targetFrame, targetFrameSize);
    badFrame[targetFrameSize - 1] ^= 0x5aU;

    dlink::DroneCfg config{10.0f, 20.0f, 1.0f, 0.05f, 0.1f, 1.0f};
    uint8_t configFrame[64]{};
    const std::size_t configFrameSize = dlink::encode(
        dlink::PKT_CONFIG,
        &config,
        static_cast<uint8_t>(sizeof(config)),
        configFrame);

    if (!writeAll(masterFd, badFrame, targetFrameSize) ||
        !writeAll(masterFd, configFrame, configFrameSize))
    {
        close(masterFd);
        return fail("CRC stream write");
    }
    if (link.readPacket(packet) != UartReadResult::Packet ||
        packet.type != dlink::PKT_CONFIG ||
        packet.len != sizeof(config) ||
        std::memcmp(packet.payload, &config, sizeof(config)) != 0)
    {
        close(masterFd);
        return fail("bad CRC recovery");
    }

    if (!link.sendControl(2.0f, -3.0f)) {
        close(masterFd);
        return fail("sendControl");
    }

    uint8_t controlFrame[64]{};
    constexpr std::size_t CONTROL_FRAME_SIZE = sizeof(dlink::Control) + 6U;
    if (!readExact(masterFd, controlFrame, CONTROL_FRAME_SIZE)) {
        close(masterFd);
        return fail("CONTROL frame read");
    }

    dlink::Parser parser{};
    uint8_t type = 0;
    uint8_t payload[260]{};
    uint8_t length = 0;
    bool decoded = false;
    for (std::size_t i = 0; i < CONTROL_FRAME_SIZE; ++i) {
        if (parser.feed(controlFrame[i], type, payload, length)) {
            decoded = true;
        }
    }

    dlink::Control control{};
    if (!decoded || type != dlink::PKT_CONTROL ||
        length != sizeof(control))
    {
        close(masterFd);
        return fail("CONTROL frame decode");
    }
    std::memcpy(&control, payload, sizeof(control));
    if (!near(control.accel, 1.0f) || !near(control.turnRate, -1.0f)) {
        close(masterFd);
        return fail("CONTROL payload");
    }

    close(masterFd);
    const UartReadResult closeResult = link.readPacket(packet);
    if (closeResult != UartReadResult::Closed &&
        closeResult != UartReadResult::Error)
    {
        return fail("closed channel detection");
    }

    std::cout << "UART_LINK_TEST_PASS\n";
    return 0;
}
