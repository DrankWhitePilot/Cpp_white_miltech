#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "drone_link.h"

struct UartPacket
{
    uint8_t type = 0;
    uint8_t len = 0;
    uint8_t payload[260]{};
};

enum class UartReadResult
{
    Packet,
    WouldBlock,
    Closed,
    Error
};

class UartLink
{
public:
    explicit UartLink(std::string device);
    ~UartLink();

    UartLink(const UartLink&) = delete;
    UartLink& operator=(const UartLink&) = delete;

    bool openPort();
    UartReadResult readPacket(UartPacket& packet);
    bool sendControl(float accel, float turnRate);

private:
    bool writeAll(const uint8_t* data, std::size_t size);

    std::string device_;
    int fd_ = -1;
    dlink::Parser parser_{};
};
