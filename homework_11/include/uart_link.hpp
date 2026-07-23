#pragma once

#include <cstdint>
#include <deque>
#include <string>

#include "drone_link.h"

struct UartPacket
{
    uint8_t type = 0;
    uint8_t len = 0;
    uint8_t payload[260]{};
};

class UartLink
{
public:
    explicit UartLink(std::string device);
    ~UartLink();

    UartLink(const UartLink&) = delete;
    UartLink& operator=(const UartLink&) = delete;

    bool openPort();
    bool readPacket(UartPacket& packet);
    bool sendControl(float accel, float turnRate);

private:
    bool poll();

    std::string device_;
    int fd_ = -1;
    dlink::Parser parser_{};
    std::deque<UartPacket> pending_;
};
