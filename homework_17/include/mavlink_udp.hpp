#pragma once

#include <chrono>
#include <cstdint>
#include <netinet/in.h>
#include <string>

#include "drone_link.h"

class MavlinkUdp
{
public:
    MavlinkUdp(std::string address, uint16_t port);
    ~MavlinkUdp();

    MavlinkUdp(const MavlinkUdp&) = delete;
    MavlinkUdp& operator=(const MavlinkUdp&) = delete;

    bool openSocket();
    bool sendTelemetry(const dlink::Telemetry& telemetry);
    bool startDropCommand(const dlink::Telemetry& telemetry);
    void poll();

    bool dropPending() const;
    bool dropAcknowledged() const;
    unsigned dropAttempts() const;

private:
    bool sendHeartbeat();
    bool sendGlobalPosition(const dlink::Telemetry& telemetry);
    bool sendAttitude(const dlink::Telemetry& telemetry);
    bool sendDropAttempt();
    bool sendMessage(const struct __mavlink_message& message);
    void receiveMessages();

    std::string address_;
    uint16_t port_ = 0;
    int socketFd_ = -1;
    sockaddr_in destination_{};
    uint32_t lastHeartbeatMs_ = 0;
    bool heartbeatSent_ = false;

    bool dropPending_ = false;
    bool dropAcknowledged_ = false;
    unsigned dropAttempts_ = 0;
    float dropLatitude_ = 0.0f;
    float dropLongitude_ = 0.0f;
    float dropAltitude_ = 0.0f;
    std::chrono::steady_clock::time_point lastDropAttempt_{};
};
