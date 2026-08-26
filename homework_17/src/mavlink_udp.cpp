#include "mavlink_udp.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <common/mavlink.h>

namespace
{
constexpr uint8_t SYSTEM_ID = 1;
constexpr uint8_t COMPONENT_ID = MAV_COMP_ID_AUTOPILOT1;
constexpr double LATITUDE_ORIGIN = 50.4501;
constexpr double LONGITUDE_ORIGIN = 30.5234;
constexpr double METRES_PER_DEGREE = 111320.0;
constexpr double PI = 3.14159265358979323846;
constexpr auto DROP_ACK_TIMEOUT = std::chrono::milliseconds(250);
constexpr unsigned MAX_DROP_ATTEMPTS = 5;

double latitudeFromY(double y)
{
    // У локальній системі Y спрямована на північ, X - на схід.
    return LATITUDE_ORIGIN + y / METRES_PER_DEGREE;
}

double longitudeFromX(double x)
{
    const double latitudeRadians = LATITUDE_ORIGIN * PI / 180.0;
    return LONGITUDE_ORIGIN +
           x / (METRES_PER_DEGREE * std::cos(latitudeRadians));
}

template <typename T>
T roundedClamp(double value)
{
    const double low = static_cast<double>(std::numeric_limits<T>::lowest());
    const double high = static_cast<double>(std::numeric_limits<T>::max());
    return static_cast<T>(std::llround(std::clamp(value, low, high)));
}

uint16_t headingCentidegrees(float directionRadians)
{
    double heading = std::fmod(
        90.0 - static_cast<double>(directionRadians) * 180.0 / PI,
        360.0);
    if (heading < 0.0) {
        heading += 360.0;
    }
    const long long centidegrees = std::llround(heading * 100.0);
    return static_cast<uint16_t>(centidegrees % 36000LL);
}

float yawRadians(float directionRadians)
{
    double yaw = std::fmod(
        PI / 2.0 - static_cast<double>(directionRadians),
        2.0 * PI);
    if (yaw > PI) {
        yaw -= 2.0 * PI;
    } else if (yaw < -PI) {
        yaw += 2.0 * PI;
    }
    return static_cast<float>(yaw);
}
}

MavlinkUdp::MavlinkUdp(std::string address, uint16_t port)
    : address_(std::move(address)), port_(port)
{
}

MavlinkUdp::~MavlinkUdp()
{
    if (socketFd_ >= 0) {
        close(socketFd_);
    }
}

bool MavlinkUdp::openSocket()
{
    socketFd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd_ < 0) {
        std::cerr << "Cannot create MAVLink UDP socket: "
                  << std::strerror(errno) << '\n';
        return false;
    }

    const int flags = fcntl(socketFd_, F_GETFL, 0);
    if (flags < 0 || fcntl(socketFd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        std::cerr << "Cannot make MAVLink UDP socket nonblocking: "
                  << std::strerror(errno) << '\n';
        return false;
    }

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* result = nullptr;
    const int resolveResult = getaddrinfo(address_.c_str(), nullptr, &hints, &result);
    if (resolveResult != 0 || result == nullptr) {
        std::cerr << "Cannot resolve MAVLink address " << address_ << ": "
                  << gai_strerror(resolveResult) << '\n';
        return false;
    }

    destination_ = *reinterpret_cast<sockaddr_in*>(result->ai_addr);
    destination_.sin_port = htons(port_);
    freeaddrinfo(result);

    return true;
}

bool MavlinkUdp::sendMessage(const mavlink_message_t& message)
{
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const uint16_t length = mavlink_msg_to_send_buffer(
        buffer.data(),
        &message);
    const ssize_t sent = sendto(
        socketFd_,
        buffer.data(),
        length,
        0,
        reinterpret_cast<const sockaddr*>(&destination_),
        sizeof(destination_));
    if (sent != length) {
        std::cerr << "Cannot send MAVLink UDP frame: "
                  << std::strerror(errno) << '\n';
        return false;
    }
    return true;
}

bool MavlinkUdp::sendHeartbeat()
{
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,
        MAV_STATE_ACTIVE);
    return sendMessage(message);
}

bool MavlinkUdp::sendGlobalPosition(const dlink::Telemetry& telemetry)
{
    mavlink_message_t message{};
    mavlink_msg_global_position_int_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        telemetry.t_ms,
        roundedClamp<int32_t>(latitudeFromY(telemetry.y) * 1.0e7),
        roundedClamp<int32_t>(longitudeFromX(telemetry.x) * 1.0e7),
        roundedClamp<int32_t>(static_cast<double>(telemetry.z) * 1000.0),
        roundedClamp<int32_t>(static_cast<double>(telemetry.z) * 1000.0),
        roundedClamp<int16_t>(static_cast<double>(telemetry.vx) * 100.0),
        roundedClamp<int16_t>(static_cast<double>(telemetry.vy) * 100.0),
        0,
        headingCentidegrees(telemetry.dir));
    return sendMessage(message);
}

bool MavlinkUdp::sendAttitude(const dlink::Telemetry& telemetry)
{
    mavlink_message_t message{};
    mavlink_msg_attitude_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        telemetry.t_ms,
        0.0f,
        0.0f,
        yawRadians(telemetry.dir),
        0.0f,
        0.0f,
        0.0f);
    return sendMessage(message);
}

bool MavlinkUdp::sendTelemetry(const dlink::Telemetry& telemetry)
{
    bool ok = true;
    // Телеметрія приходить частіше, а HEARTBEAT потрібен приблизно раз на секунду.
    if (!heartbeatSent_ || telemetry.t_ms < lastHeartbeatMs_ ||
        telemetry.t_ms - lastHeartbeatMs_ >= 1000U)
    {
        ok = sendHeartbeat() && ok;
        heartbeatSent_ = true;
        lastHeartbeatMs_ = telemetry.t_ms;
    }

    ok = sendGlobalPosition(telemetry) && ok;
    ok = sendAttitude(telemetry) && ok;
    return ok;
}

bool MavlinkUdp::startDropCommand(const dlink::Telemetry& telemetry)
{
    if (dropPending_) {
        return true;
    }

    dropLatitude_ = static_cast<float>(latitudeFromY(telemetry.y));
    dropLongitude_ = static_cast<float>(longitudeFromX(telemetry.x));
    dropAltitude_ = telemetry.z;
    dropPending_ = true;
    dropAcknowledged_ = false;
    dropAttempts_ = 0;
    return sendDropAttempt();
}

bool MavlinkUdp::sendDropAttempt()
{
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &message,
        SYSTEM_ID,
        COMPONENT_ID,
        MAV_CMD_USER_1,
        static_cast<uint8_t>(dropAttempts_),
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        dropLatitude_,
        dropLongitude_,
        dropAltitude_);

    ++dropAttempts_;
    lastDropAttempt_ = std::chrono::steady_clock::now();
    return sendMessage(message);
}

void MavlinkUdp::receiveMessages()
{
    std::array<uint8_t, 2048> buffer{};
    while (true) {
        const ssize_t count = recv(socketFd_, buffer.data(), buffer.size(), 0);
        if (count < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "Cannot receive MAVLink UDP frame: "
                      << std::strerror(errno) << '\n';
            return;
        }

        mavlink_message_t message{};
        mavlink_status_t status{};
        for (ssize_t i = 0; i < count; ++i) {
            if (mavlink_parse_char(
                    MAVLINK_COMM_0,
                    buffer[static_cast<std::size_t>(i)],
                    &message,
                    &status) == 0)
            {
                continue;
            }

            if (message.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
                continue;
            }

            mavlink_command_ack_t acknowledgement{};
            mavlink_msg_command_ack_decode(&message, &acknowledgement);
            if (dropPending_ &&
                acknowledgement.command == MAV_CMD_USER_1 &&
                acknowledgement.result == MAV_RESULT_ACCEPTED)
            {
                dropPending_ = false;
                dropAcknowledged_ = true;
                std::cout << "MAVLink drop ACK received after "
                          << dropAttempts_ << " attempt(s)\n";
            }
        }
    }
}

void MavlinkUdp::poll()
{
    if (socketFd_ < 0) {
        return;
    }

    receiveMessages();
    if (!dropPending_) {
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - lastDropAttempt_;
    if (elapsed < DROP_ACK_TIMEOUT) {
        return;
    }

    if (dropAttempts_ < MAX_DROP_ATTEMPTS) {
        // Перший пакет чекер спеціально губить, тому повторюємо ту саму команду.
        sendDropAttempt();
        return;
    }

    dropPending_ = false;
    std::cerr << "ACK не отримано після 5 спроб\n";
}

bool MavlinkUdp::dropPending() const
{
    return dropPending_;
}

bool MavlinkUdp::dropAcknowledged() const
{
    return dropAcknowledged_;
}

unsigned MavlinkUdp::dropAttempts() const
{
    return dropAttempts_;
}
