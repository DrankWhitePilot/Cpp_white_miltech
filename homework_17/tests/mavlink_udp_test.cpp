#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <poll.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include <arpa/inet.h>
#include <common/mavlink.h>

#include "mavlink_udp.hpp"

namespace
{
constexpr double EPS = 1.0e-5;

int fail(const char* message)
{
    std::cerr << "MAVLINK_UDP_TEST_FAIL: " << message << '\n';
    return 1;
}

bool receiveMessage(
    int socketFd,
    mavlink_message_t& message,
    sockaddr_in& source,
    int timeoutMs)
{
    pollfd descriptor{};
    descriptor.fd = socketFd;
    descriptor.events = POLLIN;
    if (poll(&descriptor, 1, timeoutMs) <= 0) {
        return false;
    }

    std::array<uint8_t, 2048> buffer{};
    socklen_t sourceLength = sizeof(source);
    const ssize_t count = recvfrom(
        socketFd,
        buffer.data(),
        buffer.size(),
        0,
        reinterpret_cast<sockaddr*>(&source),
        &sourceLength);
    if (count <= 0) {
        return false;
    }

    mavlink_status_t status{};
    for (ssize_t i = 0; i < count; ++i) {
        if (mavlink_parse_char(
                MAVLINK_COMM_1,
                buffer[static_cast<std::size_t>(i)],
                &message,
                &status) != 0)
        {
            return true;
        }
    }
    return false;
}
}

int main()
{
    const int server = socket(AF_INET, SOCK_DGRAM, 0);
    if (server < 0) {
        return fail("server socket");
    }

    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = 0;
    inet_pton(AF_INET, "127.0.0.1", &serverAddress.sin_addr);
    if (bind(
            server,
            reinterpret_cast<const sockaddr*>(&serverAddress),
            sizeof(serverAddress)) != 0)
    {
        close(server);
        return fail("server bind");
    }

    socklen_t serverLength = sizeof(serverAddress);
    if (getsockname(
            server,
            reinterpret_cast<sockaddr*>(&serverAddress),
            &serverLength) != 0)
    {
        close(server);
        return fail("server port");
    }

    MavlinkUdp link("127.0.0.1", ntohs(serverAddress.sin_port));
    if (!link.openSocket()) {
        close(server);
        return fail("link open");
    }

    dlink::Telemetry telemetry{};
    telemetry.t_ms = 1234;
    telemetry.x = 100.0f;
    telemetry.y = 200.0f;
    telemetry.z = 50.0f;
    telemetry.vx = 3.0f;
    telemetry.vy = 4.0f;
    telemetry.dir = static_cast<float>(3.14159265358979323846 / 2.0);
    if (!link.sendTelemetry(telemetry)) {
        close(server);
        return fail("telemetry send");
    }

    bool heartbeatSeen = false;
    bool positionSeen = false;
    bool attitudeSeen = false;
    sockaddr_in source{};
    for (int i = 0; i < 3; ++i) {
        mavlink_message_t message{};
        if (!receiveMessage(server, message, source, 500)) {
            close(server);
            return fail("telemetry receive");
        }
        if (message.magic != MAVLINK_STX ||
            message.sysid != 1 ||
            message.compid != MAV_COMP_ID_AUTOPILOT1)
        {
            close(server);
            return fail("MAVLink 2 identity");
        }

        if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            mavlink_heartbeat_t heartbeat{};
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            heartbeatSeen = heartbeat.type == MAV_TYPE_QUADROTOR &&
                            heartbeat.system_status == MAV_STATE_ACTIVE;
        } else if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
            mavlink_global_position_int_t position{};
            mavlink_msg_global_position_int_decode(&message, &position);
            positionSeen = position.time_boot_ms == telemetry.t_ms &&
                           position.lat == 504518966 &&
                           position.lon == 305248108 &&
                           position.alt == 50000 &&
                           position.relative_alt == 50000 &&
                           position.vx == 300 &&
                           position.vy == 400 &&
                           position.hdg == 0;
        } else if (message.msgid == MAVLINK_MSG_ID_ATTITUDE) {
            mavlink_attitude_t attitude{};
            mavlink_msg_attitude_decode(&message, &attitude);
            attitudeSeen = attitude.time_boot_ms == telemetry.t_ms &&
                           std::fabs(attitude.yaw) < EPS;
        }
    }

    if (!heartbeatSeen || !positionSeen || !attitudeSeen) {
        close(server);
        return fail("required telemetry messages");
    }

    if (!link.startDropCommand(telemetry)) {
        close(server);
        return fail("drop start");
    }

    mavlink_message_t commandMessage{};
    if (!receiveMessage(server, commandMessage, source, 500) ||
        commandMessage.msgid != MAVLINK_MSG_ID_COMMAND_LONG)
    {
        close(server);
        return fail("first drop command");
    }

    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&commandMessage, &command);
    if (command.command != MAV_CMD_USER_1 ||
        std::fabs(command.param5 - 50.4518966f) > EPS ||
        std::fabs(command.param6 - 30.5248108f) > EPS ||
        std::fabs(command.param7 - telemetry.z) > EPS)
    {
        close(server);
        return fail("drop command fields");
    }

    const auto retryDeadline = std::chrono::steady_clock::now() +
                               std::chrono::seconds(2);
    bool retrySeen = false;
    while (std::chrono::steady_clock::now() < retryDeadline) {
        link.poll();
        mavlink_message_t retry{};
        if (receiveMessage(server, retry, source, 20) &&
            retry.msgid == MAVLINK_MSG_ID_COMMAND_LONG)
        {
            retrySeen = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!retrySeen || link.dropAttempts() != 2) {
        close(server);
        return fail("drop retry");
    }

    mavlink_message_t acknowledgement{};
    mavlink_msg_command_ack_pack(
        42,
        99,
        &acknowledgement,
        MAV_CMD_USER_1,
        MAV_RESULT_ACCEPTED,
        100,
        0,
        1,
        MAV_COMP_ID_AUTOPILOT1);
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> acknowledgementBuffer{};
    const uint16_t acknowledgementLength = mavlink_msg_to_send_buffer(
        acknowledgementBuffer.data(),
        &acknowledgement);
    if (sendto(
            server,
            acknowledgementBuffer.data(),
            acknowledgementLength,
            0,
            reinterpret_cast<const sockaddr*>(&source),
            sizeof(source)) != acknowledgementLength)
    {
        close(server);
        return fail("ACK send");
    }

    const auto acknowledgementDeadline = std::chrono::steady_clock::now() +
                                         std::chrono::seconds(1);
    while (link.dropPending() &&
           std::chrono::steady_clock::now() < acknowledgementDeadline)
    {
        link.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (link.dropPending() || !link.dropAcknowledged() ||
        link.dropAttempts() != 2)
    {
        close(server);
        return fail("accepted ACK stops retries");
    }

    const auto silenceDeadline = std::chrono::steady_clock::now() +
                                 std::chrono::milliseconds(350);
    while (std::chrono::steady_clock::now() < silenceDeadline) {
        link.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    mavlink_message_t unexpected{};
    if (receiveMessage(server, unexpected, source, 20)) {
        close(server);
        return fail("command repeated after ACK");
    }

    if (!link.startDropCommand(telemetry)) {
        close(server);
        return fail("second drop start");
    }
    unsigned commandsWithoutAck = 0;
    const auto fiveAttemptDeadline = std::chrono::steady_clock::now() +
                                     std::chrono::seconds(3);
    while ((link.dropPending() || commandsWithoutAck < 5) &&
           std::chrono::steady_clock::now() < fiveAttemptDeadline)
    {
        link.poll();
        mavlink_message_t attempt{};
        if (receiveMessage(server, attempt, source, 20) &&
            attempt.msgid == MAVLINK_MSG_ID_COMMAND_LONG)
        {
            ++commandsWithoutAck;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (link.dropPending() || link.dropAttempts() != 5 ||
        commandsWithoutAck != 5)
    {
        close(server);
        return fail("exactly five attempts without ACK");
    }

    close(server);
    std::cout << "MAVLINK_UDP_TEST_PASS\n";
    return 0;
}
