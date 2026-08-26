#include <array>
#include <chrono>
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
int fail(const char* text)
{
    std::cerr << "MAVLINK_UDP_TEST_FAIL: " << text << '\n';
    return 1;
}

bool receiveMavlink(
    int socketFd,
    mavlink_message_t& message,
    sockaddr_in& sender,
    int timeoutMs = 500)
{
    pollfd descriptor{socketFd, POLLIN, 0};
    if (poll(&descriptor, 1, timeoutMs) <= 0) {
        return false;
    }

    std::array<uint8_t, 2048> bytes{};
    socklen_t senderSize = sizeof(sender);
    const ssize_t count = recvfrom(
        socketFd,
        bytes.data(),
        bytes.size(),
        0,
        reinterpret_cast<sockaddr*>(&sender),
        &senderSize);

    mavlink_status_t status{};
    for (ssize_t i = 0; i < count; ++i) {
        if (mavlink_parse_char(
                MAVLINK_COMM_1,
                bytes[static_cast<std::size_t>(i)],
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
        return fail("socket");
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(server, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        close(server);
        return fail("bind");
    }

    socklen_t addressSize = sizeof(address);
    getsockname(server, reinterpret_cast<sockaddr*>(&address), &addressSize);

    MavlinkUdp link("127.0.0.1", ntohs(address.sin_port));
    if (!link.openSocket()) {
        close(server);
        return fail("openSocket");
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
        return fail("sendTelemetry");
    }

    bool heartbeat = false;
    bool position = false;
    bool attitude = false;
    sockaddr_in sender{};

    for (int i = 0; i < 3; ++i) {
        mavlink_message_t message{};
        if (!receiveMavlink(server, message, sender) ||
            message.magic != MAVLINK_STX ||
            message.sysid != 1 ||
            message.compid != MAV_COMP_ID_AUTOPILOT1)
        {
            close(server);
            return fail("frame format");
        }

        heartbeat |= message.msgid == MAVLINK_MSG_ID_HEARTBEAT;
        attitude |= message.msgid == MAVLINK_MSG_ID_ATTITUDE;
        if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
            mavlink_global_position_int_t data{};
            mavlink_msg_global_position_int_decode(&message, &data);
            position = data.time_boot_ms == 1234 &&
                       data.lat == 504518966 && data.lon == 305248108 &&
                       data.alt == 50000 && data.vx == 300 &&
                       data.vy == 400 && data.hdg == 0;
        }
    }

    if (!heartbeat || !position || !attitude) {
        close(server);
        return fail("telemetry messages");
    }

    if (!link.startDropCommand(telemetry)) {
        close(server);
        return fail("drop command");
    }

    mavlink_message_t firstCommand{};
    if (!receiveMavlink(server, firstCommand, sender) ||
        firstCommand.msgid != MAVLINK_MSG_ID_COMMAND_LONG)
    {
        close(server);
        return fail("first COMMAND_LONG");
    }

    // Не відповідаємо на першу команду і чекаємо повтор, як робить чекер.
    mavlink_message_t secondCommand{};
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
        link.poll();
        if (receiveMavlink(server, secondCommand, sender, 20) &&
            secondCommand.msgid == MAVLINK_MSG_ID_COMMAND_LONG)
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondCommand.msgid != MAVLINK_MSG_ID_COMMAND_LONG ||
        link.dropAttempts() != 2)
    {
        close(server);
        return fail("retry");
    }

    mavlink_message_t ack{};
    mavlink_msg_command_ack_pack(
        42,
        99,
        &ack,
        MAV_CMD_USER_1,
        MAV_RESULT_ACCEPTED,
        100,
        0,
        1,
        MAV_COMP_ID_AUTOPILOT1);

    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> ackBytes{};
    const uint16_t ackSize = mavlink_msg_to_send_buffer(ackBytes.data(), &ack);
    sendto(
        server,
        ackBytes.data(),
        ackSize,
        0,
        reinterpret_cast<sockaddr*>(&sender),
        sizeof(sender));

    for (int i = 0; i < 20 && link.dropPending(); ++i) {
        link.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    close(server);
    if (link.dropPending() || !link.dropAcknowledged()) {
        return fail("ACK");
    }

    std::cout << "MAVLINK_UDP_TEST_PASS\n";
    return 0;
}
