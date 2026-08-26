#include <array>
#include <chrono>
#include <iostream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include <arpa/inet.h>
#include <common/mavlink.h>

#include "mavlink_udp.hpp"

namespace
{
void readAvailable(int socketFd, int& commands, int& positions, int& attitudes)
{
    std::array<uint8_t, 2048> bytes{};
    while (true) {
        const ssize_t count = recv(socketFd, bytes.data(), bytes.size(), MSG_DONTWAIT);
        if (count <= 0) {
            return;
        }

        mavlink_message_t message{};
        mavlink_status_t status{};
        for (ssize_t i = 0; i < count; ++i) {
            if (mavlink_parse_char(
                    MAVLINK_COMM_2,
                    bytes[static_cast<std::size_t>(i)],
                    &message,
                    &status) == 0)
            {
                continue;
            }
            commands += message.msgid == MAVLINK_MSG_ID_COMMAND_LONG;
            positions += message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
            attitudes += message.msgid == MAVLINK_MSG_ID_ATTITUDE;
        }
    }
}
}

int main()
{
    const int server = socket(AF_INET, SOCK_DGRAM, 0);
    if (server < 0) {
        return 1;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    bind(server, reinterpret_cast<sockaddr*>(&address), sizeof(address));

    socklen_t addressSize = sizeof(address);
    getsockname(server, reinterpret_cast<sockaddr*>(&address), &addressSize);

    MavlinkUdp link("127.0.0.1", ntohs(address.sin_port));
    if (!link.openSocket()) {
        close(server);
        return 1;
    }

    dlink::Telemetry telemetry{};
    telemetry.z = 100.0f;
    telemetry.vx = 10.0f;
    link.startDropCommand(telemetry);

    int commands = 0;
    int positions = 0;
    int attitudes = 0;
    uint32_t timeMs = 0;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);

    // Імітуємо QGC: приймаємо телеметрію, але ACK на скид не надсилаємо.
    while (link.dropPending() && std::chrono::steady_clock::now() < deadline) {
        telemetry.t_ms = timeMs;
        telemetry.x = 10.0f * static_cast<float>(timeMs) / 1000.0f;
        link.sendTelemetry(telemetry);
        link.poll();
        readAvailable(server, commands, positions, attitudes);
        timeMs += 50;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    readAvailable(server, commands, positions, attitudes);
    close(server);

    if (link.dropPending() || link.dropAttempts() != 5 || commands != 5 ||
        positions < 20 || attitudes < 20)
    {
        std::cerr << "NO_ACK_TEST_FAIL attempts=" << link.dropAttempts()
                  << " commands=" << commands
                  << " positions=" << positions
                  << " attitudes=" << attitudes << '\n';
        return 1;
    }

    std::cout << "NO_ACK_TEST_PASS attempts=5 telemetry_continues=1\n";
    return 0;
}
