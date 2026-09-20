#include "c2_controller.hpp"
#include "fc_link.hpp"
#include "udp_socket.hpp"

#include <array>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

constexpr uint16_t AUTO_STUB_PORT = 14560;

const char* state_name(C2State state) {
    switch (state) {
        case C2State::DISARMED:     return "DISARMED";
        case C2State::ARMED_HOLD:   return "ARMED_HOLD";
        case C2State::ARMED_GUIDED: return "ARMED_GUIDED";
        case C2State::ARMED_MANUAL: return "ARMED_MANUAL";
    }
    return "UNKNOWN";
}

float parse_number(const std::string& json, const std::string& key) {
    const std::string quoted_key = "\"" + key + "\"";

    const auto key_pos = json.find(quoted_key);
    if (key_pos == std::string::npos) {
        throw std::runtime_error("missing key: " + key);
    }

    const auto colon_pos = json.find(':', key_pos + quoted_key.size());
    if (colon_pos == std::string::npos) {
        throw std::runtime_error("missing ':' after key: " + key);
    }

    errno = 0;
    const char* begin = json.c_str() + colon_pos + 1;
    char* end = nullptr;
    const float value = std::strtof(begin, &end);

    if (begin == end || errno == ERANGE) {
        throw std::runtime_error("invalid number for key: " + key);
    }

    return value;
}

} // namespace

struct C2Controller::Impl {
    explicit Impl(uint16_t fc_port)
        : fc(fc_port),
          waypoints(AUTO_STUB_PORT),
          log_file("/var/log/c2/c2.log", std::ios::app) {}

    C2State state = C2State::DISARMED;
    FcLink fc;
    UdpSocket waypoints;
    std::ofstream log_file;
    bool healthy = false;

    void log(const std::string& line) {
        std::cout << line << std::endl;

        if (log_file.is_open()) {
            log_file << line << std::endl;
        }
    }

    void transition(C2State next) {
        if (next == state) {
            return;
        }

        const C2State prev = state;
        state = next;

        log(std::string("[C2] state: ") + state_name(prev) + " -> " + state_name(state));

        if (state == C2State::ARMED_HOLD) {
            fc.hold();
        }
    }

    void update_health() {
        if (!healthy && fc.is_connected()) {
            std::ofstream("/tmp/c2_healthy").close();
            healthy = true;
        }
    }

    void update_state() {
        if (!fc.is_armed()) {
            transition(C2State::DISARMED);
            return;
        }

        switch (fc.flight_mode()) {
            case FcLink::FlightMode::Guided:
                transition(C2State::ARMED_GUIDED);
                break;

            case FcLink::FlightMode::Hold:
                transition(C2State::ARMED_HOLD);
                break;

            case FcLink::FlightMode::Manual:
                transition(C2State::ARMED_MANUAL);
                break;

            case FcLink::FlightMode::Unknown:
                transition(C2State::ARMED_MANUAL);
                break;
        }
    }

    void handle_waypoint(const std::string& payload) {
        const float north = parse_number(payload, "north_m");
        const float east = parse_number(payload, "east_m");

        std::ostringstream out;

        if (state == C2State::ARMED_GUIDED) {
            fc.go_to_ned(north, east);
            out << "[C2] fwd: north=" << north << " east=" << east;
        } else {
            out << "[C2] blocked: waypoint in " << state_name(state);
        }

        log(out.str());
    }
};

C2Controller::C2Controller(uint16_t fc_port)
    : impl_(std::make_unique<Impl>(fc_port)) {}

C2Controller::~C2Controller() = default;

void C2Controller::tick() {
    impl_->update_health();
    impl_->update_state();

    std::array<char, 2048> buffer{};

    while (true) {
        const ssize_t received = impl_->waypoints.recv(buffer.data(), buffer.size());
        if (received <= 0) {
            break;
        }

        try {
            impl_->handle_waypoint(std::string(buffer.data(), static_cast<std::size_t>(received)));
        } catch (const std::exception& e) {
            impl_->log(std::string("[C2] error: invalid waypoint: ") + e.what());
        }
    }
}

C2State C2Controller::current_state() const {
    return impl_->state;
}
