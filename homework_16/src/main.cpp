#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

namespace {

constexpr std::uint8_t kMpu6050Address = 0x68;
constexpr std::uint8_t kWhoAmIRegister = 0x75;
constexpr std::uint8_t kExpectedWhoAmI = 0x68;
constexpr std::uint8_t kPowerManagementRegister = 0x6B;
constexpr std::uint8_t kMeasurementStartRegister = 0x3B;
constexpr std::size_t kMeasurementByteCount = 14;

constexpr double kAccelerometerScale = 16384.0;
constexpr double kGyroscopeScale = 131.0;
constexpr double kTemperatureScale = 340.0;
constexpr double kTemperatureOffset = 36.53;

class FileDescriptor {
public:
    explicit FileDescriptor(int fd) : fd_(fd) {}

    ~FileDescriptor()
    {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor& operator=(const FileDescriptor&) = delete;

    [[nodiscard]] int get() const
    {
        return fd_;
    }

private:
    int fd_;
};

[[noreturn]] void throw_system_error(const std::string& operation)
{
    const int saved_errno = errno;

    throw std::runtime_error(
        operation + ": " + std::strerror(saved_errno)
    );
}

void select_register(int fd, std::uint8_t register_address)
{
    if (write(fd, &register_address, 1) != 1) {
        throw_system_error("Не вдалося передати адресу регістра");
    }
}

std::uint8_t read_register(int fd, std::uint8_t register_address)
{
    select_register(fd, register_address);

    std::uint8_t value = 0;

    if (read(fd, &value, 1) != 1) {
        throw_system_error("Не вдалося прочитати регістр");
    }

    return value;
}

void write_register(
    int fd,
    std::uint8_t register_address,
    std::uint8_t value
)
{
    const std::array<std::uint8_t, 2> data{
        register_address,
        value
    };

    if (write(fd, data.data(), data.size()) !=
        static_cast<ssize_t>(data.size())) {
        throw_system_error("Не вдалося записати регістр");
    }
}

std::array<std::uint8_t, kMeasurementByteCount>
read_measurement_block(int fd)
{
    select_register(fd, kMeasurementStartRegister);

    std::array<std::uint8_t, kMeasurementByteCount> bytes{};
    const ssize_t received =
        read(fd, bytes.data(), bytes.size());

    if (received < 0) {
        throw_system_error("Помилка читання блока вимірювань");
    }

    if (received != static_cast<ssize_t>(bytes.size())) {
        throw std::runtime_error(
            "Отримано неповний блок вимірювань: " +
            std::to_string(received) + " із " +
            std::to_string(bytes.size()) + " байтів"
        );
    }

    return bytes;
}

std::int16_t make_signed_16(
    std::uint8_t high_byte,
    std::uint8_t low_byte
)
{
    int value =
        (static_cast<int>(high_byte) << 8) |
        static_cast<int>(low_byte);

    if (value >= 0x8000) {
        value -= 0x10000;
    }

    return static_cast<std::int16_t>(value);
}

struct SensorValues {
    double acceleration_x_g;
    double acceleration_y_g;
    double acceleration_z_g;
    double temperature_c;
    double gyroscope_x_dps;
    double gyroscope_y_dps;
    double gyroscope_z_dps;
};

SensorValues decode_measurements(
    const std::array<std::uint8_t, kMeasurementByteCount>& bytes
)
{
    const std::int16_t raw_acceleration_x =
        make_signed_16(bytes[0], bytes[1]);
    const std::int16_t raw_acceleration_y =
        make_signed_16(bytes[2], bytes[3]);
    const std::int16_t raw_acceleration_z =
        make_signed_16(bytes[4], bytes[5]);
    const std::int16_t raw_temperature =
        make_signed_16(bytes[6], bytes[7]);
    const std::int16_t raw_gyroscope_x =
        make_signed_16(bytes[8], bytes[9]);
    const std::int16_t raw_gyroscope_y =
        make_signed_16(bytes[10], bytes[11]);
    const std::int16_t raw_gyroscope_z =
        make_signed_16(bytes[12], bytes[13]);

    return {
        raw_acceleration_x / kAccelerometerScale,
        raw_acceleration_y / kAccelerometerScale,
        raw_acceleration_z / kAccelerometerScale,
        raw_temperature / kTemperatureScale +
            kTemperatureOffset,
        raw_gyroscope_x / kGyroscopeScale,
        raw_gyroscope_y / kGyroscopeScale,
        raw_gyroscope_z / kGyroscopeScale
    };
}

void print_measurements(const SensorValues& values)
{
    std::cout
        << std::fixed << std::setprecision(3)
        << "Прискорення [g]: "
        << "X=" << values.acceleration_x_g << "  "
        << "Y=" << values.acceleration_y_g << "  "
        << "Z=" << values.acceleration_z_g << '\n'
        << std::setprecision(2)
        << "Температура [°C]: "
        << values.temperature_c << '\n'
        << "Кутова швидкість [°/с]: "
        << "X=" << values.gyroscope_x_dps << "  "
        << "Y=" << values.gyroscope_y_dps << "  "
        << "Z=" << values.gyroscope_z_dps << '\n'
        << "----------------------------------------\n";
}

unsigned long parse_i2c_address(const char* text)
{
    std::size_t parsed_characters = 0;
    const std::string value{text};

    const unsigned long address =
        std::stoul(value, &parsed_characters, 0);

    if (parsed_characters != value.size() || address > 0x7F) {
        throw std::invalid_argument(
            "Некоректна 7-бітна I2C-адреса"
        );
    }

    return address;
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr
            << "Використання: " << argv[0]
            << " <I2C-шина> <адреса>\n"
            << "Приклад: " << argv[0]
            << " /dev/i2c-1 0x68\n";
        return EXIT_FAILURE;
    }

    try {
        const unsigned long address = parse_i2c_address(argv[2]);
        const int raw_fd = open(argv[1], O_RDWR);

        if (raw_fd < 0) {
            throw_system_error(
                std::string{"Не вдалося відкрити "} + argv[1]
            );
        }

        FileDescriptor bus{raw_fd};

        if (ioctl(bus.get(), I2C_SLAVE, address) < 0) {
            throw_system_error(
                "Пристрій не відповідає на вказаній адресі"
            );
        }

        const std::uint8_t identity =
            read_register(bus.get(), kWhoAmIRegister);

        std::cout
            << "I2C-шина: " << argv[1] << '\n'
            << "Адреса: 0x" << std::hex << std::uppercase
            << address << '\n'
            << "WHO_AM_I: 0x"
            << static_cast<unsigned int>(identity)
            << std::dec << '\n';

        if (address != kMpu6050Address) {
            std::cerr
                << "Помилка: MPU-6050 очікується за адресою 0x68\n";
            return EXIT_FAILURE;
        }

        if (identity != kExpectedWhoAmI) {
            std::cerr
                << "Помилка: отримано неправильний ID датчика\n";
            return EXIT_FAILURE;
        }

        std::cout << "MPU-6050 знайдено\n";

        write_register(
            bus.get(),
            kPowerManagementRegister,
            0x00
        );

        std::this_thread::sleep_for(
            std::chrono::milliseconds{100}
        );

        std::cout
            << "Датчик активовано. Для зупинки натисніть Ctrl+C.\n"
            << "----------------------------------------\n";

        while (true) {
            const auto bytes =
                read_measurement_block(bus.get());
            const SensorValues values =
                decode_measurements(bytes);

            print_measurements(values);

            std::this_thread::sleep_for(
                std::chrono::milliseconds{200}
            );
        }
    } catch (const std::exception& error) {
        std::cerr << "Помилка: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
