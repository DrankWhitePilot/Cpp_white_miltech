#include "gpio_controller.hpp"

#include <iostream>

#include <unistd.h>

GpioController::~GpioController()
{
    if (drop_ != nullptr) {
        gpiod_line_set_value(drop_, 0);
        gpiod_line_release(drop_);
    }

    if (start_ != nullptr) {
        gpiod_line_set_value(start_, 0);
        gpiod_line_release(start_);
    }

    if (chip_ != nullptr) {
        gpiod_chip_close(chip_);
    }
}

bool GpioController::init(const std::string& chipName, int startLine, int dropLine)
{
    chip_ = gpiod_chip_open_by_name(chipName.c_str());
    if (chip_ == nullptr) {
        std::cerr << "Cannot open GPIO chip: " << chipName << "\n";
        return false;
    }

    start_ = gpiod_chip_get_line(chip_, static_cast<unsigned int>(startLine));
    drop_ = gpiod_chip_get_line(chip_, static_cast<unsigned int>(dropLine));

    if (start_ == nullptr || drop_ == nullptr) {
        std::cerr << "Cannot get GPIO lines\n";
        return false;
    }

    if (gpiod_line_request_output(start_, "drone", 0) != 0) {
        std::cerr << "Cannot request START line\n";
        return false;
    }

    if (gpiod_line_request_output(drop_, "drone", 0) != 0) {
        std::cerr << "Cannot request DROP line\n";
        return false;
    }

    return true;
}

bool GpioController::setStart(bool value)
{
    return start_ != nullptr && gpiod_line_set_value(start_, value ? 1 : 0) == 0;
}

bool GpioController::pulseDrop(int usec)
{
    if (drop_ == nullptr) {
        return false;
    }

    if (gpiod_line_set_value(drop_, 1) != 0) {
        return false;
    }

    usleep(usec);

    return gpiod_line_set_value(drop_, 0) == 0;
}
