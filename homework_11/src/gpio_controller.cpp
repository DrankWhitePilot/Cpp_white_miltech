#include "gpio_controller.hpp"

#include <iostream>
#include <string>

namespace
{
#ifndef HW11_GPIOD_V1
std::string chipPath(const std::string& chipName)
{
    if (chipName.rfind("/dev/", 0) == 0) {
        return chipName;
    }

    return "/dev/" + chipName;
}

bool requestOutputLine(
    gpiod_chip* chip,
    unsigned int offset,
    const char* consumer,
    gpiod_line_request*& request)
{
    gpiod_line_settings* settings = gpiod_line_settings_new();
    gpiod_line_config* lineConfig = gpiod_line_config_new();
    gpiod_request_config* requestConfig = gpiod_request_config_new();

    if (settings == nullptr || lineConfig == nullptr || requestConfig == nullptr) {
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(lineConfig);
        gpiod_request_config_free(requestConfig);
        return false;
    }

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

    if (gpiod_line_config_add_line_settings(
            lineConfig,
            &offset,
            1,
            settings) != 0)
    {
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(lineConfig);
        gpiod_request_config_free(requestConfig);
        return false;
    }

    gpiod_request_config_set_consumer(requestConfig, consumer);

    request = gpiod_chip_request_lines(
        chip,
        requestConfig,
        lineConfig);

    gpiod_line_settings_free(settings);
    gpiod_line_config_free(lineConfig);
    gpiod_request_config_free(requestConfig);

    return request != nullptr;
}
#endif
}

GpioController::~GpioController()
{
#ifdef HW11_GPIOD_V1
    if (drop_ != nullptr) {
        gpiod_line_set_value(drop_, 0);
        gpiod_line_release(drop_);
    }

    if (start_ != nullptr) {
        gpiod_line_set_value(start_, 0);
        gpiod_line_release(start_);
    }
#else
    if (dropRequest_ != nullptr) {
        gpiod_line_request_set_value(
            dropRequest_,
            dropOffset_,
            GPIOD_LINE_VALUE_INACTIVE);
        gpiod_line_request_release(dropRequest_);
    }

    if (startRequest_ != nullptr) {
        gpiod_line_request_set_value(
            startRequest_,
            startOffset_,
            GPIOD_LINE_VALUE_INACTIVE);
        gpiod_line_request_release(startRequest_);
    }
#endif

    if (chip_ != nullptr) {
        gpiod_chip_close(chip_);
    }
}

bool GpioController::init(const std::string& chipName, int startLine, int dropLine)
{
#ifdef HW11_GPIOD_V1
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
#else
    startOffset_ = static_cast<unsigned int>(startLine);
    dropOffset_ = static_cast<unsigned int>(dropLine);

    const std::string path = chipPath(chipName);
    chip_ = gpiod_chip_open(path.c_str());
    if (chip_ == nullptr) {
        std::cerr << "Cannot open GPIO chip: " << path << "\n";
        return false;
    }

    if (!requestOutputLine(chip_, startOffset_, "drone", startRequest_)) {
        std::cerr << "Cannot request START line\n";
        return false;
    }

    if (!requestOutputLine(chip_, dropOffset_, "drone", dropRequest_)) {
        std::cerr << "Cannot request DROP line\n";
        return false;
    }

    return true;
#endif
}

bool GpioController::setStart(bool value)
{
#ifdef HW11_GPIOD_V1
    return start_ != nullptr && gpiod_line_set_value(start_, value ? 1 : 0) == 0;
#else
    return startRequest_ != nullptr &&
           gpiod_line_request_set_value(
               startRequest_,
               startOffset_,
               value ? GPIOD_LINE_VALUE_ACTIVE
                     : GPIOD_LINE_VALUE_INACTIVE) == 0;
#endif
}

bool GpioController::setDrop(bool value)
{
#ifdef HW11_GPIOD_V1
    return drop_ != nullptr && gpiod_line_set_value(drop_, value ? 1 : 0) == 0;
#else
    return dropRequest_ != nullptr &&
           gpiod_line_request_set_value(
               dropRequest_,
               dropOffset_,
               value ? GPIOD_LINE_VALUE_ACTIVE
                     : GPIOD_LINE_VALUE_INACTIVE) == 0;
#endif
}
