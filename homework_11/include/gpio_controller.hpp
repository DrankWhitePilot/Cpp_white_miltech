#pragma once

#include <gpiod.h>
#include <string>

class GpioController
{
public:
    GpioController() = default;
    ~GpioController();

    GpioController(const GpioController&) = delete;
    GpioController& operator=(const GpioController&) = delete;

    bool init(const std::string& chipName, int startLine, int dropLine);
    bool setStart(bool value);
    bool setDrop(bool value);

private:
    gpiod_chip* chip_ = nullptr;

#ifdef HW11_GPIOD_V1
    gpiod_line* start_ = nullptr;
    gpiod_line* drop_ = nullptr;
#else
    gpiod_line_request* startRequest_ = nullptr;
    gpiod_line_request* dropRequest_ = nullptr;
    unsigned int startOffset_ = 0;
    unsigned int dropOffset_ = 0;
#endif
};
