#pragma once

#include <string>

#include <gpiod.h>

class GpioController
{
public:
    GpioController() = default;
    ~GpioController();

    GpioController(const GpioController&) = delete;
    GpioController& operator=(const GpioController&) = delete;

    bool init(const std::string& chipName, int startLine, int dropLine);
    bool setStart(bool value);
    bool pulseDrop(int usec = 80000);

private:
    gpiod_chip* chip_ = nullptr;
    gpiod_line* start_ = nullptr;
    gpiod_line* drop_ = nullptr;
};
