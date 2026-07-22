#include "pin.h"
#include <cstdio>
#include <cerrno>
#include <string>


static bool parsePortAndPin(
    const std::string &portAndPin,
    uint8_t &port,
    uint16_t &pin)
{
    const size_t length =
        portAndPin.size();

    if ((length != 4U) &&
        (length != 5U))
    {
        return false;
    }

    if ((portAndPin[0] != 'P') ||
        (portAndPin[1] < '1') ||
        (portAndPin[1] > '4') ||
        (portAndPin[2] != '_') ||
        (portAndPin[3] < '0') ||
        (portAndPin[3] > '9'))
    {
        return false;
    }

    uint16_t parsedPin =
        static_cast<uint16_t>(
            portAndPin[3] - '0');

    if (length == 5U)
    {
        if ((portAndPin[4] < '0') ||
            (portAndPin[4] > '9'))
        {
            return false;
        }

        parsedPin =
            static_cast<uint16_t>(
                parsedPin * 10U +
                static_cast<uint16_t>(
                    portAndPin[4] - '0'));
    }

    if (parsedPin > 31U)
    {
        return false;
    }

    port =
        static_cast<uint8_t>(
            portAndPin[1] - '0');

    pin =
        parsedPin;

    return true;
}


static GPIO_Type *gpioForPort(
    uint8_t port)
{
    if (port == 1U)
    {
        return GPIO1;
    }
    else if (port == 2U)
    {
        return GPIO2;
    }
    else if (port == 3U)
    {
        return GPIO3;
    }
    else if (port == 4U)
    {
        return GPIO4;
    }

    return nullptr;
}


Pin::Pin(std::string portAndPin, int dir) :
    portAndPin(portAndPin),
    dir(0U),
    port(0U),
    pin(0U),
    config{
        kGPIO_DigitalInput,
        0U,
        kGPIO_NoIntmode},
    GPIOx(nullptr),
    mask(0U),
    valid(false)
{
    // Set direction
    if (dir == INPUT)
    {
        this->dir = static_cast<uint8_t>(dir);
    	this->config.direction = kGPIO_DigitalInput;
    	this->config.outputLogic = 1U;
    	this->config.interruptMode = kGPIO_NoIntmode;
    }
    else if (dir == OUTPUT)
    {
        this->dir = static_cast<uint8_t>(dir);
    	this->config.direction = kGPIO_DigitalOutput;
    	this->config.outputLogic = 0U;
    	this->config.interruptMode = kGPIO_NoIntmode;
    }
    else
    {
        printf("  Invalid port and pin definition\n");
        return;
    }


    printf("Creating Pin @\n");

    uint8_t parsedPort =
        0U;

    uint16_t parsedPin =
        0U;

    if (!parsePortAndPin(
            this->portAndPin,
            parsedPort,
            parsedPin))
    {
        printf("  Invalid port and pin definition\n");
        return;
    }

    GPIO_Type *parsedGPIO =
        gpioForPort(
            parsedPort);

    if (parsedGPIO == nullptr)
    {
        printf("  Invalid port and pin definition\n");
        return;
    }

    this->port =
        parsedPort;

    this->pin =
        parsedPin;

    this->GPIOx =
        parsedGPIO;

    this->mask =
        uint32_t{1} << this->pin;

    printf("  port = GPIO%d\n", this->port);
    printf("  pin = %d\n", this->pin);

    GPIO_PinInit(this->GPIOx, this->pin, &this->config);
    this->valid =
        true;
}


