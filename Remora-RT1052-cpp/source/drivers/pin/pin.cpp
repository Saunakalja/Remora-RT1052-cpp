#include "pin.h"
#include "fsl_iomuxc.h"
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <string>


static constexpr uint32_t pinConfigRegisterFromTuple(
    uint32_t,
    uint32_t,
    uint32_t,
    uint32_t,
    uint32_t configRegister)
{
    return configRegister;
}


static constexpr uint32_t padRegisterStride =
    sizeof(uint32_t);

static constexpr uint32_t gpio1FirstPadRegister =
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_AD_B0_00_GPIO1_IO00);

static constexpr uint32_t gpio2FirstPadRegister =
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_B0_00_GPIO2_IO00);

static constexpr uint32_t gpio3FirstPadRegister =
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_SD_B1_00_GPIO3_IO00);

static constexpr uint32_t gpio3SecondPadRegister =
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_SD_B0_00_GPIO3_IO12);

static constexpr uint32_t gpio3ThirdPadRegister =
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_EMC_32_GPIO3_IO18);

static constexpr uint32_t gpio4FirstPadRegister =
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_EMC_00_GPIO4_IO00);

static_assert(
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_AD_B1_15_GPIO1_IO31) ==
        gpio1FirstPadRegister +
        31U * padRegisterStride,
    "GPIO1 pad register range is not contiguous");

static_assert(
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_B1_15_GPIO2_IO31) ==
        gpio2FirstPadRegister +
        31U * padRegisterStride,
    "GPIO2 pad register range is not contiguous");

static_assert(
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_SD_B1_11_GPIO3_IO11) ==
        gpio3FirstPadRegister +
        11U * padRegisterStride,
    "GPIO3 IO00-IO11 pad register range is not contiguous");

static_assert(
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_SD_B0_05_GPIO3_IO17) ==
        gpio3SecondPadRegister +
        5U * padRegisterStride,
    "GPIO3 IO12-IO17 pad register range is not contiguous");

static_assert(
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_EMC_41_GPIO3_IO27) ==
        gpio3ThirdPadRegister +
        9U * padRegisterStride,
    "GPIO3 IO18-IO27 pad register range is not contiguous");

static_assert(
    pinConfigRegisterFromTuple(
        IOMUXC_GPIO_EMC_31_GPIO4_IO31) ==
        gpio4FirstPadRegister +
        31U * padRegisterStride,
    "GPIO4 pad register range is not contiguous");


static bool parsePortAndPin(
    const char *portAndPin,
    size_t length,
    uint8_t &port,
    uint16_t &pin)
{
    if ((portAndPin == nullptr) ||
        ((length != 4U) &&
         (length != 5U)))
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


static bool padConfigRegisterForGpio(
    uint8_t port,
    uint16_t pin,
    uint32_t &configRegister)
{
    configRegister =
        0U;

    if ((port == 1U) &&
        (pin <= 31U))
    {
        configRegister =
            gpio1FirstPadRegister +
            static_cast<uint32_t>(pin) *
            padRegisterStride;
    }
    else if ((port == 2U) &&
             (pin <= 31U))
    {
        configRegister =
            gpio2FirstPadRegister +
            static_cast<uint32_t>(pin) *
            padRegisterStride;
    }
    else if ((port == 3U) &&
             (pin <= 11U))
    {
        configRegister =
            gpio3FirstPadRegister +
            static_cast<uint32_t>(pin) *
            padRegisterStride;
    }
    else if ((port == 3U) &&
             (pin >= 12U) &&
             (pin <= 17U))
    {
        configRegister =
            gpio3SecondPadRegister +
            static_cast<uint32_t>(
                pin - 12U) *
            padRegisterStride;
    }
    else if ((port == 3U) &&
             (pin >= 18U) &&
             (pin <= 27U))
    {
        configRegister =
            gpio3ThirdPadRegister +
            static_cast<uint32_t>(
                pin - 18U) *
            padRegisterStride;
    }
    else if ((port == 4U) &&
             (pin <= 31U))
    {
        configRegister =
            gpio4FirstPadRegister +
            static_cast<uint32_t>(pin) *
            padRegisterStride;
    }

    return (configRegister != 0U) &&
           ((configRegister %
             padRegisterStride) == 0U);
}


static bool modifiedPadControlValue(
    PinModifier modifier,
    uint32_t currentValue,
    uint32_t &modifiedValue)
{
    const uint32_t pullControlMask =
        IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
        IOMUXC_SW_PAD_CTL_PAD_PUS_MASK;

    modifiedValue =
        currentValue;

    switch (modifier)
    {
        case PinModifier::None:
            return true;

        case PinModifier::OpenDrain:
            modifiedValue |=
                IOMUXC_SW_PAD_CTL_PAD_ODE(1U);
            return true;

        case PinModifier::PullUp:
            modifiedValue =
                (currentValue &
                 ~pullControlMask) |
                IOMUXC_SW_PAD_CTL_PAD_PKE(1U) |
                IOMUXC_SW_PAD_CTL_PAD_PUE(1U) |
                IOMUXC_SW_PAD_CTL_PAD_PUS(2U);
            return true;

        case PinModifier::PullDown:
            modifiedValue =
                (currentValue &
                 ~pullControlMask) |
                IOMUXC_SW_PAD_CTL_PAD_PKE(1U) |
                IOMUXC_SW_PAD_CTL_PAD_PUE(1U) |
                IOMUXC_SW_PAD_CTL_PAD_PUS(0U);
            return true;

        case PinModifier::PullNone:
            modifiedValue =
                currentValue &
                ~pullControlMask;
            return true;
    }

    return false;
}


bool parsePinModifier(
    const char *modifierName,
    PinModifier &modifier)
{
    if (modifierName == nullptr)
    {
        return false;
    }

    if (!strcmp(
            modifierName,
            "None"))
    {
        modifier =
            PinModifier::None;
    }
    else if (!strcmp(
                 modifierName,
                 "Open Drain"))
    {
        modifier =
            PinModifier::OpenDrain;
    }
    else if (!strcmp(
                 modifierName,
                 "Pull Up"))
    {
        modifier =
            PinModifier::PullUp;
    }
    else if (!strcmp(
                 modifierName,
                 "Pull Down"))
    {
        modifier =
            PinModifier::PullDown;
    }
    else if (!strcmp(
                 modifierName,
                 "Pull None"))
    {
        modifier =
            PinModifier::PullNone;
    }
    else
    {
        return false;
    }

    return true;
}


bool pinModifierIsCompatible(
    PinModifier modifier,
    int direction)
{
    switch (modifier)
    {
        case PinModifier::None:
            return (direction == INPUT) ||
                   (direction == OUTPUT);

        case PinModifier::OpenDrain:
            return direction == OUTPUT;

        case PinModifier::PullUp:
        case PinModifier::PullDown:
        case PinModifier::PullNone:
            return direction == INPUT;
    }

    return false;
}


bool pinHasPadConfigRegister(
    const char *portAndPin)
{
    if (portAndPin == nullptr)
    {
        return false;
    }

    uint8_t port =
        0U;

    uint16_t pin =
        0U;

    if (!parsePortAndPin(
            portAndPin,
            strlen(portAndPin),
            port,
            pin))
    {
        return false;
    }

    uint32_t configRegister =
        0U;

    return padConfigRegisterForGpio(
        port,
        pin,
        configRegister);
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


Pin::Pin(
    std::string portAndPin,
    int dir,
    PinModifier modifier) :
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
    if (!pinModifierIsCompatible(
            modifier,
            dir))
    {
        printf("  Invalid port and pin definition\n");
        return;
    }

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
            this->portAndPin.c_str(),
            this->portAndPin.size(),
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

    if (modifier != PinModifier::None)
    {
        uint32_t padConfigRegister =
            0U;

        if (!padConfigRegisterForGpio(
                this->port,
                this->pin,
                padConfigRegister))
        {
            printf("  No pad configuration for pin\n");
            return;
        }

        volatile uint32_t *const padControl =
            reinterpret_cast<volatile uint32_t *>(
                padConfigRegister);

        const uint32_t currentPadControl =
            *padControl;

        uint32_t newPadControl =
            currentPadControl;

        if (!modifiedPadControlValue(
                modifier,
                currentPadControl,
                newPadControl))
        {
            printf("  Invalid pin modifier\n");
            return;
        }

        IOMUXC_SetPinConfig(
            0U,
            0U,
            0U,
            0U,
            padConfigRegister,
            newPadControl);
    }

    printf("  port = GPIO%d\n", this->port);
    printf("  pin = %d\n", this->pin);

    GPIO_PinInit(this->GPIOx, this->pin, &this->config);
    this->valid =
        true;
}


