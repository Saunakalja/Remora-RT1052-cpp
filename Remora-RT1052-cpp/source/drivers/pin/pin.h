#ifndef PIN_H
#define PIN_H

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <string>

#include "MIMXRT1052.h"
#include "fsl_gpio.h"

#define INPUT 0x0
#define OUTPUT 0x1

enum class PinModifier : uint8_t
{
    None = 0U,
    OpenDrain = 1U,
    PullUp = 2U,
    PullDown = 3U,
    PullNone = 4U
};

bool parsePinModifier(
    const char *modifierName,
    PinModifier &modifier);

bool pinModifierIsCompatible(
    PinModifier modifier,
    int direction);

bool pinHasPadConfigRegister(
    const char *portAndPin);

class Pin
{
    private:

        std::string         portAndPin;
        uint8_t             dir;
        uint8_t             port;
        uint16_t            pin;
        gpio_pin_config_t   config;
        GPIO_Type *			GPIOx;
        uint32_t            mask;
        bool                valid;

    public:

        Pin(
            std::string,
            int,
            PinModifier modifier =
                PinModifier::None);

        inline bool get()
        {
            if ((!this->valid) ||
                (this->GPIOx == nullptr))
            {
                return false;
            }

            return GPIO_PinRead(this->GPIOx, this->pin);
        }

        inline void set(bool value)
        {
            if ((!this->valid) ||
                (this->GPIOx == nullptr))
            {
                return;
            }

        	GPIO_PinWrite(this->GPIOx, this->pin, value);
        }
};

#endif
