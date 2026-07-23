#include "digitalPin.h"
#include "modules/inputBitUpdate.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

void createDigitalPin()
{
    const char* comment = module["Comment"];

    if ((comment != nullptr) &&
        (comment[0] != '\0'))
    {
        printf(
            "\n%s\n",
            comment);
    }

    const char* pin = module["Pin"];
    const char* mode = module["Mode"];
    const char* invert = module["Invert"] | "False";
    const char* modifier = module["Modifier"] | "None";
    int dataBit = module["Data Bit"];

    int mod;
    bool inv;

    if (!strcmp(modifier,"Open Drain"))
    {
        mod = OPENDRAIN;
    }
    else if (!strcmp(modifier,"Pull Up"))
    {
        mod = PULLUP;
    }
    else if (!strcmp(modifier,"Pull Down"))
    {
        mod = PULLDOWN;
    }
    else if (!strcmp(modifier,"Pull None"))
    {
        mod = PULLNONE;
    }
    else
    {
        mod = NONE;
    }

    if (!strcmp(invert,"True"))
    {
        inv = true;
    }
    else inv = false;

    ptrOutputs = &rxData.outputs;
    ptrInputs = &txData.inputs;

    printf("Make Digital %s at pin %s\n", mode, pin);

    if (!strcmp(mode,"Output"))
    {
        Module* digitalPin = new DigitalPin(*ptrOutputs, 1, pin, dataBit, inv, mod);
        servoThread->registerModule(digitalPin);
    }
    else if (!strcmp(mode,"Input"))
    {
        Module* digitalPin = new DigitalPin(*ptrInputs, 0, pin, dataBit, inv, mod);
        servoThread->registerModule(digitalPin);
    }
    else
    {
        printf("Error - incorrectly defined Digital Pin\n");
    }
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

DigitalPin::DigitalPin(volatile uint32_t &ptrData, int mode, std::string portAndPin, int bitNumber, bool invert, int modifier) :
	ptrData(&ptrData),
	bitNumber(bitNumber),
    invert(invert),
	mask(0),
	mode(mode),
	modifier(modifier),
	portAndPin(portAndPin),
	pin(nullptr)
{
	this->pin = new Pin(this->portAndPin, this->mode);		// Input 0x0, Output 0x1
	this->mask = uint32_t{1} << this->bitNumber;
}


void DigitalPin::update()
{
	bool pinState;

	if (this->mode == 0)									// the pin is configured as an input
	{
		pinState = this->pin->get();
		if(this->invert)
		{
			pinState = !pinState;
		}

		updateInputBit(
			*(this->ptrData),
			this->mask,
			pinState);
	}
	else												// the pin is configured as an output
	{
		pinState = *(this->ptrData) & this->mask;		// get the value of the bit in the data source
		if(this->invert)
		{
			pinState = !pinState;
		}
		this->pin->set(pinState);			// simple conversion to boolean
	}
}

void DigitalPin::slowUpdate()
{
	return;
}
