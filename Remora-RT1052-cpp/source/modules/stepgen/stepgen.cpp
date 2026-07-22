#include "stepgen.h"

#include <limits>


/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

void createStepgen()
{
    const char* comment =
        module["Comment"];

    if ((comment != nullptr) &&
        (comment[0] != '\0'))
    {
        printf(
            "\n%s\n",
            comment);
    }

    int joint = module["Joint Number"];
    const char* step = module["Step Pin"];
    const char* dir = module["Direction Pin"];

    // configure pointers to data source and feedback location
    ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    // create the step generator, register it in the thread
    Module* stepgen = new Stepgen(base_freq, joint, step, dir, STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable);
    baseThread->registerModule(stepgen);
    baseThread->registerModulePost(stepgen);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

static int32_t incrementRawCount(
    int32_t rawCount)
{
    if (rawCount == std::numeric_limits<int32_t>::max())
    {
        return std::numeric_limits<int32_t>::min();
    }

    return rawCount + 1;
}


static int32_t decrementRawCount(
    int32_t rawCount)
{
    if (rawCount == std::numeric_limits<int32_t>::min())
    {
        return std::numeric_limits<int32_t>::max();
    }

    return rawCount - 1;
}


Stepgen::Stepgen(int32_t threadFreq, int jointNumber, std::string step, std::string direction, int stepBit, volatile int32_t &ptrFrequencyCommand, volatile int32_t &ptrFeedback, volatile uint8_t &ptrJointEnable) :
	jointNumber(jointNumber),
	mask(0U),
	step(step),
	direction(direction),
	isEnabled(false),
	isForward(false),
	frequencyCommand(0),
	ptrFrequencyCommand(&ptrFrequencyCommand),
	rawCount(0),
	ptrFeedback(&ptrFeedback),
	ptrJointEnable(&ptrJointEnable),
	DDSaccumulator(0),
	frequencyScale(0.0F),
	DDSaddValue(0),
	stepBit(stepBit),
	stepMask(0U),
	maxFrequencyCommand(0),
	isValid(false),
	stepPin(nullptr),
	directionPin(nullptr)
{
	this->stepPin = new Pin(this->step, OUTPUT);
	this->directionPin = new Pin(this->direction, OUTPUT);

	if ((threadFreq <= 0) ||
		(this->stepBit < 0) ||
		(this->stepBit > 30) ||
		(this->jointNumber < 0) ||
		(this->jointNumber >= 32))
	{
		return;
	}

	this->maxFrequencyCommand =
		threadFreq;

	this->stepMask =
		uint32_t{1} <<
		static_cast<uint32_t>(
			this->stepBit);

	this->frequencyScale =
		static_cast<float>(
			this->stepMask) /
		static_cast<float>(
			threadFreq);

	this->mask =
		uint32_t{1} <<
		static_cast<uint32_t>(
			this->jointNumber);

	this->isValid =
		true;
}


void Stepgen::update()
{
	// Use the standard Module interface to run makePulses()
	this->makePulses();
}

void Stepgen::updatePost()
{
	this->stopPulses();
}

void Stepgen::slowUpdate()
{
	return;
}

void Stepgen::makePulses()
{
	uint32_t stepNow = 0U;

	if (!this->isValid)
	{
		this->isEnabled = false;
		return;
	}

	this->isEnabled =
		((*(this->ptrJointEnable) & this->mask) != 0U);

	if (this->isEnabled == true)  												// this Step generator is enables so make the pulses
	{
		int32_t command =
			*(this->ptrFrequencyCommand);            							// Get the latest frequency command via pointer to the data source

		if (command > this->maxFrequencyCommand)
		{
			command =
				this->maxFrequencyCommand;
		}
		else if (command < -this->maxFrequencyCommand)
		{
			command =
				-this->maxFrequencyCommand;
		}

		this->frequencyCommand =
			command;

		this->DDSaddValue =
			static_cast<int32_t>(
				static_cast<float>(
					this->frequencyCommand) *
				this->frequencyScale);											// Scale the frequency command to get the DDS add value
		stepNow = this->DDSaccumulator;                           				// Save the current DDS accumulator value
		this->DDSaccumulator +=
			static_cast<uint32_t>(
				this->DDSaddValue);           	  								// Update the DDS accumulator with the new add value
		stepNow ^= this->DDSaccumulator;                          				// Test for changes in the low half of the DDS accumulator
		stepNow &= this->stepMask;                         						// Check for the step bit
		//this->rawCount = this->DDSaccumulator >> this->stepBit;   				// Update the position raw count

		if (this->DDSaddValue > 0)												// The sign of the DDS add value indicates the desired direction
		{
			this->isForward = true;
		}
		else //if (this->DDSaddValue < 0)
		{
			this->isForward = false;
		}

		if (stepNow)
		{
			this->directionPin->set(this->isForward);             		    // Set direction pin
			this->stepPin->set(true);										// Raise step pin - A4988 / DRV8825 stepper drivers only need 200ns setup time
            if (this->isForward)
            {
                this->rawCount =
					incrementRawCount(
						this->rawCount);
            }
            else
            {
                this->rawCount =
					decrementRawCount(
						this->rawCount);
            }
            *(this->ptrFeedback) = this->rawCount;							// Update position feedback via pointer to the data receiver
		}
	}


}


void Stepgen::stopPulses()
{
	if (this->stepPin != nullptr)
	{
		this->stepPin->set(false);	// Reset step pin
	}
}


void Stepgen::setEnabled(bool state)
{
	this->isEnabled = state;
}
