#include "DMAstepgen.h"

#include <limits>


static constexpr int32_t accumulatorUnitsPerDmaSlot =
	RESOLUTION / 2;


/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

void createDMAstepgen()
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
    int stepLength = module["Step Length"];
    int stepSpace = module["Step Space"];
    int dirHold = module["Dir Hold"];
    int dirSetup = module["Dir Setup"];

    // configure pointers to data source and feedback location
    ptrJointFreqCmd[joint] = &rxData.jointFreqCmd[joint];
    ptrJointFeedback[joint] = &txData.jointFeedback[joint];
    ptrJointEnable = &rxData.jointEnable;

    // create the step generator, register it in the thread
    Module* stepgen = new DMAstepgen(DMA_FREQ, joint, step, dir, DMA_BUFFER_SIZE, STEPBIT, *ptrJointFreqCmd[joint], *ptrJointFeedback[joint], *ptrJointEnable, stepLength, stepSpace, dirHold, dirSetup);
    dmaThread->registerModule(stepgen);
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


DMAstepgen::DMAstepgen(int32_t threadFreq, int jointNumber, std::string step, std::string direction, int DMAbufferSize, int stepBit, volatile int32_t &ptrFrequencyCommand, volatile int32_t &ptrFeedback, volatile uint8_t &ptrJointEnable, uint8_t stepLength, uint8_t stepSpace, uint8_t dirHold, uint8_t dirSetup) :
	mask(0),
	jointNumber(jointNumber),
	step(step),
	direction(direction),
	DMAbufferSize(DMAbufferSize),
	stepBit(stepBit),
	ptrFrequencyCommand(&ptrFrequencyCommand),
	ptrFeedback(&ptrFeedback),
	ptrJointEnable(&ptrJointEnable),
	stepLength(stepLength),
	stepSpace(stepSpace),
	dirHold(dirHold),
	dirSetup(dirSetup),
	stepDMAbuffer(nullptr),
	stepDMAbuffer_0(nullptr),
	stepDMAbuffer_1(nullptr),
	dirDMAbuffer_0(nullptr),
	dirDMAbuffer_1(nullptr),
	stepDMAactiveBuffer(nullptr),
	dirDMAactiveBuffer(nullptr),
	stepMask(0),
	dirMask(0),
	isEnabled(false),
	dir(false),
	queuedDirection(false),
	pendingDirection(false),
	pendingDirectionValue(false),
	directionSetupPending(false),
	isStepping(false),
	hasLastStepFall(false),
	frequencyCommand(0),
	rawCount(0),
	accumulator(0),
	addValue(0),
	minAddValue(0),
	oldaddValue(0),
	remainder(0),
	prevRemainder(0),
	bufferStartSlot(0),
	lastStepFallSlot(0),
	pendingStepFallSlot(0),
	pendingDirectionSlot(0),
	directionSetupEndSlot(0),
	stepPin(nullptr),
	directionPin(nullptr),
	debug(nullptr)
{
	uint8_t pin, pin2;

	stepDMAbuffer_0 = &stepgenDMAbuffer_0[0];
	stepDMAbuffer_1 = &stepgenDMAbuffer_1[0];
	stepDMAactiveBuffer = &stepgenDMAbuffer;

	dirDMAbuffer_0 = &stepgenDMAbuffer_0[0];
	dirDMAbuffer_1 = &stepgenDMAbuffer_1[0];
	dirDMAactiveBuffer = &stepgenDMAbuffer;


	this->stepPin = new Pin(this->step, OUTPUT);
	this->directionPin = new Pin(this->direction, OUTPUT);
	this->accumulator = 0;
	this->remainder = 0;
	this->mask = uint32_t{1} << this->jointNumber;
	this->isEnabled = false;
	this->dir = false;

	if (this->stepLength == 0) this->stepLength = 1;
	if (this->stepSpace == 0) this->stepSpace = 1;
	if (this->dirHold == 0) this->dirHold = 1;
	if (this->dirSetup == 0) this->dirSetup = 1;

	this->minAddValue =
		(this->stepLength + this->stepSpace) *
		accumulatorUnitsPerDmaSlot;

	// determine the step pin number from the portAndPin string
	pin = this->step[3] - '0';
	pin2 = this->step[4] - '0';
	if (pin2 <= 9) pin = pin * 10 + pin2;
	this->stepMask = uint32_t{1} << pin;

	// determine the dir pin number from the portAndPin string
	pin = this->direction[3] - '0';
	pin2 = this->direction[4] - '0';
	if (pin2 <= 9) pin = pin * 10 + pin2;
	this->dirMask = uint32_t{1} << pin;
}


void DMAstepgen::update()
{
	// Use the standard Module interface to run makePulses()
	this->makePulses();
}

void DMAstepgen::updatePost()
{
	this->stopPulses();
}

void DMAstepgen::slowUpdate()
{
	return;
}

void DMAstepgen::makePulses()
{
	/*
	(1) steplen
	(2) stepspace
	(3) dirhold
	(4) dirsetup

			   _____         _____               _____
	STEP  ____/     \_______/     \_____________/     \______
			  |     |       |     |             |     |
	Time      |-(1)-|--(2)--|-(1)-|--(3)--|-(4)-|-(1)-|
										  |__________________
	DIR   ________________________________/

	 */


	this->isEnabled = ((*(this->ptrJointEnable) & this->mask) != 0);

	// A disabled buffer-0 prefill resets the timeline, so align buffer 1 here.
	if (DMA::isPrefillActive() &&
		(this->bufferStartSlot == 0U))
	{
		this->bufferStartSlot =
			static_cast<uint64_t>(
				this->DMAbufferSize);
	}

	this->selectWriteBuffer();

	const uint64_t bufferEndSlot =
		this->bufferStartSlot +
		static_cast<uint64_t>(
			this->DMAbufferSize);

	if (this->isEnabled)  									// this Step generator is enabled so make the pulses
	{
		// Complete a STEP pulse carried from the preceding writable buffer.
		if (this->isStepping &&
			(this->pendingStepFallSlot >= this->bufferStartSlot) &&
			(this->pendingStepFallSlot < bufferEndSlot))
		{
			const uint32_t stepLow =
				static_cast<uint32_t>(
					this->pendingStepFallSlot -
					this->bufferStartSlot);

			*(this->stepDMAbuffer + stepLow) |=
				this->stepMask;

			this->isStepping = false;
		}

		this->frequencyCommand = *(this->ptrFrequencyCommand);            		// Get the latest frequency command via pointer to the data source
		if (this->frequencyCommand != 0)
		{
			this->oldaddValue = this->addValue;

			const uint32_t frequencyMagnitude =
				(this->frequencyCommand < 0)
					? static_cast<uint32_t>(
						  -static_cast<int64_t>(
							  this->frequencyCommand))
					: static_cast<uint32_t>(
						  this->frequencyCommand);

			const uint64_t addValueNumerator =
				static_cast<uint64_t>(
					BUFFER_COUNTS) *
				static_cast<uint64_t>(
					PRU_SERVOFREQ);

			const uint64_t calculatedAddValue =
				addValueNumerator /
				static_cast<uint64_t>(
					frequencyMagnitude);

			const uint64_t maximumAddValue =
				static_cast<uint64_t>(
					INT32_MAX);

			this->addValue =
				static_cast<int32_t>(
					(calculatedAddValue > maximumAddValue)
						? maximumAddValue
						: calculatedAddValue);		// determine the add value from the commanded frequency ratio

			if (this->addValue < this->minAddValue)
			{
				this->addValue = this->minAddValue;			// limit the frequency to step requirements
			}

			this->dir =
				(this->frequencyCommand > 0);

			// A direction request can be cancelled before its toggle is queued.
			if (this->pendingDirection &&
				(this->dir != this->pendingDirectionValue))
			{
				this->pendingDirection = false;
			}

			if (!this->pendingDirection &&
				(this->dir != this->queuedDirection))
			{
				this->pendingDirection = true;
				this->pendingDirectionValue = this->dir;
				this->pendingDirectionSlot =
					this->bufferStartSlot;

				if (this->hasLastStepFall)
				{
					const uint64_t holdEndSlot =
						this->lastStepFallSlot +
						static_cast<uint64_t>(
							this->dirHold);

					if (this->pendingDirectionSlot <
						holdEndSlot)
					{
						this->pendingDirectionSlot =
							holdEndSlot;
					}
				}
			}

			if (this->pendingDirection &&
				(this->pendingDirectionSlot >=
					this->bufferStartSlot) &&
				(this->pendingDirectionSlot <
					bufferEndSlot))
			{
				const uint32_t directionSlot =
					static_cast<uint32_t>(
						this->pendingDirectionSlot -
						this->bufferStartSlot);

				*(this->stepDMAbuffer + directionSlot) |=
					this->dirMask;

				this->queuedDirection =
					this->pendingDirectionValue;
				this->pendingDirection = false;
				this->directionSetupPending = true;
				this->directionSetupEndSlot =
					this->pendingDirectionSlot +
					static_cast<uint64_t>(
						this->dirSetup);
			}

			if (this->pendingDirection ||
				(this->dir != this->queuedDirection))
			{
				this->advanceDdsWithoutStep();
				this->bufferStartSlot =
					bufferEndSlot;
				return;
			}

			uint64_t firstStepSlot =
				this->bufferStartSlot;

			if (this->directionSetupPending &&
				(firstStepSlot <
					this->directionSetupEndSlot))
			{
				firstStepSlot =
					this->directionSetupEndSlot;
			}

			// accumulator cannot go negative, so keep prevRemainder within limits
			if (this->prevRemainder > this->addValue)
			{
				this->prevRemainder = this->addValue;
			}

			if (this->addValue - this->prevRemainder <= BUFFER_COUNTS)
			{
				// at least one step in this period
				this->accumulator = this->addValue - this->prevRemainder;

				if (firstStepSlot >= bufferEndSlot)
				{
					this->advanceDdsWithoutStep();
					this->bufferStartSlot =
						bufferEndSlot;
					return;
				}

				const uint64_t firstStepAccumulator =
					(firstStepSlot -
						this->bufferStartSlot) *
					static_cast<uint64_t>(
						accumulatorUnitsPerDmaSlot);

				if (static_cast<uint64_t>(
						this->accumulator) <
					firstStepAccumulator)
				{
					this->accumulator =
						static_cast<int32_t>(
							firstStepAccumulator);
				}

				this->remainder = BUFFER_COUNTS - this->accumulator;

				const uint32_t firstStep =
					static_cast<uint32_t>(
						this->accumulator /
						accumulatorUnitsPerDmaSlot);

				if (firstStep <
					this->DMAbufferSize)
				{
					this->makeStep(
						firstStep,
						this->bufferStartSlot,
						bufferEndSlot);

					this->directionSetupPending = false;

					while (this->remainder >
						this->addValue)
					{
						// we can still step in this period
						this->accumulator =
							this->accumulator +
							this->addValue;
						this->remainder =
							BUFFER_COUNTS -
							this->accumulator;

						const uint32_t nextStep =
							static_cast<uint32_t>(
								this->accumulator /
								accumulatorUnitsPerDmaSlot);

						this->makeStep(
							nextStep,
							this->bufferStartSlot,
							bufferEndSlot);
					}

					// carry elapsed DDS units since the final STEP rise
					this->prevRemainder =
						this->remainder;

					*(this->ptrFeedback) = this->rawCount;                     // Update position feedback via pointer to the data receiver
				}
				else
				{
					// A STEP exactly at the boundary belongs to the next buffer.
					this->advanceDdsWithoutStep();
				}

				this->accumulator = 0;
			}
			else
			{
				this->advanceDdsWithoutStep();
			}
		}

		this->bufferStartSlot =
			bufferEndSlot;
	}
	else
	{
		// ensure the pin is in a know state as we're using DR_TOGGLE
		this->stepPin->set(0);
		this->directionPin->set(0);

		if (DMAthreadRunning ||
			DMA::isPrefillActive())
		{
			this->dir = false;
			this->queuedDirection = false;
			this->pendingDirection = false;
			this->directionSetupPending = false;
			this->prevRemainder = 0;
			this->isStepping = false;
			this->hasLastStepFall = false;
			this->lastStepFallSlot = 0;
			this->pendingStepFallSlot = 0;
			this->pendingDirectionSlot = 0;
			this->directionSetupEndSlot = 0;
			this->bufferStartSlot =
				bufferEndSlot;
		}
		else
		{
			this->dir = false;
			this->queuedDirection = false;
			this->pendingDirection = false;
			this->pendingDirectionValue = false;
			this->directionSetupPending = false;
			this->isStepping = false;
			this->hasLastStepFall = false;
			this->accumulator = 0;
			this->remainder = 0;
			this->prevRemainder = 0;
			this->bufferStartSlot = 0;
			this->lastStepFallSlot = 0;
			this->pendingStepFallSlot = 0;
			this->pendingDirectionSlot = 0;
			this->directionSetupEndSlot = 0;
		}
	}
}


void DMAstepgen::selectWriteBuffer()
{
	if (!*this->stepDMAactiveBuffer)
	{
		this->stepDMAbuffer =
			this->stepDMAbuffer_0;
	}
	else
	{
		this->stepDMAbuffer =
			this->stepDMAbuffer_1;
	}
}


void DMAstepgen::advanceDdsWithoutStep()
{
	const int64_t advancedRemainder =
		static_cast<int64_t>(
			this->prevRemainder) +
		static_cast<int64_t>(
			BUFFER_COUNTS);

	if (advancedRemainder >
		static_cast<int64_t>(
			this->addValue))
	{
		this->prevRemainder =
			this->addValue;
	}
	else
	{
		this->prevRemainder =
			static_cast<int32_t>(
				advancedRemainder);
	}
}


void DMAstepgen::makeStep(
	uint32_t stepHigh,
	uint64_t bufferStartSlot,
	uint64_t bufferEndSlot)
{
	*(this->stepDMAbuffer + stepHigh) |=
		this->stepMask;

	const uint64_t absoluteStepHigh =
		bufferStartSlot +
		static_cast<uint64_t>(
			stepHigh);

	this->pendingStepFallSlot =
		absoluteStepHigh +
		static_cast<uint64_t>(
			this->stepLength);

	this->lastStepFallSlot =
		this->pendingStepFallSlot;
	this->hasLastStepFall = true;

	// update the raw step count
	if (this->dir)
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

	if (this->pendingStepFallSlot < bufferEndSlot)
	{
		const uint32_t stepLow =
			static_cast<uint32_t>(
				this->pendingStepFallSlot -
				bufferStartSlot);

		*(this->stepDMAbuffer + stepLow) |=
			this->stepMask;

		this->isStepping = false;
	}
	else
	{
		this->isStepping = true;
	}
}


void DMAstepgen::stopPulses()
{

}


void DMAstepgen::setEnabled(bool state)
{
	this->isEnabled = state;
}
