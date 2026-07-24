#include "RemoraComms.h"


static uint32_t calculateNoDataLimit(
	uint32_t servoFrequency)
{
	const uint64_t timeoutProduct =
		static_cast<uint64_t>(servoFrequency) *
		static_cast<uint64_t>(COMMS_TIMEOUT_US);

	const uint64_t missedCycles =
		(timeoutProduct + 1000000ULL - 1ULL) /
		1000000ULL;

	return static_cast<uint32_t>(
		(missedCycles == 0U)
			? 1U
			: missedCycles);
}


RemoraComms::RemoraComms(uint32_t servoFrequency) :
	data(false),
	status(false),
	noDataCount(0),
	noDataLimit(calculateNoDataLimit(servoFrequency)),
	CommsPin(nullptr)
{
	printf("Creating an Ethernet communication monitoring module\n");


	this->CommsPin = new Pin(LED, OUTPUT);
	this->CommsPin->set(this->status);

}


void RemoraComms::update()
{
	if (data)
	{
		this->noDataCount = 0;
		this->status = true;
		this->CommsPin->set(!this->status);
	}
	else
	{
		this->noDataCount++;
	}

	if (this->noDataCount >= this->noDataLimit)
	{
		this->noDataCount = 0;
		this->status = false;
		this->CommsPin->set(!this->status);
	}

	this->data = false;
}



void RemoraComms::dataReceived()
{
	const uint32_t primask =
		__get_PRIMASK();

	__disable_irq();

	this->data= true;

	__set_PRIMASK(
		primask);
}


void RemoraComms::resetCommunication()
{
	const uint32_t primask =
		__get_PRIMASK();

	__disable_irq();

	this->data = false;
	this->status = false;
	this->noDataCount = 0;

	__set_PRIMASK(
		primask);

	this->CommsPin->set(
		!this->status);
}


bool RemoraComms::getStatus()
{
	const uint32_t primask =
		__get_PRIMASK();

	__disable_irq();

	const bool currentStatus =
		this->status;

	__set_PRIMASK(
		primask);

	return currentStatus;
}

