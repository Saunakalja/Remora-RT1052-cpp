#include "RemoraComms.h"


RemoraComms::RemoraComms() :
	data(false),
	status(false),
	noDataCount(0),
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

	if (this->noDataCount > DATA_ERR_MAX)
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

