#ifndef REMORACOMMS_H
#define REMORACOMMS_H

#include <cstdio>

#include "configuration.h"
#include "remora.h"

#include "../module.h"
#include "../../drivers/pin/pin.h"

class RemoraComms : public Module
{
  private:

	volatile bool	data;
	volatile bool	status;

	uint32_t	noDataCount;
	uint32_t	noDataLimit;

	Pin*		CommsPin;

  public:

	RemoraComms(uint32_t servoFrequency);

	virtual void update(void);
	void dataReceived();
	void resetCommunication();
	bool getStatus();
};




#endif
