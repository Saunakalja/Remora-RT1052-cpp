#ifndef INPUTBITUPDATE_H
#define INPUTBITUPDATE_H

#include <cstdint>

#include "fsl_common.h"

static inline void updateInputBit(
	volatile uint32_t &inputWord,
	uint32_t mask,
	bool set)
{
	const uint32_t primask =
		__get_PRIMASK();

	__disable_irq();

	uint32_t value =
		inputWord;

	if (set)
	{
		value |= mask;
	}
	else
	{
		value &= ~mask;
	}

	inputWord =
		value;

	__DMB();

	__set_PRIMASK(
		primask);
}

#endif
