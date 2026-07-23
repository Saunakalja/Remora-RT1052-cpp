#include <stdio.h>

#include "dma.h"

// DMA constructor
DMA::DMA(DMA_Type* DMAn, uint32_t frequency):
	DMAn(DMAn),
	frequency(frequency),
	tcd_0(0),
	tcd_1(0),
	pendingCompletionCount(0),
	pendingBuffer(0),
	expectedBuffer(0),
	completionFault(0),
	completionTracking(0),
	completionServiceActive(0)
{

}


void DMA::EDMA_Callback(
	edma_handle_t *handle,
	void *param,
	bool transferDone,
	uint32_t tcds)
{
	DMA* const dma =
		static_cast<DMA*>(param);

	if (dma != nullptr)
	{
		dma->publishCompletion(
			handle,
			transferDone,
			tcds);
	}
}


uint8_t DMA::decodeCompletedBuffer(
	uintptr_t tcdNext,
	bool transferDone) const
{
	if (tcdNext == this->tcd_0)
	{
		return transferDone
			? 1U
			: 0U;
	}

	if (tcdNext == this->tcd_1)
	{
		return transferDone
			? 0U
			: 1U;
	}

	return UINT8_MAX;
}


void DMA::publishCompletion(
	edma_handle_t *handle,
	bool transferDone,
	uint32_t tcds)
{
	static constexpr uint8_t invalidBuffer =
		UINT8_MAX;

	if ((this->completionTracking == 0U) ||
		(this->completionFault != 0U))
	{
		return;
	}

	if ((tcds > 1U) ||
		(this->pendingCompletionCount != 0U) ||
		(this->completionServiceActive != 0U))
	{
		this->latchCompletionFault(
			handle);
		return;
	}

	const uintptr_t tcdNext =
		static_cast<uintptr_t>(
			EDMA_GetNextTCDAddress(
				handle));

	const uint8_t completedBuffer =
		this->decodeCompletedBuffer(
			tcdNext,
			transferDone);

	if ((completedBuffer == invalidBuffer) ||
		(completedBuffer != this->expectedBuffer))
	{
		this->latchCompletionFault(
			handle);
		return;
	}

	this->pendingBuffer =
		completedBuffer;

	this->expectedBuffer =
		(completedBuffer == 0U)
			? 1U
			: 0U;

	__DMB();

	this->pendingCompletionCount = 1U;
}


void DMA::latchCompletionFault(
	edma_handle_t *handle)
{
	this->completionTracking = 0U;
	this->pendingCompletionCount = 2U;

	__DMB();

	this->completionFault = 1U;

	EDMA_StopTransfer(
		handle);
}


void DMA::armCompletionTracking(void)
{
	this->pendingCompletionCount = 0U;
	this->pendingBuffer = UINT8_MAX;
	this->expectedBuffer = 0U;
	this->completionFault = 0U;
	this->completionTracking = 1U;
	this->completionServiceActive = 0U;

	__DMB();
}


void DMA::configDMA(void)
{
	memset(stepgenDMAbuffer_0, 0, sizeof(stepgenDMAbuffer_0));
	memset(stepgenDMAbuffer_1, 0, sizeof(stepgenDMAbuffer_1));

	stepgenDMAbuffer = false;
	this->clearCompletionState();

	// The Periodic Interrupt Timer (PIT) module
	CLOCK_SetMux(kCLOCK_PerclkMux, 1U);
	CLOCK_SetDiv(kCLOCK_PerclkDiv, 0U);
	PIT_GetDefaultConfig(&this->pitConfig);
	PIT_Init(PIT, &this->pitConfig);

	// PIT channel 0
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, CLOCK_GetFreq(kCLOCK_OscClk)/(2 * this->frequency));
	PIT_StartTimer(PIT, kPIT_Chnl_0);

	/* Configure DMAMUX */
	DMAMUX_Init(DMAMUX);
	DMAMUX_EnableAlwaysOn(DMAMUX, STPGEN_DMA_CHANNEL, true);
	DMAMUX_EnableChannel(DMAMUX, STPGEN_DMA_CHANNEL);

	DMAMUX_EnablePeriodTrigger(DMAMUX, STPGEN_DMA_CHANNEL);

	EDMA_GetDefaultConfig(&this->userConfig);
	EDMA_Init(this->DMAn, &this->userConfig);
	EDMA_CreateHandle(&this->EDMA_Handle, this->DMAn, STPGEN_DMA_CHANNEL);
	EDMA_SetCallback(&this->EDMA_Handle, this->EDMA_Callback, this);
	EDMA_ResetChannel(this->EDMA_Handle.base, this->EDMA_Handle.channel);

	/* prepare descriptor 0 */
	EDMA_PrepareTransfer((edma_transfer_config_t *)&this->transferConfig, stepgenDMAbuffer_0, sizeof(stepgenDMAbuffer_0[0]), (uint32_t*)&GPIO1->DR_TOGGLE, sizeof(GPIO1->DR_TOGGLE),
						 sizeof(stepgenDMAbuffer_0[0]),
						 sizeof(stepgenDMAbuffer_0[0]) * DMA_BUFFER_SIZE,
						 kEDMA_MemoryToPeripheral);
	EDMA_TcdSetTransferConfig(tcdMemoryPoolPtr, &this->transferConfig, &tcdMemoryPoolPtr[1]);
	EDMA_TcdEnableInterrupts(&tcdMemoryPoolPtr[0], kEDMA_MajorInterruptEnable);

	/* prepare descriptor 1 */
	EDMA_PrepareTransfer((edma_transfer_config_t *)&this->transferConfig, stepgenDMAbuffer_1, sizeof(stepgenDMAbuffer_1[0]), (uint32_t*)&GPIO1->DR_TOGGLE, sizeof(GPIO1->DR_TOGGLE),
						 sizeof(stepgenDMAbuffer_1[0]),
						 sizeof(stepgenDMAbuffer_1[0]) * DMA_BUFFER_SIZE,
						 kEDMA_MemoryToPeripheral);
	EDMA_TcdSetTransferConfig(&tcdMemoryPoolPtr[1], &this->transferConfig, &tcdMemoryPoolPtr[0]);
	EDMA_TcdEnableInterrupts(&tcdMemoryPoolPtr[1], kEDMA_MajorInterruptEnable);

	EDMA_InstallTCD(this->DMAn, 0, tcdMemoryPoolPtr);

	this->tcd_0 = reinterpret_cast<uintptr_t>(&tcdMemoryPoolPtr[0]);
	this->tcd_1 = reinterpret_cast<uintptr_t>(&tcdMemoryPoolPtr[1]);
}


void DMA::startDMA(void)
{
	const uint32_t primask =
		__get_PRIMASK();

	__disable_irq();

	this->armCompletionTracking();
	EDMA_StartTransfer(&this->EDMA_Handle);

	__set_PRIMASK(
		primask);

	printf("   Starting DMA Stepgen\n");
}


void DMA::stopDMA(void)
{
	 this->clearCompletionState();
	 EDMA_AbortTransfer(&this->EDMA_Handle);
	 EDMA_Deinit(this->DMAn);
	 this->configDMA();
	 printf("   Stopping DMA Stepgen\n");
}


void DMA::updateBuffers(
	CompletionResult completedBuffer)
{
	if (completedBuffer == CompletionResult::Buffer0)
	{
		stepgenDMAbuffer = false;
		memset(stepgenDMAbuffer_0, 0, sizeof(stepgenDMAbuffer_0));
	}
	else if (completedBuffer == CompletionResult::Buffer1)
	{
		stepgenDMAbuffer = true;
		memset(stepgenDMAbuffer_1, 0, sizeof(stepgenDMAbuffer_1));
	}
}


DMA::CompletionResult DMA::takeCompletion(void)
{
	const uint32_t primask =
		__get_PRIMASK();

	__disable_irq();

	CompletionResult result =
		CompletionResult::None;

	if (this->completionFault != 0U)
	{
		result =
			CompletionResult::Fault;
	}
	else if (this->pendingCompletionCount == 1U)
	{
		result =
			(this->pendingBuffer == 0U)
				? CompletionResult::Buffer0
				: CompletionResult::Buffer1;

		this->pendingCompletionCount = 0U;
		this->pendingBuffer = UINT8_MAX;
		this->completionServiceActive = 1U;
	}

	__set_PRIMASK(
		primask);

	return result;
}


bool DMA::completeBufferService(void)
{
	const uint32_t primask =
		__get_PRIMASK();

	__disable_irq();

	this->completionServiceActive = 0U;

	const bool faulted =
		(this->completionFault != 0U);

	__set_PRIMASK(
		primask);

	return faulted;
}


void DMA::clearCompletionState(void)
{
	const uint32_t primask =
		__get_PRIMASK();

	__disable_irq();

	this->completionTracking = 0U;
	this->pendingCompletionCount = 0U;
	this->pendingBuffer = UINT8_MAX;
	this->expectedBuffer = 0U;
	this->completionFault = 0U;
	this->completionServiceActive = 0U;

	__DMB();

	__set_PRIMASK(
		primask);
}
