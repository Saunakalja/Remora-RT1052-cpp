#ifndef DMA_H
#define DMA_H

#include "MIMXRT1052.h"
#include <stdint.h>

#include "fsl_pit.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"

#include "extern.h"

#define STPGEN_DMA_CHANNEL       0U	// NVMGP uses channel 1 and 2


class DMA
{
	public:

		enum class CompletionResult : uint8_t
		{
			None,
			Buffer0,
			Buffer1,
			Fault
		};

	private:

		DMA_Type* 	    		DMAn;
		uint32_t 				frequency;

		edma_transfer_config_t 	transferConfig;
		edma_config_t 			userConfig;
		pit_config_t 			pitConfig;

		uintptr_t 				tcd_0, tcd_1;

		volatile uint8_t		pendingCompletionCount;
		volatile uint8_t		pendingBuffer;
		volatile uint8_t		expectedBuffer;
		volatile uint8_t		completionFault;
		volatile uint8_t		completionTracking;
		volatile uint8_t		completionServiceActive;

		static void EDMA_Callback(
			edma_handle_t *handle,
			void *param,
			bool transferDone,
			uint32_t tcds);

		uint8_t decodeCompletedBuffer(
			uintptr_t tcdNext,
			bool transferDone) const;
		void publishCompletion(
			edma_handle_t *handle,
			bool transferDone,
			uint32_t tcds);
		void latchCompletionFault(
			edma_handle_t *handle);
		void armCompletionTracking(void);
		void prefillSecondBuffer(void);

		static bool prefillActive;

	public:

		DMA(DMA_Type*, uint32_t);
		void configDMA(void);
		void startDMA(void);
        void stopDMA(void);
        void updateBuffers(CompletionResult completedBuffer);
		CompletionResult takeCompletion(void);
		bool completeBufferService(void);
		void clearCompletionState(void);
		static bool isPrefillActive(void);

		edma_handle_t 			EDMA_Handle;
};

#endif
