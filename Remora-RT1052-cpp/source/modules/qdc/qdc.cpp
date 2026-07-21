
#include "qdc.h"

#include <cstring>

volatile bool initXBARA = true;
Module* qdc[MAX_INST_QDC_MOD] = {nullptr,nullptr,nullptr,nullptr};

bool muxPinsXBAR(const char* pin,xbar_output_signal_t kXBARA1_OutputEncInput)
{
  uint8_t mux_op_pin = 0;

  if (pin == nullptr)
  {
	printf("********The Qdc phase pin is missing********\r\n");
	printf("********The instance of the Qdc module could not be carried out********\r\n");
	return false;
  }

  if(!strcmp(pin,"P4_13"))
	mux_op_pin = 1;
  else if(!strcmp(pin,"P4_14"))
	mux_op_pin = 2;
  else if(!strcmp(pin,"P3_21"))
    mux_op_pin = 3;
  else if(!strcmp(pin,"P4_00") && !strcmp(board,"EC500"))
    mux_op_pin = 4;
  else if(!strcmp(pin,"P3_23"))
    mux_op_pin = 5;
  else if(!strcmp(pin,"P4_15"))
    mux_op_pin = 6;
  else if(!strcmp(pin,"P3_16"))
  {
	mux_op_pin = 7;
	printf("P3_16 is only 5V tolerant, danger!!!!!!\r\n");
  }
  else if(!strcmp(pin,"P3_17"))
  {
	mux_op_pin = 8;
	printf("P3_17 is only 5V tolerant, danger!!!!!!\r\n");
  }
  else if(!strcmp(pin,"P3_22"))
  {
	mux_op_pin = 9;
	printf("P3_22 is only 5V tolerant, danger!!!!!!\r\n");
  }
  else if(!strcmp(pin,"P4_16") && !strcmp(board,"EC300"))
  {
	mux_op_pin = 10;
  }
  else
  {
	printf("********The %s pin cannot be multiplexed pad(pin)-/->XBAR-->Qdc********\r\n",pin);
	printf("********The instance of the Qdc module could not be carried out********\r\n");
	return false;
  }

  switch(mux_op_pin)
  {
    case 1:
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_13_XBAR1_IN25, 1U);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_13_XBAR1_IN25, 0x10B0U);

		XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarIn25, kXBARA1_OutputEncInput);
		break;
    case 2:
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_14_XBAR1_INOUT19, 1U);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_14_XBAR1_INOUT19, 0x10B0U);

		XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarInout19, kXBARA1_OutputEncInput);
		break;
    case 3:
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_35_XBAR1_INOUT18, 1U);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_35_XBAR1_INOUT18, 0x10B0U);

		XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarInout18, kXBARA1_OutputEncInput);
		break;
    case 4:
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_00_XBAR1_IN02, 1U);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_00_XBAR1_IN02, 0x10B0U);

		XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarIn02, kXBARA1_OutputEncInput);
		break;
    case 5:
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_37_XBAR1_IN23, 1U);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_37_XBAR1_IN23, 0x10B0U);

		XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarIn23, kXBARA1_OutputEncInput);
		break;
    case 6:
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_15_XBAR1_IN20, 1U);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_15_XBAR1_IN20, 0x10B0U);

		XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarIn20, kXBARA1_OutputEncInput);
		break;
    case 7:
		IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_04_XBAR1_INOUT08, 1U);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_04_XBAR1_INOUT08, 0x10B0U);

		XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarInout08, kXBARA1_OutputEncInput);
		break;
    case 8:
		IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_05_XBAR1_INOUT09, 1U);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_05_XBAR1_INOUT09, 0x10B0U);

		XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarInout09, kXBARA1_OutputEncInput);
		break;
    case 9:
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_36_XBAR1_IN22, 1U);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_36_XBAR1_IN22, 0x10B0U);

		XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarIn22, kXBARA1_OutputEncInput);
		break;
    case 10:
		IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_16_XBAR1_IN21, 1U);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_16_XBAR1_IN21, 0x10B0U);

		XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputIomuxXbarIn21, kXBARA1_OutputEncInput);
		break;
    default:
        return false;
  }

  return true;
}

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON
************************************************************************/
bool createQdc()
{
    const char* comment = module["Comment"];
    printf("%s\n",comment);

    int pv = module["PV[i]"];
    const char* pinA = module["ChA Pin"];
    const char* pinB = module["ChB Pin"];
    const char* pinI = module["Index Pin"];
    int dataBit = module["Data Bit"];
    int encNumber = module["ENC No"];
    int filt_per = module["Filter PER"];
    int filt_cnt = module["Filter CNT"];

    ENC_Type* encBase = nullptr;
    xbar_output_signal_t phaseAInput = kXBARA1_OutputEnc1PhaseAInput;
    xbar_output_signal_t phaseBInput = kXBARA1_OutputEnc1PhaseBInput;
    GPIO_Type* gpioBase = nullptr;
    IRQn_Type IndexIrqGpioPinId = NotAvail_IRQn;
    uint8_t indexPortNumber = 0U;
    uint8_t indexPinInNumber = 0U;

    if ((pv < 0) ||
        (pv >= VARIABLES))
    {
        printf("QDC PV index %d is invalid\r\n", pv);
        return false;
    }

    if ((encNumber < 1) ||
        (encNumber > MAX_INST_QDC_MOD))
    {
        printf("QDC ENC No %d is invalid\r\n", encNumber);
        return false;
    }

    if (qdc[encNumber - 1] != nullptr)
    {
        printf("QDC ENC No %d is already configured\r\n", encNumber);
        return false;
    }

    if ((pinA == nullptr) ||
        (pinB == nullptr))
    {
        printf("QDC phase pin configuration is missing\r\n");
        return false;
    }

    switch(encNumber)
    {
      case(1):
		phaseAInput = kXBARA1_OutputEnc1PhaseAInput;
		phaseBInput = kXBARA1_OutputEnc1PhaseBInput;
		encBase = ENC1;
        break;
      case(2):
		phaseAInput = kXBARA1_OutputEnc2PhaseAInput;
		phaseBInput = kXBARA1_OutputEnc2PhaseBInput;
		encBase = ENC2;
        break;
      case(3):
		phaseAInput = kXBARA1_OutputEnc3PhaseAInput;
		phaseBInput = kXBARA1_OutputEnc3PhaseBInput;
		encBase = ENC3;
        break;
      case(4):
		phaseAInput = kXBARA1_OutputEnc4PhaseAInput;
		phaseBInput = kXBARA1_OutputEnc4PhaseBInput;
		encBase = ENC4;
        break;
      default:
        break;
    }

    if (!(pinI == nullptr))
    {
        if ((dataBit < 0) ||
            (dataBit >= 32))
        {
            printf("QDC index Data Bit %d is invalid\r\n", dataBit);
            return false;
        }

        if ((strlen(pinI) != 5U) ||
            (pinI[0] != 'P') ||
            ((pinI[1] != '3') &&
             (pinI[1] != '4')) ||
            (pinI[2] != '_') ||
            (pinI[3] < '0') ||
            (pinI[3] > '9') ||
            (pinI[4] < '0') ||
            (pinI[4] > '9'))
        {
            printf(
                "QDC index pin %s is invalid\r\n",
                pinI);
            return false;
        }

        indexPinInNumber =
            (pinI[3] - '0') * 10U +
            (pinI[4] - '0');

        if (indexPinInNumber > 31U)
        {
            printf("QDC index pin %s is invalid\r\n", pinI);
            return false;
        }

        if(pinI[1] == '3')
        {
            gpioBase = GPIO3;
            indexPortNumber = 3;
            printf("Index GPIO Pin Number: GPIO3_%d\n",indexPinInNumber);
            if(indexPinInNumber < 16)
            {
                IndexIrqGpioPinId = GPIO3_Combined_0_15_IRQn;
            }
            else
            {
                IndexIrqGpioPinId = GPIO3_Combined_16_31_IRQn;
            }
        }
        else
        {
            gpioBase = GPIO4;
            indexPortNumber = 4;
            printf("Index GPIO Pin Number: GPIO4_%d\n",indexPinInNumber);
            if(indexPinInNumber < 16)
            {
                IndexIrqGpioPinId = GPIO4_Combined_0_15_IRQn;
            }
            else
            {
                IndexIrqGpioPinId = GPIO4_Combined_16_31_IRQn;
            }
        }
    }

    if(initXBARA == true)
    {
        XBARA_Init(XBARA1);
        initXBARA = false;

    }

    if (!muxPinsXBAR(pinA, phaseAInput) ||
        !muxPinsXBAR(pinB, phaseBInput))
    {
        printf("QDC creation failed during phase-pin multiplexing\r\n");
        return false;
    }


    ptrProcessVariable[pv]  = &txData.processVariable[pv];
    ptrInputs = &txData.inputs;

    if (pinI == nullptr)
    {
        printf("  Quadrature Encoder without index pin %s\n", pinI);
        qdc[encNumber-1] = new Qdc(*ptrProcessVariable[pv],encBase, filt_per, filt_cnt);
        baseThread->registerModule(qdc[encNumber-1]);
    }
    else
    {
        printf("  Quadrature Encoder has index at pin %s\n", pinI);
        qdc[encNumber-1] = new Qdc(*ptrProcessVariable[pv], *ptrInputs, encBase, gpioBase, IndexIrqGpioPinId, indexPortNumber, indexPinInNumber, dataBit, filt_per, filt_cnt);
        NVIC_SetPriority(IndexIrqGpioPinId , 4);
        baseThread->registerModule(qdc[encNumber-1]);
    }

    return true;
}

/***********************************************************************
*                METHOD DEFINITIONS                                    *
************************************************************************/

Qdc::Qdc(volatile float &ptrEncoderCount, ENC_Type* encBase, int filt_per, int filt_cnt):
	ptrEncoderCount(&ptrEncoderCount),
	encBase(encBase),
	filt_per(filt_per),
	filt_cnt(filt_cnt)
{
    enc_config_t mEncConfigStruct;

    /* Initialize the ENC module. */
    ENC_GetDefaultConfig(&mEncConfigStruct);
    mEncConfigStruct.filterSamplePeriod = this->filt_per;
    mEncConfigStruct.filterCount = this->filt_cnt;
    ENC_Init(this->encBase, &mEncConfigStruct);
    ENC_DoSoftwareLoadInitialPositionValue(this->encBase); /* Update the position counter with initial value. */

    this->hasIndex = false;
    this->count = 0;								// initialise the count to 0
}

Qdc::Qdc(volatile float &ptrEncoderCount, volatile uint32_t &ptrData, ENC_Type* encBase,
		GPIO_Type* gpioBase, IRQn_Type irq, int indexPortNumber, int indexPinInNumber,
		int bitNumber, int filt_per, int filt_cnt) :
	ptrEncoderCount(&ptrEncoderCount),
    ptrData(&ptrData),
    encBase(encBase),
	gpioBase(gpioBase),
	irq(irq),
	indexPortNumber(indexPortNumber),
	indexPinInNumber(indexPinInNumber),
    bitNumber(bitNumber),
	filt_per(filt_per),
	filt_cnt(filt_cnt)
{

    interruptPtr = new portInterrupt(this);

    enc_config_t mEncConfigStruct;
    /* Initialize the ENC module. */
    ENC_GetDefaultConfig(&mEncConfigStruct);
    mEncConfigStruct.filterSamplePeriod = this->filt_per;
    mEncConfigStruct.filterCount = this->filt_cnt;
    ENC_Init(this->encBase, &mEncConfigStruct);
    ENC_DoSoftwareLoadInitialPositionValue(this->encBase); /* Update the position counter with initial value. */

    this->hasIndex = true;
    this->indexPulse = (PRU_BASEFREQ / PRU_SERVOFREQ) * 3;          // output the index pulse for 3 servo thread periods so LinuxCNC sees it
    this->indexCount = 0;
    this->count = 0;								                // initialise the count to 0
    this->pulseCount = 0U;                                          // number of base thread periods to pulse the index output
    this->mask =
        uint32_t{1} << this->bitNumber;
    this->indexDetected = false;
}

void Qdc::update()
{
  this->count = ENC_GetPositionValue(this->encBase);

  if (this->hasIndex)                                     // we have an index pin
  {
      bool indexCaptured = false;
      int32_t capturedIndexCount = 0;

      if (this->pulseCount == 0U)
      {
          DisableIRQ(this->irq);

          if (this->indexDetected)
          {
              __DMB();

              capturedIndexCount =
                  this->indexCount;

              this->indexDetected = false;
              indexCaptured = true;
          }

          EnableIRQ(this->irq);
      }

      // handle index, index pulse and pulse count
      if (indexCaptured)                                  // index interrupt occured: rising edge on index pulse
      {
          *(this->ptrEncoderCount) =
              capturedIndexCount;
          this->pulseCount = this->indexPulse;
          *(this->ptrData) |= this->mask;                 // set bit in data source high
      }
      else if (this->pulseCount > 0U)                     // maintain both index output and encoder count for the latch period
      {
          this->pulseCount--;                             // decrement the counter
      }
      else
      {
          *(this->ptrData) &= ~this->mask;                // set bit in data source low
          *(this->ptrEncoderCount) = this->count;         // update encoder count
      }
  }
  else
  {
      *(this->ptrEncoderCount) = this->count;             // update encoder count
  }
}

void Qdc::indexEvent()
{
	this->indexCount =
		ENC_GetPositionValue(
			this->encBase);

	__DMB();

	this->indexDetected = true;
}

void Qdc::disableInterrupt()
{
	if(this->hasIndex)
	{
		printf("	Disabling Index Gpio Irq: %d\n",this->irq);
		GPIO_PortDisableInterrupts(this->gpioBase,1<<this->indexPinInNumber);
		DisableIRQ(this->irq);
	}
}

void Qdc::enableInterrupt()
{
    if (this->hasIndex)
    {
        printf(
            "\tEnabling Index Gpio Irq: %d\n",
            this->irq);

        GPIO_PortClearInterruptFlags(
            this->gpioBase,
            1U << this->indexPinInNumber);

        GPIO_PortEnableInterrupts(
            this->gpioBase,
            1U << this->indexPinInNumber);

        EnableIRQ(this->irq);
    }
}
