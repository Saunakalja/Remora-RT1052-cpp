#include "encoder.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/
void createEncoder()
{
    const char* comment = module["Comment"];
    printf("%s\n",comment);

    int pv = module["PV[i]"];
    const char* pinA = module["ChA Pin"];
    const char* pinB = module["ChB Pin"];
    const char* pinI = module["Index Pin"];
    int dataBit = module["Data Bit"];

    printf("Creating Quadrature Encoder at pins %s and %s\n", pinA, pinB);

    
    ptrProcessVariable[pv]  = &txData.processVariable[pv];
    ptrInputs = &txData.inputs;

    if (pinI == nullptr)
    {
        Module* encoder = new Encoder(*ptrProcessVariable[pv], pinA, pinB);
        baseThread->registerModule(encoder);
    }
    else
    {
        printf("  Encoder has index at pin %s\n", pinI);
        Module* encoder = new Encoder(*ptrProcessVariable[pv], *ptrInputs, dataBit, pinA, pinB, pinI);
        baseThread->registerModule(encoder);
    }
}

/***********************************************************************
*                METHOD DEFINITIONS                                    *
************************************************************************/

Encoder::Encoder(volatile float &ptrEncoderCount, std::string ChA, std::string ChB) :
	ChA(ChA),
	ChB(ChB),
	Index(),
	hasIndex(false),
	ptrData(nullptr),
	bitNumber(0),
	mask(0),
	ptrEncoderCount(&ptrEncoderCount),
	state(0),
	indexState(false),
	count(0),
	indexCount(0),
	indexPulse(0),
	pulseCount(0),
	pinA(nullptr),
	pinB(nullptr),
	pinI(nullptr)
{
	this->pinA = new Pin(this->ChA, INPUT);			// create Pin
    this->pinB = new Pin(this->ChB, INPUT);			// create Pin
    this->state = 0;

    if (this->pinA->get())
    {
        this->state |= 1U;
    }

    if (this->pinB->get())
    {
        this->state |= 2U;
    }
}

Encoder::Encoder(volatile float &ptrEncoderCount, volatile uint32_t &ptrData, int bitNumber, std::string ChA, std::string ChB, std::string Index) :
	ChA(ChA),
	ChB(ChB),
	Index(Index),
	hasIndex(true),
	ptrData(&ptrData),
	bitNumber(bitNumber),
	mask(0),
	ptrEncoderCount(&ptrEncoderCount),
	state(0),
	indexState(false),
	count(0),
	indexCount(0),
	indexPulse(0),
	pulseCount(0),
	pinA(nullptr),
	pinB(nullptr),
	pinI(nullptr)
{
	this->pinA = new Pin(this->ChA, INPUT);			// create Pin
    this->pinB = new Pin(this->ChB, INPUT);			// create Pin
    this->state = 0;

    if (this->pinA->get())
    {
        this->state |= 1U;
    }

    if (this->pinB->get())
    {
        this->state |= 2U;
    }

    this->pinI = new Pin(this->Index, INPUT);		// create Pin
    this->indexState = this->pinI->get();
    this->indexPulse = (PRU_BASEFREQ / PRU_SERVOFREQ) * 3;          // output the index pulse for 3 servo thread periods so LinuxCNC sees it
    this->mask = uint32_t{1} << this->bitNumber;
}

void Encoder::update()
{
    uint8_t s = this->state & 3;

    if (this->pinA->get()) s |= 4;
    if (this->pinB->get()) s |= 8;

    switch (s) {
		case 0: case 5: case 10: case 15:
			break;
		case 1: case 7: case 8: case 14:
			count++; break;
		case 2: case 4: case 11: case 13:
			count--; break;
		case 3: case 12:
			count += 2; break;
		default:
			count -= 2; break;
	}

	this->state = (s >> 2);

    if (this->hasIndex)                                     // we have an index pin
    {
        const bool indexHigh =
            this->pinI->get();

        const bool indexRising =
            indexHigh &&
            !this->indexState;

        this->indexState =
            indexHigh;

        // handle index, index pulse and pulse count
        if (indexRising && (this->pulseCount == 0U))         // rising edge on index pulse
        {
            this->indexCount = this->count;                 //  capture the encoder count at the index, send this to linuxCNC for one servo period 
            *(this->ptrEncoderCount) = this->indexCount;
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


// credit to https://github.com/PaulStoffregen/Encoder/blob/master/Encoder.h

//                           _______         _______       
//               PinA ______|       |_______|       |______ PinA
// negative <---         _______         _______         __      --> positive
//               PinB __|       |_______|       |_______|   PinB

		//	new	new	old	old
		//	pinB	pinA	pinB	pinA	Result
		//	----	----	----	----	------
		//	0	0	0	0	no movement
		//	0	0	0	1	+1
		//	0	0	1	0	-1
		//	0	0	1	1	+2  (assume pinA edges only)
		//	0	1	0	0	-1
		//	0	1	0	1	no movement
		//	0	1	1	0	-2  (assume pinA edges only)
		//	0	1	1	1	+1
		//	1	0	0	0	+1
		//	1	0	0	1	-2  (assume pinA edges only)
		//	1	0	1	0	no movement
		//	1	0	1	1	-1
		//	1	1	0	0	+2  (assume pinA edges only)
		//	1	1	0	1	-1
		//	1	1	1	0	+1
		//	1	1	1	1	no movement
/*
	// Simple, easy-to-read "documentation" version :-)
	//
	void update(void) {
		uint8_t s = state & 3;
		if (digitalRead(pinA)) s |= 4;
		if (digitalRead(pinB)) s |= 8;
		switch (s) {
			case 0: case 5: case 10: case 15:
				break;
			case 1: case 7: case 8: case 14:
				position++; break;
			case 2: case 4: case 11: case 13:
				position--; break;
			case 3: case 12:
				position += 2; break;
			default:
				position -= 2; break;
		}
		state = (s >> 2);
	}
*/
