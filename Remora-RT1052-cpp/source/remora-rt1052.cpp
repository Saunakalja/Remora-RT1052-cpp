/*
Remora firmware for Novusun RT1052 based CNC controller boards
to allow use with LinuxCNC.

Copyright (C) 2023 Scott Alford aka scotta

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 3
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; If not, see <http://www.gnu.org/licenses/>.
*/


#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#include "ethernet.h"

#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "tftpserver.h"

#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_pit.h"
#include "flexspi_nor_flash.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"

#include "configuration.h"
#include "remora.h"

#include "crc32.h"

// libraries
#include <cstring>
#include <sys/errno.h>
#include "lib/ArduinoJson6/ArduinoJson-v6.21.6.h"

// drivers
#include "drivers/pin/pin.h"

// interrupts
#include "interrupt/irqHandlers.h"
#include "interrupt/interrupt.h"

// threads
#include "thread/pruThread.h"
#include "thread/createThreads.h"

// modules
#include "modules/module.h"
#include "modules/blink/blink.h"
//#include "modules/debug/debug.h"
#include "modules/DMAstepgen/DMAstepgen.h"
#include "modules/encoder/encoder.h"
#include "modules/qdc/qdc.h"
#include "modules/comms/RemoraComms.h"
#include "modules/pwm/spindlePwm.h"
#include "modules/stepgen/stepgen.h"
#include "modules/digitalPin/digitalPin.h"
#include "modules/nvmpg/nvmpg.h"


bool controlSessionHandoffPending(void);
void controlSessionHandoffComplete(void);


// state machine
enum State {
    ST_SETUP = 0,
    ST_START,
    ST_IDLE,
    ST_RUNNING,
    ST_STOP,
    ST_RESET,
    ST_WDRESET
};

uint8_t resetCnt;
uint32_t base_freq = PRU_BASEFREQ;
uint32_t servo_freq = PRU_SERVOFREQ;
uint32_t dma_freq = DMA_FREQ;

// boolean
bool configError = false;
bool hasBaseThread = false;
bool hasServoThread = false;
bool hasDMAthread = false;
bool hasQDC = false;
bool threadsRunning = false;
bool DMAthreadRunning = false;


// DMA stepgen double buffers
AT_NONCACHEABLE_SECTION_INIT(int32_t stepgenDMAbuffer_0[DMA_BUFFER_SIZE]);		// double buffers for port DMA transfers
AT_NONCACHEABLE_SECTION_INIT(int32_t stepgenDMAbuffer_1[DMA_BUFFER_SIZE]);
vector<Module*> vDMAthread;
vector<Module*>::iterator iterDMA;
bool DMAstepgen = false;
bool stepgenDMAbuffer = false;					// indicates which double buffer to use 0 or 1

//edma_handle_t stepgen_EDMA_Handle;

AT_QUICKACCESS_SECTION_DATA_ALIGN(edma_tcd_t tcdMemoryPoolPtr[3], sizeof(edma_tcd_t));
//int32_t tcd_0, tcd_1, tcd_next;

// pointers to objects with global scope
pruThread* servoThread;
pruThread* baseThread;
pruThread* dmaThread;
RemoraComms* comms;
Module* MPG;

// unions for RX, TX and MPG data
volatile rxData_t rxData;
volatile txData_t txData;
volatile bool cmdReceived;
volatile bool mpgReceived;
mpgData_t mpgData;

// pointers to data
volatile rxData_t*  ptrRxData = &rxData;
volatile txData_t*  ptrTxData = &txData;
volatile int32_t* ptrTxHeader;
volatile int32_t* ptrJointFreqCmd[JOINTS];
volatile int32_t* ptrJointFeedback[JOINTS];
volatile uint8_t* ptrJointEnable;
volatile float*   ptrSetPoint[VARIABLES];
volatile float*   ptrProcessVariable[VARIABLES];
const char*       processVariableOwner[VARIABLES] = {nullptr};
volatile uint32_t* ptrInputs;
volatile uint32_t* ptrOutputs;
volatile uint16_t* ptrNVMPGInputs;
volatile mpgData_t* ptrMpgData = &mpgData;


// JSON config file stuff

const char defaultConfig[] = DEFAULT_CONFIG;
static_assert(
	sizeof(defaultConfig) <= JSON_BUFF_SIZE,
	"Default JSON configuration exceeds JSON buffer");

typedef struct
{
  uint32_t crc32;   		// crc32 of JSON
  uint32_t length;			// length in words for CRC calculation
  uint32_t jsonLength;  	// length in of JSON config in bytes
  uint8_t padding[500];
} metadata_t;

volatile bool newJson;
uint32_t crc32;
FILE *jsonFile;
string strJson;
DynamicJsonDocument doc(JSON_BUFF_SIZE);
JsonObject thread;
const char* board;
JsonObject module;


int8_t checkJson()
{
	SCB_InvalidateDCache_by_Addr(
		reinterpret_cast<volatile void*>(
			XIP_BASE +
			JSON_UPLOAD_ADDRESS),
		static_cast<int32_t>(
			METADATA_LEN +
			JSON_BUFF_SIZE));

	metadata_t* meta = (metadata_t*)(XIP_BASE + JSON_UPLOAD_ADDRESS);
	const uint32_t jsonLength =
		meta->jsonLength;

    uint32_t table[256];
    crc32::generate_table(table);

	if ((jsonLength == 0U) ||
		(jsonLength > JSON_BUFF_SIZE))
	{
		newJson = false;
		printf("JSON Config byte length incorrect\n");
		return -1;
	}

    // For compatibility with STM32 hardware CRC32, the config is padded to a 32-bit (4-byte) boundary.
	const uint32_t paddedJsonLength =
		(jsonLength + 3U) &
		~uint32_t{3U};

	const uint32_t padding =
		paddedJsonLength - jsonLength;

	if (paddedJsonLength > JSON_BUFF_SIZE)
	{
		newJson = false;
		printf("JSON Config padded length incorrect\n");
		return -1;
	}

	const uint32_t expectedWordLength =
		paddedJsonLength /
		sizeof(uint32_t);

	if (meta->length != expectedWordLength)
	{
		newJson = false;
		printf("JSON Config word length incorrect\n");
		return -1;
	}

	// Compute CRC
    crc32 = 0;
    char* ptr = (char *)(XIP_BASE + JSON_UPLOAD_ADDRESS + METADATA_LEN);
    for (uint32_t i = 0;
		 i < paddedJsonLength;
		 i++)
    {
        crc32 = crc32::update(table, crc32, ptr, 1);
        ptr++;
    }

	printf(
		"Length (words) = %lu\n",
		static_cast<unsigned long>(meta->length));
	printf(
		"JSON length (bytes) = %lu\n",
		static_cast<unsigned long>(jsonLength));
	printf(
		"Padding = %lu\n",
		static_cast<unsigned long>(padding));
	printf(
		"Calculated crc32 = %lx\n",
		static_cast<unsigned long>(crc32));

	// Check CRC
	if (crc32 != meta->crc32)
	{
		newJson = false;
		printf("JSON Config file CRC incorrect\n");
		return -1;
	}

	// JSON is OK, don't check it again
	newJson = false;
	printf("JSON Config file received Ok\n");
	return 1;
}


bool moveJson()
{
	uint32_t pages;
	uint32_t sectors;
	uint32_t i = 0U;

	metadata_t* meta =
		reinterpret_cast<metadata_t*>(
			XIP_BASE + JSON_UPLOAD_ADDRESS);

	const uint32_t jsonLength =
		meta->jsonLength;

	uint32_t Flash_Write_Address;

	if ((jsonLength == 0U) ||
		(jsonLength > JSON_BUFF_SIZE))
	{
		printf("JSON Config byte length incorrect\n");
		return false;
	}

    // how many pages are needed to be written. The first 4 bytes of the storage location will contain the length of the JSON file
	const uint32_t storedLength =
		sizeof(uint32_t) +
		jsonLength;

	pages =
		(storedLength +
		 FLASH_PAGE_SIZE -
		 1U) /
		FLASH_PAGE_SIZE;

	const uint32_t programmedLength =
		pages *
		FLASH_PAGE_SIZE;

	sectors =
		(programmedLength +
		 SECTOR_SIZE -
		 1U) /
		SECTOR_SIZE;

    printf(
		"pages = %lu, sectors = %lu\n",
		static_cast<unsigned long>(pages),
		static_cast<unsigned long>(sectors));

    //The buffer argument points to the data to be written, which is of size size.
    //This size must be a multiple of the "page size", which is defined as the constant FLASH_PAGE_SIZE, with a value of 256 bytes.

	alignas(uint32_t)
	uint8_t pageData[FLASH_PAGE_SIZE];

	const uint8_t* uploadedJson =
		reinterpret_cast<const uint8_t*>(
			XIP_BASE +
			JSON_UPLOAD_ADDRESS +
			METADATA_LEN);

	// erase the old JSON config file

	// init flash
	flexspi_nor_flash_init(FLEXSPI);

	// Enter quad mode
	status_t status = flexspi_nor_enable_quad_mode(FLEXSPI);
	if (status != kStatus_Success)
	{
	  PRINTF("Enable quad mode failure !\r\n");
	  return false;
	}

	Flash_Write_Address = JSON_STORAGE_ADDRESS;

	for (i = 0; i < sectors; i++)
	{
		status = flexspi_nor_flash_erase_sector(FLEXSPI, Flash_Write_Address + (i * SECTOR_SIZE));
		if (status != kStatus_Success)
		{
		  PRINTF("Erase sector failure !\r\n");
		  return false;
		}
	}

	for (i = 0; i < pages; i++)
	{
		memset(pageData, 0, sizeof(pageData));

		const uint32_t pageOffset =
			i * FLASH_PAGE_SIZE;

		const uint32_t destinationOffset =
			(i == 0U) ?
			sizeof(uint32_t) :
			0U;

		const uint32_t sourceOffset =
			(i == 0U) ?
			0U :
			pageOffset - sizeof(uint32_t);

		if (i == 0U)
		{
			pageData[0] =
				static_cast<uint8_t>(
					jsonLength);

			pageData[1] =
				static_cast<uint8_t>(
					jsonLength >> 8);

			pageData[2] =
				static_cast<uint8_t>(
					jsonLength >> 16);

			pageData[3] =
				static_cast<uint8_t>(
					jsonLength >> 24);
		}

		if (sourceOffset < jsonLength)
		{
			const uint32_t remaining =
				jsonLength -
				sourceOffset;

			const uint32_t capacity =
				FLASH_PAGE_SIZE -
				destinationOffset;

			const uint32_t bytesToCopy =
				(remaining < capacity) ?
				remaining :
				capacity;

			memcpy(
				pageData + destinationOffset,
				uploadedJson + sourceOffset,
				bytesToCopy);
		}

		status =
			flexspi_nor_flash_page_program(
				FLEXSPI,
				Flash_Write_Address +
					i * FLASH_PAGE_SIZE,
				reinterpret_cast<uint32_t*>(
					pageData));

		if (status != kStatus_Success)
		{
		 PRINTF("Page program failure !\r\n");
		 return false;
		}
	}

	printf("Configuration file moved\n");
	return true;

}


void jsonFromFlash()
{
    printf("\n1. Loading JSON configuration file from Flash memory\n");

	strJson.clear();

    // read word 0 to determine length to read
    const uint32_t storedJsonLength =
		*reinterpret_cast<const uint32_t*>(
			XIP_BASE +
			JSON_STORAGE_ADDRESS);

    if (storedJsonLength == 0xFFFFFFFFU)
    {
    	printf("Flash storage location is empty - no config file\n");
        printf("Using basic default configuration - 3 step generators only\n");

		strJson.assign(
			defaultConfig,
			sizeof(defaultConfig));
    }
    else if ((storedJsonLength == 0U) ||
			 (storedJsonLength > JSON_BUFF_SIZE))
    {
		printf(
			"Stored JSON length invalid: %lu\n",
			static_cast<unsigned long>(storedJsonLength));
        printf("Using basic default configuration - 3 step generators only\n");

		strJson.assign(
			defaultConfig,
			sizeof(defaultConfig));
    }
    else
    {
		const char* storedJson =
			reinterpret_cast<const char*>(
				XIP_BASE +
				JSON_STORAGE_ADDRESS +
				sizeof(uint32_t));

		strJson.assign(
			storedJson,
			storedJsonLength);
    }

	printf(
		"\n%s\n",
		strJson.c_str());
}


void deserialiseJSON()
{
    printf("\n2. Parsing JSON configuration file\n");

    const char *json = strJson.c_str();

    // parse the json configuration file
    DeserializationError error = deserializeJson(doc, json);

    printf("Config deserialisation - ");

    switch (error.code())
    {
        case DeserializationError::Ok:
            printf("Deserialization succeeded\n");
            break;
        case DeserializationError::InvalidInput:
            printf("Invalid input!\n");
            configError = true;
            break;
        case DeserializationError::NoMemory:
            printf("Not enough memory\n");
            configError = true;
            break;
        default:
            printf("Deserialization failed\n");
            configError = true;
            break;
    }
}

static constexpr uint32_t gpioPortCount =
    4U;

static constexpr uint32_t gpioPinsPerPort =
    32U;

static constexpr uint32_t gpioRegistrySize =
    gpioPortCount *
    gpioPinsPerPort;

struct GpioIdentity
{
    uint8_t port;
    uint8_t pin;
};

struct GpioOwner
{
    const char* ownerType;
    const char* role;
    uint32_t moduleEntry;
    bool fixed;
};

struct FixedGpioReservation
{
    GpioIdentity identity;
    const char* ownerType;
    const char* role;
};

static const FixedGpioReservation fixedGpioReservations[] =
{
    {{2U, 16U}, "Debug Console", "LPUART4 TX"},
    {{2U, 17U}, "Debug Console", "LPUART4 RX"},
    {{2U, 20U}, "ENET", "RX Data 0"},
    {{2U, 21U}, "ENET", "RX Data 1"},
    {{2U, 22U}, "ENET", "RX Enable"},
    {{2U, 23U}, "ENET", "TX Data 0"},
    {{2U, 24U}, "ENET", "TX Data 1"},
    {{2U, 25U}, "ENET", "TX Enable"},
    {{2U, 26U}, "ENET", "Reference Clock"},
    {{2U, 27U}, "ENET", "RX Error"},
    {{2U, 28U}, "NVMPG UART", "LPUART5 TX"},
    {{2U, 29U}, "NVMPG UART", "LPUART5 RX"},
    {{2U, 30U}, "ENET", "MDC"},
    {{2U, 31U}, "ENET", "MDIO"},
    {{3U, 0U}, "RemoraComms", "Status LED"},
    {{3U, 5U}, "FlexSPI/XIP", "A DQS"},
    {{3U, 6U}, "FlexSPI/XIP", "A SS0"},
    {{3U, 7U}, "FlexSPI/XIP", "A SCLK"},
    {{3U, 8U}, "FlexSPI/XIP", "A Data 0"},
    {{3U, 9U}, "FlexSPI/XIP", "A Data 1"},
    {{3U, 10U}, "FlexSPI/XIP", "A Data 2"},
    {{3U, 11U}, "FlexSPI/XIP", "A Data 3"}
};

static bool parseGpioPinName(
    const char* pinName,
    GpioIdentity &identity)
{
    identity =
        {0U, 0U};

    if (pinName == nullptr)
    {
        return false;
    }

    const size_t pinNameLength =
        strlen(pinName);

    if ((pinNameLength != 4U) &&
        (pinNameLength != 5U))
    {
        return false;
    }

    if ((pinName[0] != 'P') ||
        (pinName[1] < '1') ||
        (pinName[1] > '4') ||
        (pinName[2] != '_') ||
        (pinName[3] < '0') ||
        (pinName[3] > '9'))
    {
        return false;
    }

    uint32_t pinNumber =
        static_cast<uint32_t>(
            pinName[3] - '0');

    if (pinNameLength == 5U)
    {
        if ((pinName[4] < '0') ||
            (pinName[4] > '9'))
        {
            return false;
        }

        pinNumber =
            pinNumber * 10U +
            static_cast<uint32_t>(
                pinName[4] - '0');
    }

    if (pinNumber >= gpioPinsPerPort)
    {
        return false;
    }

    identity.port =
        static_cast<uint8_t>(
            pinName[1] - '0');

    identity.pin =
        static_cast<uint8_t>(
            pinNumber);

    return true;
}

static bool gpioIdentitiesReferToSamePin(
    const GpioIdentity &first,
    const GpioIdentity &second)
{
    return (first.port == second.port) &&
           (first.pin == second.pin);
}

static bool claimGpioOwnership(
    GpioOwner gpioOwners[gpioRegistrySize],
    const GpioIdentity &identity,
    const char* ownerType,
    const char* role,
    uint32_t moduleEntry,
    bool fixed)
{
    if ((identity.port < 1U) ||
        (identity.port > gpioPortCount) ||
        (identity.pin >= gpioPinsPerPort))
    {
        printf(
            "Internal GPIO ownership identity P%u_%u is invalid\n",
            static_cast<unsigned int>(identity.port),
            static_cast<unsigned int>(identity.pin));
        return false;
    }

    const uint32_t gpioKey =
        (static_cast<uint32_t>(
             identity.port) -
         1U) *
        gpioPinsPerPort +
        static_cast<uint32_t>(
            identity.pin);

    GpioOwner &existingOwner =
        gpioOwners[gpioKey];

    if (existingOwner.ownerType != nullptr)
    {
        if (fixed)
        {
            if (existingOwner.fixed)
            {
                printf(
                    "Fixed GPIO P%u_%u %s %s conflicts with fixed %s %s\n",
                    static_cast<unsigned int>(identity.port),
                    static_cast<unsigned int>(identity.pin),
                    ownerType,
                    role,
                    existingOwner.ownerType,
                    existingOwner.role);
            }
            else
            {
                printf(
                    "Fixed GPIO P%u_%u %s %s conflicts with %s %s from module entry %lu\n",
                    static_cast<unsigned int>(identity.port),
                    static_cast<unsigned int>(identity.pin),
                    ownerType,
                    role,
                    existingOwner.ownerType,
                    existingOwner.role,
                    static_cast<unsigned long>(
                        existingOwner.moduleEntry));
            }
        }
        else if (existingOwner.fixed)
        {
            printf(
                "GPIO P%u_%u %s %s from module entry %lu conflicts with fixed %s %s\n",
                static_cast<unsigned int>(identity.port),
                static_cast<unsigned int>(identity.pin),
                ownerType,
                role,
                static_cast<unsigned long>(
                    moduleEntry),
                existingOwner.ownerType,
                existingOwner.role);
        }
        else
        {
            printf(
                "GPIO P%u_%u %s %s from module entry %lu conflicts with %s %s from module entry %lu\n",
                static_cast<unsigned int>(identity.port),
                static_cast<unsigned int>(identity.pin),
                ownerType,
                role,
                static_cast<unsigned long>(
                    moduleEntry),
                existingOwner.ownerType,
                existingOwner.role,
                static_cast<unsigned long>(
                    existingOwner.moduleEntry));
        }

        return false;
    }

    existingOwner.ownerType =
        ownerType;

    existingOwner.role =
        role;

    existingOwner.moduleEntry =
        moduleEntry;

    existingOwner.fixed =
        fixed;

    return true;
}

void getBoardType()
{
	if (configError) return;

	JsonVariantConst boardValue =
		doc["Board"];

	if (!boardValue.is<const char*>())
	{
		board = nullptr;
		printf(
			"\n3. Board Type is missing or is not a string\n");
		configError = true;
		return;
	}

	const char* configuredBoard =
		boardValue.as<const char*>();

	if ((configuredBoard == nullptr) ||
		(configuredBoard[0] == '\0'))
	{
		board = nullptr;
		printf(
			"\n3. Board Type is empty\n");
		configError = true;
		return;
	}

	board = configuredBoard;

	printf("\n3. Board Type: %s\n",board);
}

void configThreads()
{
    if (configError) return;

    printf("\n4. Configuring threads\n");

    if (!doc.containsKey("Threads"))
    {
        return;
    }

    JsonVariantConst threadsValue =
        doc["Threads"];

    if (!threadsValue.is<JsonArrayConst>())
    {
        printf("Threads configuration is not an array\n");
        configError = true;
        return;
    }

    uint32_t configuredBaseFreq =
        base_freq;

    uint32_t configuredServoFreq =
        servo_freq;

    bool baseConfigured = false;
    bool servoConfigured = false;

    const uint32_t timerClock =
        CLOCK_GetFreq(
            kCLOCK_PerClk);

    if (timerClock == 0U)
    {
        printf("Timer peripheral clock is zero\n");
        configError = true;
        return;
    }

    JsonArrayConst Threads =
        threadsValue.as<JsonArrayConst>();

    // create objects from JSON data
    uint32_t threadIndex = 0U;
    for (JsonArrayConst::iterator it=Threads.begin(); it!=Threads.end(); ++it)
    {
        JsonVariantConst threadValue =
            *it;

        if (!threadValue.is<JsonObjectConst>())
        {
            printf(
                "Thread entry %lu is not an object\n",
                static_cast<unsigned long>(threadIndex));
            configError = true;
            return;
        }

        JsonObjectConst threadObject =
            threadValue.as<JsonObjectConst>();

        JsonVariantConst threadNameValue =
            threadObject["Thread"];

        if (!threadNameValue.is<const char*>())
        {
            printf(
                "Thread entry %lu name is missing or is not a string\n",
                static_cast<unsigned long>(threadIndex));
            configError = true;
            return;
        }

        const char* configor =
            threadNameValue.as<const char*>();

        if ((configor == nullptr) ||
            (configor[0] == '\0'))
        {
            printf(
                "Thread entry %lu name is empty\n",
                static_cast<unsigned long>(threadIndex));
            configError = true;
            return;
        }

        JsonVariantConst frequencyValue =
            threadObject["Frequency"];

        if (!frequencyValue.is<uint32_t>())
        {
            printf(
                "Thread entry %lu frequency is missing or is not an unsigned integer\n",
                static_cast<unsigned long>(threadIndex));
            configError = true;
            return;
        }

        const uint32_t frequency =
            frequencyValue.as<uint32_t>();

        if ((frequency == 0U) ||
            (frequency > timerClock))
        {
            printf(
                "Thread entry %lu frequency %lu is invalid\n",
                static_cast<unsigned long>(threadIndex),
                static_cast<unsigned long>(frequency));
            configError = true;
            return;
        }

        if (!strcmp(configor,"Base"))
        {
            if (baseConfigured)
            {
                printf("Duplicate Base thread configuration\n");
                configError = true;
                return;
            }

            if ((frequency < PRU_BASEFREQ_MIN) ||
                (frequency > PRU_BASEFREQ_MAX))
            {
                printf(
                    "Base thread frequency %lu is outside allowed range %lu-%lu Hz\n",
                    static_cast<unsigned long>(frequency),
                    static_cast<unsigned long>(PRU_BASEFREQ_MIN),
                    static_cast<unsigned long>(PRU_BASEFREQ_MAX));
                configError = true;
                return;
            }

            configuredBaseFreq =
                frequency;

            baseConfigured = true;
        }
        else if (!strcmp(configor,"Servo"))
        {
            if (servoConfigured)
            {
                printf("Duplicate Servo thread configuration\n");
                configError = true;
                return;
            }

            if ((frequency < PRU_SERVOFREQ_MIN) ||
                (frequency > PRU_SERVOFREQ_MAX))
            {
                printf(
                    "Servo thread frequency %lu is outside allowed range %lu-%lu Hz\n",
                    static_cast<unsigned long>(frequency),
                    static_cast<unsigned long>(PRU_SERVOFREQ_MIN),
                    static_cast<unsigned long>(PRU_SERVOFREQ_MAX));
                configError = true;
                return;
            }

            configuredServoFreq =
                frequency;

            servoConfigured = true;
        }
        else
        {
            printf(
                "Unknown thread configuration: %s\n",
                configor);
            configError = true;
            return;
        }

        threadIndex++;
    }

    if (configuredServoFreq > configuredBaseFreq)
    {
        printf(
            "Servo thread frequency %lu exceeds Base thread frequency %lu\n",
            static_cast<unsigned long>(configuredServoFreq),
            static_cast<unsigned long>(configuredBaseFreq));
        configError = true;
        return;
    }

    base_freq =
        configuredBaseFreq;

    servo_freq =
        configuredServoFreq;

    printf(
        "Setting BASE thread frequency to %lu\n",
        static_cast<unsigned long>(base_freq));
    printf(
        "Setting SERVO thread frequency to %lu\n",
        static_cast<unsigned long>(servo_freq));
}


void loadModules(void)
{
    printf("\n5. Loading modules\n");

	// Ethernet communication monitoring
	comms = new RemoraComms(servo_freq);
	servoThread->registerModule(comms);
	hasServoThread = true;

    if (configError) return;

    if (!doc.containsKey("Modules"))
    {
        printf("No configured modules found\n");
        return;
    }

    JsonVariantConst modulesValue =
        doc["Modules"];

    if (!modulesValue.is<JsonArrayConst>())
    {
        printf("Modules configuration is not an array\n");
        configError = true;
        return;
    }

    JsonArrayConst Modules =
        modulesValue.as<JsonArrayConst>();

    bool spindlePwmConfigured =
        false;

    bool nvmpgConfigured =
        false;

    bool jointConfigured[JOINTS] = {};

    const char* processVariableProducer[VARIABLES] = {};

    const char* inputBitProducer[32] = {};

    uint32_t inputBitProducerEntry[32] = {};

    bool qdcEncoderConfigured[MAX_INST_QDC_MOD] = {};

    GpioOwner gpioOwners[gpioRegistrySize] = {};

    for (const FixedGpioReservation &reservation :
         fixedGpioReservations)
    {
        if (!claimGpioOwnership(
                gpioOwners,
                reservation.identity,
                reservation.ownerType,
                reservation.role,
                0U,
                true))
        {
            configError = true;
            return;
        }
    }

    uint32_t moduleIndex = 0U;
    for (JsonArrayConst::iterator it=Modules.begin(); it!=Modules.end(); ++it)
    {
        JsonVariantConst moduleValue =
            *it;

        if (!moduleValue.is<JsonObjectConst>())
        {
            printf(
                "Module entry %lu is not an object\n",
                static_cast<unsigned long>(moduleIndex));
            configError = true;
            return;
        }

        JsonObjectConst moduleObject =
            moduleValue.as<JsonObjectConst>();

        JsonVariantConst threadValue =
            moduleObject["Thread"];

        if (!threadValue.is<const char*>())
        {
            printf(
                "Module entry %lu thread is missing or is not a string\n",
                static_cast<unsigned long>(moduleIndex));
            configError = true;
            return;
        }

        const char* thread =
            threadValue.as<const char*>();

        if ((thread == nullptr) ||
            (thread[0] == '\0'))
        {
            printf(
                "Module entry %lu thread is empty\n",
                static_cast<unsigned long>(moduleIndex));
            configError = true;
            return;
        }

        JsonVariantConst typeValue =
            moduleObject["Type"];

        if (!typeValue.is<const char*>())
        {
            printf(
                "Module entry %lu type is missing or is not a string\n",
                static_cast<unsigned long>(moduleIndex));
            configError = true;
            return;
        }

        const char* type =
            typeValue.as<const char*>();

        if ((type == nullptr) ||
            (type[0] == '\0'))
        {
            printf(
                "Module entry %lu type is empty\n",
                static_cast<unsigned long>(moduleIndex));
            configError = true;
            return;
        }

        bool supportedCombination = false;

        if (!strcmp(thread,"DMA"))
        {
            supportedCombination =
                !strcmp(type,"DMAstepgen");
        }
        else if (!strcmp(thread,"Base"))
        {
            supportedCombination =
                (!strcmp(type,"Stepgen") ||
                 !strcmp(type,"Encoder") ||
                 !strcmp(type,"QDC"));
        }
        else if (!strcmp(thread,"Servo"))
        {
            supportedCombination =
                (!strcmp(type,"Digital Pin") ||
                 !strcmp(type,"Spindle PWM") ||
                 !strcmp(type,"NVMPG"));
        }

        if (!supportedCombination)
        {
            printf(
                "Unsupported module entry %lu: thread %s, type %s\n",
                static_cast<unsigned long>(moduleIndex),
                thread,
                type);
            configError = true;
            return;
        }

        if (!strcmp(thread,"DMA") &&
            !strcmp(type,"DMAstepgen"))
        {
            JsonVariantConst jointValue =
                moduleObject["Joint Number"];

            if (!jointValue.is<uint32_t>())
            {
                printf(
                    "DMAstepgen module entry %lu joint number is missing or is not an unsigned integer\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const uint32_t jointNumber =
                jointValue.as<uint32_t>();

            if (jointNumber >= JOINTS)
            {
                printf(
                    "DMAstepgen module entry %lu joint number %lu is out of range\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(jointNumber));
                configError = true;
                return;
            }

            if (jointConfigured[jointNumber])
            {
                printf(
                    "DMAstepgen module entry %lu joint number %lu is already configured\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(jointNumber));
                configError = true;
                return;
            }

            JsonVariantConst stepPinValue =
                moduleObject["Step Pin"];

            if (!stepPinValue.is<const char*>())
            {
                printf(
                    "DMAstepgen module entry %lu step pin is missing or is not a string\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const char* stepPin =
                stepPinValue.as<const char*>();

            GpioIdentity stepPinIdentity =
                {0U, 0U};

            if (!parseGpioPinName(
                    stepPin,
                    stepPinIdentity))
            {
                printf(
                    "DMAstepgen module entry %lu step pin is invalid\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            JsonVariantConst directionPinValue =
                moduleObject["Direction Pin"];

            if (!directionPinValue.is<const char*>())
            {
                printf(
                    "DMAstepgen module entry %lu direction pin is missing or is not a string\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const char* directionPin =
                directionPinValue.as<const char*>();

            GpioIdentity directionPinIdentity =
                {0U, 0U};

            if (!parseGpioPinName(
                    directionPin,
                    directionPinIdentity))
            {
                printf(
                    "DMAstepgen module entry %lu direction pin is invalid\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            if (stepPin[1] != '1')
            {
                printf(
                    "DMAstepgen module entry %lu step pin %s is not on GPIO1\n",
                    static_cast<unsigned long>(moduleIndex),
                    stepPin);
                configError = true;
                return;
            }

            if (directionPin[1] != '1')
            {
                printf(
                    "DMAstepgen module entry %lu direction pin %s is not on GPIO1\n",
                    static_cast<unsigned long>(moduleIndex),
                    directionPin);
                configError = true;
                return;
            }

            if (gpioIdentitiesReferToSamePin(
                    stepPinIdentity,
                    directionPinIdentity))
            {
                printf(
                    "DMAstepgen module entry %lu step and direction pins refer to the same GPIO\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const char* const timingFieldNames[] =
            {
                "Step Length",
                "Step Space",
                "Dir Hold",
                "Dir Setup"
            };

            for (const char* timingFieldName : timingFieldNames)
            {
                if (!moduleObject.containsKey(timingFieldName))
                {
                    continue;
                }

                JsonVariantConst timingValue =
                    moduleObject[timingFieldName];

                if (!timingValue.is<uint32_t>())
                {
                    printf(
                        "DMAstepgen module entry %lu %s is not an unsigned integer\n",
                        static_cast<unsigned long>(moduleIndex),
                        timingFieldName);
                    configError = true;
                    return;
                }

                const uint32_t timingValueNumber =
                    timingValue.as<uint32_t>();

                if (timingValueNumber > 255U)
                {
                    printf(
                        "DMAstepgen module entry %lu %s value %lu is out of range\n",
                        static_cast<unsigned long>(moduleIndex),
                        timingFieldName,
                        static_cast<unsigned long>(timingValueNumber));
                    configError = true;
                    return;
                }
            }

            if (!claimGpioOwnership(
                    gpioOwners,
                    stepPinIdentity,
                    "DMAstepgen",
                    "Step Pin",
                    moduleIndex,
                    false) ||
                !claimGpioOwnership(
                    gpioOwners,
                    directionPinIdentity,
                    "DMAstepgen",
                    "Direction Pin",
                    moduleIndex,
                    false))
            {
                configError = true;
                return;
            }

            jointConfigured[jointNumber] =
                true;
        }

        if (!strcmp(thread,"Base") &&
            !strcmp(type,"Stepgen"))
        {
            JsonVariantConst jointValue =
                moduleObject["Joint Number"];

            if (!jointValue.is<uint32_t>())
            {
                printf(
                    "Stepgen module entry %lu joint number is missing or is not an unsigned integer\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const uint32_t jointNumber =
                jointValue.as<uint32_t>();

            if (jointNumber >= JOINTS)
            {
                printf(
                    "Stepgen module entry %lu joint number %lu is out of range\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(jointNumber));
                configError = true;
                return;
            }

            if (jointConfigured[jointNumber])
            {
                printf(
                    "Stepgen module entry %lu joint number %lu is already configured\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(jointNumber));
                configError = true;
                return;
            }

            JsonVariantConst stepPinValue =
                moduleObject["Step Pin"];

            if (!stepPinValue.is<const char*>())
            {
                printf(
                    "Stepgen module entry %lu step pin is missing or is not a string\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const char* stepPin =
                stepPinValue.as<const char*>();

            GpioIdentity stepPinIdentity =
                {0U, 0U};

            if (!parseGpioPinName(
                    stepPin,
                    stepPinIdentity))
            {
                printf(
                    "Stepgen module entry %lu step pin is invalid\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            JsonVariantConst directionPinValue =
                moduleObject["Direction Pin"];

            if (!directionPinValue.is<const char*>())
            {
                printf(
                    "Stepgen module entry %lu direction pin is missing or is not a string\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const char* directionPin =
                directionPinValue.as<const char*>();

            GpioIdentity directionPinIdentity =
                {0U, 0U};

            if (!parseGpioPinName(
                    directionPin,
                    directionPinIdentity))
            {
                printf(
                    "Stepgen module entry %lu direction pin is invalid\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            if (gpioIdentitiesReferToSamePin(
                    stepPinIdentity,
                    directionPinIdentity))
            {
                printf(
                    "Stepgen module entry %lu step and direction pins refer to the same GPIO\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            if (!claimGpioOwnership(
                    gpioOwners,
                    stepPinIdentity,
                    "Stepgen",
                    "Step Pin",
                    moduleIndex,
                    false) ||
                !claimGpioOwnership(
                    gpioOwners,
                    directionPinIdentity,
                    "Stepgen",
                    "Direction Pin",
                    moduleIndex,
                    false))
            {
                configError = true;
                return;
            }

            jointConfigured[jointNumber] =
                true;
        }

        if (!strcmp(thread,"Base") &&
            !strcmp(type,"Encoder"))
        {
            JsonVariantConst pvValue =
                moduleObject["PV[i]"];

            if (!pvValue.is<uint32_t>())
            {
                printf(
                    "Encoder module entry %lu PV index is missing or is not an unsigned integer\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const uint32_t pvIndex =
                pvValue.as<uint32_t>();

            if (pvIndex >= VARIABLES)
            {
                printf(
                    "Encoder module entry %lu PV index %lu is out of range\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(pvIndex));
                configError = true;
                return;
            }

            JsonVariantConst channelAPinValue =
                moduleObject["ChA Pin"];

            if (!channelAPinValue.is<const char*>())
            {
                printf(
                    "Encoder module entry %lu channel A pin is missing or is not a string\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const char* channelAPin =
                channelAPinValue.as<const char*>();

            GpioIdentity channelAIdentity =
                {0U, 0U};

            if (!parseGpioPinName(
                    channelAPin,
                    channelAIdentity))
            {
                printf(
                    "Encoder module entry %lu channel A pin is invalid\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            JsonVariantConst channelBPinValue =
                moduleObject["ChB Pin"];

            if (!channelBPinValue.is<const char*>())
            {
                printf(
                    "Encoder module entry %lu channel B pin is missing or is not a string\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const char* channelBPin =
                channelBPinValue.as<const char*>();

            GpioIdentity channelBIdentity =
                {0U, 0U};

            if (!parseGpioPinName(
                    channelBPin,
                    channelBIdentity))
            {
                printf(
                    "Encoder module entry %lu channel B pin is invalid\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            if (gpioIdentitiesReferToSamePin(
                    channelAIdentity,
                    channelBIdentity))
            {
                printf(
                    "Encoder module entry %lu channel A and channel B pins refer to the same GPIO\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            bool producesInputBit = false;
            uint32_t inputDataBit = 0U;
            const char* indexPin = nullptr;

            GpioIdentity indexIdentity =
                {0U, 0U};

            if (moduleObject.containsKey("Index Pin"))
            {
                JsonVariantConst indexPinValue =
                    moduleObject["Index Pin"];

                if (!indexPinValue.is<const char*>())
                {
                    printf(
                        "Encoder module entry %lu index pin is not a string\n",
                        static_cast<unsigned long>(moduleIndex));
                    configError = true;
                    return;
                }

                indexPin =
                    indexPinValue.as<const char*>();

                if (!parseGpioPinName(
                        indexPin,
                        indexIdentity))
                {
                    printf(
                        "Encoder module entry %lu index pin is invalid\n",
                        static_cast<unsigned long>(moduleIndex));
                    configError = true;
                    return;
                }

                if (gpioIdentitiesReferToSamePin(
                        indexIdentity,
                        channelAIdentity))
                {
                    printf(
                        "Encoder module entry %lu index pin conflicts with channel A pin\n",
                        static_cast<unsigned long>(moduleIndex));
                    configError = true;
                    return;
                }

                if (gpioIdentitiesReferToSamePin(
                        indexIdentity,
                        channelBIdentity))
                {
                    printf(
                        "Encoder module entry %lu index pin conflicts with channel B pin\n",
                        static_cast<unsigned long>(moduleIndex));
                    configError = true;
                    return;
                }

                JsonVariantConst dataBitValue =
                    moduleObject["Data Bit"];

                if (!dataBitValue.is<uint32_t>())
                {
                    printf(
                        "Encoder module entry %lu data bit is missing or is not an unsigned integer\n",
                        static_cast<unsigned long>(moduleIndex));
                    configError = true;
                    return;
                }

                inputDataBit =
                    dataBitValue.as<uint32_t>();

                if (inputDataBit >= 32U)
                {
                    printf(
                        "Encoder module entry %lu data bit %lu is out of range\n",
                        static_cast<unsigned long>(moduleIndex),
                        static_cast<unsigned long>(inputDataBit));
                    configError = true;
                    return;
                }

                producesInputBit = true;
            }

            if (processVariableProducer[pvIndex] != nullptr)
            {
                printf(
                    "Encoder module entry %lu PV index %lu is already produced by %s\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(pvIndex),
                    processVariableProducer[pvIndex]);
                configError = true;
                return;
            }

            if (producesInputBit &&
                (inputBitProducer[inputDataBit] != nullptr))
            {
                printf(
                    "Encoder module entry %lu data bit %lu is already produced by %s module entry %lu\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(inputDataBit),
                    inputBitProducer[inputDataBit],
                    static_cast<unsigned long>(
                        inputBitProducerEntry[inputDataBit]));
                configError = true;
                return;
            }

            if (!claimGpioOwnership(
                    gpioOwners,
                    channelAIdentity,
                    "Encoder",
                    "ChA Pin",
                    moduleIndex,
                    false) ||
                !claimGpioOwnership(
                    gpioOwners,
                    channelBIdentity,
                    "Encoder",
                    "ChB Pin",
                    moduleIndex,
                    false) ||
                ((indexPin != nullptr) &&
                 !claimGpioOwnership(
                     gpioOwners,
                     indexIdentity,
                     "Encoder",
                     "Index Pin",
                     moduleIndex,
                     false)))
            {
                configError = true;
                return;
            }

            if (producesInputBit)
            {
                inputBitProducer[inputDataBit] =
                    "Encoder";

                inputBitProducerEntry[inputDataBit] =
                    moduleIndex;
            }

            processVariableProducer[pvIndex] =
                "Encoder";
        }

        if (!strcmp(thread,"Base") &&
            !strcmp(type,"QDC"))
        {
            const uint32_t filterPeriodMax = 255U;
            const uint32_t filterCountMax = 7U;

            JsonVariantConst pvValue =
                moduleObject["PV[i]"];

            if (!pvValue.is<uint32_t>())
            {
                printf(
                    "QDC module entry %lu PV index is missing or is not an unsigned integer\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const uint32_t pvIndex =
                pvValue.as<uint32_t>();

            if (pvIndex >= VARIABLES)
            {
                printf(
                    "QDC module entry %lu PV index %lu is out of range\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(pvIndex));
                configError = true;
                return;
            }

            JsonVariantConst encNumberValue =
                moduleObject["ENC No"];

            if (!encNumberValue.is<uint32_t>())
            {
                printf(
                    "QDC module entry %lu ENC No is missing or is not an unsigned integer\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const uint32_t encNumber =
                encNumberValue.as<uint32_t>();

            if ((encNumber < 1U) ||
                (encNumber > MAX_INST_QDC_MOD))
            {
                printf(
                    "QDC module entry %lu ENC No %lu is out of range\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(encNumber));
                configError = true;
                return;
            }

            if (qdcEncoderConfigured[encNumber - 1U])
            {
                printf(
                    "QDC module entry %lu ENC No %lu is already configured\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(encNumber));
                configError = true;
                return;
            }

            JsonVariantConst channelAPinValue =
                moduleObject["ChA Pin"];

            if (!channelAPinValue.is<const char*>())
            {
                printf(
                    "QDC module entry %lu channel A pin is missing or is not a string\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const char* channelAPin =
                channelAPinValue.as<const char*>();

            if ((channelAPin == nullptr) ||
                (channelAPin[0] == '\0'))
            {
                printf(
                    "QDC module entry %lu channel A pin is empty\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            if (!isQdcPhasePinSupported(
                    channelAPin))
            {
                printf(
                    "QDC module entry %lu channel A pin %s cannot be multiplexed to QDC\n",
                    static_cast<unsigned long>(moduleIndex),
                    channelAPin);
                configError = true;
                return;
            }

            GpioIdentity channelAIdentity =
                {0U, 0U};

            if (!parseGpioPinName(
                    channelAPin,
                    channelAIdentity))
            {
                printf(
                    "QDC module entry %lu channel A pin %s has no canonical GPIO identity\n",
                    static_cast<unsigned long>(moduleIndex),
                    channelAPin);
                configError = true;
                return;
            }

            JsonVariantConst channelBPinValue =
                moduleObject["ChB Pin"];

            if (!channelBPinValue.is<const char*>())
            {
                printf(
                    "QDC module entry %lu channel B pin is missing or is not a string\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const char* channelBPin =
                channelBPinValue.as<const char*>();

            if ((channelBPin == nullptr) ||
                (channelBPin[0] == '\0'))
            {
                printf(
                    "QDC module entry %lu channel B pin is empty\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            if (!isQdcPhasePinSupported(
                    channelBPin))
            {
                printf(
                    "QDC module entry %lu channel B pin %s cannot be multiplexed to QDC\n",
                    static_cast<unsigned long>(moduleIndex),
                    channelBPin);
                configError = true;
                return;
            }

            GpioIdentity channelBIdentity =
                {0U, 0U};

            if (!parseGpioPinName(
                    channelBPin,
                    channelBIdentity))
            {
                printf(
                    "QDC module entry %lu channel B pin %s has no canonical GPIO identity\n",
                    static_cast<unsigned long>(moduleIndex),
                    channelBPin);
                configError = true;
                return;
            }

            if (gpioIdentitiesReferToSamePin(
                    channelAIdentity,
                    channelBIdentity))
            {
                printf(
                    "QDC module entry %lu channel A pin %s conflicts with channel B pin %s\n",
                    static_cast<unsigned long>(moduleIndex),
                    channelAPin,
                    channelBPin);
                configError = true;
                return;
            }

            JsonVariantConst filterPeriodValue =
                moduleObject["Filter PER"];

            if (!filterPeriodValue.is<uint32_t>())
            {
                printf(
                    "QDC module entry %lu Filter PER is missing or is not an unsigned integer\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const uint32_t filterPeriod =
                filterPeriodValue.as<uint32_t>();

            if (filterPeriod > filterPeriodMax)
            {
                printf(
                    "QDC module entry %lu Filter PER value %lu is out of range\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(filterPeriod));
                configError = true;
                return;
            }

            JsonVariantConst filterCountValue =
                moduleObject["Filter CNT"];

            if (!filterCountValue.is<uint32_t>())
            {
                printf(
                    "QDC module entry %lu Filter CNT is missing or is not an unsigned integer\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const uint32_t filterCount =
                filterCountValue.as<uint32_t>();

            if (filterCount > filterCountMax)
            {
                printf(
                    "QDC module entry %lu Filter CNT value %lu is out of range\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(filterCount));
                configError = true;
                return;
            }

            bool producesInputBit = false;
            uint32_t inputDataBit = 0U;
            const char* indexPin = nullptr;

            GpioIdentity indexIdentity =
                {0U, 0U};

            JsonVariantConst indexPinValue =
                moduleObject["Index Pin"];

            if (!indexPinValue.isNull())
            {
                if (!indexPinValue.is<const char*>())
                {
                    printf(
                        "QDC module entry %lu index pin is not a string\n",
                        static_cast<unsigned long>(moduleIndex));
                    configError = true;
                    return;
                }

                indexPin =
                    indexPinValue.as<const char*>();

                if ((indexPin == nullptr) ||
                    (indexPin[0] == '\0'))
                {
                    printf(
                        "QDC module entry %lu index pin is empty\n",
                        static_cast<unsigned long>(moduleIndex));
                    configError = true;
                    return;
                }

                if (!isQdcIndexPinNameValid(
                        indexPin))
                {
                    printf(
                        "QDC module entry %lu index pin %s is invalid\n",
                        static_cast<unsigned long>(moduleIndex),
                        indexPin);
                    configError = true;
                    return;
                }

                if (!parseGpioPinName(
                        indexPin,
                        indexIdentity))
                {
                    printf(
                        "QDC module entry %lu index pin %s has no canonical GPIO identity\n",
                        static_cast<unsigned long>(moduleIndex),
                        indexPin);
                    configError = true;
                    return;
                }

                if (gpioIdentitiesReferToSamePin(
                        indexIdentity,
                        channelAIdentity))
                {
                    printf(
                        "QDC module entry %lu index pin %s conflicts with channel A pin %s\n",
                        static_cast<unsigned long>(moduleIndex),
                        indexPin,
                        channelAPin);
                    configError = true;
                    return;
                }

                if (gpioIdentitiesReferToSamePin(
                        indexIdentity,
                        channelBIdentity))
                {
                    printf(
                        "QDC module entry %lu index pin %s conflicts with channel B pin %s\n",
                        static_cast<unsigned long>(moduleIndex),
                        indexPin,
                        channelBPin);
                    configError = true;
                    return;
                }

                JsonVariantConst dataBitValue =
                    moduleObject["Data Bit"];

                if (!dataBitValue.is<uint32_t>())
                {
                    printf(
                        "QDC module entry %lu index Data Bit is missing or is not an unsigned integer\n",
                        static_cast<unsigned long>(moduleIndex));
                    configError = true;
                    return;
                }

                inputDataBit =
                    dataBitValue.as<uint32_t>();

                if (inputDataBit >= 32U)
                {
                    printf(
                        "QDC module entry %lu index Data Bit %lu is out of range\n",
                        static_cast<unsigned long>(moduleIndex),
                        static_cast<unsigned long>(inputDataBit));
                    configError = true;
                    return;
                }

                producesInputBit = true;
            }

            if (processVariableProducer[pvIndex] != nullptr)
            {
                printf(
                    "QDC module entry %lu PV index %lu is already produced by %s\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(pvIndex),
                    processVariableProducer[pvIndex]);
                configError = true;
                return;
            }

            if (producesInputBit &&
                (inputBitProducer[inputDataBit] != nullptr))
            {
                printf(
                    "QDC module entry %lu data bit %lu is already produced by %s module entry %lu\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(inputDataBit),
                    inputBitProducer[inputDataBit],
                    static_cast<unsigned long>(
                        inputBitProducerEntry[inputDataBit]));
                configError = true;
                return;
            }

            if (!claimGpioOwnership(
                    gpioOwners,
                    channelAIdentity,
                    "QDC",
                    "ChA Pin",
                    moduleIndex,
                    false) ||
                !claimGpioOwnership(
                    gpioOwners,
                    channelBIdentity,
                    "QDC",
                    "ChB Pin",
                    moduleIndex,
                    false) ||
                ((indexPin != nullptr) &&
                 !claimGpioOwnership(
                     gpioOwners,
                     indexIdentity,
                     "QDC",
                     "Index Pin",
                     moduleIndex,
                     false)))
            {
                configError = true;
                return;
            }

            if (producesInputBit)
            {
                inputBitProducer[inputDataBit] =
                    "QDC";

                inputBitProducerEntry[inputDataBit] =
                    moduleIndex;
            }

            processVariableProducer[pvIndex] =
                "QDC";

            qdcEncoderConfigured[encNumber - 1U] =
                true;
        }

        if (!strcmp(thread,"Servo") &&
            !strcmp(type,"Digital Pin"))
        {
            JsonVariantConst pinValue =
                moduleObject["Pin"];

            if (!pinValue.is<const char*>())
            {
                printf(
                    "Digital Pin module entry %lu pin is missing or is not a string\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const char* configuredPin =
                pinValue.as<const char*>();

            GpioIdentity configuredPinIdentity =
                {0U, 0U};

            if (!parseGpioPinName(
                    configuredPin,
                    configuredPinIdentity))
            {
                printf(
                    "Digital Pin module entry %lu pin is invalid\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            JsonVariantConst modeValue =
                moduleObject["Mode"];

            if (!modeValue.is<const char*>())
            {
                printf(
                    "Digital Pin module entry %lu mode is missing or is not a string\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const char* mode =
                modeValue.as<const char*>();

            if (strcmp(mode,"Input") &&
                strcmp(mode,"Output"))
            {
                printf(
                    "Digital Pin module entry %lu mode is invalid\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            JsonVariantConst dataBitValue =
                moduleObject["Data Bit"];

            if (!dataBitValue.is<uint32_t>())
            {
                printf(
                    "Digital Pin module entry %lu data bit is missing or is not an unsigned integer\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const uint32_t dataBit =
                dataBitValue.as<uint32_t>();

            if (dataBit >= 32U)
            {
                printf(
                    "Digital Pin module entry %lu data bit %lu is out of range\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(dataBit));
                configError = true;
                return;
            }

            if (moduleObject.containsKey("Invert"))
            {
                JsonVariantConst invertValue =
                    moduleObject["Invert"];

                if (!invertValue.is<const char*>())
                {
                    printf(
                        "Digital Pin module entry %lu invert is not a string\n",
                        static_cast<unsigned long>(moduleIndex));
                    configError = true;
                    return;
                }

                const char* invert =
                    invertValue.as<const char*>();

                if (strcmp(invert,"True") &&
                    strcmp(invert,"False"))
                {
                    printf(
                        "Digital Pin module entry %lu invert value is invalid\n",
                        static_cast<unsigned long>(moduleIndex));
                    configError = true;
                    return;
                }
            }

            const char* modifier =
                "None";

            if (moduleObject.containsKey("Modifier"))
            {
                JsonVariantConst modifierValue =
                    moduleObject["Modifier"];

                if (!modifierValue.is<const char*>())
                {
                    printf(
                        "Digital Pin module entry %lu modifier is not a string\n",
                        static_cast<unsigned long>(moduleIndex));
                    configError = true;
                    return;
                }

                modifier =
                    modifierValue.as<const char*>();
            }

            PinModifier pinModifier =
                PinModifier::None;

            if (!parsePinModifier(
                    modifier,
                    pinModifier))
            {
                printf(
                    "Digital Pin module entry %lu modifier value is invalid\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const int pinDirection =
                !strcmp(mode,"Input")
                    ? INPUT
                    : OUTPUT;

            if (!pinModifierIsCompatible(
                    pinModifier,
                    pinDirection))
            {
                printf(
                    "Digital Pin module entry %lu pin %s mode %s does not support modifier %s\n",
                    static_cast<unsigned long>(moduleIndex),
                    configuredPin,
                    mode,
                    modifier);
                configError = true;
                return;
            }

            if ((pinModifier !=
                 PinModifier::None) &&
                !pinHasPadConfigRegister(
                    configuredPin))
            {
                printf(
                    "Digital Pin module entry %lu pin %s mode %s modifier %s has no unique pad-control register\n",
                    static_cast<unsigned long>(moduleIndex),
                    configuredPin,
                    mode,
                    modifier);
                configError = true;
                return;
            }

            const bool isInput =
                !strcmp(mode,"Input");

            if (isInput &&
                (inputBitProducer[dataBit] != nullptr))
            {
                printf(
                    "Digital Pin module entry %lu data bit %lu is already produced by %s module entry %lu\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(dataBit),
                    inputBitProducer[dataBit],
                    static_cast<unsigned long>(
                        inputBitProducerEntry[dataBit]));
                configError = true;
                return;
            }

            if (!claimGpioOwnership(
                    gpioOwners,
                    configuredPinIdentity,
                    "Digital Pin",
                    isInput
                        ? "Input Pin"
                        : "Output Pin",
                    moduleIndex,
                    false))
            {
                configError = true;
                return;
            }

            if (isInput)
            {
                inputBitProducer[dataBit] =
                    "Digital Pin";

                inputBitProducerEntry[dataBit] =
                    moduleIndex;
            }
        }

        if (!strcmp(thread,"Servo") &&
            !strcmp(type,"Spindle PWM"))
        {
            if (spindlePwmConfigured)
            {
                printf(
                    "Spindle PWM module entry %lu duplicates the fixed Spindle PWM hardware\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            JsonVariantConst setPointValue =
                moduleObject["SP[i]"];

            if (!setPointValue.is<uint32_t>())
            {
                printf(
                    "Spindle PWM module entry %lu setpoint index is missing or is not an unsigned integer\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            const uint32_t setPointIndex =
                setPointValue.as<uint32_t>();

            if (setPointIndex >= VARIABLES)
            {
                printf(
                    "Spindle PWM module entry %lu setpoint index %lu is out of range\n",
                    static_cast<unsigned long>(moduleIndex),
                    static_cast<unsigned long>(setPointIndex));
                configError = true;
                return;
            }

            const GpioIdentity spindlePwmIdentity =
                {2U, 0U};

            if (!claimGpioOwnership(
                    gpioOwners,
                    spindlePwmIdentity,
                    "Spindle PWM",
                    "Output Pad",
                    moduleIndex,
                    false))
            {
                configError = true;
                return;
            }

            spindlePwmConfigured =
                true;
        }

        if (!strcmp(thread,"Servo") &&
            !strcmp(type,"NVMPG"))
        {
            if (nvmpgConfigured)
            {
                printf(
                    "NVMPG module entry %lu duplicates the fixed NVMPG hardware\n",
                    static_cast<unsigned long>(moduleIndex));
                configError = true;
                return;
            }

            nvmpgConfigured =
                true;
        }

        moduleIndex++;
    }

    JsonArray mutableModules =
        doc["Modules"].as<JsonArray>();

    // create objects from JSON data
    for (JsonArray::iterator it=mutableModules.begin(); it!=mutableModules.end(); ++it)
    {
        module = *it;

        const char* thread = module["Thread"];
        const char* type = module["Type"];

        if (!strcmp(thread,"DMA"))
        {
            printf("\nDMA thread object\n");

            if (!strcmp(type,"DMAstepgen"))
            {
            	createDMAstepgen();
            	DMAstepgen = true;
                hasDMAthread = true;
            }
        }
        else if (!strcmp(thread,"Base"))
        {
            printf("\nBase thread object\n");

            if (!strcmp(type,"Stepgen"))
            {
                createStepgen();
                hasBaseThread = true;
            }
            else if (!strcmp(type,"Encoder"))
            {
                createEncoder();
                hasBaseThread = true;
            }
            else if (!strcmp(type,"QDC"))
            {
                if (createQdc())
                {
                    hasQDC = true;
                    hasBaseThread = true;
                }
            }
         }
        else if (!strcmp(thread,"Servo"))
        {
            printf("\nServo thread object\n");

        	if (!strcmp(type,"Digital Pin"))
			{
				createDigitalPin();
			}
        	else if (!strcmp(type,"Spindle PWM"))
			{
				createSpindlePWM();
			}
        	else if (!strcmp(type,"NVMPG"))
			{
				createNVMPG();
			}
        }
    }
}


// Interrupt service for SysTick timer.
extern "C" {
	void SysTick_Handler(void)
	{
		time_isr();
	}
}


int main(void)
{


	enum State currentState;
	enum State prevState;
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    currentState = ST_SETUP;
    prevState = ST_RESET;

    printf("\nRemora RT1052 starting\n\n");

    initEthernet();

    while (1)
    {
 	   switch(currentState){
     		          case ST_SETUP:
     		              // do setup tasks
     		              if (currentState != prevState)
     		              {
     		                  printf("\n## Entering SETUP state\n\n");
     		              }
     		              prevState = currentState;

                          jsonFromFlash();
     		              deserialiseJSON();
						  getBoardType();
     		              configThreads();
     		              createThreads();
     		              //debugThreadHigh();
     		              loadModules();
     		              //debugThreadLow();
     		              udpServer_init();
     		              IAP_tftpd_init(dmaThread->DMAptr->EDMA_Handle); // pass the dmaThread EDMA handle as we need to stop the DMA during a config upload

     		              currentState = ST_START;
     		              break;

     		          case ST_START:
     		              // do start tasks
     		              if (currentState != prevState)
     		              {
     		                  printf("\n## Entering START state\n");
     		              }
     		              prevState = currentState;

     		              if (!threadsRunning)
     		              {
     		                  // Start the threads
     		            	  if (hasBaseThread)
     		            	  {
         		                  printf("\nStarting the BASE thread\n");
         		                  baseThread->startThread();
     		            	  }

     		            	  if (hasServoThread)
     		            	  {
         		                  printf("\nStarting the SERVO thread\n");
         		                  servoThread->startThread();
     		            	  }

     		                  threadsRunning = true;
     		              }

     		              currentState = ST_IDLE;

     		              break;


     		          case ST_IDLE:
     		              // do something when idle
     		              if (currentState != prevState)
     		              {
     		                  printf("\n## Entering IDLE state\n");
     		              }
     		              prevState = currentState;

     		              //wait for data before changing to running state
     		              if (comms->getStatus())
     		              {
     		                  currentState = ST_RUNNING;
     		              }

     		              break;

     		          case ST_RUNNING:
     		              // do running tasks
     		              if (currentState != prevState)
     		              {
     		                  printf("\n## Entering RUNNING state\n");
     		              }
     		              prevState = currentState;

     		              if (hasDMAthread && !DMAthreadRunning)
     		              {
     		            	 dmaThread->run();
     		            	 dmaThread->startThread();
     		            	 DMAthreadRunning = true;
     		              }

     		              if (comms->getStatus() == false)
     		              {
     		            	  currentState = ST_RESET;
     		              }

     		              break;

     		          case ST_STOP:
     		              // do stop tasks
     		              if (currentState != prevState)
     		              {
     		                  printf("\n## Entering STOP state\n");
     		              }
     		              prevState = currentState;


     		              currentState = ST_STOP;
     		              break;

     		          case ST_RESET:
     		              // do reset tasks
     		              if (currentState != prevState)
     		              {
     		                  printf("\n## Entering RESET state\n");
     		              }
     		              prevState = currentState;

     		              // set all of the rxData buffer to 0
     		              // rxData.rxBuffer is volatile so need to do this the long way. memset cannot be used for volatile
     		              printf("   Resetting rxBuffer\n");
     		              {
     		                  int n = sizeof(rxData.rxBuffer);
     		                  while(n-- > 0)
     		                  {
     		                      rxData.rxBuffer[n] = 0;
     		                  }
     		              }

                              if (hasDMAthread)
                              {
                                  if (DMAthreadRunning)
                                  {
                                      dmaThread->stopThread();
                                      DMAthreadRunning = false;
                                  }
                                  else
                                  {
                                      dmaThread->DMAptr->clearCompletionState();
                                  }

                                  // Apply the disabled state after DMA has stopped.
                                  dmaThread->run();
                              }

     		              currentState = ST_IDLE;
     		              break;

     		          case ST_WDRESET:
     		        	  // force a reset
     		        	  //NVIC_SystemReset();
     		              break;
     		  }

        EthernetTasks();

	if (controlSessionHandoffPending() &&
		threadsRunning)
	{
		if (hasBaseThread)
		{
			baseThread->stopThread();
		}

		if (hasServoThread)
		{
			servoThread->stopThread();
		}

		threadsRunning = false;

		if (hasDMAthread && DMAthreadRunning)
		{
			dmaThread->stopThread();
			DMAthreadRunning = false;
		}

		int remaining =
			sizeof(rxData.rxBuffer);

		while (remaining-- > 0)
		{
			rxData.rxBuffer[remaining] = 0;
		}

		comms->resetCommunication();

		if (hasBaseThread)
		{
			baseThread->run();
		}

		if (hasServoThread)
		{
			servoThread->run();
		}

		if (hasDMAthread)
		{
			dmaThread->run();
		}

		if (hasBaseThread)
		{
			baseThread->startThread();
		}

		if (hasServoThread)
		{
			servoThread->startThread();
		}

		threadsRunning = true;
		currentState = ST_IDLE;
		prevState = ST_RESET;

		controlSessionHandoffComplete();
	}

	DMA::CompletionResult dmaCompletion =
		dmaThread->DMAptr->takeCompletion();

	bool dmaCompletionFault =
		(dmaCompletion == DMA::CompletionResult::Fault);

	if ((dmaCompletion == DMA::CompletionResult::Buffer0) ||
		(dmaCompletion == DMA::CompletionResult::Buffer1))
	{
		dmaThread->DMAptr->updateBuffers(
			dmaCompletion);
		dmaThread->run();

		dmaCompletionFault =
			dmaThread->DMAptr->completeBufferService();
	}

	if (dmaCompletionFault)
	{
		printf(
			"DMA completion overrun. Stopping DMA Stepgen.\n");

		if (DMAthreadRunning)
		{
			dmaThread->stopThread();
			DMAthreadRunning = false;
		}
		else
		{
			dmaThread->DMAptr->clearCompletionState();
		}

		int remaining =
			sizeof(rxData.rxBuffer);

		while (remaining-- > 0)
		{
			rxData.rxBuffer[remaining] = 0;
		}

		// Apply the disabled state after DMA has stopped.
		dmaThread->run();

		currentState = ST_RESET;
	}

        if (newJson)
		{
			printf("\n\nChecking new configuration file\n");

			if (checkJson() <= 0)
			{
				IAP_tftp_finalize_upload(false);
			}
			else
			{
				printf("Moving new configuration file to Flash storage\n");
				if (moveJson())
				{
					IAP_tftp_finalize_upload(true);
					NVIC_SystemReset();
				}
				else
				{
					IAP_tftp_finalize_upload(false);
				}
			}
		}
    }
}
