
#ifndef REMORA_H
#define REMORA_H

#include <stddef.h>
#include <stdint.h>

#define JOINTS				6  			// Number of joints - set this the same as Remora firmware code!!!. Max 8 joints
#define VARIABLES          	4 			// Number of command values - set this the same Remora firmware code!!!
#define DIGITAL_OUTPUTS		32
#define DIGITAL_INPUTS		32
#define NVMPG_INPUTS		10

#define PRU_DATA			0x64617461 	// "data" payload
#define PRU_READ          	0x72656164  // "read" payload
#define PRU_WRITE         	0x77726974  // "writ" payload
#define PRU_ACKNOWLEDGE		0x61636b6e	// "ackn" payload
#define PRU_ERR		        0x6572726f	// "erro" payload
#define PRU_ESTOP           0x65737470  // "estp" payload

#define CONTROL_PROTOCOL_VERSION       UINT32_C(2)
#define CONTROL_KIND_DATA              UINT32_C(0x64617461)
#define CONTROL_KIND_READ              UINT32_C(0x72656164)
#define CONTROL_KIND_WRITE             UINT32_C(0x77726974)
#define CONTROL_KIND_ACKNOWLEDGE       UINT32_C(0x61636b6e)
#define CONTROL_KIND_OPEN              UINT32_C(0x6f70656e)
#define CONTROL_KIND_CHALLENGE         UINT32_C(0x6368616c)
#define CONTROL_KIND_ACTIVATE          UINT32_C(0x61637476)
#define CONTROL_KIND_ESTABLISHED       UINT32_C(0x65737462)
#define CONTROL_ENVELOPE_SIZE          16U
#define CONTROL_ESTABLISHMENT_SIZE     24U
#define CONTROL_COMMAND_PAYLOAD_SIZE   46U
#define CONTROL_TELEMETRY_PAYLOAD_SIZE 46U
#define CONTROL_READ_PACKET_SIZE       CONTROL_ENVELOPE_SIZE
#define CONTROL_WRITE_PACKET_SIZE      \
	(CONTROL_ENVELOPE_SIZE + CONTROL_COMMAND_PAYLOAD_SIZE)
#define CONTROL_DATA_PACKET_SIZE       \
	(CONTROL_ENVELOPE_SIZE + CONTROL_TELEMETRY_PAYLOAD_SIZE)
#define CONTROL_ACK_PACKET_SIZE        CONTROL_ENVELOPE_SIZE

// Control wire values retain the existing little-endian integer/float contract.
#pragma pack(push, 1)

typedef struct
{
	uint32_t protocolVersion;
	uint32_t sessionId;
	uint32_t sequence;
	uint32_t kind;
} controlEnvelope_t;

typedef struct
{
	controlEnvelope_t envelope;
	uint32_t challenge;
	uint32_t sessionToken;
} controlEstablishment_t;

typedef union
{
	struct
	{
		uint8_t txBuffer[CONTROL_WRITE_PACKET_SIZE];
	};
	struct
	{
		controlEnvelope_t envelope;
		int32_t jointFreqCmd[JOINTS];
		float setPoint[VARIABLES];
		uint8_t jointEnable;
		uint32_t outputs;
		uint8_t spare0;
	};
} txData_t;

typedef union
{
	struct
	{
		uint8_t rxBuffer[CONTROL_DATA_PACKET_SIZE];
	};
	struct
	{
		controlEnvelope_t envelope;
		int32_t jointFeedback[JOINTS];
		float processVariable[VARIABLES];
		uint32_t inputs;
		uint16_t NVMPGinputs;
	};
} rxData_t;

#pragma pack(pop)

_Static_assert(
	sizeof(controlEnvelope_t) == CONTROL_ENVELOPE_SIZE,
	"Control envelope size mismatch");
_Static_assert(
	_Alignof(controlEnvelope_t) == 1U,
	"Control envelope alignment mismatch");
_Static_assert(
	sizeof(controlEstablishment_t) == CONTROL_ESTABLISHMENT_SIZE,
	"Control establishment size mismatch");
_Static_assert(
	_Alignof(controlEstablishment_t) == 1U,
	"Control establishment alignment mismatch");
_Static_assert(
	offsetof(controlEstablishment_t, envelope) == 0U,
	"Control establishment envelope offset mismatch");
_Static_assert(
	offsetof(controlEstablishment_t, challenge) ==
		CONTROL_ENVELOPE_SIZE,
	"Control establishment challenge offset mismatch");
_Static_assert(
	offsetof(controlEstablishment_t, sessionToken) == 20U,
	"Control establishment session-token offset mismatch");
_Static_assert(
	CONTROL_READ_PACKET_SIZE == 16U,
	"Control READ packet size mismatch");
_Static_assert(
	CONTROL_ACK_PACKET_SIZE == 16U,
	"Control ACK packet size mismatch");
_Static_assert(
	CONTROL_WRITE_PACKET_SIZE == 62U,
	"Control WRITE packet constant mismatch");
_Static_assert(
	CONTROL_DATA_PACKET_SIZE == 62U,
	"Control DATA packet constant mismatch");
_Static_assert(
	offsetof(controlEnvelope_t, protocolVersion) == 0U,
	"Control protocol-version offset mismatch");
_Static_assert(
	offsetof(controlEnvelope_t, sessionId) == 4U,
	"Control session-ID offset mismatch");
_Static_assert(
	offsetof(controlEnvelope_t, sequence) == 8U,
	"Control sequence offset mismatch");
_Static_assert(
	offsetof(controlEnvelope_t, kind) == 12U,
	"Control kind offset mismatch");

_Static_assert(
	CONTROL_KIND_DATA == (uint32_t)PRU_DATA,
	"Control DATA kind mismatch");
_Static_assert(
	CONTROL_KIND_READ == (uint32_t)PRU_READ,
	"Control READ kind mismatch");
_Static_assert(
	CONTROL_KIND_WRITE == (uint32_t)PRU_WRITE,
	"Control WRITE kind mismatch");
_Static_assert(
	CONTROL_KIND_ACKNOWLEDGE == (uint32_t)PRU_ACKNOWLEDGE,
	"Control ACK kind mismatch");

_Static_assert(
	sizeof(txData_t) == CONTROL_WRITE_PACKET_SIZE,
	"Control WRITE packet size mismatch");
_Static_assert(
	_Alignof(txData_t) == 1U,
	"Control WRITE packet alignment mismatch");
_Static_assert(
	(sizeof(txData_t) - sizeof(controlEnvelope_t)) ==
		CONTROL_COMMAND_PAYLOAD_SIZE,
	"Control command payload size mismatch");
_Static_assert(
	offsetof(txData_t, envelope) == 0U,
	"Control WRITE envelope offset mismatch");
_Static_assert(
	offsetof(txData_t, jointFreqCmd) == CONTROL_ENVELOPE_SIZE,
	"Control joint-command offset mismatch");
_Static_assert(
	offsetof(txData_t, setPoint) == 40U,
	"Control set-point offset mismatch");
_Static_assert(
	offsetof(txData_t, jointEnable) == 56U,
	"Control joint-enable offset mismatch");
_Static_assert(
	offsetof(txData_t, outputs) == 57U,
	"Control outputs offset mismatch");
_Static_assert(
	offsetof(txData_t, spare0) == 61U,
	"Control command spare offset mismatch");

_Static_assert(
	sizeof(rxData_t) == CONTROL_DATA_PACKET_SIZE,
	"Control DATA packet size mismatch");
_Static_assert(
	_Alignof(rxData_t) == 1U,
	"Control DATA packet alignment mismatch");
_Static_assert(
	(sizeof(rxData_t) - sizeof(controlEnvelope_t)) ==
		CONTROL_TELEMETRY_PAYLOAD_SIZE,
	"Control telemetry payload size mismatch");
_Static_assert(
	offsetof(rxData_t, envelope) == 0U,
	"Control DATA envelope offset mismatch");
_Static_assert(
	offsetof(rxData_t, jointFeedback) == CONTROL_ENVELOPE_SIZE,
	"Control joint-feedback offset mismatch");
_Static_assert(
	offsetof(rxData_t, processVariable) == 40U,
	"Control process-variable offset mismatch");
_Static_assert(
	offsetof(rxData_t, inputs) == 56U,
	"Control inputs offset mismatch");
_Static_assert(
	offsetof(rxData_t, NVMPGinputs) == 60U,
	"Control NVMPG-input offset mismatch");

#define STEPBIT				22			// bit location in DDS accum
#define STEP_MASK			(1L<<STEPBIT)
#define STEP_OFFSET			(1L<<(STEPBIT-1))

#define PRU_BASEFREQ		40000 		// Base freq of the PRU stepgen in Hz



#endif
