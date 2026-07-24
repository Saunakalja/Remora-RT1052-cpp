#ifndef REMORA_H
#define REMORA_H

#include "configuration.h"
#include <cstddef>
#include <cstdint>

#define CONTROL_PROTOCOL_VERSION       UINT32_C(2)
#define CONTROL_KIND_DATA              UINT32_C(0x64617461)
#define CONTROL_KIND_READ              UINT32_C(0x72656164)
#define CONTROL_KIND_WRITE             UINT32_C(0x77726974)
#define CONTROL_KIND_ACKNOWLEDGE       UINT32_C(0x61636b6e)
#define CONTROL_KIND_OPEN              UINT32_C(0x6f70656e)
#define CONTROL_KIND_CHALLENGE         UINT32_C(0x6368616c)
#define CONTROL_KIND_ACTIVATE          UINT32_C(0x61637476)
#define CONTROL_KIND_ESTABLISHED       UINT32_C(0x65737462)
#define CONTROL_KIND_MAINTENANCE_REQUEST UINT32_C(0x6d61696e)
#define CONTROL_KIND_MAINTENANCE_READY UINT32_C(0x6d726479)
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
#define CONTROL_MAINTENANCE_REQUEST_SIZE CONTROL_ENVELOPE_SIZE
#define CONTROL_MAINTENANCE_READY_SIZE CONTROL_ENVELOPE_SIZE

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
  // this allow structured access to the incoming SPI data without having to move it
  struct
  {
    uint8_t rxBuffer[CONTROL_WRITE_PACKET_SIZE];
  };
  struct
  {
    controlEnvelope_t envelope;
    volatile int32_t jointFreqCmd[JOINTS]; 	// Base thread commands ?? - basically motion
    float setPoint[VARIABLES];		  // Servo thread commands ?? - temperature SP, PWM etc
    uint8_t jointEnable;
    uint32_t outputs;
    uint8_t spare0;
  };
} rxData_t;

extern volatile rxData_t rxData;


typedef union
{
  // this allow structured access to the out going SPI data without having to move it
  struct
  {
    uint8_t txBuffer[CONTROL_DATA_PACKET_SIZE];
  };
  struct
  {
    controlEnvelope_t envelope;
    int32_t jointFeedback[JOINTS];	  			// Base thread feedback ??
    float processVariable[VARIABLES];		    // Servo thread feedback ??
	uint32_t inputs;
	uint16_t NVMPGinputs;
  };
} txData_t;

extern volatile txData_t txData;


typedef union
{
	struct
	{
		uint8_t payload[57];
	};
	struct
	{
		int32_t	header;
		int8_t	byte0;
		int8_t	byte1;
		int32_t xPos;
		int32_t yPos;
		int32_t zPos;
		int32_t aPos;
		int32_t bPos;
		int32_t cPos;
		int8_t	byte24;
		int8_t	reset;
		int8_t	byte26;
		int32_t spindle_rpm;
		int8_t	spindle_on;
		int8_t	feed_rate_override;
		int8_t	slow_jog_rate;
		int8_t	spindle_rate_override;
		int8_t	spare35;
		int8_t	parameter_select;
		int8_t	axis_select;
		int8_t	mpg_multiplier;
		int8_t	spare39;
		int8_t	spare40;
		int8_t	spare41;
		int8_t	spare42;
		int8_t	spare43;
		int8_t	spare44;
		int8_t	spare45;
		int8_t	spare46;
		int8_t	spare47;
		int8_t	spare48;
		int8_t	spare49;
		int8_t	spare50;
	};
} mpgData_t;

extern mpgData_t mpgData;

#pragma pack(pop)

static_assert(
    sizeof(controlEnvelope_t) == CONTROL_ENVELOPE_SIZE,
    "Control envelope size mismatch");
static_assert(
    alignof(controlEnvelope_t) == 1U,
    "Control envelope alignment mismatch");
static_assert(
    sizeof(controlEstablishment_t) == CONTROL_ESTABLISHMENT_SIZE,
    "Control establishment size mismatch");
static_assert(
    alignof(controlEstablishment_t) == 1U,
    "Control establishment alignment mismatch");
static_assert(
    offsetof(controlEstablishment_t, envelope) == 0U,
    "Control establishment envelope offset mismatch");
static_assert(
    offsetof(controlEstablishment_t, challenge) ==
        CONTROL_ENVELOPE_SIZE,
    "Control establishment challenge offset mismatch");
static_assert(
    offsetof(controlEstablishment_t, sessionToken) == 20U,
    "Control establishment session-token offset mismatch");
static_assert(
    CONTROL_READ_PACKET_SIZE == 16U,
    "Control READ packet size mismatch");
static_assert(
    CONTROL_ACK_PACKET_SIZE == 16U,
    "Control ACK packet size mismatch");
static_assert(
    CONTROL_MAINTENANCE_REQUEST_SIZE == 16U,
    "Control maintenance-request packet size mismatch");
static_assert(
    CONTROL_MAINTENANCE_READY_SIZE == 16U,
    "Control maintenance-ready packet size mismatch");
static_assert(
    CONTROL_WRITE_PACKET_SIZE == 62U,
    "Control WRITE packet constant mismatch");
static_assert(
    CONTROL_DATA_PACKET_SIZE == 62U,
    "Control DATA packet constant mismatch");
static_assert(
    offsetof(controlEnvelope_t, protocolVersion) == 0U,
    "Control protocol-version offset mismatch");
static_assert(
    offsetof(controlEnvelope_t, sessionId) == 4U,
    "Control session-ID offset mismatch");
static_assert(
    offsetof(controlEnvelope_t, sequence) == 8U,
    "Control sequence offset mismatch");
static_assert(
    offsetof(controlEnvelope_t, kind) == 12U,
    "Control kind offset mismatch");

static_assert(
    CONTROL_KIND_DATA == static_cast<uint32_t>(PRU_DATA),
    "Control DATA kind mismatch");
static_assert(
    CONTROL_KIND_READ == static_cast<uint32_t>(PRU_READ),
    "Control READ kind mismatch");
static_assert(
    CONTROL_KIND_WRITE == static_cast<uint32_t>(PRU_WRITE),
    "Control WRITE kind mismatch");
static_assert(
    CONTROL_KIND_ACKNOWLEDGE ==
        static_cast<uint32_t>(PRU_ACKNOWLEDGE),
    "Control ACK kind mismatch");

static_assert(
    sizeof(rxData_t) == CONTROL_WRITE_PACKET_SIZE,
    "Control WRITE packet size mismatch");
static_assert(
    alignof(rxData_t) == 1U,
    "Control WRITE packet alignment mismatch");
static_assert(
    (sizeof(rxData_t) - sizeof(controlEnvelope_t)) ==
        CONTROL_COMMAND_PAYLOAD_SIZE,
    "Control command payload size mismatch");
static_assert(
    offsetof(rxData_t, envelope) == 0U,
    "Control WRITE envelope offset mismatch");
static_assert(
    offsetof(rxData_t, jointFreqCmd) == CONTROL_ENVELOPE_SIZE,
    "Control joint-command offset mismatch");
static_assert(
    offsetof(rxData_t, setPoint) == 40U,
    "Control set-point offset mismatch");
static_assert(
    offsetof(rxData_t, jointEnable) == 56U,
    "Control joint-enable offset mismatch");
static_assert(
    offsetof(rxData_t, outputs) == 57U,
    "Control outputs offset mismatch");
static_assert(
    offsetof(rxData_t, spare0) == 61U,
    "Control command spare offset mismatch");

static_assert(
    sizeof(txData_t) == CONTROL_DATA_PACKET_SIZE,
    "Control DATA packet size mismatch");
static_assert(
    alignof(txData_t) == 1U,
    "Control DATA packet alignment mismatch");
static_assert(
    (sizeof(txData_t) - sizeof(controlEnvelope_t)) ==
        CONTROL_TELEMETRY_PAYLOAD_SIZE,
    "Control telemetry payload size mismatch");
static_assert(
    offsetof(txData_t, envelope) == 0U,
    "Control DATA envelope offset mismatch");
static_assert(
    offsetof(txData_t, jointFeedback) == CONTROL_ENVELOPE_SIZE,
    "Control joint-feedback offset mismatch");
static_assert(
    offsetof(txData_t, processVariable) == 40U,
    "Control process-variable offset mismatch");
static_assert(
    offsetof(txData_t, inputs) == 56U,
    "Control inputs offset mismatch");
static_assert(
    offsetof(txData_t, NVMPGinputs) == 60U,
    "Control NVMPG-input offset mismatch");

#endif
