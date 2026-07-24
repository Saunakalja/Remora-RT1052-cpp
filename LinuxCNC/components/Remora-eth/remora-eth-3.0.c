/********************************************************************
* Description:  remora-eth.c
*               This file, 'remora-eth.c', is a HAL component that
*               provides an ethernet connection to a external Enthernet
* 			 	controller running Remora PRU firmware.
*  				
*
* Author: Scott Alford
* License: GPL Version 2
*
*		Credit to GP Orcullo and PICnc V2 which originally inspired this
*		and portions of this code is based on stepgen.c by John Kasunich
*		and hm2_rpspi.c by Matsche
*
* Copyright (c) 2023 All rights reserved.
*
* Last change:
********************************************************************/


#include "rtapi.h"			/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"			/* HAL public API decls */

#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <stdint.h>
#include <limits.h>

#include <math.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>


#include "remora-eth-3.0.h"

#define MODNAME "remora-eth-3.0"
#define PREFIX "remora"

MODULE_AUTHOR("Scott Alford AKA scotta");
MODULE_DESCRIPTION("Driver for Remora Ethernet capable control board");
MODULE_LICENSE("GPL v2");


/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

typedef struct {
	hal_bit_t		*enable;
	hal_bit_t		*reset;
	hal_bit_t		*PRUreset;
	bool			resetOld;
	hal_bit_t		*status;
	hal_bit_t 		*stepperEnable[JOINTS];
	int				pos_mode[JOINTS];
	hal_float_t 	*pos_cmd[JOINTS];			// pin: position command (position units)
	hal_float_t 	*vel_cmd[JOINTS];			// pin: velocity command (position units/sec)
	hal_float_t 	*pos_fb[JOINTS];			// pin: position feedback (position units)
	hal_s32_t		*count[JOINTS];				// pin: psition feedback (raw counts)
	hal_float_t 	pos_scale[JOINTS];			// param: steps per position unit
	float 			freq[JOINTS];				// param: frequency command sent to PRU
	hal_float_t 	*freq_cmd[JOINTS];			// pin: frequency command monitoring, available in LinuxCNC
	hal_float_t 	maxvel[JOINTS];				// param: max velocity, (pos units/sec)
	hal_float_t 	maxaccel[JOINTS];			// param: max accel (pos units/sec^2)
	hal_float_t		*pgain[JOINTS];
	hal_float_t		*ff1gain[JOINTS];
	hal_float_t		*deadband[JOINTS];
	float 			old_pos_cmd[JOINTS];		// previous position command (counts)
	float 			old_pos_cmd_raw[JOINTS];	// previous position command (counts)
	hal_float_t		old_scale[JOINTS];			// stored scale value
	hal_float_t		scale_recip[JOINTS];		// reciprocal value used for scaling
	float			prev_cmd[JOINTS];
	float			cmd_d[JOINTS];				// command derivative
	hal_float_t 	*setPoint[VARIABLES];
	hal_float_t 	*processVariable[VARIABLES];
	hal_bit_t   	*outputs[DIGITAL_OUTPUTS];
	hal_bit_t   	*inputs[DIGITAL_INPUTS*2];
	hal_bit_t   	*NVMPGinputs[NVMPG_INPUTS];
} data_t;

static data_t *data;

static txData_t txData;
static rxData_t rxData;


/* other globals */
static int 			comp_id;				// component ID
static const char 	*modname = MODNAME;
static const char 	*prefix = PREFIX;
static int 			num_chan = 0;			// number of step generators configured
static long 		old_dtns;				// update_freq function period in nsec - (THIS IS RUNNING IN THE PI)
static double		dt;						// update_freq period in seconds  - (THIS IS RUNNING IN THE PI)
static double 		recip_dt;				// recprocal of period, avoids divides

static int32_t 		count[JOINTS] = { 0 };

static int 			reset_gpio_pin = 25;				// debug pin

typedef enum CONTROL { POSITION, VELOCITY, INVALID } CONTROL;
char *ctrl_type[JOINTS] = { "p" };
RTAPI_MP_ARRAY_STRING(ctrl_type,JOINTS,"control type (pos or vel)");

int PRU_base_freq = -1;
RTAPI_MP_INT(PRU_base_freq, "PRU base thread frequency");

#define DST_PORT 27181
#define SRC_PORT 27181
#define SEND_TIMEOUT_US 10
#define RECV_TIMEOUT_US 10
#define READ_PCK_DELAY_NS 10000
#define COMMUNICATION_CYCLE_BUDGET_NS 500000LL
#define CONTROL_ACTIVATE_RETRY_LIMIT 3U
#define MIN_POS_SCALE 1e-20

static int udpSocket = -1;
static uint8_t readErrCount;
static uint8_t writeErrCount;
static bool enableWasActive = false;
static long long communicationCycleDeadline;
static bool communicationCycleDeadlineActive;
typedef enum
{
	CONTROL_SESSION_OPEN,
	CONTROL_SESSION_ACTIVATE,
	CONTROL_SESSION_ESTABLISHED
} controlSessionState_t;

typedef enum
{
	CONTROL_RESPONSE_ENVELOPE_ONLY,
	CONTROL_RESPONSE_NONZERO_CHALLENGE,
	CONTROL_RESPONSE_EXACT_CHALLENGE
} controlResponseValidation_t;

static controlSessionState_t controlSessionState;
static uint32_t controlSessionId;
static uint32_t controlSessionProposal;
static uint32_t establishmentSequence;
static uint32_t controlChallenge;
static uint32_t controlSessionToken;
static uint32_t nextControlSessionId;
static uint32_t nextEstablishmentSequence;
static uint32_t nextReadSequence;
static uint32_t nextWriteSequence;
static bool controlSessionNeedsRead;
static uint8_t establishmentRetryCount;
struct sockaddr_in dstAddr, srcAddr;
struct hostent *server;
static const char *dstAddress = "10.10.10.10";

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static int UDP_init(void);
static void UDP_close(void);

static void update_freq(void *arg, long period);
static void pru_write(void *arg, long period);
static void pru_read();
static bool sanitize_pos_scale(hal_float_t *scale);
static int32_t frequency_to_int32(double frequency);
static float setpoint_to_protocol_float(hal_float_t setpoint);
static long long create_communication_cycle_deadline(void);
static void initialize_control_session(void);
static void begin_control_session_attempt(void);
static void establish_control_session(
	long long responseDeadline);
static void prepare_control_request(
	controlEnvelope_t *envelope,
	uint32_t kind,
	uint32_t *nextSequence);
static bool pru_transfer(
	const uint8_t *requestBuffer,
	int txSize,
	uint8_t *responseBuffer,
	int rxSize,
	uint32_t expectedKind,
	controlResponseValidation_t responseValidation,
	uint32_t expectedChallenge,
	uint32_t expectedSessionToken,
	long long responseDeadline);
static void record_control_transfer_result(
	bool validResponse,
	uint8_t *errorCount);
static CONTROL parse_ctrl_type(const char *ctrl);



/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    char name[HAL_NAME_LEN + 1];
	int n, retval;

	// parse stepgen control type
	for (n = 0; n < JOINTS; n++) {
		if(parse_ctrl_type(ctrl_type[n]) == INVALID) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"STEPGEN: ERROR: bad control type '%s' for axis %i (must be 'p' or 'v')\n",
					ctrl_type[n], n);
			return -1;
		}
    }
	

	// check to see if the PRU base frequency has been set at the command line
	if (PRU_base_freq != -1)
	{
		if ((PRU_base_freq < 40000) || (PRU_base_freq > 500000))
		{
			rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: PRU base frequency incorrect\n");
			return -1;
		}
	}
	else
	{
		PRU_base_freq = PRU_BASEFREQ;
	}
	

    // connect to the HAL, initialise the driver
    comp_id = hal_init(modname);
    if (comp_id < 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed \n", modname);
		return -1;
    }

	// allocate shared memory
	data = hal_malloc(sizeof(data_t));
	if (data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: hal_malloc() failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	
	// Initialize the UDP socket
	if (UDP_init() < 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "Error: The board is unreachable\n");
		hal_exit(comp_id);
		return -1;
	}

	initialize_control_session();

	// export spiPRU SPI enable and status bits
	retval = hal_pin_bit_newf(HAL_IN, &(data->enable),
			comp_id, "%s.enable", prefix);
	if (retval != 0) goto error;
	
	retval = hal_pin_bit_newf(HAL_IN, &(data->reset),
			comp_id, "%s.reset", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_OUT, &(data->status),
			comp_id, "%s.status", prefix);
	if (retval != 0) goto error;



    // export all the variables for each joint
    for (n = 0; n < JOINTS; n++) {
		// export pins

		data->pos_mode[n] = (parse_ctrl_type(ctrl_type[n]) == POSITION);
/*
This is throwing errors from axis.py for some reason...
		
		if (data->pos_mode[n]){
			rtapi_print_msg(RTAPI_MSG_ERR, "Creating pos_mode[%d] = %d\n", n, data->pos_mode[n]);
			retval = hal_pin_float_newf(HAL_IN, &(data->pos_cmd[n]),
					comp_id, "%s.joint.%01d.pos-cmd", prefix, n);
			if (retval < 0) goto error;
			*(data->pos_cmd[n]) = 0.0;
		} else {
			rtapi_print_msg(RTAPI_MSG_ERR, "Creating vel_mode[%d] = %d\n", n, data->pos_mode[n]);
			retval = hal_pin_float_newf(HAL_IN, &(data->vel_cmd[n]),
					comp_id, "%s.joint.%01d.vel-cmd", prefix, n);
			if (retval < 0) goto error;
			*(data->vel_cmd[n]) = 0.0;			
		}
*/

		retval = hal_pin_bit_newf(HAL_IN, &(data->stepperEnable[n]),
				comp_id, "%s.joint.%01d.enable", prefix, n);
		if (retval != 0) goto error;

		retval = hal_pin_float_newf(HAL_IN, &(data->pos_cmd[n]),
				comp_id, "%s.joint.%01d.pos-cmd", prefix, n);
		if (retval < 0) goto error;
		*(data->pos_cmd[n]) = 0.0;
		
		if (data->pos_mode[n] == 0){
			retval = hal_pin_float_newf(HAL_IN, &(data->vel_cmd[n]),
					comp_id, "%s.joint.%01d.vel-cmd", prefix, n);
			if (retval < 0) goto error;
			*(data->vel_cmd[n]) = 0.0;			
		}

		retval = hal_pin_float_newf(HAL_OUT, &(data->freq_cmd[n]),
		        comp_id, "%s.joint.%01d.freq-cmd", prefix, n);
		if (retval < 0) goto error;
		*(data->freq_cmd[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->pos_fb[n]),
		        comp_id, "%s.joint.%01d.pos-fb", prefix, n);
		if (retval < 0) goto error;
		*(data->pos_fb[n]) = 0.0;
		
		retval = hal_param_float_newf(HAL_RW, &(data->pos_scale[n]),
		        comp_id, "%s.joint.%01d.scale", prefix, n);
		if (retval < 0) goto error;
		data->pos_scale[n] = 1.0;
		data->old_scale[n] = data->pos_scale[n];
		data->scale_recip[n] =
			(1.0 / STEP_MASK) / data->pos_scale[n];

		retval = hal_pin_s32_newf(HAL_OUT, &(data->count[n]),
		        comp_id, "%s.joint.%01d.counts", prefix, n);
		if (retval < 0) goto error;
		*(data->count[n]) = 0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->pgain[n]),
				comp_id, "%s.joint.%01d.pgain", prefix, n);
		if (retval < 0) goto error;
		*(data->pgain[n]) = 0.0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->ff1gain[n]),
				comp_id, "%s.joint.%01d.ff1gain", prefix, n);
		if (retval < 0) goto error;
		*(data->ff1gain[n]) = 0.0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->deadband[n]),
				comp_id, "%s.joint.%01d.deadband", prefix, n);
		if (retval < 0) goto error;
		*(data->deadband[n]) = 0.0;
		
		retval = hal_param_float_newf(HAL_RW, &(data->maxaccel[n]),
		        comp_id, "%s.joint.%01d.maxaccel", prefix, n);
		if (retval < 0) goto error;
		data->maxaccel[n] = 1.0;
	}

	for (n = 0; n < VARIABLES; n++) {
	// export pins

		retval = hal_pin_float_newf(HAL_IN, &(data->setPoint[n]),
		        comp_id, "%s.SP.%01d", prefix, n);
		if (retval < 0) goto error;
		*(data->setPoint[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->processVariable[n]),
		        comp_id, "%s.PV.%01d", prefix, n);
		if (retval < 0) goto error;
		*(data->processVariable[n]) = 0.0;
	}

	for (n = 0; n < DIGITAL_OUTPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_IN, &(data->outputs[n]),
				comp_id, "%s.output.%02d", prefix, n);
		if (retval != 0) goto error;
		*(data->outputs[n])=0;
	}

	for (n = 0; n < DIGITAL_INPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_OUT, &(data->inputs[n]),
				comp_id, "%s.input.%02d", prefix, n);
		if (retval != 0) goto error;
		*(data->inputs[n])=0;

		retval = hal_pin_bit_newf(HAL_OUT, &(data->inputs[n+DIGITAL_INPUTS]),
				comp_id, "%s.input.%02d.not", prefix, n);
		if (retval != 0) goto error;
		*(data->inputs[n+DIGITAL_INPUTS])=1;

	}
	
	for (n = 0; n < NVMPG_INPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_OUT, &(data->NVMPGinputs[n]),
				comp_id, "%s.NVMPGinput.%01d", prefix, n);
		if (retval != 0) goto error;
		*(data->NVMPGinputs[n])=0;
	}

	error:
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: pin export failed with err=%i\n",
		        modname, retval);
		UDP_close();
		hal_exit(comp_id);
		return -1;
	}


	// Export functions
	rtapi_snprintf(name, sizeof(name), "%s.update-freq", prefix);
	retval = hal_export_funct(name, update_freq, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: update function export failed\n", modname);
		UDP_close();
		hal_exit(comp_id);
		return -1;
	}

	rtapi_snprintf(name, sizeof(name), "%s.write", prefix);
	/* uses FP operations */
	retval = hal_export_funct(name, pru_write, 0, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: write function export failed\n", modname);
		UDP_close();
		hal_exit(comp_id);
		return -1;
	}

	rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
	retval = hal_export_funct(name, pru_read, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: read function export failed\n", modname);
		UDP_close();
		hal_exit(comp_id);
		return -1;
	}

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
	hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
	UDP_close();
    hal_exit(comp_id);
}


/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static void UDP_close(void)
{
	if (udpSocket >= 0)
	{
		if (close(udpSocket) < 0)
		{
			rtapi_print(
				"ERROR: can't close socket: %s\n",
				strerror(errno));
		}

		udpSocket = -1;
	}
}

int UDP_init(void)
{
	int ret;

	// Create a UDP socket
	udpSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (udpSocket < 0)
	{
		int saved_errno = errno;

		rtapi_print(
			"ERROR: can't open socket: %s\n",
			strerror(saved_errno));

		return -saved_errno;
	}

	bzero((char*) &dstAddr, sizeof(dstAddr));
	dstAddr.sin_family = AF_INET;
	dstAddr.sin_addr.s_addr = inet_addr(dstAddress);
	dstAddr.sin_port = htons(DST_PORT);

	bzero((char*) &srcAddr, sizeof(srcAddr));
	srcAddr.sin_family = AF_INET;
	srcAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	srcAddr.sin_port = htons(SRC_PORT);
	
	// bind the local socket to SCR_PORT
	ret = bind(udpSocket, (struct sockaddr *) &srcAddr, sizeof(srcAddr));
	if (ret < 0)
	{
		int saved_errno = errno;

		rtapi_print(
			"ERROR: can't bind: %s\n",
			strerror(saved_errno));

		UDP_close();
		return -saved_errno;
	}
	
	// Connect to send and receive only to the server_addr
	ret = connect(udpSocket, (struct sockaddr*) &dstAddr, sizeof(struct sockaddr_in));
	if (ret < 0)
	{
		int saved_errno = errno;

		rtapi_print(
			"ERROR: can't connect: %s\n",
			strerror(saved_errno));

		UDP_close();
		return -saved_errno;
	}

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = RECV_TIMEOUT_US;

	ret = setsockopt(udpSocket, SOL_SOCKET, SO_RCVTIMEO, (char*) &timeout, sizeof(timeout));
	if (ret < 0) {
		int saved_errno = errno;

		rtapi_print("ERROR: can't set receive timeout socket option: %s\n",
			strerror(saved_errno));

		UDP_close();
		return -saved_errno;
	}

	timeout.tv_usec = SEND_TIMEOUT_US;
	ret = setsockopt(udpSocket, SOL_SOCKET, SO_SNDTIMEO, (char*) &timeout,
	  sizeof(timeout));
	if (ret < 0) {
		int saved_errno = errno;

		rtapi_print("ERROR: can't set send timeout socket option: %s\n",
			strerror(saved_errno));

		UDP_close();
		return -saved_errno;
	}

	return 0;
}


static bool sanitize_pos_scale(hal_float_t *scale)
{
	if (!isfinite(*scale) || (fabs(*scale) < MIN_POS_SCALE))
	{
		*scale = 1.0;
		return false;
	}

	return true;
}


static int32_t frequency_to_int32(double frequency)
{
	if (!isfinite(frequency))
	{
		return 0;
	}

	if (frequency >= (double)INT32_MAX)
	{
		return INT32_MAX;
	}

	if (frequency <= (double)INT32_MIN)
	{
		return INT32_MIN;
	}

	return (int32_t)frequency;
}


static float setpoint_to_protocol_float(hal_float_t setpoint)
{
	if (!isfinite(setpoint) ||
		(setpoint > (hal_float_t)FLT_MAX) ||
		(setpoint < (hal_float_t)-FLT_MAX))
	{
		return 0.0F;
	}

	return (float)setpoint;
}


void update_freq(void *arg, long period)
{
	int i;
	data_t *data = (data_t *)arg;
	double periodfp;
	double periodrecip = 0.0;
	bool timingValid;

	// precalculate timing constants
	periodfp = (double)period * 0.000000001;
	timingValid = isfinite(periodfp) && (periodfp > 0.0);

	if (timingValid)
	{
		periodrecip = 1.0 / periodfp;
		timingValid = isfinite(periodrecip);
	}

	// calc constants related to the period of this function (LinuxCNC SERVO_THREAD)
	// only recalc constants if period changes
	if (period != old_dtns) 			// Note!! period = LinuxCNC SERVO_PERIOD
	{
		old_dtns = period;				// get ready to detect future period changes
		dt = timingValid ? periodfp : 0.0;
		recip_dt = timingValid ? periodrecip : 0.0;
	}

	// loop through generators
	for (i = 0; i < JOINTS; i++)
	{
		bool numericValid = timingValid;
		bool scaleStateValid;
		bool commandRepresentable = true;
		double scale;
		double scaleMagnitude;
		double reciprocal;
		double max_ac;
		double vel_cmd = 0.0;
		double dv = 0.0;
		double new_vel = 0.0;
		double max_freq = (double)PRU_base_freq;
		double desired_freq;
		double desired_accel;
		double error = 0.0;
		double command = 0.0;
		double feedback = 0.0;
		double commandDerivative = 0.0;
		double pgain = 1.0;
		double ff1gain = 1.0;
		double deadband = 0.0;
		double priorFrequency = (double)data->freq[i];
		double configuredMaxVelocity = data->maxvel[i];
		double configuredMaxAcceleration = data->maxaccel[i];

		if (!sanitize_pos_scale(&data->pos_scale[i]))
		{
			numericValid = false;
		}

		scale = data->pos_scale[i];
		scaleMagnitude = fabs(scale);
		scaleStateValid =
			isfinite(data->old_scale[i]) &&
			(fabs(data->old_scale[i]) >= MIN_POS_SCALE) &&
			isfinite(data->scale_recip[i]);

		if (!scaleStateValid)
		{
			numericValid = false;
		}

		// check for scale change
		if (!scaleStateValid || (scale != data->old_scale[i]))
		{
			reciprocal = (1.0 / STEP_MASK) / scale;

			if (!isfinite(reciprocal))
			{
				reciprocal = 0.0;
				numericValid = false;
			}

			data->old_scale[i] = scale;
			data->scale_recip[i] = reciprocal;
		}

		if (!isfinite(priorFrequency))
		{
			priorFrequency = 0.0;
			data->freq[i] = 0.0F;
			numericValid = false;
		}

		if (!isfinite(data->prev_cmd[i]))
		{
			data->prev_cmd[i] = 0.0F;
			numericValid = false;
		}

		if (!isfinite(data->cmd_d[i]))
		{
			data->cmd_d[i] = 0.0F;
			numericValid = false;
		}

		if (!isfinite(configuredMaxVelocity))
		{
			configuredMaxVelocity = 0.0;
			data->maxvel[i] = 0.0;
			numericValid = false;
		}

		// calculate frequency limit
		if (!isfinite(max_freq) || (max_freq < 0.0))
		{
			max_freq = 0.0;
			numericValid = false;
		}

		// check for user specified frequency limit parameter
		if (configuredMaxVelocity <= 0.0)
		{
			// set to zero if negative
			data->maxvel[i] = 0.0;
		}
		else
		{
			// parameter is non-zero, compare to max_freq
			desired_freq =
				configuredMaxVelocity * scaleMagnitude;

			if (!isfinite(desired_freq))
			{
				max_freq = 0.0;
				numericValid = false;
			}
			else if (desired_freq > max_freq)
			{
				// parameter is too high, limit it
				data->maxvel[i] =
					max_freq / scaleMagnitude;
			}
			else
			{
				// lower max_freq to match parameter
				max_freq = desired_freq;
			}
		}

		if (!isfinite(max_freq))
		{
			max_freq = 0.0;
			numericValid = false;
		}
		
		/* set internal accel limit to its absolute max, which is
		zero to full speed in one thread period */
		max_ac = max_freq * recip_dt;

		if (!isfinite(max_ac))
		{
			max_ac = 0.0;
			numericValid = false;
		}

		if (!isfinite(configuredMaxAcceleration))
		{
			configuredMaxAcceleration = 0.0;
			data->maxaccel[i] = 0.0;
			numericValid = false;
		}
		
		// check for user specified accel limit parameter
		if (configuredMaxAcceleration <= 0.0)
		{
			// set to zero if negative
			data->maxaccel[i] = 0.0;
		}
		else 
		{
			// parameter is non-zero, compare to max_ac
			desired_accel =
				configuredMaxAcceleration * scaleMagnitude;

			if (!isfinite(desired_accel))
			{
				max_ac = 0.0;
				numericValid = false;
			}
			else if (desired_accel > max_ac)
			{
				// parameter is too high, lower it
				data->maxaccel[i] =
					max_ac / scaleMagnitude;
			}
			else
			{
				// lower limit to match parameter
				max_ac = desired_accel;
			}
		}

		/* at this point, all scaling, limits, and other parameter
		changes have been handled - time for the main control */

		

		if (data->pos_mode[i]) {

			/* POSITION CONTROL MODE */

			// use Proportional control with feed forward (pgain, ff1gain and deadband)

			pgain = *(data->pgain[i]);
			ff1gain = *(data->ff1gain[i]);
			deadband = *(data->deadband[i]);
			command = *(data->pos_cmd[i]);
			feedback = *(data->pos_fb[i]);

			if (!isfinite(pgain) || (fabs(pgain) > FLT_MAX))
			{
				pgain = 1.0;
				numericValid = false;
			}
			else if (pgain == 0.0)
			{
				pgain = 1.0;
			}

			pgain = (double)(float)pgain;

			if (!isfinite(ff1gain) || (fabs(ff1gain) > FLT_MAX))
			{
				ff1gain = 1.0;
				numericValid = false;
			}
			else if (ff1gain == 0.0)
			{
				ff1gain = 1.0;
			}

			ff1gain = (double)(float)ff1gain;

			if (!isfinite(deadband) || (fabs(deadband) > FLT_MAX))
			{
				deadband = 0.0;
				numericValid = false;
			}
			else if (deadband == 0.0)
			{
				deadband = fabs(1.0 / scale);
			}

			deadband = (double)(float)deadband;

			commandRepresentable =
				isfinite(command) &&
				(fabs(command) <= FLT_MAX);

			if (!commandRepresentable)
			{
				command = (double)data->prev_cmd[i];
				numericValid = false;
			}

			if (!isfinite(feedback))
			{
				feedback = 0.0;
				numericValid = false;
			}

			// calcuate the error
			error = command - feedback;

			if (!isfinite(error))
			{
				error = 0.0;
				numericValid = false;
			}

			// apply the deadband
			if (error > deadband)
			{
				error -= deadband;
			}
			else if (error < -deadband)
			{
				error += deadband;
			}
			else
			{
				error = 0;
			}

			// calcuate command and derivatives
			commandDerivative =
				(command - (double)data->prev_cmd[i]) *
				periodrecip;

			if (!isfinite(commandDerivative) ||
				(fabs(commandDerivative) > FLT_MAX))
			{
				commandDerivative = 0.0;
				numericValid = false;
			}

			if (numericValid)
			{
				data->cmd_d[i] =
					(float)commandDerivative;
			}
			else
			{
				data->cmd_d[i] = 0.0F;
			}

			commandDerivative =
				(double)data->cmd_d[i];

			data->prev_cmd[i] = (float)command;

			// calculate the output value
			vel_cmd =
				pgain * error +
				commandDerivative * ff1gain;

			if (!isfinite(vel_cmd))
			{
				vel_cmd = 0.0;
				numericValid = false;
			}

			if (!numericValid)
			{
				data->cmd_d[i] = 0.0F;
			}

		} else {

			/* VELOCITY CONTROL MODE */

			// calculate velocity command in counts/sec
			vel_cmd = *(data->vel_cmd[i]);

			if (!isfinite(vel_cmd))
			{
				vel_cmd = 0.0;
				numericValid = false;
			}
		}

		if (numericValid)
		{
			vel_cmd = vel_cmd * scale;

			if (!isfinite(vel_cmd))
			{
				vel_cmd = 0.0;
				numericValid = false;
			}
		}

		// apply frequency limit
		if (numericValid && (vel_cmd > max_freq))
		{
			vel_cmd = max_freq;
		}
		else if (numericValid && (vel_cmd < -max_freq))
		{
			vel_cmd = -max_freq;
		}

		// calc max change in frequency in one period
		if (numericValid)
		{
			dv = max_ac * dt;

			if (!isfinite(dv))
			{
				dv = 0.0;
				numericValid = false;
			}
		}

		// apply accel limit
		if (numericValid &&
			(vel_cmd > (priorFrequency + dv)))
		{
			new_vel = priorFrequency + dv;
		}
		else if (numericValid &&
			(vel_cmd < (priorFrequency - dv)))
		{
			new_vel = priorFrequency - dv;
		}
		else if (numericValid)
		{
			new_vel = vel_cmd;
		}

		if (!isfinite(new_vel) ||
			(fabs(new_vel) > FLT_MAX))
		{
			new_vel = 0.0;
			numericValid = false;
		}

		// test for disabled stepgen
		if (*(data->stepperEnable[i]) == 0) {
			// set velocity to zero
			new_vel = 0;
		}

		if (!numericValid)
		{
			new_vel = 0.0;

			if (data->pos_mode[i])
			{
				data->cmd_d[i] = 0.0F;
			}
		}

		data->freq[i] = (float)new_vel;		// to be sent to the PRU
		*(data->freq_cmd[i]) = data->freq[i];	// feedback to LinuxCNC
	}

}


static long long create_communication_cycle_deadline(void)
{
	long long currentTime = rtapi_get_time();

	if (currentTime > (LLONG_MAX - COMMUNICATION_CYCLE_BUDGET_NS))
	{
		return LLONG_MAX;
	}

	return currentTime + COMMUNICATION_CYCLE_BUDGET_NS;
}

static void initialize_control_session(void)
{
	long long signedStartupTime = rtapi_get_time();
	uint64_t startupTime =
		(signedStartupTime > 0) ?
			(uint64_t)signedStartupTime :
			UINT64_C(1);
	uint32_t lowStartupTime =
		(uint32_t)startupTime;
	uint32_t highStartupTime =
		(uint32_t)(
			startupTime >> 32U);

	nextControlSessionId =
		lowStartupTime ^
		(highStartupTime * UINT32_C(0x9e3779b9));
	nextEstablishmentSequence =
		highStartupTime ^
		(lowStartupTime * UINT32_C(0x85ebca6b));

	if (nextControlSessionId == 0U)
	{
		nextControlSessionId = 1U;
	}

	if (nextEstablishmentSequence == 0U)
	{
		nextEstablishmentSequence = 1U;
	}

	begin_control_session_attempt();
}

static uint32_t advance_nonzero_control_value(
	uint32_t value)
{
	return
		(value == UINT32_MAX) ?
			1U :
			value + 1U;
}

static uint32_t control_sequence_seed(
	uint32_t sessionId,
	uint32_t directionOffset)
{
	const uint64_t nonzeroRange =
		(uint64_t)UINT32_MAX;
	const uint64_t zeroBasedSeed =
		(uint64_t)(sessionId - 1U) +
		(uint64_t)directionOffset;
	const uint64_t wrappedSeed =
		(zeroBasedSeed >= nonzeroRange) ?
			zeroBasedSeed - nonzeroRange :
			zeroBasedSeed;

	return (uint32_t)wrappedSeed +
		1U;
}

static void begin_control_session_attempt(void)
{
	controlSessionProposal =
		nextControlSessionId;
	establishmentSequence =
		nextEstablishmentSequence;
	nextControlSessionId =
		advance_nonzero_control_value(
			nextControlSessionId);
	nextEstablishmentSequence =
		advance_nonzero_control_value(
			nextEstablishmentSequence);
	controlChallenge = 0U;
	controlSessionToken = 0U;
	controlSessionId = 0U;
	nextReadSequence = 0U;
	nextWriteSequence = 0U;
	controlSessionNeedsRead = false;
	establishmentRetryCount = 0U;
	controlSessionState =
		CONTROL_SESSION_OPEN;
}

static void establish_control_session(
	long long responseDeadline)
{
	controlEstablishment_t request = {0};
	controlEstablishment_t response = {0};
	uint32_t expectedKind;
	const bool activating =
		controlSessionState ==
			CONTROL_SESSION_ACTIVATE;

	request.envelope.protocolVersion =
		CONTROL_PROTOCOL_VERSION;
	request.envelope.sessionId =
		controlSessionProposal;
	request.envelope.sequence =
		establishmentSequence;

	if (controlSessionState == CONTROL_SESSION_OPEN)
	{
		request.envelope.kind =
			CONTROL_KIND_OPEN;
		request.challenge = 0U;
		request.sessionToken = 0U;
		expectedKind =
			CONTROL_KIND_CHALLENGE;
	}
	else
	{
		request.envelope.kind =
			CONTROL_KIND_ACTIVATE;
		request.challenge =
			controlChallenge;
		request.sessionToken =
			controlSessionToken;
		expectedKind =
			CONTROL_KIND_ESTABLISHED;
	}

	bool validResponse =
		pru_transfer(
			(const uint8_t *)&request,
			CONTROL_ESTABLISHMENT_SIZE,
			(uint8_t *)&response,
			CONTROL_ESTABLISHMENT_SIZE,
			expectedKind,
			(controlSessionState ==
			 CONTROL_SESSION_OPEN) ?
				CONTROL_RESPONSE_NONZERO_CHALLENGE :
				CONTROL_RESPONSE_EXACT_CHALLENGE,
			controlChallenge,
			controlSessionToken,
			responseDeadline);

	record_control_transfer_result(
		validResponse,
		&readErrCount);

	if (!validResponse)
	{
		if (activating)
		{
			if (establishmentRetryCount <
				CONTROL_ACTIVATE_RETRY_LIMIT)
			{
				establishmentRetryCount++;
			}

			if (establishmentRetryCount >=
				CONTROL_ACTIVATE_RETRY_LIMIT)
			{
				begin_control_session_attempt();
			}
		}

		return;
	}

	establishmentRetryCount = 0U;

	if (controlSessionState ==
		CONTROL_SESSION_OPEN)
	{
		controlChallenge =
			response.challenge;
		controlSessionToken =
			response.sessionToken;
		controlSessionState =
			CONTROL_SESSION_ACTIVATE;
		return;
	}

	controlSessionId =
		controlSessionToken;
	nextReadSequence =
		control_sequence_seed(
			controlSessionId,
			UINT32_C(0x72656164));
	nextWriteSequence =
		control_sequence_seed(
			controlSessionId,
			UINT32_C(0x77726974));
	controlSessionNeedsRead = true;
	controlSessionState =
		CONTROL_SESSION_ESTABLISHED;
}

static void prepare_control_request(
	controlEnvelope_t *envelope,
	uint32_t kind,
	uint32_t *nextSequence)
{
	uint32_t sequence = *nextSequence;

	if (sequence == 0U)
	{
		sequence = 1U;
	}

	envelope->protocolVersion =
		CONTROL_PROTOCOL_VERSION;
	envelope->sessionId =
		controlSessionId;
	envelope->sequence =
		sequence;
	envelope->kind =
		kind;

	*nextSequence =
		(sequence == UINT32_MAX) ?
			1U :
			sequence + 1U;
}


void pru_read()
{
	int i;
	double curr_pos;
	long long responseDeadline;
	bool resetRising =
		*(data->reset) &&
		!data->resetOld;

	communicationCycleDeadline =
		create_communication_cycle_deadline();
	communicationCycleDeadlineActive = true;
	responseDeadline = communicationCycleDeadline;
	
	if (*(data->enable))
	{
		if (resetRising &&
			!*(data->status) &&
			(controlSessionState ==
			 CONTROL_SESSION_ESTABLISHED))
		{
			begin_control_session_attempt();
		}

		if (controlSessionState !=
			CONTROL_SESSION_ESTABLISHED)
		{
			establish_control_session(
				responseDeadline);
		}
		else if (resetRising ||
				 *(data->status) ||
				 controlSessionNeedsRead)
		{
			// reset rising edge detected, try transfer and reset OR PRU running

			prepare_control_request(
				&txData.envelope,
				CONTROL_KIND_READ,
				&nextReadSequence);
			
			// Transfer to and from the PRU
			const bool validResponse =
				pru_transfer(
				txData.txBuffer,
				CONTROL_READ_PACKET_SIZE,
				rxData.rxBuffer,
				CONTROL_DATA_PACKET_SIZE,
				CONTROL_KIND_DATA,
				CONTROL_RESPONSE_ENVELOPE_ONLY,
				0U,
				0U,
				responseDeadline);

			record_control_transfer_result(
				validResponse,
				&readErrCount);
			
			switch (rxData.envelope.kind)		// only process valid SPI payloads. This rejects bad payloads
			{
				case CONTROL_KIND_DATA:
					// we have received a GOOD payload from the PRU
					*(data->status) = 1;
					controlSessionNeedsRead = false;

					for (i = 0; i < JOINTS; i++)
					{
						bool scaleValid;

						count[i] = rxData.jointFeedback[i];
						
						*(data->count[i]) = count[i];
						scaleValid =
							sanitize_pos_scale(
								&data->pos_scale[i]);

						if (!scaleValid)
						{
							data->old_scale[i] = 0.0;
							data->scale_recip[i] = 0.0;
						}

						curr_pos =
							(float)(count[i]) /
							data->pos_scale[i];

						if (!isfinite(curr_pos))
						{
							curr_pos = 0.0;
						}

						*(data->pos_fb[i]) = curr_pos;
					}

					// Feedback
					for (i = 0; i < VARIABLES; i++)
					{
						*(data->processVariable[i]) = rxData.processVariable[i]; 
					}

					// Inputs
					for (i = 0; i < DIGITAL_INPUTS; i++)
					{
						const uint32_t inputMask =
							UINT32_C(1) << i;

						if ((rxData.inputs & inputMask) != UINT32_C(0))
						{
							*(data->inputs[i]) = 1; 		// input is high
							*(data->inputs[i+DIGITAL_INPUTS]) = 0; 		// inverted
						}
						else
						{
							*(data->inputs[i]) = 0;			// input is low
							*(data->inputs[i+DIGITAL_INPUTS]) = 1; 		// inverted
						}
					}
					
					// NVMPG Inputs
					for (i = 0; i < NVMPG_INPUTS; i++)
					{
						if ((rxData.NVMPGinputs & (1 << i)) != 0)
						{
							*(data->NVMPGinputs[i]) = 1; 		// input is high
						}
						else
						{
							*(data->NVMPGinputs[i]) = 0;			// input is low
						}
					}
					
					break;
				
				case CONTROL_KIND_ACKNOWLEDGE:
					// we've dropped a packet somewhere but comms are still up
					break;
					
				case PRU_ERR:
					// we've dropped a packet somewhere but comms are still up
					break;
				
				case PRU_ESTOP:
					// we have an eStop notification from the PRU
					*(data->status) = 0;
					 rtapi_print_msg(RTAPI_MSG_ERR, "An E-stop is active");
					break;

				default:
					// we have received a BAD payload from the PRU
					*(data->status) = 0;
					rtapi_print(
						"Bad payload = %x\n",
						rxData.envelope.kind);
					break;
			}
		}
	}
	else
	{
		*(data->status) = 0;
	}
	
	data->resetOld = *(data->reset);
}


static void pru_write(void *arg, long period)
{
	int i;
	bool enableActive = *(data->enable);
	bool sendSafeStop = enableWasActive && !enableActive;
	long long responseDeadline;

	(void)arg;
	(void)period;

	if (!communicationCycleDeadlineActive)
	{
		communicationCycleDeadline =
			create_communication_cycle_deadline();
		communicationCycleDeadlineActive = true;
	}

	responseDeadline = communicationCycleDeadline;

	enableWasActive = enableActive;

	// Joint frequency commands
	for (i = 0; i < JOINTS; i++)
	{
		txData.jointFreqCmd[i] =
			frequency_to_int32(data->freq[i]);
	}

	for (i = 0; i < JOINTS; i++)
	{
		if (*(data->stepperEnable[i]) == 1)
		{
			txData.jointEnable |= (1 << i);		
		}
		else
		{
			txData.jointEnable &= ~(1 << i);	
		}
	}

	// Set points
	for (i = 0; i < VARIABLES; i++)
	{
		txData.setPoint[i] =
			setpoint_to_protocol_float(
				*(data->setPoint[i]));
	}

	// Outputs
	for (i = 0; i < DIGITAL_OUTPUTS; i++)
	{
		const uint32_t outputMask =
			UINT32_C(1) << i;

		if (*(data->outputs[i]) == 1)
		{
			txData.outputs |= outputMask;		// output is high
		}
		else
		{
			txData.outputs &= ~outputMask;	// output is low
		}
	}

	if (sendSafeStop)
	{
		memset(
			txData.txBuffer,
			0,
			sizeof(txData.txBuffer));
	}

	if ((*(data->status) || sendSafeStop) &&
		(controlSessionState ==
		 CONTROL_SESSION_ESTABLISHED))
	{
		prepare_control_request(
			&txData.envelope,
			CONTROL_KIND_WRITE,
			&nextWriteSequence);

		// Transfer to and from the PRU
		const bool validResponse =
			pru_transfer(
			txData.txBuffer,
			CONTROL_WRITE_PACKET_SIZE,
			rxData.rxBuffer,
			CONTROL_ACK_PACKET_SIZE,
			CONTROL_KIND_ACKNOWLEDGE,
			CONTROL_RESPONSE_ENVELOPE_ONLY,
			0U,
			0U,
			responseDeadline);

		record_control_transfer_result(
			validResponse,
			&writeErrCount);
		
		switch (rxData.envelope.kind)
		{
			case CONTROL_KIND_DATA:
				// we've dropped a packet somewhere but comms are still up
				break;
			
			case CONTROL_KIND_ACKNOWLEDGE:
				// this is the response we expect
				break;
				
			case PRU_ERR:
				// there was a write error
				rtapi_print(
					"Data write error: %x\n",
					rxData.envelope.kind);
				break;
			
			case PRU_ESTOP:
				// we have an eStop notification from the PRU
				*(data->status) = 0;
				 rtapi_print_msg(RTAPI_MSG_ERR, "An E-stop is active");
				break;

			default:
				// we have received a BAD payload from the PRU
				*(data->status) = 0;
				rtapi_print(
					"Bad payload = %x\n",
					rxData.envelope.kind);
				break;
		}	
	}

	communicationCycleDeadlineActive = false;
}


static bool pru_transfer(
	const uint8_t *requestBuffer,
	int txSize,
	uint8_t *responseBuffer,
	int rxSize,
	uint32_t expectedKind,
	controlResponseValidation_t responseValidation,
	uint32_t expectedChallenge,
	uint32_t expectedSessionToken,
	long long responseDeadline)
{
	int ret;
	long long currentTime;
	uint8_t receiveBuffer[CONTROL_DATA_PACKET_SIZE];
	bool validResponse = false;
	controlEnvelope_t expectedEnvelope;
	controlEnvelope_t errorEnvelope = {0};

	memcpy(
		&expectedEnvelope,
		requestBuffer,
		sizeof(expectedEnvelope));

	// Send datagram
	ret = send(
		udpSocket,
		requestBuffer,
		txSize,
		0);

	memset(
		responseBuffer,
		0,
		rxSize);
	errorEnvelope.kind =
		(uint32_t)PRU_ERR;
	memcpy(
		responseBuffer,
		&errorEnvelope,
		sizeof(errorEnvelope));

	if (ret == txSize)
	{
		// Receive incoming datagram
	    currentTime = rtapi_get_time();
	    while (!validResponse && (currentTime < responseDeadline))
	    {
	        ret = recv(udpSocket, receiveBuffer, rxSize, MSG_TRUNC);
	        if (ret == rxSize)
	        {
		        controlEnvelope_t receivedEnvelope;

		        memcpy(
			        &receivedEnvelope,
			        receiveBuffer,
			        sizeof(receivedEnvelope));

		        if ((receivedEnvelope.protocolVersion ==
			         expectedEnvelope.protocolVersion) &&
			        (receivedEnvelope.sessionId ==
			         expectedEnvelope.sessionId) &&
			        (receivedEnvelope.sequence ==
			         expectedEnvelope.sequence) &&
			        (receivedEnvelope.kind ==
			         expectedKind))
		        {
			        bool responseContentValid = true;

			        if (responseValidation !=
				        CONTROL_RESPONSE_ENVELOPE_ONLY)
			        {
				        controlEstablishment_t
					        establishmentResponse;

				        memcpy(
					        &establishmentResponse,
					        receiveBuffer,
					        sizeof(establishmentResponse));

				        if (responseValidation ==
					        CONTROL_RESPONSE_NONZERO_CHALLENGE)
				        {
					        responseContentValid =
						        (establishmentResponse.challenge !=
						         0U) &&
						        (establishmentResponse.sessionToken !=
						         0U);
				        }
				        else
				        {
					        responseContentValid =
						        (establishmentResponse.challenge ==
						         expectedChallenge) &&
						        (establishmentResponse.sessionToken ==
						         expectedSessionToken);
				        }
			        }

			        if (responseContentValid)
			        {
				        memcpy(
					        responseBuffer,
					        receiveBuffer,
					        rxSize);

				        validResponse = true;
				        break;
			        }
		        }
	        }
	        if(ret < 0) rtapi_delay(READ_PCK_DELAY_NS);
	        currentTime = rtapi_get_time();
	    }
	}
	else
	{
		ret = -1;
	}

	return validResponse;
}

static void record_control_transfer_result(
	bool validResponse,
	uint8_t *errorCount)
{
	if (validResponse)
	{
		*errorCount = 0U;
		return;
	}

	if (*errorCount < 3U)
	{
		(*errorCount)++;
	}
	
	if (*errorCount > 2U)
	{
		*(data->status) = 0;
		rtapi_print("Ethernet ERROR: %s\n", strerror(errno));
	}
}


static CONTROL parse_ctrl_type(const char *ctrl)
{
    if(!ctrl || !*ctrl || *ctrl == 'p' || *ctrl == 'P') return POSITION;
    if(*ctrl == 'v' || *ctrl == 'V') return VELOCITY;
    return INVALID;
}
