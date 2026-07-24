#include "lwip/opt.h"

#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "lwip/init.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"

#include "board.h"
#include "ethernet.h"

#include "fsl_silicon_id.h"
#include "fsl_phy.h"
#include "fsl_phylan8720a.h"
#include "fsl_enet_mdio.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"

#include "configuration.h"
#include "remora.h"
#include "extern.h"


#define CONTROL_MAINTENANCE_INTERVAL_MS UINT32_C(30000)

static mdio_handle_t mdioHandle = {.ops = &enet_ops};
static phy_handle_t phyHandle   = {.phyAddr = BOARD_ENET0_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &phylan8720a_ops};
static bool ethernetInitialized = false;
static bool controlSessionEstablished = false;
static uint32_t activeControlSession = 0U;
static bool controlOpenPending = false;
static uint32_t pendingControlSession = 0U;
static uint32_t pendingControlSequence = 0U;
static uint32_t pendingControlChallenge = 0U;
static uint32_t pendingControlToken = 0U;
static uint32_t controlChallengeCounter = 0U;
static bool controlHandoffRequested = false;
static bool controlHandoffCompleted = false;
static bool readSequenceInitialized = false;
static uint32_t lastReadSequence = 0U;
static bool writeSequenceInitialized = false;
static uint32_t lastWriteSequence = 0U;
static bool maintenanceIntervalActive = false;
static bool maintenanceSequenceInitialized = false;
static uint32_t latestMaintenanceSequence = 0U;
static uint32_t maintenanceExpirationDeadline = 0U;
struct netif netif;

void udp_data_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udp_mpg_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

static bool controlSerialNumberIsNewer(
	uint32_t candidate,
	uint32_t reference)
{
	const uint32_t difference =
		candidate - reference;

	return
		(difference != 0U) &&
		(difference < UINT32_C(0x80000000));
}

static void clearMaintenanceState(void)
{
	maintenanceIntervalActive = false;
	maintenanceSequenceInitialized = false;
	latestMaintenanceSequence = 0U;
	maintenanceExpirationDeadline = 0U;
}

static bool maintenanceDeadlineReached(
	uint32_t currentTime)
{
	const uint32_t timeSinceDeadline =
		currentTime -
		maintenanceExpirationDeadline;

	return
		timeSinceDeadline <
			UINT32_C(0x80000000);
}

static void expireMaintenanceInterval(
	uint32_t currentTime)
{
	if (maintenanceIntervalActive &&
		maintenanceDeadlineReached(
			currentTime))
	{
		maintenanceIntervalActive = false;
		maintenanceExpirationDeadline = 0U;
	}
}

static bool nextControlChallenge(
	uint32_t &challenge)
{
	if (controlChallengeCounter ==
		UINT32_MAX)
	{
		return false;
	}

	controlChallengeCounter++;
	challenge =
		controlChallengeCounter;
	return true;
}

static bool pendingControlAttemptMatches(
	const controlEstablishment_t &request)
{
	return
		controlOpenPending &&
		(request.envelope.sessionId ==
		 pendingControlSession) &&
		(request.envelope.sequence ==
		 pendingControlSequence);
}

bool controlSessionHandoffPending(void)
{
	return
		controlOpenPending &&
		controlHandoffRequested &&
		!controlHandoffCompleted;
}

void controlSessionHandoffComplete(void)
{
	if (!controlSessionHandoffPending())
	{
		return;
	}

	activeControlSession =
		pendingControlToken;
	readSequenceInitialized = false;
	writeSequenceInitialized = false;
	clearMaintenanceState();
	controlSessionEstablished = true;
	controlHandoffRequested = false;
	controlHandoffCompleted = true;
}

static bool acceptReadSequence(
	uint32_t sequence)
{
	if (!readSequenceInitialized)
	{
		readSequenceInitialized = true;
		lastReadSequence = sequence;
		return true;
	}

	if (!controlSerialNumberIsNewer(
			sequence,
			lastReadSequence))
	{
		return false;
	}

	lastReadSequence = sequence;
	return true;
}

typedef enum
{
	WRITE_PACKET_REJECT,
	WRITE_PACKET_APPLY,
	WRITE_PACKET_ACKNOWLEDGE
} writePacketDisposition_t;

static writePacketDisposition_t classifyWriteSequence(
	uint32_t sequence)
{
	if (!writeSequenceInitialized)
	{
		writeSequenceInitialized = true;
		lastWriteSequence = sequence;
		return WRITE_PACKET_APPLY;
	}

	if (sequence == lastWriteSequence)
	{
		return WRITE_PACKET_ACKNOWLEDGE;
	}

	if (!controlSerialNumberIsNewer(
			sequence,
			lastWriteSequence))
	{
		return WRITE_PACKET_REJECT;
	}

	lastWriteSequence = sequence;
	return WRITE_PACKET_APPLY;
}

void initEthernet(void)
{
    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
    ethernetif_config_t enet_config = {
        .phyHandle = &phyHandle
    };

    const clock_enet_pll_config_t config = {.enableClkOutput = true, .enableClkOutput25M = false, .loopDivider = 1};
    CLOCK_InitEnetPll(&config);

    IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);

    mdioHandle.resource.csrClock_Hz = CLOCK_GetFreq(kCLOCK_IpgClk);

    (void)SILICONID_ConvertToMacAddr(&enet_config.macAddress);

    IP4_ADDR(&netif_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
    IP4_ADDR(&netif_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
    IP4_ADDR(&netif_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);

    lwip_init();

    struct netif *addedNetif =
        netif_add(
            &netif,
            &netif_ipaddr,
            &netif_netmask,
            &netif_gw,
            &enet_config,
            ethernetif0_init,
            ethernet_input);

    if (addedNetif == nullptr)
    {
        PRINTF("Failed to initialize Ethernet interface !\r\n");
        return;
    }

    netif_set_default(&netif);
    netif_set_up(&netif);

    ethernetInitialized = true;
}

void EthernetTasks(void)
{
    expireMaintenanceInterval(
        sys_now());

    if (ethernetInitialized)
    {
        ethernetif_input(&netif);
    }

    sys_check_timeouts();
}

void udpServer_init(void)
{
   struct udp_pcb *upcb = nullptr;
   struct udp_pcb *upcb2 = nullptr;
   err_t err;

   ip_addr_t myIPADDR;
   IP_ADDR4(&myIPADDR, 10, 10, 10, 10);

   // UDP control block for data
   upcb = udp_new();
   if (upcb == nullptr)
   {
	   PRINTF(
		   "Failed to allocate control UDP PCB !\r\n");
   }
   else
   {
	   err = udp_bind(upcb, &myIPADDR, 27181);  // 27181 is the server UDP port

	   /* 3. Set a receive callback for the upcb */
	   if(err == ERR_OK)
	   {
		   udp_recv(upcb, udp_data_callback, nullptr);
	   }
	   else
	   {
		   PRINTF(
			   "Failed to bind control UDP PCB !\r\n");

		   udp_remove(upcb);
		   upcb = nullptr;
	   }
   }


   // UDP control block for MPG
   upcb2 = udp_new();
   if (upcb2 == nullptr)
   {
	   PRINTF(
		   "Failed to allocate NVMPG UDP PCB !\r\n");
   }
   else
   {
	   err = udp_bind(upcb2, &myIPADDR, 27182);  // 27182 is the server UDP port for NVMPG

	   if(err == ERR_OK)
	   {
		   udp_recv(upcb2, udp_mpg_callback, nullptr);
	   }
	   else
	   {
		   PRINTF(
			   "Failed to bind NVMPG UDP PCB !\r\n");

		   udp_remove(upcb2);
		   upcb2 = nullptr;
	   }
   }
}

void udp_data_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	int txlen = 0;
	struct pbuf *txBuf;
	controlEnvelope_t requestEnvelope = {};
	uint8_t responseData[CONTROL_DATA_PACKET_SIZE] = {0};

	if (p == nullptr)
	{
		return;
	}

	if ((p->tot_len < sizeof(requestEnvelope)) ||
		(pbuf_copy_partial(
			p,
			&requestEnvelope,
			sizeof(requestEnvelope),
			0) != sizeof(requestEnvelope)))
	{
		pbuf_free(p);
		return;
	}

	if ((requestEnvelope.protocolVersion !=
		 CONTROL_PROTOCOL_VERSION) ||
		(requestEnvelope.sessionId == 0U) ||
		(requestEnvelope.sequence == 0U))
	{
		pbuf_free(p);
		return;
	}

	if ((requestEnvelope.kind == CONTROL_KIND_OPEN) ||
		(requestEnvelope.kind == CONTROL_KIND_ACTIVATE))
	{
		if (p->tot_len != CONTROL_ESTABLISHMENT_SIZE)
		{
			pbuf_free(p);
			return;
		}

		controlEstablishment_t request = {};

		if (pbuf_copy_partial(
				p,
				&request,
				sizeof(request),
				0) != sizeof(request))
		{
			pbuf_free(p);
			return;
		}

		controlEstablishment_t response =
			request;

		if (requestEnvelope.kind == CONTROL_KIND_OPEN)
		{
			if ((request.challenge != 0U) ||
				(request.sessionToken != 0U))
			{
				pbuf_free(p);
				return;
			}

			if (controlHandoffCompleted ||
				!pendingControlAttemptMatches(
					request))
			{
				uint32_t challenge = 0U;

				if (!nextControlChallenge(
						challenge))
				{
					pbuf_free(p);
					return;
				}

				controlOpenPending = true;
				pendingControlSession =
					request.envelope.sessionId;
				pendingControlSequence =
					request.envelope.sequence;
				pendingControlChallenge =
					challenge;
				pendingControlToken =
					pendingControlChallenge;
				controlHandoffRequested = false;
				controlHandoffCompleted = false;
			}

			response.envelope.kind =
				CONTROL_KIND_CHALLENGE;
			response.challenge =
				pendingControlChallenge;
			response.sessionToken =
				pendingControlToken;
		}
		else
		{
			if (!pendingControlAttemptMatches(
					request) ||
				(request.challenge == 0U) ||
				(request.challenge !=
				 pendingControlChallenge) ||
				(request.sessionToken == 0U) ||
				(request.sessionToken !=
				 pendingControlToken))
			{
				pbuf_free(p);
				return;
			}

			if (!controlHandoffCompleted)
			{
				controlHandoffRequested = true;
				pbuf_free(p);
				return;
			}

			response.envelope.kind =
				CONTROL_KIND_ESTABLISHED;
		}

		memcpy(
			responseData,
			&response,
			sizeof(response));

		txlen = CONTROL_ESTABLISHMENT_SIZE;
	}
	else if (requestEnvelope.kind ==
			 CONTROL_KIND_MAINTENANCE_REQUEST)
	{
		if (p->tot_len !=
			CONTROL_MAINTENANCE_REQUEST_SIZE)
		{
			pbuf_free(p);
			return;
		}

		const uint32_t currentTime =
			sys_now();

		expireMaintenanceInterval(
			currentTime);

		if (!controlSessionEstablished ||
			(requestEnvelope.sessionId !=
			 activeControlSession))
		{
			pbuf_free(p);
			return;
		}

		bool respondToRequest = false;

		if (maintenanceIntervalActive)
		{
			respondToRequest =
				maintenanceSequenceInitialized &&
				(requestEnvelope.sequence ==
				 latestMaintenanceSequence);
		}
		else if (!maintenanceSequenceInitialized ||
				 controlSerialNumberIsNewer(
					requestEnvelope.sequence,
					latestMaintenanceSequence))
		{
			maintenanceSequenceInitialized = true;
			latestMaintenanceSequence =
				requestEnvelope.sequence;
			maintenanceExpirationDeadline =
				currentTime +
				CONTROL_MAINTENANCE_INTERVAL_MS;
			maintenanceIntervalActive = true;
			respondToRequest = true;
		}

		if (!respondToRequest)
		{
			pbuf_free(p);
			return;
		}

		controlEnvelope_t responseEnvelope =
			requestEnvelope;

		responseEnvelope.kind =
			CONTROL_KIND_MAINTENANCE_READY;

		memcpy(
			responseData,
			&responseEnvelope,
			sizeof(responseEnvelope));

		txlen =
			CONTROL_MAINTENANCE_READY_SIZE;
	}
	else if (requestEnvelope.kind == CONTROL_KIND_READ)
	{
		if (p->tot_len != CONTROL_READ_PACKET_SIZE)
		{
			pbuf_free(p);
			return;
		}

		if (!controlSessionEstablished ||
			(requestEnvelope.sessionId !=
			 activeControlSession) ||
			!acceptReadSequence(
				requestEnvelope.sequence))
		{
			pbuf_free(p);
			return;
		}

		const uint32_t readPrimask =
			__get_PRIMASK();

		__disable_irq();

		for (size_t i = 0;
			 i < sizeof(txData.txBuffer);
			 i++)
		{
			responseData[i] =
				txData.txBuffer[i];
		}

		__set_PRIMASK(
			readPrimask);

		controlEnvelope_t responseEnvelope =
			requestEnvelope;

		responseEnvelope.kind =
			CONTROL_KIND_DATA;

		memcpy(
			responseData,
			&responseEnvelope,
			sizeof(responseEnvelope));

		txlen = CONTROL_DATA_PACKET_SIZE;
	}
	else if (requestEnvelope.kind == CONTROL_KIND_WRITE)
	{
		if (p->tot_len != CONTROL_WRITE_PACKET_SIZE)
		{
			pbuf_free(p);
			return;
		}

		uint8_t incomingData[sizeof(rxData.rxBuffer)];

		if (pbuf_copy_partial(
				p,
				incomingData,
				sizeof(incomingData),
				0) != sizeof(incomingData))
		{
			pbuf_free(p);
			return;
		}

		if (!controlSessionEstablished ||
			(requestEnvelope.sessionId !=
			 activeControlSession))
		{
			pbuf_free(p);
			return;
		}

		const writePacketDisposition_t disposition =
			classifyWriteSequence(
				requestEnvelope.sequence);

		if (disposition == WRITE_PACKET_REJECT)
		{
			pbuf_free(p);
			return;
		}

		if (disposition == WRITE_PACKET_ACKNOWLEDGE)
		{
			for (size_t i = 0;
				 i < sizeof(incomingData);
				 i++)
			{
				if (rxData.rxBuffer[i] !=
					incomingData[i])
				{
					pbuf_free(p);
					return;
				}
			}
		}

		if (disposition == WRITE_PACKET_APPLY)
		{
			// ensure an atomic access to the rxBuffer
			// disable thread interrupts
			const uint32_t writePrimask =
				__get_PRIMASK();

			__disable_irq();

			// then move the data
			//pragma pack should support memcpy, we should later look into the networking driver and see where p comes from,
			// and see if we can provide a data location to memcpy it in to which should further reduce cpu-time.
			// Of course not a lot but every little bit is enough.
			for (size_t i = 0;
				 i < sizeof(incomingData);
				 i++)
			{
				rxData.rxBuffer[i] =
					incomingData[i];
			}

			// restore previous interrupt state
			__set_PRIMASK(
				writePrimask);

			comms->dataReceived();
		}

		controlEnvelope_t responseEnvelope =
			requestEnvelope;

		responseEnvelope.kind =
			CONTROL_KIND_ACKNOWLEDGE;

		memcpy(
			responseData,
			&responseEnvelope,
			sizeof(responseEnvelope));

		txlen = CONTROL_ACK_PACKET_SIZE;
	}
	else
	{
		pbuf_free(p);
		return;
	}


	// allocate pbuf from RAM
	txBuf = pbuf_alloc(PBUF_TRANSPORT, txlen, PBUF_RAM);
	if (txBuf == nullptr)
	{
		pbuf_free(p);
		return;
	}

	// copy the data into the buffer
	if (pbuf_take(txBuf, responseData, txlen) != ERR_OK)
	{
		pbuf_free(txBuf);
		pbuf_free(p);
		return;
	}

	// Connect to the remote client
	const err_t connectStatus =
		udp_connect(
			upcb,
			addr,
			port);

	if (connectStatus != ERR_OK)
	{
		PRINTF(
			"Failed to connect control UDP reply PCB !\r\n");

		pbuf_free(txBuf);
		pbuf_free(p);
		return;
	}

	// Send a Reply to the Client
	const err_t sendStatus =
		udp_send(
			upcb,
			txBuf);

	// free the UDP connection, so we can accept new clients
	udp_disconnect(upcb);

	if (sendStatus != ERR_OK)
	{
		PRINTF(
			"Failed to send control UDP reply !\r\n");
	}

	// Free the p_tx buffer
	pbuf_free(txBuf);

	// Free the p buffer
	pbuf_free(p);
}

void udp_mpg_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	if (p == nullptr)
	{
		return;
	}

	if (p->tot_len != sizeof(mpgData.payload))
	{
		pbuf_free(p);
		return;
	}

	mpgData_t incomingMpgData = {};

	// copy the UDP payload into the nvmpg structure
	if (pbuf_copy_partial(
			p,
			incomingMpgData.payload,
			sizeof(incomingMpgData.payload),
			0) != sizeof(incomingMpgData.payload))
	{
		pbuf_free(p);
		return;
	}

	// Free the p buffer
	pbuf_free(p);

	if (incomingMpgData.header != PRU_NVMPG)
	{
		return;
	}

	if (MPG == nullptr)
	{
		return;
	}

	const uint32_t mpgPrimask =
		__get_PRIMASK();

	__disable_irq();

	memcpy(
		mpgData.payload,
		incomingMpgData.payload,
		sizeof(mpgData.payload));

	// use a standard module interface to trigger the update of the MPG
	MPG->configure();

	__set_PRIMASK(
		mpgPrimask);
}
