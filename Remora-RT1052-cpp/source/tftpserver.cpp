/**
  ******************************************************************************
  * @file    LwIP/LwIP_IAP/Src/tftpserver.c
  * @author  MCD Application Team
  * @brief   basic tftp server implementation for IAP (only Write Req supported)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tftpserver.h"
#include "flexspi_nor_flash.h"
#include "lwip/timeouts.h"
#include <string.h>
#include <stdio.h>
#include "configuration.h"
#include "extern.h"


/* Private variables ---------------------------------------------------------*/
static uint32_t Flash_Write_Address;
static struct udp_pcb *UDPpcb;
static __IO uint32_t total_count=0;
static edma_handle_t edma_handle;
static bool tftpTransferActive =
    false;
static bool tftpFinalizationPending =
    false;
static bool tftpPendingRestoreThreads =
    false;
static bool tftpPendingRestoreDMA =
    false;
static bool tftpPendingRestoreQdc =
    false;
extern bool threadsRunning;


/* Private function prototypes -----------------------------------------------*/

static void IAP_wrq_recv_callback(void *_args, struct udp_pcb *upcb, struct pbuf *pkt_buf,
                        const ip_addr_t *addr, u16_t port);

static int IAP_tftp_process_write(struct udp_pcb *upcb, const ip_addr_t *to, int to_port);

static void IAP_tftp_recv_callback(void *arg, struct udp_pcb *Upcb, struct pbuf *pkt_buf,
                        const ip_addr_t *addr, u16_t port);

static void IAP_tftp_cleanup_wr(struct udp_pcb *upcb, tftp_connection_args *args);
static void IAP_tftp_timeout(void *arg);
static void IAP_tftp_arm_timeout(
    tftp_connection_args *args);
static void IAP_tftp_restore_execution(
    bool restoreThreads,
    bool restoreDMA,
    bool restoreQdc);
static void IAP_tftp_abort_transfer(
    struct udp_pcb *upcb,
    tftp_connection_args *args,
    struct pbuf *pkt_buf);
static tftp_opcode IAP_tftp_decode_op(char *buf);
static bool IAP_tftp_validate_wrq_payload(
    const char *payload,
    u16_t packetLength);
static u16_t IAP_tftp_extract_block(const char *buf);
static void IAP_tftp_set_opcode(char *buffer, tftp_opcode opcode);
static void IAP_tftp_set_block(char* packet, u16_t block);
static err_t IAP_tftp_send_ack_packet(struct udp_pcb *upcb, const ip_addr_t *to, int to_port, int block);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief Returns the TFTP opcode
  * @param buf: pointer on the TFTP packet
  * @retval None
  */
static tftp_opcode IAP_tftp_decode_op(char *buf)
{
  return (tftp_opcode)(buf[1]);
}


static bool IAP_tftp_validate_wrq_payload(
    const char *payload,
    u16_t packetLength)
{
  if ((payload == nullptr) ||
      (packetLength < TFTP_OPCODE_LEN))
  {
    return false;
  }

  const uint8_t *bytes =
      reinterpret_cast<const uint8_t *>(
          payload);

  if ((bytes[0] != 0x00U) ||
      (bytes[1] != static_cast<uint8_t>(
                       TFTP_WRQ)))
  {
    return false;
  }

  u16_t index =
      TFTP_OPCODE_LEN;

  if ((index >= packetLength) ||
      (bytes[index] == 0x00U))
  {
    return false;
  }

  while ((index < packetLength) &&
         (bytes[index] != 0x00U))
  {
    index++;
  }

  if (index >= packetLength)
  {
    return false;
  }

  index++;

  const char expectedMode[] =
      {'o', 'c', 't', 'e', 't'};

  for (u16_t modeIndex = 0;
       modeIndex < sizeof(expectedMode);
       modeIndex++)
  {
    if (index >= packetLength)
    {
      return false;
    }

    uint8_t modeByte =
        bytes[index];

    if ((modeByte >= static_cast<uint8_t>('A')) &&
        (modeByte <= static_cast<uint8_t>('Z')))
    {
      modeByte =
          static_cast<uint8_t>(
              modeByte +
              (static_cast<uint8_t>('a') -
               static_cast<uint8_t>('A')));
    }

    if (modeByte != static_cast<uint8_t>(
                        expectedMode[modeIndex]))
    {
      return false;
    }

    index++;
  }

  if ((index >= packetLength) ||
      (bytes[index] != 0x00U))
  {
    return false;
  }

  index++;

  while (index < packetLength)
  {
    if (bytes[index] == 0x00U)
    {
      return false;
    }

    while ((index < packetLength) &&
           (bytes[index] != 0x00U))
    {
      index++;
    }

    if (index >= packetLength)
    {
      return false;
    }

    index++;

    if ((index >= packetLength) ||
        (bytes[index] == 0x00U))
    {
      return false;
    }

    while ((index < packetLength) &&
           (bytes[index] != 0x00U))
    {
      index++;
    }

    if (index >= packetLength)
    {
      return false;
    }

    index++;
  }

  return true;
}

/**
  * @brief  Extracts the block number
  * @param  buf: pointer on the TFTP packet
  * @retval block number
  */
static u16_t IAP_tftp_extract_block(
    const char *buf)
{
  const uint8_t *bytes =
      reinterpret_cast<const uint8_t *>(
          buf);

  return static_cast<u16_t>(
      (static_cast<u16_t>(
           bytes[2]) << 8U) |
      static_cast<u16_t>(
          bytes[3]));
}

/**
  * @brief Sets the TFTP opcode
  * @param  buffer: pointer on the TFTP packet
  * @param  opcode: TFTP opcode
  * @retval None
  */
static void IAP_tftp_set_opcode(char *buffer, tftp_opcode opcode)
{
  buffer[0] = 0;
  buffer[1] = (u8_t)opcode;
}

/**
  * @brief Sets the TFTP block number
  * @param packet: pointer on the TFTP packet
  * @param  block: block number
  * @retval None
  */
static void IAP_tftp_set_block(
    char *packet,
    u16_t block)
{
  uint8_t *bytes =
      reinterpret_cast<uint8_t *>(
          packet);

  bytes[2] =
      static_cast<uint8_t>(
          block >> 8U);

  bytes[3] =
      static_cast<uint8_t>(
          block);
}

/**
  * @brief Sends TFTP ACK packet
  * @param upcb: pointer on udp_pcb structure
  * @param to: pointer on the receive IP address structure
  * @param to_port: receive port number
  * @param block: block number
  * @retval: err_t: error code
  */
static err_t IAP_tftp_send_ack_packet(struct udp_pcb *upcb, const ip_addr_t *to, int to_port, int block)
{
  err_t err;
  struct pbuf *pkt_buf; /* Chain of pbuf's to be sent */

  if ((upcb == nullptr) ||
      (to == nullptr))
  {
    return ERR_ARG;
  }

  /* create the maximum possible size packet that a TFTP ACK packet can be */
  char packet[TFTP_ACK_PKT_LEN];

	memset(packet, 0, TFTP_ACK_PKT_LEN *sizeof(char));

  /* define the first two bytes of the packet */
  IAP_tftp_set_opcode(packet, TFTP_ACK);

  /* Specify the block number being ACK'd.
   * If we are ACK'ing a DATA pkt then the block number echoes that of the DATA pkt being ACK'd (duh)
   * If we are ACK'ing a WRQ pkt then the block number is always 0
   * RRQ packets are never sent ACK pkts by the server, instead the server sends DATA pkts to the
   * host which are, obviously, used as the "acknowledgement".  This saves from having to sEndTransferboth
   * an ACK packet and a DATA packet for RRQs - see RFC1350 for more info.  */
  IAP_tftp_set_block(packet, block);

  /* PBUF_TRANSPORT - specifies the transport layer */
  pkt_buf = pbuf_alloc(PBUF_TRANSPORT, TFTP_ACK_PKT_LEN, PBUF_RAM);

  if (!pkt_buf)      /*if the packet pbuf == NULL exit and EndTransfertransmission */
  {
    return ERR_MEM;
  }

  if (pkt_buf->tot_len < TFTP_ACK_PKT_LEN)
  {
    pbuf_free(pkt_buf);
    return ERR_MEM;
  }

  /* Copy the original data buffer over to the packet buffer's payload */
  err = pbuf_take(pkt_buf, packet, TFTP_ACK_PKT_LEN);
  if (err != ERR_OK)
  {
    pbuf_free(pkt_buf);
    return err;
  }

  /* Sending packet by UDP protocol */
  err = udp_sendto(upcb, pkt_buf, to, to_port);

  /* free the buffer pbuf */
  pbuf_free(pkt_buf);

  return err;
}


static void IAP_tftp_arm_timeout(
    tftp_connection_args *args)
{
    if (args == nullptr)
    {
        return;
    }

    sys_untimeout(
        IAP_tftp_timeout,
        args);

    sys_timeout(
        TFTP_TIMEOUT_INTERVAL * 1000U,
        IAP_tftp_timeout,
        args);
}


static void IAP_tftp_timeout(void *arg)
{
    if (arg == nullptr)
    {
        return;
    }

    tftp_connection_args *args =
        static_cast<tftp_connection_args *>(arg);

    struct udp_pcb *upcb =
        args->upcb;

    const bool restoreThreads =
        args->restoreThreads;

    const bool restoreDMA =
        args->restoreDMA;

    const bool restoreQdc =
        args->restoreQdc;

    PRINTF(
        "TFTP configuration upload timed out !\r\n");

    IAP_tftp_cleanup_wr(
        upcb,
        args);

    IAP_tftp_restore_execution(
        restoreThreads,
        restoreDMA,
        restoreQdc);
}


static void IAP_tftp_abort_transfer(
    struct udp_pcb *upcb,
    tftp_connection_args *args,
    struct pbuf *pkt_buf)
{
    const bool restoreThreads =
        args->restoreThreads;

    const bool restoreDMA =
        args->restoreDMA;

    const bool restoreQdc =
        args->restoreQdc;

    IAP_tftp_cleanup_wr(upcb, args);
    pbuf_free(pkt_buf);

    IAP_tftp_restore_execution(
        restoreThreads,
        restoreDMA,
        restoreQdc);
}

/**
  * @brief  Processes data transfers after a TFTP write request
  * @param  _args: used as pointer on TFTP connection args
  * @param  upcb: pointer on udp_pcb structure
  * @param pkt_buf: pointer on a pbuf stucture
  * @param ip_addr: pointer on the receive IP_address structure
  * @param port: receive port address
  * @retval None
  */
static void IAP_wrq_recv_callback(void *_args, struct udp_pcb *upcb, struct pbuf *pkt_buf, const ip_addr_t *addr, u16_t port)
{
  tftp_connection_args *args = (tftp_connection_args *)_args;
  bool expectedBlockAccepted = false;

  if (pkt_buf == nullptr)
  {
    return;
  }

  if (args == nullptr)
  {
    pbuf_free(pkt_buf);
    return;
  }

  if ((addr == nullptr) ||
      (port != static_cast<u16_t>(
                   args->to_port)) ||
      !ip_addr_cmp(
          addr,
          &args->to_ip))
  {
    pbuf_free(
        pkt_buf);
    return;
  }

  uint8_t data_buffer[512];
  status_t status;

  memset(data_buffer, 0x0, sizeof(data_buffer));

  if ((pkt_buf->len != pkt_buf->tot_len) ||
      (pkt_buf->len < TFTP_DATA_PKT_HDR_LEN) ||
      (pkt_buf->len > TFTP_DATA_PKT_LEN_MAX))
  {
    pbuf_free(pkt_buf);
    return;
  }

  if (IAP_tftp_decode_op(
        static_cast<char *>(pkt_buf->payload)) != TFTP_DATA)
  {
    pbuf_free(pkt_buf);
    return;
  }

  const u16_t receivedBlock =
      IAP_tftp_extract_block(
          static_cast<const char *>(
              pkt_buf->payload));

  /* Does this packet have any valid data to write? */
  if ((pkt_buf->len > TFTP_DATA_PKT_HDR_LEN) &&
      (receivedBlock == (args->block + 1)))
  {
    const uint32_t payloadSize =
        pkt_buf->len - TFTP_DATA_PKT_HDR_LEN;

    const uint32_t maxUploadSize =
        METADATA_LEN + JSON_BUFF_SIZE;

    if ((static_cast<uint32_t>(args->tot_bytes) +
         payloadSize) > maxUploadSize)
    {
      PRINTF(
          "Configuration upload exceeds maximum size !\r\n");

      IAP_tftp_abort_transfer(upcb, args, pkt_buf);
      return;
    }

    /* copy packet payload to data_buffer */
    pbuf_copy_partial(pkt_buf, data_buffer, payloadSize,
                      TFTP_DATA_PKT_HDR_LEN);

    /* Write received data in Flash */

    // A UDP packet is 512 bytes = 2 x 256 byte pages
    // First page
	status = flexspi_nor_flash_page_program(FLEXSPI, Flash_Write_Address + args->block * 512, (uint32_t *)(data_buffer));
	if (status != kStatus_Success)
	{
	 PRINTF("Page program failure !\r\n");
	 IAP_tftp_abort_transfer(upcb, args, pkt_buf);
	 return;
	}

	// Second page
	status = flexspi_nor_flash_page_program(FLEXSPI, Flash_Write_Address + args->block * 512 + FLASH_PAGE_SIZE, (uint32_t *)(data_buffer + FLASH_PAGE_SIZE));
	if (status != kStatus_Success)
	{
	  PRINTF("Page program failure !\r\n");
	  IAP_tftp_abort_transfer(upcb, args, pkt_buf);
	  return;
	}

    total_count += payloadSize;

    /* update our block number to match the block number just received */
    args->block++;
    /* update total bytes  */
    (args->tot_bytes) += payloadSize;
    expectedBlockAccepted = true;

    /* This is a valid pkt but it has no data.  This would occur if the file being
       written is an exact multiple of 512 bytes.  In this case, the args->block
       value must still be updated, but we can skip everything else.    */
  }
  else if (receivedBlock == (args->block + 1))
  {
    /* update our block number to match the block number just received  */
    args->block++;
    expectedBlockAccepted = true;
  }

  /* Send the appropriate ACK pkt*/
  const err_t ackStatus =
      IAP_tftp_send_ack_packet(
          upcb,
          addr,
          port,
          args->block);

  if (ackStatus != ERR_OK)
  {
    PRINTF("TFTP DATA ACK failure !\r\n");
    expectedBlockAccepted = false;

    IAP_tftp_abort_transfer(
        upcb,
        args,
        pkt_buf);

    return;
  }

  /* If the last write returned less than the maximum TFTP data pkt length,
   * then we've received the whole file and so we can quit (this is how TFTP
   * signals the EndTransferof a transfer!)
   */
  if (expectedBlockAccepted &&
      (pkt_buf->len < TFTP_DATA_PKT_LEN_MAX))
  {
    tftpPendingRestoreThreads =
        args->restoreThreads;
    tftpPendingRestoreDMA =
        args->restoreDMA;
    tftpPendingRestoreQdc =
        args->restoreQdc;
    tftpFinalizationPending =
        true;

    IAP_tftp_cleanup_wr(upcb, args);
    pbuf_free(pkt_buf);
    newJson = true;
  }
  else
  {
    IAP_tftp_arm_timeout(
        args);

    pbuf_free(pkt_buf);
    return;
  }
}


static void IAP_tftp_restore_execution(
    bool restoreThreads,
    bool restoreDMA,
    bool restoreQdc)
{
    printf(
        "\nRestoring execution after "
        "TFTP setup failure.\n");

    if (restoreDMA)
    {
        dmaThread->DMAptr->configDMA();

        // Apply the still-zero command state before DMA starts.
        dmaThread->run();

        dmaThread->startThread();
        DMAthreadRunning = true;
    }

    if (restoreThreads)
    {
        if (hasBaseThread)
        {
            baseThread->startThread();
        }

        if (hasServoThread)
        {
            servoThread->startThread();
        }

        threadsRunning = true;
    }

    if (restoreQdc)
    {
        for (uint8_t i = 0;
             i < MAX_INST_QDC_MOD;
             i++)
        {
            if (qdc[i] != nullptr)
            {
                qdc[i]->enableInterrupt();
            }
        }
    }
}


/**
  * @brief  Processes TFTP write request
  * @param  to: pointer on the receive IP address
  * @param  to_port: receive port number
  * @retval None
  */
static int IAP_tftp_process_write(struct udp_pcb *upcb, const ip_addr_t *to, int to_port)
{
  tftp_connection_args *args = NULL;
  /* This function is called from a callback,
  * therefore interrupts are disabled,
  * therefore we can use regular malloc   */
  args = (tftp_connection_args*)mem_malloc(sizeof *args);
  if (!args)
  {
    IAP_tftp_cleanup_wr(upcb, args);
    return 0;
  }

  args->op = TFTP_WRQ;
  args->to_ip.addr = to->addr;
  args->to_port = to_port;
  /* the block # used as a positive response to a WRQ is _always_ 0!!! (see RFC1350)  */
  args->block = 0;
  args->tot_bytes = 0;
  args->upcb = upcb;

  const bool restoreThreads =
      threadsRunning;

  const bool restoreDMA =
      hasDMAthread && DMAthreadRunning;

  const bool restoreQdc =
      hasQDC;

  args->restoreThreads = restoreThreads;
  args->restoreDMA = restoreDMA;
  args->restoreQdc = restoreQdc;

  /* set callback for receives on this UDP PCB (Protocol Control Block) */
  udp_recv(upcb, IAP_wrq_recv_callback, args);

  total_count =0;

  // Get ready to upload configuration

  printf(
      "\nReceiving new configuration. "
      "Clearing outputs and stopping threads..\n");

  {
      int remaining = sizeof(rxData.rxBuffer);

      while (remaining-- > 0)
      {
          rxData.rxBuffer[remaining] = 0;
      }
  }

  if (restoreThreads)
  {
      if (hasBaseThread)
      {
          baseThread->stopThread();
      }

      if (hasServoThread)
      {
          servoThread->stopThread();
      }

      if (hasBaseThread)
      {
          baseThread->run();
      }

      if (hasServoThread)
      {
          servoThread->run();
      }

      threadsRunning = false;
  }

  if (restoreDMA)
  {
	  dmaThread->DMAptr->clearCompletionState();

	  EDMA_StopTransfer(&edma_handle);
	  EDMA_ResetChannel(edma_handle.base, edma_handle.channel);
	  EDMA_Deinit(DMA0);

	  DMAMUX_DisableChannel(DMAMUX, 0);
	  DMAMUX_Deinit(DMAMUX);

	  DMAthreadRunning = false;

	  // Apply the disabled state once after DMA has stopped.
	  dmaThread->run();
  }

  if (restoreQdc)
  {
	  printf("\nDisabling Index Irqs Gpio Interrupts.\n");
	  for(uint8_t i=0; i<MAX_INST_QDC_MOD;i++)
	  {
		  if(qdc[i]!=nullptr)
			  qdc[i]->disableInterrupt();
	  }
  }


  /* init flash */
  flexspi_nor_flash_init(FLEXSPI);

  /* Enter quad mode. */
  status_t status = flexspi_nor_enable_quad_mode(FLEXSPI);
  if (status != kStatus_Success)
  {
      PRINTF("Enable quad mode failure !\r\n");
      IAP_tftp_cleanup_wr(upcb, args);
      IAP_tftp_restore_execution(
          restoreThreads,
          restoreDMA,
          restoreQdc);
      return status;
  }

  Flash_Write_Address = JSON_UPLOAD_ADDRESS;

  /* erase user flash area for metadata and JSON payload */
  ////FLASH_If_Erase(JSON_UPLOAD_ADDRESS);
  const uint32_t uploadSize =
      JSON_BUFF_SIZE + METADATA_LEN;

  const uint32_t uploadSectors =
      (uploadSize + SECTOR_SIZE - 1U) / SECTOR_SIZE;

  for (uint32_t sector = 0;
       sector < uploadSectors;
       sector++)
  {
      status = flexspi_nor_flash_erase_sector(
          FLEXSPI,
          Flash_Write_Address + sector * SECTOR_SIZE);

      if (status != kStatus_Success)
      {
          PRINTF("Erase sector failure !\r\n");
          IAP_tftp_cleanup_wr(upcb, args);
          IAP_tftp_restore_execution(
              restoreThreads,
              restoreDMA,
              restoreQdc);
          return -1;
      }
  }

  /* initiate the write transaction by sending the first ack */
  const err_t ackStatus =
      IAP_tftp_send_ack_packet(
          upcb,
          to,
          to_port,
          args->block);

  if (ackStatus != ERR_OK)
  {
      PRINTF("Initial TFTP ACK failure !\r\n");
      IAP_tftp_cleanup_wr(upcb, args);
      IAP_tftp_restore_execution(
          restoreThreads,
          restoreDMA,
          restoreQdc);
      return ackStatus;
  }

  IAP_tftp_arm_timeout(
      args);

  return 0;
}


/**
  * @brief  Processes traffic received on UDP port 69
  * @param  args: pointer on tftp_connection arguments
  * @param  upcb: pointer on udp_pcb structure
  * @param  pbuf: pointer on packet buffer
  * @param  addr: pointer on the receive IP address
  * @param  port: receive port number
  * @retval None
  */
static void IAP_tftp_recv_callback(void *arg, struct udp_pcb *upcb, struct pbuf *pkt_buf,
                        const ip_addr_t *addr, u16_t port)
{
  struct udp_pcb *upcb_tftp_data;
  err_t err;

  if (pkt_buf == nullptr)
  {
    return;
  }

  if ((addr == nullptr) ||
      (pkt_buf->payload == nullptr) ||
      (pkt_buf->len != pkt_buf->tot_len) ||
      (pkt_buf->len < TFTP_OPCODE_LEN))
  {
    pbuf_free(pkt_buf);
    return;
  }

  if (!IAP_tftp_validate_wrq_payload(
          static_cast<const char *>(
              pkt_buf->payload),
          pkt_buf->len))
  {
    pbuf_free(pkt_buf);
    return;
  }

  if (tftpTransferActive ||
      tftpFinalizationPending)
  {
    PRINTF(
        "TFTP configuration upload already active !\r\n");

    pbuf_free(
        pkt_buf);

    return;
  }

  /* create new UDP PCB structure */
  upcb_tftp_data = udp_new();
  if (!upcb_tftp_data)
  {
    /* Error creating PCB. Out of Memory  */
    pbuf_free(pkt_buf);
    return;
  }

  /* bind to port 0 to receive next available free port */
  /* NOTE:  This is how TFTP works.  There is a UDP PCB for the standard port
  * 69 which al transactions begin communication on, however, _all_ subsequent
  * transactions for a given "stream" occur on another port  */
  err = udp_bind(upcb_tftp_data, IP_ADDR_ANY, 0);
  if (err != ERR_OK)
  {
    /* Unable to bind to port */
    udp_remove(upcb_tftp_data);
    pbuf_free(pkt_buf);
    return;
  }

  tftpTransferActive =
      true;

  /* Start the TFTP write mode*/
  IAP_tftp_process_write(upcb_tftp_data, addr, port);
  pbuf_free(pkt_buf);
}


/**
  * @brief  disconnect and close the connection
  * @param  upcb: pointer on udp_pcb structure
  * @param  args: pointer on tftp_connection arguments
  * @retval None
  */
static void IAP_tftp_cleanup_wr(struct udp_pcb *upcb, tftp_connection_args *args)
{
  if (args != nullptr)
  {
    sys_untimeout(
        IAP_tftp_timeout,
        args);

    /* Free the tftp_connection_args structure */
    mem_free(args);
  }

  if ((upcb != nullptr) &&
      (upcb != UDPpcb))
  {
    /* Disconnect the udp_pcb */
    udp_disconnect(upcb);

    /* close the connection */
    udp_remove(upcb);
  }

  tftpTransferActive =
      false;

  if (UDPpcb != nullptr)
  {
    /* reset the callback function */
    udp_recv(UDPpcb, IAP_tftp_recv_callback, NULL);
  }

}

/* Global functions ---------------------------------------------------------*/

void IAP_tftp_finalize_upload(
    bool uploadSucceeded)
{
  if (!tftpFinalizationPending)
  {
    return;
  }

  const bool restoreThreads =
      tftpPendingRestoreThreads;
  const bool restoreDMA =
      tftpPendingRestoreDMA;
  const bool restoreQdc =
      tftpPendingRestoreQdc;

  tftpFinalizationPending =
      false;
  tftpPendingRestoreThreads =
      false;
  tftpPendingRestoreDMA =
      false;
  tftpPendingRestoreQdc =
      false;

  if (!uploadSucceeded)
  {
    IAP_tftp_restore_execution(
        restoreThreads,
        restoreDMA,
        restoreQdc);
  }
}

/**
  * @brief  Creates and initializes a UDP PCB for TFTP receive operation
  * @param  None
  * @retval None
  */
void IAP_tftpd_init(edma_handle_t handle)
{
  edma_handle = handle;
  unsigned port = 69; /* 69 is the port used for TFTP protocol initial transaction */

  /* create a new UDP PCB structure  */
  UDPpcb = udp_new();
  if (!UDPpcb)
  {
    /* Error creating PCB. Out of Memory  */
    PRINTF(
        "Failed to allocate TFTP UDP PCB !\r\n");
    return;
  }

  /* Bind this PCB to port 69  */
  const err_t bindStatus =
      udp_bind(
          UDPpcb,
          IP_ADDR_ANY,
          port);

  if (bindStatus != ERR_OK)
  {
    PRINTF(
        "Failed to bind TFTP UDP PCB to port 69 !\r\n");

    udp_remove(
        UDPpcb);

    UDPpcb =
        nullptr;

    return;
  }

  /* Initialize receive callback function  */
  udp_recv(
      UDPpcb,
      IAP_tftp_recv_callback,
      nullptr);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
