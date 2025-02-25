<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    stlink_mcptl.h.ftl
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file is part of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#ifndef stlnk_protocol_h
#define stlnk_protocol_h
#include "mcptl.h"

#define STLNK_ERROR_PACKET_SIZE     4
#define STLNK_CRC_SIZE              2U

#define STLNK_OK 0U
#define STLNK_SYNC_NOT_EXPECTED     1
#define STLNK_BUFFER_ERROR          2
#define STLNK_ASYNC_NOT_EXPECTED    3

#define STLNK_SYNC_PACKET_TYPE_SIZE 1   /* Sync Packet Types are STLNK_ERROR_PACKET or STLNK_RESPONSE_PACKET */
#define STLNK_ERROR_PACKET          0xF
#define STLNK_RESPONSE_PACKET       0xA
/* STLNK Protocol errors */
#define STLNK_BAD_CRC_DATA          1
#define STLNK_BAD_PACKET_SIZE       2

#define STLNK_ASYNC_BUFFER          0xA /* To be replaced by MCTL_ASYNC once updated in MC Pilot */
#define STLNK_SYNC_BUFFER           0x6 /* To be replaced by MCTL_SYNC  once updated in MC Pilot */
#define STLNK_NO_BUFFER             0x0

typedef struct
{
        uint8_t bufferType;            /* Allow Stlink layer to descrimine sync or Async buffers */
        uint8_t bufferID;              /* Used to select asyncBuffer A or B */
        uint16_t bufferSize;
        uint8_t * rxBuffer;
        uint8_t * syncBuffer;
        uint8_t * asyncBufferA;
        uint8_t * asyncBufferB;
        uint16_t maxSyncPayloadSize;
        uint16_t maxAsyncPayloadSize;
} STLNK_Control_t;

typedef struct 
{
  MCTL_Handle_t _Super;                /* 17 bytes */
  void *lockBuffer;
  uint8_t *rxBuffer;
  MCTL_Buff_t syncBuffer;
<#if MC.MCP_ASYNC_OVER_STLNK_EN>
  MCTL_Buff_t *asyncNextBuffer;
  MCTL_Buff_t asyncBufferA;
  MCTL_Buff_t asyncBufferB;
</#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
} STLNK_Handle_t;

bool STLNK_getBuffer(MCTL_Handle_t *pSupHandle, void **buffer,  uint8_t syncAsync);
uint8_t STLNK_sendPacket(MCTL_Handle_t *pSupHandle, void *txBuffer, uint16_t txDataLength, uint8_t syncAsync);
uint8_t* STLNK_RXframeProcess(MCTL_Handle_t *pSupHandle, uint16_t *packetLength);
void STLNK_HWDataTransmittedIT(STLNK_Handle_t *pHandle );
void STLNK_init(STLNK_Handle_t *pHandle);

#endif /* stlnk_protocol_h */
/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
