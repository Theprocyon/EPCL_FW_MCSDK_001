<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    stlink_mcptl.c.ftl
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

#include "mc_stm_types.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "stlink_mcptl.h"

static uint8_t STLNK_TXframeProcess (STLNK_Handle_t *pHandle, uint8_t dataType, void *txBuffer, uint16_t bufferLength);
#ifdef NOT_IMPLEMENTED /* Not yet implemented */
static void STLNK_sendError(STLNK_Handle_t *pHandle, uint8_t errorMessage);
#endif
static inline void disable_hftask_irq();
static inline void enable_hftask_irq();

/* TODO: Disable High frequency task is enough */
static inline void disable_hftask_irq(void)
{
  __disable_irq();
}

/* TODO: Enable High frequency task is enough */
static inline void enable_hftask_irq(void)
{
  __enable_irq();
}

void STLNK_init(STLNK_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STL_MNG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
<#if MC.MCP_ASYNC_OVER_STLNK_EN>
    uint32_t *ptrToBuff;
    /* The first 4 bytes of the buffer array is used to store the address of the MCTL_Buff structure */
    /* This allow us to find back the structure from the address of the buffer */
    ptrToBuff = (uint32_t *)pHandle->asyncBufferA.buffer;  //cstat !MISRAC2012-Rule-11.3
    *ptrToBuff = (uint32_t)&pHandle->asyncBufferA; //cstat !MISRAC2012-Rule-11.4
    ptrToBuff = (uint32_t *)pHandle->asyncBufferB.buffer; //cstat !MISRAC2012-Rule-11.3
    *ptrToBuff = (uint32_t)&pHandle->asyncBufferB; //cstat !MISRAC2012-Rule-11.4
    /* Allow buffer identification when buffers has to be send by MCP layer */
    pHandle->asyncBufferA.buffer[((uint16_t)STLNK_TX_ASYNCBUFFER_SIZE) - 1U] = 0xAU;
    pHandle->asyncBufferB.buffer[((uint16_t)STLNK_TX_ASYNCBUFFER_SIZE) - 1U] = 0xBU;
    stlnkCtrl.maxAsyncPayloadSize = STLNK_TX_ASYNC_PAYLOAD_MAX; 
    pHandle->_Super.txAsyncMaxPayload = STLNK_TX_ASYNC_PAYLOAD_MAX;
</#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
    stlnkCtrl.maxSyncPayloadSize = MCP_TX_SYNC_PAYLOAD_MAX; 
    pHandle->_Super.txSyncMaxPayload = MCP_TX_SYNC_PAYLOAD_MAX;
#ifdef NULL_PTR_CHECK_STL_MNG
  }
#endif
}

bool STLNK_getBuffer(MCTL_Handle_t *pSupHandle, void **buffer, uint8_t syncAsync)
{
  bool result = true;
#ifdef NULL_PTR_CHECK_STL_MNG  
  if (MC_NULL == buffer)
  {
    result = false;
  }
  else
  {
#endif
    STLNK_Handle_t *pHandle = (STLNK_Handle_t *)pSupHandle; //cstat !MISRAC2012-Rule-11.3
    
    if (MCTL_SYNC == syncAsync)
    {
      if (pHandle->syncBuffer.state <= writeLock) /* Possible values are free or writeLock */
      {
        *buffer = pHandle->syncBuffer.buffer;
        pHandle->syncBuffer.state = writeLock;
      }
      else
      {
        result = false;
      }  
    }
    else /* Asynchronous buffer request */
    {
<#if MC.MCP_ASYNC_OVER_STLNK_EN>
      if ((pHandle->asyncBufferA.state > writeLock) && (pHandle->asyncBufferB.state > writeLock))
      {
        result = false;
      }
      else 
      {
        if (pHandle->asyncBufferA.state <= writeLock)
        {
          pHandle->asyncBufferA.state = writeLock;
          *buffer = &pHandle->asyncBufferA.buffer[4]; /* The 4 first byte are used to store asyncBufferA */
#ifdef MCP_DEBUG_METRICS
          pHandle->asyncBufferA.RequestedNumber++;
#endif
        } 
        else if (pHandle->asyncBufferB.state <= writeLock)
        {
          pHandle->asyncBufferB.state = writeLock;
          *buffer = &pHandle->asyncBufferB.buffer[4];
#ifdef MCP_DEBUG_METRICS
          pHandle->asyncBufferB.RequestedNumber++;
#endif    
        }
        else
        {
          /* Nothing to do */
        }
      }
<#else><#-- MC.MCP_ASYNC_OVER_STLNK_EN == false -->
      result = false; /* Asynchronous buffer not supported */
</#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
    }
#ifdef NULL_PTR_CHECK_STL_MNG
  }
#endif
  return (result);
}

uint8_t STLNK_sendPacket(MCTL_Handle_t *pSupHandle, void *txBuffer, uint16_t txDataLength, uint8_t syncAsync)
{
  uint8_t result = STLNK_SYNC_NOT_EXPECTED;
#ifdef NULL_PTR_CHECK_STL_MNG  
  if (MC_NULL == pSupHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint8_t *trailerPacket = (uint8_t *)txBuffer; //cstat !MISRAC2012-Rule-11.5
    uint16_t totalLength = 0U;

    STLNK_Handle_t *pHandle = (STLNK_Handle_t *)pSupHandle; //cstat !MISRAC2012-Rule-11.3
      
    if (MCTL_SYNC == syncAsync)
    {
      if (pSupHandle->MCP_PacketAvailable)
      {
        /* We need to send an acknowledge packet with answer */
        pSupHandle-> MCP_PacketAvailable = false; /* CMD from master is processed */

        /* Add CRC (to be implemented) */
        trailerPacket[txDataLength] = ((uint8_t)0xCA); /* Dummy CRC */
        trailerPacket[txDataLength + 1U] = ((uint8_t)0xFE); /* Dummy CRC */
        trailerPacket[txDataLength + 2U] = ((uint8_t)STLNK_RESPONSE_PACKET);
        totalLength = txDataLength + STLNK_CRC_SIZE + 1U; /* 1 additional byte for packet type */
        result = STLNK_OK;
      }
      else 
      {
        result = STLNK_SYNC_NOT_EXPECTED;
      }
    }
    else /* Packet is an ASync packet */
    {
<#if MC.MCP_ASYNC_OVER_STLNK_EN>
      /* In case of ASync packet, Only CRC has to be added */
      trailerPacket[txDataLength] = ((uint8_t)0xCA); /* Dummy CRC */
      trailerPacket[txDataLength + 1U] = ((uint8_t)0xFE); /* Dummy CRC */
      totalLength = txDataLength + STLNK_CRC_SIZE;
      result = STLNK_OK;
<#else><#-- MC.MCP_ASYNC_OVER_STLNK_EN == false -->
      result = STLNK_ASYNC_NOT_EXPECTED;
</#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
    }
    if (STLNK_OK == result)
    {
      result = STLNK_TXframeProcess(pHandle, syncAsync, txBuffer, totalLength);
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_CHECK_STL_MNG
  }
#endif
  return (result);
}

#ifdef NOT_IMPLEMENTED /* Not yet implemented */
void STLNK_sendError(STLNK_Handle_t *pHandle, uint8_t errorMessage)
{
  uint32_t *packet = (uint32_t*)pHandle->syncBuffer.buffer; //cstat !MISRAC2012-Rule-11.3
  *packet = (((uint32_t)STLNK_ERROR_PACKET) << 28U) | (uint32_t)errorMessage;
  (void)STLNK_TXframeProcess(pHandle, MCTL_SYNC, &pHandle->syncBuffer.buffer, STLNK_ERROR_PACKET_SIZE);
}
#endif

uint8_t STLNK_TXframeProcess(STLNK_Handle_t *pHandle, uint8_t syncAsync, void *txBuffer, uint16_t bufferLength)
{
  uint8_t result = STLNK_OK;
  
<#if MC.MCP_ASYNC_OVER_STLNK_EN>
  MCTL_Buff_t *txAsyncBuffer;
  uint32_t txAsyncBufferAddr;
  uint8_t *txBuffer8 = (uint8_t *)txBuffer;  //cstat !MISRAC2012-Rule-11.5
</#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->

  disable_hftask_irq();
  if (NULL == pHandle->lockBuffer) /* Communication Ip free to send data */
  {
    if (MCTL_SYNC == syncAsync)
    {
      pHandle->syncBuffer.state = readLock;
      pHandle->lockBuffer = (void *)&pHandle->syncBuffer;
      stlnkCtrl.bufferID =  0; /* Useless here. Only one Sync buffer exists */
      stlnkCtrl.bufferSize = bufferLength;
      stlnkCtrl.bufferType = STLNK_SYNC_BUFFER;
    }
<#if MC.MCP_ASYNC_OVER_STLNK_EN>
    else if (MCTL_ASYNC == syncAsync)
    {
      /* In case of ASYNC, two flipflop buffers are used, the txBuffer points always to 
       * lastRequestedAsyncBuff->buffer */
      txAsyncBufferAddr = *(uint32_t *)(txBuffer8 - 4U); //cstat !MISRAC2012-Rule-11.3 !MISRAC2012-Rule-18.4
      txAsyncBuffer = (MCTL_Buff_t *)txAsyncBufferAddr; //cstat !MISRAC2012-Rule-11.4
      txAsyncBuffer->state = readLock;
      pHandle->lockBuffer = (void *)txAsyncBuffer;
#ifdef MCP_DEBUG_METRICS
      txAsyncBuffer->SentNumber++;
#endif
      stlnkCtrl.bufferID =  txAsyncBuffer->buffer[STLNK_TX_ASYNCBUFFER_SIZE - 1U];
      stlnkCtrl.bufferSize = bufferLength;
      stlnkCtrl.bufferType = STLNK_ASYNC_BUFFER;
    }
</#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
    else 
    {
      /* No other packets type supported */
    }
    /* Enable HF task It */
    enable_hftask_irq();
    /* */
  }
  else /* HW resource busy, saving packet to sent it once resource will be freed */
  { 
    enable_hftask_irq();
  
    if (MCTL_SYNC == syncAsync) 
    {
      if (pHandle->syncBuffer.state != writeLock)
      {
        result = STLNK_BUFFER_ERROR;
      }
      else
      {
        pHandle ->syncBuffer.state = pending;
        pHandle ->syncBuffer.length = bufferLength;
      }
    }
<#if MC.MCP_ASYNC_OVER_STLNK_EN>
    else if (MCTL_ASYNC == syncAsync)
    {
    
      txAsyncBufferAddr = *(uint32_t *)(txBuffer8 - 4U); //cstat !MISRAC2012-Rule-11.3 !MISRAC2012-Rule-18.4
      txAsyncBuffer = (MCTL_Buff_t *)txAsyncBufferAddr; //cstat !MISRAC2012-Rule-11.4
      if (NULL == pHandle->asyncNextBuffer)
      { /* Required to keep the right sending order */
        pHandle->asyncNextBuffer = txAsyncBuffer;
      }
    else
    {
      /* Nothing to do */
    }
      txAsyncBuffer->state = pending;
      txAsyncBuffer->length = bufferLength;
#ifdef MCP_DEBUG_METRICS
      txAsyncBuffer->PendingNumber++;
#endif
    }
</#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
    else  
    {
      /* No other packet types supported */
    }
  }
return (result);
}

void STLNK_HWDataTransmittedIT(STLNK_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STL_MNG
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* First, free the buffer just sent */
      MCTL_Buff_t *tempBuff = (MCTL_Buff_t *)pHandle -> lockBuffer; //cstat !MISRAC2012-Rule-11.5
      tempBuff->state = available;
    
    /* Second prepare transfer of pending buffer */
    if (pending == pHandle->syncBuffer.state)
    {
      pHandle->lockBuffer = (void *)&pHandle->syncBuffer;
      /* */
      pHandle ->syncBuffer.state = readLock;
      stlnkCtrl.bufferSize = pHandle->syncBuffer.length;
      stlnkCtrl.bufferType = STLNK_SYNC_BUFFER;
      /* */
    }
    else
    { 
<#if MC.MCP_ASYNC_OVER_STLNK_EN>
      __disable_irq();
      if (pHandle->asyncNextBuffer != NULL)
      {
        pHandle->lockBuffer = (void *)pHandle->asyncNextBuffer;
        pHandle->asyncNextBuffer->state = readLock;
        #ifdef MCP_DEBUG_METRICS
        pHandle->asyncNextBuffer->SentNumber++;
        #endif
        /* */
        stlnkCtrl.bufferID =  pHandle->asyncNextBuffer->buffer[STLNK_TX_ASYNCBUFFER_SIZE - 1U];
        stlnkCtrl.bufferSize = pHandle->asyncNextBuffer->length;
        stlnkCtrl.bufferType = STLNK_ASYNC_BUFFER;
        /* */
        /* If one Async buffer is still pending, assign it to the asyncNextBuffer pointer */
        if ((pHandle->asyncBufferA.state == pending) || (pHandle->asyncBufferB.state == pending))
        { 
          uint32_t temp = (uint32_t)&pHandle->asyncBufferA //cstat !MISRAC2012-Rule-11.4
                        + (uint32_t)&pHandle->asyncBufferB //cstat !MISRAC2012-Rule-11.4
                        - (uint32_t)pHandle->asyncNextBuffer; //cstat !MISRAC2012-Rule-11.4
          pHandle->asyncNextBuffer = (MCTL_Buff_t *)temp; //cstat !MISRAC2012-Rule-11.4
        }
        else
        {
          pHandle->asyncNextBuffer = NULL;
        }
      }
      else /* No TX packet are pending, HW resource is free */
      {
        pHandle->lockBuffer = NULL;
        stlnkCtrl.bufferType = STLNK_NO_BUFFER;
      }
      __enable_irq();
<#else><#-- MC.MCP_ASYNC_OVER_STLNK_EN == false -->
        pHandle->lockBuffer = NULL;
        stlnkCtrl.bufferType = STLNK_NO_BUFFER;
</#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
    }
#ifdef NULL_PTR_CHECK_STL_MNG
  }
#endif
}

uint8_t* STLNK_RXframeProcess(MCTL_Handle_t *pSupHandle, uint16_t *packetLength)
{
  uint8_t *result = NULL;
#ifdef NULL_PTR_CHECK_STL_MNG  
  if ((MC_NULL == packetLength) || (MC_NULL == pSupHandle))
  {
    /* Nothing to do */
  }
  else 
  {
#endif
    STLNK_Handle_t *pHandle = (STLNK_Handle_t *)pSupHandle; //cstat !MISRAC2012-Rule-11.3

    /* Length of the packet send by the STLink is stored in the two first bytes */
    *packetLength = *((uint16_t *)pHandle->rxBuffer); //cstat !MISRAC2012-Rule-11.3
    
    if (*packetLength != 0U)
    {
      /* Consumes new packet by clearing the packet size */
      *((uint16_t *)pHandle->rxBuffer) = 0U; //cstat !MISRAC2012-Rule-11.3
      pSupHandle->MCP_PacketAvailable = true; /* Will be consumed in FCP_sendPacket */
      result = pHandle->rxBuffer + 4U; /* Header is 4 bytes 2x16bits size */
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_CHECK_STL_MNG
  }
#endif
  return (result);
}

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
