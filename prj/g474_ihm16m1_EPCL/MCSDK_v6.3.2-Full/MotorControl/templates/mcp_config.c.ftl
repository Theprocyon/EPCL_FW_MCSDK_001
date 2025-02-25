<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    mcp_config.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides configuration information of the MCP protocol
  *
  *
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
  
#include "parameters_conversion.h"
<#if MC.MCP_ASPEP_OVER_UART>
  <#if MCP_DMA>
#include "usart_aspep_driver.h"
  <#else><#-- !MCP_DMA -->
#include "usart_aspep_driver_no_dma.h"
  </#if><#-- MCP_DMA -->
</#if><#-- MC.MCP_ASPEP_OVER_UART -->
<#if MC.MCP_OVER_ASPEP_EN>
#include "aspep.h"
</#if><#-- MC.MCP_OVER_ASPEP_EN -->
<#if MC.MCP_OVER_STLNK_EN>
#include "stlink_mcptl.h"
</#if><#-- MC.MCP_OVER_STLNK_EN -->
#include "mcp.h"
<#if MC.MCP_ASYNC_EN>
#include "mcpa.h"
</#if><#-- MC.MCP_ASYNC_EN -->
#include "mcp_config.h"

static uint8_t MCPSyncTxBuff[MCP_TX_SYNCBUFFER_SIZE] __attribute__((aligned(4))); //cstat !MISRAC2012-Rule-1.4_a
static uint8_t MCPSyncRXBuff[MCP_RX_SYNCBUFFER_SIZE] __attribute__((aligned(4))); //cstat !MISRAC2012-Rule-1.4_a

<#if MC.MCP_ASYNC_OVER_UART_A_EN>
/* Asynchronous buffer dedicated to UART_A */
static uint8_t MCPAsyncBuffUARTA_A[MCP_TX_ASYNCBUFFER_SIZE_A] __attribute__((aligned(4))); //cstat !MISRAC2012-Rule-1.4_a
static uint8_t MCPAsyncBuffUARTA_B[MCP_TX_ASYNCBUFFER_SIZE_A] __attribute__((aligned(4))); //cstat !MISRAC2012-Rule-1.4_a

/* Buffer dedicated to store pointer of data to be streamed over UART_A */
static void *dataPtrTableA[MCPA_OVER_UARTA_STREAM];
static void *dataPtrTableBuffA[MCPA_OVER_UARTA_STREAM];
static uint8_t dataSizeTableA[MCPA_OVER_UARTA_STREAM];
static uint8_t dataSizeTableBuffA[MCPA_OVER_UARTA_STREAM]; /* buffered version of dataSizeTableA */
</#if><#-- MC.MCP_ASYNC_OVER_UART_A_EN -->
<#if MC.MCP_ASYNC_OVER_UART_B_EN>
/* Asynchronous buffer dedicated to UART_B */
static uint8_t MCPAsyncBuffUARTB_A[MCP_TX_ASYNCBUFFER_SIZE_B] __attribute__((aligned(4)));
static uint8_t MCPAsyncBuffUARTB_B[MCP_TX_ASYNCBUFFER_SIZE_B] __attribute__((aligned(4)));

/* Buffer dedicated to store pointer of data to be streamed over UART_B */
void *DataPtrTableB[MCPA_OVER_UARTB_STREAM];
void *DataPtrTableBuffB[MCPA_OVER_UARTB_STREAM];
uint8_t dataSizeTableB[MCPA_OVER_UARTB_STREAM];
uint8_t dataSizeTableBuffB[MCPA_OVER_UARTB_STREAM];
</#if><#-- MC.MCP_ASYNC_OVER_UART_B_EN -->
<#if MC.MCP_ASYNC_OVER_STLNK_EN>
static uint8_t STLnkAsyncBuffA[STLNK_TX_ASYNCBUFFER_SIZE] __attribute__((aligned(4))); 
static uint8_t STLnkAsyncBuffB[STLNK_TX_ASYNCBUFFER_SIZE] __attribute__((aligned(4)));

/* Buffer dedicated to store pointer of data to be streamed over STLnk */
void *DataPtrTableSTlnk[MCPA_OVER_STLNK_STREAM];
void *DataPtrTableBuffSTlnk[MCPA_OVER_STLNK_STREAM];
uint8_t dataSizeTableSTlnk[MCPA_OVER_STLNK_STREAM];
uint8_t dataSizeTableBuffSTlnk[MCPA_OVER_STLNK_STREAM];
</#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->

MCP_user_cb_t MCP_UserCallBack[MCP_USER_CALLBACK_MAX];

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCP
  * @{
  */

<#if MC.MCP_OVER_UART_A_EN>
static UASPEP_Handle_t UASPEP_A =
{
 .USARTx = USARTA,
  <#if MCP_DMA>
 .rxDMA = DMA_RX_A,
 .txDMA = DMA_TX_A,
 .rxChannel = DMACH_RX_A,
 .txChannel = DMACH_TX_A,
  </#if><#-- MCP_DMA -->
};

ASPEP_Handle_t aspepOverUartA =
{
  ._Super = 
   {
    .fGetBuffer = &ASPEP_getBuffer,
    .fSendPacket = &ASPEP_sendPacket,
    .fRXPacketProcess = &ASPEP_RXframeProcess,
    },
  .ASPEPIp = &UASPEP_A,
  .Capabilities =
  {
    .DATA_CRC = 0U,
    .RX_maxSize =  (MCP_RX_SYNC_PAYLOAD_MAX >> 5U) - 1U,
    .TXS_maxSize = (MCP_TX_SYNC_PAYLOAD_MAX >> 5U) - 1U,
    .TXA_maxSize = <#if MC.MCP_ASYNC_OVER_UART_A_EN > (MCP_TX_ASYNC_PAYLOAD_MAX_A >> 6U), <#else> 0U, </#if>
    .version = 0x0U,
  },
  .syncBuffer =
  {
   .buffer = MCPSyncTxBuff,
  },
  <#if MC.MCP_ASYNC_OVER_UART_A_EN>
  .asyncBufferA =
  {
    .buffer = MCPAsyncBuffUARTA_A,
  },
  .asyncBufferB =
  {
    .buffer = MCPAsyncBuffUARTA_B,
  },
  </#if><#-- MC.MCP_ASYNC_OVER_UART_A_EN -->
  .rxBuffer = <#if MC.MCP_OVER_STLNK_EN >&MCPSyncRXBuff[4]<#else>MCPSyncRXBuff</#if>, 
  .fASPEP_HWInit = &UASPEP_INIT,
  .fASPEP_HWSync = &UASPEP_IDLE_ENABLE,
  .fASPEP_cfg_recept = &UASPEP_CFG_RECEPTION,
  .fASPEP_cfg_trans = &UASPEP_CFG_TRANSMISSION,
  <#if !MCP_DMA>
  .fASPEP_trig_recept = &UASPEP_TRIG_RECEPTION,
  .fASPEP_trig_trans = &UASPEP_TRIG_TRANSMISSION,
  </#if><#-- !MCP_DMA -->
  .liid = 0,
};

MCP_Handle_t MCP_Over_UartA =
{
  .pTransportLayer = (MCTL_Handle_t *)&aspepOverUartA, //cstat !MISRAC2012-Rule-11.3
};

</#if><#-- MC.MCP_OVER_UART_A_EN -->

<#if MC.MCP_OVER_UART_B_EN>
UASPEP_Handle_t UASPEP_B =
{
 .USARTx = USARTB,
  <#if MCP_DMA>
 .rxDMA = DMA_RX_B,
 .txDMA = DMA_TX_B,
 .rxChannel = DMACH_RX_B,
 .txChannel = DMACH_TX_B,
  </#if><#-- MCP_DMA -->
};

ASPEP_Handle_t aspepOverUartB =
{
  ._Super = 
   {
    .fGetBuffer = &ASPEP_getBuffer,
    .fSendPacket = &ASPEP_sendPacket,
    .fRXPacketProcess = &ASPEP_RXframeProcess,
    },
  .ASPEPIp = &UASPEP_B,
  .Capabilities =
  {
    .DATA_CRC = 0,
    .RX_maxSize =  (MCP_RX_SYNC_PAYLOAD_MAX >> 5U) - 1U,
    .TXS_maxSize = (MCP_TX_SYNC_PAYLOAD_MAX >> 5U) - 1U,
    .TXA_maxSize = <#if MC.MCP_ASYNC_OVER_UART_B_EN > (MCP_TX_ASYNC_PAYLOAD_MAX_B >> 6U), <#else> 0U, </#if>
    .version = 0x0,
  },
  .syncBuffer =
  {
   .buffer = MCPSyncTxBuff,
  },
  <#if MC.MCP_ASYNC_OVER_UART_B_EN>
  .asyncBufferA =
  {
    .buffer = MCPAsyncBuffUARTB_A,
  },
  .asyncBufferB =
  {
    .buffer = MCPAsyncBuffUARTB_B,
  },
  </#if><#-- MC.MCP_ASYNC_OVER_UART_B_EN -->
  .rxBuffer = <#if MC.MCP_OVER_STLNK_EN >&MCPSyncRXBuff[4]<#else>MCPSyncRXBuff</#if>, 
  .fASPEP_HWInit = &UASPEP_INIT,
  .fASPEP_HWSync = &UASPEP_IDLE_ENABLE,
  .fASPEP_receive = &UASPEP_RECEIVE_BUFFER,
  .fASPEP_send = &UASPEP_SEND_PACKET,
  .liid = 1,
};

MCP_Handle_t MCP_Over_UartB =
{
  .pTransportLayer = (MCTL_Handle_t *)&aspepOverUartB, //cstat !MISRAC2012-Rule-11.3
};

</#if><#-- MC.MCP_OVER_UART_B_EN -->

<#if MC.MCP_OVER_STLNK_EN>

STLNK_Handle_t STLNK =
{
  ._Super = 
   {
     .fGetBuffer = &STLNK_getBuffer,
     .fSendPacket = &STLNK_sendPacket,
     .fRXPacketProcess = &STLNK_RXframeProcess,
   },
  .syncBuffer =
  {
   .buffer = MCPSyncTxBuff,
  },
  <#if MC.MCP_ASYNC_OVER_STLNK_EN>
  .asyncBufferA =
  {
    .buffer = STLnkAsyncBuffA,
  },
  .asyncBufferB =
  {
    .buffer = STLnkAsyncBuffB,
  },
  </#if><#--  MC.MCP_ASYNC_OVER_STLNK_EN -->
  .rxBuffer = MCPSyncRXBuff,

};

MCP_Handle_t MCP_Over_STLNK =
{
  .pTransportLayer = (MCTL_Handle_t *)&STLNK, //cstat !MISRAC2012-Rule-11.3
};

//force Variable to section data in RAM -> variable is place has the beginning of the section and this is what we want

#if defined (__ICCARM__)
#pragma location=0x20000000
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".data")))
#endif
STLNK_Control_t stlnkCtrl =
{
  .rxBuffer = MCPSyncRXBuff,
  .syncBuffer = MCPSyncTxBuff,
  .asyncBufferA = <#if MC.MCP_ASYNC_OVER_STLNK_EN > &STLnkAsyncBuffA[4], <#else> 0, </#if>
  .asyncBufferB = <#if MC.MCP_ASYNC_OVER_STLNK_EN > &STLnkAsyncBuffB[4], <#else> 0, </#if>
}; 
</#if><#-- MC.MCP_OVER_STLNK_EN -->

<#if MC.MCP_ASYNC_OVER_UART_A_EN>
MCPA_Handle_t MCPA_UART_A =
{
  .pTransportLayer = (MCTL_Handle_t *) &aspepOverUartA, //cstat !MISRAC2012-Rule-11.3
  .dataPtrTable = dataPtrTableA,
  .dataPtrTableBuff = dataPtrTableBuffA,
  .dataSizeTable = dataSizeTableA,
  .dataSizeTableBuff = dataSizeTableBuffA,
  .nbrOfDataLog = MCPA_OVER_UARTA_STREAM,
};
</#if><#-- MC.MCP_ASYNC_OVER_UART_A_EN -->

<#if MC.MCP_ASYNC_OVER_UART_B_EN>
MCPA_Handle_t MCPA_UART_B =
{
  .pTransportLayer = (MCTL_Handle_t *) &aspepOverUartB, //cstat !MISRAC2012-Rule-11.3
  .dataPtrTable = dataPtrTableB,
  .dataPtrTableBuff = dataPtrTableBuffB,  
  .dataSizeTable = dataSizeTableB,
  .dataSizeTableBuff = dataSizeTableBuffB,
  .nbrOfDataLog = MCPA_OVER_UARTB_STREAM,
};

</#if><#-- MC.MCP_ASYNC_OVER_UART_B_EN -->

<#if MC.MCP_ASYNC_OVER_STLNK_EN>
MCPA_Handle_t MCPA_STLNK =
{
  .pTransportLayer = (MCTL_Handle_t *) &STLNK, //cstat !MISRAC2012-Rule-11.3
  .dataPtrTable = dataPtrTableSTlnk,
  .dataPtrTableBuff = dataPtrTableBuffSTlnk,  
  .dataSizeTable = dataSizeTableSTlnk,
  .dataSizeTableBuff = dataSizeTableBuffSTlnk,
  .nbrOfDataLog = MCPA_OVER_STLNK_STREAM,
  
};

</#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
