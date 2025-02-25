<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
/**
  ******************************************************************************
  * @file    stm32_mc_common_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control for the STM32 Family
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
<#if CondFamily_STM32C0>
  * @ingroup STM32C0xx_IRQ_Handlers
<#elseif CondFamily_STM32F0>
  * @ingroup STM32F0xx_IRQ_Handlers
<#elseif CondFamily_STM32F4>
  * @ingroup STM32F4xx_IRQ_Handlers
<#elseif CondFamily_STM32F7>
  * @ingroup STM32F7xx_IRQ_Handlers
<#elseif CondFamily_STM32F3>
  * @ingroup STM32F30x_IRQ_Handlers
<#elseif CondFamily_STM32G0>
  * @ingroup STM32G0xx_IRQ_Handlers
<#elseif CondFamily_STM32G4>
  * @ingroup STM32G4xx_IRQ_Handlers
<#elseif CondFamily_STM32H5>
  * @ingroup STM32H5xx_IRQ_Handlers
<#elseif CondFamily_STM32H7>
  * @ingroup STM32H7xx_IRQ_Handlers
<#elseif CondFamily_STM32L4>
  * @ingroup STM32L4xx_IRQ_Handlers
</#if><#-- CondFamily_STM32C0 CondFamily_STM32F0 CondFamily_STM32F4 CondFamily_STM32F7 CondFamily_STM32F3 CondFamily_STM32G0 CondFamily_STM32G4 CondFamily_STM32H7 CondFamily_STM32L4 -->
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mc_type.h"
<#if CondFamily_STM32G4>
//cstat -MISRAC2012-Rule-3.1
#include "mc_tasks.h"
//cstat +MISRAC2012-Rule-3.1
<#else>
#include "mc_tasks.h"
</#if><#-- CondFamily_STM32G4 -->
#include "parameters_conversion.h"
#include "motorcontrol.h"
<#if CondFamily_STM32C0>
  <#if (MC.START_STOP_BTN == true) || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#include "stm32c0xx_ll_exti.h"
  </#if>
#include "stm32c0xx_hal.h"
#include "stm32c0xx.h"
<#elseif CondFamily_STM32F0> 
  <#if (MC.START_STOP_BTN == true) || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#include "stm32f0xx_ll_exti.h"
  </#if>
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
<#elseif CondFamily_STM32F4>
  <#if (MC.START_STOP_BTN == true) || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#include "stm32f4xx_ll_exti.h"
  </#if>
<#elseif CondFamily_STM32F7>
  <#if (MC.START_STOP_BTN == true) || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#include "stm32f7xx_ll_exti.h"
  </#if>
<#elseif CondFamily_STM32F3>
  <#if MC.RTOS == "FREERTOS">
#include "cmsis_os.h"
  </#if>
  <#if (MC.START_STOP_BTN == true) || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#include "stm32f3xx_ll_exti.h"
  </#if>
#include "stm32f3xx_it.h"
<#elseif CondFamily_STM32G0>
  <#if (MC.START_STOP_BTN == true) || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#include "stm32g0xx_ll_exti.h"
  </#if>
<#elseif CondFamily_STM32G4>
  <#if (MC.START_STOP_BTN == true) || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#include "stm32g4xx_ll_exti.h"
  </#if>
#include "stm32g4xx_hal.h"
#include "stm32g4xx.h"
<#elseif CondFamily_STM32H5>
  <#if (MC.START_STOP_BTN == true) || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#include "stm32h5xx_ll_exti.h"
#include "stm32h5xx_hal.h"
  </#if>
#include "stm32h5xx.h"
<#elseif CondFamily_STM32H7>
  <#if MC.START_STOP_BTN == true>
#include "stm32h7xx_ll_exti.h"
  </#if>
<#elseif CondFamily_STM32L4>
  <#if (MC.START_STOP_BTN == true) || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#include "stm32l4xx_ll_exti.h"
  </#if>
</#if><#-- CondFamily_STM32C0 CondFamily_STM32F0 CondFamily_STM32F4 CondFamily_STM32F7 CondFamily_STM32F3 CondFamily_STM32G0 CondFamily_STM32G4 CondFamily_STM32H5 CondFamily_STM32H7 CondFamily_STM32L4 -->
<#if MC.MCP_EN>
#include "mcp_config.h"  
</#if> <#-- MC.MCP_EN -->

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */
<#if CondFamily_STM32C0>
/** @addtogroup STM32C0xx_IRQ_Handlers STM32C0xx IRQ Handlers
<#elseif CondFamily_STM32F0>
/** @addtogroup STM32F0xx_IRQ_Handlers STM32F0xx IRQ Handlers
<#elseif CondFamily_STM32F4>
/** @addtogroup STM32F4xx_IRQ_Handlers STM32F4xx IRQ Handlers
<#elseif CondFamily_STM32F7>
/** @addtogroup STM32F7xx_IRQ_Handlers STM32F7xx IRQ Handlers
<#elseif CondFamily_STM32F3>
/** @addtogroup STM32F30x_IRQ_Handlers STM32F30x IRQ Handlers
<#elseif CondFamily_STM32G0>
/** @addtogroup STM32G0xx_IRQ_Handlers STM32F0xx IRQ Handlers
<#elseif CondFamily_STM32G4>
/** @addtogroup STM32G4xx_IRQ_Handlers STM32G4xx IRQ Handlers
<#elseif CondFamily_STM32H5>
/** @addtogroup STM32H5xx_IRQ_Handlers STM32H5xx IRQ Handlers
<#elseif CondFamily_STM32H7>
/** @addtogroup STM32H7xx_IRQ_Handlers STM32H7xx IRQ Handlers
<#elseif CondFamily_STM32L4>
/** @addtogroup STM32L4xx_IRQ_Handlers STM32L4xx IRQ Handlers
</#if><#-- CondFamily_STM32C0 CondFamily_STM32F0 CondFamily_STM32F4 CondFamily_STM32F7 CondFamily_STM32F3 CondFamily_STM32G0 CondFamily_STM32G4 CondFamily_STM32H5 CondFamily_STM32H7 CondFamily_STM32L4 -->
  * @{
  */

/* USER CODE BEGIN PRIVATE */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000U)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
<#if !CondFamily_STM32F0>
bool bdmaActivTc = false;
</#if> <#-- !CondFamily_STM32F0 -->

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* USER CODE END PRIVATE */

<#if MC.START_STOP_BTN == true || ((MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") && CondFamily_STM32F3)>
void ${EXT_IRQHandler(_last_word(MC.START_STOP_GPIO_PIN)?number)}(void);
</#if><#-- MC.START_STOP_BTN == true || ((MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") && CondFamily_STM32F3) -->
void HardFault_Handler(void);
void SysTick_Handler(void);
<#if CondFamily_STM32C0 || CondFamily_STM32G0>
  <#if M1_ENCODER == true >
${IRQHandler_name(_last_word(MC.M1_ENC_TIMER_SELECTION))};
  </#if><#-- M1_ENCODER == true -->
<#else> <#--CondFamily_STM32F0 || CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 || CondFamily_STM32H7 || CondFamily_STM32L4>
  <#if (M1_ENCODER == true) >
void SPD_TIM_M1_IRQHandler(void);
  </#if><#-- (M1_ENCODER == true) -->
  <#if CondFamily_STM32F4 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
    <#if MC.DRIVE_NUMBER != "1">
      <#if (M2_ENCODER == true)>
void SPD_TIM_M2_IRQHandler(void);
      </#if><#-- (M2_ENCODER == true) -->
    </#if><#-- MC.DRIVE_NUMBER > 1 -->
  </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 -->
</#if><#-- CondFamily_STM32C0 CondFamily_STM32F0 CondFamily_STM32F4 CondFamily_STM32F7 CondFamily_STM32F3 CondFamily_STM32G0 CondFamily_STM32G4 CondFamily_STM32H5 CondFamily_STM32H7 CondFamily_STM32L4 -->
<#if CondFamily_STM32F3>
  <#if MC.PFC_ENABLED == true && MC.M1_PWM_TIMER_SELECTION == 'PWM_TIM8' && MC.DRIVE_NUMBER == 1>
void TIM1_UP_TIM16_IRQHandler(void);
  </#if><#-- MC.PFC_ENABLED == true && MC.M1_PWM_TIMER_SELECTION == 'PWM_TIM8' && MC.DRIVE_NUMBER == 1 -->
</#if><#-- CondFamily_STM32F3 -->


<#if CondFamily_STM32F3 || CondFamily_STM32G4>
  <#if MC.MCP_OVER_STLNK_EN >
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */

  STLNK_HWDataTransmittedIT(&STLNK);
  
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}
  </#if><#-- MC.MCP_OVER_STLNK_EN -->
</#if><#-- CondFamily_STM32F3 || CondFamily_STM32G4 -->

<#if (M1_ENCODER == true)>
/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  */
  <#if CondFamily_STM32C0 || CondFamily_STM32G0>
${IRQHandler_name(SensorTimer)}
  <#else>
void SPD_TIM_M1_IRQHandler(void)
  </#if><#-- CondFamily_STM32C0 || CondFamily_STM32G0 -->
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */ 

  /* Encoder Timer UPDATE IT is dynamicaly enabled/disabled, checking enable state is required */
  if (LL_TIM_IsEnabledIT_UPDATE(ENCODER_M1.TIMx) != 0U)
  {
    if (LL_TIM_IsActiveFlag_UPDATE(ENCODER_M1.TIMx) != 0U)
    {
      LL_TIM_ClearFlag_UPDATE(ENCODER_M1.TIMx);
      (void)ENC_IRQHandler(&ENCODER_M1);

      /* USER CODE BEGIN M1 ENCODER_Update */

      /* USER CODE END M1 ENCODER_Update */ 
    }
    else
    {
      /* No other IT to manage for encoder config */
    }
  }
  else
  {
    /* No other IT to manage for encoder config */
  }

  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */ 
}
</#if><#-- (M1_ENCODER == true) -->

<#if CondFamily_STM32F4 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
  <#if MC.DRIVE_NUMBER != "1">
    <#if (M2_ENCODER==true)>
/**
  * @brief  This function handles TIMx global interrupt request for M2 Speed Sensor.
  * @param  None
  */
void SPD_TIM_M2_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M2_IRQn 0 */

  /* USER CODE END SPD_TIM_M2_IRQn 0 */ 

  /* Encoder Timer UPDATE IT is dynamicaly enabled/disabled, checking enable state is required */
  if (LL_TIM_IsEnabledIT_UPDATE(ENCODER_M2.TIMx) && LL_TIM_IsActiveFlag_UPDATE(ENCODER_M2.TIMx))
  { 
    LL_TIM_ClearFlag_UPDATE(ENCODER_M2.TIMx);
    ENC_IRQHandler(&ENCODER_M2);

    /* USER CODE BEGIN M2 ENCODER_Update */

    /* USER CODE END M2 ENCODER_Update   */ 
  }
  else
  {
    /* No other IT to manage for encoder config */
  }

  /* USER CODE BEGIN SPD_TIM_M2_IRQn 1 */

  /* USER CODE END SPD_TIM_M2_IRQn 1 */ 
}
    </#if><#-- (M2_ENCODER == true)  -->
  </#if><#-- MC.DRIVE_NUMBER != "1"  -->
</#if><#-- CondFamily_STM32F4 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 -->
<#-- ST MCWB monitoring usage management (used when MC.MCP_OVER_UART_A_EN == true) -->
<#if MC.MCP_OVER_UART_A_EN == true>
/* This section is present only when MCP over UART_A is used */
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  */
  <#if CondFamily_STM32G4>
//cstat !MISRAC2012-Rule-8.4
  </#if><#-- CondFamily_STM32G4 -->
void ${MC.MCP_IRQ_HANDLER_UART_A}(void)
{
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_UART_A} 0 */
  
  /* USER CODE END ${MC.MCP_IRQ_HANDLER_UART_A} 0 */
  uint32_t flags;
  uint32_t activeIdleFlag;
  uint32_t isEnabledIdleFlag;
  
  <#if MCP_DMA>
  if (0U == LL_USART_IsActiveFlag_TC(USARTA))
  {
    /* Nothing to do */
  }
  else
  {
    /* Disable the DMA channel to prepare the next chunck of data*/
  <#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32H7>
    LL_DMA_DisableStream(DMA_TX_A, DMACH_TX_A);
  <#else>
    LL_DMA_DisableChannel(DMA_TX_A, DMACH_TX_A);
  </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32H7 -->
    LL_USART_ClearFlag_TC(USARTA);
    /* Data Sent by UART*/
    /* Need to free the buffer, and to check pending transfer*/
    ASPEP_HWDataTransmittedIT(&aspepOverUartA);
  }
  <#else><#-- !MCP_DMA -->
  uint32_t activeRxFlag;
  uint32_t isEnabledRxFlag;

  activeRxFlag = LL_USART_IsActiveFlag_RXNE(USARTA);
  isEnabledRxFlag = LL_USART_IsEnabledIT_RXNE(USARTA);
  flags = activeRxFlag & isEnabledRxFlag;

  if (0U == flags)
  {
    /* Nothing to do */
  }
  else  /* Valid data have been received */
  {
    aspepOverUartA.rxComplete = aspepOverUartA.fASPEP_trig_recept(aspepOverUartA.ASPEPIp);
  }
  uint32_t activeTxFlag;
  uint32_t isEnabledTxFlag;
  
  activeTxFlag = LL_USART_IsActiveFlag_TXE(USARTA);
  isEnabledTxFlag = LL_USART_IsEnabledIT_TXE_TXFNF(USARTA);

  flags = activeTxFlag & isEnabledTxFlag;
  if(0U == flags)
  {
    /* Nothing to do */
  }
  else  /* Data to be transmitted */
  {
    aspepOverUartA.txComplete = aspepOverUartA.fASPEP_trig_trans(aspepOverUartA.ASPEPIp);
  }
  </#if><#-- MCP_DMA -->
  <#if CondFamily_STM32C0 || CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G0 || CondFamily_STM32G4 || CondFamily_STM32H5 || CondFamily_STM32L4>
  uint32_t oreFlag;
  uint32_t feFlag;
  uint32_t neFlag;
  uint32_t errorMask;

  oreFlag = LL_USART_IsActiveFlag_ORE(USARTA);
  feFlag = LL_USART_IsActiveFlag_FE(USARTA);
  neFlag = LL_USART_IsActiveFlag_NE(USARTA);
  errorMask = LL_USART_IsEnabledIT_ERROR(USARTA);
  
  flags = ((oreFlag | feFlag | neFlag) & errorMask);
  if (0U == flags)
  {
    /* Nothing to do */
  }
  else
  { /* Stopping the debugger will generate an OverRun error*/
    <#if CondFamily_STM32C0  || CondFamily_STM32G0 || CondFamily_STM32G4 || CondFamily_STM32H5 || CondFamily_STM32L4>
    WRITE_REG(USARTA->ICR, USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
   <#elseif CondFamily_STM32F3 || CondFamily_STM32F7>
    WRITE_REG(USARTA->ICR, USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NCF);
   </#if> <#-- CondFamily_STM32C0 CondFamily_STM32F7 CondFamily_STM32G0 CondFamily_STM32G4 CondFamily_STM32H5  CondFamily_STM32L4 CondFamily_STM32F3 -->
    <#if CondFamily_STM32F4 || CondFamily_STM32H5>
    LL_USART_ClearFlag_FE(USARTA);
    LL_USART_ClearFlag_ORE(USARTA);
    LL_USART_ClearFlag_NE(USARTA);
    </#if><#-- CondFamily_STM32C0 CondFamily_STM32F4 CondFamily_STM32F7 CondFamily_STM32G0 CondFamily_STM32G4 CondFamily_STM32H5 CondFamily_STM32L4 -->
    /* We disable ERROR interrupt to avoid to trig one Overrun IT per additional byte recevied*/
    LL_USART_DisableIT_ERROR(USARTA);
    LL_USART_EnableIT_IDLE(USARTA);
  }
  <#elseif CondFamily_STM32F0>
  if (LL_USART_IsActiveFlag_ORE(USARTA))
  { /* Stopping the debugger will generate an OverRun error*/
    LL_USART_ClearFlag_ORE(USARTA);
    LL_USART_EnableIT_IDLE(USARTA);
  }
  else
  {
    /* Nothing to do */
  }
  <#elseif CondFamily_STM32H7>
  if ((LL_USART_IsActiveFlag_ORE(USARTA) || LL_USART_IsActiveFlag_FE(USARTA) || LL_USART_IsActiveFlag_NE(USARTA)) 
    && LL_USART_IsEnabledIT_ERROR(USARTA))
  { /* We disable ERROR interrupt to avoid to trig one Overrun IT per additional byte recevied*/
    LL_USART_DisableIT_ERROR(USARTA);
    LL_USART_EnableIT_IDLE(USARTA);
  }
  else
  {
    /* Nothing to do */
  }
  </#if><#-- CondFamily_STM32C0 CondFamily_STM32F0 CondFamily_STM32F4 CondFamily_STM32F7 CondFamily_STM32F3 CondFamily_STM32G0 CondFamily_STM32G4 CondFamily_STM32H5 CondFamily_STM32L4 -->

  activeIdleFlag = LL_USART_IsActiveFlag_IDLE(USARTA);
  isEnabledIdleFlag = LL_USART_IsEnabledIT_IDLE(USARTA);

  flags = activeIdleFlag & isEnabledIdleFlag;
  if (0U == flags)
  {
    /* Nothing to do */
  }
  else
  { /* Stopping the debugger will generate an OverRun error*/
  <#if CondFamily_STM32H7>
    LL_USART_ClearFlag_FE(USARTA);
    LL_USART_ClearFlag_ORE(USARTA);
    LL_USART_ClearFlag_NE(USARTA);
  </#if><#-- CondFamily_STM32H7 -->
    LL_USART_DisableIT_IDLE(USARTA);
  <#if CondFamily_STM32C0 || CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G0 || CondFamily_STM32G4 || CondFamily_STM32H5 || CondFamily_STM32H7 || CondFamily_STM32L4>
    /* Once the complete unexpected data are received, we enable back the error IT*/
    LL_USART_EnableIT_ERROR(USARTA);
  </#if><#-- CondFamily_STM32C0 || CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32G0 || CondFamily_STM32G4 || CondFamily_STM32H5 || CondFamily_STM32H7 CondFamily_STM32L4 -->
  <#if MCP_DMA>
    /* To be sure we fetch the potential pending data*/
    /* We disable the DMA request, Read the dummy data, endable back the DMA request */
    LL_USART_DisableDMAReq_RX(USARTA);
    (void)LL_USART_ReceiveData8(USARTA);
    LL_USART_EnableDMAReq_RX(USARTA);
    <#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32H7>
    LL_DMA_ClearFlag_TE(DMA_RX_A, DMACH_RX_A);
    </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32H7 -->
    <#if MC.MCP_OVER_UART_A_EN>
    /* Clear pending DMA TC to process only new received packet */ 
    LL_DMA_ClearFlag_TC(DMA_RX_A, DMACH_RX_A);
    </#if><#--  MC.MCP_OVER_UART_A_EN -->
    <#if MC.MCP_OVER_UART_B_EN>
    /* Clear pending DMA TC to process only new received packet */
    LL_DMA_ClearFlag_TC(DMA_RX_B, DMACH_RX_B);
    </#if><#--  MC.MCP_OVER_UART_B_EN -->
  <#else><#-- !MCP_DMA -->
    (void)LL_USART_ReceiveData8(USARTA);
  </#if><#-- MCP_DMA -->
    ASPEP_HWReset(&aspepOverUartA);
  }

  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_UART_A} 1 */
 
  /* USER CODE END ${MC.MCP_IRQ_HANDLER_UART_A} 1 */
}
</#if><#-- MC.MCP_OVER_UART_A_EN -->

<#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32G4 || CondFamily_STM32H5>
  <#if MC.MCP_OVER_UART_B_EN>
/* This section is present only when serial communication is used */
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  */
void USARTB_IRQHandler(void)
{
  /* USER CODE BEGIN USARTB_IRQn 0 */
  
  /* USER CODE END USARTB_IRQn 0 */
  if (0U == LL_USART_IsActiveFlag_TC(USARTB))
  {
    /* Nothing to do */
  }
  else
  {
    /* Disable the DMA channel to prepare the next chunck of data*/
    <#if CondFamily_STM32G4 || CondFamily_STM32H5>
    LL_DMA_DisableChannel(DMA_TX_B, DMACH_TX_B);
    <#else>
    LL_DMA_DisableStream(DMA_TX_B, DMACH_TX_B);
    </#if><#-- CondFamily_STM32G4 || CondFamily_STM32H5 -->
    LL_USART_ClearFlag_TC(USARTB);
    /* Data Sent by UART*/
    /* Need to free the buffer, and to check pending transfer*/
    ASPEP_HWDataTransmittedIT(&aspepOverUartB);
  }

  uint32_t flags;
  uint32_t oreFlag;
  uint32_t feFlag;
  uint32_t neFlag;
  uint32_t errorMask;
  uint32_t activeIdleFlag;
  uint32_t isEnabledIdleFlag;
  oreFlag = LL_USART_IsActiveFlag_ORE(USARTA);
  feFlag = LL_USART_IsActiveFlag_FE(USARTA);
  neFlag = LL_USART_IsActiveFlag_NE(USARTA);
  errorMask = LL_USART_IsEnabledIT_ERROR(USARTA);
  flags = ((oreFlag | feFlag | neFlag) & errorMask);
  if (0U == flags)
  {
    /* Nothing to do */
  }
  else
  { /* Stopping the debugger will generate an OverRun error*/
    <#if CondFamily_STM32F4 || CondFamily_STM32H5>
    LL_USART_ClearFlag_FE(USARTB);
    LL_USART_ClearFlag_ORE(USARTB);
    LL_USART_ClearFlag_NE(USARTB);
    <#elseif CondFamily_STM32F7>
    WRITE_REG(USARTB->ICR, USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NCF);
    <#elseif CondFamily_STM32G4>
    WRITE_REG(USARTB->ICR, USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);
    </#if><#-- CondFamily_STM32F4 CondFamily_STM32F7 CondFamily_STM32G4 CondFamily_STM32H5 -->
    /* We disable ERROR interrupt to avoid to trig one Overrun IT per additional byte recevied*/
    LL_USART_DisableIT_ERROR(USARTB);
    LL_USART_EnableIT_IDLE(USARTB);        
  }

  activeIdleFlag = LL_USART_IsActiveFlag_IDLE(USARTB);
  isEnabledIdleFlag = LL_USART_IsEnabledIT_IDLE(USARTB);
  flags = activeIdleFlag & isEnabledIdleFlag;
  if (0U == flags)
  {
    /* Nothing to do */
  }
  else
  { /* Stopping the debugger will generate an OverRun error*/
    LL_USART_DisableIT_IDLE(USARTB);
    /* Once the complete unexpected data are received, we enable back the error IT*/
    LL_USART_EnableIT_ERROR(USARTB);    
    /* To be sure we fetch the potential pendig data*/
    /* We disable the DMA request, Read the dummy data, endable back the DMA request */
    LL_USART_DisableDMAReq_RX(USARTB);
    (void)LL_USART_ReceiveData8(USARTB);
    LL_USART_EnableDMAReq_RX(USARTB);
    <#if !CondFamily_STM32G4 || CondFamily_STM32H5>
    LL_DMA_ClearFlag_TE(DMA_RX_B, DMACH_RX_B);
    </#if><#-- !CondFamily_STM32G4 || CondFamily_STM32H5 -->
    ASPEP_HWReset(&aspepOverUartB);
  }  
 
  /* USER CODE BEGIN USARTB_IRQn 1 */
 
  /* USER CODE END USARTB_IRQn 1 */
}
  </#if><#-- MC.MCP_OVER_UART_B_EN -->
</#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32G4 || CondFamily_STM32H5 -->
/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  */
void HardFault_Handler(void)
{
 /* USER CODE BEGIN HardFault_IRQn 0 */

 /* USER CODE END HardFault_IRQn 0 */

  TSK_HardwareFaultTask();
  
  /* Go to infinite loop when Hard Fault exception occurs */
  while (true)
  {
    /* Nothing to do */
  }

 /* USER CODE BEGIN HardFault_IRQn 1 */

 /* USER CODE END HardFault_IRQn 1 */
}

<#if MC.RTOS == "NONE">
void SysTick_Handler(void)
{
#ifdef MC_HAL_IS_USED
static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  if (SystickDividerCounter == SYSTICK_DIVIDER)
  {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    SystickDividerCounter = 0;
  }
  else
  {
    /* Nothing to do */
  }

  SystickDividerCounter ++;  
#endif /* MC_HAL_IS_USED */
  <#if MCP_DMA>
  /* Buffer is ready by the HW layer to be processed */
  /* NO DMA interrupt */
    <#if MC.MCP_OVER_UART_A_EN>
  if (LL_DMA_IsActiveFlag_TC(DMA_RX_A, DMACH_RX_A))
  {
    LL_DMA_ClearFlag_TC(DMA_RX_A, DMACH_RX_A);
    ASPEP_HWDataReceivedIT(&aspepOverUartA);
  }
  else
  {
    /* Nothing to do */
  }
    </#if><#--  MC.MCP_OVER_UART_A_EN -->
    <#if MC.MCP_OVER_UART_B_EN>
  if (LL_DMA_IsActiveFlag_TC(DMA_RX_B, DMACH_RX_B))
  {
    LL_DMA_ClearFlag_TC(DMA_RX_B, DMACH_RX_B);
    ASPEP_HWDataReceivedIT(&aspepOverUartB);
  }
  else
  {
    /* Nothing to do */
  }
    </#if><#--  MC.MCP_OVER_UART_B_EN -->
  <#else><#-- !MCP_DMA -->
    <#if MC.MCP_OVER_UART_A_EN>
  if(aspepOverUartA.rxComplete == true)
  {
    ASPEP_HWDataReceivedIT(&aspepOverUartA);
    aspepOverUartA.rxComplete = false;
  }
  else if(aspepOverUartA.txComplete == true)
  {
    ASPEP_HWDataTransmittedIT(&aspepOverUartA);
    aspepOverUartA.txComplete = false;
  }
    </#if><#-- MC.MCP_OVER_UART_A_EN -->
  </#if><#-- MCP_DMA -->
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */

    MC_RunMotorControlTasks();

  <#if CondFamily_STM32F0 || CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G0 || CondFamily_STM32G4 || CondFamily_STM32H5 || CondFamily_STM32L4>
    <#if  MC.M1_POSITION_CTRL_ENABLING == true >
    TC_IncTick(&PosCtrlM1);
    </#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->
  </#if><#-- CondFamily_STM32F0 || CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G0 || CondFamily_STM32G4 || CondFamily_STM32H5 || CondFamily_STM32L4 -->
  <#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
    <#if  MC.M2_POSITION_CTRL_ENABLING == true >
    TC_IncTick(&PosCtrlM2);
    </#if><#-- MC.M2_POSITION_CTRL_ENABLING == true -->
  </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 -->

  /* USER CODE BEGIN SysTick_IRQn 2 */

  /* USER CODE END SysTick_IRQn 2 */
}
</#if> <#--  MC.RTOS == "NONE" -->

<#if CondFamily_STM32H7>
  <#if MC.START_STOP_BTN == true>
/* GUI, this section is present only if start/stop button is enabled */
/**
  * @brief  This function handles Button IRQ on PIN P${ _last_char(MC.START_STOP_GPIO_PORT)}${_last_word(MC.START_STOP_GPIO_PIN)}.
  */
void ${EXT_IRQHandler(_last_word(MC.START_STOP_GPIO_PIN)?number)}(void)
{
/* USER CODE BEGIN START_STOP_BTN */

  /* Start/Stop button handled by CM4 (C2) */  
    <#if _last_word(MC.START_STOP_GPIO_PIN)?number < 32 >
  LL_C2_EXTI_ClearFlag_0_31(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});
    <#else>
  LL_C2_EXTI_ClearFlag_32_63(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});
    </#if>
  UI_HandleStartStopButton_cb();

/* USER CODE END START_STOP_BTN */
}
  </#if>
<#else>
<#if MC.START_STOP_BTN == true || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")
 || ((MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z") && (CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5)) >

<#-- GUI, this section is present only if start/stop button and/or Position Control with Z channel is enabled -->
  
  <#assign EXT_IRQHandler_StartStopName = "" >
  <#assign EXT_IRQHandler_ENC_Z_M1_Name = "" >
  <#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
  <#assign EXT_IRQHandler_ENC_Z_M2_Name = "" >
  </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 | CondFamily_STM32G4 || CondFamily_STM32H5 -->
  <#assign Template_StartStop ="">
  <#assign Template_Encoder_Z_M1 ="">
  <#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
  <#assign Template_Encoder_Z_M2 ="">
  </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 -->

  <#if MC.START_STOP_BTN == true>
    <#assign EXT_IRQHandler_StartStopName = "${EXT_IRQHandler(_last_word(MC.START_STOP_GPIO_PIN)?number)}" >
    <#if CondFamily_STM32C0 || CondFamily_STM32G0>
      <#if _last_word(MC.START_STOP_GPIO_PIN)?number < 32 >
      <#assign Template_StartStop = '/* USER CODE BEGIN START_STOP_BTN */
  if (LL_EXTI_ReadRisingFlag_0_31(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)}) ||
      LL_EXTI_ReadFallingFlag_0_31(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)})) 
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});  
    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});
    UI_HandleStartStopButton_cb();
  }'> 
      <#else>
      <#assign Template_StartStop = '/* USER CODE BEGIN START_STOP_BTN */
  if (LL_EXTI_ReadRisingFlag_32_63(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)}) ||
      LL_EXTI_ReadFallingFlag_32_63(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)}))
  {
    LL_EXTI_ClearRisingFlag_32_63(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});  
    LL_EXTI_ClearFallingFlag_32_63(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});
    UI_HandleStartStopButton_cb();
  }'>
      </#if> <#-- _last_word(MC.START_STOP_GPIO_PIN)?number < 32 -->
    <#elseif CondFamily_STM32H5>
      <#if _last_word(MC.START_STOP_GPIO_PIN)?number < 32 >
        <#assign Template_StartStop = '/* USER CODE BEGIN START_STOP_BTN */
  if ( LL_EXTI_ReadFallingFlag_0_31(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)}) ) 
  {                                                                                
    LL_EXTI_ClearFallingFlag_0_31 (LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});  
    (void)UI_HandleStartStopButton_cb();
  }'> 
    <#else><#-- _last_word(MC.START_STOP_GPIO_PIN)?number >= 32 -->
    <#assign Template_StartStop = '/* USER CODE BEGIN START_STOP_BTN */
  if (LL_EXTI_ReadFlag_32_63(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)}))
  {
    LL_EXTI_ClearFlag_32_63(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});
    (void)UI_HandleStartStopButton_cb();
  }'>
      </#if> <#-- _last_word(MC.START_STOP_GPIO_PIN)?number < 32 -->
    <#elseif CondFamily_STM32F0 || CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 || CondFamily_STM32L4>
      <#if _last_word(MC.START_STOP_GPIO_PIN)?number < 32 >
        <#assign Template_StartStop = '/* USER CODE BEGIN START_STOP_BTN */
  if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)}))
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});
    (void)UI_HandleStartStopButton_cb();
  }'>
    <#else><#-- _last_word(MC.START_STOP_GPIO_PIN)?number >= 32 -->
    <#assign Template_StartStop = '/* USER CODE BEGIN START_STOP_BTN */
  if (LL_EXTI_ReadFlag_32_63(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)}))
  {
    LL_EXTI_ClearFlag_32_63(LL_EXTI_LINE_${_last_word(MC.START_STOP_GPIO_PIN)});
    (void)UI_HandleStartStopButton_cb();
  }'>
      </#if> <#-- _last_word(MC.START_STOP_GPIO_PIN)?number < 32 -->
    </#if><#-- CondFamily_STM32C0 CondFamily_STM32F0 CondFamily_STM32F4 CondFamily_STM32F7 CondFamily_STM32F3 CondFamily_STM32G0 CondFamily_STM32G4 CondFamily_STM32H5 CondFamily_STM32L4 --> 
  </#if> <#-- MC.START_STOP_BTN == true -->
  <#if MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z">
    <#assign EXT_IRQHandler_ENC_Z_M1_Name = "${EXT_IRQHandler(_last_word(MC.M1_ENC_Z_GPIO_PIN)?number)}" >
    <#if CondFamily_STM32C0 || CondFamily_STM32G0>
      <#if _last_word(MC.M1_ENC_Z_GPIO_PIN)?number < 32 >
      <#assign Template_Encoder_Z_M1 = '/* USER CODE BEGIN ENCODER Z INDEX M1 */
  if (LL_EXTI_ReadRisingFlag_0_31(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)}) ||
      LL_EXTI_ReadFallingFlag_0_31(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)}))
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)});
    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)});
    TC_EncoderReset(&PosCtrlM1); 
  }'> 
      <#else>
  <#assign Template_StartStop = '/* USER CODE BEGIN ENCODER Z INDEX M1 */
  if (LL_EXTI_ReadRisingFlag_32_63(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)}) ||
      LL_EXTI_ReadFallingFlag_32_63(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)}))
  {
    LL_EXTI_ClearRisingFlag_32_63(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)});
    LL_EXTI_ClearFallingFlag_32_63(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)});
    TC_EncoderReset(&PosCtrlM1); 
  }'>
      </#if> <#-- _last_word(MC.M1_ENC_Z_GPIO_PIN)?number < 32 -->
    <#elseif CondFamily_STM32F0 || CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 || CondFamily_STM32L4>
      <#if _last_word(MC.M1_ENC_Z_GPIO_PIN)?number < 32 >
        <#assign Template_Encoder_Z_M1 = '/* USER CODE BEGIN ENCODER Z INDEX M1 */
  if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)}))
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)});
    TC_EncoderReset(&PosCtrlM1);
  }'>
    <#else><#-- _last_word(MC.M1_ENC_Z_GPIO_PIN)?number >= 32 -->
  <#assign Template_Encoder_Z_M1 = '/* USER CODE BEGIN ENCODER Z INDEX M1 */
  if (LL_EXTI_ReadFlag_32_63(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)})) 
  {
    LL_EXTI_ClearFlag_32_63(LL_EXTI_LINE_${_last_word(MC.M1_ENC_Z_GPIO_PIN)});
    TC_EncoderReset(&PosCtrlM1);
  }'>
      </#if> <#-- _last_word(MC.M1_ENC_Z_GPIO_PIN)?number < 32 -->
    </#if><#-- CondFamily_STM32C0 CondFamily_STM32F0 CondFamily_STM32F4 CondFamily_STM32F7 CondFamily_STM32F3 CondFamily_STM32G0 CondFamily_STM32G4 CondFamily_STM32H5 CondFamily_STM32L4 --> 
  </#if> <#-- MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z" -->
  
  <#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
    <#if MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z">
      <#assign EXT_IRQHandler_ENC_Z_M2_Name = "${EXT_IRQHandler(_last_word(MC.M2_ENC_Z_GPIO_PIN)?number)}" >
      <#if _last_word(MC.M2_ENC_Z_GPIO_PIN)?number < 32>
        <#assign Template_Encoder_Z_M2 = '/* USER CODE BEGIN ENCODER Z INDEX M2 */
  if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_${_last_word(MC.M2_ENC_Z_GPIO_PIN)}))
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_${_last_word(MC.M2_ENC_Z_GPIO_PIN)});
    TC_EncoderReset(&PosCtrlM2);
  }'> 
      <#else><#-- _last_word(MC.M2_ENC_Z_GPIO_PIN)?number >= 32 -->
        <#assign Template_Encoder_Z_M2 = '/* USER CODE BEGIN ENCODER Z INDEX M2 */
  if (LL_EXTI_ReadFlag_32_63(LL_EXTI_LINE_${_last_word(MC.M2_ENC_Z_GPIO_PIN)}))
  {
    LL_EXTI_ClearFlag_32_63(LL_EXTI_LINE_${_last_word(MC.M2_ENC_Z_GPIO_PIN)});
    TC_EncoderReset(&PosCtrlM2);
  }'>
      </#if><#-- _last_word(MC.M2_ENC_Z_GPIO_PIN)?number < 32 -->
    </#if> <#-- MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z" -->
  </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 -->

  <#if MC.START_STOP_BTN == true>
/**
  * @brief  This function handles Button IRQ on PIN P${ _last_char(MC.START_STOP_GPIO_PORT)}${_last_word(MC.START_STOP_GPIO_PIN)}.
    <#if (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") && "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M1_Name}">
  *                 and M1 Encoder Index IRQ on PIN P${ _last_char(MC.M1_ENC_Z_GPIO_PORT)}${_last_word(MC.M1_ENC_Z_GPIO_PIN)}.
    </#if><#-- MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z" && "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M1_Name}" -->
    <#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
      <#if (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z") && "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M2_Name}">
  *                 and M2 Encoder Index IRQ on PIN P${ _last_char(MC.M2_ENC_Z_GPIO_PORT)}${_last_word(MC.M2_ENC_Z_GPIO_PIN)}.
      </#if><#-- MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z" && "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M2_Name}" -->

    </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 -->
  */
void ${EXT_IRQHandler_StartStopName}(void)
{
  ${Template_StartStop}
  else
  {
    /* Nothing to do */
  }

    <#if "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M1_Name}" >
  ${Template_Encoder_Z_M1}
  else
  {
    /* Nothing to do */
  }
    </#if><#-- "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M1_Name}" -->
    <#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
      <#if "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M2_Name}" >
  ${Template_Encoder_Z_M2}
  else
  {
    /* Nothing to do */
  }
      </#if><#-- "${EXT_IRQHandler_StartStopName}" == "${EXT_IRQHandler_ENC_Z_M2_Name}" -->
    </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 -->
}
  </#if> <#-- MC.START_STOP_BTN == true -->
  
  <#if MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z">
    <#if "${EXT_IRQHandler_StartStopName}" != "${EXT_IRQHandler_ENC_Z_M1_Name}" >
/**
  * @brief  This function handles M1 Encoder Index IRQ on PIN P${ _last_char(MC.M1_ENC_Z_GPIO_PORT)}${_last_word(MC.M1_ENC_Z_GPIO_PIN)}.
  <#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
    <#if (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z") && "${EXT_IRQHandler_ENC_Z_M1_Name}" == "${EXT_IRQHandler_ENC_Z_M2_Name}" >
  *                 and M2 Encoder Index IRQ on PIN P${ _last_char(MC.M2_ENC_Z_GPIO_PORT)}${_last_word(MC.M2_ENC_Z_GPIO_PIN)}.
    </#if><#-- MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z" && "${EXT_IRQHandler_ENC_Z_M1_Name}" == "${EXT_IRQHandler_ENC_Z_M2_Name}" -->
  </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 -->
  */

void ${EXT_IRQHandler_ENC_Z_M1_Name}(void)
{
  ${Template_Encoder_Z_M1}
  else
  {
    /* Nothing to do */
  }
  <#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
    <#if "${EXT_IRQHandler_ENC_Z_M1_Name}" == "${EXT_IRQHandler_ENC_Z_M2_Name}">
  ${Template_Encoder_Z_M2}
  else
  {
    /* Nothing to do */
  }
    </#if><#-- "${EXT_IRQHandler_ENC_Z_M1_Name}" == "${EXT_IRQHandler_ENC_Z_M2_Name}" -->
  </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 -->
}  
    </#if> <#-- "${EXT_IRQHandler_StartStopName}" != "${EXT_IRQHandler_ENC_Z_M1_Name}" -->
  </#if> <#-- MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z" --> 

  <#if CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
    <#if MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z">
      <#if "${EXT_IRQHandler_StartStopName}" != "${EXT_IRQHandler_ENC_Z_M2_Name}"
        && "${EXT_IRQHandler_ENC_Z_M1_Name}" != "${EXT_IRQHandler_ENC_Z_M2_Name}">
/**
  * @brief  This function handles M2 Encoder Index IRQ on PIN 
            P${ _last_char(MC.M2_ENC_Z_GPIO_PORT)}${_last_word(MC.M2_ENC_Z_GPIO_PIN)}.
  */  
void ${EXT_IRQHandler_ENC_Z_M2_Name}(void)
{
  ${Template_Encoder_Z_M2}
  else
  {
    /* Nothing to do */
  }
}
      </#if><#-- ${EXT_IRQHandler_StartStopName}" != "${EXT_IRQHandler_ENC_Z_M2_Name}" 
             && "${EXT_IRQHandler_ENC_Z_M1_Name}" != "${EXT_IRQHandler_ENC_Z_M2_Name}" -->
    </#if><#--MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z" -->
  </#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5 -->
</#if> <#-- MC.START_STOP_BTN == true || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") || ((MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z") 
    && (CondFamily_STM32F4 || CondFamily_STM32F7 || CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5)) -->
</#if> <#-- CondFamily_STM32H7 -->

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

