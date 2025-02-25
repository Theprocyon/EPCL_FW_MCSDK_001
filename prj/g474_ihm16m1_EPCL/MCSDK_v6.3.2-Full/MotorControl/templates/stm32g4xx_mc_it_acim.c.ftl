<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
/**
  ******************************************************************************
  * @file    stm32g4xx_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control for the STM32G4 Family.
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
  * @ingroup STM32G4xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mc_type.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"

#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx.h"
<#if MC.MCP_EN>
#include "mcp_config.h"  
</#if>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32G4xx_IRQ_Handlers STM32G4xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

<#if MC.MCP_OVER_STLNK_EN >
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */

  STLNK_HWDataTransmittedIT (&STLNK );
  
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}
</#if>
/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void ADC1_2_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);

void USART_IRQHandler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void EXTI15_10_IRQHandler (void);

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */

  // Clear Flags M1
  LL_ADC_ClearFlag_JEOS( ADC1 );

#if !defined(ACIM_VF)
  // Highfrequency task
  TSK_HighFrequencyTask();

 /* USER CODE BEGIN HighFreq */
#endif
 /* USER CODE END HighFreq  */

 /* USER CODE BEGIN ADC1_2_IRQn 1 */

 /* USER CODE END ADC1_2_IRQn 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  */
void TIMx_UP_M1_IRQHandler(void)
{
 /* USER CODE BEGIN TIMx_UP_M1_IRQn 0 */

 /* USER CODE END  TIMx_UP_M1_IRQn 0 */

    LL_TIM_ClearFlag_UPDATE(TIM1);
#if !defined(ACIM_VF)
    R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);

 /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */
#else // V/f conytol is actived
   TSK_HighFrequencyTask();
#endif
 /* USER CODE END  TIMx_UP_M1_IRQn 1 */
}

void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */
  if (LL_TIM_IsActiveFlag_BRK(TIM1))
  {
    LL_TIM_ClearFlag_BRK(TIM1);
<#if (MC.M1_OCP_TOPOLOGY != "NONE") &&  (MC.M1_OCP_DESTINATION == "TIM_BKIN")>
    PWMC_OCP_Handler(&PWM_Handle_M1._Super);
<#elseif (MC.M1_DP_TOPOLOGY != "NONE") &&  (MC.M1_DP_DESTINATION == "TIM_BKIN")>
    PWMC_DP_Handler(&PWM_Handle_M1._Super);
<#else>
    PWMC_OVP_Handler(&PWM_Handle_M1._Super, ${_last_word(MC.M1_PWM_TIMER_SELECTION)});
</#if>
  }
  if (LL_TIM_IsActiveFlag_BRK2(TIM1))
  {
    LL_TIM_ClearFlag_BRK2(TIM1);
<#if (MC.M1_OCP_TOPOLOGY != "NONE") &&  (MC.M1_OCP_DESTINATION == "TIM_BKIN2")>
    PWMC_OCP_Handler(&PWM_Handle_M1._Super);
<#elseif (MC.M1_DP_TOPOLOGY != "NONE") &&  (MC.M1_DP_DESTINATION == "TIM_BKIN2")>
    PWMC_DP_Handler(&PWM_Handle_M1._Super);
<#else>
    PWMC_OVP_Handler(&PWM_Handle_M1._Super, ${_last_word(MC.M1_PWM_TIMER_SELECTION)});
</#if>
  }
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();

  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */
}


/* This section is present only when serial communication is used */
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  */
void ${MC.MCP_IRQ_HANDLER_UART_A}(void)
{
  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_UART_A} 0 */
  
  /* USER CODE END ${MC.MCP_IRQ_HANDLER_UART_A} 0 */
  uint32_t flags;
  uint32_t activeIdleFlag;
  uint32_t isEnabledIdelFlag;
    
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
  isEnabledIdelFlag = LL_USART_IsEnabledIT_IDLE(USARTA);

  flags = activeIdleFlag & isEnabledIdelFlag;
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
    /* To be sure we fetch the potential pendig data*/
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
    ASPEP_HWReset(&aspepOverUartA);
  }

  /* USER CODE BEGIN ${MC.MCP_IRQ_HANDLER_UART_A} 1 */
 
  /* USER CODE END ${MC.MCP_IRQ_HANDLER_UART_A} 1 */
}

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
  uint32_t isEnabledIdelFlag;
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
  isEnabledIdelFlag = LL_USART_IsEnabledIT_IDLE(USARTB);
  flags = activeIdleFlag & isEnabledIdelFlag;
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
</#if>

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
  while (1)
  {
  }
 /* USER CODE BEGIN HardFault_IRQn 1 */

 /* USER CODE END HardFault_IRQn 1 */
}

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
  SystickDividerCounter ++;
#endif /* MC_HAL_IS_USED */

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
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
    MC_RunMotorControlTasks();

  /* USER CODE BEGIN SysTick_IRQn 2 */
  /* USER CODE END SysTick_IRQn 2 */
}

/**
  * @brief  This function handles Button IRQ on PIN PC13.
  */
void EXTI15_10_IRQHandler (void)
{
	/* USER CODE BEGIN START_STOP_BTN */
  if ( LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_13) )
  {
    LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_13);
    UI_HandleStartStopButton_cb ();
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
