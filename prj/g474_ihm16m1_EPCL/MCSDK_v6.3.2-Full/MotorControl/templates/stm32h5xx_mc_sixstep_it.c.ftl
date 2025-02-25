<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">

/**
  ******************************************************************************
  * @file    stm32h5xx_mc_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control for the STM32H5 Family.
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
  * @ingroup STM32H5xx_IRQ_Handlers
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_config.h"
#include "stm32h5xx.h"
#include "stm32h5xx_it.h"
#include "6step_core.h"
<#-- Only for the TERATERM usage -->
<#if MC.SIX_STEP_COMMUNICATION_IF == "TERATERM_IF"><#-- TERATERM I/F usage -->
#include "6step_com.h"
</#if>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32H5xx_IRQ_Handlers STM32H5xx IRQ Handlers
  * @{
  */
  
/* USER CODE BEGIN PRIVATE */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern MC_Handle_t Motor_Device1;
void SysTick_Handler(void);
void TIM1_BRK_TIM9_IRQHandler(void);
void USART_IRQHandler(void);
<#if (M1_HALL_SENSOR == true)>
void SPD_TIM_M1_IRQHandler(void);
</#if><#-- (M1_HALL_SENSOR == true) -->


<#if (M1_HALL_SENSOR == true)>
/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  */
void SPD_TIM_M1_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */ 
  
  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx))
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    HALL_TIMx_UP_IRQHandler(&HALL_M1);

    /* USER CODE BEGIN M1 HALL_Update */

    /* USER CODE END M1 HALL_Update   */ 
  }
  else
  {
    /* Nothing to do */
  }

  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M1.TIMx)) 
  {
    LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
    HALL_TIMx_CC_IRQHandler(&HALL_M1);
 
    /* USER CODE BEGIN M1 HALL_CC1 */

    /* USER CODE END M1 HALL_CC1 */ 
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */ 
}
</#if>





<#-- Only for the TERATERM usage -->
<#if MC.SIX_STEP_COMMUNICATION_IF == "TERATERM_IF"><#-- TERATERM I/F usage -->
/* This section is present only when TeraTerm is used for 6-steps */
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  */
void USART_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */

  HAL_UART_IRQHandler(UI_Params.pUART);

  /* USER CODE BEGIN USART2_IRQn 1 */
  MC_Com_ProcessInput();
  /* USER CODE END USART2_IRQn 1 */
}
</#if>

/**
  * @brief This function handles System tick timer.
  * @param  None
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  HAL_IncTick();

  /* USER CODE BEGIN SysTick_IRQn 1 */
  HAL_SYSTICK_IRQHandler();
  /* USER CODE END SysTick_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  * @param  None
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */
  Motor_Device1.status = MC_OVERCURRENT;
  MC_Core_LL_Error(&Motor_Device1);
  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */

  HAL_TIM_IRQHandler(&htim1);

  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

<#if MC.SIX_STEP_COMMUNICATION_IF == "PWM_IF"><#-- PWM I/F usage -->
/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  <#if MC.SIX_STEP_SENSING == "SENSORS_LESS"><#-- Sensorless usage -->
  /* PWM INTERFACE BEGIN 1 */
  MC_Core_LL_PwmInterfaceIrqHandler((uint32_t *) &htim2);
  /* PWM INTERFACE END 1 */
  </#if>
  /* USER CODE END TIM2_IRQn 0 */

  <#if MC.SIX_STEP_SENSING == "HALL_SENSORS"><#-- Hall Sensors usage -->
  HAL_TIM_IRQHandler(&htim2);
  </#if>

  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  <#if MC.SIX_STEP_SENSING == "HALL_SENSORS"><#-- Hall Sensors usage -->
  /* PWM INTERFACE BEGIN 1 */
  MC_Core_LL_PwmInterfaceIrqHandler((uint32_t *) &htim4);
  /* PWM INTERFACE END 1 */
  </#if>
  /* USER CODE END TIM4_IRQn 0 */

  <#if MC.SIX_STEP_SENSING == "SENSORS_LESS"><#-- Sensorless usage -->
  HAL_TIM_IRQHandler(&htim4);
  </#if>

  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}
</#if>

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
