<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
/**
  ******************************************************************************
  * @file    stm32c0xx_mc_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control
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
  * @ingroup STM32C0xx_IRQ_Handlers
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mc_type.h"
#include "mc_tasks.h"
#include "motorcontrol.h"
#include "parameters_conversion.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32C0xx_IRQ_Handlers STM32C0xx IRQ Handlers
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


/* Public prototypes of IRQ handlers called from assembly code ---------------*/
${IRQHandler_name("DMA", "Channel1")};
${IRQHandler_name(_last_word(MC.M1_PWM_TIMER_SELECTION), "UP")};
<#if (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR")>
${IRQHandler_name(_last_word(MC.M1_HALL_TIMER_SELECTION))};
</#if><#-- (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR") -->
<#if M1_HALL_SENSOR == true>
${IRQHandler_name(_last_word(MC.M1_HALL_TIMER_SELECTION))};
</#if>

/**
  * @brief  This function handles current regulation interrupt request.
  * @param  None
  */
${IRQHandler_name("DMA", "Channel1")}
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */
  
  /* Clear Flags */
  DMA1->IFCR = (LL_DMA_ISR_GIF1|LL_DMA_ISR_TCIF1|LL_DMA_ISR_HTIF1);

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 1 */

  /* USER CODE END CURRENT_REGULATION_IRQn 1 */   
  
    TSK_HighFrequencyTask();

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */   
}

/**
  * @brief  This function handles first motor TIMx Update, Break-in interrupt request.
  * @param  None
  */
${IRQHandler_name(_last_word(MC.M1_PWM_TIMER_SELECTION), "UP")}
{
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 0 */   

  if(LL_TIM_IsActiveFlag_UPDATE(${_last_word(MC.M1_PWM_TIMER_SELECTION)}) && LL_TIM_IsEnabledIT_UPDATE(${_last_word(MC.M1_PWM_TIMER_SELECTION)}))
  {
    LL_TIM_ClearFlag_UPDATE(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
<#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>  
    R1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
<#elseif ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && (MC.M1_CS_ADC_NUM == '1'))>
    R3_1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
</#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) --> 

    /* USER CODE BEGIN PWM_Update */

    /* USER CODE END PWM_Update */  
  }
  else
  {
    /* Nothing to do */
  }

  if(LL_TIM_IsActiveFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)}) && LL_TIM_IsEnabledIT_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)})) 
  {
    LL_TIM_ClearFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    <#if (MC.M1_OCP_TOPOLOGY != "NONE") &&  (MC.M1_OCP_DESTINATION == "TIM_BKIN")>
    PWMC_OCP_Handler(&PWM_Handle_M1._Super);
    <#elseif (MC.M1_DP_TOPOLOGY != "NONE") &&  (MC.M1_DP_DESTINATION == "TIM_BKIN")>
    PWMC_DP_Handler(&PWM_Handle_M1._Super);
    <#else>
    PWMC_OVP_Handler(&PWM_Handle_M1._Super, ${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    </#if>

    /* USER CODE BEGIN Break */

    /* USER CODE END Break */ 
  }
  else
  {
    /* Nothing to do */
  }
  
  if (LL_TIM_IsActiveFlag_BRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)}) && LL_TIM_IsEnabledIT_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)})) 
  {
    LL_TIM_ClearFlag_BRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    <#if (MC.M1_OCP_TOPOLOGY != "NONE") &&  (MC.M1_OCP_DESTINATION == "TIM_BKIN2")>
    PWMC_OCP_Handler(&PWM_Handle_M1._Super);
    <#elseif (MC.M1_DP_TOPOLOGY != "NONE") &&  (MC.M1_DP_DESTINATION == "TIM_BKIN2")>
    PWMC_DP_Handler(&PWM_Handle_M1._Super);
    <#else>
    PWMC_OVP_Handler(&PWM_Handle_M1._Super, ${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    </#if>


    /* USER CODE BEGIN Break */

    /* USER CODE END Break */ 
  }
  else 
  {
    /* No other interrupts are routed to this handler */
  }

  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 1 */   
}

<#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>     
/**
  * @brief  This function handles first motor DMAx TC interrupt request. 
  *         Required only for R1 with rep rate > 1
  * @param  None
  */
${IRQHandler_name("DMA", "Channel2")}
{
  /* USER CODE BEGIN DMAx_R1_M1_IRQn 0 */

  /* USER CODE END DMAx_R1_M1_IRQn 0 */ 

  if (LL_DMA_IsActiveFlag_HT2(DMA1) && LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_2))
  {
    R1_DMAx_HT_IRQHandler(&PWM_Handle_M1);
    LL_DMA_ClearFlag_HT2(DMA1);

    /* USER CODE BEGIN DMAx_R1_M1_HT2 */

    /* USER CODE END DMAx_R1_M1_HT2 */     
  } 
  else
  {
    /* Nothing to do */
  }
  
  if (LL_DMA_IsActiveFlag_TC2(DMA1))
  {
    LL_DMA_ClearFlag_TC2(DMA1);   
    R1_DMAx_TC_IRQHandler(&PWM_Handle_M1);

    /* USER CODE BEGIN DMAx_R1_M1_TC2 */

    /* USER CODE END DMAx_R1_M1_TC2 */     
  }    
  else
  {
    /* Nothing to do */
  }
        
  /* USER CODE BEGIN DMAx_R1_M1_IRQn 1 */

  /* USER CODE END DMAx_R1_M1_IRQn 1 */ 
}
</#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->

<#if (M1_HALL_SENSOR == true)>
/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  */
${IRQHandler_name(SensorTimer)}
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */
  /* USER CODE END SPD_TIM_M1_IRQn 0 */ 
  
  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx) != 0)
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    HALL_TIMx_UP_IRQHandler(&HALL_M1);

    /* USER CODE BEGIN HALL_Update */

    /* USER CODE END HALL_Update   */ 
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

    /* USER CODE BEGIN HALL_CC1 */

    /* USER CODE END HALL_CC1 */ 
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */ 
}
</#if><#-- (M1_HALL_SENSOR == true) -->

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

