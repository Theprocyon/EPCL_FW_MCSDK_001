<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/foc_assign.ftl">

/**
  ******************************************************************************
  * @file    stm32f4xx_mc_it.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt 
  *          service routine related to Motor Control for the STM32F4 Family.
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
  * @ingroup STM32F4xx_IRQ_Handlers
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_config.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "mcp_config.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32F4xx_IRQ_Handlers STM32F4xx IRQ Handlers
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
void ADC_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
<#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) >
void DMAx_R1_M1_IRQHandler(void);  
</#if>
<#if (M1_HALL_SENSOR == true)>
void SPD_TIM_M1_IRQHandler(void);
</#if>
<#if MC.DRIVE_NUMBER != "1">
void TIMx_UP_M2_IRQHandler(void);
void TIMx_BRK_M2_IRQHandler(void);
  <#if (M2_HALL_SENSOR == true)>
void SPD_TIM_M2_IRQHandler(void);
  </#if>
  <#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
void DMAx_R1_M2_IRQHandler(void);  
  </#if>
</#if>

/**
  * @brief  This function handles ADC interrupt request.
  * @param  None
  */

  
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */

  /* Shared IRQ management - begin */
  <#if HIGH_FREQ_TRIGGER == "TIM_UP_IT">

  if(LL_ADC_IsActiveFlag_JEOS(ADC1))
  {
    /* Clear Flags */
    ${M1_ADC}->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);
    TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
  }
  else
  {
    /* Nothing to do */
  }
  <#else><#-- HIGH_FREQ_TRIGGER == "ADC_IT"-->
  if (LL_ADC_IsActiveFlag_JEOS(${M1_ADC}))
  {
    /* Clear Flags M1 */
    ${M1_ADC}->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);
    TSK_HighFrequencyTask(M1);
  }
  else if (LL_ADC_IsActiveFlag_JEOS(${M2_ADC}))
  {
    /* Clear Flags M2 */
    ${M2_ADC}->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);
    TSK_HighFrequencyTask(M2);
  }
  </#if><#-- HIGH_FREQ_TRIGGER == "TIM_UP_IT"-->

  /* USER CODE BEGIN HighFreq */

  /* USER CODE END HighFreq  */  
 
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  */
void TIMx_UP_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_M1_IRQn 0 */
  
  LL_TIM_ClearFlag_UPDATE(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
<#if ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && (MC.M1_CS_ADC_NUM == '2'))>
  <#if MC.DRIVE_NUMBER == "1">
  R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);
  <#else><#-- MC.DRIVE_NUMBER != 1 -->
  R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);
  </#if><#-- MC.DRIVE_NUMBER == 1 -->
<#elseif ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && (MC.M1_CS_ADC_NUM == '1'))>
  R3_1_TIMx_UP_IRQHandler(&PWM_Handle_M1);
<#elseif (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
  ICS_TIMx_UP_IRQHandler(&PWM_Handle_M1);
<#elseif ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>      
  <#if MC.DRIVE_NUMBER == "1">
    <#if (MC.M1_PWM_TIMER_SELECTION == 'PWM_TIM1') || (MC.M1_PWM_TIMER_SELECTION == 'TIM1')>
  R1_TIM1_UP_IRQHandler(&PWM_Handle_M1);
    <#elseif (MC.M1_PWM_TIMER_SELECTION == 'PWM_TIM8') || (MC.M1_PWM_TIMER_SELECTION == 'TIM8')>
  R1_TIM8_UP_IRQHandler(&PWM_Handle_M1);
    </#if>
  <#else><#-- MC.DRIVE_NUMBER != 1 -->
    <#if (MC.M1_PWM_TIMER_SELECTION == 'PWM_TIM1') || (MC.M1_PWM_TIMER_SELECTION == 'TIM1')>
  R1_TIM1_UP_IRQHandler(&PWM_Handle_M1);
    <#elseif (MC.M1_PWM_TIMER_SELECTION == 'PWM_TIM8') || (MC.M1_PWM_TIMER_SELECTION == 'TIM8')>
  R1_TIM8_UP_IRQHandler(&PWM_Handle_M1);
    </#if>
  </#if><#-- MC.DRIVE_NUMBER == 1 -->
</#if>
<#if MC.DRIVE_NUMBER != "1">
  TSK_DualDriveFIFOUpdate( M1 );
</#if>

  /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_M1_IRQn 1 */  
}

 <#if MC.DRIVE_NUMBER != "1">
/**
  * @brief  This function handles second motor TIMx Update interrupt request.
  * @param  None
  */
void TIMx_UP_M2_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_M2_IRQn 0 */

  /* USER CODE END TIMx_UP_M2_IRQn 0 */

  LL_TIM_ClearFlag_UPDATE(${_last_word(MC.M2_PWM_TIMER_SELECTION)});
  <#if ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>  
    <#if MC.DRIVE_NUMBER == "1">
  R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M2);
    <#else><#-- MC.DRIVE_NUMBER != 1 -->
  R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M2);
    </#if><#-- MC.DRIVE_NUMBER == 1 -->
  <#elseif MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
  ICS_TIMx_UP_IRQHandler(&PWM_Handle_M2);
  <#elseif ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
    <#if MC.DRIVE_NUMBER == "1">
     <#-- Any chance this can ever be valid? -->
      <#if (MC.M2_PWM_TIMER_SELECTION == 'PWM_TIM1') || (MC.M2_PWM_TIMER_SELECTION == 'TIM1')>
  R1_TIM1_UP_IRQHandler(&PWM_Handle_M2);
      <#elseif (MC.M2_PWM_TIMER_SELECTION == 'PWM_TIM8') || (MC.M2_PWM_TIMER_SELECTION == 'TIM8')>
  R1_TIM8_UP_IRQHandler(&PWM_Handle_M2);
      </#if>
  <#else><#-- MC.DRIVE_NUMBER != 1 -->
      <#if (MC.M2_PWM_TIMER_SELECTION == 'PWM_TIM1') || (MC.M2_PWM_TIMER_SELECTION == 'TIM1')>
  R1_TIM1_UP_IRQHandler(&PWM_Handle_M2);
      <#elseif (MC.M2_PWM_TIMER_SELECTION == 'PWM_TIM8') || (MC.M2_PWM_TIMER_SELECTION == 'TIM8')>
  R1_TIM8_UP_IRQHandler(&PWM_Handle_M2);
      </#if>
    </#if><#-- MC.DRIVE_NUMBER == 1 -->
  </#if>
  TSK_DualDriveFIFOUpdate( M2 );

  /* USER CODE BEGIN TIMx_UP_M2_IRQn 1 */

  /* USER CODE END TIMx_UP_M2_IRQn 1 */ 
}
</#if>

<#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) >
/**
  * @brief  This function handles first motor DMAx TC interrupt request.
  *         Required only for R1 with rep rate > 1
  * @param  None
  */
void DMAx_R1_M1_IRQHandler(void)
{
  /* USER CODE BEGIN DMAx_R1_M1_IRQn 0 */

  /* USER CODE END DMAx_R1_M1_IRQn 0 */ 
  if (LL_DMA_IsActiveFlag_HT${Stream}(DMA2) && LL_DMA_IsEnabledIT_HT(DMA2, LL_DMA_STREAM_${Stream}))
  {
    R1_DMAx_HT_IRQHandler(&PWM_Handle_M1);
    LL_DMA_ClearFlag_HT${Stream}(DMA2);
  } 
  else
  {
    /* Nothing to do */
  }
  
  if (LL_DMA_IsActiveFlag_TC${Stream}(DMA2))
  {
    LL_DMA_ClearFlag_TC${Stream}(DMA2);
    R1_DMAx_TC_IRQHandler(&PWM_Handle_M1);
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN DMAx_R1_M1_IRQn 1 */

  /* USER CODE END DMAx_R1_M1_IRQn 1 */ 
}
</#if>

<#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
/**
  * @brief  This function handles second motor DMAx TC interrupt request.
  *         Required only for R1 with rep rate > 1
  * @param  None
  */
void DMAx_R1_M2_IRQHandler(void)
{
  /* USER CODE BEGIN DMAx_R1_M2_IRQn 0 */

  /* USER CODE END DMAx_R1_M2_IRQn 0 */

  if (LL_DMA_IsActiveFlag_HT${Stream2}(DMA2) && LL_DMA_IsEnabledIT_HT(DMA2, LL_DMA_STREAM_${Stream2}))
  {
    R1_DMAx_HT_IRQHandler(&PWM_Handle_M2);
    LL_DMA_ClearFlag_HT${Stream2}(DMA2);
  }
  else
  {
    /* Nothing to do */
  }

  if (LL_DMA_IsActiveFlag_TC${Stream2}(DMA2))
  {
    LL_DMA_ClearFlag_TC${Stream2}(DMA2);
    R1_DMAx_TC_IRQHandler(&PWM_Handle_M2);
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN DMAx_R1_M2_IRQn 1 */

  /* USER CODE END DMAx_R1_M2_IRQn 1 */
}
</#if>

/**
  * @brief  This function handles first motor BRK interrupt.
  * @param  None
  */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */ 
  if (LL_TIM_IsActiveFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)}))
  {
    LL_TIM_ClearFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
<#if (MC.M1_OCP_TOPOLOGY != "NONE") &&  (MC.M1_OCP_DESTINATION == "TIM_BKIN")>
    PWMC_OCP_Handler(&PWM_Handle_M1._Super);
<#elseif (MC.M1_DP_TOPOLOGY != "NONE") &&  (MC.M1_DP_DESTINATION == "TIM_BKIN")>
    PWMC_DP_Handler(&PWM_Handle_M1._Super);
<#else>
    PWMC_OVP_Handler(&PWM_Handle_M1._Super, ${_last_word(MC.M1_PWM_TIMER_SELECTION)});
</#if>
  }
  else
  {
    /* Nothing to do */
  }

  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_RunMotorControlTasks();
  
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */ 
}

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

<#if MC.DRIVE_NUMBER != "1">
/**
  * @brief  This function handles second motor BRK interrupt.
  * @param  None
  */
void TIMx_BRK_M2_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M2_IRQn 0 */

  /* USER CODE END TIMx_BRK_M2_IRQn 0 */

  if (LL_TIM_IsActiveFlag_BRK(${_last_word(MC.M2_PWM_TIMER_SELECTION)}))
  {
    LL_TIM_ClearFlag_BRK(${_last_word(MC.M2_PWM_TIMER_SELECTION)});
  <#if (MC.M2_OCP_TOPOLOGY != "NONE") &&  (MC.M2_OCP_DESTINATION == "TIM_BKIN")>
    PWMC_OCP_Handler(&PWM_Handle_M2._Super); 
  <#elseif (MC.M2_DP_TOPOLOGY != "NONE") &&  (MC.M2_DP_DESTINATION == "TIM_BKIN")>
    PWMC_DP_Handler(&PWM_Handle_M2._Super);
  <#else>
    PWMC_OVP_Handler(&PWM_Handle_M2._Super, ${_last_word(MC.M2_PWM_TIMER_SELECTION)});
  </#if>

    /* USER CODE BEGIN BRK */

    /* USER CODE END BRK */
  }
  else
  {
    /* Nothing to do */
  }

  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_RunMotorControlTasks();

  /* USER CODE BEGIN TIMx_BRK_M2_IRQn 1 */

  /* USER CODE END TIMx_BRK_M2_IRQn 1 */
}

  <#if (M2_HALL_SENSOR == true)>
/**
  * @brief  This function handles TIMx global interrupt request for M2 Speed Sensor.
  * @param  None
  */
void SPD_TIM_M2_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M2_IRQn 0 */

  /* USER CODE END SPD_TIM_M2_IRQn 0 */ 

  /* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if (LL_TIM_IsActiveFlag_UPDATE(HALL_M2.TIMx))
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M2.TIMx);
    HALL_TIMx_UP_IRQHandler(&HALL_M2);
    /* USER CODE BEGIN M2 HALL_Update */

    /* USER CODE END M2 HALL_Update   */ 
  }
  else
  {
    /* Nothing to do */
  }

  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M2.TIMx)) 
  {
    LL_TIM_ClearFlag_CC1(HALL_M2.TIMx);
    HALL_TIMx_CC_IRQHandler(&HALL_M2);
    /* USER CODE BEGIN M2 HALL_CC1 */

    /* USER CODE END M2 HALL_CC1 */ 
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN SPD_TIM_M2_IRQn 1 */

  /* USER CODE END SPD_TIM_M2_IRQn 1 */ 
}
  </#if>
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
