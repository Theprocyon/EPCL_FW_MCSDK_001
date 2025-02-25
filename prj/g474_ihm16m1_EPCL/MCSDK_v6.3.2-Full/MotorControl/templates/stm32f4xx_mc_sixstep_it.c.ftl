<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
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
#include "mc_config.h"
#include "mc_type.h"
//cstat -MISRAC2012-Rule-3.1
#include "mc_tasks.h"
//cstat +MISRAC2012-Rule-3.1
#include "parameters_conversion.h"
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

void PERIOD_COMM_IRQHandler(void);
void BEMF_READING_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
<#if (M1_HALL_SENSOR == true)>
void SPD_TIM_M1_IRQHandler(void);
</#if><#-- (M1_HALL_SENSOR == true) -->

  <#function _last_word text sep="_"><#return text?split(sep)?last></#function>
  <#function _last_char text><#return text[text?length-1]></#function>


<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
/**
  * @brief  This function handles BEMF sensing interrupt request.
  * @param[in] None
  */
void BEMF_READING_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */

  <#if MC.PHASE_U_BEMF_ADC == "ADC1" || MC.PHASE_V_BEMF_ADC == "ADC1" || MC.PHASE_W_BEMF_ADC == "ADC1">
  if(LL_ADC_IsActiveFlag_AWD1(ADC1) && LL_ADC_IsEnabledIT_AWD1(ADC1))
  {
    /* Clear Flags */
    LL_ADC_ClearFlag_AWD1(ADC1);
    BADC_IsZcDetected(&Bemf_ADC_M1, PWM_Handle_M1.Step);
  }
  else
  {
    /* Nothing to do */
  }
  </#if><#-- MC.PHASE_U_BEMF_ADC == "ADC1" || MC.PHASE_V_BEMF_ADC == "ADC1" || MC.PHASE_W_BEMF_ADC == "ADC1" -->

  <#if MC.PHASE_U_BEMF_ADC == "ADC2" || MC.PHASE_V_BEMF_ADC == "ADC2" || MC.PHASE_W_BEMF_ADC == "ADC2">
  if(LL_ADC_IsActiveFlag_AWD1(ADC2) && LL_ADC_IsEnabledIT_AWD1(ADC2))
  {
    /* Clear Flags */
    LL_ADC_ClearFlag_AWD1(ADC2);
    BADC_IsZcDetected(&Bemf_ADC_M1, PWM_Handle_M1.Step);
  }
  else
  {
    /* Nothing to do */
  }
  </#if><#-- MC.PHASE_U_BEMF_ADC == "ADC2" || MC.PHASE_V_BEMF_ADC == "ADC2" || MC.PHASE_W_BEMF_ADC == "ADC2" -->

  <#if MC.PHASE_U_BEMF_ADC == "ADC3" || MC.PHASE_V_BEMF_ADC == "ADC3" || MC.PHASE_W_BEMF_ADC == "ADC3">
  if(LL_ADC_IsActiveFlag_AWD1(ADC3) && LL_ADC_IsEnabledIT_AWD1(ADC3))
  {
    /* Clear Flags */
    LL_ADC_ClearFlag_AWD1(ADC3);
    BADC_IsZcDetected(&Bemf_ADC_M1, PWM_Handle_M1.Step);
  }
  else
  {
    /* Nothing to do */
  }
  </#if><#-- MC.PHASE_U_BEMF_ADC == "ADC3" || MC.PHASE_V_BEMF_ADC == "ADC3" || MC.PHASE_W_BEMF_ADC == "ADC3" -->

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 1 */

  /* USER CODE END CURRENT_REGULATION_IRQn 1 */

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */
}

/**
  * @brief     LFtimer interrupt handler.
  * @param[in] None
  */
void PERIOD_COMM_IRQHandler(void)
{
  /* TIM Update event */

  if(LL_TIM_IsActiveFlag_CC1(Bemf_ADC_M1.pParams_str->LfTim) && LL_TIM_IsEnabledIT_CC1(Bemf_ADC_M1.pParams_str->LfTim))
  {
    LL_TIM_ClearFlag_CC1(Bemf_ADC_M1.pParams_str->LfTim);
    if ((Bemf_ADC_M1.SpeedTimerState == COMMUTATION) && (true == Bemf_ADC_M1.IsLoopClosed))
    {
      BADC_StepChangeEvent(&Bemf_ADC_M1, 0);
      SixStep_StepCommution();
    }
    else if (Bemf_ADC_M1.SpeedTimerState == DEMAGNETIZATION)
    {
      PWMC_SetADCTriggerChannel( &PWM_Handle_M1, *Bemf_ADC_M1.pSensing_Point);
      BADC_Start(&Bemf_ADC_M1, PWM_Handle_M1.Step, PWM_Handle_M1.LSModArray);
      Bemf_ADC_M1.SpeedTimerState = COMMUTATION;
    }
  }
  else
  {
    /* Nothing to do */
  }
}
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->

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
  if (0U == LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx))
  {
    /* Nothing to do */
  }
  else
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    (void)HALL_TIMx_UP_IRQHandler(&HALL_M1);
    
    /* USER CODE BEGIN M1 HALL_Update */

    /* USER CODE END M1 HALL_Update   */
  }

  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
  if (LL_TIM_IsActiveFlag_CC1 (HALL_M1.TIMx) != 0U) 
  {
    LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
    (void)HALL_TIMx_CC_IRQHandler(&HALL_M1);
    
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
</#if><#-- (M1_HALL_SENSOR == true) -->

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
 
  LL_TIM_ClearFlag_UPDATE(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
  (void)TSK_HighFrequencyTask();

 /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

 /* USER CODE END  TIMx_UP_M1_IRQn 1 */
}


void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */

  if (0U == LL_TIM_IsActiveFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)}))
  {
    /* Nothing to do */
  }
  else
  {
    LL_TIM_ClearFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    PWMC_BRK_IRQHandler(&PWM_Handle_M1);
  }
  
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here */
  MC_RunMotorControlTasks();

  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */
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
