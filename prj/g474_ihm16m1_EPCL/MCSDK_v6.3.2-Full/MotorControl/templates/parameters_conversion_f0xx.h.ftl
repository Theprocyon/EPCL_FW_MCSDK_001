<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
/**
  ******************************************************************************
  * @file    parameters_conversion_f0xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32F0 Family.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_F0XX_H
#define __PARAMETERS_CONVERSION_F0XX_H

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ                      ${SYSCLKFreq}uL
#define TIM_CLOCK_DIVIDER                1
#define ADV_TIM_CLK_MHz                  ${(SYSCLKFreq/(1000000))?floor}
#define ADC_CLK_MHz                      14uL /* Maximum ADC Clock Frequency expressed in MHz */
#define HALL_TIM_CLK                     ${SYSCLKFreq}uL
#define ADC1_2                           ADC1
#define REF_TIM_CLK                      ${SYSCLKFreq}uL
#define REF_TIM_CLK_MHz                  ${(SYSCLKFreq/(1000000))?floor}

/*************************  IRQ Handler Mapping  *********************/  
<#if FOC>                            
#define CURRENT_REGULATION_IRQHandler    DMA1_Channel1_IRQHandler
#define DMAx_R1_M1_IRQHandler            DMA1_Channel4_5_IRQHandler
</#if>
<#if SIX_STEP && MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
#define BEMF_READING_IRQHandler          ADC1_COMP_IRQHandler
  <#if MC.LF_TIMER_SELECTION == 'LF_TIM2'>
#define PERIOD_COMM_IRQHandler           TIM2_IRQHandler
  <#elseif MC.LF_TIMER_SELECTION == 'LF_TIM4'>
#define PERIOD_COMM_IRQHandler           TIM4_IRQHandler
  <#elseif MC.LF_TIMER_SELECTION == 'LF_TIM3'>
#define PERIOD_COMM_IRQHandler           TIM3_IRQHandler
  <#elseif MC.LF_TIMER_SELECTION == 'LF_TIM16'>
#define PERIOD_COMM_IRQHandler           TIM16_IRQHandler
  </#if>
</#if>
#define TIMx_UP_BRK_M1_IRQHandler        TIM1_BRK_UP_TRG_COM_IRQHandler

<#if FOC>  
/**********  AUXILIARY TIMER (SINGLE SHUNT) *************/
  <#if CondMcu_STM32F051xxx || CondMcu_STM32F072xxx >
#define R1_PWM_AUX_TIM                   TIM15
  <#elseif CondMcu_STM32F050xxx || CondMcu_STM32F030xxx || CondMcu_STM32F031xxx > <#-- Also needed for STSPIN -->
#define R1_PWM_AUX_TIM                   TIM3
  <#else>
#error "Invalid configuration: MCU definition in parameters_conversion.h"
  </#if>
</#if>

/* Referred to Maximum value indicated in the Datasheet Table 50 if ADC clock = HSI14 converted in number of cycle*/ 
/* 4 Cycles at 14Mhz = 285 ns - Table mentions 259 ns  */
#define ADC_TRIG_CONV_LATENCY_CYCLES     4
#define ADC_SAR_CYCLES                   12.5
#define M1_VBUS_SW_FILTER_BW_FACTOR      10u

#endif /*__PARAMETERS_CONVERSION_F0XX_H*/

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
