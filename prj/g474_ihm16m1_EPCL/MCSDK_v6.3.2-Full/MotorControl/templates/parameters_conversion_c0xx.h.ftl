<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
/**
  ******************************************************************************
  * @file    parameters_conversion_c0xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32C0 Family.
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
#ifndef __PARAMETERS_CONVERSION_C0XX_H
#define __PARAMETERS_CONVERSION_C0XX_H

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ                      ${SYSCLKFreq}uL
#define TIM_CLOCK_DIVIDER                1 
#define ADV_TIM_CLK_MHz                  ${(SYSCLKFreq/(1000000))?floor}
#define ADC_CLK_MHz                      35uL /* Maximum ADC Clock Frequency expressed in MHz */
#define HALL_TIM_CLK                     ${SYSCLKFreq}uL
#define ADC1_2                           ADC1

/*************************  IRQ Handler Mapping  *********************/
<#if FOC >
#define CURRENT_REGULATION_IRQHandler     DMA1_Channel1_IRQHandler
</#if>
<#if SIX_STEP && MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
#define BEMF_READING_IRQHandler             ADC1_IRQHandler
  <#if MC.LF_TIMER_SELECTION == 'LF_TIM2'>
#define PERIOD_COMM_IRQHandler            TIM2_IRQHandler
  <#elseif MC.LF_TIMER_SELECTION == 'LF_TIM4'>
#define PERIOD_COMM_IRQHandler            TIM4_IRQHandler
  <#elseif MC.LF_TIMER_SELECTION == 'LF_TIM3'>
#define PERIOD_COMM_IRQHandler            TIM3_IRQHandler
  <#elseif MC.LF_TIMER_SELECTION == 'LF_TIM16'>
#define PERIOD_COMM_IRQHandler            TIM16_IRQHandler
  </#if>  
</#if>
#define TIMx_UP_BRK_M1_IRQHandler        TIM1_BRK_UP_TRG_COM_IRQHandler

/*************************  ADC Physical characteristics  ************/
#define ADC_TRIG_CONV_LATENCY_CYCLES      3
#define ADC_SAR_CYCLES                    12.5 
#define M1_VBUS_SW_FILTER_BW_FACTOR       10u

#endif /*__PARAMETERS_CONVERSION_C0XX_H*/

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
