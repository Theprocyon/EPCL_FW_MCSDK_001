<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
/**
  ******************************************************************************
  * @file    parameters_conversion_g4xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32G4 Family.
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
#ifndef PARAMETERS_CONVERSION_G4XX_H
#define PARAMETERS_CONVERSION_G4XX_H

#include "drive_parameters.h"
#include "power_stage_parameters.h"
#include "mc_math.h"

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ                      ${SYSCLKFreq}uL
#define TIM_CLOCK_DIVIDER                ${TimerDivider} 
#define ADV_TIM_CLK_MHz                  ${(ADV_TIM_CLKFreq/(1000000*TimerDivider))?floor} /* Actual TIM clk including Timer clock divider*/
#define ADC_CLK_MHz                      ${(ADC_TIM_CLKFreq/1000000)?floor}
#define HALL_TIM_CLK                     ${APB1Tim_CLKFreq}uL
#define APB1TIM_FREQ                     ${APB1Tim_CLKFreq}uL

 <#if MC.DRIVE_NUMBER != "1">
#define TIM_CLOCK_DIVIDER2               ${TimerDivider2}
#define ADV_TIM_CLK_MHz2                 ${(ADV_TIM_CLKFreq2/(1000000*TimerDivider2))?floor} /* Actual TIM clk including Timer clock divider*/
#define ADC_CLK_MHz2                     ${(ADC_TIM_CLKFreq2/1000000)?floor}
#define HALL_TIM_CLK2                    ${APB1Tim_CLKFreq}uL
 </#if>

/*************************  IRQ Handler Mapping  *********************/
<#if MC.M1_PWM_TIMER_SELECTION == 'PWM_TIM1' || MC.M1_PWM_TIMER_SELECTION == 'TIM1'>
#define TIMx_UP_M1_IRQHandler            TIM1_UP_TIM16_IRQHandler

#define TIMx_BRK_M1_IRQHandler           TIM1_BRK_TIM15_IRQHandler
</#if>
<#if MC.M1_PWM_TIMER_SELECTION == 'PWM_TIM8' || MC.M1_PWM_TIMER_SELECTION == 'TIM8'>
#define TIMx_UP_M1_IRQHandler            TIM8_UP_IRQHandler
#define TIMx_BRK_M1_IRQHandler           TIM8_BRK_IRQHandler
#define MC1USETIM8
</#if>
<#if SIX_STEP && MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
  <#if MC.PHASE_U_BEMF_ADC == "ADC1" || MC.PHASE_V_BEMF_ADC == "ADC1" || MC.PHASE_W_BEMF_ADC == "ADC1" || MC.PHASE_U_BEMF_ADC == "ADC2" || MC.PHASE_V_BEMF_ADC == "ADC2" || MC.PHASE_W_BEMF_ADC == "ADC2">
#define BEMF_ADC1_2_READING_IRQHandler   ADC1_2_IRQHandler
  </#if> <#-- ADC1 || ADC2-->
  <#if MC.PHASE_U_BEMF_ADC == "ADC3" || MC.PHASE_V_BEMF_ADC == "ADC3" || MC.PHASE_W_BEMF_ADC == "ADC3">
#define BEMF_ADC3_READING_IRQHandler     ADC3_IRQHandler
  </#if> <#-- ADC3-->
  <#if MC.PHASE_U_BEMF_ADC == "ADC4" || MC.PHASE_V_BEMF_ADC == "ADC4" || MC.PHASE_W_BEMF_ADC == "ADC4">
#define BEMF_ADC4_READING_IRQHandler     ADC4_IRQHandler
  </#if> <#-- ADC4-->
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
<#if FOC>
  <#if MC.DRIVE_NUMBER != "1">
    <#if MC.M2_PWM_TIMER_SELECTION == 'PWM_TIM1' || MC.M2_PWM_TIMER_SELECTION == 'TIM1'>
#define TIMx_UP_M2_IRQHandler            TIM1_UP_TIM16_IRQHandler
#define TIMx_BRK_M2_IRQHandler           TIM1_BRK_TIM15_IRQHandler
    </#if>
    <#if MC.M2_PWM_TIMER_SELECTION == 'PWM_TIM8' || MC.M2_PWM_TIMER_SELECTION == 'TIM8'>
#define TIMx_UP_M2_IRQHandler            TIM8_UP_IRQHandler
#define TIMx_BRK_M2_IRQHandler           TIM8_BRK_IRQHandler
    </#if>
  </#if>
</#if>

#define ADC_TRIG_CONV_LATENCY_CYCLES     3.5
#define ADC_SAR_CYCLES                   12.5 <#-- work around 30 for one shunt phase shift todo -->

#define M1_VBUS_SW_FILTER_BW_FACTOR      6u
<#if MC.DRIVE_NUMBER != "1">
#define M2_VBUS_SW_FILTER_BW_FACTOR      6u
</#if>
<#if MC.ESC_ENABLE >
/* ESC parameters conversion */
/* 1060 us ON time is the minimum value to spin the motor*/
/* 1860 us ON time is the target for the maximum speed */
#define ESC_TON_MIN                      (APB1TIM_FREQ / 1000000 * 1060 )
#define ESC_TON_MAX                      (APB1TIM_FREQ / 1000000 * 1860 )
/* Arming : if 800 us <= TOn < 1060 us for 200 us, ESC is armed */
#define ESC_TON_ARMING                   ((APB1TIM_FREQ /1000000) * 800 )
</#if>
<#if FOC>
#define OPAMP1_InvertingInput_PC5        LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP1_InvertingInput_PA3        LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP1_InvertingInput_PGA        LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP1_InvertingInput_FOLLOWER   LL_OPAMP_MODE_FOLLOWER
#define OPAMP2_InvertingInput_PC5        LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP2_InvertingInput_PA5        LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP2_InvertingInput_PGA        LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP2_InvertingInput_FOLLOWER   LL_OPAMP_MODE_FOLLOWER
#define OPAMP3_InvertingInput_PB10       LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP3_InvertingInput_PB2        LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP3_InvertingInput_PGA        LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP3_InvertingInput_FOLLOWER   LL_OPAMP_MODE_FOLLOWER
#define OPAMP4_InvertingInput_PB10       LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP4_InvertingInput_PD8        LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP4_InvertingInput_PGA        LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP4_InvertingInput_FOLLOWER   LL_OPAMP_MODE_FOLLOWER

#define OPAMP1_NonInvertingInput_PA1     LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP1_NonInvertingInput_PA3     LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP1_NonInvertingInput_PA7     LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP2_NonInvertingInput_PA7     LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP2_NonInvertingInput_PB14    LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP2_NonInvertingInput_PB0     LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP2_NonInvertingInput_PD14    LL_OPAMP_INPUT_NONINVERT_IO3
#define OPAMP3_NonInvertingInput_PB0     LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP3_NonInvertingInput_PB13    LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP3_NonInvertingInput_PA1     LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP4_NonInvertingInput_PB13    LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP4_NonInvertingInput_PD11    LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP4_NonInvertingInput_PB11    LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP5_NonInvertingInput_PB14    LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP5_NonInvertingInput_PD12    LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP5_NonInvertingInput_PC3     LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP6_NonInvertingInput_PB12    LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP6_NonInvertingInput_PD9     LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP6_NonInvertingInput_PB13    LL_OPAMP_INPUT_NONINVERT_IO2

#define OPAMP1_PGAConnect_PC5            OPAMP_CSR_PGGAIN_3
#define OPAMP1_PGAConnect_PA3            (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)
#define OPAMP2_PGAConnect_PC5            OPAMP_CSR_PGGAIN_3
#define OPAMP2_PGAConnect_PA5            (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)
#define OPAMP3_PGAConnect_PB10           OPAMP_CSR_PGGAIN_3
#define OPAMP3_PGAConnect_PB2            (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)
#define OPAMP4_PGAConnect_PB10           OPAMP_CSR_PGGAIN_3
#define OPAMP4_PGAConnect_PD8            (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)

#define COMP1_InvertingInput_PA0         LL_COMP_INPUT_MINUS_IO1
#define COMP2_InvertingInput_PA2         LL_COMP_INPUT_MINUS_IO1
#define COMP3_InvertingInput_PD15        LL_COMP_INPUT_MINUS_IO1
#define COMP3_InvertingInput_PB12        LL_COMP_INPUT_MINUS_IO2
#define COMP4_InvertingInput_PE8         LL_COMP_INPUT_MINUS_IO1
#define COMP4_InvertingInput_PB2         LL_COMP_INPUT_MINUS_IO2
#define COMP5_InvertingInput_PD13        LL_COMP_INPUT_MINUS_IO1
#define COMP5_InvertingInput_PB10        LL_COMP_INPUT_MINUS_IO2
#define COMP6_InvertingInput_PD10        LL_COMP_INPUT_MINUS_IO1
#define COMP6_InvertingInput_PB15        LL_COMP_INPUT_MINUS_IO2
#define COMP7_InvertingInput_PC0         LL_COMP_INPUT_MINUS_IO1

#define COMPX_InvertingInput_DAC1        LL_COMP_INPUT_MINUS_DAC1_CH1
#define COMPX_InvertingInput_DAC2        LL_COMP_INPUT_MINUS_DAC1_CH2
#define COMPX_InvertingInput_VREF        LL_COMP_INPUT_MINUS_VREFINT
#define COMPX_InvertingInput_VREF_1_4    LL_COMP_INPUT_MINUS_1_4VREFINT
#define COMPX_InvertingInput_VREF_1_2    LL_COMP_INPUT_MINUS_1_2VREFINT
#define COMPX_InvertingInput_VREF_3_4    LL_COMP_INPUT_MINUS_3_4VREFINT

#define COMP1_NonInvertingInput_PA1      LL_COMP_INPUT_PLUS_IO1
#define COMP2_NonInvertingInput_PA3      LL_COMP_INPUT_PLUS_IO2
#define COMP2_NonInvertingInput_PA7      LL_COMP_INPUT_PLUS_IO1
#define COMP3_NonInvertingInput_PB14     LL_COMP_INPUT_PLUS_IO1
#define COMP3_NonInvertingInput_PD14     LL_COMP_INPUT_PLUS_IO2
#define COMP4_NonInvertingInput_PB0      LL_COMP_INPUT_PLUS_IO1
#define COMP4_NonInvertingInput_PE7      LL_COMP_INPUT_PLUS_IO2
#define COMP5_NonInvertingInput_PB13     LL_COMP_INPUT_PLUS_IO2
#define COMP5_NonInvertingInput_PD12     LL_COMP_INPUT_PLUS_IO1
#define COMP6_NonInvertingInput_PB11     LL_COMP_INPUT_PLUS_IO2
#define COMP6_NonInvertingInput_PD11     LL_COMP_INPUT_PLUS_IO1
#define COMP7_NonInvertingInput_PC1      LL_COMP_INPUT_PLUS_IO2
#define COMP7_NonInvertingInput_PA0      LL_COMP_INPUT_PLUS_IO1
</#if>

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */  

#endif /*PARAMETERS_CONVERSION_G4XX_H*/

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
