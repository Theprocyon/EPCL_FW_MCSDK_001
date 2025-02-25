<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/foc_assign.ftl">
<#include "*/ftl/ip_assign.ftl">
<#include "*/ftl/ip_fct.ftl">
<#include "*/ftl/ip_macro.ftl">
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the 
  *          configuration of the subsystem.
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


/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "parameters_conversion.h"
<#if CondFamily_STM32F4>
  <#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
#include "r3_1_f4xx_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
#include "r3_2_f4xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
  <#if (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
#include "ics_f4xx_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
</#if><#-- CondFamily_STM32F4 -->
<#if CondFamily_STM32F0>
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
#include "r3_f0xx_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if><#-- CondFamily_STM32F0 -->
<#if CondFamily_STM32F3><#-- CondFamily_STM32F3 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
#include "ics_f30x_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') ||  (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
#include "r3_2_f30x_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1'))
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1'))>
#include "r3_1_f30x_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
</#if><#-- CondFamily_STM32F3 --->
<#if CondFamily_STM32G4><#-- CondFamily_STM32G4 --->
  <#if MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST">
#include "r3_g4xx_pwm_curr_fdbk.h"
  <#else>
    <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
    </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
    <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
#include "r3_2_g4xx_pwm_curr_fdbk.h"
    </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
    <#if  ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1'))>
#include "r3_1_g4xx_pwm_curr_fdbk.h"
    </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1')) -->
    <#if (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
#include "ics_g4xx_pwm_curr_fdbk.h"
    </#if>
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
</#if><#-- CondFamily_STM32G4 --->
<#if CondFamily_STM32G0><#-- CondFamily_STM32G0 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
#include "r3_g0xx_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
</#if><#-- CondFamily_STM32G4 --->
<#if CondFamily_STM32C0><#-- CondFamily_STM32C0 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
#include "r3_c0xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
</#if><#-- CondFamily_STM32C0 --->
<#if CondFamily_STM32L4><#-- CondFamily_STM32L4 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
#include "ics_l4xx_pwm_curr_fdbk.h"
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2'))>
#include "r3_2_l4xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) -->
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
#include "r3_1_l4xx_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
</#if><#-- CondFamily_STM32L4 --->
<#if CondFamily_STM32F7><#-- CondFamily_STM32F7 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
#include "ics_f7xx_pwm_curr_fdbk.h"
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2'))>
#include "r3_2_f7xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) -->
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
#include "r3_1_f7xx_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
</#if><#-- CondFamily_STM32F7 --->
<#if CondFamily_STM32H7><#-- CondFamily_STM32H7 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#error " H7 Single shunt not supported yet "
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
#error " H7 ICS not supported yet "
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
#include "r3_2_h7xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
</#if><#-- CondFamily_STM32H7 --->
<#if CondFamily_STM32H5 > 
/* Include H5 */
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) >
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT' || MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT'>
    <#if MC.M1_CS_ADC_NUM=='1'>
      /* Include H5 three shunt 1 ADC */
#include "r3_1_h5xx_pwm_curr_fdbk.h"
    <#elseif MC.M1_CS_ADC_NUM=='2'>
      /* Include H5 three shunt 2 ADC */
#include "r3_2_h5xx_pwm_curr_fdbk.h"
    <#else>
#error h5 config not define
    </#if> <#-- MC.M1_CS_ADC_NUM=='1' -->
  </#if> <#-- MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT' || MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT' -->
</#if> <#-- CondFamily_STM32H5 -->
<#if MC.PFC_ENABLED == true>
#include "pfc.h"
</#if><#-- MC.PFC_ENABLED == true -->
<#if (MC.MOTOR_PROFILER == true) && (MC.M1_SPEED_SENSOR != "HSO"  && MC.M1_SPEED_SENSOR != "ZEST" )>
#include "mp_self_com_ctrl.h"
#include "mp_one_touch_tuning.h"
</#if><#-- MC.MOTOR_PROFILER == true -->
<#if MC.ESC_ENABLE>
#include "esc.h"
</#if><#-- MC.ESC_ENABLE -->

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */  

<#if MC.DRIVE_NUMBER == "1">
#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */
</#if><#-- MC.DRIVE_NUMBER == 1 -->

<#if CondFamily_STM32F0 || CondFamily_STM32G0 || CondFamily_STM32H5 || CondFamily_STM32C0 >
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
extern  PWMC_R3_1_Handle_t PWM_Handle_M1;
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
</#if><#-- CondFamily_STM32F0 || CondFamily_STM32G0 || CondFamily_STM32H5 || CondFamily_STM32C0 -->

<#if CondFamily_STM32F4>
  <#if ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))><#-- inside CondFamily_STM32F4 -->
/**
  * @brief  Current sensor parameters Motor 2 - three shunt
  */
const R3_2_Params_t R3_2_ParamsM2 =
{

/* Dual MC parameters --------------------------------------------------------*/
  .Tw                = MAX_TWAIT2,
  .bFreqRatio        = FREQ_RATIO,
  .bIsHigherFreqTim  = FREQ_RELATION2,

/* PWM generation parameters --------------------------------------------------*/
  .TIMx              = ${_last_word(MC.M2_PWM_TIMER_SELECTION)},
  .hDeadTime         = DEAD_TIME_COUNTS2,
  .RepetitionCounter = REP_COUNTER2,
  .hTafter           = TW_AFTER2,
  .hTbefore          = TW_BEFORE2,

   //cstat -MISRAC2012-Rule-12.1 -MISRAC2012-Rule-10.1_R6 
  .ADCConfig1 = {
                   (uint32_t)(${getADC("CFG",1,"PHASE_1",2)}U << ADC_JSQR_JSQ4_Pos)
                  ,(uint32_t)(${getADC("CFG",2,"PHASE_1",2)}U << ADC_JSQR_JSQ4_Pos)
                  ,(uint32_t)(${getADC("CFG",3,"PHASE_1",2)}U << ADC_JSQR_JSQ4_Pos)
                  ,(uint32_t)(${getADC("CFG",4,"PHASE_1",2)}U << ADC_JSQR_JSQ4_Pos)
                  ,(uint32_t)(${getADC("CFG",5,"PHASE_1",2)}U << ADC_JSQR_JSQ4_Pos)
                  ,(uint32_t)(${getADC("CFG",6,"PHASE_1",2)}U << ADC_JSQR_JSQ4_Pos)
                },
  .ADCConfig2 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos)
                 ,(uint32_t)(${getADC("CFG",2,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos)
                 ,(uint32_t)(${getADC("CFG",3,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos)
                 ,(uint32_t)(${getADC("CFG",4,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos)
                 ,(uint32_t)(${getADC("CFG",5,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos)
                 ,(uint32_t)(${getADC("CFG",6,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos)
                },
  .ADCDataReg1 = {
                   ${getADC("IP", 1,"PHASE_1",2)}
                  ,${getADC("IP", 2,"PHASE_1",2)}
                  ,${getADC("IP", 3,"PHASE_1",2)}
                  ,${getADC("IP", 4,"PHASE_1",2)}
                  ,${getADC("IP", 5,"PHASE_1",2)}
                  ,${getADC("IP", 6,"PHASE_1",2)}
                 },
  .ADCDataReg2 = {
                   ${getADC("IP", 1,"PHASE_2",2)}
                  ,${getADC("IP", 2,"PHASE_2",2)}
                  ,${getADC("IP", 3,"PHASE_2",2)}
                  ,${getADC("IP", 4,"PHASE_2",2)}
                  ,${getADC("IP", 5,"PHASE_2",2)}
                  ,${getADC("IP", 6,"PHASE_2",2)}
                 }
};
  </#if><#-- ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
 
   <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')><#-- inside CondFamily_STM32F4 -->
  /**
  * @brief  Current sensor parameters Motor 1 - three shunt - STM32F401x8
  */
const R3_1_Params_t R3_1_ParamsM1 =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx              = ${MC.M1_CS_ADC_U},

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .hTafter           = TW_AFTER,
  .hTbefore          = TW_BEFORE_R3_1,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,

  .ADCConfig = {
                 (uint32_t)(${getADC("CFG",1,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",1,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",2,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",2,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",3,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",3,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",4,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",4,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",5,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",5,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",6,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",6,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
               }
};
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->

  <#if (MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1')><#-- inside CondFamily_STM32F4 -->
  /**
  * @brief  Current sensor parameters Motor 2 - three shunt - STM32F401x8
  */
const R3_1_Params_t R3_1_ParamsM2 =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx              = ${MC.M2_CS_ADC_U},

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER2,
  .hTafter           = TW_AFTER2,
  .hTbefore          = TW_BEFORE_R3_1_2,
  .TIMx              = ${_last_word(MC.M2_PWM_TIMER_SELECTION)},
  .Tsampling         = (uint16_t)SAMPLING_TIME2,
  .Tcase2            = (uint16_t)SAMPLING_TIME2 + (uint16_t)TDEAD2 + (uint16_t)TRISE2,
  .Tcase3            = ((uint16_t)TDEAD2 + (uint16_t)TNOISE2 + (uint16_t)SAMPLING_TIME2) / 2u,

  .ADCConfig = {
                 (uint32_t)(${getADC("CFG",1,"PHASE_1",2)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",1,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",2,"PHASE_1",2)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",2,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",3,"PHASE_1",2)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",3,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",4,"PHASE_1",2)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",4,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",5,"PHASE_1",2)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",5,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",6,"PHASE_1",2)}U << ADC_JSQR_JSQ3_Pos)
                          | ${getADC("CFG",6,"PHASE_2",2)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
               }
};
  </#if><#-- (MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1') -->

  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2'))><#-- inside CondFamily_STM32F4 -->

/**
  * @brief  Current sensor parameters Motor 1 - three shunt
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
  .Tw                = MAX_TWAIT,
  .bFreqRatio        = FREQ_RATIO,
  .bIsHigherFreqTim  = FREQ_RELATION,

/* PWM generation parameters --------------------------------------------------*/
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},
  .RepetitionCounter = REP_COUNTER,
  .hTafter           = TW_AFTER,
  .hTbefore          = TW_BEFORE,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,

/* Current reading A/D Conversions initialization ----------------------------*/
  .ADCConfig1 = {
                   (uint32_t)(${getADC("CFG",1,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos)
                  ,(uint32_t)(${getADC("CFG",2,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos)
                  ,(uint32_t)(${getADC("CFG",3,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos)
                  ,(uint32_t)(${getADC("CFG",4,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos)
                  ,(uint32_t)(${getADC("CFG",5,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos)
                  ,(uint32_t)(${getADC("CFG",6,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos)
                },
  .ADCConfig2 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos)
                 ,(uint32_t)(${getADC("CFG",2,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos)
                 ,(uint32_t)(${getADC("CFG",3,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos)
                 ,(uint32_t)(${getADC("CFG",4,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos)
                 ,(uint32_t)(${getADC("CFG",5,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos)
                 ,(uint32_t)(${getADC("CFG",6,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos)
                },
  .ADCDataReg1 = {
                   ${getADC("IP", 1,"PHASE_1",1)}
                  ,${getADC("IP", 2,"PHASE_1",1)}
                  ,${getADC("IP", 3,"PHASE_1",1)}
                  ,${getADC("IP", 4,"PHASE_1",1)}
                  ,${getADC("IP", 5,"PHASE_1",1)}
                  ,${getADC("IP", 6,"PHASE_1",1)}
                 },
  .ADCDataReg2 = {
                   ${getADC("IP", 1,"PHASE_2",1)}
                  ,${getADC("IP", 2,"PHASE_2",1)}
                  ,${getADC("IP", 3,"PHASE_2",1)}
                  ,${getADC("IP", 4,"PHASE_2",1)}
                  ,${getADC("IP", 5,"PHASE_2",1)}
                  ,${getADC("IP", 6,"PHASE_2",1)}
                 }
};
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) -->

  <#if MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
/**
  * @brief  Current sensor parameters Dual Drive Motor 1 - ICS
  */
const ICS_Params_t ICS_ParamsM1 = {

/* Dual MC parameters --------------------------------------------------------*/
  .InstanceNbr       = 1,
  .Tw                = MAX_TWAIT,
  .FreqRatio         = FREQ_RATIO,
  .IsHigherFreqTim   = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .IaChannel         = MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_U},
  .IbChannel         = MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_V},
  
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)}
  
};
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->

  <#if MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
/**
  * @brief  Current sensor parameters Dual Drive Motor 2 - ICS
  */
const ICS_Params_t ICS_ParamsM2 = {
/* Dual MC parameters --------------------------------------------------------*/
  .InstanceNbr       = 2,
  .Tw                = MAX_TWAIT2,
  .FreqRatio         = FREQ_RATIO,
  .IsHigherFreqTim   = FREQ_RELATION2,
 

/* Current reading A/D Conversions initialization -----------------------------*/
  .IaChannel         = MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_U},
  .IbChannel         = MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_V},

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER2,
  .TIMx              = ${_last_word(MC.M2_PWM_TIMER_SELECTION)}
};
  </#if><#-- MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
</#if><#-- <#if CondFamily_STM32F4 -->


<#if CondFamily_STM32H5 >
  <#if  MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'> <#-- Inside CondFamily_STM32H5 --->
    <#if MC.M1_CS_ADC_NUM=='1'> <#-- Inside CondFamily_STM32H5 --->

/**
  * @brief  Current sensor parameters Single Drive - three shunt, STM32H5X
  */
const R3_1_Params_t R3_1_ParamsM1 =
{

  .ADCx           = ${MC.M1_CS_ADC_U},
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .hTafter = TW_AFTER,
  .hTbefore = TW_BEFORE_R3_1, 
  .TIMx = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},
  .Tsampling                  = (uint16_t)SAMPLING_TIME,
  .Tcase2                     = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3                     = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,
  
/* PWM Driving signals initialization ----------------------------------------*/
  .ADCConfig = { 
                 ( ${getADC("CFG",1,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",1,"PHASE_2")} << ADC_JSQR_JSQ2_Pos ) | 1 << ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT ),
                 ( ${getADC("CFG",2,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",2,"PHASE_2")} << ADC_JSQR_JSQ2_Pos ) | 1 << ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT ),
                 ( ${getADC("CFG",3,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",3,"PHASE_2")} << ADC_JSQR_JSQ2_Pos ) | 1 << ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT ),
                 ( ${getADC("CFG",4,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",4,"PHASE_2")} << ADC_JSQR_JSQ2_Pos ) | 1 << ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT ),
                 ( ${getADC("CFG",5,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",5,"PHASE_2")} << ADC_JSQR_JSQ2_Pos ) | 1 << ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT ),
                 ( ${getADC("CFG",6,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | ( ${getADC("CFG",6,"PHASE_2")} << ADC_JSQR_JSQ2_Pos ) | 1 << ADC_JSQR_JL_Pos | ( LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT ),                 
               },
};

    <#elseif MC.M1_CS_ADC_NUM=='2' > <#-- inside CondFamily_STM32H5 -->

/**
  * @brief  Current sensor parameters Motor 1 - three shunt 2ADC
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
  .Tw                       = MAX_TWAIT,
  .bFreqRatio               = FREQ_RATIO,
  .bIsHigherFreqTim         = FREQ_RELATION,

/* PWM generation parameters --------------------------------------------------*/
  .TIMx                       =  ${_last_word(MC.M1_PWM_TIMER_SELECTION)},
  .RepetitionCounter          =  REP_COUNTER,
  .hTafter                    =  TW_AFTER,
  .hTbefore                   =  TW_BEFORE,
  .Tsampling                  = (uint16_t)SAMPLING_TIME,
  .Tcase2                     = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3                     = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,
  
/* Current reading A/D Conversions initialization ----------------------------*/
   .ADCConfig1 = { 
                   (uint32_t)( ${getADC("CFG",1,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                   (uint32_t)( ${getADC("CFG",2,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                   (uint32_t)( ${getADC("CFG",3,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                   (uint32_t)( ${getADC("CFG",4,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                   (uint32_t)( ${getADC("CFG",5,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                   (uint32_t)( ${getADC("CFG",6,"PHASE_1")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 },
   .ADCConfig2 = { 
                   (uint32_t)( ${getADC("CFG",1,"PHASE_2")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                   (uint32_t)( ${getADC("CFG",2,"PHASE_2")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                   (uint32_t)( ${getADC("CFG",3,"PHASE_2")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                   (uint32_t)( ${getADC("CFG",4,"PHASE_2")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                   (uint32_t)( ${getADC("CFG",5,"PHASE_2")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                   (uint32_t)( ${getADC("CFG",6,"PHASE_2")} << ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 },              
  .ADCDataReg1 = { ${getADC("IP", 1,"PHASE_1")}
                 , ${getADC("IP", 2,"PHASE_1")}
                 , ${getADC("IP", 3,"PHASE_1")}
                 , ${getADC("IP", 4,"PHASE_1")}
                 , ${getADC("IP", 5,"PHASE_1")}
                 , ${getADC("IP", 6,"PHASE_1")}
                 },
  .ADCDataReg2 =  { ${getADC("IP", 1,"PHASE_2")}
                 , ${getADC("IP", 2,"PHASE_2")}
                 , ${getADC("IP", 3,"PHASE_2")}
                 , ${getADC("IP", 4,"PHASE_2")}
                 , ${getADC("IP", 5,"PHASE_2")}
                 , ${getADC("IP", 6,"PHASE_2")}
                  }
};
    </#if> <#--  MC.M1_CS_ADC_NUM=='1' -->
  </#if> <#-- MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'  -->
</#if> <#-- CondFamily_STM32H5 -->

<#if CondFamily_STM32F0>
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
/**
  * @brief  Current sensor parameters Single Drive - three shunt, STM32F0X
  */
const R3_1_Params_t R3_1_Params =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .b_ISamplingTime   = LL_ADC_SAMPLINGTIME_${MC.M1_CURR_SAMPLING_TIME}<#if MC.M1_CURR_SAMPLING_TIME != "1">CYCLES_5<#else>CYCLE_5</#if>,

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime         = DEAD_TIME_COUNTS,
  .RepetitionCounter = REP_COUNTER,
  .hTafter           = TW_AFTER,
  .hTbefore          = TW_BEFORE_R3_1,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,

  .ADCConfig = {
                 (uint32_t)(1<< ${getADC("CFG",1,"PHASE_1",1)}U ) | (uint32_t)(1<< ${getADC("CFG",1,"PHASE_2",1)}U ),
                 (uint32_t)(1<< ${getADC("CFG",2,"PHASE_1",1)}U ) | (uint32_t)(1<< ${getADC("CFG",2,"PHASE_2",1)}U ),
                 (uint32_t)(1<< ${getADC("CFG",3,"PHASE_1",1)}U ) | (uint32_t)(1<< ${getADC("CFG",3,"PHASE_2",1)}U ),
                 (uint32_t)(1<< ${getADC("CFG",4,"PHASE_1",1)}U ) | (uint32_t)(1<< ${getADC("CFG",4,"PHASE_2",1)}U ),
                 (uint32_t)(1<< ${getADC("CFG",5,"PHASE_1",1)}U ) | (uint32_t)(1<< ${getADC("CFG",5,"PHASE_2",1)}U ),
                 (uint32_t)(1<< ${getADC("CFG",6,"PHASE_1",1)}U ) | (uint32_t)(1<< ${getADC("CFG",6,"PHASE_2",1)}U ),
               },
  .ADCScandir = {
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_V Ph2=MC.M1_CS_CHANNEL_W/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_U Ph2=MC.M1_CS_CHANNEL_W/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_W Ph2=MC.M1_CS_CHANNEL_U/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_V Ph2=MC.M1_CS_CHANNEL_U/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_U Ph2=MC.M1_CS_CHANNEL_V/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_W Ph2=MC.M1_CS_CHANNEL_V/>
                },
  .ADCDataReg1 = {
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                 },
     
  .ADCDataReg2 = {
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                 },
};
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')  -->
</#if><#-- CondFamily_STM32F0 -->

<#if CondFamily_STM32G0>
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
/**
  * @brief  Current sensor parameters Single Drive - three shunt, STM32G0X
  */
const R3_1_Params_t R3_1_Params =
{
/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime         = DEAD_TIME_COUNTS,
  .RepetitionCounter = REP_COUNTER,
  .hTafter           = TW_AFTER,
  .hTbefore          = TW_BEFORE_R3_1,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,

  .ADCConfig = {
                 (uint32_t)(1<< ${getADC("CFG",1,"PHASE_1",1)}U) | (uint32_t)(1<< ${getADC("CFG",1,"PHASE_2",1)}U),
                 (uint32_t)(1<< ${getADC("CFG",2,"PHASE_1",1)}U) | (uint32_t)(1<< ${getADC("CFG",2,"PHASE_2",1)}U),
                 (uint32_t)(1<< ${getADC("CFG",3,"PHASE_1",1)}U) | (uint32_t)(1<< ${getADC("CFG",3,"PHASE_2",1)}U),
                 (uint32_t)(1<< ${getADC("CFG",4,"PHASE_1",1)}U) | (uint32_t)(1<< ${getADC("CFG",4,"PHASE_2",1)}U),
                 (uint32_t)(1<< ${getADC("CFG",5,"PHASE_1",1)}U) | (uint32_t)(1<< ${getADC("CFG",5,"PHASE_2",1)}U),
                 (uint32_t)(1<< ${getADC("CFG",6,"PHASE_1",1)}U) | (uint32_t)(1<< ${getADC("CFG",6,"PHASE_2",1)}U),
               },
  .ADCScandir = {
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_V Ph2=MC.M1_CS_CHANNEL_W/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_U Ph2=MC.M1_CS_CHANNEL_W/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_W Ph2=MC.M1_CS_CHANNEL_U/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_V Ph2=MC.M1_CS_CHANNEL_U/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_U Ph2=MC.M1_CS_CHANNEL_V/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_W Ph2=MC.M1_CS_CHANNEL_V/>
                },
  .ADCDataReg1 = {
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                 },
     
  .ADCDataReg2 = {
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                 },
};
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')  -->
</#if><#-- CondFamily_STM32G0 -->

<#if CondFamily_STM32C0>
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
/**
  * @brief  Current sensor parameters Single Drive - three shunt, STM32F0X
  */
const R3_1_Params_t R3_1_Params =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .b_ISamplingTime   = LL_ADC_SAMPLINGTIME_${MC.M1_CURR_SAMPLING_TIME}<#if MC.M1_CURR_SAMPLING_TIME != "1">CYCLES_5<#else>CYCLE_5</#if>,

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime         = DEAD_TIME_COUNTS,
  .RepetitionCounter = REP_COUNTER,
  .hTafter           = TW_AFTER,
  .hTbefore          = TW_BEFORE_R3_1,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,

  .ADCConfig = {
                 (uint32_t)(1<< ${getADC("CFG",1,"PHASE_1")}U ) | (uint32_t)(1<< ${getADC("CFG",1,"PHASE_2")}U ),
                 (uint32_t)(1<< ${getADC("CFG",2,"PHASE_1")}U ) | (uint32_t)(1<< ${getADC("CFG",2,"PHASE_2")}U ),
                 (uint32_t)(1<< ${getADC("CFG",3,"PHASE_1")}U ) | (uint32_t)(1<< ${getADC("CFG",3,"PHASE_2")}U ),
                 (uint32_t)(1<< ${getADC("CFG",4,"PHASE_1")}U ) | (uint32_t)(1<< ${getADC("CFG",4,"PHASE_2")}U ),
                 (uint32_t)(1<< ${getADC("CFG",5,"PHASE_1")}U ) | (uint32_t)(1<< ${getADC("CFG",5,"PHASE_2")}U ),
                 (uint32_t)(1<< ${getADC("CFG",6,"PHASE_1")}U ) | (uint32_t)(1<< ${getADC("CFG",6,"PHASE_2")}U ),
               },
  .ADCScandir = {
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_V Ph2=MC.M1_CS_CHANNEL_W/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_U Ph2=MC.M1_CS_CHANNEL_W/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_W Ph2=MC.M1_CS_CHANNEL_U/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_V Ph2=MC.M1_CS_CHANNEL_U/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_U Ph2=MC.M1_CS_CHANNEL_V/>
                  <@setScandir Ph1=MC.M1_CS_CHANNEL_W Ph2=MC.M1_CS_CHANNEL_V/>
                },
  .ADCDataReg1 = {
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                 },
     
  .ADCDataReg2 = {
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                 },
};
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))  -->
</#if><#-- CondFamily_STM32C0 -->

<#if CondFamily_STM32L4><#-- CondFamily_STM32L4 --->
  <#if MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'><#-- Inside CondFamily_STM32L4 --->
const ICS_Params_t ICS_ParamsM1 = 
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio         = FREQ_RATIO,
  .IsHigherFreqTim   = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1            = ${MC.M1_CS_ADC_U},
  .ADCx_2            = ${MC.M1_CS_ADC_V},
    <#if MC.M1_PWM_TIMER_SELECTION == "PWM_TIM1">
  .ADCConfig1                   = (MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_U} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                 | LL_ADC_INJ_TRIG_EXT_TIM1_CH4,
  .ADCConfig2                   = (MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_V} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                 | LL_ADC_INJ_TRIG_EXT_TIM1_CH4,
    <#elseif MC.M1_PWM_TIMER_SELECTION == "PWM_TIM8">
  .ADCConfig1                   = (MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_U} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM8_CH4,
  .ADCConfig2                   = (MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_V} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM8_CH4,
    </#if><#-- MC.M1_PWM_TIMER_SELECTION == "PWM_TIM1" -->

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)}
};
  <#elseif  MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'><#-- Inside CondFamily_STM32L4 --->
    <#if MC.M1_CS_ADC_NUM =='1'><#-- Inside CondFamily_STM32L4 --->
/**
  * @brief  Current sensor parameters Motor 1 - three shunt 1 ADC 
  */
const R3_1_Params_t R3_1_ParamsM1 =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx              = ${MC.M1_CS_ADC_U},
  .ADCConfig = {
                 (uint32_t)(${getADC("CFG",1,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos) 
               | ${getADC("CFG",1,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 (uint32_t)(${getADC("CFG",2,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
               | ${getADC("CFG",2,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 (uint32_t)(${getADC("CFG",3,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
               | ${getADC("CFG",3,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 (uint32_t)(${getADC("CFG",4,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
               | ${getADC("CFG",4,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 (uint32_t)(${getADC("CFG",5,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
               | ${getADC("CFG",5,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                 (uint32_t)(${getADC("CFG",6,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
               | ${getADC("CFG",6,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
               },

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .hTafter           = TW_AFTER,
  .hTbefore          = TW_BEFORE_R3_1,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},
};
     <#elseif MC.M1_CS_ADC_NUM=='2'><#-- Inside CondFamily_STM32L4 --->
/**
  * @brief  Current sensor parameters Motor 1 - three shunt - L4XX - Independent Resources
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .bFreqRatio        = FREQ_RATIO,
  .bIsHigherFreqTim  = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCConfig1 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  (uint32_t)(${getADC("CFG",2,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  (uint32_t)(${getADC("CFG",3,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  (uint32_t)(${getADC("CFG",4,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  (uint32_t)(${getADC("CFG",5,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  (uint32_t)(${getADC("CFG",6,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                },
  .ADCConfig2 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  (uint32_t)(${getADC("CFG",2,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  (uint32_t)(${getADC("CFG",3,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  (uint32_t)(${getADC("CFG",4,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  (uint32_t)(${getADC("CFG",5,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                  (uint32_t)(${getADC("CFG",6,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos) | 0U << ADC_JSQR_JL_Pos
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT),
                },
  .ADCDataReg1 = {
                   ${getADC("IP", 1,"PHASE_1",1)}
                  ,${getADC("IP", 2,"PHASE_1",1)}
                  ,${getADC("IP", 3,"PHASE_1",1)}
                  ,${getADC("IP", 4,"PHASE_1",1)}
                  ,${getADC("IP", 5,"PHASE_1",1)}
                  ,${getADC("IP", 6,"PHASE_1",1)}
                 },
  .ADCDataReg2 = {
                   ${getADC("IP", 1,"PHASE_2",1)}
                  ,${getADC("IP", 2,"PHASE_2",1)}
                  ,${getADC("IP", 3,"PHASE_2",1)}
                  ,${getADC("IP", 4,"PHASE_2",1)}
                  ,${getADC("IP", 5,"PHASE_2",1)}
                  ,${getADC("IP", 6,"PHASE_2",1)}
                 },

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .hTafter           = TW_AFTER,
  .hTbefore          = TW_BEFORE,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},

};
    </#if><#-- MC.M1_CS_ADC_NUM =='1' --><#-- CondFamily_STM32L4 --->
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' --><#-- CondFamily_STM32L4 --->
</#if><#-- CondFamily_STM32L4 --->


<#if CondFamily_STM32F7><#-- CondFamily_STM32F7 --->
  <#if MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'><#-- Inside CondFamily_STM32F7 --->
/**
  * @brief  Current sensor parameters Dual Drive Motor 1 - ICS
  */
const ICS_Params_t ICS_ParamsM1 = {

/* Dual MC parameters --------------------------------------------------------*/
  .InstanceNbr       = 1,
  .Tw                = MAX_TWAIT,
  .FreqRatio         = FREQ_RATIO,
  .IsHigherFreqTim   = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .IaChannel         = MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_U},
  .IbChannel         = MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_V},

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)}

}; 
  <#elseif  MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'><#-- Inside CondFamily_STM32F7 --->
    <#if MC.M1_CS_ADC_NUM=='1'><#-- Inside CondFamily_STM32F7 --->
/**
  * @brief  Current sensor parameters Motor 1 - three shunt 1 ADC
  */
const R3_1_Params_t R3_1_ParamsM1 =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx = ${MC.M1_CS_ADC_U},
  
  .ADCConfig = {
                 (uint32_t)(${getADC("CFG",1,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
               | ${getADC("CFG",1,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",2,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
               | ${getADC("CFG",2,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",3,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
               | ${getADC("CFG",3,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",4,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
               | ${getADC("CFG",4,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",5,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
               | ${getADC("CFG",5,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
                 (uint32_t)(${getADC("CFG",6,"PHASE_1",1)}U << ADC_JSQR_JSQ3_Pos)
               | ${getADC("CFG",6,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos | 1<< ADC_JSQR_JL_Pos,
               },

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE_R3_1,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)}
};  
    <#elseif MC.M1_CS_ADC_NUM=='2'><#-- Inside CondFamily_STM32F7 --->
/**
  * @brief  Current sensor parameters Motor 1 - three shunt
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .Tw                =	MAX_TWAIT,
  .bFreqRatio        =	FREQ_RATIO,
  .bIsHigherFreqTim  =	FREQ_RELATION,

/* Current reading A/D Conversions initialization ----------------------------*/
  .ADCConfig1 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                  (uint32_t)(${getADC("CFG",2,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                  (uint32_t)(${getADC("CFG",3,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                  (uint32_t)(${getADC("CFG",4,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                  (uint32_t)(${getADC("CFG",5,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                  (uint32_t)(${getADC("CFG",6,"PHASE_1",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                },
  .ADCConfig2 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                  (uint32_t)(${getADC("CFG",2,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                  (uint32_t)(${getADC("CFG",3,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                  (uint32_t)(${getADC("CFG",4,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                  (uint32_t)(${getADC("CFG",5,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                  (uint32_t)(${getADC("CFG",6,"PHASE_2",1)}U << ADC_JSQR_JSQ4_Pos) | 0U << ADC_JSQR_JL_Pos,
                },
  .ADCDataReg1 = {
                   ${getADC("IP", 1,"PHASE_1",1)}
                  ,${getADC("IP", 2,"PHASE_1",1)}
                  ,${getADC("IP", 3,"PHASE_1",1)}
                  ,${getADC("IP", 4,"PHASE_1",1)}
                  ,${getADC("IP", 5,"PHASE_1",1)}
                  ,${getADC("IP", 6,"PHASE_1",1)}
                 },
  .ADCDataReg2 = {
                   ${getADC("IP", 1,"PHASE_2",1)}
                  ,${getADC("IP", 2,"PHASE_2",1)}
                  ,${getADC("IP", 3,"PHASE_2",1)}
                  ,${getADC("IP", 4,"PHASE_2",1)}
                  ,${getADC("IP", 5,"PHASE_2",1)}
                  ,${getADC("IP", 6,"PHASE_2",1)}
                 },

/* PWM generation parameters --------------------------------------------------*/
  .TIMx              =	${_last_word(MC.M1_PWM_TIMER_SELECTION)},
  .RepetitionCounter =	REP_COUNTER,
  .hTafter           =	TW_AFTER,
  .hTbefore          =	TW_BEFORE,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u
};
    </#if><#-- MC.M1_CS_ADC_NUM=='1' -->
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->
</#if><#-- CondFamily_STM32F7 --->

<#if CondFamily_STM32H7><#-- CondFamily_STM32H7 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#error " H7 Single shunt not supported yet "
  <#elseif MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'><#-- Inside CondFamily_STM32H7 --->
#error " H7 ICS not supported yet "

  <#elseif MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'><#-- Inside CondFamily_STM32H7 --->
    <#if MC.M1_CS_ADC_NUM=='2'>
      <#if MC.M1_USE_INTERNAL_OPAMP>
/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - H7 
  */
const R3_2_OPAMPParams_t R3_2_OPAMPParamsM1 =
{
  .OPAMPSelect_1 = {
                     ${getOPAMP("IP",1,"PHASE_1",1)}
                    ,${getOPAMP("IP",2,"PHASE_1",1)}
                    ,${getOPAMP("IP",3,"PHASE_1",1)}
                    ,${getOPAMP("IP",4,"PHASE_1",1)}
                    ,${getOPAMP("IP",5,"PHASE_1",1)}
                    ,${getOPAMP("IP",6,"PHASE_1",1)}
                   },
  .OPAMPSelect_2 = {
                     ${getOPAMP("IP",1,"PHASE_2",1)}
                    ,${getOPAMP("IP",2,"PHASE_2",1)}
                    ,${getOPAMP("IP",3,"PHASE_2",1)}
                    ,${getOPAMP("IP",4,"PHASE_2",1)}
                    ,${getOPAMP("IP",5,"PHASE_2",1)}
                    ,${getOPAMP("IP",6,"PHASE_2",1)}
                   },
  .OPAMPConfig1 = {
                    ${getOPAMP("CFG",1,"PHASE_1",1)}
                   ,${getOPAMP("CFG",2,"PHASE_1",1)}
                   ,${getOPAMP("CFG",3,"PHASE_1",1)}
                   ,${getOPAMP("CFG",4,"PHASE_1",1)}
                   ,${getOPAMP("CFG",5,"PHASE_1",1)}
                   ,${getOPAMP("CFG",6,"PHASE_1",1)}
                  },
  .OPAMPConfig2 = {
                    ${getOPAMP("CFG",1,"PHASE_2",1)}
                   ,${getOPAMP("CFG",2,"PHASE_2",1)}
                   ,${getOPAMP("CFG",3,"PHASE_2",1)}
                   ,${getOPAMP("CFG",4,"PHASE_2",1)}
                   ,${getOPAMP("CFG",5,"PHASE_2",1)}
                   ,${getOPAMP("CFG",6,"PHASE_2",1)}
                  },
};
      </#if><#-- MC.M1_USE_INTERNAL_OPAMP -->

  /**
  * @brief  Current sensor parameters Motor 1 - three shunt - H7 - Shared Resources
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCConfig1 = {
                  (${getADC("CFG",1,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",2,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",3,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",4,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",5,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",6,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
  },
  .ADCConfig2 = {
                  (${getADC("CFG",1,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",2,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",3,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",4,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",5,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",6,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
  .ADCDataReg1 = {
                   ${getADC("IP", 1,"PHASE_1",1)}
                  ,${getADC("IP", 2,"PHASE_1",1)}
                  ,${getADC("IP", 3,"PHASE_1",1)}
                  ,${getADC("IP", 4,"PHASE_1",1)}
                  ,${getADC("IP", 5,"PHASE_1",1)}
                  ,${getADC("IP", 6,"PHASE_1",1)}
                 },
  .ADCDataReg2 = {
                   ${getADC("IP", 1,"PHASE_2",1)}
                  ,${getADC("IP", 2,"PHASE_2",1)}
                  ,${getADC("IP", 3,"PHASE_2",1)}
                  ,${getADC("IP", 4,"PHASE_2",1)}
                  ,${getADC("IP", 5,"PHASE_2",1)}
                  ,${getADC("IP", 6,"PHASE_2",1)}
                 },

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter     = REP_COUNTER,
  .Tafter                = TW_AFTER,
  .Tbefore               = TW_BEFORE,
  .TIMx                  = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},

/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.M1_USE_INTERNAL_OPAMP>
  .OPAMPParams           = &R3_2_OPAMPParamsM1,
      <#else><#-- MC.M1_USE_INTERNAL_OPAMP == false -->
  .OPAMPParams           = MC_NULL,
      </#if><#-- MC.M1_USE_INTERNAL_OPAMP -->

/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
      <#else><#-- MC.M1_OCP_TOPOLOGY == "EMBEDDED" -->
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
      </#if><#-- MC.M1_OCP_TOPOLOGY == "EMBEDDED" -->

      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
      <#else><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION == flase -->
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
      </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION -->

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold     = ${MC.M1_OVPREF},
};
    <#elseif MC.M1_CS_ADC_NUM=='1'><#-- Inside CondFamily_STM32H7 --->
    #error " H7 Single ADC not supported yet"
    </#if><#-- MC.M1_CS_ADC_NUM=='2' -->
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if><#-- CondFamily_STM32H7 -->


<#if CondFamily_STM32F3><#-- CondFamily_STM32F3 --->
  <#if MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'><#-- Inside CondFamily_STM32F3 --->
const ICS_Params_t ICS_ParamsM1 = 
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio         = FREQ_RATIO,
  .IsHigherFreqTim   = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ${MC.M1_CS_ADC_U},
  .ADCx_2 = ${MC.M1_CS_ADC_V},
    <#if MC.M1_PWM_TIMER_SELECTION == "PWM_TIM1">
  .ADCConfig1                   = (MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_U} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM1_CH4,
  .ADCConfig2                   = (MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_V} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM1_CH4,
      <#elseif MC.M1_PWM_TIMER_SELECTION == "PWM_TIM8">
        <#if (((MC.M1_CS_ADC_U == "ADC1") || (MC.M1_CS_ADC_U == "ADC2")) && ((MC.M1_CS_ADC_V == "ADC1") || (MC.M1_CS_ADC_V == "ADC2")))>
  .ADCConfig1                   = (MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_U} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM8_CH4_ADC12,
  .ADCConfig2                   = (MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_V} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM8_CH4_ADC12,
        <#elseif (((MC.M1_CS_ADC_U == "ADC3") || (MC.M1_CS_ADC_U == "ADC4")) && ((MC.M1_CS_ADC_V == "ADC3") || (MC.M1_CS_ADC_V == "ADC4")))>
  .ADCConfig1                   = (MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_U} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM8_CH4__ADC34,
  .ADCConfig2                   = (MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_V} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM8_CH4__ADC34,
        <#else>
          #error "Not supported ICS configuration"
    </#if><#-- (((MC.M1_CS_ADC_U == "ADC1") || (MC.M1_CS_ADC_U == "ADC2"))... -->
  </#if><#-- MC.M1_PWM_TIMER_SELECTION == "PWM_TIM1" -->

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)}
  
};

  <#elseif MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'><#-- Inside CondFamily_STM32F3 --->
    <#if MC.M1_CS_ADC_NUM=='2'>
      <#if MC.M1_USE_INTERNAL_OPAMP>
/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - F3xx 
  */
const R3_2_OPAMPParams_t R3_2_OPAMPParamsM1 =
{
  .OPAMPSelect_1 = {
                     ${getOPAMP("IP",1,"PHASE_1",1)}
                    ,${getOPAMP("IP",2,"PHASE_1",1)}
                    ,${getOPAMP("IP",3,"PHASE_1",1)}
                    ,${getOPAMP("IP",4,"PHASE_1",1)}
                    ,${getOPAMP("IP",5,"PHASE_1",1)}
                    ,${getOPAMP("IP",6,"PHASE_1",1)}
                   },
  .OPAMPSelect_2 = {
                     ${getOPAMP("IP",1,"PHASE_2",1)}
                    ,${getOPAMP("IP",2,"PHASE_2",1)}
                    ,${getOPAMP("IP",3,"PHASE_2",1)}
                    ,${getOPAMP("IP",4,"PHASE_2",1)}
                    ,${getOPAMP("IP",5,"PHASE_2",1)}
                    ,${getOPAMP("IP",6,"PHASE_2",1)}
                   },
  
  .OPAMPConfig1 = {
                    ${getOPAMP("CFG",1,"PHASE_1",1)}
                   ,${getOPAMP("CFG",2,"PHASE_1",1)}
                   ,${getOPAMP("CFG",3,"PHASE_1",1)}
                   ,${getOPAMP("CFG",4,"PHASE_1",1)}
                   ,${getOPAMP("CFG",5,"PHASE_1",1)}
                   ,${getOPAMP("CFG",6,"PHASE_1",1)}
                  },
  .OPAMPConfig2 = {
                    ${getOPAMP("CFG",1,"PHASE_2",1)}
                   ,${getOPAMP("CFG",2,"PHASE_2",1)}
                   ,${getOPAMP("CFG",3,"PHASE_2",1)}
                   ,${getOPAMP("CFG",4,"PHASE_2",1)}
                   ,${getOPAMP("CFG",5,"PHASE_2",1)}
                   ,${getOPAMP("CFG",6,"PHASE_2",1)}
                  },
};
      </#if><#-- MC.M1_USE_INTERNAL_OPAMP -->

  /**
  * @brief  Current sensor parameters Motor 1 - three shunt - F30x - Shared Resources
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCConfig1 = {
                  (${getADC("CFG",1,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",2,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",3,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",4,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",5,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",6,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)                      
                },
  .ADCConfig2 = {
                  (${getADC("CFG",1,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",2,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",3,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",4,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",5,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",6,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)                      
                },
  .ADCDataReg1 = {
                   ${getADC("IP", 1,"PHASE_1",1)}
                  ,${getADC("IP", 2,"PHASE_1",1)}
                  ,${getADC("IP", 3,"PHASE_1",1)}
                  ,${getADC("IP", 4,"PHASE_1",1)}
                  ,${getADC("IP", 5,"PHASE_1",1)}
                  ,${getADC("IP", 6,"PHASE_1",1)}
                 },
  .ADCDataReg2 = {
                   ${getADC("IP", 1,"PHASE_2",1)}
                  ,${getADC("IP", 2,"PHASE_2",1)}
                  ,${getADC("IP", 3,"PHASE_2",1)}
                  ,${getADC("IP", 4,"PHASE_2",1)}
                  ,${getADC("IP", 5,"PHASE_2",1)}
                  ,${getADC("IP", 6,"PHASE_2",1)}
                 },
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter                = TW_AFTER,
  .Tbefore               = TW_BEFORE,
  .Tsampling             = (uint16_t)SAMPLING_TIME,
  .Tcase2                = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3                = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,
  .TIMx                  = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},

/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.M1_USE_INTERNAL_OPAMP>
  .OPAMPParams           = &R3_2_OPAMPParamsM1,
      <#else><#-- MC.M1_USE_INTERNAL_OPAMP == flase -->
  .OPAMPParams           = MC_NULL,
      </#if><#-- MC.M1_USE_INTERNAL_OPAMP -->

/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
      <#else><#-- MC.M1_OCP_TOPOLOGY != "EMBEDDED" -->
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
      </#if><#-- MC.M1_OCP_TOPOLOGY == "EMBEDDED" -->

      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
      <#else><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION == false -->
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
      </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION -->

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold     = ${MC.M1_OVPREF},
};
    <#elseif MC.M1_CS_ADC_NUM=='1'><#-- Inside CondFamily_STM32F3 --->
/**
  * @brief  Current sensor parameters Motor 1 - three shunt 1 ADC (STM32F302x8)
  */
const R3_1_Params_t R3_1_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx                  = ${MC.M1_CS_ADC_U},
  .ADCConfig = {
                 (${getADC("CFG",1,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",1,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",2,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",2,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",3,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",3,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",4,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",4,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",5,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",5,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",6,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",6,"PHASE_2",1)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)                       
               },

 
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter                = TW_AFTER,
  .Tbefore               = TW_BEFORE_R3_1,
  .Tsampling             = (uint16_t)SAMPLING_TIME,
  .Tcase2                = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3                = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,
  .TIMx                  = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},

/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
      <#else><#-- MC.M1_OCP_TOPOLOGY != "EMBEDDED" -->
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
      </#if><#-- MC.M1_OCP_TOPOLOGY == "EMBEDDED" -->

      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
      <#else><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION == false -->
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
      </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION -->

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold     = ${MC.M1_OVPREF},
};
    </#if><#--  MC.M1_CS_ADC_NUM=='2' -->
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->
  <#if MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS'><#-- Inside CondFamily_STM32F3 --->
const ICS_Params_t ICS_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio         = FREQ_RATIO,
  .IsHigherFreqTim   = FREQ_RELATION2,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ${MC.M2_CS_ADC_U},
  .ADCx_2 = ${MC.M2_CS_ADC_V},
    <#if MC.M2_PWM_TIMER_SELECTION == "PWM_TIM1">
  .ADCConfig1                   = (MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_U} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM1_CH4,
  .ADCConfig2                   = (MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_V} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM1_CH4,
    <#elseif MC.M2_PWM_TIMER_SELECTION == "PWM_TIM8">
      <#if (((MC.M2_CS_ADC_U == "ADC1") || (MC.M2_CS_ADC_U == "ADC2")) && ((MC.M2_CS_ADC_V == "ADC1") || (MC.M2_CS_ADC_V == "ADC2")))>
  .ADCConfig1                   = (MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_U} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM8_CH4_ADC12,
  .ADCConfig2                   = (MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_V} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM8_CH4_ADC12,
      <#elseif (((MC.M2_CS_ADC_U == "ADC3") || (MC.M2_CS_ADC_U == "ADC4")) && ((MC.M2_CS_ADC_V == "ADC3") || (MC.M2_CS_ADC_V == "ADC4")))>
  .ADCConfig1                   = (MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_U} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM8_CH4__ADC34,
  .ADCConfig2                   = (MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_V} << 8) | LL_ADC_INJ_TRIG_EXT_RISING
                                | LL_ADC_INJ_TRIG_EXT_TIM8_CH4__ADC34,
      <#else>
          #error "Not supported ICS configuration"
      </#if><#-- (((MC.M2_CS_ADC_U == "ADC1") || (MC.M2_CS_ADC_U == "ADC2"))... -->
    </#if><#-- MC.M2_PWM_TIMER_SELECTION == "PWM_TIM1" -->

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER2,
  .TIMx              = ${_last_word(MC.M2_PWM_TIMER_SELECTION)}
};
  <#elseif ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
    <#-- Inside CondFamily_STM32F3 --->
    <#if MC.M2_USE_INTERNAL_OPAMP>
/**
  * @brief  Internal OPAMP parameters Motor 2 - three shunt - F3xx 
  */
R3_2_OPAMPParams_t R3_2_OPAMPParamsM2 =
{
  .OPAMPSelect_1 = {
                     ${getOPAMP("IP",1,"PHASE_1",2)}
                    ,${getOPAMP("IP",2,"PHASE_1",2)}
                    ,${getOPAMP("IP",3,"PHASE_1",2)}
                    ,${getOPAMP("IP",4,"PHASE_1",2)}
                    ,${getOPAMP("IP",5,"PHASE_1",2)}
                    ,${getOPAMP("IP",6,"PHASE_1",2)}
                   },
  .OPAMPSelect_2 = {
                     ${getOPAMP("IP",1,"PHASE_2",2)}
                    ,${getOPAMP("IP",2,"PHASE_2",2)}
                    ,${getOPAMP("IP",3,"PHASE_2",2)}
                    ,${getOPAMP("IP",4,"PHASE_2",2)}
                    ,${getOPAMP("IP",5,"PHASE_2",2)}
                    ,${getOPAMP("IP",6,"PHASE_2",2)}
                   },
  
  .OPAMPConfig1 = {
                    ${getOPAMP("CFG",1,"PHASE_1",2)}
                   ,${getOPAMP("CFG",2,"PHASE_1",2)}
                   ,${getOPAMP("CFG",3,"PHASE_1",2)}
                   ,${getOPAMP("CFG",4,"PHASE_1",2)}
                   ,${getOPAMP("CFG",5,"PHASE_1",2)}
                   ,${getOPAMP("CFG",6,"PHASE_1",2)}
                  },
  .OPAMPConfig2 = {
                    ${getOPAMP("CFG",1,"PHASE_2",2)}
                   ,${getOPAMP("CFG",2,"PHASE_2",2)}
                   ,${getOPAMP("CFG",3,"PHASE_2",2)}
                   ,${getOPAMP("CFG",4,"PHASE_2",2)}
                   ,${getOPAMP("CFG",5,"PHASE_2",2)}
                   ,${getOPAMP("CFG",6,"PHASE_2",2)}
                  },
};
    </#if><#-- MC.M2_USE_INTERNAL_OPAMP -->

  /**
  * @brief  Current sensor parameters Motor 2 - three shunt - F30x 
  */
const R3_2_Params_t R3_2_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION2,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCConfig1 = {
                  (${getADC("CFG",1,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",2,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",3,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",4,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",5,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",6,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
  .ADCConfig2 = {
                  (${getADC("CFG",1,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",2,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",3,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",4,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",5,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(${getADC("CFG",6,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
  .ADCDataReg1 = {
                   ${getADC("IP", 1,"PHASE_1",2)}
                  ,${getADC("IP", 2,"PHASE_1",2)}
                  ,${getADC("IP", 3,"PHASE_1",2)}
                  ,${getADC("IP", 4,"PHASE_1",2)}
                  ,${getADC("IP", 5,"PHASE_1",2)}
                  ,${getADC("IP", 6,"PHASE_1",2)}
                 },
  .ADCDataReg2 = {
                   ${getADC("IP", 1,"PHASE_2",2)}
                  ,${getADC("IP", 2,"PHASE_2",2)}
                  ,${getADC("IP", 3,"PHASE_2",2)}
                  ,${getADC("IP", 4,"PHASE_2",2)}
                  ,${getADC("IP", 5,"PHASE_2",2)}
                  ,${getADC("IP", 6,"PHASE_2",2)}
                 },
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter     = REP_COUNTER2,
  .Tafter                = TW_AFTER2,
  .Tbefore               = TW_BEFORE2,
  .TIMx                  = ${_last_word(MC.M2_PWM_TIMER_SELECTION)},

/* Internal OPAMP common settings --------------------------------------------*/
    <#if MC.M2_USE_INTERNAL_OPAMP>
  .OPAMPParams           = &R3_2_OPAMPParamsM2,
    <#else><#-- MC.M2_USE_INTERNAL_OPAMP == false -->
  .OPAMPParams           = MC_NULL,
    </#if><#-- MC.M2_USE_INTERNAL_OPAMP -->

/* Internal COMP settings ----------------------------------------------------*/
    <#if MC.M2_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M2_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE2},
  .CompOCPBSelection     = ${MC.M2_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE2},
  .CompOCPCSelection     = ${MC.M2_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE2},
    <#else><#-- MC.M2_OCP_TOPOLOGY != "EMBEDDED" -->
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
    </#if><#-- MC.M2_OCP_TOPOLOGY == "EMBEDDED" -->

    <#if MC.INTERNAL_OVERVOLTAGEPROTECTION2>
  .CompOVPSelection      = OVP_SELECTION2,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE2,
    <#else><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION2 == false -->
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
    </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION2 -->

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = ${MC.M2_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold     = ${MC.M2_OVPREF},
};

  <#elseif (MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1')><#-- Inside CondFamily_STM32F3 --->
/**
  * @brief  Current sensor parameters Motor 2 - three shunt 1 ADC (STM32F302x8)
  */
const R3_1_Params_t R3_1_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION2,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx                  = ${MC.M2_CS_ADC_U},
  .ADCConfig = {
                 (${getADC("CFG",1,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",1,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",2,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",2,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",3,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",3,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",4,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",4,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",5,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",5,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",6,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",6,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
               },

 
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER2,
  .Tafter                = TW_AFTER2,
  .Tbefore               = TW_BEFORE_R3_1_2,
  .Tsampling             = (uint16_t)SAMPLING_TIME2,
  .Tcase2                = (uint16_t)SAMPLING_TIME2 + (uint16_t)TDEAD2 + (uint16_t)TRISE2,
  .Tcase3                = ((uint16_t)TDEAD2 + (uint16_t)TNOISE2 + (uint16_t)SAMPLING_TIME2) / 2u,
  .TIMx                  = ${_last_word(MC.M2_PWM_TIMER_SELECTION)},

/* Internal COMP settings ----------------------------------------------------*/
    <#if MC.M2_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M2_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE2},
  .CompOCPBSelection     = ${MC.M2_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE2},
  .CompOCPCSelection     = ${MC.M2_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE2},
    <#else><#-- MC.M2_OCP_TOPOLOGY != "EMBEDDED" -->
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
    </#if><#-- MC.M2_OCP_TOPOLOGY == "EMBEDDED" -->

    <#if MC.INTERNAL_OVERVOLTAGEPROTECTION2>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
    <#else><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION2 == false -->
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
    </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION2 -->

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = ${MC.M2_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold     = ${MC.M2_OVPREF},
};
  </#if><#-- MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->
</#if><#-- CondFamily_STM32F3 --->

<#if CondFamily_STM32G4>
  <#if MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT' >
    <#if MC.M2_USE_INTERNAL_OPAMP>
R3_3_OPAMPParams_t R3_3_OPAMPParamsM2 = 
{ 
  .OPAMPSelect_1 = {
                     ${getOPAMP("IP",1,"PHASE_1",2)}
                    ,${getOPAMP("IP",2,"PHASE_1",2)}
                    ,${getOPAMP("IP",3,"PHASE_1",2)}
                    ,${getOPAMP("IP",4,"PHASE_1",2)}
                    ,${getOPAMP("IP",5,"PHASE_1",2)}
                    ,${getOPAMP("IP",6,"PHASE_1",2)}
                   },
  .OPAMPSelect_2 = {
                     ${getOPAMP("IP",1,"PHASE_2",2)}
                    ,${getOPAMP("IP",2,"PHASE_2",2)}
                    ,${getOPAMP("IP",3,"PHASE_2",2)}
                    ,${getOPAMP("IP",4,"PHASE_2",2)}
                    ,${getOPAMP("IP",5,"PHASE_2",2)}
                    ,${getOPAMP("IP",6,"PHASE_2",2)}
                   },

  .OPAMPConfig1 = {
                    ${getOPAMP("CFG",1,"PHASE_1",2)}
                   ,${getOPAMP("CFG",2,"PHASE_1",2)}
                   ,${getOPAMP("CFG",3,"PHASE_1",2)}
                   ,${getOPAMP("CFG",4,"PHASE_1",2)}
                   ,${getOPAMP("CFG",5,"PHASE_1",2)}
                   ,${getOPAMP("CFG",6,"PHASE_1",2)}
  },
  .OPAMPConfig2 = {${getOPAMP("CFG",1,"PHASE_2",2)}
                  ,${getOPAMP("CFG",2,"PHASE_2",2)}
                  ,${getOPAMP("CFG",3,"PHASE_2",2)}
                  ,${getOPAMP("CFG",4,"PHASE_2",2)}
                  ,${getOPAMP("CFG",5,"PHASE_2",2)}
                  ,${getOPAMP("CFG",6,"PHASE_2",2)}
                 },
};
    </#if><#-- MC.M2_USE_INTERNAL_OPAMP -->
    <#if MC.M2_CS_ADC_NUM=='2' >
/**
  * @brief  Current sensor parameters Motor 2 - three shunt - G4 
  */
const R3_2_Params_t R3_2_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION2,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCConfig1 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",2,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",3,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",4,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",5,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",6,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
  .ADCConfig2 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",2,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",3,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",4,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",5,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",6,"PHASE_2",2)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
 
  .ADCDataReg1 = {
                   ${getADC("IP", 1,"PHASE_1",2)}
                  ,${getADC("IP", 2,"PHASE_1",2)}
                  ,${getADC("IP", 3,"PHASE_1",2)}
                  ,${getADC("IP", 4,"PHASE_1",2)}
                  ,${getADC("IP", 5,"PHASE_1",2)}
                  ,${getADC("IP", 6,"PHASE_1",2)}
                 },
  .ADCDataReg2 = {
                   ${getADC("IP", 1,"PHASE_2",2)}
                  ,${getADC("IP", 2,"PHASE_2",2)}
                  ,${getADC("IP", 3,"PHASE_2",2)}
                  ,${getADC("IP", 4,"PHASE_2",2)}
                  ,${getADC("IP", 5,"PHASE_2",2)}
                  ,${getADC("IP", 6,"PHASE_2",2)}
                  },

  /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter     = REP_COUNTER2,
  .Tafter                = TW_AFTER2,
  .Tbefore               = TW_BEFORE2,
  .Tsampling             = (uint16_t)SAMPLING_TIME2,
  .Tcase2                = (uint16_t)SAMPLING_TIME2 + (uint16_t)TDEAD2 + (uint16_t)TRISE2,
  .Tcase3                = ((uint16_t)TDEAD2 + (uint16_t)TNOISE2 + (uint16_t)SAMPLING_TIME2) / 2u,
  .TIMx                  = ${_last_word(MC.M2_PWM_TIMER_SELECTION)},

/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.M2_USE_INTERNAL_OPAMP>
  .OPAMPParams           = &R3_3_OPAMPParamsM2,
      <#else><#-- MC.M2_USE_INTERNAL_OPAMP == false -->
  .OPAMPParams           = MC_NULL,
      </#if><#-- MC.M2_USE_INTERNAL_OPAMP -->

/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M2_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE2},
  .CompOCPBSelection     = ${MC.M2_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE2},
  .CompOCPCSelection     = ${MC.M2_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE2},
        <#if MC.M2_OCP_COMP_SRC == "DAC">
  .DAC_OCP_ASelection    = ${MC.M2_OCP_DAC_U},
  .DAC_OCP_BSelection    = ${MC.M2_OCP_DAC_V},
  .DAC_OCP_CSelection    = ${MC.M2_OCP_DAC_W},
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_${_last_word(MC.M2_OCP_DAC_CHANNEL_U, "OUT")},
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_${_last_word(MC.M2_OCP_DAC_CHANNEL_V, "OUT")},
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_${_last_word(MC.M2_OCP_DAC_CHANNEL_W, "OUT")},
        <#else><#-- MC.M2_OCP_COMP_SRC != "DAC" -->
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t)0,
  .DAC_Channel_OCPB      = (uint32_t)0,
  .DAC_Channel_OCPC      = (uint32_t)0,
        </#if><#-- MC.M2_OCP_COMP_SRC == "DAC" -->
      <#else><#-- MC.M2_OCP_TOPOLOGY != "EMBEDDED" -->
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t)0,
  .DAC_Channel_OCPB      = (uint32_t)0,
  .DAC_Channel_OCPC      = (uint32_t)0,
      </#if><#-- MC.M2_OCP_TOPOLOGY == "EMBEDDED" -->

      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION2>
  .CompOVPSelection      = OVP_SELECTION2,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE2,
        <#if MC.DAC_OVP_M2 != "NOT_USED">
  .DAC_OVP_Selection     = ${_first_word(MC.DAC_OVP_M2)},
  .DAC_Channel_OVP       = LL_DAC_CHANNEL_${_last_word(MC.DAC_OVP_M2, "CH")},
        <#else><#-- MC.DAC_OVP_M2 == "NOT_USED" -->
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t)0,
        </#if><#-- MC.DAC_OVP_M2 != "NOT_USED" -->
      <#else><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION2 == false -->
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t)0,
      </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION2 -->

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = ${MC.M2_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold     = ${MC.M2_OVPREF},

}; 
    </#if><#-- MC.M2_CS_ADC_NUM=='2' -->
    <#if MC.M2_CS_ADC_NUM=='1'>
/**
  * @brief  Current sensor parameters Motor 2 - three shunt - G4 
  */
const R3_1_Params_t R3_1_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,                         
  .IsHigherFreqTim = FREQ_RELATION2,                      
                                                          
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx            = ${MC.M2_CS_ADC_U},

  .ADCConfig = {
                 (${getADC("CFG",1,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",1,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",2,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",2,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",3,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",3,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",4,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",4,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",5,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",5,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",6,"PHASE_1",2)}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",6,"PHASE_2",2)}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M2}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
               },

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER2,
  .Tafter            = TW_AFTER2,
  .Tbefore           = TW_BEFORE_R3_1_2,
  .Tsampling         = (uint16_t)SAMPLING_TIME2,
  .Tcase2            = (uint16_t)SAMPLING_TIME2 + (uint16_t)TDEAD2 + (uint16_t)TRISE2,
  .Tcase3            = ((uint16_t)TDEAD2 + (uint16_t)TNOISE2 + (uint16_t)SAMPLING_TIME2)/2u,
  .TIMx               = ${_last_word(MC.M2_PWM_TIMER_SELECTION)},

/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.M2_USE_INTERNAL_OPAMP>
  .OPAMPParams     = &R3_3_OPAMPParamsM2,
      <#else>  
  .OPAMPParams     = MC_NULL,
      </#if>
/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M2_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M2_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE2},
  .CompOCPBSelection     = ${MC.M2_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE2},
  .CompOCPCSelection     = ${MC.M2_OCP_COMP_W},  
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE2},
        <#if MC.M2_OCP_COMP_SRC == "DAC">  
  .DAC_OCP_ASelection    = ${MC.M2_OCP_DAC_U},
  .DAC_OCP_BSelection    = ${MC.M2_OCP_DAC_V},
  .DAC_OCP_CSelection    = ${MC.M2_OCP_DAC_W},
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_${_last_word(MC.M2_OCP_DAC_CHANNEL_U, "OUT")},
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_${_last_word(MC.M2_OCP_DAC_CHANNEL_V, "OUT")},
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_${_last_word(MC.M2_OCP_DAC_CHANNEL_W, "OUT")},   
        <#else>
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t) 0,
  .DAC_Channel_OCPB      = (uint32_t) 0,
  .DAC_Channel_OCPC      = (uint32_t) 0,  
        </#if>
      <#else>  
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,   
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t) 0,
  .DAC_Channel_OCPB      = (uint32_t) 0,
  .DAC_Channel_OCPC      = (uint32_t) 0,
      </#if>

      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION2,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE2,
        <#if MC.DAC_OVP_M2 != "NOT_USED">
  .DAC_OVP_Selection     = ${_first_word(MC.DAC_OVP_M2)},
  .DAC_Channel_OVP       = LL_DAC_CHANNEL_${_last_word(MC.DAC_OVP_M2, "CH")},
        <#else>
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t) 0,
        </#if>
      <#else>
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t) 0,
      </#if>

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  ${MC.M2_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold =  ${MC.M2_OVPREF},

};
    </#if><#-- MC.MC.M2_CS_ADC_NUM=='1' -->
  </#if> <#-- MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT' --> <#-- Inside CondFamily_STM32G4 -->

  <#if MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
const ICS_Params_t ICS_ParamsM2 = 
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION2,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ${MC.M2_CS_ADC_U},
  .ADCx_2 = ${MC.M2_CS_ADC_V},
    <#if MC.M2_PWM_TIMER_SELECTION == "PWM_TIM1">
  .ADCConfig1        = (uint32_t)(MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_U} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING
                     | LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
  .ADCConfig2        = (uint32_t)(MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_V} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING
                     | LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
    <#elseif MC.M2_PWM_TIMER_SELECTION == "PWM_TIM8">
  .ADCConfig1        = (uint32_t)(MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_U} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING
                     | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,
  .ADCConfig2        = (uint32_t)(MC_ADC_CHANNEL_${MC.M2_CS_CHANNEL_V} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING
                     | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,
    </#if><#-- MC.M2_PWM_TIMER_SELECTION == "PWM_TIM1" -->

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER2,
  .TIMx              = ${_last_word(MC.M2_PWM_TIMER_SELECTION)}

};
  </#if><#-- MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->

  <#if MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
    <#if MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST">
  const ICS_Params_t ICS_ParamsM1 = 
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio         = FREQ_RATIO,
  .IsHigherFreqTim   = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ${MC.M1_CS_ADC_U},
  .ADCx_2 = ${MC.M1_CS_ADC_V},
    <#if MC.M1_PWM_TIMER_SELECTION == "PWM_TIM1">
  .ADCConfig1        = (uint32_t)(MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_U} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING
                     | LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
  .ADCConfig2        = (uint32_t)(MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_V} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING
                     | LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
    <#elseif MC.M1_PWM_TIMER_SELECTION == "PWM_TIM8">
  .ADCConfig1        = (uint32_t)(MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_U} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING
                     | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,
  .ADCConfig2        = (uint32_t)(MC_ADC_CHANNEL_${MC.M1_CS_CHANNEL_V} << ADC_JSQR_JSQ1_Pos) | LL_ADC_INJ_TRIG_EXT_RISING
                     | LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,
    </#if><#-- MC.M1_PWM_TIMER_SELECTION == "PWM_TIM1" -->

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)}

};
    <#else>
  
      <#if MC.M1_USE_INTERNAL_OPAMP == true>
/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - G4xx 
  */
R3_3_OPAMPParams_t R3_3_OPAMPParamsM1 =
{
   .OPAMPx_1 = ${MC.M1_CS_OPAMP_U},
   .OPAMPx_2 = ${MC.M1_CS_OPAMP_V},
   .OPAMPx_3 = ${MC.M1_CS_OPAMP_W},
};
      </#if>

/**
  * @brief  Current sensor parameters Motor 1 - three shunt - G4 HSO
  */
const R3_Params_t R3_ParamsM1 =
{

      <#if MC.M1_CS_ADC_NUM == "3">  <#-- Inside G4 Family HSO Speed Sensor -->
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1           = ${MC.M1_CS_ADC_U},                   
  .ADCx_2           = ${MC.M1_CS_ADC_V},
  .ADCx_3           = ${MC.M1_CS_ADC_W},
      <#else>
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1           = ${MC.M1_CS_ADC_U},                   
  .ADCx_2           = <#if MC.M1_CS_ADC_U != MC.M1_CS_ADC_V>${MC.M1_CS_ADC_V}<#else>${MC.M1_CS_ADC_W}</#if>,

      </#if>
  /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,  
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)}, 
  .TIMx_Oversample   = TIM3,

/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.M1_USE_INTERNAL_OPAMP == true>
  .OPAMPParams     = &R3_3_OPAMPParamsM1,
      <#else>
  .OPAMPParams     = MC_NULL,
      </#if>

/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
        <#if MC.M1_OCP_COMP_SRC == "DAC">
  .DAC_OCP_ASelection    = ${MC.M1_OCP_DAC_U},
  .DAC_OCP_BSelection    = ${MC.M1_OCP_DAC_V},
  .DAC_OCP_CSelection    = ${MC.M1_OCP_DAC_W},
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_U, "OUT")},
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_V, "OUT")},
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_W, "OUT")},
        <#else><#-- MC.M1_OCP_COMP_SRC != "DAC" -->
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t)0,
  .DAC_Channel_OCPB      = (uint32_t)0,
  .DAC_Channel_OCPC      = (uint32_t)0,
        </#if><#-- MC.M1_OCP_COMP_SRC == "DAC" -->
      <#else><#-- MC.M1_OCP_TOPOLOGY != "EMBEDDED" -->
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t)0,
  .DAC_Channel_OCPB      = (uint32_t)0,
  .DAC_Channel_OCPC      = (uint32_t)0,
      </#if><#-- MC.M1_OCP_TOPOLOGY == "EMBEDDED" -->
      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
        <#if MC.DAC_OVP_M1 != "NOT_USED">
  .DAC_OVP_Selection     = ${_first_word(MC.DAC_OVP_M1)},
  .DAC_Channel_OVP       = LL_DAC_CHANNEL_${_last_word(MC.DAC_OVP_M1, "CH")},
        <#else><#-- MC.DAC_OVP_M1 == "NOT_USED" -->
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t)0,
        </#if><#-- MC.DAC_OVP_M1 != "NOT_USED" -->
      <#else><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION == false -->
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t)0,
      </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION -->

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold     = ${MC.M1_OVPREF},

      <#if MC.M1_CS_ADC_NUM == "2">
          <#assign DMA_CHX=MC.M1_HSO_DMACH_ADC_U>
        <#if MC.M1_HSO_DMACH_ADC_U == MC.M1_HSO_DMACH_ADC_V>
            <#assign DMA_CHY=MC.M1_HSO_DMACH_ADC_W>
        <#else>
            <#assign DMA_CHY=MC.M1_HSO_DMACH_ADC_V>
        </#if>
      <#else> <#-- MC.M1_CS_ADC_NUM == "2" -->
          <#assign DMA_CHX=MC.M1_HSO_DMACH_ADC_U>
          <#assign DMA_CHY=MC.M1_HSO_DMACH_ADC_V>
      </#if> <#-- MC.M1_CS_ADC_NUM == "2" -->
<#assign DMA_CHZ=MC.M1_HSO_DMACH_ADC_W>
  .DMA_ADCx_1 = ${MC.M1_HSO_DMA_ADC_U}_Channel${DMA_CHX},
  .DMA_ADCx_2 = ${MC.M1_HSO_DMA_ADC_V}_Channel${DMA_CHY},
  .DMA_ADCx_3 = ${MC.M1_HSO_DMA_ADC_W}_Channel${DMA_CHZ},

};
    </#if><#--  MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST" -->
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->
  <#if MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT'><#-- Inside CondFamily_STM32G4 --->
    <#if MC.M1_SPEED_SENSOR == "HSO" ||  MC.M1_SPEED_SENSOR == "ZEST" >
      <#if MC.M1_USE_INTERNAL_OPAMP == true>
/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - G4xx 
  */
R3_3_OPAMPParams_t R3_3_OPAMPParamsM1 =
{
   .OPAMPx_1 = ${MC.M1_CS_OPAMP_U},
   .OPAMPx_2 = ${MC.M1_CS_OPAMP_V},
   .OPAMPx_3 = ${MC.M1_CS_OPAMP_W},
};
      </#if>
/**
  * @brief  Current sensor parameters Motor 1 - three shunt - G4 HSO
  */
const R3_Params_t R3_ParamsM1 =
{
      <#if MC.M1_CS_ADC_NUM == "3">  <#-- Inside G4 Family HSO Speed Sensor -->
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1           = ${MC.M1_CS_ADC_U},                   
  .ADCx_2           = ${MC.M1_CS_ADC_V},
  .ADCx_3           = ${MC.M1_CS_ADC_W},
      <#else>
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1           = ${MC.M1_CS_ADC_U},                   
  .ADCx_2           = <#if MC.M1_CS_ADC_U != MC.M1_CS_ADC_V>${MC.M1_CS_ADC_V}<#else>${MC.M1_CS_ADC_W}</#if>,
      </#if>
  /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,  
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)}, 
  .TIMx_Oversample   = TIM3,
/* Internal OPAMP common settings --------------------------------------------*/
      <#if MC.M1_USE_INTERNAL_OPAMP == true>
  .OPAMPParams     = &R3_3_OPAMPParamsM1,
      <#else>
  .OPAMPParams     = MC_NULL,
      </#if>
/* Internal COMP settings ----------------------------------------------------*/
      <#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
        <#if MC.M1_OCP_COMP_SRC == "DAC">
  .DAC_OCP_ASelection    = ${MC.M1_OCP_DAC_U},
  .DAC_OCP_BSelection    = ${MC.M1_OCP_DAC_V},
  .DAC_OCP_CSelection    = ${MC.M1_OCP_DAC_W},
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_U, "OUT")},
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_V, "OUT")},
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_W, "OUT")},
        <#else><#-- MC.M1_OCP_COMP_SRC != "DAC" -->
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t)0,
  .DAC_Channel_OCPB      = (uint32_t)0,
  .DAC_Channel_OCPC      = (uint32_t)0,
        </#if><#-- MC.M1_OCP_COMP_SRC == "DAC" -->
      <#else><#-- MC.M1_OCP_TOPOLOGY != "EMBEDDED" -->
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t)0,
  .DAC_Channel_OCPB      = (uint32_t)0,
  .DAC_Channel_OCPC      = (uint32_t)0,
      </#if><#-- MC.M1_OCP_TOPOLOGY == "EMBEDDED" -->
      <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
        <#if MC.DAC_OVP_M1 != "NOT_USED">
  .DAC_OVP_Selection     = ${_first_word(MC.DAC_OVP_M1)},
  .DAC_Channel_OVP       = LL_DAC_CHANNEL_${_last_word(MC.DAC_OVP_M1, "CH")},
        <#else><#-- MC.DAC_OVP_M1 == "NOT_USED" -->
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t)0,
        </#if><#-- MC.DAC_OVP_M1 != "NOT_USED" -->
      <#else><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION == false -->
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t)0,
      </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION -->
/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold     = ${MC.M1_OVPREF},
      <#if MC.M1_CS_ADC_NUM == "2">
          <#assign DMA_CHX=MC.M1_HSO_DMACH_ADC_U>
        <#if MC.M1_HSO_DMACH_ADC_U == MC.M1_HSO_DMACH_ADC_V>
            <#assign DMA_CHY=MC.M1_HSO_DMACH_ADC_W>
        <#else>
            <#assign DMA_CHY=MC.M1_HSO_DMACH_ADC_V>
        </#if>
      <#else> <#-- MC.M1_CS_ADC_NUM == "2" -->
          <#assign DMA_CHX=MC.M1_HSO_DMACH_ADC_U>
          <#assign DMA_CHY=MC.M1_HSO_DMACH_ADC_V>
      </#if> <#-- MC.M1_CS_ADC_NUM == "2" -->
<#assign DMA_CHZ=MC.M1_HSO_DMACH_ADC_W>
  .DMA_ADCx_1 = ${MC.M1_HSO_DMA_ADC_U}_Channel${DMA_CHX},
  .DMA_ADCx_2 = ${MC.M1_HSO_DMA_ADC_V}_Channel${DMA_CHY},
  .DMA_ADCx_3 = ${MC.M1_HSO_DMA_ADC_W}_Channel${DMA_CHZ},
};

    <#else> <#-- MC.M1_SPEED_SENSOR != "HSO" -->  
      <#if MC.M1_USE_INTERNAL_OPAMP> <#-- Inside G4 Family 3 shunts -->
/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - G4xx - Shared Resources
  * 
  */
R3_3_OPAMPParams_t R3_3_OPAMPParamsM1 =
{

  .OPAMPSelect_1 = {
                     ${getOPAMP("IP",1,"PHASE_1",1)}
                    ,${getOPAMP("IP",2,"PHASE_1",1)}
                    ,${getOPAMP("IP",3,"PHASE_1",1)}
                    ,${getOPAMP("IP",4,"PHASE_1",1)}
                    ,${getOPAMP("IP",5,"PHASE_1",1)}
                    ,${getOPAMP("IP",6,"PHASE_1",1)}
                   },
  .OPAMPSelect_2 = {
                     ${getOPAMP("IP",1,"PHASE_2",1)}
                    ,${getOPAMP("IP",2,"PHASE_2",1)}
                    ,${getOPAMP("IP",3,"PHASE_2",1)}
                    ,${getOPAMP("IP",4,"PHASE_2",1)}
                    ,${getOPAMP("IP",5,"PHASE_2",1)}
                    ,${getOPAMP("IP",6,"PHASE_2",1)}
                   },

  .OPAMPConfig1 = {
                    ${getOPAMP("CFG",1,"PHASE_1",1)}
                   ,${getOPAMP("CFG",2,"PHASE_1",1)}
                   ,${getOPAMP("CFG",3,"PHASE_1",1)}
                   ,${getOPAMP("CFG",4,"PHASE_1",1)}
                   ,${getOPAMP("CFG",5,"PHASE_1",1)}
                   ,${getOPAMP("CFG",6,"PHASE_1",1)}
  },
  .OPAMPConfig2 = {${getOPAMP("CFG",1,"PHASE_2",1)}
                  ,${getOPAMP("CFG",2,"PHASE_2",1)}
                  ,${getOPAMP("CFG",3,"PHASE_2",1)}
                  ,${getOPAMP("CFG",4,"PHASE_2",1)}
                  ,${getOPAMP("CFG",5,"PHASE_2",1)}
                  ,${getOPAMP("CFG",6,"PHASE_2",1)}
                 },
};
      </#if><#-- MC.M1_USE_INTERNAL_OPAMP -->
      <#if MC.M1_CS_ADC_NUM=='2'>
/**
  * @brief  Current sensor parameters Motor 1 - three shunt - G4 
  */
//cstat !MISRAC2012-Rule-8.4
const R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCConfig1 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",2,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",3,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",4,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",5,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",6,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
  .ADCConfig2 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",2,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",3,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",4,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",5,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(${getADC("CFG",6,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
 
  .ADCDataReg1 = {
                   ${getADC("IP", 1,"PHASE_1",1)}
                  ,${getADC("IP", 2,"PHASE_1",1)}
                  ,${getADC("IP", 3,"PHASE_1",1)}
                  ,${getADC("IP", 4,"PHASE_1",1)}
                  ,${getADC("IP", 5,"PHASE_1",1)}
                  ,${getADC("IP", 6,"PHASE_1",1)}
                 },
  .ADCDataReg2 = {
                   ${getADC("IP", 1,"PHASE_2",1)}
                  ,${getADC("IP", 2,"PHASE_2",1)}
                  ,${getADC("IP", 3,"PHASE_2",1)}
                  ,${getADC("IP", 4,"PHASE_2",1)}
                  ,${getADC("IP", 5,"PHASE_2",1)}
                  ,${getADC("IP", 6,"PHASE_2",1)}
                  },
 //cstat +MISRAC2012-Rule-12.1 +MISRAC2012-Rule-10.1_R6

  /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter     = REP_COUNTER,
  .Tafter                = TW_AFTER,
  .Tbefore               = TW_BEFORE,
  .Tsampling             = (uint16_t)SAMPLING_TIME,
  .Tcase2                = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3                = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,
  .TIMx                  = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},

/* Internal OPAMP common settings --------------------------------------------*/
        <#if MC.M1_USE_INTERNAL_OPAMP>
  .OPAMPParams           = &R3_3_OPAMPParamsM1,
        <#else><#-- MC.M1_USE_INTERNAL_OPAMP == false -->
  .OPAMPParams           = MC_NULL,
        </#if><#-- MC.M1_USE_INTERNAL_OPAMP -->

/* Internal COMP settings ----------------------------------------------------*/
        <#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
          <#if MC.M1_OCP_COMP_SRC == "DAC">
  .DAC_OCP_ASelection    = ${MC.M1_OCP_DAC_U},
  .DAC_OCP_BSelection    = ${MC.M1_OCP_DAC_V},
  .DAC_OCP_CSelection    = ${MC.M1_OCP_DAC_W},
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_U, "OUT")},
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_V, "OUT")},
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_W, "OUT")},
          <#else><#-- MC.M1_OCP_COMP_SRC != "DAC" -->
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t)0,
  .DAC_Channel_OCPB      = (uint32_t)0,
  .DAC_Channel_OCPC      = (uint32_t)0,
          </#if><#-- MC.M1_OCP_COMP_SRC == "DAC" -->
        <#else><#-- MC.M1_OCP_TOPOLOGY != "EMBEDDED" -->
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t)0,
  .DAC_Channel_OCPB      = (uint32_t)0,
  .DAC_Channel_OCPC      = (uint32_t)0,
        </#if><#-- MC.M1_OCP_TOPOLOGY == "EMBEDDED" -->
        <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
          <#if MC.DAC_OVP_M1 != "NOT_USED">
  .DAC_OVP_Selection     = ${_first_word(MC.DAC_OVP_M1)},
  .DAC_Channel_OVP       = LL_DAC_CHANNEL_${_last_word(MC.DAC_OVP_M1, "CH")},
          <#else><#-- MC.DAC_OVP_M1 == "NOT_USED" -->
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t)0,
          </#if><#-- MC.DAC_OVP_M1 != "NOT_USED" -->
        <#else><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION == false -->
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t)0,
        </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION -->

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold     = ${MC.M1_OVPREF},

};
      </#if> <#-- <#if MC.M1_CS_ADC_NUM=='2' -->
      <#if MC.M1_CS_ADC_NUM=='1'> <#-- Inside G4 Family HSO excluded -->
/**
  * @brief  Current sensor parameters Motor 1 - three shunt - G4 
  */
const R3_1_Params_t R3_1_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx            = ${MC.M1_CS_ADC_U},

  .ADCConfig = {
                 (${getADC("CFG",1,"PHASE_1")}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",1,"PHASE_2")}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",2,"PHASE_1")}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",2,"PHASE_2")}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",3,"PHASE_1")}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",3,"PHASE_2")}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",4,"PHASE_1")}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",4,"PHASE_2")}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",5,"PHASE_1")}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",5,"PHASE_2")}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                ,(${getADC("CFG",6,"PHASE_1")}U << ADC_JSQR_JSQ1_Pos)
               | (${getADC("CFG",6,"PHASE_2")}U << ADC_JSQR_JSQ2_Pos) | 1<< ADC_JSQR_JL_Pos
               | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT) 
               },

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE_R3_1,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u, 
  .TIMx               = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},

/* Internal OPAMP common settings --------------------------------------------*/
        <#if MC.M1_USE_INTERNAL_OPAMP>
  .OPAMPParams     = &R3_3_OPAMPParamsM1,
        <#else>
  .OPAMPParams     = MC_NULL,
        </#if>
/* Internal COMP settings ----------------------------------------------------*/
        <#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
          <#if MC.M1_OCP_COMP_SRC == "DAC">
  .DAC_OCP_ASelection    = ${MC.M1_OCP_DAC_U},
  .DAC_OCP_BSelection    = ${MC.M1_OCP_DAC_V},
  .DAC_OCP_CSelection    = ${MC.M1_OCP_DAC_W},
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_U, "OUT")},
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_V, "OUT")},
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_W, "OUT")},   
          <#else>
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t) 0,
  .DAC_Channel_OCPB      = (uint32_t) 0,
  .DAC_Channel_OCPC      = (uint32_t) 0,
          </#if> <#-- M1_OCP_COMP_SRC == DAC -->
        <#else> <#-- M1_OCP_TOPLOGY != EMBEDDED -->
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,   
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t) 0,
  .DAC_Channel_OCPB      = (uint32_t) 0,
  .DAC_Channel_OCPC      = (uint32_t) 0,
        </#if> <#-- MC.M1_OCP_TOPOLOGY == "EMBEDDED"-->
        <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
          <#if MC.DAC_OVP_M1 != "NOT_USED">
  .DAC_OVP_Selection     = ${_first_word(MC.DAC_OVP_M1)},
  .DAC_Channel_OVP       = LL_DAC_CHANNEL_${_last_word(MC.DAC_OVP_M1, "CH")},
          <#else>
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t) 0,
          </#if>
        <#else> <#-- ! MC.INTERNAL_OVERVOLTAGEPROTECTION -->
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t) 0,
        </#if> <#-- #if MC.INTERNAL_OVERVOLTAGEPROTECTION -->

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold =  ${MC.M1_OVPREF},

};
      </#if> <#-- <#if MC.M1_CS_ADC_NUM=='1' --> <#-- Inside CondFamily_STM32G4 -->
    </#if> <#-- <#else of if MC.M1_SPEED_SENSOR == "HSO">  -->
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT' --><#-- Inside CondFamily_STM32G4 -->
</#if><#-- CondFamily_STM32G4 -->

<#-- one shunt pahse shift********************************* -->
<#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
/**
  * @brief  Current sensor parameters Motor 1 - single shunt phase shift
  */
//cstat !MISRAC2012-Rule-8.4
const R1_Params_t R1_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx                  = ${MC.M1_CS_ADC_U},
  .IChannel              = ${getADCChannel(MC.M1_CS_ADC_U,MC.M1_CS_CHANNEL_U)},
  <#if CondFamily_STM32F0 || CondFamily_STM32G0 || CondFamily_STM32C0>
  .ISamplingTime = LL_ADC_SAMPLINGTIME_${MC.M1_CURR_SAMPLING_TIME}<#if MC.M1_CURR_SAMPLING_TIME != "1">CYCLES_5<#else>CYCLE_5</#if>,
  </#if><#-- CondFamily_STM32F0 || CondFamily_STM32G0 || CondFamily_STM32C0 -->

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter     = REP_COUNTER, 
  .TMin                  = TMIN,
  .TSample               = (uint16_t)(TBEFORE),
  .TIMx                  = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},
  .DMAx                  = ${R1_DMA_M1},
  .DMAChannelX           = ${R1_DMA_CH_M1},
  <#if HAS_TIM_6_CH == false>
  .DMASamplingPtChannelX = ${R1_DMA_CH_AUX_M1},
  </#if><#-- HAS_TIM_6_CH == false -->
  <#if HAS_ADC_INJ == false>
  .DMA_ADC_DR_ChannelX   = ${R1_DMA_CH_ADC_M1},
  </#if><#-- HAS_ADC_INJ == false -->
  .hTADConv              = (uint16_t)((ADC_SAR_CYCLES+ADC_TRIG_CONV_LATENCY_CYCLES) * (ADV_TIM_CLK_MHz/ADC_CLK_MHz)),
<#-- ADC_TRIG_CONV_LATENCY_CYCLES = 40 for G4 TODO -->

/* Internal OPAMP common settings --------------------------------------------*/
  <#if MC.M1_USE_INTERNAL_OPAMP>
  .OPAMP_Selection       = ${_filter_opamp (MC.M1_CS_OPAMP_U)},
  </#if><#-- MC.M1_USE_INTERNAL_OPAMP -->

  <#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
/* Internal COMP settings ----------------------------------------------------*/   
  .CompOCPSelection      = ${MC.M1_OCP_COMP_U},
  .CompOCPInvInput_MODE  = <#if MC.M1_OCP_COMP_SRC=="DAC">DAC_MODE<#else>NONE</#if>,
  </#if><#-- M1_OCP_TOPOLOGY == "EMBEDDED" -->
  <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
  </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION -->

  <#if MC.M1_OCP_COMP_SRC == "DAC">
/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = ${MC.M1_DAC_CURRENT_THRESHOLD},
  </#if><#-- MC.M1_OCP_COMP_SRC -->
  <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .DAC_OVP_Threshold     = ${MC.M1_OVPREF},
  </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION -->

};  
</#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
<#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
/**
  * @brief  Current sensor parameters Motor 2 - single shunt phase shift
  */
//cstat !MISRAC2012-Rule-8.4
const R1_Params_t R1_ParamsM2 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION2,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx                  = ${MC.M2_CS_ADC_U},
  .IChannel              = ${getADCChannel(MC.M2_CS_ADC_U,MC.M2_CS_CHANNEL_U)},
  <#if CondFamily_STM32F0 || CondFamily_STM32G0 || CondFamily_STM32C0>
  .ISamplingTime = LL_ADC_SAMPLINGTIME_${MC.M2_CURR_SAMPLING_TIME}<#if MC.M2_CURR_SAMPLING_TIME != "1">CYCLES_5<#else>CYCLE_5</#if>,
  </#if><#-- CondFamily_STM32F0 || CondFamily_STM32G0 || CondFamily_STM32C0 -->

/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter     = REP_COUNTER2, 
  .TMin                  = TMIN2,
  .TSample               = (uint16_t)(TBEFORE2),
  .TIMx                  = ${_last_word(MC.M2_PWM_TIMER_SELECTION)},
  .DMAx                  = ${R1_DMA_M2},
  .DMAChannelX           = ${R1_DMA_CH_M2},
  <#if HAS_TIM_6_CH == false>
  .DMASamplingPtChannelX = ${R1_DMA_CH_AUX_M2},
  </#if><#-- HAS_TIM_6_CH == false -->
  <#if HAS_ADC_INJ == false>
  .DMA_ADC_DR_ChannelX   = ${R1_DMA_CH_ADC_M2},
  </#if><#-- HAS_ADC_INJ == false -->
  .hTADConv              = (uint16_t)((ADC_SAR_CYCLES+ADC_TRIG_CONV_LATENCY_CYCLES) * (ADV_TIM_CLK_MHz/ADC_CLK_MHz)),
<#-- ADC_TRIG_CONV_LATENCY_CYCLES = 40 for G4 TODO -->

/* Internal OPAMP common settings --------------------------------------------*/
  <#if MC.M2_USE_INTERNAL_OPAMP>
  .OPAMP_Selection       = ${_filter_opamp (MC.M2_CS_OPAMP_U)},
  </#if><#-- MC.M2_USE_INTERNAL_OPAMP -->

<#if MC.M2_OCP_TOPOLOGY == "EMBEDDED">
/* Internal COMP settings ----------------------------------------------------*/
  .CompOCPSelection      = ${MC.M2_OCP_COMP_U},
  .CompOCPInvInput_MODE  = <#if MC.M2_OCP_COMP_SRC=="DAC">DAC_MODE<#else>NONE</#if>,
  </#if><#-- MC.M2_OCP_TOPOLOGY -->
  <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
  </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION -->

/* DAC settings --------------------------------------------------------------*/
  <#if MC.M2_OCP_COMP_SRC=="DAC">
  .DAC_OCP_Threshold     = ${MC.M2_DAC_CURRENT_THRESHOLD},
  </#if><#-- MC.M2_OCP_COMP_SRC=="DAC" -->
  <#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .DAC_OVP_Threshold     = ${MC.M2_OVPREF},
  </#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION -->

};  
</#if><#-- ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
<#-- one shunt pahse shift********************************* -->

<#if MC.PFC_ENABLED == true>
const PFC_Parameters_t PFC_Params = 
{
    <#if CondFamily_STM32F3>
  .ADCx                     = ADC4,
  .TIMx                     = TIM16,
  .TIMx_2                   = TIM4,
  </#if><#-- CondFamily_STM32F3 -->
  .wCPUFreq                 = SYSCLK_FREQ,
  .wPWMFreq                 = PFC_PWMFREQ,
  .hPWMARR                  = (SYSCLK_FREQ / PFC_PWMFREQ),
  .bFaultPolarity           = PFC_FAULTPOLARITY,
  .hFaultPort               = PFC_FAULTPORT,
  .hFaultPin                = PFC_FAULTPIN,
  .bCurrentLoopRate         = (uint8_t)(PFC_PWMFREQ / PFC_CURRCTRLFREQUENCY),
  .bVoltageLoopRate         = (uint8_t)(SYS_TICK_FREQUENCY / PFC_VOLTCTRLFREQUENCY),
  .hNominalCurrent          = (uint16_t)PFC_NOMINALCURRENTS16A,
  .hMainsFreqLowTh          = PFC_MAINSFREQLOWTHR,
  .hMainsFreqHiTh           = PFC_MAINSFREQHITHR,
  .OutputPowerActivation    = PFC_OUTPUTPOWERACTIVATION,
  .OutputPowerDeActivation  = PFC_OUTPUTPOWERDEACTIVATION,
  .SWOverVoltageTh          = PFC_SWOVERVOLTAGETH,
  .hPropDelayOnTimCk        = (uint16_t)(PFC_PROPDELAYON / PFC_TIMCK_NS),
  .hPropDelayOffTimCk       = (uint16_t)(PFC_PROPDELAYOFF / PFC_TIMCK_NS),
  .hADCSamplingTimeTimCk    = (uint16_t)(SYSCLK_FREQ / (ADC_CLK_MHz * 1000000.0) * PFC_ISAMPLINGTIMEREAL),
  .hADCConversionTimeTimCk  = (uint16_t)(SYSCLK_FREQ / (ADC_CLK_MHz * 1000000.0) * 12),
  .hADCLatencyTimeTimCk     = (uint16_t)(SYSCLK_FREQ / (ADC_CLK_MHz * 1000000.0) * 3),
  .hMainsConversionFactor   = (uint16_t)(65535.0 / (3.3 * PFC_VMAINS_PARTITIONING_FACTOR)),
  .hCurrentConversionFactor = (uint16_t)((PFC_SHUNTRESISTOR * PFC_AMPLGAIN * 65536.0) / 3.3),
    <#if CondFamily_STM32F3>
  .hDAC_OCP_Threshold       = (uint16_t)PFC_OCP_REF,
  </#if><#-- CondFamily_STM32F3 -->
};
</#if><#-- MC.PFC_ENABLED == true -->

<#if (MC.MOTOR_PROFILER == true) && ( MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST")>

/*** Motor Profiler ***/

SCC_Params_t SCC_Params = 
{
  {
    .FrequencyHz = TF_REGULATION_RATE,
  },
  .fRshunt                 = RSHUNT,
  .fAmplificationGain      = AMPLIFICATION_GAIN,
  .fVbusConvFactor         = BUS_VOLTAGE_CONVERSION_FACTOR,
  .fVbusPartitioningFactor = VBUS_PARTITIONING_FACTOR,

  .fRVNK                   = (float)(RESISTOR_OFFSET),

  .fRSMeasCurrLevelMax     = (float)(DC_CURRENT_RS_MEAS),

  .hDutyRampDuration       = (uint16_t)8000,
  
  .hAlignmentDuration      = (uint16_t)(1000),
  .hRSDetectionDuration    = (uint16_t)(500),
  .fLdLqRatio              = (float)(LDLQ_RATIO),
  .fCurrentBW              = (float)(CURRENT_REGULATOR_BANDWIDTH),
  .bPBCharacterization     = false,

  .wNominalSpeed           = MOTOR_MAX_SPEED_RPM,
  .hPWMFreqHz              = (uint16_t)(PWM_FREQUENCY),
  .bFOCRepRate             = (uint8_t)(REGULATION_EXECUTION_RATE),
  .fMCUPowerSupply         = (float)ADC_REFERENCE_VOLTAGE,
  .IThreshold              = I_THRESHOLD

};

OTT_Params_t OTT_Params =
{
  {
    .FrequencyHz        = MEDIUM_FREQUENCY_TASK_RATE,         /*!< Frequency expressed in Hz at which the user
                                                                  clocks the OTT calling OTT_MF method */
  },
  .fBWdef               = (float)(SPEED_REGULATOR_BANDWIDTH), /*!< Default bandwidth of speed regulator.*/
  .fMeasWin             = 1.0f,                               /*!< Duration of measurement window for speed and
                                                                  current Iq, expressed in seconds.*/
  .bPolesPairs          = POLE_PAIR_NUM,                      /*!< Number of motor poles pairs.*/
  .hMaxPositiveTorque   = (int16_t)NOMINAL_CURRENT,           /*!< Maximum positive value of motor
                                                                   torque. This value represents
                                                                   actually the maximum Iq current
                                                                   expressed in digit.*/
  .fCurrtRegStabTimeSec = 10.0f,                              /*!< Current regulation stabilization time in seconds.*/
  .fOttLowSpeedPerc     = 0.6f,                               /*!< OTT lower speed percentage.*/
  .fOttHighSpeedPerc    = 0.8f,                               /*!< OTT higher speed percentage.*/
  .fSpeedStabTimeSec    = 20.0f,                              /*!< Speed stabilization time in seconds.*/
  .fTimeOutSec          = 10.0f,                              /*!< Timeout for speed stabilization.*/
  .fSpeedMargin         = 0.90f,                              /*!< Speed margin percentage to validate speed ctrl.*/
  .wNominalSpeed        = MOTOR_MAX_SPEED_RPM,                /*!< Nominal speed set by the user expressed in RPM.*/
  .spdKp                = MP_KP,                              /*!< Initial KP factor of the speed regulator to be
                                                                   tuned.*/
  .spdKi                = MP_KI,                              /*!< Initial KI factor of the speed regulator to be 
                                                                   tuned.*/
  .spdKs                = 0.1f,                               /*!< Initial antiwindup factor of the speed regulator to
                                                                   be tuned.*/
  .fRshunt              = (float)(RSHUNT),                    /*!< Value of shunt resistor.*/
  .fAmplificationGain   = (float)(AMPLIFICATION_GAIN)         /*!< Current sensing amplification gain.*/

};
</#if><#-- MC.MOTOR_PROFILER == true -->

<#if MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST">
const FLASH_Params_t  flashParams =
{
  .motor = {
    .polePairs = POLE_PAIR_NUM,
    .ratedFlux = MOTOR_RATED_FLUX,
    .rs = RS,
    .rsSkinFactor = MOTOR_RS_SKINFACTOR,
    .ls = LS,
    .maxCurrent = NOMINAL_CURRENT_A,
    .mass_copper_kg = MOTOR_MASS_COPPER_KG,
    .cooling_tau_s = MOTOR_COOLINGTAU_S, 
    .name = MOTOR_NAME,
  },
  .polPulse =
  {
    .N  = NB_PULSE_PERIODS,
    .Nd = NB_DECAY_PERIODS,
    .PulseCurrentGoal = PULSE_CURRENT_GOAL,   
  },
  <#if MC.M1_SPEED_SENSOR == "ZEST">
  .zest = 
  {
    .zestThresholdFreqHz = MOTOR_ZEST_THRESHOLD_FREQ_HZ,
    .zestInjectFreq = MOTOR_ZEST_INJECT_FREQ,
    .zestInjectD = MOTOR_ZEST_INJECT_D,
    .zestGainD = MOTOR_ZEST_GAIN_D,
    .zestGainQ = MOTOR_ZEST_GAIN_Q,
  },
  <#else>
  .zest = 
  {
    .zestThresholdFreqHz = 0,
    .zestInjectFreq = 0,
    .zestInjectD = 0,
    .zestGainD = 0,
    .zestGainQ = 0,
  },
  </#if>
  .PIDSpeed = 
  {
    .pidSpdKp = PID_SPD_KP,
    .pidSpdKi = PID_SPD_KI,
  },
  .board = 
  {
   .limitOverVoltage = BOARD_LIMIT_OVERVOLTAGE,
   .limitRegenHigh = BOARD_LIMIT_REGEN_HIGH,
   .limitRegenLow = BOARD_LIMIT_REGEN_LOW,
   .limitAccelHigh = BOARD_LIMIT_ACCEL_HIGH,
   .limitAccelLow = BOARD_LIMIT_ACCEL_LOW,
   .limitUnderVoltage = BOARD_LIMIT_UNDERVOLTAGE,
   .maxModulationIndex = BOARD_MAX_MODULATION,
   .softOverCurrentTrip = BOARD_SOFT_OVERCURRENT_TRIP,
},
  .KSampleDelay = KSAMPLE_DELAY,
  .throttle = 
  {
    .offset = THROTTLE_OFFSET, /*TODO: Tobe defined */
	  .gain = THROTTLE_GAIN, /*TODO: Tobe defined */
	  .speedMaxRPM = THROTTLE_SPEED_MAX_RPM,
    .direction = 1,
  },
  .scale = 
  {
    .voltage = VOLTAGE_SCALE,
    .current = CURRENT_SCALE,
    .frequency = FREQUENCY_SCALE
  },
};

  <#if MC.M1_SPEED_SENSOR == "ZEST">
ZEST_Params ZeST_params_M1 =
{
  .backgroundFreq_Hz = SPEED_LOOP_FREQUENCY_HZ,   /* Frequency for ZEST_runBackground() */
  .isrFreq_Hz = (PWM_FREQUENCY / REGULATION_EXECUTION_RATE), /* Frequency of ZEST_run() calls */
  .speedPole_rps = SPEED_POLE_RPS,
};
  </#if>

const MotorConfig_reg_t *motorParams = &flashParams.motor;
const zestFlashParams_t *zestParams = &flashParams.zest; 
const boardFlashParams_t *boardParams = &flashParams.board;
const scaleFlashParams_t *scaleParams = &flashParams.scale;
const throttleParams_t *throttleParams = &flashParams.throttle;
const float *KSampleDelayParams = &flashParams.KSampleDelay;
const PIDSpeedFlashParams_t *PIDSpeedParams = &flashParams.PIDSpeed;

</#if>


  <#if MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST">
ScaleParams_t scaleParams_M1 =
{
 .voltage = NOMINAL_BUS_VOLTAGE_V/(1.73205 * 32767), /* sqrt(3) = 1.73205 */
 .current = CURRENT_CONV_FACTOR_INV,
 .frequency = (1.15 * MAX_APPLICATION_SPEED_UNIT * U_RPM)/(32768* SPEED_UNIT)
};
</#if> <#-- MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST"-->
<#if MC.DRIVE_NUMBER != "1">
      <#if MC.M2_SPEED_SENSOR != "HSO" && MC.M2_SPEED_SENSOR != "ZEST">
ScaleParams_t scaleParams_M2 =
{
 .voltage = NOMINAL_BUS_VOLTAGE_V2/(1.73205 * 32767), /* sqrt(3) = 1.73205 */
 .current = CURRENT_CONV_FACTOR_INV2,
 .frequency = (1.15 * MAX_APPLICATION_SPEED_UNIT2 * U_RPM)/(32768* SPEED_UNIT)
};
  </#if> <#-- MC.M2_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST"-->
</#if><#-- MC.DRIVE_NUMBER > 1 -->


<#if MC.ESC_ENABLE>
const ESC_Params_t ESC_ParamsM1 =
{
  .Command_TIM        = TIM2,
  .Motor_TIM          = TIM1,
  .ARMING_TIME        = 200,
  .PWM_TURNOFF_MAX    = 500,
  .TURNOFF_TIME_MAX   = 500,
  .Ton_max            = ESC_TON_MAX,               /*!<  Maximum ton value for PWM (by default is 1800 us) */
  .Ton_min            = ESC_TON_MIN,               /*!<  Minimum ton value for PWM (by default is 1080 us) */ 
  .Ton_arming         = ESC_TON_ARMING,            /*!<  Minimum value to start the arming of PWM */ 
  .delta_Ton_max      = ESC_TON_MAX - ESC_TON_MIN,
  .speed_max_valueRPM = MOTOR_MAX_SPEED_RPM,       /*!< Maximum value for speed reference from Workbench */
  .speed_min_valueRPM = 1000,                      /*!< Set the minimum value for speed reference */
  .motor              = M1,
};
</#if><#-- MC.ESC_ENABLE -->

/* USER CODE BEGIN Additional parameters */


/* USER CODE END Additional parameters */  

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

