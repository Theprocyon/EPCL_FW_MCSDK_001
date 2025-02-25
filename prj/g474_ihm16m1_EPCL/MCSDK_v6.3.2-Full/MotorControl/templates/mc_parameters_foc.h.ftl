<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    mc_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides declarations of HW parameters specific to the 
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
#ifndef MC_PARAMETERS_H
#define MC_PARAMETERS_H

#include "mc_interface.h"  
<#if MC.ESC_ENABLE == true>
#include "esc.h"
</#if><#-- MC.ESC_ENABLE == true -->
<#if CondFamily_STM32F4>
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
#include "r3_1_f4xx_pwm_curr_fdbk.h"
  </#if><#--  (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
#include "r3_2_f4xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
  <#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
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
<#if CondFamily_STM32L4><#-- CondFamily_STM32L4 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
#include "ics_l4xx_pwm_curr_fdbk.h"
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) >
#include "r3_2_l4xx_pwm_curr_fdbk.h"
  </#if><#--((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2'))  -->
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
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) >
#include "r3_2_f7xx_pwm_curr_fdbk.h"
  </#if><#--((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2'))  -->
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
#include "r3_1_f7xx_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
</#if><#-- CondFamily_STM32F7 --->
<#if CondFamily_STM32H7><#-- CondFamily_STM32H7 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) >
#include "r3_2_h7xx_pwm_curr_fdbk.h"
  </#if><#--((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2'))  -->
  </#if><#-- CondFamily_STM32H7 --->
<#if CondFamily_STM32H5 > <#-- CondFamily_STM32H5 -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) >
#include "r1_ps_pwm_curr_fdbk.h"  
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if  ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )) >
#include "r3_2_h5xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )) -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) >
#include "r3_1_h5xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
</#if> <#-- CondFamily_STM32H5 -->
<#if CondFamily_STM32F3><#-- CondFamily_STM32F3 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"    
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
#include "ics_f30x_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
#include "r3_2_f30x_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')
    || (MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1'))>
#include "r3_1_f30x_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
</#if><#-- CondFamily_STM32F3 --->
<#if CondFamily_STM32G4><#-- CondFamily_STM32G4 --->
  <#if MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST">
    <#if MC.M1_SPEED_SENSOR == "ZEST">
#include "zest.h"
    </#if>
#include "flash_parameters.h"
#include "r3_g4xx_pwm_curr_fdbk.h" 
  <#else>
    <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
      || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) >
#include "r1_ps_pwm_curr_fdbk.h" 
    </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
            || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
    <#if (((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2'))  
       || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')))>
#include "r3_2_g4xx_pwm_curr_fdbk.h"
    </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) 
            || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
    <#if (((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1'))  
       || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1'))) >
#include "r3_1_g4xx_pwm_curr_fdbk.h"
    </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')) 
            || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1')) -->
    <#if (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')  >
#include "ics_g4xx_pwm_curr_fdbk.h"
    </#if><#-- (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
  </#if>
</#if> <#-- CondFamily_STM32G4 --->
<#if (MC.MOTOR_PROFILER == true) && (MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST")>
#include "mp_self_com_ctrl.h"
#include "mp_one_touch_tuning.h"
</#if><#-- MC.MOTOR_PROFILER == true -->
<#if CondFamily_STM32G0>
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
#include "r3_g0xx_pwm_curr_fdbk.h"
  <#elseif ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
</#if><#-- CondFamily_STM32G0 --->
<#if CondFamily_STM32C0>
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
#include "r3_c0xx_pwm_curr_fdbk.h"
  <#elseif ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
</#if><#-- CondFamily_STM32C0 --->

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

<#if CondFamily_STM32F4>
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
extern R3_1_Params_t R3_1_ParamsM1;
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
  <#if (MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1')>
extern R3_1_Params_t R3_1_ParamsM2;
  </#if><#-- (MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1') -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) >
extern const R3_2_Params_t R3_2_ParamsM1;
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2'))  -->
  <#if ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
extern const R3_2_Params_t R3_2_ParamsM2;
  </#if><#-- ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM1;
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM2;
  </#if><#-- ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
extern const ICS_Params_t ICS_ParamsM1;
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->
  <#if (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
extern const ICS_Params_t ICS_ParamsM2;
  </#if><#-- (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
</#if><#-- CondFamily_STM32F4 --->
<#if CondFamily_STM32F0>
  <#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
extern const R3_1_Params_t R3_1_Params;
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1') -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM1;
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if><#-- CondFamily_STM32F0 --->
<#if CondFamily_STM32F3><#-- CondFamily_STM32F3 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM1;
  <#elseif MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
extern const ICS_Params_t ICS_ParamsM1;
  <#elseif ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2'))>
    <#if MC.M1_USE_INTERNAL_OPAMP>
extern const R3_2_OPAMPParams_t R3_2_OPAMPParamsM1;
    </#if><#-- MC.M1_USE_INTERNAL_OPAMP -->
extern const R3_2_Params_t R3_2_ParamsM1;
  <#elseif (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
extern const R3_1_Params_t R3_1_ParamsM1;
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM2;
  <#elseif (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')> 
extern const ICS_Params_t ICS_ParamsM2;
  <#elseif ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
    <#if MC.M2_USE_INTERNAL_OPAMP>
extern const R3_2_OPAMPParams_t R3_2_OPAMPParamsM2;
    </#if><#-- MC.M2_USE_INTERNAL_OPAMP -->
extern const R3_2_Params_t R3_2_ParamsM2;
  <#elseif (MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1')> 
extern const R3_1_Params_t R3_1_ParamsM2;
  </#if><#-- ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if><#-- CondFamily_STM32F3 --->
<#if CondFamily_STM32L4 ><#-- CondFamily_STM32L4 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM1;
  <#elseif MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' >
extern const ICS_Params_t ICS_ParamsM1;
  <#elseif  (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
extern const R3_1_Params_t R3_1_ParamsM1;
  <#elseif ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) >
extern const R3_2_Params_t R3_2_ParamsM1;
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if> <#-- CondFamily_STM32L4 --->
<#if CondFamily_STM32H5 > <#-- CondFamily_STM32H5 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM1;    
  <#elseif MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' >
extern const ICS_Params_t ICS_ParamsM1;
  <#elseif ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
extern const R3_1_Params_t R3_1_ParamsM1;
  <#elseif ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))>
extern const R3_2_Params_t R3_2_ParamsM1;
  </#if> <#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if> <#-- CondFamily_STM32H5 --->
<#if CondFamily_STM32F7><#-- CondFamily_STM32F7 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM1;
  <#elseif MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
extern const ICS_Params_t ICS_ParamsM1;
  <#elseif (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
extern const R3_1_Params_t R3_1_ParamsM1;
  <#elseif ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) >
extern const R3_2_Params_t R3_2_ParamsM1;
  </#if><#--  ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if><#-- CondFamily_STM32F7 --->
<#if CondFamily_STM32H7><#-- CondFamily_STM32H7 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM1;
  <#elseif MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
extern const ICS_Params_t ICS_ParamsM1;
  <#elseif (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
extern const R3_1_Params_t R3_1_ParamsM1;
  <#elseif ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) >
extern const R3_2_Params_t R3_2_ParamsM1;
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if><#-- CondFamily_STM32H7 --->
<#if CondFamily_STM32G4><#-- CondFamily_STM32G4 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM1; 
  <#elseif ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2'))>
    <#if MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST" >    
extern const R3_2_Params_t R3_2_ParamsM1;
    </#if> <#-- MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST" --->
  <#elseif  ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1'))>
extern const R3_1_Params_t R3_1_ParamsM1;
  <#elseif MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
    <#if MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST" > 
extern const ICS_Params_t ICS_ParamsM1;
    </#if> <#-- MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST" --->
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM2; 
  <#elseif ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
extern R3_2_Params_t R3_2_ParamsM2; 
  <#elseif (MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1')>
extern const R3_1_Params_t R3_1_ParamsM2;
  <#elseif (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
extern const ICS_Params_t ICS_ParamsM2;
  </#if><#-- ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if><#-- CondFamily_STM32G4 --->
<#if CondFamily_STM32G0><#-- CondFamily_STM32G0 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM1;
  <#elseif (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')>
extern const R3_1_Params_t R3_1_Params; 
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if><#-- CondFamily_STM32G0 --->
<#if CondFamily_STM32C0><#-- CondFamily_STM32C0 --->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern const R1_Params_t R1_ParamsM1;
  <#elseif ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
extern const R3_1_Params_t R3_1_Params; 
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if><#-- CondFamily_STM32C0 --->
<#if MC.PFC_ENABLED == true>
#include "pfc.h"
extern const PFC_Parameters_t PFC_Params;
</#if><#-- MC.PFC_ENABLED == true -->
<#if (MC.MOTOR_PROFILER == true) && (MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST")>
extern SCC_Params_t SCC_Params;
extern OTT_Params_t OTT_Params;
</#if><#--MC.MOTOR_PROFILER == true -->
<#if MC.ESC_ENABLE == true>
extern const ESC_Params_t ESC_ParamsM1;
</#if><#-- MC.ESC_ENABLE == true -->
<#if MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST">
  <#if MC.M1_SPEED_SENSOR == "ZEST">
extern ZEST_Params ZeST_params_M1;
extern const zestFlashParams_t *zestParams;
  </#if>
extern R3_Params_t R3_ParamsM1; 
extern const FLASH_Params_t flashParams;
extern const MotorConfig_reg_t *motorParams;
extern const boardFlashParams_t *boardParams;
extern const scaleFlashParams_t *scaleParams;
extern const throttleParams_t *throttleParams;
extern const float *KSampleDelayParams;
extern const PIDSpeedFlashParams_t *PIDSpeedParams;
</#if>
<#if MC.M1_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST">
extern ScaleParams_t scaleParams_M1;
</#if> <#-- MC.M1_SPEED_SENSOR != "HSO" -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_SPEED_SENSOR != "HSO" && MC.M1_SPEED_SENSOR != "ZEST">
extern ScaleParams_t scaleParams_M2;
  </#if> <#-- MC.M2_SPEED_SENSOR != "HSO" -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

/* USER CODE BEGIN Additional extern */

/* USER CODE END Additional extern */  

#endif /* MC_PARAMETERS_H */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
