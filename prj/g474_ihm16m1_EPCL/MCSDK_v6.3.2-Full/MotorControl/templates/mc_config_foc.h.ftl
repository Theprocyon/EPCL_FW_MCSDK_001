<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    mc_config.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler 
  *          structures declarations.
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
  
#ifndef MC_CONFIG_H
#define MC_CONFIG_H

#include "pid_regulator.h"
#include "speed_torq_ctrl.h"
#include "revup_ctrl.h"
#include "mc_config_common.h"
#include "pwm_curr_fdbk.h"
<#if MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true || MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true>
#include "feed_forward_ctrl.h"
</#if><#-- MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true || MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true -->
<#if MC.M1_FLUX_WEAKENING_ENABLING == true || MC.M2_FLUX_WEAKENING_ENABLING == true>
#include "flux_weakening_ctrl.h"
</#if><#-- MC.M1_FLUX_WEAKENING_ENABLING == true || MC.M2_FLUX_WEAKENING_ENABLING == true -->
<#if MC.M1_POSITION_CTRL_ENABLING == true || MC.M2_POSITION_CTRL_ENABLING == true>
#include "trajectory_ctrl.h"
</#if><#-- MC.M1_POSITION_CTRL_ENABLING == true || MC.M2_POSITION_CTRL_ENABLING == true -->
#include "pqd_motor_power_measurement.h"
<#if MC.USE_STGAP1S>
#include "gap_gate_driver_ctrl.h"
</#if><#-- MC.USE_STGAP1S -->

<#-- Specific to F3 family usage -->
<#if CondFamily_STM32F3 && (((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))
  || ((MC.M2_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M2_CS_ADC_NUM == '1' )))>
#include "r3_1_f30x_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F3 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
<#if CondFamily_STM32F3 
  && (((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
  || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')))>
#include "r1_ps_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F3 && (((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || -->
<#if CondFamily_STM32F3 && (((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')))>
#include "r3_2_f30x_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F3 && (((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))) -->
<#if CondFamily_STM32F3 && ((MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS'))>
#include "ics_f30x_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F3 && ((MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')) -->

<#-- Specific to F4 family usage -->
<#if CondFamily_STM32F4 && (((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))
  || ((MC.M2_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M2_CS_ADC_NUM == '1' )))>
#include "r3_1_f4xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F4 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
<#if CondFamily_STM32F4 
  && (((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
  || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')))>
#include "r1_ps_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F4 && (((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || -->
<#if CondFamily_STM32F4 && (((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')))>
#include "r3_2_f4xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F4 && (((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )) 
    || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))) -->
<#if CondFamily_STM32F4 && ((MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS'))>
#include "ics_f4xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F4 && ((MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')) -->

<#-- Specific to G0 family usage -->
<#if CondFamily_STM32G0>
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
#include "r3_g0xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
   <#if (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
#include "ics_g0xx_pwm_curr_fdbk.h"
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->
</#if><#-- CondFamily_STM32G0 -->
<#-- Specific to C0 family usage -->
<#if CondFamily_STM32C0>
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
#include "r3_c0xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
  <#if MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->
</#if><#-- CondFamily_STM32C0 -->
<#-- Specific to L4 family usage -->
<#if CondFamily_STM32L4 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
#include "r3_1_l4xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32L4 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
<#if CondFamily_STM32L4 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))>
#include "r3_2_l4xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32L4 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )) -->
<#if CondFamily_STM32L4 && ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') 
  || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32L4 && ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') 
  || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
<#if CondFamily_STM32L4 && (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
#include "ics_l4xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32L4 && (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
<#-- Specific to F7 family usage -->
<#if CondFamily_STM32F7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
#include "r3_1_f7xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
<#if CondFamily_STM32F7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))>
#include "r3_2_f7xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )) -->
<#if CondFamily_STM32F7 && ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') 
  || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F7 && ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') 
  || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
<#if CondFamily_STM32F7 && (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
#include "ics_f7xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F7 && (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
<#-- Specific to H7 family usage -->
<#if CondFamily_STM32H7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
#include "r3_1_h7xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32H7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
<#if CondFamily_STM32H7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))>
#include "r3_2_h7xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32H7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )) -->
<#if CondFamily_STM32H7 && (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
#include "ics_h7xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32H7 && (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
<#-- Specific to F0 family usage -->
<#if CondFamily_STM32F0 && ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') 
  || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"      
</#if><#-- CondFamily_STM32F0 && ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') 
  || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
<#if CondFamily_STM32F0 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
#include "r3_f0xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32F0 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
<#-- Specific to H5 family usage -->
<#if CondFamily_STM32H5 && ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))> 
#include "r1_ps_pwm_curr_fdbk.h"      
</#if><#-- CondFamily_STM32H5 && ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
<#if CondFamily_STM32H5 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
#include "r3_1_h5xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32H5 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
<#if CondFamily_STM32H5 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))> 
#include "r3_2_h5xx_pwm_curr_fdbk.h"
</#if><#-- CondFamily_STM32H5 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )) -->
<#-- Specific to G4 family usage -->
<#if CondFamily_STM32G4>
  <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#include "r1_ps_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
#include "r3_2_g4xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
  <#if  ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='1')) || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='1'))  >
#include "r3_1_g4xx_pwm_curr_fdbk.h"
  </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M1_CS_ADC_NUM=='2')) || ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
   <#if (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
#include "ics_g4xx_pwm_curr_fdbk.h"
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') || (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
</#if><#-- CondFamily_STM32G4 -->
<#-- MTPA feature usage -->
<#if MC.M1_MTPA_ENABLING == true || MC.M2_MTPA_ENABLING == true>
#include "max_torque_per_ampere.h"
</#if><#-- MC.M1_MTPA_ENABLING == true || MC.M2_MTPA_ENABLING == true -->
<#-- ICL feature usage -->
<#if MC.M1_ICL_ENABLED == true || MC.M2_ICL_ENABLED == true>
#include "inrush_current_limiter.h"
</#if><#-- MC.M1_ICL_ENABLED == true || MC.M2_ICL_ENABLED == true -->
<#-- Open Loop feature usage -->
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
#include "open_loop.h"
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
<#-- Position sensors feature usage -->
#include "ramp_ext_mngr.h"
#include "circle_limitation.h"
<#if (MC.M1_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") || 
     (MC.M1_SPEED_SENSOR == "STO_CORDIC") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
#include "sto_speed_pos_fdbk.h"
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") || 
           (MC.M1_SPEED_SENSOR == "STO_CORDIC") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->
<#if (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||
     (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_PLL")>
#include "sto_pll_speed_pos_fdbk.h"
</#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||
           (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_PLL") -->
<#if (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") ||
     (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")>
#include "sto_cordic_speed_pos_fdbk.h"
</#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") ||
           (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") -->
<#-- PFC feature usage -->
<#if MC.PFC_ENABLED == true>
#include "pfc.h"
</#if><#-- MC.PFC_ENABLED == true -->
<#if (M1_HALL_SENSOR == true) || (M2_HALL_SENSOR == true)>
#include "hall_speed_pos_fdbk.h"
</#if><#-- (M1_HALL_SENSOR == true) || (M2_HALL_SENSOR == true) -->

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;
<#-- Flux Weakening feature usage -->
<#if MC.M1_FLUX_WEAKENING_ENABLING == true>
extern PID_Handle_t PIDFluxWeakeningHandle_M1;
extern FW_Handle_t FW_M1;
</#if><#-- MC.M1_FLUX_WEAKENING_ENABLING == true -->
<#if MC.M1_POSITION_CTRL_ENABLING == true>
extern PID_Handle_t PID_PosParamsM1;
extern PosCtrl_Handle_t PosCtrlM1;
</#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->
<#if (CondFamily_STM32H7 == false) && ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') 
  || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern PWMC_R1_Handle_t PWM_Handle_M1;
</#if><#-- (CondFamily_STM32H7 == false) && ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') 
  || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
<#if CondFamily_STM32H7 == false && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' ))>
extern PWMC_R3_1_Handle_t PWM_Handle_M1;
</#if><#-- CondFamily_STM32H7 == false && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '1' )) -->
<#if (MC.DRIVE_NUMBER != "1") && ((MC.M2_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M2_CS_ADC_NUM == '1' ))>
extern PWMC_R3_1_Handle_t PWM_Handle_M2;
</#if><#-- (MC.DRIVE_NUMBER != "1") && ((MC.M2_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M2_CS_ADC_NUM == '1' )) -->
<#if (CondFamily_STM32F0 == false || CondFamily_STM32G0 == false || CondFamily_STM32C0 == false || CondFamily_STM32H7 == false)
     && (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
extern PWMC_ICS_Handle_t PWM_Handle_M1;
</#if><#-- (CondFamily_STM32F0 == false || CondFamily_STM32G0 == false || CondFamily_STM32C0 == false || CondFamily_STM32H7 == false)
          && (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
<#if ((CondFamily_STM32F4 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )))
   || (CondFamily_STM32F3 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )))
   || (CondFamily_STM32L4 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )))
   || (CondFamily_STM32F7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))) 
   || (CondFamily_STM32H5 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))) 
   || (CondFamily_STM32H7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))) 
   || (CondFamily_STM32G4 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))))>
extern PWMC_R3_2_Handle_t PWM_Handle_M1;
</#if><#-- ((CondFamily_STM32F4 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )))
         || (CondFamily_STM32F3 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )))
         || (CondFamily_STM32L4 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )))
         || (CondFamily_STM32F7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))) 
         || (CondFamily_STM32H5 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))) 
         || (CondFamily_STM32H7 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' ))) 
         || (CondFamily_STM32G4 && ((MC.M1_CURRENT_SENSING_TOPO == 'THREE_SHUNT') && ( MC.M1_CS_ADC_NUM == '2' )))) -->
<#-- Specific to F4 family usage -->
<#if CondFamily_STM32F4>
  <#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern PWMC_R1_Handle_t PWM_Handle_M2;
  <#elseif MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
extern PWMC_ICS_Handle_t PWM_Handle_M2;
  </#if><#--  ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
</#if><#-- CondFamily_STM32F4 -->
<#-- Specific to G4 family usage -->
<#if CondFamily_STM32G4>
  <#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern PWMC_R1_Handle_t PWM_Handle_M2;
  </#if><#-- ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
extern PWMC_ICS_Handle_t PWM_Handle_M2;
  </#if><#-- MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS' -->
</#if><#-- CondFamily_STM32G4 -->

<#-- Specific to Dual Drive feature usage -->
<#if MC.DRIVE_NUMBER != "1">
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM2;
extern PID_Handle_t PIDSpeedHandle_M2;
extern PID_Handle_t PIDIqHandle_M2;
extern PID_Handle_t PIDIdHandle_M2;
  <#if (MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true) >
extern RegConv_t TempRegConv_M2;
  </#if><#--(MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true) -->
extern NTC_Handle_t TempSensor_M2;
  <#if MC.M2_FLUX_WEAKENING_ENABLING == true>
extern PID_Handle_t PIDFluxWeakeningHandle_M2;
extern FW_Handle_t FW_M2;
  </#if><#-- MC.M2_FLUX_WEAKENING_ENABLING == true -->
  <#if MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true>
extern FF_Handle_t FF_M2;
  </#if><#-- MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true -->
  <#if MC.M2_POSITION_CTRL_ENABLING == true>
extern PID_Handle_t PID_PosParamsM2;
extern PosCtrl_Handle_t PosCtrlM2;
  </#if><#-- MC.M2_POSITION_CTRL_ENABLING == true -->
  <#if CondFamily_STM32F3 && ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
extern PWMC_R1_Handle_t PWM_Handle_M2;
  </#if><#-- CondFamily_STM32F3 && ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
  <#if ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2'))>
extern PWMC_R3_2_Handle_t PWM_Handle_M2;
  </#if><#-- ((MC.M2_CURRENT_SENSING_TOPO=='THREE_SHUNT') && (MC.M2_CS_ADC_NUM=='2')) -->
  <#if CondFamily_STM32F3 && (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
extern PWMC_ICS_Handle_t PWM_Handle_M2;
  </#if><#-- CondFamily_STM32F3 && (MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1; 
<#if MC.USE_STGAP1S>
extern GAP_Handle_t STGAP_M1;
</#if><#-- MC.USE_STGAP1S -->
<#if MC.DRIVE_NUMBER != "1">
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM2;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM2; 
</#if><#-- MC.DRIVE_NUMBER > 1 -->
<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC")>
extern STO_Handle_t STO_M1;
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") -->
<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
extern RevUpCtrl_Handle_t RevUpControlM1;
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") 
         || MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
<#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")>
extern RevUpCtrl_Handle_t RevUpControlM2;
extern STO_Handle_t STO_M2;
</#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") -->
<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL")>
extern STO_PLL_Handle_t STO_PLL_M1;
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") -->
<#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL")>
extern STO_PLL_Handle_t STO_PLL_M2;
</#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL") -->
<#if (MC.M2_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
extern STO_CR_Handle_t STO_CR_M2;
</#if><#-- (MC.M2_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->
<#if (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
extern STO_CR_Handle_t STO_CR_M1;
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->

<#if M2_HALL_SENSOR == true>
extern HALL_Handle_t HALL_M2;
</#if><#-- M2_HALL_SENSOR == true -->
<#if MC.M1_ICL_ENABLED == true>
extern ICL_Handle_t ICL_M1;
</#if><#-- MC.M1_ICL_ENABLED == true -->
<#if MC.M2_ICL_ENABLED == true && MC.DRIVE_NUMBER != "1">
extern ICL_Handle_t ICL_M2;
</#if><#-- MC.M2_ICL_ENABLED == true && MC.DRIVE_NUMBER > 1 -->
extern CircleLimitation_Handle_t CircleLimitationM1;
<#if MC.DRIVE_NUMBER != "1">
extern CircleLimitation_Handle_t CircleLimitationM2;
</#if><#-- MC.DRIVE_NUMBER > 1 -->
extern RampExtMngr_Handle_t RampExtMngrHFParamsM1;
<#if MC.DRIVE_NUMBER != "1">
extern RampExtMngr_Handle_t RampExtMngrHFParamsM2;
</#if><#-- MC.DRIVE_NUMBER > 1 -->
<#if MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true>
extern FF_Handle_t FF_M1;
</#if><#-- MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true -->
<#if MC.M1_MTPA_ENABLING == true>
extern MTPA_Handle_t MTPARegM1;
</#if><#-- MC.M1_MTPA_ENABLING == true -->
<#if MC.DRIVE_NUMBER != "1" && MC.M2_MTPA_ENABLING == true>
extern MTPA_Handle_t MTPARegM2;
</#if><#-- MC.DRIVE_NUMBER > 1 && MC.M2_MTPA_ENABLING == true -->
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
extern OpenLoop_Handle_t OpenLoop_ParamsM1;
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
<#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
extern OpenLoop_Handle_t OpenLoop_ParamsM2;
</#if><#-- MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_HW_OV_CURRENT_PROT_BYPASS == true && MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES">
extern DOUT_handle_t DOUT_OCPDisablingParamsM2;
   </#if><#-- MC.M2_HW_OV_CURRENT_PROT_BYPASS == true && MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" -->
   <#if MC.M2_ICL_ENABLED == true>
extern DOUT_handle_t ICLDOUTParamsM2;
  </#if><#-- MC.M2_ICL_ENABLED == true -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->
<#if MC.M1_HW_OV_CURRENT_PROT_BYPASS == true && MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES">
extern DOUT_handle_t DOUT_OCPDisablingParamsM1;
</#if><#-- MC.M1_HW_OV_CURRENT_PROT_BYPASS == true && MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" -->
<#if MC.M1_ICL_ENABLED == true>
extern DOUT_handle_t ICLDOUTParamsM1;
</#if><#-- MC.M1_ICL_ENABLED == true -->
extern RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS]; 
<#-- PFC feature usage -->
<#if MC.PFC_ENABLED == true>
extern PFC_Handle_t PFC;
</#if><#-- MC.PFC_ENABLED == true -->  
extern FOCVars_t FOCVars[NBR_OF_MOTORS];
<#-- Motor Profiler feature usage -->
<#if MC.MOTOR_PROFILER == true>
extern RampExtMngr_Handle_t RampExtMngrParamsSCC;
extern RampExtMngr_Handle_t RampExtMngrParamsOTT;
extern SCC_Handle_t SCC;
extern OTT_Handle_t OTT;
  <#if MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR">
extern HT_Handle_t HT;  
  </#if><#-- MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->
</#if><#-- MC.MOTOR_PROFILER == true -->
extern PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDId[NBR_OF_MOTORS];
extern PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS]; 
<#if MC.M1_POSITION_CTRL_ENABLING == true || MC.M2_POSITION_CTRL_ENABLING == true>
extern PosCtrl_Handle_t *pPosCtrl[NBR_OF_MOTORS];
</#if><#-- MC.M1_POSITION_CTRL_ENABLING == true || MC.M2_POSITION_CTRL_ENABLING == true -->
<#if MC.M1_FLUX_WEAKENING_ENABLING==true || MC.M2_FLUX_WEAKENING_ENABLING==true>
extern FW_Handle_t *pFW[NBR_OF_MOTORS];
</#if><#-- MC.M1_FLUX_WEAKENING_ENABLING==true || MC.M2_FLUX_WEAKENING_ENABLING==true -->
<#if MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true || MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true>
extern FF_Handle_t *pFF[NBR_OF_MOTORS];
</#if><#-- MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true || MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true -->
<#if MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE">
extern DOUT_handle_t *pR_Brake[NBR_OF_MOTORS];
</#if><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" -->  
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
extern SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
extern MCI_Handle_t Mci[NBR_OF_MOTORS];
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
extern PID_Handle_t PIDSpeedHandle_M1;
<#if M1_HALL_SENSOR == true>
extern HALL_Handle_t HALL_M1;
</#if><#-- M1_HALL_SENSOR == true -->

/* USER CODE BEGIN Additional extern */

/* USER CODE END Additional extern */  
 
#endif /* MC_CONFIG_H */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
