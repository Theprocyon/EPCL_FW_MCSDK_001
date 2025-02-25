<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    power_stage_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a power stage.
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
#ifndef POWER_STAGE_PARAMETERS_H
#define POWER_STAGE_PARAMETERS_H
<#if MC.USE_STGAP1S>  
#include "gap_gate_driver_ctrl.h"
</#if>

<#if MC.DRIVE_NUMBER != "1">
/**************************
 *** Motor 1 Parameters ***
 **************************/
<#else>
/************************
 *** Motor Parameters ***
 ************************/
</#if><#-- MC.DRIVE_NUMBER > 1 -->


/************* PWM Driving signals section **************/
<#if FOC>
#define HW_DEAD_TIME_NS                      ${MC.M1_HW_DEAD_TIME_NS} /*!< Dead-time inserted 
                                                      by HW if low side signals 
                                                      are not used */
</#if><#-- FOC -->
<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
/******** BEMF reading parameters section ******/

#define BEMF_ON_SENSING_DIVIDER              ${MC.BEMF_ON_SENSING_DIVIDER} /*!< BEMF voltage divider during PWM ON time 
                                                   for zero crossing detection */
</#if><#--  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR             ${MC.M1_VBUS_PARTITIONING_FACTOR} /*!< It expresses how 
                                                      much the Vbus is attenuated  
                                                      before being converted into 
                                                      digital value */
#define NOMINAL_BUS_VOLTAGE_V                ${MC.M1_NOMINAL_BUS_VOLTAGE_V} 
/******** Current reading parameters section ******/
/*** Topology ***/
#define ${MC.M1_CURRENT_SENSING_TOPO}

<#if MC.M1_CURRENT_SENSING_TOPO != "ICS_SENSORS">
#define RSHUNT                               ${MC.M1_RSHUNT} 
</#if><#-- MC.M1_CURRENT_SENSING_TOPO != "ICS_SENSORS" -->

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
<#-- abs is not present in our FTL version -->
<#if FOC || ACIM >
#define AMPLIFICATION_GAIN                   ${MC.M1_AMPLIFICATION_GAIN?replace("-","")} 
  <#if MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST">
#define VOLTAGE_FILTER_POLE_RPS              (${MC.M1_VOLTAGE_FILTER_POLE_RPS}) /*!< @brief  Time constant of the  
                                                 hardware RC filter applied to the phase voltage */
#define M1_PHASE_VOLTAGE_PARTITIONING_FACTOR (${MC.M1_PHASE_VOLTAGE_PARTITIONING_FACTOR}) /*!< It expresses how 
                                                       much the phase voltage is attenuated  
                                                       before being converted into 
                                                       digital value */
#define BOARD_LIMIT_REGEN_HIGH               (float_t)(${MC.M1_BOARD_LIMIT_REGEN_HIGH}) /*!< @brief Bus voltage upper 
                                                                  limit for regenerative current control. */
#define BOARD_LIMIT_REGEN_LOW                (float_t)(${MC.M1_BOARD_LIMIT_REGEN_LOW}) /*!< @brief  Bus voltage lower 
                                                                  limit for regenerative current control. */
#define BOARD_LIMIT_ACCEL_HIGH               (float_t)(${MC.M1_BOARD_LIMIT_ACCEL_HIGH}) /*!< @brief DC bus voltage 
                                                                  threshold above which maximum current is allowed */
#define BOARD_LIMIT_ACCEL_LOW                (float_t)(${MC.M1_BOARD_LIMIT_ACCEL_LOW}) /*!< @brief DC bus voltage 
                                                                  threshold under which current needs to be limited */

  </#if>



/*** Noise parameters ***/
#define TNOISE_NS                            ${MC.M1_TNOISE_NS}
<#-- Keep this one for now. Used in parameters_conversion_f*_h.htl -->
#define TRISE_NS                             ${MC.M1_TRISE_NS} 
<#if (MC.M1_TNOISE_NS?number gt MC.M1_TRISE_NS?number)>
#define MAX_TNTR_NS                          TNOISE_NS
<#else>
#define MAX_TNTR_NS                          TRISE_NS
</#if>
</#if><#-- FOC -->
<#if SIX_STEP>
#define AMPLIFICATION_GAIN                   ${MC.M1_AMPLIFICATION_GAIN}
  <#if MC.DRIVE_MODE == "CM">
#define CURR_REF_DIVIDER                     ${MC.CURR_REF_DIVIDER}  /*!< Divider of the current limiter threshold  */
  </#if> 
  <#if MC.CURRENT_LIMITER_OFFSET>
#define OCP_INT_REF                          ${MC.OCP_INT_REF}  /*!< Internal over-current protecion threshold (mV) */
  </#if> 
</#if><#-- SIX_STEP -->

/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V                                 ${MC.M1_V0_V} /*!< in Volts */
#define T0_C                                 ${MC.M1_T0_C} /*!< in Celsius degrees */
#define dV_dT                                ${MC.M1_dV_dT} /*!< V/Celsius degrees */
#define T_MAX                                ${MC.M1_T_MAX} /*!< Sensor measured 
                                                     temperature at maximum 
                                                     power stage working 
                                                     temperature, Celsius degrees */
<#if MC.USE_STGAP1S>
/************ STGAP1AS section ***************/
/* These configuration value will come from the Workbench in the future */
#define M1_STAGAP1AS_NUM 7
#define GAP_CFG1                             GAP_CFG1_SD_FLAG | GAP_CFG1_DIAG_EN   |GAP_CFG1_DT_DISABLE
#define GAP_CFG2                             GAP_CFG2_DESATTH_3V | GAP_CFG2_DESATCURR_500UA
#define GAP_CFG3                             GAP_CFG3_2LTOTH_7_0V | GAP_CFG3_2LTOTIME_3_00_US
#define GAP_CFG4                             GAP_CFG4_UVLOTH_VH_DISABLE | GAP_CFG4_UVLOTH_VL_DISABLE
#define GAP_CFG5                             GAP_CFG5_CLAMP_EN | GAP_CFG5_SENSE_EN | GAP_CFG5_DESAT_EN | GAP_CFG5_2LTO_ON_FAULT
#define GAP_DIAG1                            GAP_DIAG_SPI_REGERR | GAP_DIAG_DESAT_SENSE /* |GAP_DIAG_UVLOD_OVLOD | GAP_DIAG_OVLOH_OVLOL */
#define GAP_DIAG2                            GAP_DIAG_NONE

/* By default all STGAP1AS in the daisy chain are configured identicaly */ 

#define STGAP1AS_BRAKE {GAP_CFG1, GAP_CFG2, GAP_CFG3, GAP_CFG4, GAP_CFG5, GAP_DIAG1, GAP_DIAG2 }
#define STGAP1AS_UH {GAP_CFG1, GAP_CFG2, GAP_CFG3, GAP_CFG4, GAP_CFG5, GAP_DIAG1, GAP_DIAG2 }
#define STGAP1AS_UL {GAP_CFG1, GAP_CFG2, GAP_CFG3, GAP_CFG4, GAP_CFG5, GAP_DIAG1, GAP_DIAG2 }
#define STGAP1AS_VH {GAP_CFG1, GAP_CFG2, GAP_CFG3, GAP_CFG4, GAP_CFG5, GAP_DIAG1, GAP_DIAG2 }
#define STGAP1AS_VL {GAP_CFG1, GAP_CFG2, GAP_CFG3, GAP_CFG4, GAP_CFG5, GAP_DIAG1, GAP_DIAG2 }
#define STGAP1AS_WH {GAP_CFG1, GAP_CFG2, GAP_CFG3, GAP_CFG4, GAP_CFG5, GAP_DIAG1, GAP_DIAG2 }
#define STGAP1AS_WL {GAP_CFG1, GAP_CFG2, GAP_CFG3, GAP_CFG4, GAP_CFG5, GAP_DIAG1, GAP_DIAG2 }
</#if>

<#if MC.DRIVE_NUMBER != "1">
/**************************
 *** Motor 2 Parameters ***
 **************************/

#define HW_DEAD_TIME_NS2                     ${MC.M2_HW_DEAD_TIME_NS} /*!< Dead-time inserted 
                                                         by HW if low side signals 
                                                         are not used */

/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR2            ${MC.M2_VBUS_PARTITIONING_FACTOR} /*!< It expresses how 
                                                       much the Vbus is attenuated  
                                                       before being converted into 
                                                       digital value */
#define NOMINAL_BUS_VOLTAGE_V2               ${MC.M2_NOMINAL_BUS_VOLTAGE_V} 
/******** Current reading parameters section ******/
/*** Topology ***/
#define ${MC.M2_CURRENT_SENSING_TOPO}

<#if MC.M2_CURRENT_SENSING_TOPO != "ICS_SENSORS">
#define RSHUNT2                              ${MC.M2_RSHUNT} 
</#if><#-- MC.M2_CURRENT_SENSING_TOPO != "ICS_SENSORS" -->

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN2                  ${MC.M2_AMPLIFICATION_GAIN?replace("-","")} 

/*** Noise parameters ***/
#define TNOISE_NS2                           ${MC.M2_TNOISE_NS}
<#-- Keep this one for now. Used in parameters_conversion_f*_h.htl --> 
#define TRISE_NS2                            ${MC.M2_TRISE_NS} 
<#if (MC.M2_TNOISE_NS?number gt MC.M2_TRISE_NS?number)>
#define MAX_TNTR_NS2                         TNOISE_NS2
<#else>
#define MAX_TNTR_NS2                         TRISE_NS2
</#if>
   
/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V2                                ${MC.M2_V0_V} /*!< in Volts */
#define T0_C2                                ${MC.M2_T0_C} /*!< in Celsius degrees */
#define dV_dT2                               ${MC.M2_dV_dT} /*!< V/Celsius degrees */
#define T_MAX2                               ${MC.M2_T_MAX} /*!< Sensor measured 
                                                       temperature at maximum 
                                                       power stage working 
                                                       temperature, Celsius degrees */
</#if><#-- MC.DRIVE_NUMBER > 1 -->

#endif /*POWER_STAGE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
