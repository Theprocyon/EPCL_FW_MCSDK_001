<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
<#include "*/ftl/ip_macro.ftl">
<#include "*/ftl/sixstep_assign.ftl">
/**
  ******************************************************************************
  * @file    parameters_conversion.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
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
#ifndef PARAMETERS_CONVERSION_H
#define PARAMETERS_CONVERSION_H

#include "mc_math.h"
<#if CondFamily_STM32F0>
#include "parameters_conversion_f0xx.h"
<#elseif CondFamily_STM32F3>
#include "parameters_conversion_f30x.h"
<#elseif CondFamily_STM32F4>
#include "parameters_conversion_f4xx.h"
<#elseif CondFamily_STM32L4>
#include "parameters_conversion_l4xx.h"
<#elseif CondFamily_STM32F7>
#include "parameters_conversion_f7xx.h"
<#elseif CondFamily_STM32H5>
#include "parameters_conversion_h5xx.h"
<#elseif CondFamily_STM32H7>
#include "parameters_conversion_h7xx.h"
<#elseif CondFamily_STM32G4>
#include "parameters_conversion_g4xx.h"
<#elseif CondFamily_STM32G0>
#include "parameters_conversion_g0xx.h"
<#elseif CondFamily_STM32C0>
#include "parameters_conversion_c0xx.h"
</#if><#-- CondFamily_STM32F0 -->
#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "power_stage_parameters.h"


<#if MC.M1_CURRENT_SENSING_TOPO != "ICS_SENSORS">
  <#if MC.CURRENT_LIMITER_OFFSET>
#define CURRENT_CONV_FACTOR                 ((1000 * RSHUNT * AMPLIFICATION_GAIN))
#define CURRENT_CONV_FACTOR_INV             (1.0 / ((1000 * RSHUNT * AMPLIFICATION_GAIN))
  <#else><#-- !MC.CURRENT_LIMITER_OFFSET -->
#define CURRENT_CONV_FACTOR                 ((RSHUNT * AMPLIFICATION_GAIN) / ADC_REFERENCE_VOLTAGE)
#define CURRENT_CONV_FACTOR_INV             (1.0 / ((RSHUNT * AMPLIFICATION_GAIN) / ADC_REFERENCE_VOLTAGE))
  </#if><#-- MC.CURRENT_LIMITER_OFFSET -->
<#else><#-- M1_CURRENT_SENSING_TOPO == "ICS_SENSORS" -->
  <#if MC.CURRENT_LIMITER_OFFSET>
#define CURRENT_CONV_FACTOR                 ((1000 * AMPLIFICATION_GAIN))
#define CURRENT_CONV_FACTOR_INV             (1.0 / ((1000 * AMPLIFICATION_GAIN))
  <#else><#-- !MC.CURRENT_LIMITER_OFFSET -->
#define CURRENT_CONV_FACTOR                 ((AMPLIFICATION_GAIN) / ADC_REFERENCE_VOLTAGE)
#define CURRENT_CONV_FACTOR_INV             (1.0 / ((AMPLIFICATION_GAIN) / ADC_REFERENCE_VOLTAGE))
  </#if><#-- MC.CURRENT_LIMITER_OFFSET -->
</#if><#-- M1_CURRENT_SENSING_TOPO != "ICS_SENSORS" --> 

#define NOMINAL_CURRENT                     NOMINAL_CURRENT_A
#define ADC_REFERENCE_VOLTAGE               ${MC.ADC_REFERENCE_VOLTAGE}
#define M1_MAX_READABLE_CURRENT             (ADC_REFERENCE_VOLTAGE / ( RSHUNT * AMPLIFICATION_GAIN))

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE                  (uint32_t)((uint32_t)(PWM_FREQUENCY) / (REGULATION_EXECUTION_RATE))

/* TF_REGULATION_RATE_SCALED is TF_REGULATION_RATE divided by PWM_FREQ_SCALING to allow more dynamic */ 
#define TF_REGULATION_RATE_SCALED           (uint16_t)((uint32_t)(PWM_FREQUENCY) / (REGULATION_EXECUTION_RATE\
                                            * PWM_FREQ_SCALING))

/* DPP_CONV_FACTOR is introduce to compute the right DPP with TF_REGULATOR_SCALED  */
#define DPP_CONV_FACTOR                     (65536 / PWM_FREQ_SCALING) 

#define REP_COUNTER                         (uint16_t)(REGULATION_EXECUTION_RATE - 1u)
#define SYS_TICK_FREQUENCY                  (uint16_t)2000
#define UI_TASK_FREQUENCY_HZ                10U
<#if MC.M1_DRIVE_TYPE != "SIX_STEP" ||  MC.DRIVE_MODE != "VM">
#define PHASE1_FINAL_CURRENT                (PHASE1_FINAL_CURRENT_A * CURRENT_CONV_FACTOR)
#define PHASE2_FINAL_CURRENT                (PHASE2_FINAL_CURRENT_A * CURRENT_CONV_FACTOR)
#define PHASE3_FINAL_CURRENT                (PHASE3_FINAL_CURRENT_A * CURRENT_CONV_FACTOR)
#define PHASE4_FINAL_CURRENT                (PHASE4_FINAL_CURRENT_A * CURRENT_CONV_FACTOR)
#define PHASE5_FINAL_CURRENT                (PHASE5_FINAL_CURRENT_A * CURRENT_CONV_FACTOR)
</#if>

<#if (MC.DRIVE_NUMBER != "1") && (MC.M1_DRIVE_TYPE != "SIX_STEP" ||  MC.DRIVE_MODE != "VM")>
#define PHASE1_FINAL_CURRENT2               (PHASE1_FINAL_CURRENT2_A * CURRENT_CONV_FACTOR2)
#define PHASE2_FINAL_CURRENT2               (PHASE2_FINAL_CURRENT2_A * CURRENT_CONV_FACTOR2)
#define PHASE3_FINAL_CURRENT2               (PHASE3_FINAL_CURRENT2_A * CURRENT_CONV_FACTOR2)
#define PHASE4_FINAL_CURRENT2               (PHASE4_FINAL_CURRENT2_A * CURRENT_CONV_FACTOR2)
#define PHASE5_FINAL_CURRENT2               (PHASE5_FINAL_CURRENT2_A * CURRENT_CONV_FACTOR2)
</#if>
<#if MC.M1_POSITION_CTRL_ENABLING == true>
#define MEDIUM_FREQUENCY_TASK_RATE          (uint16_t)POSITION_LOOP_FREQUENCY_HZ
#define MF_TASK_OCCURENCE_TICKS             (SYS_TICK_FREQUENCY / POSITION_LOOP_FREQUENCY_HZ) - 1u
<#else><#-- MC.M1_POSITION_CTRL_ENABLING == true -->
#define MEDIUM_FREQUENCY_TASK_RATE          (uint16_t)SPEED_LOOP_FREQUENCY_HZ
#define MF_TASK_OCCURENCE_TICKS             (SYS_TICK_FREQUENCY / SPEED_LOOP_FREQUENCY_HZ) - 1u
</#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->
#define UI_TASK_OCCURENCE_TICKS             (SYS_TICK_FREQUENCY / UI_TASK_FREQUENCY_HZ) - 1u
#define SERIALCOM_TIMEOUT_OCCURENCE_TICKS   (SYS_TICK_FREQUENCY / SERIAL_COM_TIMEOUT_INVERSE) - 1u
#define SERIALCOM_ATR_TIME_TICKS            (uint16_t)(((SYS_TICK_FREQUENCY * SERIAL_COM_ATR_TIME_MS) / 1000u) - 1u)

<#if MC.M1_ICL_ENABLED>
/* Inrush current limiter parameters */
#define M1_ICL_RELAY_SWITCHING_DELAY_TICKS  (uint16_t)(((M1_ICL_RELAY_SWITCHING_DELAY_MS\
                                            * MEDIUM_FREQUENCY_TASK_RATE) / 1000) - 1u)
#define M1_ICL_CAPS_CHARGING_DELAY_TICKS    (uint16_t)(((M1_ICL_CAPS_CHARGING_DELAY_MS *\
                                            MEDIUM_FREQUENCY_TASK_RATE) / 1000) - 1u)
#define M1_ICL_VOLTAGE_THRESHOLD_VOLT       (uint16_t)M1_ICL_VOLTAGE_THRESHOLD
</#if><#-- MC.M1_ICL_ENABLED -->

<#if MC.DRIVE_NUMBER != "1">
#define TF_REGULATION_RATE2                 (uint32_t)((uint32_t)(PWM_FREQUENCY2) / REGULATION_EXECUTION_RATE2)
#define IQMAX2                              (IQMAX2_A * CURRENT_CONV_FACTOR2)
#define ID_DEMAG2                           (ID_DEMAG2_A * CURRENT_CONV_FACTOR2)
#define NOMINAL_CURRENT2                    (NOMINAL_CURRENT2_A * CURRENT_CONV_FACTOR2)
#define FINAL_I_ALIGNMENT2                  (uint16_t)(FINAL_I_ALIGNMENT2_A * CURRENT_CONV_FACTOR2)
#define DEFAULT_FLUX_COMPONENT2             (DEFAULT_FLUX_COMPONENT2_A * CURRENT_CONV_FACTOR2)
#define DEFAULT_TORQUE_COMPONENT2           (DEFAULT_TORQUE_COMPONENT2_A * CURRENT_CONV_FACTOR2)

/* TF_REGULATION_RATE_SCALED2 is TF_REGULATION_RATE2 divided by PWM_FREQ_SCALING2 to allow more dynamic */ 
#define TF_REGULATION_RATE_SCALED2          (uint16_t)((uint32_t)(PWM_FREQUENCY2) / (REGULATION_EXECUTION_RATE2\
                                            * PWM_FREQ_SCALING2))

/* DPP_CONV_FACTOR2 is introduce to compute the right DPP with TF_REGULATOR_SCALED2  */
#define DPP_CONV_FACTOR2                    (65536 / PWM_FREQ_SCALING2)
#define REP_COUNTER2                        (uint16_t)((REGULATION_EXECUTION_RATE2 * 2u) - 1u)
  <#if MC.M2_POSITION_CTRL_ENABLING == true>
#define MEDIUM_FREQUENCY_TASK_RATE2         (uint16_t)POSITION_LOOP_FREQUENCY_HZ2
  <#else><#-- MC.M2_POSITION_CTRL_ENABLING == true -->
#define MEDIUM_FREQUENCY_TASK_RATE2         (uint16_t)SPEED_LOOP_FREQUENCY_HZ2
   </#if><#-- MC.M2_POSITION_CTRL_ENABLING == true -->
#define MF_TASK_OCCURENCE_TICKS2            (SYS_TICK_FREQUENCY/MEDIUM_FREQUENCY_TASK_RATE2)-1u
  <#if MC.M2_ICL_ENABLED>
/* Inrush current limiter parameters */
#define M2_ICL_RELAY_SWITCHING_DELAY_TICKS  (uint16_t)(((M2_ICL_RELAY_SWITCHING_DELAY_MS\
                                            * MEDIUM_FREQUENCY_TASK_RATE) / 1000) - 1u)
#define M2_ICL_CAPS_CHARGING_DELAY_TICKS    (uint16_t)(((M2_ICL_CAPS_CHARGING_DELAY_MS\
                                            * MEDIUM_FREQUENCY_TASK_RATE) / 1000) - 1u)
#define M2_ICL_VOLTAGE_THRESHOLD_U16VOLT    (uint16_t)(uint16_t)((M2_ICL_VOLTAGE_THRESHOLD * 65536U)\
                                            / (ADC_REFERENCE_VOLTAG / VBUS_PARTITIONING_FACTOR2))
  </#if><#-- MC.M2_ICL_ENABLED -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->
#define MAX_APPLICATION_SPEED_UNIT          ((MAX_APPLICATION_SPEED_RPM * SPEED_UNIT) / U_RPM)
#define MIN_APPLICATION_SPEED_UNIT          ((MIN_APPLICATION_SPEED_RPM * SPEED_UNIT) / U_RPM)
#define MAX_APPLICATION_SPEED_UNIT2         ((MAX_APPLICATION_SPEED_RPM2 * SPEED_UNIT) / U_RPM)
#define MIN_APPLICATION_SPEED_UNIT2         ((MIN_APPLICATION_SPEED_RPM2 * SPEED_UNIT) / U_RPM)

<#if  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
#define PERCENTAGE_FACTOR                   (uint16_t)(VARIANCE_THRESHOLD * 128u)  

/************************* BEMF OBSERVER PARAMETERS **************************/
#define OBS_MINIMUM_SPEED_UNIT              (uint16_t)((OBS_MINIMUM_SPEED_RPM * SPEED_UNIT) / U_RPM)
/*!< Conversion factor from bus voltage digits to bemf threshold digits */
#define BEMF_BUS2THRES_FACTOR               (uint16_t)(1000 * VBUS_PARTITIONING_FACTOR * BEMF_ON_SENSING_DIVIDER)
/*!< Correction factor for bemf divider diode voltage drop */
#define BEMF_CORRECT_FACTOR                 (uint16_t) (65536 * BEMF_DIVIDER_DIODE_V / (BEMF_ON_SENSING_DIVIDER * ADC_REFERENCE_VOLTAGE))

#define BEMF_ADC_TRIG_TIME                  (uint16_t)(PWM_PERIOD_CYCLES * BEMF_ADC_TRIG_TIME_DPP / 1024)

#define ZCD_RISING_TO_COMM_9BIT            (uint16_t) (ZCD_RISING_TO_COMM * 512 / 60) /*!< Zero Crossing detection to commutation delay in 60/512 degrees */
#define ZCD_FALLING_TO_COMM_9BIT           (uint16_t) (ZCD_FALLING_TO_COMM * 512 / 60) /*!< Zero Crossing detection to commutation delay in 60/512 degrees */

<#if MC.DRIVE_MODE == "VM">
#define BEMF_ADC_TRIG_TIME_ON               (uint16_t)(PWM_PERIOD_CYCLES * BEMF_ADC_TRIG_TIME_ON_DPP / 1024)
#define BEMF_PWM_ON_ENABLE_THRES            (uint16_t)(PWM_PERIOD_CYCLES * BEMF_PWM_ON_ENABLE_THRES_DPP / 1024)
#define BEMF_PWM_ON_DISABLE_THRES           (uint16_t)(PWM_PERIOD_CYCLES * (BEMF_PWM_ON_ENABLE_THRES_DPP\
                                            - BEMF_PWM_ON_ENABLE_HYSTERESIS_DPP) / 1024)

</#if><#-- MC.DRIVE_MODE == "VM"-->
<#if MC.DRIVE_MODE == "VM"> <#-- VOLTAGE MODE -->
#define PHASE1_VOLTAGE_DPP                  ((PWM_PERIOD_CYCLES * PHASE1_VOLTAGE_RMS * PHASE1_VOLTAGE_RMS)\
                                            / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE2_VOLTAGE_DPP                  ((PWM_PERIOD_CYCLES * PHASE2_VOLTAGE_RMS * PHASE2_VOLTAGE_RMS)\
                                            / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE3_VOLTAGE_DPP                  ((PWM_PERIOD_CYCLES * PHASE3_VOLTAGE_RMS * PHASE3_VOLTAGE_RMS)\
                                            / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE4_VOLTAGE_DPP                  ((PWM_PERIOD_CYCLES * PHASE4_VOLTAGE_RMS * PHASE4_VOLTAGE_RMS)\
                                            / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE5_VOLTAGE_DPP                  ((PWM_PERIOD_CYCLES * PHASE5_VOLTAGE_RMS * PHASE5_VOLTAGE_RMS)\
                                            / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
<#else> <#-- CURRENT MODE -->
  <#if MC.CURRENT_LIMITER_OFFSET>
#define PHASE1_FINAL_CURRENT_DPP            (uint16_t) (1024*CURR_REF_DIVIDER*(2*OCP_INT_REF-PHASE1_FINAL_CURRENT)\
                                            /(1000*ADC_REFERENCE_VOLTAGE))
#define PHASE2_FINAL_CURRENT_DPP            (uint16_t) (1024*CURR_REF_DIVIDER*(2*OCP_INT_REF-PHASE2_FINAL_CURRENT)\
                                            /(1000*ADC_REFERENCE_VOLTAGE))
#define PHASE3_FINAL_CURRENT_DPP            (uint16_t) (1024*CURR_REF_DIVIDER*(2*OCP_INT_REF-PHASE3_FINAL_CURRENT)\
                                            /(1000*ADC_REFERENCE_VOLTAGE))
#define PHASE4_FINAL_CURRENT_DPP            (uint16_t) (1024*CURR_REF_DIVIDER*(2*OCP_INT_REF-PHASE4_FINAL_CURRENT)\
                                            /(1000*ADC_REFERENCE_VOLTAGE))
#define PHASE5_FINAL_CURRENT_DPP            (uint16_t) (1024*CURR_REF_DIVIDER*(2*OCP_INT_REF-PHASE5_FINAL_CURRENT)\
                                            /(1000*ADC_REFERENCE_VOLTAGE))
  <#else><#-- MC.CURRENT_LIMITER_OFFSETE -->
#define PHASE1_FINAL_CURRENT_DPP            (uint16_t)(1024 * PHASE1_FINAL_CURRENT)
#define PHASE2_FINAL_CURRENT_DPP            (uint16_t)(1024 * PHASE2_FINAL_CURRENT)
#define PHASE3_FINAL_CURRENT_DPP            (uint16_t)(1024 * PHASE3_FINAL_CURRENT)
#define PHASE4_FINAL_CURRENT_DPP            (uint16_t)(1024 * PHASE4_FINAL_CURRENT)
#define PHASE5_FINAL_CURRENT_DPP            (uint16_t)(1024 * PHASE5_FINAL_CURRENT)
   </#if><#-- MC.CURRENT_LIMITER_OFFSETE -->
#define PHASE1_FINAL_CURRENT_REF            (PWM_PERIOD_CYCLES_REF * PHASE1_FINAL_CURRENT_DPP / 1024)
#define PHASE2_FINAL_CURRENT_REF            (PWM_PERIOD_CYCLES_REF * PHASE2_FINAL_CURRENT_DPP / 1024)
#define PHASE3_FINAL_CURRENT_REF            (PWM_PERIOD_CYCLES_REF * PHASE3_FINAL_CURRENT_DPP / 1024)
#define PHASE4_FINAL_CURRENT_REF            (PWM_PERIOD_CYCLES_REF * PHASE4_FINAL_CURRENT_DPP / 1024)
#define PHASE5_FINAL_CURRENT_REF            (PWM_PERIOD_CYCLES_REF * PHASE5_FINAL_CURRENT_DPP / 1024)
  </#if><#-- MC.DRIVE_MODE == "VM" -->

#define DEMAG_MINIMUM_SPEED                    SPEED_THRESHOLD_DEMAG * SPEED_UNIT / U_RPM
#define DEMAG_REVUP_CONV_FACTOR                (uint32_t)((((DEMAG_REVUP_STEP_RATIO * PWM_FREQUENCY * SPEED_UNIT)\
                                               / (600 * POLE_PAIR_NUM )) * PWM_PERIOD_CYCLES * TIM_CLOCK_DIVIDER ) / (LF_TIMER_PSC + 1))
#define DEMAG_RUN_CONV_FACTOR                  (uint32_t)((((DEMAG_RUN_STEP_RATIO * PWM_FREQUENCY * SPEED_UNIT)\
                                               / (600 * POLE_PAIR_NUM )) * PWM_PERIOD_CYCLES * TIM_CLOCK_DIVIDER )  / (LF_TIMER_PSC + 1))
#define MIN_DEMAG_COUNTER_TIME                 (uint32_t) ((MIN_DEMAG_TIME * PWM_PERIOD_CYCLES * TIM_CLOCK_DIVIDER ) / (LF_TIMER_PSC + 1))
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC -->

/**************************   VOLTAGE CONVERSIONS  Motor 1 *************************/
<#if MC.M1_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE">  
#define OVERVOLTAGE_THRESHOLD_d             (uint16_t)(OV_VOLTAGE_THRESHOLD_V * 65535 /\
                                            (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR))
#define OVERVOLTAGE_THRESHOLD_LOW_d         (uint16_t)(OV_VOLTAGE_THRESHOLD_V * 65535 /\
                                            (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR))
<#else><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" -->
#define OVERVOLTAGE_THRESHOLD_d             (uint16_t)(M1_OVP_THRESHOLD_HIGH * 65535 /\
                                            (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR))
#define OVERVOLTAGE_THRESHOLD_LOW_d         (uint16_t)(M1_OVP_THRESHOLD_LOW * 65535 /\
                                            (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR))
</#if><#-- MC.M1_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE" -->
#define UNDERVOLTAGE_THRESHOLD_d            (uint16_t)((UD_VOLTAGE_THRESHOLD_V * 65535) /\
                                            ((uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR)))
#define INT_SUPPLY_VOLTAGE                  (uint16_t)(65536 / ADC_REFERENCE_VOLTAGE)
#define DELTA_TEMP_THRESHOLD                (OV_TEMPERATURE_THRESHOLD_C - T0_C)
#define DELTA_V_THRESHOLD                   (dV_dT * DELTA_TEMP_THRESHOLD)
#define OV_TEMPERATURE_THRESHOLD_d          ((V0_V + DELTA_V_THRESHOLD) * INT_SUPPLY_VOLTAGE)
#define DELTA_TEMP_HYSTERESIS               (OV_TEMPERATURE_HYSTERESIS_C)
#define DELTA_V_HYSTERESIS                  (dV_dT * DELTA_TEMP_HYSTERESIS)
#define OV_TEMPERATURE_HYSTERESIS_d         (DELTA_V_HYSTERESIS * INT_SUPPLY_VOLTAGE)
<#if MC.DRIVE_NUMBER != "1">

/**************************   VOLTAGE CONVERSIONS  Motor 2 *************************/
<#if MC.M2_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE">  
#define OVERVOLTAGE_THRESHOLD_d2            (uint16_t)(OV_VOLTAGE_THRESHOLD_V2 * 65535 /\
                                            (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR2))
#define OVERVOLTAGE_THRESHOLD_LOW_d2        (uint16_t)(OV_VOLTAGE_THRESHOLD_V2) * 65535 /\
                                            (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR2))
  <#else><#-- MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" -->
#define OVERVOLTAGE_THRESHOLD_d2            (uint16_t)(M2_OVP_THRESHOLD_HIGH * 65535 /\
                                            (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR2))
#define OVERVOLTAGE_THRESHOLD_LOW_d2        (uint16_t)(M2_OVP_THRESHOLD_LOW * 65535 /\
                                            (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR2))
  </#if><#-- MC.M2_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE" -->
#define UNDERVOLTAGE_THRESHOLD_d2           (uint16_t)((UD_VOLTAGE_THRESHOLD_V2 * 65535) /\
                                            ((uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR2)))
#define INT_SUPPLY_VOLTAGE2                 (uint16_t)(65536 / ADC_REFERENCE_VOLTAGE)
#define DELTA_TEMP_THRESHOLD2               (OV_TEMPERATURE_THRESHOLD_C2 - T0_C2)
#define DELTA_V_THRESHOLD2                  (dV_dT2 * DELTA_TEMP_THRESHOLD2)
#define OV_TEMPERATURE_THRESHOLD_d2         ((V0_V2 + DELTA_V_THRESHOLD2) * INT_SUPPLY_VOLTAGE2)
#define DELTA_TEMP_HYSTERESIS2              (OV_TEMPERATURE_HYSTERESIS_C2)
#define DELTA_V_HYSTERESIS2                 (dV_dT2 * DELTA_TEMP_HYSTERESIS2)
#define OV_TEMPERATURE_HYSTERESIS_d2        (DELTA_V_HYSTERESIS2 * INT_SUPPLY_VOLTAGE2)
</#if><#-- MC.DRIVE_NUMBER > 1 -->

<#if M2_ENCODER>
/*************** Encoder Alignemnt ************************/    
#define ALIGNMENT_ANGLE_S162                (int16_t)(M2_ALIGNMENT_ANGLE_DEG * 65536u / 360u)
</#if><#-- M2_ENCODER -->
<#if M1_ENCODER>
#define ALIGNMENT_ANGLE_S16                 (int16_t)(M1_ALIGNMENT_ANGLE_DEG * 65536u / 360u)
#define FINAL_I_ALIGNMENT (uint16_t)(FINAL_I_ALIGNMENT_A * CURRENT_CONV_FACTOR)
</#if><#-- M1_ENCODER -->

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES                    (uint16_t)(((uint32_t)ADV_TIM_CLK_MHz * (uint32_t)1000000u\
                                             / ((uint32_t)(PWM_FREQUENCY))) & (uint16_t)0xFFFE)
<#if MC.DRIVE_MODE == "CM">
#define PWM_PERIOD_CYCLES_REF                (uint16_t)(((uint32_t)ADV_TIM_CLK_MHz * (uint32_t)1000000u\
                                             / ((uint32_t)(PWM_FREQUENCY_REF))) & (uint16_t)0xFFFE)
</#if><#-- MC.DRIVE_MODE == "CM" -->
<#if MC.DRIVE_NUMBER != "1">
#define PWM_PERIOD_CYCLES2                   (uint16_t)(((uint32_t)ADV_TIM_CLK_MHz2 * (uint32_t)1000000u\
                                             / ((uint32_t)(PWM_FREQUENCY2))) & (uint16_t)0xFFFE)
</#if><#-- MC.DRIVE_NUMBER > 1 -->
<#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
#define DEADTIME_NS                          SW_DEADTIME_NS
<#else><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING != "LS_PWM_TIMER" -->
#define DEADTIME_NS                          HW_DEAD_TIME_NS
</#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER" -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
#define DEADTIME_NS2                         SW_DEADTIME_NS2
  <#else><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING != "LS_PWM_TIMER" -->
#define DEADTIME_NS2                         HW_DEAD_TIME_NS2
  </#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER" -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->
#define DEAD_TIME_ADV_TIM_CLK_MHz           (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1                  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS / 500uL)
#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS                    (uint16_t)DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS                    (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS                    (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS                    (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif

/**********************/
/* MOTOR 1 ADC Timing */
/**********************/
/* In ADV_TIMER CLK cycles*/
#define SAMPLING_TIME                       ((ADC_SAMPLING_CYCLES * ADV_TIM_CLK_MHz) / ADC_CLK_MHz) 

/* USER CODE BEGIN temperature */
#define M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE 25u
#define M1_TEMP_SW_FILTER_BW_FACTOR         250u
/* USER CODE END temperature */

/****** Prepares the UI configurations according the MCconfxx settings ********/
<#if MC.DEBUG_DAC_FUNCTIONALITY_EN == true>
#define DAC_ENABLE | OPT_DAC
#define DAC_OP_ENABLE | UI_CFGOPT_DAC
<#else><#-- MC.DEBUG_DAC_FUNCTIONALITY_EN == false -->
#define DAC_ENABLE
#define DAC_OP_ENABLE
</#if><#-- MC.DEBUG_DAC_FUNCTIONALITY_EN == true -->

/******************************************************************************* 
  * UI configurations settings. It can be manually overwritten if special 
  * configuartion is required. 
*******************************************************************************/
/* Specific options of UI */
#define UI_CONFIG_M1                        (UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE\
                                            | (MAIN_SCFG << MAIN_SCFG_POS)\
                                            | (AUX_SCFG << AUX_SCFG_POS)\
                                            | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE UI_CFGOPT_PFC_ENABLE\
                                            | UI_CFGOPT_PLLTUNING)
<#if MC.DRIVE_NUMBER == "1">
#define UI_CONFIG_M2
<#else><#-- MC.DRIVE_NUMBER != 1 -->

/* Specific options of UI, Motor 2 */
#define UI_CONFIG_M2                        (UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE2\
                                            | (MAIN_SCFG2 << MAIN_SCFG_POS)\
                                            | (AUX_SCFG2 << AUX_SCFG_POS)\
                                            | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE2)
</#if><#-- MC.DRIVE_NUMBER == 1 -->

<#-- Can't this be removed? Seems only used in START_STOP_POLARITY which comes from WB... -->
#define DIN_ACTIVE_LOW                      Bit_RESET
#define DIN_ACTIVE_HIGH                     Bit_SET

<#-- Only left because of the FreeRTOS project that we will need to adapt sooner or later... 
#define USE_EVAL                            (defined(USE_STM32446E_EVAL) || defined(USE_STM324xG_EVAL)\
                                            || defined(USE_STM32F4XX_DUAL))
 -->

#define DOUT_ACTIVE_HIGH                    DOutputActiveHigh
#define DOUT_ACTIVE_LOW                     DOutputActiveLow

<#if (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR")>
/**********  AUXILIARY HALL TIMER MOTOR 1 *************/
#define M1_HALL_TIM_PERIOD                  65535
<@define_IC_FILTER motor=1 sensor='HALL' driver=DRIVER icx_filter=MC.M1_HALL_ICx_FILTER?number />
#define SPD_TIM_M1_IRQHandler               ${TimerHandler(_last_word(MC.M1_HALL_TIMER_SELECTION))}
</#if><#-- (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR") -->

<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_OVERMODULATION == false && MC.M2_CURRENT_SENSING_TOPO != "ICS_SENSORS">
/* MMI Table Motor 2 MAX_MODULATION_${MC.M2_MAX_MODULATION_INDEX}_PER_CENT */
#define MAX_MODULE2                         (uint16_t)((${MC.M2_MAX_MODULATION_INDEX}* 32767)/100)
  <#else><#-- MC.M2_OVERMODULATION == true || MC.M2_CURRENT_SENSING_TOPO == "ICS_SENSORS" -->
/* MMI Table Motor 2 100% */
#define MAX_MODULE2                         32767
  </#if><#-- MC.M2_OVERMODULATION == false && MC.M2_CURRENT_SENSING_TOPO != "ICS_SENSORS" -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->
 
<#if CondFamily_STM32F4 || CondFamily_STM32F7 >
#define  SAMPLING_CYCLE_CORRECTION          0 /* ${McuName} ADC sampling time is an integer number */
<#else><#-- CondFamily_STM32F4 || CondFamily_STM32F7 -->
<#-- Addition of the Half cycle of ADC sampling time-->
#define SAMPLING_CYCLE_CORRECTION           0.5 /* Add half cycle required by ${McuName} ADC */
#define LL_ADC_SAMPLINGTIME_1CYCLES_5       LL_ADC_SAMPLINGTIME_1CYCLE_5
</#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 -->

//cstat !MISRAC2012-Rule-20.10 !DEFINE-hash-multiple
#define LL_ADC_SAMPLING_CYCLE(CYCLE)        LL_ADC_SAMPLINGTIME_ ## CYCLE ## ${LL_ADC_CYCLE_SUFFIX} 
  
#endif /*PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
