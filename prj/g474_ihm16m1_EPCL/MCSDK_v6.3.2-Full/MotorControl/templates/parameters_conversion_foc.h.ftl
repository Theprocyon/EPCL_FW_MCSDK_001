<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
<#include "*/ftl/ip_macro.ftl">
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

/* Current conversion from Ampere unit to 16Bit Digit */
<#if MC.M1_CURRENT_SENSING_TOPO != "ICS_SENSORS">
#define CURRENT_CONV_FACTOR                 (uint16_t)((65536.0 * RSHUNT * AMPLIFICATION_GAIN)/ADC_REFERENCE_VOLTAGE)
#define CURRENT_CONV_FACTOR_INV             (1.0 / ((65536.0 * RSHUNT * AMPLIFICATION_GAIN) / ADC_REFERENCE_VOLTAGE))
<#else><#-- M1_CURRENT_SENSING_TOPO == "ICS_SENSORS" -->
#define CURRENT_CONV_FACTOR                 (uint16_t)((65536.0 * AMPLIFICATION_GAIN)/ADC_REFERENCE_VOLTAGE)
#define CURRENT_CONV_FACTOR_INV             (1.0 / ((65536.0 * AMPLIFICATION_GAIN) / ADC_REFERENCE_VOLTAGE))
</#if><#-- M1_CURRENT_SENSING_TOPO != "ICS_SENSORS" -->

/* Current conversion from Ampere unit to 16Bit Digit */
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_CURRENT_SENSING_TOPO != "ICS_SENSORS">
#define CURRENT_CONV_FACTOR2                (uint16_t)((65536.0 * RSHUNT2 * AMPLIFICATION_GAIN2)/ADC_REFERENCE_VOLTAGE)
#define CURRENT_CONV_FACTOR_INV2            (1.0 / ((65536.0 * RSHUNT2 * AMPLIFICATION_GAIN2) / ADC_REFERENCE_VOLTAGE))
  <#else><#-- M2_CURRENT_SENSING_TOPO == "ICS_SENSORS" -->
#define CURRENT_CONV_FACTOR2                (uint16_t)((65536.0 * AMPLIFICATION_GAIN2)/ADC_REFERENCE_VOLTAGE)
#define CURRENT_CONV_FACTOR_INV2            (1.0 / ((65536.0 * AMPLIFICATION_GAIN2) / ADC_REFERENCE_VOLTAGE))
  </#if><#-- M2_CURRENT_SENSING_TOPO != "ICS_SENSORS" -->
<#elseif MC.M2_DRIVE_TYPE == "SIX_STEP" && MC.DRIVE_NUMBER != "1">
  <#if MC.M2_CURRENT_SENSING_TOPO != "ICS_SENSORS">
    <#if MC.CURRENT_LIMITER_OFFSET>
#define CURRENT_CONV_FACTOR2                ((1000 * RSHUNT2 * AMPLIFICATION_GAIN2))
#define CURRENT_CONV_FACTOR_INV2            (1.0 / ((1000 * RSHUNT2 * AMPLIFICATION_GAIN2))
    <#else><#-- !MC.CURRENT_LIMITER_OFFSET -->
#define CURRENT_CONV_FACTOR2                ((RSHUNT2 * AMPLIFICATION_GAIN2) / ADC_REFERENCE_VOLTAGE)
#define CURRENT_CONV_FACTOR_INV2            (1.0 / ((RSHUNT2 * AMPLIFICATION_GAIN2) / ADC_REFERENCE_VOLTAGE))
    </#if><#-- MC.CURRENT_LIMITER_OFFSET -->
  <#else><#-- M2_CURRENT_SENSING_TOPO == "ICS_SENSORS" -->
    <#if MC.CURRENT_LIMITER_OFFSET>
#define CURRENT_CONV_FACTOR2                ((1000 * AMPLIFICATION_GAIN2))
#define CURRENT_CONV_FACTOR_INV2            (1.0 / ((1000 * AMPLIFICATION_GAIN2))
    <#else><#-- !MC.CURRENT_LIMITER_OFFSET -->
#define CURRENT_CONV_FACTOR2                ((AMPLIFICATION_GAIN2) / ADC_REFERENCE_VOLTAGE)
#define CURRENT_CONV_FACTOR_INV2            (1.0 / ((AMPLIFICATION_GAIN2) / ADC_REFERENCE_VOLTAGE))
    </#if><#-- MC.CURRENT_LIMITER_OFFSET -->
  </#if><#-- M2_CURRENT_SENSING_TOPO != "ICS_SENSORS" --> 
</#if><#-- MC.DRIVE_NUMBER != "1" --> 

<#if MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST">
/**
  * @brief  definition of ADC used for oversampling
  */
enum _OVS_ADC_e_
{
  <#if MC.M1_CS_ADC_NUM == "2">
  OVS_${MC.M1_CS_ADC_U} = 0,
  OVS_<#if MC.M1_CS_ADC_U == MC.M1_CS_ADC_V>${MC.M1_CS_ADC_W}<#else>${MC.M1_CS_ADC_V}</#if> 
  <#else>
    OVS_${MC.M1_CS_ADC_U}, 
    OVS_${MC.M1_CS_ADC_V}, 
    OVS_${MC.M1_CS_ADC_W} 
  </#if>  
};

/**
  * @brief  definition of ADC channel rank used for oversampling
  */
enum _OVS_RANK_e_
{
  OVS_RANK1 = 0,
  OVS_RANK2,
  OVS_RANK3,
  OVS_RANK4,
  OVS_RANK5,
  OVS_RANK6,
  OVS_RANK7,
  OVS_RANK8,
  // etc...
};
<#else>
#define NOMINAL_CURRENT                     (NOMINAL_CURRENT_A * CURRENT_CONV_FACTOR)
</#if>
#define ADC_REFERENCE_VOLTAGE               ${MC.ADC_REFERENCE_VOLTAGE}
<#if MC.M1_CURRENT_SENSING_TOPO != "ICS_SENSORS">
#define M1_MAX_READABLE_CURRENT  (ADC_REFERENCE_VOLTAGE / (2 * RSHUNT * AMPLIFICATION_GAIN))
<#else><#-- MC.M1_CURRENT_SENSING_TOPO == "ICS_SENSORS" -->
#define M1_MAX_READABLE_CURRENT  (ADC_REFERENCE_VOLTAGE / (2 * AMPLIFICATION_GAIN))
</#if><#-- MC.M1_CURRENT_SENSING_TOPO != "ICS_SENSORS" -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_CURRENT_SENSING_TOPO != "ICS_SENSORS">
#define M2_MAX_READABLE_CURRENT (ADC_REFERENCE_VOLTAGE / (2 * RSHUNT2 * AMPLIFICATION_GAIN2))
  <#else><#-- MC.M1_CURRENT_SENSING_TOPO == "ICS_SENSORS" -->
#define M2_MAX_READABLE_CURRENT  (ADC_REFERENCE_VOLTAGE / (2 * AMPLIFICATION_GAIN2))
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO != "ICS_SENSORS" -->
</#if>

/************************* CONTROL FREQUENCIES & DELAIES **********************/
#define TF_REGULATION_RATE                  (uint32_t)((uint32_t)(PWM_FREQUENCY) / (REGULATION_EXECUTION_RATE))

/* TF_REGULATION_RATE_SCALED is TF_REGULATION_RATE divided by PWM_FREQ_SCALING to allow more dynamic */ 
#define TF_REGULATION_RATE_SCALED           (uint16_t)((uint32_t)(PWM_FREQUENCY) / (REGULATION_EXECUTION_RATE\
                                            * PWM_FREQ_SCALING))

/* DPP_CONV_FACTOR is introduce to compute the right DPP with TF_REGULATOR_SCALED  */
#define DPP_CONV_FACTOR                     (65536 / PWM_FREQ_SCALING) 

/* Current conversion from Ampere unit to 16Bit Digit */
#define ID_DEMAG                            (ID_DEMAG_A * CURRENT_CONV_FACTOR)
#define IQMAX                               (IQMAX_A * CURRENT_CONV_FACTOR)

#define DEFAULT_TORQUE_COMPONENT            (DEFAULT_TORQUE_COMPONENT_A * CURRENT_CONV_FACTOR)
#define DEFAULT_FLUX_COMPONENT              (DEFAULT_FLUX_COMPONENT_A * CURRENT_CONV_FACTOR)
#define REP_COUNTER                         (uint16_t)((REGULATION_EXECUTION_RATE * 2u) - 1u)
#define SYS_TICK_FREQUENCY                  (uint16_t)2000
#define UI_TASK_FREQUENCY_HZ                10U
#define PHASE1_FINAL_CURRENT                (PHASE1_FINAL_CURRENT_A * CURRENT_CONV_FACTOR)
#define PHASE2_FINAL_CURRENT                (PHASE2_FINAL_CURRENT_A * CURRENT_CONV_FACTOR)
#define PHASE3_FINAL_CURRENT                (PHASE3_FINAL_CURRENT_A * CURRENT_CONV_FACTOR)
#define PHASE4_FINAL_CURRENT                (PHASE4_FINAL_CURRENT_A * CURRENT_CONV_FACTOR)
#define PHASE5_FINAL_CURRENT                (PHASE5_FINAL_CURRENT_A * CURRENT_CONV_FACTOR)
<#if (MC.DRIVE_NUMBER != "1") >
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
#define M1_ICL_VOLTAGE_THRESHOLD_VOLT    (uint16_t) M1_ICL_VOLTAGE_THRESHOLD 
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
#define M2_ICL_VOLTAGE_THRESHOLD_VOLT       (uint16_t)M2_ICL_VOLTAGE_THRESHOLD 
  </#if><#-- MC.M2_ICL_ENABLED -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

<#if (MC.M1_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
/************************* COMMON OBSERVER PARAMETERS **************************/
#define MAX_BEMF_VOLTAGE                    (uint16_t)((MAX_APPLICATION_SPEED_RPM * 1.2 *\
                                            MOTOR_VOLTAGE_CONSTANT * SQRT_2) / (1000u * SQRT_3))
  <#if MC.M1_BUS_VOLTAGE_READING>
/* max phase voltage, 0-peak Volts*/
#define MAX_VOLTAGE                         (int16_t)((ADC_REFERENCE_VOLTAGE / SQRT_3) / VBUS_PARTITIONING_FACTOR)
  <#else><#-- MC.M1_BUS_VOLTAGE_READING -->
#define MAX_VOLTAGE                         (int16_t)(500 / 2) /* Virtual sensor conversion factor */
  </#if><#-- MC.M1_BUS_VOLTAGE_READING -->
  <#if (MC.M1_CURRENT_SENSING_TOPO == "ICS_SENSORS")> 
#define MAX_CURRENT                         (ADC_REFERENCE_VOLTAGE / (2 * AMPLIFICATION_GAIN))
  <#else><#--(MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
#define MAX_CURRENT                         (ADC_REFERENCE_VOLTAGE / (2 * RSHUNT * AMPLIFICATION_GAIN))
  </#if><#-- (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
#define OBS_MINIMUM_SPEED_UNIT              (uint16_t)((OBS_MINIMUM_SPEED_RPM * SPEED_UNIT) / U_RPM)
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC")
        || (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->
#define MAX_APPLICATION_SPEED_UNIT          ((MAX_APPLICATION_SPEED_RPM * SPEED_UNIT) / U_RPM)
#define MIN_APPLICATION_SPEED_UNIT          ((MIN_APPLICATION_SPEED_RPM * SPEED_UNIT) / U_RPM)

<#if (MC.M1_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL")>
/************************* PLL PARAMETERS **************************/
#define C1                                  (int32_t)((((int16_t)F1) * RS) / (LS * TF_REGULATION_RATE))
#define C2                                  (int32_t)GAIN1
#define C3                                  (int32_t)((((int16_t)F1) * MAX_BEMF_VOLTAGE)\
                                            / (LS * MAX_CURRENT * TF_REGULATION_RATE))
#define C4                                  (int32_t)GAIN2
#define C5                                  (int32_t)((((int16_t)F1) * MAX_VOLTAGE)\
                                            / (LS * MAX_CURRENT * TF_REGULATION_RATE))
#define PERCENTAGE_FACTOR                   (uint16_t)(VARIANCE_THRESHOLD*128u)
#define HFI_MINIMUM_SPEED                   (uint16_t) (HFI_MINIMUM_SPEED_RPM/6u)
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") -->

<#if (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
/*********************** OBSERVER + CORDIC PARAMETERS *************************/
#define CORD_C1                             (int32_t)((((int16_t)CORD_F1) * RS) / (LS * TF_REGULATION_RATE))
#define CORD_C2                             (int32_t)CORD_GAIN1
#define CORD_C3                             (int32_t)((((int16_t)CORD_F1) * MAX_BEMF_VOLTAGE) / (LS * MAX_CURRENT\
                                            * TF_REGULATION_RATE))
#define CORD_C4                             (int32_t)CORD_GAIN2
#define CORD_C5                             (int32_t)((((int16_t)CORD_F1) * MAX_VOLTAGE) / (LS * MAX_CURRENT\
                                            * TF_REGULATION_RATE))
#define CORD_PERCENTAGE_FACTOR              (uint16_t)(CORD_VARIANCE_THRESHOLD*128u)
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->

<#if (MC.M2_SPEED_SENSOR == "STO_PLL") ||  (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")
  || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
/************************* COMMON OBSERVER PARAMETERS Motor 2 **************************/
#define MAX_BEMF_VOLTAGE2                  (uint16_t)((MAX_APPLICATION_SPEED_RPM2 * 1.2\
                                            * MOTOR_VOLTAGE_CONSTANT2 * SQRT_2) / (1000u * SQRT_3))

  <#if MC.M2_BUS_VOLTAGE_READING>
/* max phase voltage, 0-peak Volts*/
#define MAX_VOLTAGE2                        (int16_t)((ADC_REFERENCE_VOLTAGE / SQRT_3) / VBUS_PARTITIONING_FACTOR2)
  <#else><#-- MC.M2_BUS_VOLTAGE_READING -->
#define MAX_VOLTAGE2                        (int16_t)(500 / 2) /* Virtual sensor conversion factor */
  </#if><#-- MC.M2_BUS_VOLTAGE_READING -->

  <#if MC.M2_CURRENT_SENSING_TOPO == "ICS_SENSORS"> 
#define MAX_CURRENT2                        (ADC_REFERENCE_VOLTAGE / (2 * AMPLIFICATION_GAIN2))
  <#else><#--ICS_SENSORS2 -->
#define MAX_CURRENT2                        (ADC_REFERENCE_VOLTAGE / (2 * RSHUNT2 * AMPLIFICATION_GAIN2))
  </#if><#--ICS_SENSORS2 -->
#define OBS_MINIMUM_SPEED_UNIT2             (uint16_t)((OBS_MINIMUM_SPEED_RPM2 * SPEED_UNIT) / U_RPM)
</#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") ||  (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")
        || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->
#define MAX_APPLICATION_SPEED_UNIT2         ((MAX_APPLICATION_SPEED_RPM2 * SPEED_UNIT) / U_RPM)
#define MIN_APPLICATION_SPEED_UNIT2         ((MIN_APPLICATION_SPEED_RPM2 * SPEED_UNIT) / U_RPM)

<#if (MC.M2_SPEED_SENSOR == "STO_PLL") ||  (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL")>
/************************* PLL PARAMETERS **************************/
#define C12                                 (int32_t)((((int16_t)F12) * RS2) / (LS2 * TF_REGULATION_RATE2))
#define C22                                 (int32_t) GAIN12
#define C32                                 (int32_t)((((int16_t)F12) * MAX_BEMF_VOLTAGE2) / (LS2 * MAX_CURRENT2\
                                            * TF_REGULATION_RATE2))
#define C42                                 (int32_t) GAIN22
#define C52                                 (int32_t)((((int16_t)F12) * MAX_VOLTAGE2) / (LS2 * MAX_CURRENT2\
                                            * TF_REGULATION_RATE2))
#define PERCENTAGE_FACTOR2                  (uint16_t)(VARIANCE_THRESHOLD2 * 128u)
#define HFI_MINIMUM_SPEED2                  (uint16_t)(HFI_MINIMUM_SPEED_RPM2 / 6u)
</#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") ||  (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL") -->

<#if (MC.M2_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
#define CORD_C12                            (int32_t)((((int16_t)CORD_F12) * RS2) / (LS2 * TF_REGULATION_RATE2))
#define CORD_C22                            (int32_t)CORD_GAIN12
#define CORD_C32                            (int32_t)((((int16_t)CORD_F12) * MAX_BEMF_VOLTAGE2)\
                                            / (LS2 * MAX_CURRENT2 * TF_REGULATION_RATE2))
#define CORD_C42                            (int32_t)CORD_GAIN22
#define CORD_C52                            (int32_t)((((int16_t)CORD_F12) * MAX_VOLTAGE2)\
                                            / (LS2 * MAX_CURRENT2 * TF_REGULATION_RATE2))
#define CORD_PERCENTAGE_FACTOR2             (uint16_t)(CORD_VARIANCE_THRESHOLD2 * 128u)
</#if><#-- (MC.M2_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")-->


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
#define OVERVOLTAGE_THRESHOLD_LOW_d2           (uint16_t)(M2_OVP_THRESHOLD_LOW * 65535 /\
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
<#if MC.MOTOR_PROFILER == true>
#undef MAX_APPLICATION_SPEED_RPM
#define MAX_APPLICATION_SPEED_RPM           50000
#undef F1
#define F1                                  0
#undef CORD_F1
#define CORD_F1                             0
#define SPEED_REGULATOR_BANDWIDTH           0 /* Dummy value */
#define LDLQ_RATIO                          1.000 /*!< Ld vs Lq ratio.*/
#define RESISTOR_OFFSET                     ${MC.RESISTOR_OFFSET} /* computed resitance offset to compensate error meaasuremnt due to board */
#define BUS_VOLTAGE_CONVERSION_FACTOR       63.2
#define CURRENT_REGULATOR_BANDWIDTH         6000
#define MP_KP                               1.00f  /* Initial Kp factor of the speed regulator */
#define MP_KI                               0.1f   /* Initial Ki factor of the speed regulator */
#define DC_CURRENT_RS_MEAS                  ((${MC.M1_BOARD_MAX_CURRENT}) * 80) / 100 /* Maximum level of direct current = 80% of Max Board current */
#define I_THRESHOLD                         0.05
</#if><#-- MC.MOTOR_PROFILER -->

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES                   (uint16_t)(((uint32_t)ADV_TIM_CLK_MHz * (uint32_t)1000000u\
                                            / ((uint32_t)(PWM_FREQUENCY))) & (uint16_t)0xFFFE)

<#if MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST">
#define M1_AUX_TIM_PERIOD_CYCLES            (uint16_t)((((uint32_t)ADV_TIM_CLK_MHz * (uint32_t)1000000u\
                                            / ((uint32_t)(PWM_FREQUENCY)*OVS_COUNT)))-1)
</#if><#-- MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST" -->

<#if MC.DRIVE_NUMBER != "1">
#define PWM_PERIOD_CYCLES2                  (uint16_t)(((uint32_t)ADV_TIM_CLK_MHz2 * (uint32_t)1000000u\
                                            / ((uint32_t)(PWM_FREQUENCY2))) & (uint16_t)0xFFFE)
</#if><#-- MC.DRIVE_NUMBER > 1 -->

<#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
#define DEADTIME_NS                         SW_DEADTIME_NS
<#else><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING != "LS_PWM_TIMER" -->
#define DEADTIME_NS                         HW_DEAD_TIME_NS
</#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER" -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
#define DEADTIME_NS2                        SW_DEADTIME_NS2
  <#else><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING != "LS_PWM_TIMER" -->
#define DEADTIME_NS2                        HW_DEAD_TIME_NS2
  </#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER" -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->
#define DEAD_TIME_ADV_TIM_CLK_MHz           (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1                  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS / 1000uL)
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
<#if MC.DRIVE_NUMBER != "1">
#define DEAD_TIME_ADV_TIM_CLK_MHz2          (ADV_TIM_CLK_MHz2 * TIM_CLOCK_DIVIDER2)
#define DEAD_TIME_COUNTS2_1                 (DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/1000uL)
#if (DEAD_TIME_COUNTS2_1 <= 255)
#define DEAD_TIME_COUNTS2                   (uint16_t) DEAD_TIME_COUNTS2_1
#elif (DEAD_TIME_COUNTS2_1 <= 508)
#define DEAD_TIME_COUNTS2                   (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/2)\
                                            / 1000uL) + 128)
#elif (DEAD_TIME_COUNTS2_1 <= 1008)
#define DEAD_TIME_COUNTS2                   (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/8)\
                                            / 1000uL) + 320)
#elif (DEAD_TIME_COUNTS2_1 <= 2015)
#define DEAD_TIME_COUNTS2                   (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz2 * DEADTIME_NS2/16)\
                                            / 1000uL) + 384)
#else
#define DEAD_TIME_COUNTS2                   510
#endif
</#if><#-- MC.DRIVE_NUMBER > 1 -->
#define DTCOMPCNT                           (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz) / 2000)
#define TON_NS                              500
#define TOFF_NS                             500
#define TON                                 (uint16_t)((TON_NS * ADV_TIM_CLK_MHz)  / 2000)
#define TOFF                                (uint16_t)((TOFF_NS * ADV_TIM_CLK_MHz) / 2000)
<#if MC.DRIVE_NUMBER != "1">
#define DTCOMPCNT2                          (uint16_t)((DEADTIME_NS2 * ADV_TIM_CLK_MHz2) / 2000)
#define TON_NS2                             500
#define TOFF_NS2                            500
#define TON2                                (uint16_t)((TON_NS2 * ADV_TIM_CLK_MHz2)  / 2000)
#define TOFF2                               (uint16_t)((TOFF_NS2 * ADV_TIM_CLK_MHz2) / 2000)
</#if><#-- MC.DRIVE_NUMBER > 1 -->

/**********************/
/* MOTOR 1 ADC Timing */
/**********************/
/* In ADV_TIMER CLK cycles*/
#define SAMPLING_TIME                       ((ADC_SAMPLING_CYCLES * ADV_TIM_CLK_MHz) / ADC_CLK_MHz) 

#define TRISE                               ((TRISE_NS * ADV_TIM_CLK_MHz)/1000uL)
#define TDEAD                               ((uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz)/1000))
#define TNOISE                              ((uint16_t)((TNOISE_NS*ADV_TIM_CLK_MHz)/1000))
<#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#define TMAX_TNTR                           ((uint16_t)((MAX_TNTR_NS * ADV_TIM_CLK_MHz)/1000uL))
#define TAFTER                              ((uint16_t)(TDEAD + TMAX_TNTR))
#define TBEFORE                             ((uint16_t)(((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES)\ 
                                            * ADV_TIM_CLK_MHz) / ADC_CLK_MHz)  + 1U)
#define TMIN                                ((uint16_t)( TAFTER + TBEFORE ))
#define HTMIN                               ((uint16_t)(TMIN >> 1))
#define CHTMIN                              ((uint16_t)(TMIN / (REGULATION_EXECUTION_RATE * 2)))
#if (TRISE > SAMPLING_TIME)
#define MAX_TRTS                            (2 * TRISE)
#else
#define MAX_TRTS                            (2 * SAMPLING_TIME)
#endif
<#else>
#define HTMIN 1 /* Required for main.c compilation only, CCR4 is overwritten at runtime */
#define TW_AFTER                            ((uint16_t)(((DEADTIME_NS+MAX_TNTR_NS)*ADV_TIM_CLK_MHz)/1000UL))
#define MAX_TWAIT                           ((uint16_t)((TW_AFTER - SAMPLING_TIME)/2))
  <#if MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST">
    <#if MC.M1_CS_ADC_NUM == "2" >
#define TW_BEFORE                           ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + (ADC_SAMPLING_CYCLES * 2)\
                                            + ADC_SAR_CYCLES) * ADV_TIM_CLK_MHz) / ADC_CLK_MHz  + 1u)
    <#else>
#define TW_BEFORE                           ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES)\
                                            * ADV_TIM_CLK_MHz) / ADC_CLK_MHz  + 1u)
    </#if>
  <#else>
#define TW_BEFORE                           ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES)\
                                            * ADV_TIM_CLK_MHz) / ADC_CLK_MHz  + 1u)
#define TW_BEFORE_R3_1                      ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + (ADC_SAMPLING_CYCLES * 2)\
                                            + ADC_SAR_CYCLES) * ADV_TIM_CLK_MHz) / ADC_CLK_MHz  + 1u)
  </#if>
</#if>
  
<#if MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST">
/* Number of Timer ticks above which low a side current shunt sample is disqualified 
 * and needs to be reconstructed. 
 * Duty cycle fraction converted into ticks in the same scale as PWM duty */
#define MAX_TICKS_CURRENT_RECONSTRUCTION_THRESHOLD ((((CURRENT_RECONSTRUCTION_THRESHOLD + 1.0) / 2.0))\
                                                    * PWM_PERIOD_CYCLES / 2)

  <#if (MC.M1_AMPLIFICATION_GAIN?number >0)> <#assign ampSign = "-"> <#else> <#assign ampSign = ""> </#if>
  <#if MC.M1_CURRENT_SENSING_TOPO == "THREE_SHUNT" >
#define ADC_CURRENT_SCALE                    (${ampSign}(float_t)ADC_REFERENCE_VOLTAGE / (float_t)AMPLIFICATION_GAIN\
                                             / (float_t)RSHUNT)
  <#elseif MC.M1_CURRENT_SENSING_TOPO == "ICS_SENSORS">	
#define ADC_CURRENT_SCALE                    (${ampSign}(float_t)ADC_REFERENCE_VOLTAGE / (float_t)AMPLIFICATION_GAIN)
  </#if>
  
#define ADC_VOLTAGE_SCALE                    ((float_t)ADC_REFERENCE_VOLTAGE\
                                             / (float_t)M1_PHASE_VOLTAGE_PARTITIONING_FACTOR)
#define ADC_BUSVOLTAGE_SCALE                 ((float_t)ADC_REFERENCE_VOLTAGE / (float_t)VBUS_PARTITIONING_FACTOR)
#define CURRENT_FACTOR(CurrentScale)         (ADC_CURRENT_SCALE/(CurrentScale))
#define VOLTAGE_FACTOR(VoltageScale)         (ADC_VOLTAGE_SCALE/(VoltageScale))
#define BUSVOLTAGE_FACTOR(VoltageScale)      (ADC_BUSVOLTAGE_SCALE/(VoltageScale))
#define OVS_COUNT                            (${MC.OVS_COUNT}) /*!< @brief  Oversampling level. default value is 4 */
#define OVS_NUM_ADCS                         (${MC.M1_CS_ADC_NUM}) /*!< @brief number of ADCs used for Motor Control */
#define OVS_LONGEST_TASK                     (${maxLength}) /*!< @brief Longest ADC sequence length */
#define OVS_ADC1_TASKLENGTH                  <#if ADC1Length == 0>(0)<#else>(${maxLength})</#if> /*!< @brief ADC1 sequence length */
#define OVS_ADC2_TASKLENGTH                  <#if ADC2Length == 0>(0)<#else>(${maxLength})</#if> /*!< @brief ADC2 sequence length */
#define OVS_ADC3_TASKLENGTH                  <#if ADC3Length == 0>(0)<#else>(${maxLength})</#if> /*!< @brief ADC3 sequence length */
#define OVS_ADC4_TASKLENGTH                  <#if ADC4Length == 0>(0)<#else>(${maxLength})</#if> /*!< @brief ADC4 sequence length */
#define OVS_ADC5_TASKLENGTH                  <#if ADC5Length == 0>(0)<#else>(${maxLength})</#if> /*!< @brief ADC5 sequence length */
#define OVS_ADC_I_R                          OVS_${MC.M1_CS_ADC_U} /*!< @brief ADC used to sample phase current R */
#define OVS_RANK_I_R                         OVS_RANK${MC.M1_CS_ADC_RANK_U} /*!< @brief rank of the ADC channel used to sample phase current R */
#define OVS_ADC_I_S                          OVS_${MC.M1_CS_ADC_V} /*!< @brief ADC used to sample phase current S */
#define OVS_RANK_I_S                         OVS_RANK${MC.M1_CS_ADC_RANK_V} /*!< @brief rank of the ADC channel used to sample phase current S */
#define OVS_ADC_I_T                          OVS_${MC.M1_CS_ADC_W} /*!< @brief ADC used to sample phase current T */
#define OVS_RANK_I_T                         OVS_RANK${MC.M1_CS_ADC_RANK_W} /*!< @brief rank of the ADC channel used to sample phase current T */
#define OVS_ADC_U_R                          OVS_${MC.M1_VS_ADC_U} /*!< @brief ADC used to sample voltage R */
#define OVS_RANK_U_R                         OVS_RANK${MC.M1_VS_ADC_RANK_U} /*!< @brief rank of the ADC channel used to sample voltage R */
#define OVS_ADC_U_S                          OVS_${MC.M1_VS_ADC_V} /*!< @brief ADC used to sample voltage S */
#define OVS_RANK_U_S                         OVS_RANK${MC.M1_VS_ADC_RANK_V} /*!< @brief rank of the ADC channel used to sample voltage S */
#define OVS_ADC_U_T                          OVS_${MC.M1_VS_ADC_W} /*!< @brief ADC used to sample voltage T */
#define OVS_RANK_U_T                         OVS_RANK${MC.M1_VS_ADC_RANK_W} /*!< @brief rank of the ADC channel used to sample voltage T */

    <#if MC.M1_BUS_VOLTAGE_READING>
      <#if (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_U) || (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_V) || (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_W)>
#define OVS_ADC_U_DC                         OVS_${MC.M1_VBUS_ADC} /*!< @brief ADC used to sample DC Bus */
#define OVS_RANK_U_DC                        OVS_RANK${MC.M1_VBUS_ADC_RANK} /*!< @brief rank of the ADC channel used to sample DC Bus */
      </#if>
    </#if>
    <#if MC.M1_POTENTIOMETER_ENABLE>
      <#if (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_U) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_V) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_W)>
#define OVS_ADC_THROTTLE                     OVS_${MC.POTENTIOMETER_ADC} /*!< @brief ADC used to sample throttle/potentiometer */
#define OVS_RANK_THROTTLE                    OVS_RANK${MC.M1_POTENTIOMETER_ADC_RANK} /*!< @brief rank of the ADC channel used to sample throttle / potentiometer */
      </#if>
    </#if>
    <#if MC.M1_TEMPERATURE_READING >
      <#if (MC.M1_TEMP_FDBK_ADC ==  MC.M1_CS_ADC_U) || (MC.M1_TEMP_FDBK_ADC ==  MC.M1_CS_ADC_V) || (MC.M1_TEMP_FDBK_ADC ==  MC.M1_CS_ADC_W)>
#define OVS_ADC_TEMP                         OVS_${MC.M1_TEMP_FDBK_ADC} /*!< @brief ADC used to sample temperature */
#define OVS_RANK_TEMP                        OVS_RANK${MC.M1_TEMP_ADC_RANK} /*!< @brief rank of the ADC channel used to sample temperature */
      </#if>
    </#if>
  <#if (MC.M1_CURRENT_SENSING_TOPO == "ICS_SENSORS")>
  
/* Factor for <OVS_COUNT> samples per period for non-shunt */
#define OVERSAMPLING_FIXP_CURRENT_FACTOR(Currentscale) (FIXP30(CURRENT_FACTOR(Currentscale)\
                                                       / (OVS_COUNT * REGULATION_EXECUTION_RATE)))
  <#else>
/* Factor for one sample per period for shunt */
#define OVERSAMPLING_FIXP_CURRENT_FACTOR(Currentscale) (FIXP30(CURRENT_FACTOR(Currentscale)\
                                                       / REGULATION_EXECUTION_RATE))
   </#if>
#define OVERSAMPLING_FIXP_VOLTAGE_FACTOR(VoltageScale) (FIXP30(VOLTAGE_FACTOR(VoltageScale)\
                                                       / (OVS_COUNT * REGULATION_EXECUTION_RATE)))
#define OVERSAMPLING_FIXP_BUSVOLTAGE_FACTOR(VoltageScale) (FIXP30(BUSVOLTAGE_FACTOR(VoltageScale)\
                                                       / (OVS_COUNT * REGULATION_EXECUTION_RATE)))

/* Index of the sample used for single-sampled channels, the low side shunt measurements  */
#if (OVS_COUNT == 1)
#define BOARD_SHUNT_SAMPLE_SELECT (0)	
#else
#define BOARD_SHUNT_SAMPLE_SELECT ((OVS_COUNT/2) - 1)	
#endif
</#if>

<#if MC.DRIVE_NUMBER != "1">
/**********************/
/* MOTOR 2 ADC Timing */
/**********************/
/* In ADV_TIMER2 CLK cycles*/ 
#define SAMPLING_TIME2                      ((ADC_SAMPLING_CYCLES2 * ADV_TIM_CLK_MHz2) / ADC_CLK_MHz2)
#define TRISE2                              (((TRISE_NS2) * ADV_TIM_CLK_MHz2)/1000uL)
#define TDEAD2                              ((uint16_t)((DEADTIME_NS2 * ADV_TIM_CLK_MHz2) / 1000uL))
#define TNOISE2                             ((uint16_t)((TNOISE_NS2*ADV_TIM_CLK_MHz2) / 1000ul))
  <#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
#define TMAX_TNTR2                          ((uint16_t)((MAX_TNTR_NS2 * ADV_TIM_CLK_MHz) / 1000uL))
#define TAFTER2                             ((uint16_t)( TDEAD2 + TMAX_TNTR2 ))
#define TBEFORE2                            (((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES2)\
                                            * ADV_TIM_CLK_MHz2) / ADC_CLK_MHz2)  + 1u)
#define TMIN2                               (TAFTER2 + TBEFORE2)
#define HTMIN2                              (uint16_t)(TMIN2 >> 1)
#define CHTMIN2                             (uint16_t)(TMIN2 / (REGULATION_EXECUTION_RATE2 * 2))
#if (TRISE2 > SAMPLING_TIME2)
#define MAX_TRTS2                           (2 * TRISE2)
#else
#define MAX_TRTS2                           (2 * SAMPLING_TIME2)
#endif
  <#else>
/* Required for main.c compilation only, CCR4 is overwritten at runtime */
#define HTMIN2                              0
#define TW_BEFORE2                          ((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + ADC_SAMPLING_CYCLES2)\
                                            * ADV_TIM_CLK_MHz2) / ADC_CLK_MHz2  + 1u)
#define TW_BEFORE_R3_1_2                    (((uint16_t)((ADC_TRIG_CONV_LATENCY_CYCLES + (ADC_SAMPLING_CYCLES2 * 2)\
                                            + ADC_SAR_CYCLES) * ADV_TIM_CLK_MHz2) / ADC_CLK_MHz2)  + 1u)
#define TW_AFTER2                           ((uint16_t)(((DEADTIME_NS2 + MAX_TNTR_NS2) * ADV_TIM_CLK_MHz2) / 1000UL))
#define MAX_TWAIT2                          ((uint16_t)((TW_AFTER2 - SAMPLING_TIME2) / 2))
  </#if><#-- ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))-->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

/* USER CODE BEGIN temperature */
#define M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE 25u
#define M1_TEMP_SW_FILTER_BW_FACTOR         250u
/* USER CODE END temperature */

<#if MC.M1_FLUX_WEAKENING_ENABLING || MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING>
/* Flux Weakening - Feed forward */
#define M1_VQD_SW_FILTER_BW_FACTOR          128u
#define M1_VQD_SW_FILTER_BW_FACTOR_LOG      LOG2(M1_VQD_SW_FILTER_BW_FACTOR)
</#if><#-- MC.M1_FLUX_WEAKENING_ENABLING || MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING -->

<#if MC.DRIVE_NUMBER != "1">
/* USER CODE BEGIN temperature M2*/
#define M2_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE 25u
#define M2_TEMP_SW_FILTER_BW_FACTOR         250u
/* USER CODE END temperature M2*/

  <#if MC.M2_FLUX_WEAKENING_ENABLING || MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING>
/* Flux Weakening - Feed forward Motor 2*/
#define M2_VQD_SW_FILTER_BW_FACTOR          128u
#define M2_VQD_SW_FILTER_BW_FACTOR_LOG      LOG2(M2_VQD_SW_FILTER_BW_FACTOR)
  </#if><#-- MC.M2_FLUX_WEAKENING_ENABLING || MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

<#if (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS')>
#define PQD_CONVERSION_FACTOR               (float_t)(((1.732 * ADC_REFERENCE_VOLTAGE) /\
                                            (AMPLIFICATION_GAIN)) / 65536.0f)
<#else>
#define PQD_CONVERSION_FACTOR               (float_t)(((1.732 * ADC_REFERENCE_VOLTAGE) /\
                                            (RSHUNT * AMPLIFICATION_GAIN)) / 65536.0f)
</#if><#-- (MC.M1_CURRENT_SENSING_TOPO == 'ICS_SENSORS') -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_CURRENT_SENSING_TOPO == 'ICS_SENSORS'>
#define PQD_CONVERSION_FACTOR2              (float_t)(((1.732 * ADC_REFERENCE_VOLTAGE) /\
                                            (AMPLIFICATION_GAIN2)) / 65536.0f)
  <#else><#-- ICS_SENSORS2 -->
#define PQD_CONVERSION_FACTOR2              (float_t)(((1.732 * ADC_REFERENCE_VOLTAGE) /\
                                            (RSHUNT2 * AMPLIFICATION_GAIN2)) / 65536.0f)
  </#if><#-- ICS_SENSORS2 -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->
<#if MC.DRIVE_NUMBER != "1">
<#-- If this check really needed? -->
  <#if M1_FREQ_RELATION == 'HIGHEST_FREQ'> 
<#-- First instance has higher frequency --> 
#define PWM_FREQUENCY_CHECK_RATIO           (PWM_FREQUENCY * 10000u / PWM_FREQUENCY2)
  <#else><#-- ${M1_FREQ_RELATION} != 'HIGHEST_FREQ' -->
<#-- Second instance has higher frequency -->
#define PWM_FREQUENCY_CHECK_RATIO           (PWM_FREQUENCY2 * 10000u / PWM_FREQUENCY)
  </#if><#-- ${M1_FREQ_RELATION} == 'HIGHEST_FREQ' -->
#define MAGN_FREQ_RATIO                     (${FREQ_RATIO} * 10000u)
#if (PWM_FREQUENCY_CHECK_RATIO != MAGN_FREQ_RATIO)
#error "The two motor PWM frequencies should be integer multiple"  
#endif
</#if><#-- MC.DRIVE_NUMBER > 1 -->

/****** Prepares the UI configurations according the MCconfxx settings ********/
<#if MC.DEBUG_DAC_FUNCTIONALITY_EN == true>
#define DAC_ENABLE | OPT_DAC
#define DAC_OP_ENABLE | UI_CFGOPT_DAC
<#else><#-- MC.DEBUG_DAC_FUNCTIONALITY_EN == false -->
#define DAC_ENABLE
#define DAC_OP_ENABLE
</#if><#-- MC.DEBUG_DAC_FUNCTIONALITY_EN == true -->

/* Motor 1 settings */
<#if MC.M1_FLUX_WEAKENING_ENABLING>
#define FW_ENABLE | UI_CFGOPT_FW
<#else><#-- MC.M1_FLUX_WEAKENING_ENABLING -->
#define FW_ENABLE
</#if><#-- MC.M1_FLUX_WEAKENING_ENABLING -->

<#if MC.DIFFERENTIAL_TERM_ENABLED>
#define DIFFTERM_ENABLE | UI_CFGOPT_SPEED_KD | UI_CFGOPT_Iq_KD | UI_CFGOPT_Id_KD
<#else><#-- MC.DIFFERENTIAL_TERM_ENABLED -->
#define DIFFTERM_ENABLE
</#if><#-- MC.DIFFERENTIAL_TERM_ENABLED -->
<#if MC.DRIVE_NUMBER != "1">
/* Motor 2 settings */
  <#if MC.M2_FLUX_WEAKENING_ENABLING>
#define FW_ENABLE2 | UI_CFGOPT_FW
  <#else><#-- MC.M2_FLUX_WEAKENING_ENABLING -->
#define FW_ENABLE2
  </#if><#-- MC.M2_FLUX_WEAKENING_ENABLING -->

  <#if MC.DIFFERENTIAL_TERM_ENABLED2>
#define DIFFTERM_ENABLE2 | UI_CFGOPT_SPEED_KD | UI_CFGOPT_Iq_KD | UI_CFGOPT_Id_KD
  <#else><#-- MC.DIFFERENTIAL_TERM_ENABLED2 -->
#define DIFFTERM_ENABLE2
  </#if><#-- MC.DIFFERENTIAL_TERM_ENABLED2 -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

/* Sensors setting */
<#-- All this could be simpler... -->
<#if MC.M1_SPEED_SENSOR == "STO_PLL">
#define MAIN_SCFG                           UI_SCODE_STO_PLL
</#if><#-- MC.M1_SPEED_SENSOR == "STO_PLL" -->
<#if MC.M1_SPEED_SENSOR == "STO_CORDIC">
#define MAIN_SCFG                           UI_SCODE_STO_CR
</#if><#-- MC.M1_SPEED_SENSOR == "STO_CORDIC" -->
<#if MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL">
#define AUX_SCFG                            UI_SCODE_STO_PLL
</#if><#-- MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL" -->
<#if MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC">
#define AUX_SCFG                            UI_SCODE_STO_CR
</#if><#-- MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC" -->
<#if (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#define MAIN_SCFG                           UI_SCODE_ENC
</#if><#-- (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") -->
<#if (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#define AUX_SCFG                            UI_SCODE_ENC
</#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z") -->
<#if MC.M1_SPEED_SENSOR == "HALL_SENSOR">
#define MAIN_SCFG                           UI_SCODE_HALL
</#if><#-- MC.M1_SPEED_SENSOR == "HALL_SENSOR" -->
<#if MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR">
#define AUX_SCFG                            UI_SCODE_HALL
</#if><#-- MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->
<#if (MC.M1_AUXILIARY_SPEED_SENSOR != "STO_CORDIC") && (MC.M1_AUXILIARY_SPEED_SENSOR != "STO_PLL") && (MC.M1_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M1_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER_Z")
  && (MC.M1_AUXILIARY_SPEED_SENSOR != "HALL_SENSOR")>
#define AUX_SCFG                               0x0
</#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR != "STO_CORDIC") && (MC.M1_AUXILIARY_SPEED_SENSOR != "STO_PLL") && (MC.M1_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M1_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER_Z")
        && (MC.M1_AUXILIARY_SPEED_SENSOR != "HALL_SENSOR") -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_SPEED_SENSOR == "STO_PLL">
#define MAIN_SCFG2                          UI_SCODE_STO_PLL
  </#if><#-- MC.M2_SPEED_SENSOR == "STO_PLL" -->
  <#if MC.M2_SPEED_SENSOR == "STO_CORDIC">
#define MAIN_SCFG2                          UI_SCODE_STO_CR
  </#if><#-- MC.M2_SPEED_SENSOR == "STO_CORDIC" -->
  <#if MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL">
#define AUX_SCFG2                           UI_SCODE_STO_PLL
  </#if><#-- MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL" -->
  <#if MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC">
#define AUX_SCFG2                           UI_SCODE_STO_CR
  </#if><#-- MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC" -->
  <#if (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#define MAIN_SCFG2                          UI_SCODE_ENC
  </#if><#-- (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z") -->
  <#if (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z")>
#define AUX_SCFG2                           UI_SCODE_ENC
  </#if><#-- (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z") -->
  <#if MC.M2_SPEED_SENSOR == "HALL_SENSOR">
#define MAIN_SCFG2                          UI_SCODE_HALL
  </#if><#-- MC.M2_SPEED_SENSOR == "HALL_SENSOR" -->
  <#if MC.M2_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR">
#define AUX_SCFG2                           UI_SCODE_HALL
  </#if><#-- MC.M2_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->
  <#if (MC.M2_AUXILIARY_SPEED_SENSOR != "STO_CORDIC") && (MC.M2_AUXILIARY_SPEED_SENSOR != "STO_PLL") && ((MC.M2_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M2_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER_Z"))
    && (MC.M2_AUXILIARY_SPEED_SENSOR != "HALL_SENSOR")>
#define AUX_SCFG2                           0x0
  </#if><#-- (MC.M2_AUXILIARY_SPEED_SENSOR != "STO_CORDIC") && (MC.M2_AUXILIARY_SPEED_SENSOR != "STO_PLL") && ((MC.M2_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M2_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER_Z"))
          && (MC.M2_AUXILIARY_SPEED_SENSOR != "HALL_SENSOR") -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

<#-- Seems Useless -->
<#if MC.PLLTUNING?? && MC.PLLTUNING==true>
#define PLLTUNING_ENABLE | UI_CFGOPT_PLLTUNING
<#else><#-- MC.PLLTUNING?? && MC.PLLTUNING==true -->
#define PLLTUNING_ENABLE
</#if><#-- MC.PLLTUNING?? && MC.PLLTUNING==true -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.PLLTUNING?? && MC.PLLTUNING==true>
#define PLLTUNING_ENABLE2 | UI_CFGOPT_PLLTUNING
  <#else><#-- MC.PLLTUNING?? && MC.PLLTUNING==true -->
#define PLLTUNING_ENABLE2
  </#if><#-- MC.PLLTUNING?? && MC.PLLTUNING==true -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

<#if MC.PFC_ENABLED>
#define UI_CFGOPT_PFC_ENABLE | UI_CFGOPT_PFC
<#else><#-- MC.PFC_ENABLED -->
#define UI_CFGOPT_PFC_ENABLE
</#if><#-- MC.PFC_ENABLED -->

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
/**********  HALL TIMER MOTOR 1 *************/
#define M1_HALL_TIM_PERIOD                  65535
<@define_IC_FILTER motor=1 sensor='HALL' driver=DRIVER icx_filter=MC.M1_HALL_ICx_FILTER?number />
#define SPD_TIM_M1_IRQHandler               ${TimerHandler(_last_word(MC.M1_HALL_TIMER_SELECTION))}
</#if><#-- (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR") -->


<#if (MC.M2_SPEED_SENSOR == "HALL_SENSOR") || (MC.M2_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR")>
/**********  HALL TIMER MOTOR 2 *************/
#define M2_HALL_TIM_PERIOD                  65535
<@define_IC_FILTER motor=2 sensor='HALL' driver=DRIVER icx_filter=MC.M2_HALL_ICx_FILTER?number />
#define SPD_TIM_M2_IRQHandler               ${TimerHandler(_last_word(MC.M2_HALL_TIMER_SELECTION))}
</#if><#-- (MC.M2_SPEED_SENSOR == "HALL_SENSOR") || (MC.M2_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR") -->

<#if M1_ENCODER>
/**********  ENCODER TIMER MOTOR 1 *************/
#define M1_PULSE_NBR                        ((4 * (M1_ENCODER_PPR)) - 1)
<@define_IC_FILTER motor=1 sensor='ENC' driver=DRIVER icx_filter=MC.M1_ENC_ICx_FILTER?number />
#define SPD_TIM_M1_IRQHandler               ${TimerHandler(_last_word(MC.M1_ENC_TIMER_SELECTION))}
</#if><#-- M1_ENCODER -->
  
<#if M2_ENCODER>
/**********  ENCODER TIMER MOTOR 2 *************/
#define M2_PULSE_NBR                        ((4 * (M2_ENCODER_PPR)) - 1)
<@define_IC_FILTER motor=2 sensor='ENC' driver=DRIVER icx_filter=MC.M2_ENC_ICx_FILTER?number />
#define SPD_TIM_M2_IRQHandler               ${TimerHandler(_last_word(MC.M2_ENC_TIMER_SELECTION))}
</#if><#-- M2_ENCODER -->

<#if MC.PFC_ENABLED == true>
#define PFC_ETRFILTER_IC                    ${ Fx_ic_filter(MC.ETRFILTER?number) }
#define PFC_SYNCFILTER_IC                   ${ Fx_ic_filter(MC.SYNCFILTER?number) }
</#if><#-- MC.PFC_ENABLED == true -->
#define LPF_FILT_CONST                      ((int16_t)(32767 * 0.5))

<#if MC.M1_OVERMODULATION == false && MC.M1_CURRENT_SENSING_TOPO != "ICS_SENSORS">
/* MMI Table Motor 1 MAX_MODULATION_${MC.M1_MAX_MODULATION_INDEX}_PER_CENT */
#define MAX_MODULE                          (uint16_t)((${MC.M1_MAX_MODULATION_INDEX}* 32767)/100)
<#else><#-- MC.M1_OVERMODULATION == true || MC.M1_CURRENT_SENSING_TOPO == "ICS_SENSORS" -->
/* MMI Table Motor 1 100% */
#define MAX_MODULE                          32767
</#if><#-- MC.M1_OVERMODULATION == false && MC.M1_CURRENT_SENSING_TOPO != "ICS_SENSORS" -->

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
<#assign LL_ADC_CYCLE_SUFFIX = 'CYCLES'>
#define  SAMPLING_CYCLE_CORRECTION          0 /* ${McuName} ADC sampling time is an integer number */
<#else><#-- CondFamily_STM32F4 || CondFamily_STM32F7 -->
<#assign LL_ADC_CYCLE_SUFFIX = 'CYCLES_5'>
<#-- Addition of the Half cycle of ADC sampling time-->
#define SAMPLING_CYCLE_CORRECTION           0.5 /* Add half cycle required by ${McuName} ADC */
#define LL_ADC_SAMPLINGTIME_1CYCLES_5       LL_ADC_SAMPLINGTIME_1CYCLE_5
</#if><#-- CondFamily_STM32F4 || CondFamily_STM32F7 -->

//cstat !MISRAC2012-Rule-20.10 !DEFINE-hash-multiple
#define LL_ADC_SAMPLING_CYCLE(CYCLE)        LL_ADC_SAMPLINGTIME_ ## CYCLE ## ${LL_ADC_CYCLE_SUFFIX} 
  
#endif /*PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
