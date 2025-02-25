<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
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
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

<#if MC.DRIVE_NUMBER != "1">
/**************************
 *** Motor 1 Parameters ***
 **************************/
<#else>
/************************
 *** Motor Parameters ***
 ************************/
</#if>

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM          ${MC.M1_MAX_APPLICATION_SPEED} /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM          (${MC.M1_MIN_APPLICATION_SPEED} + U_RPM / SPEED_UNIT) /*!< rpm, mechanical, absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS    ${MC.M1_SS_MEAS_ERRORS_BEFORE_FAULTS} /*!< Number of speed measurement errors before main sensor goes in fault */

<#if M1_ENCODER >
/*** Encoder **********************/
#define ENC_AVERAGING_FIFO_DEPTH           ${MC.M1_ENC_AVERAGING_FIFO_DEPTH} /*!< depth of the FIFO used to average  mechanical speed in 0.1Hz resolution */
</#if><#-- M1_ENCODER -->

<#if (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR")>
/****** Hall sensors ************/ 
#define HALL_AVERAGING_FIFO_DEPTH          ${MC.M1_HALL_AVERAGING_FIFO_DEPTH} /*!< depth of the FIFO used to average  mechanical speed in 0.1Hz resolution */
#define HALL_MTPA <#if MC.M1_HALL_MTPA > true <#else> false </#if>
</#if><#-- (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR") -->

<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
/****** Bemf Observer ****/
#define BEMF_AVERAGING_FIFO_DEPTH          ${MC.BEMF_AVERAGING_FIFO_DEPTH} /*!< depth of the FIFO used to average  mechanical speed in 0.1Hz resolution */  
#define VARIANCE_THRESHOLD                 ${MC.M1_VARIANCE_THRESHOLD} /*!< Maximum accepted variance on speed estimates (percentage) */
#define BEMF_THRESHOLD_PWM_PERC            ${MC.M1_BEMF_THRESHOLD_PWM} /*!< Percentage of Bus for zero crossing detection with on-sensing */
#define BEMF_THRESHOLD_HIGH_PERC           ${MC.M1_BEMF_THRESHOLD_HIGH} /*!< Percentage of Bus for zero crossing detection with on-sensing and low side modulation */
#define BEMF_THRESHOLD_LOW_PERC  	       ${MC.M1_BEMF_THRESHOLD_LOW}	/*!< Percentage of Bus for zero crossing detection with on-sensing and high side modulation */
#define BEMF_ADC_TRIG_TIME_DPP             ((uint16_t)${MC.BEMF_ADC_TRIG_TIME_DPP}) /*!< 1/1024 of PWM period elapsed */
  <#if  MC.DRIVE_MODE == "VM">                                                             
#define BEMF_ADC_TRIG_TIME_ON_DPP          ((uint16_t)${MC.BEMF_ADC_TRIG_TIME_ON_DPP}) /*!< 1/1024 of PWM period  elapsed */
#define BEMF_PWM_ON_ENABLE_THRES_DPP       ((uint16_t)${MC.BEMF_PWM_ON_ENABLE_THRES_DPP}) /*!< 1/1024 of PWM period   elapsed */
#define BEMF_PWM_ON_ENABLE_HYSTERESIS_DPP  ((uint16_t)${MC.BEMF_PWM_ON_ENABLE_HYSTERESIS_DPP}) /*!< 1/1024 of PWM period    elapsed */
  </#if> 
#define ZCD_RISING_TO_COMM                 ((uint16_t)${MC.M1_ZCD_RISING_TO_COMM}) /*!< Zero Crossing detection to commutation delay in degrees */
#define ZCD_FALLING_TO_COMM                ((uint16_t)${MC.M1_ZCD_FALLING_TO_COMM}) /*!< Zero Crossing detection to commutation delay in degrees */
#define MIN_DEMAG_TIME                     ((uint16_t)${MC.MIN_DEMAG_TIME}) /*!< Demagnetization delay in number of HF timer periods elapsed before a first 
                                                           BEMF ADC measurement is processed in an attempt to detect the BEMF zero crossing */
#define SPEED_THRESHOLD_DEMAG              ((uint32_t)${MC.SPEED_THRESHOLD_DEMAG}) /*!< Speed threshold above which the  RUN_DEMAGN_DELAY_MIN is applied */
#define DEMAG_RUN_STEP_RATIO               ((uint16_t)${MC.RUN_DEMAG_TIME}) /*!< Percentage of step time allowed for  demagnetization */
#define DEMAG_REVUP_STEP_RATIO             ((uint16_t)${MC.STARTUP_DEMAG_TIME}) /*!< Percentage of step time allowed for demagnetization */

  <#if CondFamily_STM32G4> 
#define ADC_AWD_FILTER_NUMBER              (uint8_t)${MC.M1_ADC_AWD_FILTER_NUMBER} /*!< Analog watchdog filtering configuration */
  </#if><#-- CondFamily_STM32G4-->

#define COMPUTATION_DELAY                  ${MC.M1_COMPUTATION_DELAY} /*!< Selection of the period for computation of the delay between the zero crossing and the step change */

  <#if MC.BEMF_DIVIDER_AVAILABLE == true>
#define BEMF_DIVIDER_DIODE_V               ${MC.BEMF_DIVIDER_DIODE_V} /*!< Voltage drop of the GPIO bemf divider diode */
  <#else>
#define BEMF_DIVIDER_DIODE_V               0 /*!< Voltage drop of the GPIO bemf divider diode */
  </#if>
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true> 
#define M1_COMMUTATION_STEP_TIME_BUF_SIZE  (uint8_t)${MC.M1_COMMUTATION_STEP_TIME_BUF_SIZE} /*!< size of the buffer to store commutation step time */
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true-->

<#if MC.M2_SPEED_SENSOR == "SENSORLESS_ADC">
/****** Bemf Observer ****/
#define BEMF_AVERAGING_FIFO_DEPTH          ${MC.BEMF_AVERAGING_FIFO_DEPTH} /*!< depth of the FIFO used to average  mechanical speed in 0.1Hz resolution */  
#define VARIANCE_THRESHOLD                 ${MC.M2_VARIANCE_THRESHOLD} /*!< Maximum accepted variance on speed estimates (percentage) */
#define BEMF_THRESHOLD_PWM_PERC            ${MC.M2_BEMF_THRESHOLD_PWM} /*!< Percentage of Bus for zero crossing detection with on-sensing */
#define BEMF_THRESHOLD_HIGH_PERC           ${MC.M2_BEMF_THRESHOLD_HIGH} /*!< Percentage of Bus for zero crossing detection with on-sensing and low side modulation */
#define BEMF_THRESHOLD_LOW_PERC  	       ${MC.M2_BEMF_THRESHOLD_LOW}	/*!< Percentage of Bus for zero crossing detection with on-sensing and high side modulation */
#define BEMF_ADC_TRIG_TIME_DPP             ((uint16_t)${MC.BEMF_ADC_TRIG_TIME_DPP}) /*!< 1/1024 of PWM period elapsed */
  <#if  MC.DRIVE_MODE == "VM">                                                             
#define BEMF_ADC_TRIG_TIME_ON_DPP          ((uint16_t)${MC.BEMF_ADC_TRIG_TIME_ON_DPP}) /*!< 1/1024 of PWM period  elapsed */
#define BEMF_PWM_ON_ENABLE_THRES_DPP       ((uint16_t)${MC.BEMF_PWM_ON_ENABLE_THRES_DPP}) /*!< 1/1024 of PWM period   elapsed */
#define BEMF_PWM_ON_ENABLE_HYSTERESIS_DPP  ((uint16_t)${MC.BEMF_PWM_ON_ENABLE_HYSTERESIS_DPP}) /*!< 1/1024 of PWM period    elapsed */
  </#if>
#define MIN_DEMAG_TIME                     ((uint16_t)${MC.MIN_DEMAG_TIME}) /*!< Demagnetization delay in number of HF timer periods elapsed before a first 
                                                           BEMF ADC measurement is processed in an attempt to detect the BEMF zero crossing */
#define SPEED_THRESHOLD_DEMAG              ((uint32_t)${MC.SPEED_THRESHOLD_DEMAG}) /*!< Speed threshold above which the RUN_DEMAGN_DELAY_MIN is applied */
#define DEMAG_RUN_STEP_RATIO               ((uint16_t)${MC.RUN_DEMAG_TIME}) /*!< Percentage of step time allowed for demagnetization */
#define DEMAG_REVUP_STEP_RATIO             ((uint16_t) ${MC.STARTUP_DEMAG_TIME}) /*!< Percentage of step time allowed for demagnetization */
</#if><#-- MC.M2_SPEED_SENSOR == "SENSORLESS_ADC" -->

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                      ${MC.M1_PWM_FREQUENCY}
#define PWM_FREQ_SCALING                   ${Fx_Freq_Scaling((MC.M1_PWM_FREQUENCY)?number)}
#define PWM_FREQUENCY_REF                  ${MC.REF_TIMER_FREQUENCY}
#define LOW_SIDE_SIGNALS_ENABLING          ${MC.M1_LOW_SIDE_SIGNALS_ENABLING}
<#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == 'LS_PWM_TIMER'>
#define SW_DEADTIME_NS                     ${MC.M1_SW_DEADTIME_NS} /*!< Dead-time to be inserted by FW, only if low side signals are enabled */
</#if>

/* High frequency task regulation loop */
#define REGULATION_EXECUTION_RATE          ${MC.M1_REGULATION_EXECUTION_RATE} /*!< Execution rate in number of PWM  cycles */
<#if MC.M1_POSITION_CTRL_ENABLING == true >
#define POSITION_LOOP_FREQUENCY_HZ         (uint16_t)${MC.M1_POSITION_LOOP_FREQUENCY_HZ} /*!<Execution rate of position control regulation loop (Hz) */
<#else>

/* Speed control loop */ 
#define SPEED_LOOP_FREQUENCY_HZ            (uint16_t)${MC.M1_SPEED_LOOP_FREQUENCY_HZ} /*!<Execution rate of speed regulation loop (Hz) */
</#if>
<#if MC.CURRENT_LIMITER_OFFSET>                                        
#define PID_SPEED_KP_DEFAULT          -${MC.M1_PID_SPEED_KP_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT          -${MC.M1_PID_SPEED_KI_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          ${MC.M1_PID_SPEED_KD_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
<#else>
#define PID_SPEED_KP_DEFAULT          ${MC.M1_PID_SPEED_KP_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT          ${MC.M1_PID_SPEED_KI_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          ${MC.M1_PID_SPEED_KD_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
</#if>

/* Speed PID parameter dividers */
#define SP_KPDIV                           ${MC.M1_SP_KPDIV}
#define SP_KIDIV                           ${MC.M1_SP_KIDIV}
#define SP_KDDIV                           ${MC.M1_SP_KDDIV}
#define SP_KPDIV_LOG                       LOG2((${MC.M1_SP_KPDIV}))
#define SP_KIDIV_LOG                       LOG2((${MC.M1_SP_KIDIV}))
#define SP_KDDIV_LOG                       LOG2((${MC.M1_SP_KDDIV}))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING     DISABLE
<#if MC.DRIVE_MODE == "VM">
#define PERIODMAX                          (uint16_t)((PWM_PERIOD_CYCLES * ${MC.PERIODMAX} / 100) + 1)
<#else>
#define PERIODMAX_REF                      (uint16_t)((PWM_PERIOD_CYCLES_REF * ${MC.PERIODMAX} / 100) + 1)
</#if>
#define LF_TIMER_ARR                       ${MC.LF_TIMER_ARR}
#define LF_TIMER_PSC                       ${MC.LF_TIMER_PSC}

/* Default settings */
#define DEFAULT_CONTROL_MODE               MCM_SPEED_MODE
#define DEFAULT_DRIVE_MODE                 ${MC.DRIVE_MODE} /*!< VOLTAGE_MODE (VM) or CURRENT_MODE (CM) */
#define DEFAULT_TARGET_SPEED_RPM           ${MC.M1_DEFAULT_TARGET_SPEED_RPM}
#define DEFAULT_TARGET_SPEED_UNIT          (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
<#if  MC.M1_POSITION_CTRL_ENABLING == true >
#define PID_POSITION_KP_GAIN               ${MC.M1_POSITION_CTRL_KP_GAIN}
#define PID_POSITION_KI_GAIN               ${MC.M1_POSITION_CTRL_KI_GAIN}
#define PID_POSITION_KD_GAIN               ${MC.M1_POSITION_CTRL_KD_GAIN}
#define PID_POSITION_KPDIV                 ${MC.M1_POSITION_CTRL_KPDIV}
#define PID_POSITION_KIDIV                 ${MC.M1_POSITION_CTRL_KIDIV}
#define PID_POSITION_KDDIV                 ${MC.M1_POSITION_CTRL_KDDIV}
#define PID_POSITION_KPDIV_LOG             LOG2((${MC.M1_POSITION_CTRL_KPDIV}))
#define PID_POSITION_KIDIV_LOG             LOG2((${MC.M1_POSITION_CTRL_KIDIV}))
#define PID_POSITION_KDDIV_LOG             LOG2((${MC.M1_POSITION_CTRL_KDDIV}))
#define PID_POSITION_ANGLE_STEP            ${MC.M1_POSITION_CTRL_ANGLE_STEP}
#define PID_POSITION_MOV_DURATION          ${MC.M1_POSITION_CTRL_MOV_DURATION}
</#if>

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
<#if   MC.M1_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE">  
#define OV_VOLTAGE_THRESHOLD_V             ${MC.M1_OV_VOLTAGE_THRESHOLD_V} /*!< Over-voltage threshold */
<#else>
#define M1_OVP_THRESHOLD_HIGH              ${MC.M1_OVP_THRESHOLD_HIGH} /*!< Over-voltage hysteresis high threshold */
#define M1_OVP_THRESHOLD_LOW               ${MC.M1_OVP_THRESHOLD_LOW} /*!< Over-voltage hysteresis low threshold */
</#if>
#define UD_VOLTAGE_THRESHOLD_V             ${MC.M1_UD_VOLTAGE_THRESHOLD_V} /*!< Under-voltage threshold */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE                    ${MC.M1_ON_OVER_VOLTAGE} /*!< TURN_OFF_PWM, TURN_ON_R_BRAKE or TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C         ${MC.M1_OV_TEMPERATURE_THRESHOLD_C} /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C        ${MC.M1_OV_TEMPERATURE_HYSTERESIS_C} /*!< Celsius degrees */
#define HW_OV_CURRENT_PROT_BYPASS          DISABLE /*!< In case ON_OVER_VOLTAGE is set to TURN_ON_LOW_SIDES this feature
                                                        may be used to bypass HW over-current protection (if supported 
                                                        by power stage) */
#define OVP_INVERTINGINPUT_MODE            ${MC.M1_OVP_INVERTINGINPUT_MODE}
#define OVP_INVERTINGINPUT_MODE2           ${MC.M2_OVP_INVERTINGINPUT_MODE}
#define OVP_SELECTION                      ${MC.M1_OVP_SELECTION}
#define OVP_SELECTION2                     ${MC.M2_OVP_SELECTION}

/******************************   START-UP PARAMETERS   **********************/
<#if M1_ENCODER >
/* Encoder alignment */
#define M1_ALIGNMENT_DURATION              ${MC.M1_ALIGNMENT_DURATION} /*!< milliseconds */
#define M1_ALIGNMENT_ANGLE_DEG             ${MC.M1_ALIGNMENT_ANGLE_DEG} /*!< degrees [0...359] */
#define FINAL_I_ALIGNMENT_A                ${MC.M1_FINAL_I_ALIGNMENT} /*!< s16A */
/* With ALIGNMENT_ANGLE_DEG equal to 90 degrees final alignment */
/* phase current = (FINAL_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt) */
/* being Av the voltage gain between Rshunt and A/D input */
</#if><#-- M1_ENCODER -->

<#if (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC")>
/* Phase 1 */
#define PHASE1_DURATION                    ${MC.M1_PHASE1_DURATION} /* milliseconds */
#define PHASE1_FINAL_SPEED_UNIT            (${MC.M1_PHASE1_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM) 
  <#if MC.DRIVE_MODE == "VM">
#define PHASE1_VOLTAGE_RMS                 ${MC.M1_PHASE1_VOLTAGE_RMS}
   <#else>
#define PHASE1_FINAL_CURRENT_A             ${MC.M1_PHASE1_FINAL_CURRENT}
  </#if>
  
/* Phase 2 */
#define PHASE2_DURATION                    ${MC.M1_PHASE2_DURATION} /* milliseconds */
#define PHASE2_FINAL_SPEED_UNIT            (${MC.M1_PHASE2_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM)
  <#if MC.DRIVE_MODE == "VM">
#define PHASE2_VOLTAGE_RMS                 ${MC.M1_PHASE2_VOLTAGE_RMS}
   <#else>
#define PHASE2_FINAL_CURRENT_A             ${MC.M1_PHASE2_FINAL_CURRENT}
  </#if>
  
/* Phase 3 */
#define PHASE3_DURATION                    ${MC.M1_PHASE3_DURATION} /* milliseconds */
#define PHASE3_FINAL_SPEED_UNIT            (${MC.M1_PHASE3_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM)
  <#if MC.DRIVE_MODE == "VM">
#define PHASE3_VOLTAGE_RMS                 ${MC.M1_PHASE3_VOLTAGE_RMS}
   <#else>
#define PHASE3_FINAL_CURRENT_A             ${MC.M1_PHASE3_FINAL_CURRENT}
  </#if>
  
/* Phase 4 */
#define PHASE4_DURATION                    ${MC.M1_PHASE4_DURATION} /* milliseconds */
#define PHASE4_FINAL_SPEED_UNIT            (${MC.M1_PHASE4_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM)
  <#if MC.DRIVE_MODE == "VM">
#define PHASE4_VOLTAGE_RMS                 ${MC.M1_PHASE4_VOLTAGE_RMS}
   <#else>
#define PHASE4_FINAL_CURRENT_A             ${MC.M1_PHASE4_FINAL_CURRENT}
  </#if>

/* Phase 5 */
#define PHASE5_DURATION                    ${MC.M1_PHASE5_DURATION} /* milliseconds */
#define PHASE5_FINAL_SPEED_UNIT            (${MC.M1_PHASE5_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM)
  <#if  MC.DRIVE_MODE == "VM">
#define PHASE5_VOLTAGE_RMS                 ${MC.M1_PHASE5_VOLTAGE_RMS}
   <#else>
#define PHASE5_FINAL_CURRENT_A             ${MC.M1_PHASE5_FINAL_CURRENT}
   </#if>
#define ENABLE_SL_ALGO_FROM_PHASE          ${MC.M1_ENABLE_SL_ALGO_FROM_PHASE}

/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG                 ${MC.M1_STARTING_ANGLE_DEG}  /*!< degrees [0...359] */   
</#if>

<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM              ${MC.M1_OBS_MINIMUM_SPEED_RPM}

#define NB_CONSECUTIVE_TESTS               ${MC.M1_NB_CONSECUTIVE_TESTS} /* corresponding to former NB_CONSECUTIVE_TESTS / (TF_REGULATION_RATE /  MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT             ${MC.M1_SPEED_BAND_UPPER_LIMIT} /*!< It expresses how much estimated speed can exceed forced stator electrical 
                                                without being considered wrong. In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT             ${MC.M1_SPEED_BAND_LOWER_LIMIT}  /*!< It expresses how much estimated speed can be below forced stator electrical 
                                                without being considered wrong. In 1/16 of forced speed */ 
</#if>
#define TRANSITION_DURATION                ${MC.M1_TRANSITION_DURATION} /* Switch over duration, ms */ 

<#if   MC.M1_BUS_VOLTAGE_READING >
/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME             LL_ADC_SAMPLING_CYCLE(${MC.M1_VBUS_ADC_SAMPLING_TIME})
</#if>

<#if MC.M1_TEMPERATURE_READING >
/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME             LL_ADC_SAMPLING_CYCLE(${MC.M1_TEMP_ADC_SAMPLING_TIME})
</#if>

/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES                (${MC.M1_CURR_SAMPLING_TIME} + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/
<#if MC.M1_ICL_ENABLED>
/* Inrush current limiter parameters */
#define M1_ICL_RELAY_SWITCHING_DELAY_MS    ${MC.M1_ICL_RELAY_SWITCHING_DELAY_MS} /* milliseconds */
#define M1_ICL_CAPS_CHARGING_DELAY_MS      ${MC.M1_ICL_CAPS_CHARGING_DELAY_MS} /* milliseconds */  
#define M1_ICL_VOLTAGE_THRESHOLD           ${MC.M1_ICL_VOLTAGE_THRESHOLD} /* volts */
</#if>

<#if MC.M1_POTENTIOMETER_ENABLE == true>
/* **** Potentiometer parameters **** */
/** @brief Sampling time set to the ADC channel used by the potentiometer component */
#define POTENTIOMETER_ADC_SAMPLING_TIME_M1 LL_ADC_SAMPLING_CYCLE(${MC.POTENTIOMETER_ADC_SAMPLING_TIME})
<#--
This value is stated in "u16digit" to be compatible with the values acquired by the ADC of the poentiometer.
The values read from the ADC range from 0 to 65520 -->

/**
 * @brief Speed reference set to Motor 1 when the potentiometer is at its maximum
 *
 * This value is expressed in #SPEED_UNIT. 
 *
 * Default value is #MAX_APPLICATION_SPEED_UNIT. 
 *
 * @sa POTENTIOMETER_MIN_SPEED_M1
 */
#define POTENTIOMETER_MAX_SPEED_M1         MAX_APPLICATION_SPEED_UNIT

/**
 * @brief Speed reference set to Motor 1 when the potentiometer is at its minimum
 *
 * This value is expressed in #SPEED_UNIT. 
 *
 * Default value is 10 % of #MAX_APPLICATION_SPEED_UNIT.
 *
 * @sa POTENTIOMETER_MAX_SPEED_M1
 */
#define POTENTIOMETER_MIN_SPEED_M1         ((MAX_APPLICATION_SPEED_UNIT)/10)

<#--
/**
 * @brief Conversion factor from #SPEED_UNIT to Potentiometer ADC scale for Motor 1
 *
 * This Factor is used to convert values read from an ADC by the potentiometer 
 * component into values expressed in #SPEED_UNIT according to the following
 * formula: `V_SPEED_UNIT = V_ADC / POTENTIOMETER_SPEED_CONV_FACTOR_M1`
 */
#define POTENTIOMETER_SPEED_CONV_FACTOR_M1 ((65520.0)/((POTENTIOMETER_MAX_SPEED_M1)-(POTENTIOMETER_MIN_SPEED_M1)))
-->
/**
 * @brief Potentiometer change threshold to trigger speed reference update for Motor 1
 *
 * When the potentiometer value differs from the current speed reference by more than this 
 * threshold, the speed reference set to the motor is adjusted to match the potentiometer value. 
 *
 * The threshold is expressed in u16digits. Its default value is set to 13% of the potentiometer 
 * aquisition range
 *
 */
 #define POTENTIOMETER_SPEED_ADJUSTMENT_RANGE_M1 (655)

/**
 * @brief Acceleration used to compute ramp duration when setting speed reference to Motor 1
 *
 * This acceleration is expressed in #SPEED_UNIT/s. Its default value is 100 Hz/s (provided 
 * that #SPEED_UNIT is #U_01HZ).
 *
 */
 #define POTENTIOMETER_RAMP_SLOPE_M1       1000

/**
 * @brief Bandwith of the low pass filter applied on the potentiometer values
 *
 * @see SpeedPotentiometer_Handle_t::LPFilterBandwidthPOW2
 */
#define POTENTIOMETER_LPF_BANDWIDTH_POW2_M1 4
</#if>

/*** On the fly start-up ***/
<#-- ToDo: On the Fly start-up -->

<#if MC.DRIVE_NUMBER != "1">
/**************************
 *** Motor 2 Parameters ***
 **************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM2         ${MC.M2_MAX_APPLICATION_SPEED} /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM2         ${MC.M2_MIN_APPLICATION_SPEED} /*!< rpm, mechanical, absolute value */
#define M2_SS_MEAS_ERRORS_BEFORE_FAULTS    ${MC.M2_SS_MEAS_ERRORS_BEFORE_FAULTS} /*!< Number of speed measurement errors before main sensor goes in fault */

<#if M2_ENCODER >
/*** Encoder **********************/
#define ENC_AVERAGING_FIFO_DEPTH2          ${MC.M2_ENC_AVERAGING_FIFO_DEPTH} /*!< depth of the FIFO used to average mechanical speed in 0.1Hz resolution */
</#if><#-- M2_ENCODER -->
<#if (MC.M2_SPEED_SENSOR == "HALL_SENSOR") || (MC.M2_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR") >

/****** Hall sensors ************/ 
#define HALL_AVERAGING_FIFO_DEPTH2         ${MC.M2_HALL_AVERAGING_FIFO_DEPTH} /*!< depth of the FIFO used to average mechanical speed in 0.1Hz resolution */ 
#define HALL_MTPA2                         <#if MC.M2_HALL_MTPA > true <#else> false </#if>
</#if><#-- (MC.M2_SPEED_SENSOR == "HALL_SENSOR") || (MC.M2_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR") -->
#define REV_PARK_ANGLE_COMPENSATION_FACTOR2 0

/* USER CODE END angle reconstruction M2 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* Dual drive specific parameters */
#define FREQ_RATIO                          ${FREQ_RATIO} /* Higher PWM frequency/lower PWM frequency */
#define FREQ_RELATION                       ${M1_FREQ_RELATION} /* It refers to motor 1 and can be HIGHEST_FREQ or LOWEST frequency depending on motor 1 and 2 
                                                 frequency relationship */
#define FREQ_RELATION2                      ${M2_FREQ_RELATION} /* It refers to motor 2 and can be HIGHEST_FREQ or LOWEST frequency depending on motor 1 and 2
                                                 frequency relationship */

/* PWM generation and current reading */
#define PWM_FREQUENCY2                      ${MC.M2_PWM_FREQUENCY}
#define PWM_FREQ_SCALING2                   ${Fx_Freq_Scaling((MC.M2_PWM_FREQUENCY)?number)} 
#define LOW_SIDE_SIGNALS_ENABLING2          ${MC.M2_LOW_SIDE_SIGNALS_ENABLING}
<#if MC.M2_LOW_SIDE_SIGNALS_ENABLING == 'LS_PWM_TIMER'>
#define SW_DEADTIME_NS2                     ${MC.M2_SW_DEADTIME_NS} /*!< Dead-time to be inserted by FW, only if low side signals are enabled */
</#if>

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE2          ${MC.M2_REGULATION_EXECUTION_RATE} /*!< FOC execution rate in number of PWM cycles */

/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT2              ${MC.M2_PID_TORQUE_KP_DEFAULT}
#define PID_TORQUE_KI_DEFAULT2              ${MC.M2_PID_TORQUE_KI_DEFAULT}
#define PID_TORQUE_KD_DEFAULT2              ${MC.M2_PID_TORQUE_KD_DEFAULT}
#define PID_FLUX_KP_DEFAULT2                ${MC.M2_PID_FLUX_KP_DEFAULT}
#define PID_FLUX_KI_DEFAULT2                ${MC.M2_PID_FLUX_KI_DEFAULT}
#define PID_FLUX_KD_DEFAULT2                ${MC.M2_PID_FLUX_KD_DEFAULT}

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV2                           ${MC.M2_TF_KPDIV}
#define TF_KIDIV2                           ${MC.M2_TF_KIDIV}
#define TF_KDDIV2                           ${MC.M2_TF_KDDIV}
#define TF_KPDIV_LOG2                       LOG2((${MC.M2_TF_KPDIV}))
#define TF_KIDIV_LOG2                       LOG2((${MC.M2_TF_KIDIV}))
#define TF_KDDIV_LOG2                       LOG2((${MC.M2_TF_KDDIV}))

#define TFDIFFERENTIAL_TERM_ENABLING2       DISABLE
<#if MC.M2_POSITION_CTRL_ENABLING == true >
#define POSITION_LOOP_FREQUENCY_HZ2         ${MC.M2_POSITION_LOOP_FREQUENCY_HZ} /*!< Execution rate of position control regulation loop (Hz) */
<#else>
/* Speed control loop */ 
#define SPEED_LOOP_FREQUENCY_HZ2            ${MC.M2_SPEED_LOOP_FREQUENCY_HZ} /*!< Execution rate of speed  regulation loop (Hz) */
</#if>
#define PID_SPEED_KP_DEFAULT2               ${MC.M2_PID_SPEED_KP_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit */
#define PID_SPEED_KI_DEFAULT2               ${MC.M2_PID_SPEED_KI_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit */
#define PID_SPEED_KD_DEFAULT2               ${MC.M2_PID_SPEED_KD_DEFAULT}/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit */

/* Speed PID parameter dividers */
#define SP_KPDIV2                           ${MC.M2_SP_KPDIV}
#define SP_KIDIV2                           ${MC.M2_SP_KIDIV}
#define SP_KDDIV2                           ${MC.M2_SP_KDDIV}
#define SP_KPDIV_LOG2                       LOG2((${MC.M2_SP_KPDIV}))
#define SP_KIDIV_LOG2                       LOG2((${MC.M2_SP_KIDIV}))
#define SP_KDDIV_LOG2                       LOG2((${MC.M2_SP_KDDIV}))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV2 */
#define PID_SPEED_INTEGRAL_INIT_DIV2 1 
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV2 */

#define SPD_DIFFERENTIAL_TERM_ENABLING2 DISABLE
#define IQMAX2_A                            ${MC.M2_IQMAX}

/* Default settings */
<#if MC.M2_DEFAULT_CONTROL_MODE == 'STC_SPEED_MODE'>
#define DEFAULT_CONTROL_MODE2               MCM_SPEED_MODE
<#else>
#define DEFAULT_CONTROL_MODE2               MCM_TORQUE_MODE
</#if>
#define DEFAULT_TARGET_SPEED_RPM2           ${MC.M2_DEFAULT_TARGET_SPEED_RPM}
#define DEFAULT_TARGET_SPEED_UNIT2          (DEFAULT_TARGET_SPEED_RPM2*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT2_A         ${MC.M2_DEFAULT_TORQUE_COMPONENT}
#define DEFAULT_FLUX_COMPONENT2_A           ${MC.M2_DEFAULT_FLUX_COMPONENT}

<#if  MC.M2_POSITION_CTRL_ENABLING == true >
#define PID_POSITION_KP_GAIN2               ${MC.M2_POSITION_CTRL_KP_GAIN}
#define PID_POSITION_KI_GAIN2               ${MC.M2_POSITION_CTRL_KI_GAIN}
#define PID_POSITION_KD_GAIN2               ${MC.M2_POSITION_CTRL_KD_GAIN}
#define PID_POSITION_KPDIV2                 ${MC.M2_POSITION_CTRL_KPDIV}
#define PID_POSITION_KIDIV2                 ${MC.M2_POSITION_CTRL_KIDIV}
#define PID_POSITION_KDDIV2                 ${MC.M2_POSITION_CTRL_KDDIV}
#define PID_POSITION_KPDIV_LOG2             LOG2((${MC.M2_POSITION_CTRL_KPDIV}))
#define PID_POSITION_KIDIV_LOG2             LOG2((${MC.M2_POSITION_CTRL_KIDIV})) 
#define PID_POSITION_KDDIV_LOG2             LOG2((${MC.M2_POSITION_CTRL_KDDIV})) 
#define PID_POSITION_ANGLE_STEP2            ${MC.M2_POSITION_CTRL_ANGLE_STEP}
#define PID_POSITION_MOV_DURATION2          ${MC.M2_POSITION_CTRL_MOV_DURATION}
</#if>

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
<#if   MC.M2_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE">  
#define OV_VOLTAGE_THRESHOLD_V2             ${MC.M2_OV_VOLTAGE_THRESHOLD_V} /*!< Over-voltage threshold */
<#else>
#define M2_OVP_THRESHOLD_HIGH               ${MC.M2_OVP_THRESHOLD_HIGH} /*!< Over-voltage hysteresis high threshold */
#define M2_OVP_THRESHOLD_LOW                ${MC.M2_OVP_THRESHOLD_LOW} /*!< Over-voltage hysteresis low threshold */
</#if>
#define UD_VOLTAGE_THRESHOLD_V2             ${MC.M2_UD_VOLTAGE_THRESHOLD_V} /*!< Under-voltage threshold */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE2                    ${MC.M2_ON_OVER_VOLTAGE} /*!< TURN_OFF_PWM, TURN_ON_R_BRAKE or TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C2         ${MC.M2_OV_TEMPERATURE_THRESHOLD_C} /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C2        ${MC.M2_OV_TEMPERATURE_HYSTERESIS_C} /*!< Celsius degrees */
#define HW_OV_CURRENT_PROT_BYPASS2          DISABLE /*!< In case ON_OVER_VOLTAGE is set to TURN_ON_LOW_SIDES this 
                                                         feature may be used to bypass HW over-current protection
                                                         (if supported by power stage) */

/******************************   START-UP PARAMETERS   **********************/
/* Encoder alignment */
#define M2_ALIGNMENT_DURATION               ${MC.M2_ALIGNMENT_DURATION} /*!< milliseconds */
#define M2_ALIGNMENT_ANGLE_DEG              ${MC.M2_ALIGNMENT_ANGLE_DEG} /*!< degrees [0...359] */
#define FINAL_I_ALIGNMENT2_A                ${MC.M2_FINAL_I_ALIGNMENT} /*!< s16A */
/* With ALIGNMENT_ANGLE_DEG equal to 90 degrees final alignment */
/* phase current = (FINAL_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt) */
/* being Av the voltage gain between Rshunt and A/D input */
<#if MC.M2_SPEED_SENSOR == "SENSORLESS_ADC">
  <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
  
/* USER CODE BEGIN OPENLOOP M2 */
#define OPEN_LOOP_VOLTAGE_d2                0 /*!< Three Phase voltage amplitude in int16_t format */
#define OPEN_LOOP_SPEED_RPM2                100 /*!< Final forced speed in rpm */
#define OPEN_LOOP_SPEED_RAMP_DURATION_MS2   1000 /*!< 0-to-Final speed ramp duration  */      
#define OPEN_LOOP_VF2                       false /*!< true to enable V/F mode */
#define OPEN_LOOP_K2                        44 /*! Slope of V/F curve expressed in int16_t Voltage for each 0.1Hz of 
                                                   mecchanical frequency increment. */
#define OPEN_LOOP_OFF2                      4400 /*! Offset of V/F curve expressed in int16_t Voltage applied when 
                                                     frequency is zero. */
/* USER CODE END OPENLOOP M2 */

  </#if><#-- MC.M2_DBG_OPEN_LOOP_ENABLE == false inside (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") -->
/* Phase 1 */
#define PHASE1_DURATION2                    ${MC.M2_PHASE1_DURATION} /*milliseconds */
#define PHASE1_FINAL_SPEED_UNIT2            (${MC.M2_PHASE1_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM) /* rpm */
#define PHASE1_FINAL_CURRENT2_A             ${MC.M2_PHASE1_FINAL_CURRENT}
/* Phase 2 */
#define PHASE2_DURATION2                    ${MC.M2_PHASE2_DURATION} /*milliseconds */
#define PHASE2_FINAL_SPEED_UNIT2            (${MC.M2_PHASE2_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM) /* rpm */
#define PHASE2_FINAL_CURRENT2_A             ${MC.M2_PHASE2_FINAL_CURRENT}
/* Phase 3 */
#define PHASE3_DURATION2                    ${MC.M2_PHASE3_DURATION} /*milliseconds */
#define PHASE3_FINAL_SPEED_UNIT2            (${MC.M2_PHASE3_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM) /* rpm */
#define PHASE3_FINAL_CURRENT2_A             ${MC.M2_PHASE3_FINAL_CURRENT}
/* Phase 4 */
#define PHASE4_DURATION2                    ${MC.M2_PHASE4_DURATION} /*milliseconds */
#define PHASE4_FINAL_SPEED_UNIT2            (${MC.M2_PHASE4_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM) /* rpm */
#define PHASE4_FINAL_CURRENT2_A             ${MC.M2_PHASE4_FINAL_CURRENT}
/* Phase 5 */
#define PHASE5_DURATION2                    ${MC.M2_PHASE5_DURATION} /* milliseconds */
#define PHASE5_FINAL_SPEED_UNIT2            (${MC.M2_PHASE5_FINAL_SPEED_RPM}*SPEED_UNIT/U_RPM) /* rpm */
#define PHASE5_FINAL_CURRENT2_A             ${MC.M2_PHASE5_FINAL_CURRENT}

#define ENABLE_SL_ALGO_FROM_PHASE2          ${MC.M2_ENABLE_SL_ALGO_FROM_PHASE}

/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG2                 ${MC.M2_STARTING_ANGLE_DEG}  /*!< degrees [0...359] */
</#if>

<#if MC.M2_SPEED_SENSOR == "SENSORLESS_ADC">
/* Observer start-up output conditions */
#define OBS_MINIMUM_SPEED_RPM2              ${MC.M2_OBS_MINIMUM_SPEED_RPM}
#define NB_CONSECUTIVE_TESTS2               ${MC.M2_NB_CONSECUTIVE_TESTS} /* corresponding toformer NB_CONSECUTIVE_TESTS / (TF_REGULATION_RATE / MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT2             ${MC.M2_SPEED_BAND_UPPER_LIMIT} /*!< It expresses how much estimated speed can exceed forced stator electrical 
                                                 without being considered wrong. In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT2             ${MC.M2_SPEED_BAND_LOWER_LIMIT}  /*!< It expresses how much estimated speed can be below forced stator electrical 
                                                 without being considered wrong. In 1/16 of forced speed */  
</#if>
#define TRANSITION_DURATION2                ${MC.M2_TRANSITION_DURATION} /* Switch over duration, ms */

<#if   MC.M2_BUS_VOLTAGE_READING >
/******************************   BUS VOLTAGE  Motor 2  **********************/
#define  M2_VBUS_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(${MC.M2_VBUS_ADC_SAMPLING_TIME})
</#if>

<#if  MC.M2_TEMPERATURE_READING >
/******************************   Temperature sensing Motor 2  **********************/
#define  M2_TEMP_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(${MC.M2_TEMP_ADC_SAMPLING_TIME})
</#if>

/******************************   Current sensing Motor 2   **********************/
#define ADC_SAMPLING_CYCLES2                (${MC.M2_CURR_SAMPLING_TIME} + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/
<#if MC.M2_ICL_ENABLED>
/* Inrush current limiter parameters */
#define M2_ICL_RELAY_SWITCHING_DELAY_MS     ${MC.M2_ICL_RELAY_SWITCHING_DELAY_MS} /* milliseconds */
#define M2_ICL_CAPS_CHARGING_DELAY_MS       ${MC.M2_ICL_CAPS_CHARGING_DELAY_MS} /* milliseconds */
#define M2_ICL_VOLTAGE_THRESHOLD            ${MC.M2_ICL_VOLTAGE_THRESHOLD} /* volts */
</#if>

<#if MC.M2_POTENTIOMETER_ENABLE == true>
/* **** Potentiometer parameters **** */
/** @brief Sampling time set to the ADC channel used by the potentiometer component */
#define POTENTIOMETER_ADC_SAMPLING_TIME_M2  LL_ADC_SAMPLING_CYCLE(${MC.POTENTIOMETER_ADC_SAMPLING_TIME2})

<#--
This value is stated in "u16digit" to be compatible with the values acquired 
by the ADC of the poentiometer. The values read from the ADC range from 0 to 
65520
-->
/**
 * @brief Speed reference set to Motor 2 when the potentiometer is at its maximum
 *
 * This value is expressed in #SPEED_UNIT. 
 *
 * Default value is #MAX_APPLICATION_SPEED_UNIT2. 
 *
 * @sa POTENTIOMETER_MIN_SPEED_M2
 */
#define POTENTIOMETER_MAX_SPEED_M2          MAX_APPLICATION_SPEED_UNIT2

/**
 * @brief Speed reference set to Motor 1 when the potentiometer is at its minimum
 *
 * This value is expressed in #SPEED_UNIT. 
 *
 * Default value is 10 % of #MAX_APPLICATION_SPEED_UNIT2.
 *
 * @sa POTENTIOMETER_MAX_SPEED_M2
 */
#define POTENTIOMETER_MIN_SPEED_M2          ((MAX_APPLICATION_SPEED_UNIT2)/10)

<#--
/**
 * @brief Conversion factor from #SPEED_UNIT to Potentiometer ADC scale for Motor 1
 *
 * This Factor is used to convert values read from an ADC by the potentiometer 
 * component into values expressed in #SPEED_UNIT according to the following
 * formula: `V_SPEED_UNIT2 = V_ADC / POTENTIOMETER_SPEED_CONV_FACTOR_M2`
 */
#define POTENTIOMETER_SPEED_CONV_FACTOR_M2  ((65520.0)/((POTENTIOMETER_MAX_SPEED_M2)-(POTENTIOMETER_MIN_SPEED_M2)))
-->
/**
 * @brief Potentiometer change threshold to trigger speed reference update for Motor 2
 *
 * When the potentiometer value differs from the current speed reference by more than this 
 * threshold, the speed reference set to the motor is adjusted to match the potentiometer value. 
 *
 * The threshold is expressed in u16digits. Its default value is set to 13% of the potentiometer 
 * aquisition range
 *
 */
 #define POTENTIOMETER_SPEED_ADJUSTMENT_RANGE_M2 (655)

/**
 * @brief Acceleration used to compute ramp duration when setting speed reference to Motor 2
 *
 * This acceleration is expressed in #SPEED_UNIT/s. Its default value is 100 Hz/s (provided 
 * that #SPEED_UNIT is #U_01HZ).
 *
 */
 #define POTENTIOMETER_RAMP_SLOPE_M2        1000

/**
 * @brief Bandwith of the low pass filter applied on the potentiometer values
 *
 * @see SpeedPotentiometer_Handle_t::LPFilterBandwidthPOW2
 */
#define POTENTIOMETER_LPF_BANDWIDTH_POW2_M2 4
</#if>

/*** On the fly start-up ***/
<#-- ToDo: On the Fly start-up -->
</#if>

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
