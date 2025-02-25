<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    pmsm_motor_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure the motor to drive.
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
#ifndef PMSM_MOTOR_PARAMETERS_H
#define PMSM_MOTOR_PARAMETERS_H

<#if MC.DRIVE_NUMBER != "1">
/**************************
 *** Motor 1 Parameters ***
 **************************/
<#else>
/************************
 *** Motor Parameters ***
 ************************/
</#if><#-- MC.DRIVE_NUMBER > 1 -->

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM           ${MC.M1_POLE_PAIR_NUM} /* Number of motor pole pairs */
#define RS                      ${MC.M1_RS} /* Stator resistance , ohm*/
#define LS                      ${MC.M1_LS} /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

<#if MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST">
#define MOTOR_NAME "MOTOR_Shinano"  /*!< @brief motor name */
#define MOTOR_RATED_FLUX        (float_t)(${MC.M1_MOTOR_RATED_FLUX}) /*!< @brief Motor rated flux in V/Hz */
#define PID_SPD_KP              (float_t)(${MC.M1_PID_SPD_KP})  /*!< @brief PI Speed regulator proportional gain */
#define PID_SPD_KI              (float_t)(${MC.M1_PID_SPD_KI})  /*!< @brief PI Speed regulator integral gain */
#define MOTOR_RS_SKINFACTOR     (float_t)(${MC.M1_MOTOR_RS_SKINFACTOR})  /*!< @brief factor to compensate motor skin effect. */
#define MOTOR_MASS_COPPER_KG    (float_t)(${MC.M1_MOTOR_MASS_COPPER_KG}) /*!< @brief This is the total copper mass of the windings. */
#define MOTOR_COOLINGTAU_S      (float_t)(${MC.M1_MOTOR_COOLINGTAU_S})  /*!< @brief Thermal time constant of copper */
<#if MC.M1_SPEED_SENSOR == "ZEST">
#define MOTOR_ZEST_THRESHOLD_FREQ_HZ (float_t)(${MC.M1_MOTOR_ZEST_THRESHOLD_FREQ_HZ}) 
#define MOTOR_ZEST_INJECT_FREQ       (float_t)(${MC.M1_MOTOR_ZEST_INJECT_FREQ})
#define MOTOR_ZEST_INJECT_D          (float_t)(${MC.M1_MOTOR_ZEST_INJECT_D})
#define MOTOR_ZEST_GAIN_D            (float_t)(${MC.M1_MOTOR_ZEST_GAIN_D})
#define MOTOR_ZEST_GAIN_Q            (float_t)(${MC.M1_MOTOR_ZEST_GAIN_Q})

#define MOTOR_PERCENTAGE_MAX_CURRENT   (0.1f)   /*!< @brief Threshold in percentage of maximum torque to change ZeST config. */
#define MOTOR_ZEST_INJECT_FREQ_ALT     (70.0f)  /*!< @brief Alternative ZeST injected Id signal frequency in inertia zone */
#define MOTOR_ZEST_GAIN_D_ALT 	       (-0.5f)  /*!< @brief Alternative ZeST D-mobility gain reacts to the mechanical damping of the motor.  */
#define MOTOR_ZEST_GAIN_Q_ALT	       (0.0f)   /*!< @brief Alternative ZeST Q-mobility gain reacts to the mechanical inertia of the motor.  */
</#if>
#define THROTTLE_SPEED_MAX_RPM  (4000.0) /*!< @brief Speed when throttle is at maximum */ 
#define MOTOR_CURRENT_KP_ALIGN  (RS/4)  
#define THROTTLE_OFFSET    0.2f
#define THROTTLE_GAIN      1.6f
<#else>
#define MOTOR_MAX_SPEED_RPM     ${MC.M1_MOTOR_MAX_SPEED_RPM} /*!< Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT  ${MC.M1_MOTOR_VOLTAGE_CONSTANT} /*!< Volts RMS ph-ph /kRPM */
</#if>
#define NOMINAL_CURRENT_A       ${MC.M1_NOMINAL_CURRENT}

<#if MC.M1_DRIVE_TYPE == "FOC" || MC.M2_DRIVE_TYPE == "FOC">
#define ID_DEMAG_A              ${MC.M1_ID_DEMAG} /*!< Demagnetization current */

</#if>
/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  ${MC.M1_HALL_SENSORS_PLACEMENT} /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        ${MC.M1_HALL_PHASE_SHIFT} /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
<#if MC.M1_DRIVE_TYPE == "FOC" || MC.M2_DRIVE_TYPE == "FOC">
/*** Quadrature encoder ***/
#define M1_ENCODER_PPR          ${MC.M1_ENCODER_PPR}  /*!< Number of pulses per
                                            revolution */

<#if MC.DRIVE_NUMBER != "1">
/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM2          ${MC.M2_POLE_PAIR_NUM} /* Number of motor pole pairs */
#define RS2                     ${MC.M2_RS} /* Stator resistance , ohm*/
#define LS2                     ${MC.M2_LS} /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define NOMINAL_CURRENT2_A       ${MC.M2_NOMINAL_CURRENT}
#define MOTOR_MAX_SPEED_RPM2     ${MC.M2_MOTOR_MAX_SPEED_RPM} /*!< Maximum rated speed  */
#define MOTOR_VOLTAGE_CONSTANT2  ${MC.M2_MOTOR_VOLTAGE_CONSTANT} /*!< Volts RMS ph-ph /kRPM */
#define ID_DEMAG2_A              ${MC.M2_ID_DEMAG} /*!< Demagnetization current */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT2  ${MC.M2_HALL_SENSORS_PLACEMENT} /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT2        ${MC.M2_HALL_PHASE_SHIFT} /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define M2_ENCODER_PPR           ${MC.M2_ENCODER_PPR}  /*!< Number of pulses per
                                            revolution */
</#if>
</#if><#-- MC.DRIVE_NUMBER > 1 -->
#endif /* PMSM_MOTOR_PARAMETERS_H */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
