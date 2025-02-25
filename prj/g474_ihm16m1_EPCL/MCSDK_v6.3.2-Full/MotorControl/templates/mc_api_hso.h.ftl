<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    mc_api.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file defines the high level interface of the Motor Control SDK.
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
  * @ingroup MCIAPIHSO
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_API_H
#define MC_API_H

#include "mc_interface.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

 /** @addtogroup MCSDK
   * @{
   */

 /** @addtogroup HSOAI
   * @{
   */

 /** @addtogroup MCIAPIHSO
   * @{
   */

/* Initiates the startup procedure for Motor 1. */
MC_RetStatus_t MC_StartMotor1(void);

/* Initiates the startup procedure for Motor 1 and the polarization offsets measurement procedure if needed. */
MC_RetStatus_t MC_StartWithPolarizationMotor1(void);

MC_RetStatus_t MC_StartOffsetMeasurmentsMotor1(void);

/* Initiates the stop procedure for Motor 1. */
MC_RetStatus_t MC_StopMotor1(void);

/* Sets the acceleration to apply on Motor 1 when its speed reference is changed
 * The format of the Acceleration parameter is fixed point, and the value in Hz/s. */
void MC_SetAccelerationMotor1( fixp16_t Acceleration );

/* Sets the acceleration to apply on Motor 1 when its speed reference is changed
 * The format of the Acceleration parameter is float, and the value in Hz/s. */
void MC_SetAccelerationMotor1_F( float Acceleration );

/* Sets the acceleration to apply on Motor 1 when its speed reference is changed
 * The format of the Acceleration parameter is fixed point, and the value in "Per Unit". */
void MC_SetAccelerationMotor1_PU( fixp30_t Acceleration );

/* Returns the Acceleration applied on Motor 1 when its speed reference is changed
 * The format of the returned acceleration is fixed point and the value in Hz/s. */
fixp16_t MC_GetAccelerationMotor1(void);

/* Returns the Acceleration applied on Motor 1 when its speed reference is changed
 * The format of the returned acceleration is float and the value in Hz/s. */
float MC_GetAccelerationMotor1_F(void);

/* Returns the Acceleration applied on Motor 1 when its speed reference is changed
 * The format of the returned acceleration is fixed point and the value in "Per Unit"/s. */
fixp30_t MC_GetAccelerationMotor1_PU(void);

/* Sets the rotation speed of Motor 1.
 * The format of the Speed parameter is fixed point, and the value in Hz.
 * It is an electrical speed. */
void MC_SetSpeedReferenceMotor1( fixp16_t Speed );

/* Sets the rotation speed of Motor 1.
 * The format of the Speed parameter is float, and the value in Hz.
 * It is an electrical speed. */
void MC_SetSpeedReferenceMotor1_F( float Speed );

/* Sets the rotation speed of Motor 1.
 * The format of the Speed parameter is fixed point, and the value in "Per Unit".
 * It is an electrical speed. */
void MC_SetSpeedReferenceMotor1_PU( fixp30_t Speed );

/* Returns the electrical rotor speed reference set for Motor 1
 * The format of the returned speed is fixed point, and the value is in Hz */
fixp16_t MC_GetSpeedReferenceMotor1(void);

/* Returns the electrical speed reference set for Motor 1
 * The format of the returned speed is float, and the value is in Hz. */
float MC_GetSpeedReferenceMotor1_F(void);

/* Returns the electrical speed reference set for Motor 1
 * The format of the returned speed is fixed point, and the value is in "Per Unit". */
fixp30_t MC_GetSpeedReferenceMotor1_PU(void);

/* Returns the average electrical speed for Motor 1
 * The format of the returned speed is fixed point, and the value is in Hz. */
fixp16_t MC_GetSpeedMotor1(void);

/* Returns the average electrical speed for Motor 1
 * The format of the returned speed is float, and the value is in Hz. */
float MC_GetSpeedMotor1_F(void);

/* Returns the average electrical rotor speed for Motor 1
 * The format of the returned speed is fixed point, and the value is in "Per Unit". */
fixp30_t MC_GetSpeedMotor1_PU(void);

/* Returns the magnitude of EMF Electro Motive Force for Motor 1
 * The format of the returned Emf is float, and the value is in "Volts". */
float MC_GetEmfMagnitudeMotor1_F(void);

/* Returns the magnitude of EMF Electro Motive Force for Motor 1
 * The format of the returned Emf is fixed point, and the value is in "Per Unit". */
fixp30_t MC_GetEmfMagnitudeMotor1_PU(void);

/* Sets the track angle flag for Motor 1
 * Allow to fix or track the angle provided by the High Sensitivity Observer. */
void MC_SetFlagFreezeTrackAngle(bool value);

/* Sets the rated flux of Motor 1.
 * The format of the rated flux parameter is fixed point, and the value in V/Hz. */
void MC_SetRatedFluxMotor1(fixp24_t flux_VpHz);

/* Sets the torque reference of Motor 1.
 * The format of the Torque parameter is fixed point, and the value in Nm. */
void MC_SetTorqueReferenceMotor1( fixp16_t Torque );

/* Sets the torque reference of Motor 1.
 * The format of the Torque parameter is float, and the value in Nm. */
void MC_SetTorqueReferenceMotor1_F( float Torque );

/* Sets the torque reference of Motor 1.
 * The format of the Torque parameter is fixed point, and the value in "Per Unit". */
void MC_SetTorqueReferenceMotor1_PU( fixp30_t Torque );

/* Returns the torque reference for Motor 1.
 * The format of the returned torque is fixed point, and the value is in Nm. */
fixp16_t MC_GetTorqueReferenceMotor1(void);

/* Returns the torque reference for Motor 1.
 * The format of the returned torque is float, and the value is in Nm. */
float MC_GetTorqueReferenceMotor1_F(void);

/* Returns the torque reference for Motor 1.
 * The format of the returned torque is fixed point, and the value is in "Per Unit". */
fixp30_t MC_GetTorqueReferenceMotor1_PU(void);

/* Returns the torque currently produced on the shaft of Motor 1.
 * The format of the returned torque is fixed point, and the value is in Nm. */
fixp16_t MC_GetTorqueMotor1( void );

/* Returns the torque currently produced on the shaft of Motor 1.
 * The format of the returned torque is float, and the value is in Nm. */
float MC_GetTorqueMotor1_F(void);

/* Returns the torque currently produced on the shaft of Motor 1.
 * The format of the returned torque is fixed point, and the value is in "Per Unit". */
fixp30_t MC_GetTorqueMotor1_PU(void);

/* Sets the current reference to Motor 1 for later or immediate execution.
 * The format of the Torque parameter is fixed point, and the value in A. */
void MC_SetCurrentReferenceMotor1( dq_fixp16_t Idqref );

/* Sets the current reference to Motor 1 for later or immediate execution.
 * The format of the Torque parameter is fixed point, and the value in A. */
void MC_SetCurrentReferenceMotor1_F( dq_float_t Idqref );

/* Sets the current reference to Motor 1 for later or immediate execution.
 * The format of the Torque parameter is fixed point, and the value in "Per Unit". */
void MC_SetCurrentReferenceMotor1_PU( dq_fixp30_t Idqref );

/* Returns the Current reference set for motor 1
 * The format of the returned current is fixed point, and the value is in Ampere. */
dq_fixp16_t MC_GetCurrentReferenceMotor1(void);

/* Returns the Current reference set for motor 1
 * The format of the returned current is float, and the value is in Ampere. */
dq_float_t MC_GetCurrentReferenceMotor1_F(void);

/* Returns the Current reference set for motor 1
 * The format of the returned current is fixed point, and the value is in "Per Unit". */
dq_fixp30_t MC_GetCurrentReferenceMotor1_PU(void);

/* Returns the Current measured for motor 1
 * The format of the returned current is fixed point, and the value is in Ampere. */
dq_fixp16_t MC_GetCurrentMotor1(void);

/* Returns the Current measured for motor 1
 * The format of the returned current is float, and the value is in Ampere. */
dq_float_t MC_GetCurrentMotor1_F(void);

/* Returns the Current measured for motor 1
 * The format of the returned current is fixed point, and the value is in "Per Unit". */
dq_fixp30_t MC_GetCurrentMotor1_PU(void);

/* Specifies the duty cycles to apply on the phase voltages in the DQ frame */
void MC_SetDutyCycleMotor1_PU( dq_fixp30_t DutyCycle );

/* Returns the duty cycles applied on the phase voltages in the DQ frame reference */
dq_fixp30_t MC_GetDutyCycleMotor1_PU(void);

/* TODO: API for position control */

/* Sets the control mode for Motor 1. */
bool MC_SetControlModeMotor1( MC_ControlMode_t mode );

/* Returns the Control Mode used for Motor 1. */
MC_ControlMode_t MC_GetControlModeMotor1(void);

/* Sets the polarization offsets to use for Motor 1 */
MC_RetStatus_t MC_SetPolarizationOffsetsMotor1( PolarizationOffsets_t * PolarizationOffsets );

/* Returns the polarization offsets measured or set for Motor 1 */
MC_RetStatus_t MC_GetPolarizationOffsetsMotor1( PolarizationOffsets_t * PolarizationOffsets );

/* Starts the polarization offsets measurement procedure. */
MC_RetStatus_t MC_StartPolarizationOffsetsMeasurementMotor1(void);

/* Returns the state of the polarization offsets measurement procedure */
PolarizationState_t MC_GetPolarizationState(void);

/* Acknowledges Motor Control faults that occurred on Motor 1 */
bool MC_AcknowledgeFaultsMotor1( void );

/* Returns a bit-field showing any non acknowledged fault that occurred on Motor 1 */
uint32_t MC_GetOccurredFaultsMotor1(void);

/* Returns a bit-field showing any currently active fault on Motor 1 */
uint32_t MC_GetCurrentFaultsMotor1(void);

/* returns the current state of Motor 1 state machine */
MCI_State_t MC_GetSTMStateMotor1(void);

/* sets HSO minimum crossover frequency */
void MC_SetMinHSOCrossoverFreq(float frequency);
<#if MC.M1_SPEED_SENSOR == "ZEST">
/* sets alternate HSO minimum crossover frequency */
void MC_SetAltMinHSOCrossoverFreq(float frequency);
</#if>
/* sets HSO crossover frequency */
void MC_SetHSOCrossoverFreq(float frequency);

/* Configures RsDC estimation procedure, full procedure (Rs
   estimation and motor aligment) or only motor aligment */
void MC_SetRsDCEstimationAlignOnlyMotor1(bool value);

/* Enables RsDC estimation procedure. to be done before doing an FOC mode
   transition */
void MC_SetRsDCEstimationEnableMotor1(void);

/* Disables RsDC estimation procedure. to be done before doing an FOC mode
   transition*/
void MC_SetRsDCEstimationDisableMotor1(void);

/* Returns the state of the RsDC estimation procedure */
RsDC_estimationState_t MC_GetRsDCEstimationStateMotor1(void);

/* Enables angle estimation procedure at startup. To be done before doing a FOC mode
   transition */
void MC_EnablePulseMotor1(void);

/* Disables angle estimation procedure at startup. To be done before doing a FOC mode
   transition */
void MC_DisablePulseMotor1(void);

/**
  * Indicates whether pulses processing is completed.
  */
bool MC_IsPulseBusyMotor1(void);

/* Call the Profiler command */
uint8_t MC_ProfilerCommand (uint16_t rxLength, uint8_t *rxBuffer, int16_t txSyncFreeSpace, uint16_t *txLength, uint8_t *txBuffer);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MC_API_H */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

