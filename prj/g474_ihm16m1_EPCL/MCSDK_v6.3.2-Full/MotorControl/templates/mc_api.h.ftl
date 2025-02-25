<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
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
  * @ingroup MCIAPI
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_API_H
#define MC_API_H

#include "mc_type.h"
#include "mc_interface.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup CAI
  * @{
  */

/** @addtogroup MCIAPI
  * @{
  */

/* Starts Motor 1 */
bool MC_StartMotor1(void);

/* Stops Motor 1 */
bool MC_StopMotor1(void);

/* Programs a Speed ramp for Motor 1 */
void MC_ProgramSpeedRampMotor1(int16_t hFinalSpeed, uint16_t hDurationms);

<#if FOC>
/* Programs a Speed ramp for Motor 1 */
void MC_ProgramSpeedRampMotor1_F(float_t FinalSpeed, uint16_t hDurationms);

/* Programs a Torque ramp for Motor 1 */
void MC_ProgramTorqueRampMotor1(int16_t hFinalTorque, uint16_t hDurationms);

/* Programs a Torque ramp for Motor 1 */
void MC_ProgramTorqueRampMotor1_F(float_t FinalTorque, uint16_t hDurationms);
</#if>

<#if MC.M1_POSITION_CTRL_ENABLING == true>
/* Programs a target position for Motor 1 */
void MC_ProgramPositionCommandMotor1(float_t fTargetPosition, float_t fDuration);
</#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->

<#if FOC>
/* Programs a current reference for Motor 1 */
void MC_SetCurrentReferenceMotor1(qd_t Iqdref);

/* Programs a current reference for Motor 1 */
void MC_SetCurrentReferenceMotor1_F(qd_f_t IqdRef);
</#if><#-- FOC -->

<#if SIX_STEP>
/* Returns the current PWM duty cycle reference for Motor 1 */
int16_t MCI_GetDutyCycleRefMotor1(void);
</#if><#-- SIX_STEP -->

/* Returns the state of the last submited command for Motor 1 */
MCI_CommandState_t MC_GetCommandStateMotor1(void);

/* Stops the execution of the current speed ramp for Motor 1 if any */
bool MC_StopSpeedRampMotor1(void);

/* Stops the execution of the on going ramp for Motor 1 if any. 
   Note: this function is deprecated and should not be used anymore. It will be removed in a future version. */
void MC_StopRampMotor1(void);

/* Returns true if the last submited ramp for Motor 1 has completed, false otherwise */
bool MC_HasRampCompletedMotor1(void);

/* Returns the current mechanical rotor speed reference set for Motor 1, expressed in the unit defined by #SPEED_UNIT */
int16_t MC_GetMecSpeedReferenceMotor1(void);

/* Returns the current mechanical rotor speed reference set for Motor 1, expressed in rpm */
float_t MC_GetMecSpeedReferenceMotor1_F(void);

/* Returns the last computed average mechanical rotor speed for Motor 1, expressed in the unit defined by #SPEED_UNIT */
int16_t MC_GetMecSpeedAverageMotor1(void);

/* Returns the last computed average mechanical rotor speed for Motor 1, expressed in rpm */
float_t MC_GetAverageMecSpeedMotor1_F(void);

<#if FOC>
  <#if  AUX_SPEED_FDBK_M1 == true>
/* Returns the last computed average mechanical rotor speed from auxiliary sensor for Motor 1, expressed in the unit defined by #SPEED_UNIT */
int16_t MC_GetMecAuxiliarySpeedAverageM1(void);

/* Returns the last computed average mechanical rotor speed from auxiliary sensor for Motor 1, expressed in RPM */
float_t MC_GetMecAuxiliarySpeedAvgM1_F(void);

/* Returns the electrical angle of the rotor from auxiliary sensor of Motor 1, in DDP format */
int16_t MC_GetAuxiliaryElAngledppMotor1(void);

/* Returns the electrical angle of the rotor from auxiliary sensor of Motor 1, expressed in radians */
float_t MC_GetAuxiliaryElAngleMotor1_F(void);
  </#if><#-- AUX_SPEED_FDBK_M1 == true -->
  <#if AUX_SPEED_FDBK_M2 == true>

/* Returns the last computed average mechanical rotor speed from auxiliary sensor for Motor 2, expressed in the unit defined by #SPEED_UNIT */
int16_t MC_GetMecAuxiliarySpeedAverageMotor2(void);

/* Returns the last computed average mechanical rotor speed from auxiliary sensor for Motor 2, expressed in RPM */
float_t MC_GetMecAuxiliarySpeedAverageMotor2_F(void);

/* Returns the electrical angle of the rotor from auxiliary sensor of Motor 2, in DDP format */
int16_t MC_GetAuxiliaryElAngledppMotor2(void);

/* Returns the electrical angle of the rotor from auxiliary sensor of Motor 2, expressed in radians */
float_t MC_GetAuxiliaryElAngleMotor2_F(void);
  </#if><#-- AUX_SPEED_FDBK_M2 == true -->
</#if><#-- FOC -->

/* Returns the final speed of the last ramp programmed for Motor 1, if this ramp was a speed ramp */
int16_t MC_GetLastRampFinalSpeedMotor1(void);

/* Returns the final speed of the last ramp programmed for Motor 1, if this ramp was a speed ramp */
float_t MC_GetLastRampFinalSpeedM1_F(void);
<#if FOC>
/** Returns the final torque reference for Motor 1, expressed in Ampere. */
float_t MC_GetFinalTorqueReferenceMotor1_F(void);

/** Returns the final torque reference for Motor 1, expressed in digit. */
int16_t MC_GetFinalTorqueReferenceMotor1(void);
</#if><#-- FOC -->
/* Returns the current Control Mode for Motor 1 (either Speed or Torque) */
MC_ControlMode_t MC_GetControlModeMotor1(void);

/* Returns the direction imposed by the last command on Motor 1 */
int16_t MC_GetImposedDirectionMotor1(void);

/* Returns the current reliability of the speed sensor used for Motor 1 */
bool MC_GetSpeedSensorReliabilityMotor1(void);

<#if FOC>
/* returns the amplitude of the phase current injected in Motor 1 */
int16_t MC_GetPhaseCurrentAmplitudeMotor1(void);

/* returns the amplitude of the phase voltage applied to Motor 1 */
int16_t MC_GetPhaseVoltageAmplitudeMotor1(void);

/* returns current Ia and Ib values for Motor 1 */
ab_t MC_GetIabMotor1(void);

/* returns current Ia and Ib values in Ampere for Motor 1 */
ab_f_t MC_GetIabMotor1_F(void);

/* returns current Ialpha and Ibeta values for Motor 1 */
alphabeta_t MC_GetIalphabetaMotor1(void);

/* returns current Iq and Id values for Motor 1 */
qd_t MC_GetIqdMotor1(void);

/* returns current Iq and Id values in Ampere for Motor 1 */
qd_f_t MC_GetIqdMotor1_F(void);

/* returns Iq and Id reference values for Motor 1 */
qd_t MC_GetIqdrefMotor1(void);

/* returns Iq and Id reference values for Motor 1 */
qd_f_t MC_GetIqdrefMotor1_F(void);

/* returns current Vq and Vd values for Motor 1 */
qd_t MC_GetVqdMotor1(void);

/* returns current Valpha and Vbeta values for Motor 1 */
alphabeta_t MC_GetValphabetaMotor1(void);

/* returns the electrical angle of the rotor of Motor 1, in DDP format */
int16_t MC_GetElAngledppMotor1(void);

/* returns the current electrical torque reference for Motor 1 */
int16_t MC_GetTerefMotor1(void);

/* returns the current electrical torque reference in Ampere for Motor 1 */
float_t MC_GetTerefMotor1_F(void);

/* re-initializes Iq and Id references to their default values */
void MC_Clear_IqdrefMotor1(void);

/* Sets the polarization offset values to use for Motor 1*/
bool MC_SetPolarizationOffsetsMotor1(PolarizationOffsets_t * PolarizationOffsets);

/* @brief Returns the polarization offset values measured or set for Motor 1 */
bool MC_GetPolarizationOffsetsMotor1(PolarizationOffsets_t * PolarizationOffsets);
</#if><#-- FOC -->

<#if FOC>
/* Starts the polarization offsets measurement procedure for Motor 1. */  
bool MC_StartPolarizationOffsetsMeasurementMotor1(void);
</#if><#-- FOC -->

/* Acknowledge a Motor Control fault on Motor 1 */
bool MC_AcknowledgeFaultMotor1(void);

/* Returns a bitfiled showing faults that occured since the State Machine of Motor 1 was moved to FAULT_NOW state */
uint16_t MC_GetOccurredFaultsMotor1(void);

/* Returns a bitfield showing all current faults on Motor 1 */
uint16_t MC_GetCurrentFaultsMotor1(void);

/* returns the current state of Motor 1 state machine */
MCI_State_t  MC_GetSTMStateMotor1(void);

<#if FOC>
/* returns the current power of Motor 1 in float_t format */
float_t MC_GetAveragePowerMotor1_F(void);
</#if><#-- FOC -->

<#if MC.M1_POSITION_CTRL_ENABLING == true>
/* returns the current control position state of Motor 1 */
PosCtrlStatus_t  MC_GetControlPositionStatusMotor1(void);

/* returns the alignment state of Motor 1 */
AlignStatus_t  MC_GetAlignmentStatusMotor1(void);

/* returns the current position of Motor 1. */
float_t MC_GetCurrentPosition1(void);

/* returns the target position of Motor 1. */
float_t MC_GetTargetPosition1(void);

/* returns the total movement duration to reach the target position of Motor 1. */
float_t MC_GetMoveDuration1(void);
</#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->


/* Call the Profiler command */
uint8_t MC_ProfilerCommand (uint16_t rxLength, uint8_t *rxBuffer, int16_t txSyncFreeSpace, uint16_t *txLength, uint8_t *txBuffer);


<#if MC.DRIVE_NUMBER != "1">
/* Starts Motor 2 */
bool MC_StartMotor2(void);

/* Stops Motor 2 */
bool MC_StopMotor2(void);

/* Programs a Speed ramp for Motor 2 */
void MC_ProgramSpeedRampMotor2(int16_t hFinalSpeed, uint16_t hDurationms);

/* Programs a Speed ramp for Motor 2 */
void MC_ProgramSpeedRampMotor2_F(float_t FinalSpeed, uint16_t hDurationms);

  <#if FOC>
/* Programs a Torque ramp for Motor 2 */
void MC_ProgramTorqueRampMotor2(int16_t hFinalTorque, uint16_t hDurationms);

/* Programs a Torque ramp for Motor 2 */
void MC_ProgramTorqueRampMotor2_F(float_t FinalTorque, uint16_t hDurationms);
  </#if><#-- FOC -->

  <#if MC.M2_POSITION_CTRL_ENABLING == true>
/* Programs a target position for Motor 2 */
void MC_ProgramPositionCommandMotor2(float_t fTargetPosition, float_t fDuration);
  </#if><#-- MC.M2_POSITION_CTRL_ENABLING == true -->

/* Programs a current reference for Motor 2 */
void MC_SetCurrentReferenceMotor2(qd_t Iqdref);

/* Programs a current reference for Motor 2 */
void MC_SetCurrentReferenceMotor2_F(qd_f_t IqdRef);

/* Returns the state of the last submited command for Motor 2 */
MCI_CommandState_t  MC_GetCommandStateMotor2(void);

/* Stops the execution of the current speed ramp for Motor 2 if any */
bool MC_StopSpeedRampMotor2(void);

/* Stops the execution of the on going ramp for Motor 2 if any
   Note: this function is deprecated and should not be used anymore. It will be removed in a future version. */
void MC_StopRampMotor2(void);

/* Returns true if the last submited ramp for Motor 2 has completed, false otherwise */
bool MC_HasRampCompletedMotor2(void);

/* Returns the current mechanical rotor speed reference set for Motor 2, expressed in the unit defined by SPEED_UNIT */
int16_t MC_GetMecSpeedReferenceMotor2(void);

/* Returns the current mechanical rotor speed reference set for Motor 2, expressed in rpm */
float_t MC_GetMecSpeedReferenceMotor2_F(void);

/* Returns the last computed average mechanical rotor speed for Motor 2, expressed in the unit defined by SPEED_UNIT */
int16_t MC_GetMecSpeedAverageMotor2(void);

/* Returns the last computed average mechanical rotor speed for Motor 2, expressed in rpm */
float_t MC_GetAverageMecSpeedMotor2_F(void);

/* Returns the final speed of the last ramp programmed for Motor 2, if this ramp was a speed ramp */
int16_t MC_GetLastRampFinalSpeedMotor2(void);

/* Returns the final speed of the last ramp programmed for Motor 2, if this ramp was a speed ramp */
float_t MC_GetLastRampFinalSpeedMotor2_F(void);

/* Returns the current Control Mode for Motor 2 (either Speed or Torque) */
MC_ControlMode_t MC_GetControlModeMotor2(void);

/* Returns the direction imposed by the last command on Motor 2 */
int16_t MC_GetImposedDirectionMotor2(void);

/* Returns the current reliability of the speed sensor used for Motor 2 */
bool MC_GetSpeedSensorReliabilityMotor2(void);

  <#if FOC>
/* returns the amplitude of the phase current injected in Motor 2 */
int16_t MC_GetPhaseCurrentAmplitudeMotor2(void);

/* returns the amplitude of the phase voltage applied to Motor 2 */
int16_t MC_GetPhaseVoltageAmplitudeMotor2(void);

/* returns current Ia and Ib values for Motor 2 */
ab_t MC_GetIabMotor2(void);

/* returns current Ia and Ib values in Ampere for Motor 2 */
ab_f_t MC_GetIabMotor2_F(void);

/* returns current Ialpha and Ibeta values for Motor 2 */
alphabeta_t MC_GetIalphabetaMotor2(void);

/* returns current Iq and Id values for Motor 2 */
qd_t MC_GetIqdMotor2(void);

/* returns current Iq and Id values in Ampere for Motor 2 */
qd_f_t MC_GetIqdMotor2_F(void);

/* returns Iq and Id reference values for Motor 2 */
qd_t MC_GetIqdrefMotor2(void);

/* returns current Vq and Vd values for Motor 2 */
qd_t MC_GetVqdMotor2(void);

/* returns current Valpha and Vbeta values for Motor 2 */
alphabeta_t MC_GetValphabetaMotor2(void);

/* returns the electrical angle of the rotor of Motor 2, in DDP format */
int16_t MC_GetElAngledppMotor2(void);

/** Returns the final torque reference for Motor 1, expressed in Ampere. */
float_t MC_GetFinalTorqueReferenceMotor2_F(void);

/** Returns the final torque reference for Motor 1, expressed in digit. */
int16_t MC_GetFinalTorqueReferenceMotor2(void);

/* returns the current electrical torque reference for Motor 2 */
int16_t MC_GetTerefMotor2(void);

/* returns the current electrical torque reference in Ampere for Motor 2 */
float_t MC_GetTerefMotor2_F(void);

/* re-initializes Iq and Id references their default values */
void MC_Clear_IqdrefMotor2(void);
  </#if><#-- FOC -->

/* Acknowledge a Motor Control fault on Motor 2 */
bool MC_AcknowledgeFaultMotor2(void);

/* Returns a bitfiled showing faults that occured since the State Machine of Motor 2 was moved to FAULT_NOW state */
uint16_t MC_GetOccurredFaultsMotor2(void);

/* Returns a bitfield showing all current faults on Motor 2 */
uint16_t MC_GetCurrentFaultsMotor2(void);

/* returns the current state of Motor 2 state machine */
MCI_State_t  MC_GetSTMStateMotor2(void);

  <#if FOC>
/* Sets the polarization offset values to use for Motor 2*/
bool MC_SetPolarizationOffsetsMotor2(PolarizationOffsets_t * PolarizationOffsets);

/* @brief Returns the polarization offset values measured or set for Motor 2 */
bool MC_GetPolarizationOffsetsMotor2(PolarizationOffsets_t * PolarizationOffsets);

/* Starts the polarization offsets measurement procedure for Motor 2. */  
bool MC_StartPolarizationOffsetsMeasurementMotor2(void);

/* returns the current power of Motor 2 in float_t format */
float_t MC_GetAveragePowerMotor2_F(void);
  </#if><#-- FOC -->

  <#if MC.M2_POSITION_CTRL_ENABLING == true>
/* returns the current control position state of Motor 1 */
PosCtrlStatus_t  MC_GetControlPositionStatusMotor2(void);

/* returns the alignment state of Motor 2 */
AlignStatus_t  MC_GetAlignmentStatusMotor2(void);

/* returns the current position of Motor 2. */
float_t MC_GetCurrentPosition2(void);

/* returns the target position of Motor 2. */
float_t MC_GetTargetPosition2(void);

/* returns the total movement duration to reach the target position of Motor 2. */
float_t MC_GetMoveDuration2(void);

  </#if><#-- FOC -->

</#if><#-- MC.DRIVE_NUMBER > 1 -->

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
