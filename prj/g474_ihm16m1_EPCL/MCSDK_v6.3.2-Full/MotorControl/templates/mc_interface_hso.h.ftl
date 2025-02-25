<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    mc_interface.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          MC Interface component of the Motor Control SDK.
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
  * @ingroup MCInterfaceHSO
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_INTERFACE_H
#define __MC_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pwm_curr_fdbk.h"
#include "mc_curr_ctrl.h"
#include "speed_pos_fdbk_hso.h"
#include "speed_torq_ctrl_hso.h"   
#include "rsdc_est.h"
#include "mc_polpulse.h"
#include "profiler.h"
#include "bus_voltage.h"
<#if MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true>
#include "mc_potentiometer.h"
</#if>
<#if MC.DBG_MCU_LOAD_MEASURE == true>
#include "mc_perf.h"
</#if>

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup HSOAI
  * @{
  */

/** @addtogroup MCInterfaceHSO
  * @{
  */
/**
  * @brief  State_t enum type definition, it lists all the possible state machine states
  */
typedef enum
{
  ICLWAIT = 12,             /**< The system is waiting for ICL deactivation. Is not possible 
                              *  to run the motor if ICL is active. While the ICL is active 
                              *  the state is forced to #ICLWAIT; when ICL become inactive the 
                              *  state is moved to #IDLE. */
  IDLE = 0,                 /**< The state machine remains in this state as long as the   
                              *  application is not controlling the motor. This state is exited 
                              *  when the application sends a motor command or when a fault occurs. */
  CHARGE_BOOT_CAP = 16,     /**< The gate driver boot capacitors are being charged. */
  OFFSET_CALIB = 17,        /**< The offset of motor currents and voltages measurement cirtcuitry 
                              *  are being calibrated. */
  START = 4,                /**< The motor start-up is procedure is being executed. */
  RUN = 6,                  /**< The state machien remains in this state as long as the 
                              *  application is running (controlling) the motor. This state 
                              *  is exited when the application isues a stop command or when 
                              *  a fault occurs. */
  STOP = 8,                 /**< The stop motor procedure is being executed. */
  FAULT_NOW = 10,           /**< The state machine is moved from any state directly to this 
                              *  state when a fault occurs. The next state can only be 
                              *  #FAULT_OVER. */
  FAULT_OVER = 11,           /*!< The state machine transitions from #FAULT_NOW to this state
                              *   when there is no active fault condition anymore. It remains
                              * in this state until either a new fault condition occurs (in which
                              * case it goes back to the #FAULT_NOW state) or the application
                              * acknowledges the faults (in which case it goes to the #IDLE state).
                              */
  STATE_RSDCESTIMATE = 30,  /**< @brief estimate stator resistance at DC with D-current */
} MCI_State_t;

/**
  * @brief  Direct command enum type definition, it lists all the possible direct commands
  */
typedef enum
{
  MCI_NO_COMMAND = 0,   /*!< @brief No Command --- Set when going to IDLE */
  MCI_START,            /*!< @brief Start controling the Motor */
  MCI_ACK_FAULTS,       /*!< @brief Acknowledge Motor Control subsystem faults */
  MCI_MEASURE_OFFSETS,  /*!< @brief Start the ADCs Offset measurements procedure */
  MCI_STOP              /*!< @brief Stop the Motor and the control */
} MCI_DirectCommands_t;

/**
  * @brief  Motor Control Interface data structure definition
  */
typedef struct
{
  PWMC_Handle_t * pPWM;                   /*!< @brief PWM component handler */
  STC_Handle_t       *pSTC;               /*!< @brief Speed & torque component handler */  
  CurrCtrl_Handle_t  *pCurrCtrl;          /*!< @brief Current controller component handler */
  SPD_Handle_t *pSPD;           /*!< @brief Speed & position component handler */
  RsDCEstimation_Handle_t *pRsDCEst;      /*!< @brief Rs DC estimation component handler */
  PROFILER_Obj         *pProfiler;        /*!< @brief Profiler component handler */
  MC_PolPulse_Handle_t *pPolPulse;        /*!< @brief PolPulse component handler */
  BusVoltageSensor_Handle_t *pVBus;       /*!< @brief Bus voltage sensor handler */
<#if MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true>
  Potentiometer_Handle_t *pPotentiometer; /*!< @brief potentiometer handler */
</#if>
<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Handle_t  * pPerfMeasure;
</#if> 
  pFOCVars_t pFOCVars;                    /*!< @brief FOC variable data structure.*/
  MCI_DirectCommands_t DirectCommand;     /*!< @brief Direct command variable */
  MCI_State_t State;                      /*!< @brief State machine */
  uint16_t CurrentFaults;                 /*!< @brief on going faults */
  uint16_t PastFaults;                    /*!< @brief past faults not already acknowledged*/
} MCI_Handle_t;

/* Exported functions ------------------------------------------------------- */
void MCI_Init( MCI_Handle_t * pHandle, pFOCVars_t pFOCVars );

void MCI_SetCurrentReferences( MCI_Handle_t * pHandle, dq_fixp16_t Idq );
void MCI_SetCurrentReferences_F( MCI_Handle_t * pHandle, dq_float_t Idq );
void MCI_SetCurrentReferences_PU( MCI_Handle_t * pHandle, Currents_Idq_t Idq );

bool MCI_StartMotor( MCI_Handle_t * pHandle );
bool MCI_StartWithPolarizationMotor( MCI_Handle_t * pHandle );
bool MCI_StartOffsetMeasurments(MCI_Handle_t *pHandle);
bool MCI_StopMotor( MCI_Handle_t * pHandle );
MCI_State_t  MCI_GetSTMState( MCI_Handle_t * pHandle );

/* Acknowledges Motor Control faults that occurred on Motor 1 */
bool MCI_FaultAcknowledged( MCI_Handle_t * pHandle );
/* Returns a bit-field showing any non acknowledged fault that occured on Motor 1 */
uint32_t MCI_GetOccurredFaults( MCI_Handle_t * pHandle );
/* Returns a bit-field showing any currently active fault on Motor 1 */
uint32_t MCI_GetCurrentFaults( MCI_Handle_t * pHandle );
uint32_t MCI_GetFaultState(MCI_Handle_t *pHandle);
void MCI_FaultProcessing(MCI_Handle_t *pHandle, uint16_t hSetErrors, uint16_t hResetErrors);

fixp16_t MCI_GetAvrgSpeed( MCI_Handle_t * pHandle );
float MCI_GetAvrgSpeed_F( MCI_Handle_t * pHandle );
fixp30_t MCI_GetAvrgSpeed_PU( MCI_Handle_t * pHandle );
float MCI_GetEmfMagnitude_F( MCI_Handle_t * pHandle );
fixp30_t MCI_GetEmfMagnitude_PU( MCI_Handle_t * pHandle );
void MCI_SetFlagFreezeTrackAngle( MCI_Handle_t * pHandle, bool value );

/* Sets the control mode. */
bool MCI_SetControlMode( MCI_Handle_t * pHandle, MC_ControlMode_t mode );
/* Returns the Control Mode used for Motor 1. */
MC_ControlMode_t MCI_GetControlMode( MCI_Handle_t * pHandle );
bool MCI_StopRamp( MCI_Handle_t * pHandle );
dq_fixp16_t MCI_GetCurrentReferences( MCI_Handle_t * pHandle );
dq_float_t MCI_GetCurrentReferences_F( MCI_Handle_t * pHandle );
Currents_Idq_t MCI_GetCurrentReferences_PU( MCI_Handle_t * pHandle );
dq_fixp16_t MCI_GetCurrent( MCI_Handle_t * pHandle );
dq_float_t MCI_GetCurrent_F( MCI_Handle_t * pHandle );
Currents_Idq_t MCI_GetCurrent_PU( MCI_Handle_t * pHandle );
void MCI_SetDutyCycle_PU(  MCI_Handle_t * pHandle, Duty_Ddq_t DutyCycle );
Duty_Ddq_t MCI_GetDutyCycle_PU( const MCI_Handle_t * pHandle );

fixp24_t MCI_GetActiveFlux(const MCI_Handle_t* pHandle);
fixp24_t MCI_GetActiveLs(const MCI_Handle_t* pHandle);
fixp20_t MCI_GetActiveRs(const MCI_Handle_t* pHandle);
uint8_t MCI_GetFocControlMode(const MCI_Handle_t* pHandle);
fixp16_t MCI_GetIdRef(const MCI_Handle_t* pHandle);
fixp16_t MCI_GetIqRef(const MCI_Handle_t* pHandle);
fixp16_t MCI_GetIqRefSpeed(const MCI_Handle_t* pHandle);
fixp16_t MCI_GetMaxCurrent(const MCI_Handle_t* pHandle);
fixp30_t MCI_GetMaxModulation(const MCI_Handle_t* pHandle);
float_t MCI_GetPidCurrentKp(const MCI_Handle_t *pHandle);
float_t MCI_GetPidCurrentWi(const MCI_Handle_t *pHandle);
float_t MCI_GetPidSpeedKi(const MCI_Handle_t *pHandle);
float_t MCI_GetPidSpeedKp(const MCI_Handle_t *pHandle);
float_t MCI_GetPolePairs(const MCI_Handle_t *pHandle);
float_t MCI_GetProfilerFlux_Wb(const MCI_Handle_t *pHandle);
float_t MCI_GetProfilerFluxEstFreq_Hz(const MCI_Handle_t* pHandle);
float_t MCI_GetProfilerLd_H(const MCI_Handle_t *pHandle);
float_t MCI_GetProfilerPowerGoal_W(const MCI_Handle_t *pHandle);
float_t MCI_GetProfilerRs_DC(const MCI_Handle_t* pHandle);
float_t MCI_GetProfilerRs_AC(const MCI_Handle_t* pHandle);
uint8_t MCI_GetProfilerState(const MCI_Handle_t* pHandle);
fixp24_t MCI_GetRatedFlux(const MCI_Handle_t* pHandle);
fixp24_t MCI_GetRatedLs(const MCI_Handle_t* pHandle);
fixp20_t MCI_GetRatedRs(const MCI_Handle_t* pHandle);
fixp11_t MCI_GetSpeedRamp(const MCI_Handle_t* pHandle);
void MCI_SetTorqueReference( const MCI_Handle_t *pHandle, fixp16_t Torque );
void MCI_SetTorqueReference_F( const MCI_Handle_t *pHandle, float Torque );
void MCI_SetTorqueReference_PU( const MCI_Handle_t *pHandle, fixp30_t Torque );
fixp16_t MCI_GetTorqueReference( const MCI_Handle_t *pHandle );
float MCI_GetTorqueReference_F( const MCI_Handle_t *pHandle );
fixp30_t MCI_GetTorqueReference_PU( const MCI_Handle_t *pHandle );
fixp16_t MCI_GetTorque_Nm( const MCI_Handle_t *pHandle );
float MCI_GetTorque_Nm_F( const MCI_Handle_t *pHandle );
fixp30_t MCI_GetTorque_PU( const MCI_Handle_t *pHandle );
fixp24_t MCI_GetOneOverMaxIq(MCI_Handle_t* pHandle);
void MCI_SetActiveLs(MCI_Handle_t* pHandle, const fixp24_t ls_henry);
void MCI_SetActiveRs(MCI_Handle_t* pHandle, const fixp20_t rs_ohm);
void MCI_SetIdRef(MCI_Handle_t* pHandle, const fixp16_t id_ref_A);
void MCI_SetIqRef(MCI_Handle_t* pHandle, const fixp16_t iq_ref_A);
void MCI_SetMaxCurrent(MCI_Handle_t* pHandle, const fixp16_t I_max_A);
void MCI_SetMaxModulation(MCI_Handle_t* pHandle, const fixp30_t maxModulation);
void MCI_SetPidCurrentKp(MCI_Handle_t *pHandle, const float_t kp_si);
void MCI_SetPidCurrentWi(MCI_Handle_t *pHandle, const float_t wi_si);
void MCI_SetPidSpeedKi(MCI_Handle_t *pHandle, const float_t ki_si);
void MCI_SetPidSpeedKp(MCI_Handle_t *pHandle, const float_t kp_si);
void MCI_SetPolePairs(MCI_Handle_t *pHandle, const float_t pp);
void MCI_SetProfilerCommand(MCI_Handle_t* pHandle, const uint8_t command);
void MCI_SetProfilerFluxEstFreq_Hz(MCI_Handle_t* pHandle, const float_t fluxEstFreq_Hz);
void MCI_SetProfilerPowerGoal_W(MCI_Handle_t* pHandle, const float_t powerGoal_W);
void MCI_SetRatedFlux(MCI_Handle_t* pHandle, const fixp24_t flux_VpHz);
void MCI_SetRatedLs(MCI_Handle_t* pHandle, const fixp24_t ls_henry);
void MCI_SetRatedRs(MCI_Handle_t* pHandle, const fixp20_t rs_ohm);
void MCI_SetSpeedRamp( MCI_Handle_t* pHandle, const fixp16_t RampHzPerSec );
void MCI_SetSpeedRamp_F(MCI_Handle_t* pHandle, const float RampHzPerSec );
void MCI_SetSpeedRamp_PU(MCI_Handle_t* pHandle, const fixp30_t Acceleration_pu );
fixp16_t MCI_GetSpeedRamp( const MCI_Handle_t* pHandle );
float MCI_GetSpeedRamp_F( const MCI_Handle_t* pHandle );
fixp30_t MCI_GetSpeedRamp_PU( const MCI_Handle_t* pHandle );
void MCI_SetSpeedReference( const MCI_Handle_t* pHandle, const fixp16_t Speed );
void MCI_SetSpeedReference_F(const MCI_Handle_t* pHandle, const float Speed );
void MCI_SetSpeedReference_PU(const MCI_Handle_t* pHandle, const fixp30_t Speed );
fixp8_t MCI_GetSpeedReference( const MCI_Handle_t* pHandle );
float MCI_GetSpeedReference_F( const MCI_Handle_t* pHandle );
fixp30_t MCI_GetSpeedReference_PU( const MCI_Handle_t* pHandle );
#if 0
void MCI_ProgramSpeedRamp(MCI_Handle_t* pHandle, fixp16_t FinalSpeed, uint32_t DurationUs);
void MCI_ProgramSpeedRamp_F(MCI_Handle_t* pHandle, float FinalSpeed, uint32_t DurationUs);
void MCI_ProgramSpeedRamp_PU(MCI_Handle_t* pHandle, fixp30_t FinalSpeed,uint32_t DurationUs);
#endif
void MCI_SetMaxAcceleration(MCI_Handle_t* pHandle, fixp16_t Acceleration);
void MCI_SetMaxAcceleration_F(MCI_Handle_t* pHandle, float Acceleration);
void MCI_SetMaxAcceleration_PU(MCI_Handle_t* pHandle, fixp30_t Acceleration);
fixp16_t MCI_GetMaxAcceleration(MCI_Handle_t* pHandle);
float  MCI_GetMaxAcceleration_F(MCI_Handle_t* pHandle);
fixp30_t MCI_GetMaxAcceleration_PU(MCI_Handle_t* pHandle);
PolarizationState_t MCI_GetOffsetsCalibrationState( MCI_Handle_t* pHandle );
MC_RetStatus_t MCI_GetCalibratedOffsetsMotor(MCI_Handle_t* pHandle, PolarizationOffsets_t * PolarizationOffsets);
bool MCI_SetCalibratedOffsetsMotor( MCI_Handle_t* pHandle, PolarizationOffsets_t * PolarizationOffsets);
bool MCI_StartOffsetsCalibrationMotor(MCI_Handle_t* pHandle);
void MCI_SetCurrentPIDByRsLs(MCI_Handle_t* pHandle, const float_t rs, const float_t ls);
void MCI_SetMinHSOMinCrossoverFreqHz(MCI_Handle_t* pHandle, float frequency);
<#if MC.M1_SPEED_SENSOR == "ZEST">
void MCI_SetAltMinHSOMinCrossoverFreqHz(MCI_Handle_t* pHandle, float frequency);
</#if>
void MCI_SetHSOCrossoverFreqHz(MCI_Handle_t* pHandle, float frequency);
void MCI_SetRsDCEstimationAlignOnly(MCI_Handle_t* pHandle, bool value);
void MCI_SetRsDCEstimationEnable(MCI_Handle_t* pHandle);
void MCI_SetRsDCEstimationDisable(MCI_Handle_t* pHandle);
RsDC_estimationState_t MCI_GetRsDCEstimationState(MCI_Handle_t* pHandle);
void MCI_EnablePulse(MCI_Handle_t* pHandle);
void MCI_DisablePulse(MCI_Handle_t* pHandle);
void MCI_SetPulsePeriods(MCI_Handle_t *pHandle, const uint16_t N);
bool MCI_IsPulseBusy(MCI_Handle_t *pHandle);
void MCI_SetDecayDuration(MCI_Handle_t *pHandle, const uint16_t Nd);
void MCI_SetPulseCurrentGoal(MCI_Handle_t *pHandle, float PulseCurrentGoal_A);
uint16_t MCI_GetPulsePeriods(MCI_Handle_t *pHandle);
uint16_t MCI_GetDecayDuration(MCI_Handle_t *pHandle);
float MCI_GetPulseCurrentGoal( const MCI_Handle_t *pHandle );
fixp30_t MCI_GetPulseDuty( const MCI_Handle_t *pHandle );
fixp30_t MCI_GetPulseAngleEstimated( const MCI_Handle_t *pHandle );
void MCI_Clear_Iqdref(MCI_Handle_t *pHandle);
<#if MC.DBG_MCU_LOAD_MEASURE == true>
void MCI_Clear_PerfMeasure(MCI_Handle_t *pHandle, uint8_t bMotor);
</#if>
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

#endif /* __MC_INTERFACE_H */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

