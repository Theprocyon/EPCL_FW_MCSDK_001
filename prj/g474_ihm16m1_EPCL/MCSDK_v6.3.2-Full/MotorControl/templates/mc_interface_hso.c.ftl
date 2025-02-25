<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    mc_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the MC Interface component of the Motor Control SDK.
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
  * @ingroup MCInterfaceHSO
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_math.h"
#include "mc_interface.h"
#include "drive_parameters.h" 
#include "parameters_conversion.h"
#include "mc_parameters.h"
<#if MC.MOTOR_PROFILER ==true>  
#include "profiler.h"
#include "profiler_dcac.h"
</#if>


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup HSOAI
  * @{
  */

/** @defgroup MCInterfaceHSO Motor Control Interface for HSO
  * @brief MC Interface component of the Motor Control SDK for HSO applications
  *
  *  This interface allows for performing basic operations on the motor driven by a
  *  Motor Control SDK based application. With it, motors can be started and stopped, speed or
  *  torque ramps can be programmed and executed and information on the state of the motor can
  *  be retrieved, among others.
  *
  *  These functions aims at being the main interface used by an application to control the motor.
  * 
  * @{
  */

/* Private macros ------------------------------------------------------------*/

/* Functions -----------------------------------------------*/

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation. It is also used to assign the FOC
  *         drive object to be used by MC Interface.
  * @param  pHandle pointer on the component instance to initialize.
  * @param  pFOCVars pointer to FOC vars to be used by MCI.
  */
__weak void MCI_Init( MCI_Handle_t * pHandle, pFOCVars_t pFOCVars )
{

  pHandle->pFOCVars = pFOCVars;
  pHandle->DirectCommand = MCI_NO_COMMAND;
  pHandle->State = IDLE;
  pHandle->CurrentFaults = MC_NO_FAULTS;
  pHandle->PastFaults = MC_NO_FAULTS;
}

/**
  * @brief  Sets the motor current references $I_q$ and $I_d$ directly.
  *
  * $I_d$ and $I_q$ are provided in floating point format an in Amperes. 
  *
  * @param pHandle handle on the targer motor
  * @param Idq the current references
  */
void MCI_SetCurrentReferences_F( MCI_Handle_t * pHandle, dq_float_t Idq )
{
    fixp30_t Id_ref_pu, Iq_ref_pu;
    fixp30_t I_max_pu = pHandle->pSTC->I_max_pu;

    Id_ref_pu = FIXP30( Idq.D / scaleParams->current );
    Id_ref_pu = FIXP_sat( Id_ref_pu, I_max_pu, -I_max_pu );

    Iq_ref_pu = FIXP30( Idq.Q / scaleParams->current );
    Iq_ref_pu = FIXP_sat( Iq_ref_pu, I_max_pu, -I_max_pu );

    /* TODO: Protect with a critical section */
    pHandle->pSTC->Idq_ref_pu.D = Id_ref_pu;
    pHandle->pSTC->Idq_ref_pu.Q = Iq_ref_pu;
}

/**
  * @brief  Sets the motor current references $I_q$ and $I_d$ directly.
  *
  * $I_d$ and $I_q$ are provided in in Per Unit using a fixed point format. 
  *
  * @param pHandle handle on the targer motor
  * @param Idq the current references
  */
void MCI_SetCurrentReferences_PU( MCI_Handle_t * pHandle, Currents_Idq_t Idq )
{
    /* TODO: Protect with a critical section */
    pHandle->pSTC->Idq_ref_pu.D = Idq.D;
    pHandle->pSTC->Idq_ref_pu.Q = Idq.Q;
}

/**
  * @brief  Sets the motor current references $I_q$ and $I_d$ directly.
  *
  * $I_d$ and $I_q$ are provided in Amperes using the Q16 format. 
  *
  * @param pHandle handle on the targer motor
  * @param Idq the current references
  */
void MCI_SetCurrentReferences( MCI_Handle_t * pHandle, dq_fixp16_t Idq )
{
    /* Convert from fixp16_t A to fixp30_t _pu_A */
    fixp30_t Id_ref_pu, Iq_ref_pu;
    fixp30_t I_max_pu = pHandle->pSTC->I_max_pu;

    Id_ref_pu = FIXP16_mpy(Idq.D, FIXP30(1.0f / scaleParams->current));
    Id_ref_pu = FIXP_sat(Id_ref_pu, I_max_pu, -I_max_pu);

    Iq_ref_pu = FIXP16_mpy(Idq.Q, FIXP30(1.0f / scaleParams->current));
    Iq_ref_pu = FIXP_sat( Iq_ref_pu, I_max_pu, -I_max_pu );

    /* TODO: Protect with a critical section */
    pHandle->pSTC->Idq_ref_pu.D = Id_ref_pu;
    pHandle->pSTC->Idq_ref_pu.Q = Iq_ref_pu;
}

/**
  * @brief  Initiates the start-up procedure for a Motor
  *
  *  The MCI_StartMotor() function triggers the start-up procedure of the target motor
  * and returns immediately. It returns true if successful, or an error
  * code identifying the reason for failure if not.
  *
  *  This function is not blocking: it only initiates the start-up procedure. When
  * the procedure completes, the state machine of the motor goes to the #RUN state.
  * The state of the motor can be queried with the MCI_GetSTMState() function.
  *
  *  At the end of the start-up procedure, the motor is ready to be controlled by the
  * application.
  *
  *  The state machine of the motor must be in the #IDLE state for this function to
  * succeed. If it is not the case, the start-up procedure is not triggered and
  * the false status is returned.
  *
  * @param  pHandle Pointer on the component instance to work on.
  * @retval returns true if successfully executed, or an error code otherwise.
  * 
  */
__weak bool MCI_StartMotor( MCI_Handle_t * pHandle )
{
  bool RetVal = false;

  if ((IDLE == MCI_GetSTMState(pHandle)) &&
            (MC_NO_FAULTS == MCI_GetOccurredFaults(pHandle)) &&
            (MC_NO_FAULTS == MCI_GetCurrentFaults(pHandle)))
  {
    pHandle->DirectCommand = MCI_START;
    RetVal = true;
  }

  return (RetVal);
}

/**
  * @brief  Initiates the polarization offsets measurement procedure
  *
  *  The MCI_StartOffsetMeasurments() function triggers the polarization offsets 
  * measurement procedure and returns immediately. It returns true if successful, 
  * or an error code identifying the reason for failure if not.
  *
  *  This function is not blocking: it only initiates the polarization offsets measurement. 
  * When the procedure completes, the state machine of the motor goes to the #IDLE state.
  * The state of the motor can be queried with the MCI_GetSTMState() function.
  *
  *  The state machine of the motor must be in the #IDLE state for this function to
  * succeed. If it is not the case, the start-up procedure is not triggered and
  * the false status is returned.
  * 
  * @param  pHandle Pointer on the component instance to work on.
  * @retval returns true if successfully executed, or an error code otherwise.
  * 
  */
__weak bool MCI_StartOffsetMeasurments(MCI_Handle_t *pHandle)
{
  bool RetVal;

  if ((IDLE == MCI_GetSTMState(pHandle)) &&
      (MC_NO_FAULTS == MCI_GetOccurredFaults(pHandle)) &&
      (MC_NO_FAULTS == MCI_GetCurrentFaults(pHandle)))
  {
    pHandle->DirectCommand = MCI_MEASURE_OFFSETS;
    pHandle->pFOCVars->PolarizationState = NOT_DONE;
    RetVal = true;
  }
  else
  {
    /* reject the command as the condition are not met */
    RetVal = false;
  }

  return (RetVal);
}

/**
  * @brief  Initiates the start-up procedure for a Motor and the polarization offsets
  *         measurement procedure if needed
  *
  *  The MCI_StartWithPolarizationMotor() function triggers the start-up procedure of 
  * the target motor and returns immediately. It returns true if successful, or 
  * an error code identifying the reason for failure if not.
  *
  *  This function is not blocking: it only initiates the start-up procedure. When
  * the procedure completes, the state machine of the motor goes to the #RUN state.
  * The state of the motor can be queried with the MCI_GetSTMState() function.
  *
  *  At the end of the start-up procedure, the motor is ready to be controlled by the
  * application.
  *
  *  The state machine of the motor must be in the #IDLE state for this function to
  * succeed. If it is not the case, the start-up procedure is not triggered and
  * the false status is returned.
  *
  *  If the polarization offsets have not been either measured by or provided to the
  * Motor Control subsystem before this function is called, the polarization offsets
  * measurement procedure is executed before the motor is started. See
  * MC_StartPolarizationOffsetsMeasurementMotor1() for the polarization offsets
  * measurements procedure and the MC_SetPolarizationOffsetsMotor1() function.
  * 
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool returns true if successfully executed, or an
  *         false otherwise.
  */
__weak bool MCI_StartWithPolarizationMotor( MCI_Handle_t * pHandle )
{
  bool RetVal = false;

  if ((IDLE == MCI_GetSTMState(pHandle)) &&
            (MC_NO_FAULTS == MCI_GetOccurredFaults(pHandle)) &&
            (MC_NO_FAULTS == MCI_GetCurrentFaults(pHandle)))
  {
    pHandle->DirectCommand = MCI_START;
    pHandle->pFOCVars->flagStartWithCalibration = true;
    pHandle->pFOCVars->PolarizationState = NOT_DONE;

    RetVal = true;
  }

  return (RetVal);
}

/**
  * @brief  Initiates the stop procedure for a Motor.
  *
  *  The MCI_StopMotor() function triggers the stop procedure of the motor
  * and returns immediately. It returns true if successful, or an error
  * code identifying the reason for failure if not.
  *
  *  This function is not blocking: it only initiates the stop procedure. When
  * this procedure completes, the state machine of the motor goes to the #IDLE state.
  * The state of the motor can be queried with the MCI_GetSTMState() function.
  *
  *  At the end of the stop procedure, the motor is not under control anymore.
  *
  *  The state machine of the motor can be in be in the #RUN, #CHARGE_BOOT_CAP,
  * #OFFSET_CALIB or #STATE_RSDCESTIMATE states for this function to
  * succeed. If it is not the case, the stop procedure is not triggered and
  * the false status is returned.
  * 
  * @param  pHandle Pointer on the component instance to work on.
  * @retval returns true if successfully executed, or an error code otherwise.
  */
__weak bool MCI_StopMotor( MCI_Handle_t * pHandle )
{
  bool RetVal;
  bool status;
  MCI_State_t State;

  State = MCI_GetSTMState(pHandle);
  if (IDLE == State  || ICLWAIT == State)
  {
    status = false;
  }
  else
  {
    status = true;
  }

  if ((MC_NO_FAULTS == MCI_GetOccurredFaults(pHandle)) &&
      (MC_NO_FAULTS == MCI_GetCurrentFaults(pHandle)) &&
       status == true )
  {
    pHandle->DirectCommand = MCI_STOP;
    RetVal = true;
  }
  else
  {
    /* reject the command as the condition are not met */
    RetVal = false;
  }
  return(RetVal);
}

/**
  * @brief Returns current state of the state machine of the target motor.
  *
  * @param  pHandle Pointer on the target motor drive structure.
  */
__weak MCI_State_t  MCI_GetSTMState( MCI_Handle_t * pHandle )
{
  return pHandle->State;
}

/**
  * @brief Returns the lists of current and past faults that occurred on the target motor
  *
  *  This function returns two bitfields containing information about the faults currently 
  * present and the faults occurred since the state machine has been moved into the #FAULT_NOW 
  * state. 
  *
  * These two bitfields are 16 bits wide each and are concatenated into the 32-bit data. The 
  * 16 most significant bits contains the status of the current faults while that of the 
  * past faults is in the 16 least significant bits. 
  * 
  * @sa MCI_GetOccurredFaults, MCI_GetCurrentFaults
  *
  * @param  pHandle Pointer on the target motor drive structure.
  */
__weak uint32_t MCI_GetFaultState(MCI_Handle_t *pHandle)
{
  uint32_t LocalFaultState;

  LocalFaultState = (uint32_t)(pHandle->PastFaults);
  LocalFaultState |= (uint32_t)(pHandle->CurrentFaults) << 16;

  return (LocalFaultState);
}

/**
 * @brief Acknowledges Motor Control faults that occurred on the target motor 1.
 *
 *  This function must be called before the motor can be started again when a fault 
 * condition has occured. It clears the faults status and resets the state machine 
 * of the target motor to the #IDLE state provided that there is no active fault 
 * condition anymore.
 *
 *  If the state machine of the target motor is in the #FAULT_OVER state, the function
 * clears the list of past faults, transitions to the #IDLE state and returns true.
 * Otherwise, it oes nothing and returns false.
 *
 * @param  pHandle Pointer on the target motor drive structure.
 */
__weak bool MCI_FaultAcknowledged( MCI_Handle_t * pHandle )
{
  bool RetVal;

  if ((FAULT_OVER == MCI_GetSTMState(pHandle)) && (MC_NO_FAULTS == MCI_GetCurrentFaults(pHandle)))
  {
    pHandle->PastFaults = MC_NO_FAULTS;
    pHandle->DirectCommand = MCI_ACK_FAULTS;
    RetVal = true;
  }
  else
  {
    /* reject the command as the conditions are not met */
    RetVal = false;
  }
  return (RetVal);
}

/**
  * @brief Upadtes the fault status of the target motor  
  *
  *  The faults indicated in the @p hSetErrors parameter are added to the list of 
  * currently active faults conditions while those indicated in the @p hResetErrors 
  * parameter are removed from it. 
  *
  * The list of past faults (non active faults that occured in the past and that have 
  * not been cleared) is also updated with the @p hSetErrors parameter.
  *
  * @note This function is not meant to be called by the application.
  *
  * @param  pHandle Pointer on the target motor drive structure.
  * @param  hSetErrors Bit field reporting faults currently present
  * @param  hResetErrors Bit field reporting faults to be cleared
  */
__weak void MCI_FaultProcessing(MCI_Handle_t *pHandle, uint16_t hSetErrors, uint16_t hResetErrors)
{
  /* Set current errors */
  pHandle->CurrentFaults = (pHandle->CurrentFaults | hSetErrors ) & (~hResetErrors);
  pHandle->PastFaults |= hSetErrors;

  return;
}

/**
  * @brief Returns the list of non-acknowledged faults that occured on the target motor
  *
  * This function returns a bitfield indicating the faults that occured since the state machine
  * of the target motor has been moved into the #FAULT_NOW state.
  *
  * Possible error codes are listed in the @ref fault_codes "Fault codes" section.
  *
  * @param  pHandle Pointer on the target motor drive structure.
  */
uint32_t MCI_GetOccurredFaults( MCI_Handle_t * pHandle )
{
    /* For the time being, faults are still internally stored on 16 bits. */
  return ((uint16_t)pHandle->PastFaults);
}

/**
  * @brief Returns the list of faults that are currently active on the target motor
  *
  * Possible error codes are listed in the @ref fault_codes "Fault codes" section.
  *
  * @param  pHandle Pointer on the target motor drive structure.
  */
uint32_t MCI_GetCurrentFaults( MCI_Handle_t * pHandle )
{
    /* For the time being, faults are still internally stored on 16 bits. */
  return ((uint16_t)pHandle->CurrentFaults);
}

/** @brief Sets the control mode of the motor represented by @p pHandle to @p mode. */
bool MCI_SetControlMode( MCI_Handle_t * pHandle, MC_ControlMode_t mode )
{
  bool ret_val = true;
  FOCVars_t* pFOCVars = pHandle->pFOCVars;

  switch ( mode )
  {
    case MCM_OBSERVING_MODE:
    case MCM_OPEN_LOOP_VOLTAGE_MODE:
    case MCM_OPEN_LOOP_CURRENT_MODE:
    case MCM_PROFILING_MODE:      
    case MCM_SHORTED_MODE:      
      pFOCVars->controlMode = mode;      
      break;

    case MCM_TORQUE_MODE:
    case MCM_SPEED_MODE:      
      pFOCVars->controlMode = mode;
      if (RSDC_EST_COMPLETED == pHandle->pRsDCEst->RSDCestimate_state)
      {
        pHandle->pRsDCEst->RSDCestimate_state = RSDC_EST_RESET;
      }
      break;

    default:
      /* Incorrect control mode set */
      pHandle->pSTC->SpeedControlEnabled = false;
      pHandle->pCurrCtrl->currentControlEnabled = false;
      pHandle->pSPD->closedLoopAngleEnabled = false;
      pHandle->pSPD->flagHsoAngleEqualsOpenLoopAngle = false;
      pHandle->pCurrCtrl->forceZeroPwm = true;
<#if MC.M1_SPEED_SENSOR == "ZEST">    
      ZEST_setInjectMode(pHandle->pSPD->pZeST,ZEST_INJECTMODE_Auto );
</#if>      
      ret_val = false;
    }

    return ret_val;
}

/** @brief Returns the Control Mode used for the motor represented by @p pHandle. */
MC_ControlMode_t MCI_GetControlMode( MCI_Handle_t * pHandle )
{
  MC_ControlMode_t current_mode = pHandle->pFOCVars->controlMode;

  if (pHandle->pFOCVars->controlMode_prev != pHandle->pFOCVars->controlMode)
  {
    current_mode = pHandle->pFOCVars->controlMode_prev;
  }
  return current_mode;
}

/** @brief returns the $I_d$ and $I_q$ current references set to the motor represented by @p pHandle in Amperes using the Q16 format. */
dq_fixp16_t MCI_GetCurrentReferences( MCI_Handle_t * pHandle )
{
    dq_fixp16_t ret_val;

    ret_val.D = FIXP16( FIXP30_toF( pHandle->pSTC->Idq_ref_pu.D ) * scaleParams->current);
    ret_val.Q = FIXP16( FIXP30_toF( pHandle->pSTC->Idq_ref_pu.Q ) * scaleParams->current);

    return ret_val;
}

/** @brief returns the $I_d$ and $I_q$ current references set to the motor represented by @p pHandle in Amperes using the float format. */
dq_float_t MCI_GetCurrentReferences_F( MCI_Handle_t * pHandle )
{
    dq_float_t ret_val;

    ret_val.D = FIXP30_toF( pHandle->pSTC->Idq_ref_pu.D ) * scaleParams->current;
    ret_val.Q = FIXP30_toF( pHandle->pSTC->Idq_ref_pu.Q ) * scaleParams->current;

    return ret_val;
}

/** @brief returns the $I_d$ and $I_q$ current references set to the motor represented by @p pHandle in "Per Unit". */
Currents_Idq_t MCI_GetCurrentReferences_PU( MCI_Handle_t * pHandle )
{
    return pHandle->pSTC->Idq_ref_pu;
}

/** @brief returns the last $I_d$ and $I_q$ currents measured on the motor represented by @p pHandle in Amperes using the Q16 format. */
dq_fixp16_t MCI_GetCurrent( MCI_Handle_t * pHandle )
{
    dq_fixp16_t ret_val;

    ret_val.D = FIXP16( FIXP30_toF( pHandle->pCurrCtrl->Idq_in_pu.D ) * scaleParams->current);
    ret_val.Q = FIXP16( FIXP30_toF( pHandle->pCurrCtrl->Idq_in_pu.Q ) * scaleParams->current);

    return ret_val;
}

/** @brief returns the last $I_d$ and $I_q$ currents measured on the motor represented by @p pHandle in Amperes using the float format. */
dq_float_t MCI_GetCurrent_F( MCI_Handle_t * pHandle )
{
    dq_float_t ret_val;

    ret_val.D = FIXP30_toF( pHandle->pCurrCtrl->Idq_in_pu.D ) * scaleParams->current;
    ret_val.Q = FIXP30_toF( pHandle->pCurrCtrl->Idq_in_pu.Q ) * scaleParams->current;

    return ret_val;
}

/** @brief returns the last $I_d$ and $I_q$ currents measured on the motor represented by @p pHandle in "Per Unit". */
Currents_Idq_t MCI_GetCurrent_PU( MCI_Handle_t * pHandle )
{
    return pHandle->pCurrCtrl->Idq_in_pu;
}

/** @brief Sets the phase voltage duty cycles for the motor represented by @p pHandle to @p DutyCycle. */
void MCI_SetDutyCycle_PU(  MCI_Handle_t * pHandle, Duty_Ddq_t DutyCycle )
{
    pHandle->pCurrCtrl->Ddq_ref_pu = DutyCycle;
}

/** @brief Returns the phase voltage duty cycles set for the motor represented by @p pHandle. */
Voltages_Udq_t MCI_GetDutyCycle_PU( const  MCI_Handle_t * pHandle )
{
    return pHandle->pCurrCtrl->Ddq_ref_pu;
}

/* Local helper functions */

static fixp24_t MCI_ConvertL_pu_fps_to_fixp24(const MCI_Handle_t* pHandle, const FIXP_scaled_t* pFps)
{
  float_t ls_pu = FIXPSCALED_FIXPscaledToFloat(pFps);
  float_t ls_henry = ls_pu * (scaleParams->voltage / scaleParams->current / (float_t)VOLTAGE_FILTER_POLE_RPS);
  return FIXP24(ls_henry);
}

static fixp20_t MCI_ConvertR_pu_fps_to_fixp20(const MCI_Handle_t* pHandle, const FIXP_scaled_t* pFps)
{
  float_t rs_pu = FIXPSCALED_FIXPscaledToFloat(pFps);
  float_t rs_ohm = rs_pu * (scaleParams->voltage / scaleParams->current);
  return FIXP20(rs_ohm);
}

static void MCI_ConvertL_fixp24_to_pu_fps(const MCI_Handle_t* pHandle, const fixp24_t l, FIXP_scaled_t* pFps)
{
  float_t l_pu_flt = FIXP24_toF(l) / (scaleParams->voltage / scaleParams->current / (float_t)VOLTAGE_FILTER_POLE_RPS);
  FIXPSCALED_floatToFIXPscaled(l_pu_flt, pFps);
}

/* Accessors for motor control */

/** @brief Returns the flux of the motor represented by @p pHandle in Weber using the Q24 format */
fixp24_t MCI_GetActiveFlux(const MCI_Handle_t* pHandle)
{
  return FIXP30_mpy(pHandle->pSPD->Flux_Active_Wb, FIXP24(M_TWOPI));
}

/** @brief Returns the phase inductance of the motor represented by @p pHandle in Henry using the Q24 format */
fixp24_t MCI_GetActiveLs(const MCI_Handle_t* pHandle)
{
<#if MC.M1_SPEED_SENSOR == "HSO">
  return MCI_ConvertL_pu_fps_to_fixp24(pHandle, &pHandle->pSPD->Ls_Active_pu_fps);
<#else>
  return MCI_ConvertL_pu_fps_to_fixp24(pHandle, &pHandle->pSPD->Ls_Active_pu_fps) + FIXP24(ZEST_getL(pHandle->pSPD->pZeST));
</#if>
}

/** @brief Returns the phase resistance of the motor represented by @p pHandle in HenOhmry using the Q20 format */
fixp20_t MCI_GetActiveRs(const MCI_Handle_t* pHandle)
{
  /* Read from ImpedCorr module, which is the authorative source for the active resistance */
  FIXP_scaled_t rs_pu_fps = IMPEDCORR_getRs_pu_fps(pHandle->pSPD->pImpedCorr);
  return MCI_ConvertR_pu_fps_to_fixp20(pHandle, &rs_pu_fps);
}

/** @brief Returns the control mode of the motor represented by @p pHandle. */
uint8_t MCI_GetFocControlMode(const MCI_Handle_t* pHandle)
{
  return pHandle->pFOCVars->controlMode;
}

/** @brief Returns the $I_d$ current reference set to the motor represented by @p pHandle in Ampere using Q16 format. */
fixp16_t MCI_GetIdRef(const MCI_Handle_t* pHandle)
{
  return FIXP30_mpy(pHandle->pSTC->Idq_ref_pu.D, FIXP16(scaleParams->current));
}

/** @brief Returns the $I_q$ current reference set to the motor represented by @p pHandle in Ampere using Q16 format. */
fixp16_t MCI_GetIqRef(const MCI_Handle_t* pHandle)
{
  return FIXP30_mpy(pHandle->pSTC->Idq_ref_pu.Q, FIXP16(scaleParams->current));
}

/** @brief Returns the $I_q$ current reference set by the speed controler to the motor represented by @p pHandle in Ampere using Q16 format. */
fixp16_t MCI_GetIqRefSpeed(const MCI_Handle_t* pHandle)
{
  return FIXP30_mpy(pHandle->pSTC->Iq_ref_spd_pu, FIXP16(scaleParams->current));
}

/** @brief Returns the maximum allowed current set on the motor represented by @p pHandle in Ampere using Q16 format. */
fixp16_t MCI_GetMaxCurrent(const MCI_Handle_t* pHandle)
{
  /* Convert per-unit maximum to _fixp16 Ampere */
  return FIXP30_mpy(pHandle->pSTC->I_max_pu, FIXP16(scaleParams->current));
}

/** @brief Returns the maximum modulation index on the motor represented by @p pHandle. */
fixp30_t MCI_GetMaxModulation(const MCI_Handle_t* pHandle)
{
  return pHandle->pCurrCtrl->maxModulation;
}

float_t MCI_GetPidCurrentKp(const MCI_Handle_t *pHandle)
{
  return PIDREGDQX_CURRENT_getKp_si(&pHandle->pCurrCtrl->pid_IdIqX_obj);
}

float_t MCI_GetPidCurrentWi(const MCI_Handle_t *pHandle)
{
  return PIDREGDQX_CURRENT_getWi_si(&pHandle->pCurrCtrl->pid_IdIqX_obj);
}

float_t MCI_GetPidSpeedKi(const MCI_Handle_t *pHandle)
{
  return PIDREG_SPEED_getKi_si(&pHandle->pSTC->PIDSpeed);
}

float_t MCI_GetPidSpeedKp(const MCI_Handle_t *pHandle)
{
  return PIDREG_SPEED_getKp_si(&pHandle->pSTC->PIDSpeed);
}

float_t MCI_GetPolePairs(const MCI_Handle_t *pHandle)
{
  return pHandle->pFOCVars->polePairs;
}

float_t MCI_GetProfilerFlux_Wb(const MCI_Handle_t *pHandle)
{
<#if MC.MOTOR_PROFILER ==true>  
  return PROFILER_getFlux_Wb(pHandle->pProfiler);
<#else>
  return 0.0f;
</#if>
}

float_t MCI_GetProfilerFluxEstFreq_Hz(const MCI_Handle_t* pHandle)
{
<#if MC.MOTOR_PROFILER ==true>  
  return PROFILER_getFluxEstFreq_Hz(pHandle->pProfiler);
<#else>
  return 0.0f;  
</#if> /* PROFILER_DISABLE */
}

float_t MCI_GetProfilerLd_H(const MCI_Handle_t *pHandle)
{
<#if MC.MOTOR_PROFILER ==true>  
  return PROFILER_getLd_H(pHandle->pProfiler);
<#else>
  return 0.0f;
</#if>
}

float_t MCI_GetProfilerPowerGoal_W(const MCI_Handle_t *pHandle)
{
<#if MC.MOTOR_PROFILER ==true>  
  return PROFILER_getPowerGoal_W(pHandle->pProfiler);
<#else>
  return 0.0f;
</#if>
}

float_t MCI_GetProfilerRs_DC(const MCI_Handle_t *pHandle)
{
<#if MC.MOTOR_PROFILER ==true>  
  return PROFILER_getRs_dc(pHandle->pProfiler);
<#else>
  return 0.0f;
</#if>
}

float_t MCI_GetProfilerRs_AC(const MCI_Handle_t *pHandle)
{
<#if MC.MOTOR_PROFILER ==true>  
  return PROFILER_getRs_ac(pHandle->pProfiler);
<#else>
  return 0.0f;
</#if>
}

uint8_t MCI_GetProfilerState(const MCI_Handle_t* pHandle)
{
<#if MC.MOTOR_PROFILER ==true>  
  return pHandle->pProfiler->state;
<#else>
  return 0;
</#if>  
}

fixp24_t MCI_GetRatedFlux(const MCI_Handle_t* pHandle)
{
  return pHandle->pFOCVars->Flux_Rated_VpHz;
}

fixp24_t MCI_GetRatedLs(const MCI_Handle_t* pHandle)
{
  return MCI_ConvertL_pu_fps_to_fixp24(pHandle, &pHandle->pFOCVars->Ls_Rated_pu_fps);
}

fixp20_t MCI_GetRatedRs(const MCI_Handle_t* pHandle)
{
<#if MC.M1_SPEED_SENSOR == "ZEST">
  /* Read from RsEst module, which is the authorative source for the rated resistance */
  FIXP_scaled_t rs_rated_pu_fps = RSEST_getRsRated_fps(pHandle->pSPD->pRsEst);
  return MCI_ConvertR_pu_fps_to_fixp20(pHandle, &rs_rated_pu_fps);
<#else>
  return pHandle->pFOCVars->Rs_Rated_Ohm;
</#if>
}

float MCI_GetSpeedRamp_F( const MCI_Handle_t * pHandle )
{
    float pu_hz_per_isr = FIXP30_toF(pHandle->pSTC->speed_ramp_pu_per_isr);
    float hz_per_isr = pu_hz_per_isr * scaleParams->frequency;
    float hz_elec_per_sec = hz_per_isr * TF_REGULATION_RATE;

    return hz_elec_per_sec;
}

fixp16_t MCI_GetSpeedRamp(  const MCI_Handle_t * pHandle )
{
    return FIXP16( MCI_GetSpeedRamp_F( pHandle ) );
}

// PU and electrical
fixp30_t MCI_GetSpeedRamp_PU( const MCI_Handle_t * pHandle )
{
    return ( pHandle->pSTC->speed_ramp_pu_per_isr );
}

/** 
  * @brief Sets the electrical rotation speed of the target motor (Per Unit version).
  *
  * If the motor's state machine is in the #RUN state, the electrical speed of the
  * motor is set to @p Speed, otherwise the function does nothing.
  *
  *  The new Speed is reached with the acceleration set thanks to the MCI_SetSpeedRamp(),
  * MCI_SetSpeedRamp_F() or MCI_SetSpeedRamp_PU() functions. If no acceleration was set, 
  * the default one, set in the application's parameters is used.
  *
  * The sign of the @p Speed parameter indicates the direction of the rotation.
  *
  * @sa MCI_SetSpeedReference_F, MCI_SetSpeedReference
  *
  * @param pHandle handle on the targer motor
  * @param  Speed Electrical rotor speed reference. Expressed in "Per Unit".
  */
void MCI_SetSpeedReference_PU( const MCI_Handle_t* pHandle, const fixp30_t Speed )
{
    pHandle->pSTC->speed_ref_pu = Speed;
    pHandle->pSTC->speed_ref_active_pu = pHandle->pSTC->speed_ref_pu;
}

/** 
  * @brief Sets the electrical rotation speed of the target motor (floating point version).
  *
  * If the motor's state machine is in the #RUN state, the electrical speed of the
  * motor is set to @p Speed, otherwise the function does nothing.
  *
  *  The new Speed is reached with the acceleration set thanks to the MCI_SetSpeedRamp(),
  * MCI_SetSpeedRamp_F() or MCI_SetSpeedRamp_PU() functions. If no acceleration was set, 
  * the default one, set in the application's parameters is used.
  *
  * The sign of the @p Speed parameter indicates the direction of the rotation.
  *
  * @sa MCI_SetSpeedReference, MCI_SetSpeedReference_PU
  *
  * @param pHandle handle on the targer motor
  * @param  Speed Electrical rotor speed reference. Expressed in Hz.
  */
void MCI_SetSpeedReference_F( const MCI_Handle_t* pHandle, const float Speed )
{
    pHandle->pSTC->speed_ref_pu = FIXP30( Speed / scaleParams->frequency );
    pHandle->pSTC->speed_ref_active_pu = pHandle->pSTC->speed_ref_pu;
}

/** 
  * @brief Sets the electrical rotation speed of the target motor.
  *
  * If the motor's state machine is in the #RUN state, the electrical speed of the
  * motor is set to @p Speed, otherwise the function does nothing.
  *
  *  The new Speed is reached with the acceleration set thanks to the MCI_SetSpeedRamp(),
  * MCI_SetSpeedRamp_F() or MCI_SetSpeedRamp_PU() functions. If no acceleration was set, 
  * the default one, set in the application's parameters is used.
  *
  * The sign of the @p Speed parameter indicates the direction of the rotation.
  *
  * @sa MCI_SetSpeedReference_F, MCI_SetSpeedReference_PU
  *
  * @param pHandle handle on the targer motor
  * @param  Speed Electrical rotor speed reference. Expressed in Hzn, using the Q16 format.
  */
void MCI_SetSpeedReference( const MCI_Handle_t* pHandle, const fixp16_t Speed )
{
    MCI_SetSpeedReference_F( pHandle, FIXP16_toF(Speed) );
}

// Per Unit and Electrical
fixp30_t MCI_GetSpeedReference_PU( const MCI_Handle_t* pHandle )
{
    return pHandle->pSTC->speed_ref_pu;
}

float MCI_GetSpeedReference_F( const MCI_Handle_t* pHandle )
{
    return FIXP30_toF(pHandle->pSTC->speed_ref_pu) * scaleParams->frequency;
}

fixp16_t MCI_GetSpeedReference( const MCI_Handle_t* pHandle )
{
    return FIXP16( MCI_GetSpeedReference_F( pHandle ) );
}

// PU and electrical
fixp30_t MCI_GetAvrgSpeed_PU( MCI_Handle_t * pHandle )
{
    return HSO_getSpeedLP_pu(pHandle->pSPD->pHSO);
}

float MCI_GetAvrgSpeed_F( MCI_Handle_t * pHandle )
{
  fixp30_t speed =  HSO_getSpeedLP_pu(pHandle->pSPD->pHSO);
  
  return FIXP30_toF(speed) * scaleParams->frequency;
}

fixp16_t MCI_GetAvrgSpeed( MCI_Handle_t * pHandle )
{
    return FIXP16( MCI_GetAvrgSpeed_F( pHandle ) );
}

float MCI_GetEmfMagnitude_F( MCI_Handle_t * pHandle )
{
    return ( (FIXP30_toF(pHandle->pSPD->EmfMag_pu) * scaleParams->voltage) / motorParams->ratedFlux );
}

// PU
fixp30_t MCI_GetEmfMagnitude_PU( MCI_Handle_t * pHandle )
{
    return ( pHandle->pSPD->EmfMag_pu );
}

void MCI_SetFlagFreezeTrackAngle( MCI_Handle_t * pHandle, bool value )
{
  pHandle->pSPD->flagFreezeHsoAngle = value;
}

// PU and per pole
void MCI_SetTorqueReference_PU( const MCI_Handle_t *pHandle, fixp30_t Torque )
{
    pHandle->pSTC->Idq_ref_pu.Q = Torque;
}

//
void MCI_SetTorqueReference_F( const MCI_Handle_t *pHandle, float Torque )
{
    float iqref = Torque * pHandle->pFOCVars->Kt;  /* Kt in Ampere/Nm */
    pHandle->pSTC->Idq_ref_pu.Q = FIXP30(iqref / scaleParams->current);
}

void MCI_SetTorqueReference( const MCI_Handle_t *pHandle, fixp16_t Torque )
{
    MCI_SetTorqueReference_F( pHandle, FIXP16_toF( Torque ) );
}

// PU & per pole
fixp30_t MCI_GetTorqueReference_PU( const MCI_Handle_t *pHandle )
{
    return ( pHandle->pSTC->Idq_ref_pu.Q );
}

float MCI_GetTorqueReference_F( const MCI_Handle_t *pHandle )
{
    fixp30_t torque_pp_pu = FIXP30_mpy( pHandle->pSTC->Idq_ref_pu.Q, HSO_getFluxRef_Wb( pHandle->pSPD->pHSO ) );

    return FIXP30_toF( torque_pp_pu ) * 1.5f * scaleParams->current * pHandle->pFOCVars->polePairs;
}

fixp16_t MCI_GetTorqueReference( const MCI_Handle_t *pHandle )
{
    return FIXP16( MCI_GetTorqueReference_F( pHandle ) );
}

fixp16_t MCI_GetTorque_Nm(const MCI_Handle_t *pHandle)
{
    fixp16_t torque_Nm;

    Vector_ab_t Iab_in_pu = pHandle->pPWM->Iab_in_pu;
    Vector_ab_t flux_ab_Wb = HSO_getFlux_ab_Wb(pHandle->pSPD->pHSO);
    fixp24_t Torque_pp_pu = FIXP30_toFIXP(FIXP30_mpy(Iab_in_pu.B, flux_ab_Wb.A) - FIXP30_mpy(Iab_in_pu.A, flux_ab_Wb.B));

    torque_Nm = FIXP24_mpy(Torque_pp_pu, FIXP16(1.5f * scaleParams->current * pHandle->pFOCVars->polePairs)); // 16 = 24 x 16 >> 24

    return torque_Nm;
}

float MCI_GetTorque_Nm_F( const MCI_Handle_t *pHandle )
{
    Vector_ab_t Iab_in_pu = pHandle->pPWM->Iab_in_pu;
    Vector_ab_t flux_ab_Wb = HSO_getFlux_ab_Wb(pHandle->pSPD->pHSO);
    float Torque_pp_pu = FIXP30_toF(FIXP30_mpy(Iab_in_pu.B, flux_ab_Wb.A) - FIXP30_mpy(Iab_in_pu.A, flux_ab_Wb.B));

    return (1.5f * Torque_pp_pu * scaleParams->current * pHandle->pFOCVars->polePairs);
}

// PU and per pole
fixp30_t MCI_GetTorque_PU( const MCI_Handle_t *pHandle )
{
    Vector_ab_t Iab_in_pu = pHandle->pPWM->Iab_in_pu;
    Vector_ab_t flux_ab_Wb = HSO_getFlux_ab_Wb(pHandle->pSPD->pHSO);

    return ( FIXP30_mpy(Iab_in_pu.B, flux_ab_Wb.A) - FIXP30_mpy(Iab_in_pu.A, flux_ab_Wb.B) );
}

void MCI_SetActiveLs(MCI_Handle_t* pHandle, const fixp24_t ls_henry)
{
  pHandle->pFOCVars->Ls_Active_H = ls_henry;
  MCI_ConvertL_fixp24_to_pu_fps(pHandle, ls_henry, &pHandle->pSPD->Ls_Active_pu_fps);
}

void MCI_SetActiveRs(MCI_Handle_t* pHandle, const fixp20_t rs_ohm)
{
  /* Set active value in RsEst (Actual active value in ImpedCorr is updated indirectly only) */ 
<#if MC.M1_SPEED_SENSOR == "ZEST">
  float rs_ohm_flt = FIXP20_toF(rs_ohm);
  RSEST_setRsOhm(pHandle->pSPD->pRsEst, rs_ohm_flt);
</#if>
}

void MCI_SetIdRef(MCI_Handle_t* pHandle, const fixp16_t id_ref_A)
{
  /* Set user's D-current reference */

  /* Convert from _fixp16 A to fixp30_t _pu_A */
  fixp30_t Id_ref_pu = FIXP16_mpy(id_ref_A, FIXP30(1.0f / scaleParams->current));
  fixp30_t I_max_pu = pHandle->pSTC->I_max_pu;
  Id_ref_pu = FIXP_sat(Id_ref_pu, I_max_pu, -I_max_pu);
  pHandle->pSTC->Idq_ref_pu.D = Id_ref_pu;
}

void MCI_SetIqRef(MCI_Handle_t* pHandle, const fixp16_t iq_ref_A)
{
  /* Set user's Q-current reference */

  /* Convert from _fixp16 A to fixp30_t _pu_A */
  fixp30_t Iq_ref_pu = FIXP16_mpy(iq_ref_A, FIXP30(1.0f / scaleParams->current));
  fixp30_t I_max_pu = pHandle->pSTC->I_max_pu;
  Iq_ref_pu = FIXP_sat(Iq_ref_pu, I_max_pu, -I_max_pu);
  pHandle->pSTC->Idq_ref_pu.Q = Iq_ref_pu;
}

void MCI_SetMaxCurrent(MCI_Handle_t* pHandle, const fixp16_t I_max_A)
{
  fixp16_t I_max_fixp16;
  /* Limit the current to the allowable global maximum */
  if (MCM_PROFILING_MODE == MCI_GetFocControlMode(pHandle) || (MCI_GetProfilerState(pHandle) == PROFILER_STATE_Complete))
  {
    /* during and after profiling limit to board maximum current */
    I_max_fixp16 = FIXP_sat(I_max_A, FIXP16(M1_MAX_READABLE_CURRENT), FIXP16(0.0f));  
  }
  else
  {
    I_max_fixp16 = FIXP_sat(I_max_A, FIXP16(NOMINAL_CURRENT_A), FIXP16(0.0f));
  }
  /* Convert to per unit */
  pHandle->pSTC->I_max_pu = FIXP16_mpy(I_max_fixp16, FIXP30(1.0f / scaleParams->current));
<#if MC.M1_SPEED_SENSOR == "ZEST"> 
  ZEST_setThresholdResistanceCurrent_A(pHandle->pSPD->pZeST, 0.1f * FIXP16_toF(I_max_fixp16));
</#if>
<#if MC.MOTOR_PROFILER ==true>  
    PROFILER_DCAC_setIdDCMax(&pHandle->pProfiler->dcacObj, FIXP16_toF(I_max_fixp16));
</#if>
  pHandle->pSTC->OneOverMaxIq_pu = FIXP24(1.0f / FIXP30_toF(pHandle->pSTC->I_max_pu));

}

fixp24_t MCI_GetOneOverMaxIq(MCI_Handle_t* pHandle)
{
  return (pHandle->pSTC->OneOverMaxIq_pu);
}


void MCI_SetMaxModulation(MCI_Handle_t* pHandle, const fixp30_t maxModulation)
{
  /* 1.15f = (a bit less than) maximum duty using space vector modulation, 2/sqrt(3) */

  fixp30_t ModulationLimit = FIXP30(BOARD_MAX_MODULATION);

  if (pHandle->pPWM->modulationMode == FOC_MODULATIONMODE_Sine)
  {
    ModulationLimit = FIXP30(0.999f);
  }

  fixp30_t DutyLimit = FIXP_sat(maxModulation, ModulationLimit, FIXP30(0.0f));
  pHandle->pCurrCtrl->maxModulation = DutyLimit;

  /* set current regulator limits */
  float_t DutyLimit_f = FIXP30_toF(DutyLimit);
  PIDREGDQX_CURRENT_setOutputLimitsD(&pHandle->pCurrCtrl->pid_IdIqX_obj, FIXP30(DutyLimit_f*0.95f), FIXP30(-DutyLimit_f*0.95f));
  PIDREGDQX_CURRENT_setOutputLimitsQ(&pHandle->pCurrCtrl->pid_IdIqX_obj, FIXP30(DutyLimit_f), FIXP30(-DutyLimit_f));
  PIDREGDQX_CURRENT_setMaxModulation_squared(&pHandle->pCurrCtrl->pid_IdIqX_obj, DutyLimit_f);
}

void MCI_SetPidCurrentKp(MCI_Handle_t *pHandle, const float_t kp_si)
{
  PIDREGDQX_CURRENT_setKp_si(&pHandle->pCurrCtrl->pid_IdIqX_obj, kp_si);
}

void MCI_SetPidCurrentWi(MCI_Handle_t *pHandle, const float_t wi_si)
{
  PIDREGDQX_CURRENT_setWi_si(&pHandle->pCurrCtrl->pid_IdIqX_obj, wi_si);
}

void MCI_SetPidSpeedKi(MCI_Handle_t *pHandle, const float_t ki_si)
{
  PIDREG_SPEED_setKi_si(&pHandle->pSTC->PIDSpeed, ki_si);
}

void MCI_SetPidSpeedKp(MCI_Handle_t *pHandle, const float_t kp_si)
{
  PIDREG_SPEED_setKp_si(&pHandle->pSTC->PIDSpeed, kp_si);
}

void MCI_SetPolePairs(MCI_Handle_t *pHandle, const float_t pp)
{
  /* Gearbox on motor may result in non-integer polepair count being convenient */
  pHandle->pFOCVars->polePairs = pp;
<#if MC.MOTOR_PROFILER ==true>  
  PROFILER_setPolePairs(pHandle->pProfiler, pp);
</#if>
}

void MCI_SetProfilerCommand(MCI_Handle_t* pHandle, const uint8_t command)
{
<#if MC.MOTOR_PROFILER ==true>  
  //Clear Zest params
  if ((PROFILER_Command_e) command == PROFILER_COMMAND_Start)
  {
      //store cuurrent Zest params
<#if MC.M1_SPEED_SENSOR == "ZEST">   
      pHandle->pProfiler->injectFreq_HzZest = pHandle->pSPD->zestControl.injectFreq_kHz;
      pHandle->pProfiler->injectIdZest = pHandle->pSPD->zestControl.injectId_A_pu;
      pHandle->pProfiler->gainDZest = pHandle->pSPD->zestControl.feedbackGainD;
      pHandle->pProfiler->gainQZest = pHandle->pSPD->zestControl.feedbackGainQ;

      pHandle->pSPD->zestControl.injectFreq_kHz = FIXP30(80.0f / 1000.0f);
      pHandle->pSPD->zestControl.feedbackGainD = FIXP30(0.0f / scaleParams->current);
      pHandle->pSPD->zestControl.feedbackGainQ = FIXP30(0.0f / scaleParams->current);
      pHandle->pSPD->zestControl.injectId_A_pu = FIXP30(1.0f / scaleParams->current);
  </#if>
  }
  PROFILER_setCommand(pHandle->pProfiler, (PROFILER_Command_e) command);
  if ((PROFILER_Command_e) command == PROFILER_COMMAND_Reset)
  {
    // ToDo: Profiler has been reset, now ask motor control to reset itself (re-initialize) to a known good state
      // This is too complex for profiler to handle

  }
</#if>
}

void MCI_SetProfilerFluxEstFreq_Hz(MCI_Handle_t* pHandle, const float_t fluxEstFreq_Hz)
{
<#if MC.MOTOR_PROFILER ==true>  
  PROFILER_setFluxEstFreq_Hz(pHandle->pProfiler, fluxEstFreq_Hz);
</#if>
}

void MCI_SetProfilerPowerGoal_W(MCI_Handle_t* pHandle, const float_t powerGoal_W)
{
<#if MC.MOTOR_PROFILER ==true>  
  PROFILER_setPowerGoal_W(pHandle->pProfiler, powerGoal_W);
</#if>
}

void MCI_SetRatedFlux(MCI_Handle_t* pHandle, const fixp24_t flux_VpHz)
{
  float_t flux_Wb = FIXP24_toF(flux_VpHz) / M_TWOPI;
  HSO_setFluxRef_Wb(pHandle->pSPD->pHSO, FIXP30(flux_Wb));
  pHandle->pFOCVars->Flux_Rated_VpHz = flux_VpHz;
}

void MCI_SetRatedLs(MCI_Handle_t* pHandle, const fixp24_t ls_henry)
{
  pHandle->pFOCVars->Ls_Rated_H = ls_henry;
  IMPEDCORR_setLs_si(pHandle->pSPD->pImpedCorr, FIXP24_toF(ls_henry));
  MCI_ConvertL_fixp24_to_pu_fps(pHandle, ls_henry, &pHandle->pFOCVars->Ls_Rated_pu_fps);
}

void MCI_SetRatedRs(MCI_Handle_t* pHandle, const fixp20_t rs_ohm)
{
  /* Set rated value in RsEst, which is the authorative source for this value */
  float_t rs_ohm_flt = FIXP20_toF(rs_ohm);
  IMPEDCORR_setRs_si(pHandle->pSPD->pImpedCorr, rs_ohm_flt);
<#if MC.M1_SPEED_SENSOR == "ZEST">  
  RSEST_setRsRatedOhm(pHandle->pSPD->pRsEst, rs_ohm_flt);
</#if>  
}

void MCI_SetSpeedRamp_F( MCI_Handle_t* pHandle, const float RampHzPerSec )
{
    float hz_per_isr = RampHzPerSec / TF_REGULATION_RATE;
    float pu_hz_per_isr = hz_per_isr / scaleParams->frequency;

    pHandle->pSTC->speed_ramp_pu_per_isr = FIXP30(pu_hz_per_isr);
}

void MCI_SetSpeedRamp(MCI_Handle_t* pHandle, const fixp16_t RampHzPerSec )
{
    /* fixp16_t gives a range of 2^15 => 32768 Hz/s,
     * and a resolution of 2^-16 =~ 0.0000152 Hz/sec */

    MCI_SetSpeedRamp_F( pHandle, FIXP16_toF(RampHzPerSec) );
}

/// PU and electrical
void MCI_SetSpeedRamp_PU( MCI_Handle_t* pHandle, const fixp30_t Acceleration_pu )
{
    pHandle->pSTC->speed_ramp_pu_per_isr = Acceleration_pu;
}

bool MCI_StopRamp(MCI_Handle_t* pHandle)
{
  bool retval;

  switch (MCI_GetControlMode(pHandle))
  {
    case MCM_SPEED_MODE:
      pHandle->pSTC->speed_ref_active_pu = pHandle->pSTC->speed_ref_ramped_pu;
      pHandle->pSTC->speed_ref_pu = pHandle->pSTC->speed_ref_ramped_pu;
      break;

    case MCM_TORQUE_MODE:
      /*todo*/
      retval = false;
      break;

    default:
      retval = false;
      break;
  }
  return retval;
}

void MCI_SetMaxAcceleration(MCI_Handle_t* pHandle, fixp16_t Acceleration)
{
  float acc;
  
  acc = FIXP16_toF(Acceleration);
  MCI_SetMaxAcceleration_F(pHandle, acc);
}


void MCI_SetMaxAcceleration_F(MCI_Handle_t* pHandle, float Acceleration)
{
  float acc;
  
  acc = Acceleration/scaleParams->frequency;
  MCI_SetMaxAcceleration_PU(pHandle, FIXP30(acc));
}

void MCI_SetMaxAcceleration_PU(MCI_Handle_t* pHandle, fixp30_t Acceleration)
{
  float acc = FIXP30_toF(Acceleration)/TF_REGULATION_RATE;
  pHandle->pSTC->max_accel_pu_per_isr = FIXP30(acc);
}

fixp16_t MCI_GetMaxAcceleration(MCI_Handle_t* pHandle)
{
  float acc;

  acc = MCI_GetMaxAcceleration_F(pHandle);
  return FIXP16(acc);
}

float  MCI_GetMaxAcceleration_F(MCI_Handle_t* pHandle)
{
  float acc;

  acc = FIXP30_toF(MCI_GetMaxAcceleration_PU(pHandle))*scaleParams->frequency;
  return acc;
}

fixp30_t MCI_GetMaxAcceleration_PU(MCI_Handle_t* pHandle)
{
  float acc;

  acc = FIXP30_toF(pHandle->pSTC->max_accel_pu_per_isr)*TF_REGULATION_RATE;
  return FIXP30(acc);
}

void MCI_SetCurrentPIDByRsLs(MCI_Handle_t* pHandle, const float_t rs, const float_t ls)
{
  /* PID Controllers, maximum duty */
  float foc_frequency_hz = (PWM_FREQUENCY / REGULATION_EXECUTION_RATE);
  float duty_scale = 2.0f; /* PWM duty ranges from -1 to +1, so the scale is 2.0 */

  /* Calculate current controller default PID parameters, based on Rs and Ls */
  float kp_idq = 0.8f * (0.25f * ls * foc_frequency_hz * duty_scale);	/* unit V/A somewhat lower to allow for unoptimized delay by oversampling*/
  float wi_idq = (rs / ls);					/* unit rad/s */

  PIDREGDQX_CURRENT_setKp_si(&pHandle->pCurrCtrl->pid_IdIqX_obj, kp_idq);
  PIDREGDQX_CURRENT_setWi_si(&pHandle->pCurrCtrl->pid_IdIqX_obj, wi_idq);
}

PolarizationState_t MCI_GetOffsetsCalibrationState( MCI_Handle_t* pHandle )
{
  return (pHandle->pFOCVars->PolarizationState);
}

MC_RetStatus_t MCI_GetCalibratedOffsetsMotor(MCI_Handle_t* pHandle, PolarizationOffsets_t * PolarizationOffsets)
{
  uint32_t RetVal = MC_NO_POLARIZATION_OFFSETS_ERROR;

  if ( MCI_GetOffsetsCalibrationState(pHandle) ==  COMPLETED ) {
    PWMC_GetOffsets(pHandle->pPWM, PolarizationOffsets);
    RetVal = MC_SUCCESS;
  }

  return(RetVal);
}

bool MCI_SetCalibratedOffsetsMotor( MCI_Handle_t* pHandle, PolarizationOffsets_t * PolarizationOffsets)
{
  bool RetVal = false;

  if ( IDLE == pHandle->State)
  {
      PWMC_SetOffsets(pHandle->pPWM, PolarizationOffsets);
      pHandle->pFOCVars->PolarizationState = COMPLETED;
      RetVal = true;
  }

    return(RetVal);
}

bool MCI_StartOffsetsCalibrationMotor(MCI_Handle_t* pHandle)
{
   uint32_t RetVal = false;

   if (IDLE == pHandle->State)
   {
     pHandle->State = OFFSET_CALIB;
     pHandle->pFOCVars->PolarizationState = NOT_DONE;
     RetVal = true;
   }

   return(RetVal);

}

void MCI_SetMinHSOMinCrossoverFreqHz(MCI_Handle_t* pHandle, float frequency)
{
  HSO_setMinCrossOver_Hz(pHandle->pSPD->pHSO, frequency);
}

<#if MC.M1_SPEED_SENSOR == "ZEST">  
void MCI_SetAltMinHSOMinCrossoverFreqHz(MCI_Handle_t* pHandle, float frequency)
{
  SPD_SetMinCrossOverAtLowSpeed_Hz(pHandle->pSPD, frequency);
}
</#if>

void MCI_SetHSOCrossoverFreqHz(MCI_Handle_t* pHandle, float frequency)
{
  HSO_setCrossOver_Hz(pHandle->pSPD->pHSO, frequency);
}

void MCI_SetRsDCEstimationEnable(MCI_Handle_t* pHandle)
{
  pHandle->pRsDCEst->flag_enableRSDCestimate = true;
<#if MC.M1_SPEED_SENSOR == "ZEST">
  if (pHandle->pPolPulse->flagPolePulseActivation == true)
  {
    pHandle->pRsDCEst->RSDCestimate_Fast = true;
  }
  else {
    pHandle->pRsDCEst->RSDCestimate_Fast = false;
  }
</#if>      
}

void MCI_SetRsDCEstimationDisable(MCI_Handle_t* pHandle)
{
  pHandle->pRsDCEst->flag_enableRSDCestimate = false;
<#if MC.M1_SPEED_SENSOR == "ZEST">
  pHandle->pRsDCEst->RSDCestimate_Fast = false;
</#if>
}

void MCI_SetRsDCEstimationAlignOnly(MCI_Handle_t* pHandle, bool value)
{
  pHandle->pRsDCEst->RSDCestimate_AlignOnly = value;
}

RsDC_estimationState_t MCI_GetRsDCEstimationState(MCI_Handle_t* pHandle)
{
  return pHandle->pRsDCEst->RSDCestimate_state;
}

void MCI_EnablePulse(MCI_Handle_t* pHandle)
{
  pHandle->pPolPulse->flagPolePulseActivation = true;
<#if MC.M1_SPEED_SENSOR == "ZEST">  
  if (pHandle->pRsDCEst->flag_enableRSDCestimate == true)
  {
    pHandle->pRsDCEst->RSDCestimate_Fast = true;
  }
  else {
    pHandle->pRsDCEst->RSDCestimate_Fast = false;
  }
</#if>  
}

void MCI_DisablePulse(MCI_Handle_t* pHandle)
{
  pHandle->pPolPulse->flagPolePulseActivation = false;
<#if MC.M1_SPEED_SENSOR == "ZEST">
  pHandle->pRsDCEst->RSDCestimate_Fast = false;
</#if>  
}

void MCI_SetPulsePeriods(MCI_Handle_t *pHandle, const uint16_t N)
{
  POLPULSE_setPulsePeriods(&pHandle->pPolPulse->PolpulseObj, N);
}

bool MCI_IsPulseBusy(MCI_Handle_t *pHandle)
{
  return POLPULSE_isBusy(&pHandle->pPolPulse->PolpulseObj);
}

void MCI_SetDecayDuration(MCI_Handle_t *pHandle, const uint16_t Nd)
{
  POLPULSE_setDecayPeriods(&pHandle->pPolPulse->PolpulseObj, Nd);
}

void MCI_SetPulseCurrentGoal(MCI_Handle_t *pHandle, float PulseCurrentGoal_A)
{
<#if MC.MOTOR_PROFILER == false>
<#-- set clamping when motor profiler is not used --> 
  PulseCurrentGoal_A = (PulseCurrentGoal_A > (float_t)BOARD_SOFT_OVERCURRENT_TRIP) ? (float_t)BOARD_SOFT_OVERCURRENT_TRIP : PulseCurrentGoal_A;
</#if>
  POLPULSE_setCurrentGoal(&pHandle->pPolPulse->PolpulseObj, PulseCurrentGoal_A);
}

uint16_t MCI_GetPulsePeriods(MCI_Handle_t *pHandle)
{
  return (POLPULSE_getPulsePeriods(&pHandle->pPolPulse->PolpulseObj));
}

uint16_t MCI_GetDecayDuration(MCI_Handle_t *pHandle)
{
  return(POLPULSE_getDecayPeriods(&pHandle->pPolPulse->PolpulseObj));
}

float MCI_GetPulseCurrentGoal( const MCI_Handle_t *pHandle )
{
   return(POLPULSE_getCurrentGoal(&pHandle->pPolPulse->PolpulseObj));
}

fixp30_t MCI_GetPulseDuty( const MCI_Handle_t *pHandle )
{
   return(POLPULSE_getPulseDuty(&pHandle->pPolPulse->PolpulseObj));
}

fixp30_t MCI_GetPulseAngleEstimated( const MCI_Handle_t *pHandle )
{
   return(POLPULSE_getAngleEstimated(&pHandle->pPolPulse->PolpulseObj));
}

/**
  * @brief  It re-initializes Iqdref variables with their default values.
  * @param  pHandle Pointer on the component instance to work on.
  */
__weak void MCI_Clear_Iqdref(MCI_Handle_t *pHandle)
{
  pHandle->pSTC->Idq_ref_pu.D = 0;
  pHandle->pSTC->Idq_ref_pu.Q = 0;
}

<#if MC.DBG_MCU_LOAD_MEASURE == true>
__weak void MCI_Clear_PerfMeasure(MCI_Handle_t *pHandle, uint8_t bMotor)
{
  MC_Perf_Clear(pHandle->pPerfMeasure,bMotor);
}
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

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/


