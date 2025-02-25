<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
 ******************************************************************************
 * @file    speed_pos_fdbk_hso.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the  features
 *          of the Speed & Position Feedback HSO component of the Motor Control 
 *          SDK.
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
  * @ingroup speed_pos_fdbk_hso
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk_hso.h"
#include "mc_config.h"
#include "mc_parameters.h"
#include "parameters_conversion.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @defgroup speed_pos_fdbk_hso High Sensitivity Observer Speed & position feedback 
  *
  * @brief Speed & Position feedback HSO component of the Motor Control SDK
  *
  * This component provides the speed and the angular position of the rotor based
  * on a sensorless approach.
  * 
  * By considering a perfect measurements of three phase currents and voltages with knowledge
  * of stator resistance Rs and inductance Ls the back-emf can be estimated. 
  * The electrical angle is then provided by the High Sensitivity Observer after an estimation of
  * the flux based on the bak-emf. 
  * 
  * ![Speed and position feeback](speed_position_fdbk.svg)
  *
  * @{
  */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */

/**
  * @brief  Initializes speed & position control component.
  *         It should be called during Motor control middleware initialization
  * @param  pHandle: speed & position feedback control handler
  * @param  flashParams: flash parameters
  */
void SPD_Init(SPD_Handle_t *pHandle, FLASH_Params_t const *flashParams)
{

  /* Initialize the objects which return a Handle here */
  pHandle->pImpedCorr = IMPEDCORR_init(&ImpedCorr_M1, sizeof(ImpedCorr_M1));
  pHandle->pHSO = HSO_init(&HSO_M1, sizeof(HSO_M1));
<#if MC.M1_SPEED_SENSOR == "ZEST">    
  pHandle->pZeST = ZEST_init(&ZeST_M1, sizeof(ZeST_M1));
  pHandle->pRsEst = RSEST_init(&RsEst_M1, sizeof(RsEst_M1));
</#if> 

<#if MC.M1_SPEED_SENSOR == "ZEST">  
  if (pHandle->pImpedCorr == 0 ||
      pHandle->pHSO == 0 ||
      pHandle->pZeST == 0 ||
      pHandle->pRsEst == 0)
<#else>
  if (pHandle->pImpedCorr == 0 ||
      pHandle->pHSO == 0 )
</#if>      
  {
    /* Handle initialization failed, better to spin here than to crash later */
    for(;;);
  }

  /* Impedance correction */
  pHandle->flagDisableImpedanceCorrection = false;

  ImpedCorr_params_M1.fullScaleFreq_Hz = flashParams->scale.frequency;
  ImpedCorr_params_M1.KsampleDelay = flashParams->KSampleDelay;
  ImpedCorr_params_M1.fullScaleCurrent_A = flashParams->scale.current;
  ImpedCorr_params_M1.fullScaleVoltage_V = flashParams->scale.voltage;

  IMPEDCORR_setParams(pHandle->pImpedCorr, &ImpedCorr_params_M1);
  IMPEDCORR_setRs_si(pHandle->pImpedCorr, flashParams->motor.rs);
  IMPEDCORR_setLs_si(pHandle->pImpedCorr, flashParams->motor.ls);

  /* High Sensitivity Observer */
  Hso_params_M1.Flux_Wb = (double)flashParams->motor.ratedFlux / M_TWOPI;
  Hso_params_M1.FullScaleVoltage_V = flashParams->scale.voltage;
  Hso_params_M1.FullScaleFreq_Hz = flashParams->scale.frequency;
  HSO_setParams(pHandle->pHSO, &Hso_params_M1);
  pHandle->DirCheck_threshold_pu = FIXP30(4.0f / Hso_params_M1.FullScaleFreq_Hz);
  
<#if MC.M1_SPEED_SENSOR == "ZEST">  
 /* set alternate min cross-over frequency to use when Zest fraction is abobe 80% */
  SPD_SetMinCrossOverAtLowSpeed_Hz(pHandle, 10.0f);
  /* Disable offset tracking, to be reactivated when required */
  HSO_setFlag_EnableOffsetUpdate(pHandle->pHSO, false);

  /* Initialize the structures used to control ZEST */
  FOC_ZESTControl_s* pControl = &pHandle->zestControl;
  pControl->injectFreq_kHz = FIXP30(zestParams->zestInjectFreq / 1000.0f);
  pControl->injectId_A_pu = FIXP30(zestParams->zestInjectD / flashParams->scale.current);
  pControl->injectIq_A_pu = FIXP30(zestParams->zestInjectQ / flashParams->scale.current);
  pControl->feedbackGainD = FIXP24(zestParams->zestGainD);
  pControl->feedbackGainQ = FIXP24(zestParams->zestGainQ);
  pControl->CurrentLimitToSwitchZestConfig_pu = FIXP30((float_t)(MOTOR_PERCENTAGE_MAX_CURRENT * flashParams->motor.maxCurrent) / CURRENT_SCALE);

  Zest_params_M1.thresholdFreq_Hz = flashParams->zest.zestThresholdFreqHz; /* Injection active below this frequency */
  Zest_params_M1.backgroundFreq_Hz = SPEED_LOOP_FREQUENCY_HZ; /* interval for ZEST_runBackground() */
  Zest_params_M1.fullScaleFreq_Hz = flashParams->scale.frequency;
  Zest_params_M1.fullScaleCurrent_A = flashParams->scale.current;
  Zest_params_M1.fullScaleVoltage_V = flashParams->scale.voltage;
  Zest_params_M1.isrFreq_Hz = (PWM_FREQUENCY / REGULATION_EXECUTION_RATE); /* Frequency of ZEST_run() calls */
  Zest_params_M1.thresholdInjectActiveCurrent_A = (0.005f * flashParams->motor.maxCurrent); // 0.5% of max motor current used for minimal injection
  Zest_params_M1.thresholdResistanceCurrent_A = (0.1f * flashParams->motor.maxCurrent);	// 10% of max motor current
  Zest_params_M1.speedPole_rps = SPEED_POLE_RPS;

  /* Set flux for ZEST */
  // ToDo: conversion to pu flux should be done somewhere else
  // ToDo: FluxScale in ZEST should be updated at boot AND after profiler
  float_t FullScaleFlux = flashParams->scale.voltage / (PWM_FREQUENCY / REGULATION_EXECUTION_RATE);
  float_t RatedFlux_pu = flashParams->motor.ratedFlux / FullScaleFlux;
  float_t oneOverRatedFlux_pu = 1.0f / RatedFlux_pu;
  FIXPSCALED_floatToFIXPscaled(oneOverRatedFlux_pu, &pHandle->oneOverFlux_pu_fps);
  ZEST_setParams(pHandle->pZeST, &Zest_params_M1);

  pHandle->Idq_in_pu.D = FIXP30(0.0f);
  pHandle->Idq_in_pu.Q = FIXP30(0.0f);

  /* Call Update to pass all these settings to the ZEST object */
  MC_ZEST_Control_Update(pHandle);

  Rsest_params_M1.FullScaleCurrent_A = flashParams->scale.current;
  Rsest_params_M1.FullScaleVoltage_V = flashParams->scale.voltage;
  Rsest_params_M1.RsRatedOhm         = flashParams->motor.rs;
  Rsest_params_M1.m_copper_kg        = flashParams->motor.mass_copper_kg;
  Rsest_params_M1.coolingTau_s       = flashParams->motor.cooling_tau_s; //Thermal time constant of copper+direct environment: estimated from measurements
  Rsest_params_M1.MotorSkinFactor    = flashParams->motor.rsSkinFactor;

  RSEST_setParams(pHandle->pRsEst, &Rsest_params_M1);
  pHandle->Rs_update_select = FOC_Rs_update_Auto;

</#if>
  pHandle->AngleCorrection_pu = FIXP30(0.0f);

  pHandle->Hz_pu_to_step_per_isr = FIXP30(flashParams->scale.frequency / TF_REGULATION_RATE);

  pHandle->OpenLoopAngle = FIXP30(0.0f);

  float_t fs_l = (flashParams->scale.voltage / flashParams->scale.current / (float_t)VOLTAGE_FILTER_POLE_RPS);
  float_t ls_pu_flt = flashParams->motor.ls / fs_l;
  FIXPSCALED_floatToFIXPscaled(ls_pu_flt, &pHandle->Ls_Active_pu_fps);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Runs speed & position control component.
  *         It should be called during Motor control middleware initialization
  * @param  pHandle: speed & position feedback control handler
  * @param  In: input data structure
  * @param  Out: output data structure
  */
void SPD_Run(SPD_Handle_t *pHandle, Sensorless_Input_t *In, Sensorless_Output_t *Out)
{

  /* Impedance correction ***************************************************************************/

  /* Update the Rs and Ls used in the impedance correction */
  if (!pHandle->flagDisableImpedanceCorrection)
  {
<#if MC.M1_SPEED_SENSOR == "ZEST">     
    // ToDo: ImpedCorr setters taking 2 &fps parameters
    {
      // ToDo: rsestObj.rs_pu_fps is written in MFT, but read in HFT - This can lead to inconsistent data being read (value and fixpFmt do not belong together)
      FIXP_scaled_t rs_pu_fps = RSEST_getRs_pu_fps(pHandle->pRsEst);
      IMPEDCORR_setRs(pHandle->pImpedCorr, rs_pu_fps.value, rs_pu_fps.fixpFmt);
    }
</#if>          
    IMPEDCORR_setLs(pHandle->pImpedCorr, pHandle->Ls_Active_pu_fps.value, pHandle->Ls_Active_pu_fps.fixpFmt);
  }

  /* Use the previous cycle's estimated electrical frequency to perform the impedance correction */
  fixp30_t w_corr_pu = 0;
  if (true == pHandle->closedLoopAngleEnabled)
  {
    w_corr_pu = HSO_getEmfSpeed_pu(pHandle->pHSO);
  }

  Out->Uab_emf_pu = IMPEDCORR_run(pHandle->pImpedCorr, &In->Iab_pu, &In->Uab_pu, w_corr_pu);

  /* Store EMF's magnitude */
  pHandle->EmfMag_pu = FIXP_mag(Out->Uab_emf_pu.A, Out->Uab_emf_pu.B);

<#if MC.M1_SPEED_SENSOR == "ZEST">   
  /* Resistance estimation **************************************************************************/
  Currents_Iab_t Iab_filt_pu;
  IMPEDCORR_getIab_filt_pu(pHandle->pImpedCorr, &Iab_filt_pu);
  RSEST_run(pHandle->pRsEst, &Iab_filt_pu, &In->Uab_pu);
</#if>

  /* High Sensitivity Observer (HSO) **********************************************************************/
  HSO_Handle handleHso = pHandle->pHSO;

  fixp30_t correction_angle_pu = FIXP30(0.0f);
<#if MC.M1_SPEED_SENSOR == "ZEST">     
  if (pHandle->zestControl.enableCorrection)
  {
    correction_angle_pu = pHandle->AngleCorrection_pu;
  }
</#if>

  if ( (pHandle->flagHsoAngleEqualsOpenLoopAngle == true) || (pHandle->flagFreezeHsoAngle == true) )
  {
    HSO_setFlag_TrackAngle(handleHso, false);
    if (pHandle->flagHsoAngleEqualsOpenLoopAngle == true)
    {
      /* copy the OPENLOOP angle for ZEST identification procedure */
      HSO_setAngle_pu(handleHso, pHandle->OpenLoopAngle); 
    }
  }
  else
  {
    HSO_setFlag_TrackAngle(handleHso, true);
  }

  HSO_run(handleHso, &Out->Uab_emf_pu, correction_angle_pu);
<#if MC.M1_SPEED_SENSOR == "ZEST">  
  /* ZEST *******************************************************************************************/

  /* Estimated angle in cosine/sine */
  FIXP_CosSin_t park_cossin;
  HSO_getCosSinTh_ab(pHandle->pHSO, &park_cossin);

  /* Get the corrected and filtered Emf vector from ImpedCorr */
  Voltages_Uab_t Uab_filt_pu;
  IMPEDCORR_getEmf_ab_filt_pu(pHandle->pImpedCorr, &Uab_filt_pu);

  /* Perform the Park transformation for Emf */
  Voltages_Udq_t Udq_filt_pu;
  Udq_filt_pu = MCM_Park_Voltage(Uab_filt_pu, &park_cossin);

  /* Get the corrected and filtered current vector */
  Currents_Iab_t Iab_filt;
  IMPEDCORR_getIab_filt_pu(pHandle->pImpedCorr, &Iab_filt);

  /* Perform the Park transformation for current signal */
  Currents_Idq_t Idq_filt_pu;  
  Idq_filt_pu = MCM_Park_Current(Iab_filt, &park_cossin);

  ZEST_run(pHandle->pZeST, &Idq_filt_pu, &Udq_filt_pu, &pHandle->oneOverFlux_pu_fps, &pHandle->AngleCorrection_pu);

  // Delay ZeST contribution according to level of Zest fraction (80%) 
  if (ZEST_getInjectFactor(pHandle->pZeST) < FIXP(0.8f))
  {
    pHandle->AngleCorrection_pu = FIXP30(0.0);  /* reset ZeST correction during transient phase */
  }

  pHandle->zestOutD += (ZEST_getSignalD(pHandle->pZeST) - pHandle->zestOutD) >> 12;
  pHandle->zestOutQ += (ZEST_getSignalQ(pHandle->pZeST) - pHandle->zestOutQ) >> 12;

  ZEST_getIdq_ref_inject_pu(pHandle->pZeST, &Out->Idq_inject_pu);
</#if>

  /* store flux, angle and frequency for monitoring */
  pHandle->Flux_ab = HSO_getFlux_ab_Wb(handleHso);
  pHandle->Flux_Active_Wb = HSO_getFluxAmpl_Wb(handleHso);
  pHandle->theta_pu = HSO_getAngle_pu(handleHso);
  pHandle->Fe_hso_pu = HSO_getEmfSpeed_pu(handleHso);
<#if MC.M1_SPEED_SENSOR == "ZEST">
  pHandle->ZestcorrectionShifted = pHandle->AngleCorrection_pu << 8;
  pHandle->SpeedLP_pu = HSO_getSpeedLP_pu(handleHso);
  pHandle->RsEstimated = FIXP30(RSEST_getRsOhm(pHandle->pRsEst));
  pHandle->CheckDir = HSO_getCheckDir(handleHso); 
  pHandle->qEQdqrip_D = (ZEST_getSignalD(pHandle->pZeST) << 8);
  pHandle->qEQdqrip_Q = (ZEST_getSignalQ(pHandle->pZeST) << 8);
  pHandle->fraction = ZEST_getInjectFactor(pHandle->pZeST);
</#if>

  Out->angSpeed = pHandle->theta_pu;
  Out->speed = pHandle->Fe_hso_pu;

} /* end of SPD_Run() */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Exports the estimated electrical speed.
  * 
  * @param  pHandle: speed & position feedback control handler
  * @param  emfSpeed: Estimated speed from delta angle and taking into account ZeST correction signal
  * @param  speedLP: Estimated speed from delta angle
  */
void SPD_GetSpeed(SPD_Handle_t *pHandle, fixp30_t *emfSpeed, fixp30_t *speedLP)
{

  if (true == pHandle->closedLoopAngleEnabled)
  {
    *emfSpeed = HSO_getEmfSpeed_pu(pHandle->pHSO);
    *speedLP = HSO_getSpeedLP_pu(pHandle->pHSO);
  }
  else
  {
    *emfSpeed = 0;
    *speedLP	= 0;
  }
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Exports the estimated electrical angle.
  *         If the application is in closed loop then the angle is the one estimated
  *         by HSO. While in open loop case, the angle is the one forced during the open loop.
  * 
  * @param  pHandle: speed & position feedback control handler
  * @param  angle: Estimated electrical angle from HSO or from open loop update routine
  */
void SPD_GetAngle(SPD_Handle_t *pHandle, fixp30_t *angle)
{

  if (true == pHandle->closedLoopAngleEnabled)
  {
    *angle = HSO_getAngle_pu(pHandle->pHSO);
  }
  else
  {
    *angle = pHandle->OpenLoopAngle;
  }
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Updates the open loop electrical angle
  *         this routines is necessary when open loop mode is selected
  * @param  pHandle: speed & position feedback control handler
  * @param  speedRef: electrical speed reference
  */
void SPD_UpdtOpenLoopAngle(SPD_Handle_t *pHandle, fixp30_t speedRef)
{
  fixp30_t angle_step_pu = FIXP30_mpy(speedRef, pHandle->Hz_pu_to_step_per_isr);
  fixp30_t angle_pu = pHandle->OpenLoopAngle;
  angle_pu += angle_step_pu;
  angle_pu &= (FIXP30(1.0f) - 1);
  pHandle->OpenLoopAngle = angle_pu;
}

<#if MC.M1_SPEED_SENSOR == "ZEST">   
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Not applicable
  * @param  pHandle: speed & position feedback control handler
  */
void MC_ZEST_Control_Update(SPD_Handle_t *pHandle)
{
  /* Update the ZEST configuration from the control structure */

  FOC_ZESTControl_s* pControl = &pHandle->zestControl;

  fixp30_t injectFreq_kHz = pControl->injectFreq_kHz;
  fixp_t gainQ = pControl->feedbackGainQ;
  fixp_t gainD = pControl->feedbackGainD;

  // Allow to switch ZeST configuration: injection frequency & Gains in run time
  if ( pHandle->flagDynamicZestConfig && (pHandle->Idq_in_pu.Q > pControl->CurrentLimitToSwitchZestConfig_pu) )
  {
    injectFreq_kHz = pControl->injectFreq_Alt_kHz;
    gainQ = pControl->feedbackGainQ_Alt;    
    gainD = pControl->feedbackGainD_Alt;        
  }

  ZEST_setFreqInjectkHz(pHandle->pZeST, injectFreq_kHz);
  ZEST_setGainD(pHandle->pZeST, gainD);
  ZEST_setGainQ(pHandle->pZeST, gainQ);

  ZEST_setIdInject_pu(pHandle->pZeST, pControl->injectId_A_pu);
  ZEST_setIqInject_pu(pHandle->pZeST, pControl->injectIq_A_pu);

  /* Read feedback from ZEST */
  FOC_ZestFeedback_s* pFeedback = &pHandle->zestFeedback;

  pFeedback->signalD = ZEST_getSignalD(pHandle->pZeST);
  pFeedback->signalQ = ZEST_getSignalQ(pHandle->pZeST);
  pFeedback->L = ZEST_getL(pHandle->pZeST);
  pFeedback->R = ZEST_getR(pHandle->pZeST);
  pFeedback->thresholdFreq_pu = ZEST_getThresholdFreq_pu(pHandle->pZeST);
}

/**
  * @brief  Sets an alternative minimum cross over frequency used at low speed.
  */
void SPD_SetMinCrossOverAtLowSpeed_Hz(SPD_Handle_t *pHandle, float_t value)
{
  pHandle->MinCrossOverAtLowSpeed_Hz = value;
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