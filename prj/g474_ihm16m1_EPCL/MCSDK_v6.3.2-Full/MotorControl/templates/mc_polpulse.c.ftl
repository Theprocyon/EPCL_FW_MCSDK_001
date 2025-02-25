<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
/**
  ******************************************************************************
  * @file    mc_polpulse.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides the code to trigger the PolPulse component.
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
  * @ingroup MC_PolPulse
  */
#include "mc_polpulse.h"
#include "parameters_conversion.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MC_PolPulse Polarisation detection
  *
  * @brief PolPulse component of the Motor Control SDK
  *
  *  PolPulse is an acronym for Polarity detection thru Pulsing.
  *  To ensure a proper start up, the PolPulse is used to determine the actual angle of
  *  a near-stand-still machine by utilizing the non-linear magnetic properties of
  *  the stator-core.
  * 
  * @{
  */

/** @brief  Used to enable or disable all PWM channels */
#define TIMxCCER_MASK_CH123        ((uint16_t)  (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                                 LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                                 LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))
 
/**
  * @brief  Initializes PolPulse component.
  * 
  *         The pulse current goal is set arbitrary to 1/4 of short circuit current by taking
  *         into account the rated flux and the inductance value of the motor. Knowing that it will
  *         be limited to the board soft overcurrent trip value.  
  *         The number of pulse-periods, decay-periods and directions to pulses are initialized according to 
  *         the values present in flash. Each of them can be changed in tuning phase by using dedicated API. 
  * 
  * @param  pHandle: PolPulse component handler
  * @param  pParams PolPulse parameters
  * @param  flashParams: flash parameters
  */
void MC_POLPULSE_init(MC_PolPulse_Handle_t *pHandle, 
                      POLPULSE_Params *params, 
                      FLASH_Params_t const *flashParams)
{
  
  params->VoltageScale = flashParams->scale.voltage;
  params->Lsd = flashParams->motor.ls;
  float_t PulseCurrentGoal_A = flashParams->polPulse.PulseCurrentGoal; 
  PulseCurrentGoal_A = (PulseCurrentGoal_A > flashParams->board.softOverCurrentTrip) ? flashParams->board.softOverCurrentTrip : PulseCurrentGoal_A;
  params->PulseCurrentGoal_A = PulseCurrentGoal_A;

  params->N = flashParams->polPulse.N;
  params->Nd = flashParams->polPulse.Nd;        
  params->N_Angles = 4;                  // Nb angles fixed to 4 according to specification
  POLPULSE_setParams(&pHandle->PolpulseObj, params);

  pHandle->pulse_countdown = -1;
}

/**
  * @brief  Executes PolPulse procedure.
  *         The PolPulse is initiated at each transition to close-loop speed or torque control. 
  *         When the PolPulse is trigged, the PWM channels are activated here according to the pulses duration.
  *         The computed duties will be used throughout the pulse period. Once pulse sequence is completed the 
  *         estimated angle is used to initialize the start angle for the observer (HSO).     
  * 
  *         This procedure is called during current controller task and before the space vector modulation procedure.
  *
  *  @param  pHandle: PolPulse handler
  *  @param  angle: optional
  */
void MC_POLPULSE_run(MC_PolPulse_Handle_t *pHandle, const fixp30_t angle_pu)
{
  /* Static variables to remember the state of the PWM before the pulse was triggered */
  static bool polpulse_overrule_prev = false;
  static bool polpulse_pwm_output_prev = false;
  POLPULSE_Handle pPulseHandle = &pHandle->PolpulseObj;
  TIM_TypeDef * TIMx = pHandle->TIMx;

  // Iab_in_pu/Uab_in_pu are updated in PWMC_GetMeasurements()
  POLPULSE_run(pPulseHandle, &pHandle->pPWM->Iab_in_pu, &pHandle->pPWM->Uab_in_pu, angle_pu);

  /* Remember and restore PWM Output state */
  bool ppoverrule = POLPULSE_getOverruleDuty(pPulseHandle);
  if (ppoverrule != polpulse_overrule_prev)
  {
    if (ppoverrule)
    {
      // Overrule starts

      // remember previous pwm output state
      polpulse_pwm_output_prev = LL_TIM_IsEnabledAllOutputs(TIMx); // Read PWM Output state

      // Disable pwm output, by MOE and AOE
      TIMx->BDTR &= ~(TIM_BDTR_MOE | TIM_BDTR_AOE);
      
      LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH1);
      LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH2);
      LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH3);
    
    }
    else
    {
      // overrule ends

      // Set PWM output state to the stored state
      if (polpulse_pwm_output_prev)
      {
        /* set 50% duty on all channels to overwrite the last pulse duty before to switch on close loop */
        uint32_t ticksFor50PercentDuty = LL_TIM_GetAutoReload(TIMx) >> 1;
      	LL_TIM_OC_SetCompareCH1(TIMx, ticksFor50PercentDuty);
       	LL_TIM_OC_SetCompareCH2(TIMx, ticksFor50PercentDuty);
       	LL_TIM_OC_SetCompareCH3(TIMx, ticksFor50PercentDuty);

        /* re-enable preload which has been disable at the beginning of the PolPulse*/
        LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH1);
        LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH2);
        LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH3);
       
<#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") >
        /* deactivate driver protection input while STDRIVE101 exiting standby mode */
        LL_TIM_Disable${brkNb}(TIMx);
</#if>
        /* re-enable TIM outputs */
        LL_TIM_CC_EnableChannel(pHandle->TIMx, TIMxCCER_MASK_CH123);
        LL_TIM_EnableAllOutputs(TIMx);
      }
      else
      {
        LL_TIM_DisableAllOutputs(TIMx);
      }

      // Set angle for startup
      HSO_setAngle_pu(pHandle->pSPD->pHSO, POLPULSE_getAngleEstimated(pPulseHandle));
      HSO_adjustAngle_pu(pHandle->pSPD->pHSO, 0);
      HSO_setFlag_TrackAngle(pHandle->pSPD->pHSO, false); /*fix present angle*/
      pHandle->pSPD->OpenLoopAngle = HSO_getAngle_pu(pHandle->pSPD->pHSO);
      /* reset ramped speed */
      pHandle->pSTC->speed_ref_ramped_pu = 0;

    }
    // Disable countdown
    pHandle->pulse_countdown = -1;
    polpulse_overrule_prev = ppoverrule;
  }

  if (ppoverrule)
  {
    if (POLPULSE_getTriggerPulse(pPulseHandle))
    {
      // Clear trigger, prevent auto repeats
      POLPULSE_clearTriggerPulse(pPulseHandle);

      /* Create a pulse */
      {
        // Set countdown so the PWM turns off at end of the required number of pulse
        pHandle->pulse_countdown = POLPULSE_getPulsePeriods(pPulseHandle) * REGULATION_EXECUTION_RATE;

        // Enable MOE at start of period via BDTR.AOE
        LL_TIM_ClearFlag_CC4(TIMx);
        
        /* Enable interrupt that will execute pulse generation (PulseCountDown()) */
        LL_TIM_EnableIT_CC4(TIMx);
      }
    }
  }
} /* end of MC_POLPULSE_run() function */

/**
  * @brief  handles pulse generation.
  *         The PWM is generated during the countdown period and switched off automatically at end.
  *         It should be called during TIMx CC4 IRQ handler.
  *         
  * @param  pHandle: PolPulse handler
  */
void PulseCountDown(MC_PolPulse_Handle_t *pHandle)
{
  if (pHandle->pulse_countdown > 0 )
  {
    if (LL_TIM_IsEnabledAllOutputs(pHandle->TIMx) == 0UL)
    {
<#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") >
      /* deactivate driver protection input while STDRIVE101 exiting standby mode */
      LL_TIM_Disable${brkNb}(pHandle->TIMx);
</#if>
      LL_TIM_CC_EnableChannel(pHandle->TIMx, TIMxCCER_MASK_CH123);
      LL_TIM_EnableAllOutputs(pHandle->TIMx);
    }
    else
    {
      /* nothing to do*/
    }
  }
  else
  {
    /* counter reached 0, stop pulsing */
    /* Disable interrupt */
    LL_TIM_DisableIT_CC4(pHandle->TIMx);

    /* Report pulse is done */
    POLPULSE_setPulseDone(&pHandle->PolpulseObj);

    /* Disable PWM output and TIM PWM channels to be in Hi-Z,
       this reuslts to a fast decay */ 
    LL_TIM_DisableAllOutputs(pHandle->TIMx);
    LL_TIM_CC_DisableChannel(pHandle->TIMx, TIMxCCER_MASK_CH123);
  }
  pHandle->pulse_countdown--;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
