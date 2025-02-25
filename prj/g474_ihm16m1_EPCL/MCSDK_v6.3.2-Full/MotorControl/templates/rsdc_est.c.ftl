<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
 ******************************************************************************
 * @file    rsdc_est.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file implements Rs DC estimation routines
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
 * @ingroup rsdcest
 */

/* Includes ------------------------------------------------------------------*/
#include "rsdc_est.h"
#include "mc_parameters.h"
#include "parameters_conversion.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup rsdcest Rs DC estimation
  * @{
  */

/**
 * @defgroup rsdcest
 *
 * @brief Rs DC estimation process
 *
 * The resistance estimator starts with an offset corrected inverter, running at 50%
 * duty. Current @f$i_d@f$ is ramped up to a preset value in about 250 ms to allow for
 * smooth alignment in case the rotor is mis-aligned. During the ramp-up, filters
 * are calculated at interrupt frequency to low-pass filter @f$u_{\alpha\beta LP}@f$ and @f$i_{\alpha\beta LP}@f$
 * After about 750 ms from the start of the ramp, the resistance is estimated by 
 * @f$ Rs = \frac {u_{\alpha\beta LP}}{i_{\alpha\beta LP}} @f$.
 * Figure below shows the sequence for a mis-aligned motor that aligns after 300 ms
 * after the start of the ramp. The ZeST injection is started half-way the ramp-down
 * to ensure demodulator readiness at the transit to speed or torque control.
 *
 * ![](rsdc_est.png)
 * 
 * @{
 */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/

/* Performs the CPU load measure of FOC main tasks. */

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  Initializes Rs DC estimation process
  * @param  pFOCVars: FOCvars data structure
  * @param  pRsDCEst: Rs DC estimation data structure  
  */
void RsDC_estimationInit(FOCVars_t *pFOCVars, RsDCEstimation_Handle_t *pRsDCEst)
{
  HSO_setFlag_TrackAngle(pRsDCEst->pSPD->pHSO, false); /*fix present angle*/
  pRsDCEst->pSPD->flagHsoAngleEqualsOpenLoopAngle = true;
  pRsDCEst->pSPD->flagFreezeHsoAngle = true;
  PIDREGDQX_CURRENT_setUiD_pu(&pRsDCEst->pCurrCtrl->pid_IdIqX_obj, FIXP30(0.0f)); /* reset current controllers */
  PIDREGDQX_CURRENT_setUiQ_pu(&pRsDCEst->pCurrCtrl->pid_IdIqX_obj, FIXP30(0.0f));

<#if MC.M1_SPEED_SENSOR == "ZEST">    
  if (pRsDCEst->RSDCestimate_Fast == true)
  {
    /* ramp configuration for fast RsDC measure */
    pRsDCEst->RSDCestimate_counterInitialValue = pRsDCEst->FastRsDC_measure_duration;
    pRsDCEst->RSDCestimate_counterMeasureValue = pRsDCEst->FastRsDC_ramp_duration;
    pRsDCEst->RSDCestimate_FilterShift = 5;
  }
  else
  {
    /* ramp configuration for default RsDC measure */
    pRsDCEst->RSDCestimate_counterInitialValue = RSDC_TOTAL_MEASURE_TIME_TICKS; 
    pRsDCEst->RSDCestimate_counterMeasureValue = RSDC_SLOP_TICKS;     
    pRsDCEst->RSDCestimate_FilterShift = 10;
    /* Set current Kp dedicated to alignment procedure */	
    PIDREGDQX_CURRENT_setKp_si(&pRsDCEst->pCurrCtrl->pid_IdIqX_obj, pRsDCEst->pCurrCtrl->pid_IdIqX_obj.KpAlign);
  }

  ZEST_setInjectMode(pRsDCEst->pSPD->pZeST, ZEST_INJECTMODE_None); /* de-activate ZEST */
<#else>
  /* ramp configuration for default RsDC measure */
  pRsDCEst->RSDCestimate_counterInitialValue = RSDC_TOTAL_MEASURE_TIME_TICKS; 
  pRsDCEst->RSDCestimate_counterMeasureValue = RSDC_SLOP_TICKS;     
  pRsDCEst->RSDCestimate_FilterShift = 10;
  /* Set current Kp dedicated to alignment procedure */	
  PIDREGDQX_CURRENT_setKp_si(&pRsDCEst->pCurrCtrl->pid_IdIqX_obj, pRsDCEst->pCurrCtrl->pid_IdIqX_obj.KpAlign);
</#if>  
  pRsDCEst->RSDCestimate_counter = pRsDCEst->RSDCestimate_counterInitialValue;
  pRsDCEst->pCurrCtrl->forceZeroPwm = false;
  pRsDCEst->pCurrCtrl->currentControlEnabled = true;
  pRsDCEst->pSTC->SpeedControlEnabled = false;
  pRsDCEst->pSPD->closedLoopAngleEnabled = false;
  pRsDCEst->pSTC->Idq_ref_pu.D  = FIXP30(0.0f);
  /*save Iq ref in case of torque control */
  pRsDCEst->RSDCestimate_IqRefStore = pRsDCEst->pSTC->Idq_ref_pu.Q;
  pRsDCEst->pSTC->Idq_ref_pu.Q  = FIXP30(0.0f);
  pRsDCEst->Iab_inLP_pu.A = FIXP30(0.0f); /* reset LP filters */
  pRsDCEst->Iab_inLP_pu.B = FIXP30(0.0f);
  pRsDCEst->Uab_inLP_pu.A = FIXP30(0.0f);
  pRsDCEst->Uab_inLP_pu.B = FIXP30(0.0f);
  /* set Max current for RsDC estimation */
  pRsDCEst->RSDCestimate_Id_ref_pu = FIXP30_mpy(pRsDCEst->pSTC->I_max_pu,FIXP30(0.5f));
  pRsDCEst->RSDCestimate_rampRate = FIXP30_mpy(pRsDCEst->RSDCestimate_Id_ref_pu, FIXP30(1.0f / pRsDCEst->RSDCestimate_counterMeasureValue));
  PWMC_SwitchOnPWM(pRsDCEst->pPWM);
  pRsDCEst->RSDCestimate_state = RSDC_EST_ONGOING;
}

/**
  * @brief  Executes Rs DC estimation process
  * @param  pFOCVars: FOCvars data structure
  * @param  pRsDCEst: Rs DC estimation data structure  
  */
void RsDC_estimationRun(FOCVars_t *pFOCVars, RsDCEstimation_Handle_t *pRsDCEst)
{
  if (pRsDCEst->RSDCestimate_counter > 0) pRsDCEst->RSDCestimate_counter--;

  uint32_t counter = pRsDCEst->RSDCestimate_counter; /* counting downwards */
  fixp30_t  ramprate = pRsDCEst->RSDCestimate_rampRate;
  uint32_t transitcount = pRsDCEst->RSDCestimate_counterMeasureValue;
  if (counter > transitcount)
  {
    /* ramp up */
    fixp30_t idref = pRsDCEst->pSTC->Idq_ref_pu.D + ramprate;
    pRsDCEst->pSTC->Idq_ref_pu.D = FIXP_sat(idref, pRsDCEst->RSDCestimate_Id_ref_pu, FIXP30(0.0f));
  }
  else if (counter < transitcount)
  {
    /* ramp down */
    fixp30_t idref = pRsDCEst->pSTC->Idq_ref_pu.D - ramprate;
    pRsDCEst->pSTC->Idq_ref_pu.D = FIXP_sat(idref, pRsDCEst->RSDCestimate_Id_ref_pu, FIXP30(0.0f));
  }
  else
  {}

  if (counter == transitcount)
  {
<#if MC.M1_SPEED_SENSOR == "ZEST">     
    if (pRsDCEst->RSDCestimate_Fast == false)
    {
      /* start ramp down, restore current Kp */
      PIDREGDQX_CURRENT_setKp_si(&pRsDCEst->pCurrCtrl->pid_IdIqX_obj, pRsDCEst->pCurrCtrl->pid_IdIqX_obj.Kp);
    }
<#else>
    /* start ramp down, restore current Kp */
    PIDREGDQX_CURRENT_setKp_si(&pRsDCEst->pCurrCtrl->pid_IdIqX_obj, pRsDCEst->pCurrCtrl->pid_IdIqX_obj.Kp);
</#if>    
  }

<#if MC.M1_SPEED_SENSOR == "ZEST">  
  if ((counter < (transitcount >> 1)) &&  (MCM_OBSERVING_MODE != pFOCVars->controlMode))
  {  
      ZEST_setInjectMode(pRsDCEst->pSPD->pZeST, ZEST_INJECTMODE_Auto); /* activate ZEST before ramp-finish */
  }
</#if>  
  if (pRsDCEst->RSDCestimate_counter == 0)
  {
      if (false == pRsDCEst->RSDCestimate_AlignOnly)
      {
        /* calculate resistance and load to impedance correction and Rsest */
        fixp30_t Umag_pu = FIXP30_mag(pRsDCEst->Uab_inLP_pu.A, pRsDCEst->Uab_inLP_pu.B);
        fixp30_t Imag_pu = FIXP30_mag(pRsDCEst->Iab_inLP_pu.A, pRsDCEst->Iab_inLP_pu.B);
        pFOCVars->Rs_Ohm = FIXP30_toF(Umag_pu) * scaleParams->voltage / (FIXP30_toF(Imag_pu) * scaleParams->current);
        IMPEDCORR_setRs_si(pRsDCEst->pSPD->pImpedCorr, pFOCVars->Rs_Ohm);
<#if MC.M1_SPEED_SENSOR == "ZEST">          
        RSEST_setRsOhm(pRsDCEst->pSPD->pRsEst, pFOCVars->Rs_Ohm);
        pRsDCEst->pSPD->pRsEst->rs_ohm = pFOCVars->Rs_Ohm;		
</#if>        
      }

      pRsDCEst->pSTC->Idq_ref_pu.D = FIXP30(0.0f);
      pRsDCEst->pSTC->Idq_ref_pu.Q = pRsDCEst->RSDCestimate_IqRefStore;

      pRsDCEst->pSPD->flagFreezeHsoAngle = false;
      pRsDCEst->pSPD->flagHsoAngleEqualsOpenLoopAngle = false;
      HSO_setFlag_TrackAngle(pRsDCEst->pSPD->pHSO, true); /* release angle in HSO */
      pRsDCEst->pSTC->speed_ref_ramped_pu = FIXP30(0.0f);
      pRsDCEst->RSDCestimate_state = RSDC_EST_COMPLETED;

  }
}

/**
  * @brief  Rs DC estimation filtering routine
  *
  * The routines applies a low pass filter on @f$ \alpha\beta @f$ currents and voltages.
  * The output is then used during Rs DC estimation.  
  *
  * @param  pRsDCEst: Rs DC estimation data structure  
  * @param  Iab_in_pu: input currents in @f$ \alpha\beta @f$ coordinates 
  * @param  Uab_in_pu: input voltages in @f$ \alpha\beta @f$ coordinates 
  */
void RsDC_estimationFiltering(RsDCEstimation_Handle_t *pRsDCEst,   Currents_Iab_t   *Iab_in_pu,  Voltages_Uab_t   *Uab_in_pu)
{
  int16_t shift = pRsDCEst->RSDCestimate_FilterShift;

  /* Calculate low pass version of current and voltage, for resistance measurement */
  pRsDCEst->Iab_inLP_pu.A += ((Iab_in_pu->A - pRsDCEst->Iab_inLP_pu.A) >> shift);
  pRsDCEst->Iab_inLP_pu.B += ((Iab_in_pu->B - pRsDCEst->Iab_inLP_pu.B) >> shift);
  pRsDCEst->Uab_inLP_pu.A += ((Uab_in_pu->A - pRsDCEst->Uab_inLP_pu.A) >> shift);
  pRsDCEst->Uab_inLP_pu.B += ((Uab_in_pu->B - pRsDCEst->Uab_inLP_pu.B) >> shift);
}

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