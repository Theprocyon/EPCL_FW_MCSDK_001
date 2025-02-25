/**
  ******************************************************************************
  * @file    f0xx_bemf_ADC_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement Bemf sensing
  *          class to be stantiated when the six-step sensorless driving mode
  *          topology is used.
  * 
  *          It is specifically designed for STM32F0XX
  *          microcontrollers and implements the sensing using one ADC with
  *          DMA support.
  *           + MCU peripheral and handle initialization fucntion
  *           + ADC sampling function
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
  * @ingroup SpeednPosFdbk_Bemf
  */

/* Includes ------------------------------------------------------------------*/
#include "f0xx_bemf_ADC_fdbk.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @addtogroup SpeednPosFdbk_Bemf
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define MAX_PSEUDO_SPEED        ((int16_t)0x7FFF)

#define EL_ANGLE_BASE_CW (- S16_60_PHASE_SHIFT / 2)
#define EL_ANGLE_BASE_CCW (S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2)

/* Private function prototypes -----------------------------------------------*/
void BADC_CalcAvrgElSpeedDpp(Bemf_ADC_Handle_t * pHandle);
/* Private functions ---------------------------------------------------------*/

/*
  * @brief  Initializes ADC1 and NVIC for three bemf voltages reading
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  */
__weak void BADC_Init(Bemf_ADC_Handle_t *pHandle)
{

  if ((uint32_t)pHandle == (uint32_t)&pHandle->_Super)
  {
    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_AWD1(ADC1);
    LL_ADC_ClearFlag_AWD1(ADC1);

    /* ADC Calibration */
    LL_ADC_StartCalibration(ADC1);
    while ((LL_ADC_IsCalibrationOnGoing(ADC1) == SET) ||
           (LL_ADC_REG_IsConversionOngoing(ADC1) == SET) ||
           (LL_ADC_REG_IsStopConversionOngoing(ADC1) == SET) ||
           (LL_ADC_IsDisableOngoing(ADC1) == SET))
    {
      /* wait */
    }

    /* Enables the ADC peripheral */
    LL_ADC_Enable(ADC1);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
    /* Wait ADC Ready */
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == RESET)
    {
      /* wait */
    }

    /* We allow ADC usage for regular conversion on Systick*/
    pHandle->ADCRegularLocked=false;

    uint16_t hMinReliableElSpeedUnit = pHandle->_Super.hMinReliableMecSpeedUnit * pHandle->_Super.bElToMecRatio;
    uint16_t hMaxReliableElSpeedUnit = pHandle->_Super.hMaxReliableMecSpeedUnit * pHandle->_Super.bElToMecRatio;
    uint8_t bSpeedBufferSize;
    uint8_t bIndex;
    uint16_t MinBemfTime, BemfTimeout;

    /* Adjustment factor: minimum measurable speed is x time less than the minimum
    reliable speed */
    hMinReliableElSpeedUnit /= 4U;

    /* Adjustment factor: maximum measurable speed is x time greater than the
    maximum reliable speed */
    hMaxReliableElSpeedUnit *= 2U;

    /* SW Init */
    if (0U == hMinReliableElSpeedUnit)
    {

      /* Set fixed to 150 ms */
      BemfTimeout = 150U;
    }
    else
    {
      /* Set accordingly the min reliable speed */
      /* 1000 comes from mS
      * 6 comes from the fact that bemf zero crossing is sensed each 60 deg = 360/6 deg */
      BemfTimeout = (1000U * (uint16_t)SPEED_UNIT) / (6U * hMinReliableElSpeedUnit);
    }

    pHandle->MaxPeriod = (uint32_t)((pHandle->TIMClockFreq * BemfTimeout) / 1000U);
    MinBemfTime = ((1000U * (uint16_t)SPEED_UNIT) << 8) / (6U * hMaxReliableElSpeedUnit);
    pHandle->MinPeriod = (uint32_t)(((pHandle->TIMClockFreq * MinBemfTime) >> 8) / 1000U);
    pHandle->SatSpeed = hMaxReliableElSpeedUnit;

    pHandle->PseudoPeriodConv = ((pHandle->TIMClockFreq / 6U) / pHandle->_Super.hMeasurementFrequency)
                              * pHandle->_Super.DPPConvFactor;

    pHandle->PWMNbrPSamplingFreq = ((pHandle->_Super.hMeasurementFrequency * pHandle->PWMFreqScaling) /
                                    pHandle->SpeedSamplingFreqHz) - 1U;

    pHandle->pSensing_Point = &(pHandle->Pwm_H_L.SamplingPointOff);
    pHandle->IsOnSensingEnabled = false;
    pHandle->ElPeriodSum = 0;
    pHandle->ZcEvents = 0;
  
    /* Erase speed buffer */
    bSpeedBufferSize = pHandle->SpeedBufferSize;
    for (bIndex = 0u;bIndex < bSpeedBufferSize;bIndex++)
    {
      pHandle->SpeedBufferDpp[bIndex]  = (int32_t)pHandle->MaxPeriod;
    }
    LL_TIM_EnableCounter(pHandle->pParams_str->LfTim);
  }
}

/*
  * @brief  Resets the ADC status
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  */
__weak void BADC_Clear(Bemf_ADC_Handle_t *pHandle)
{
  pHandle->ZcEvents = 0;
  pHandle->ElPeriodSum = 0;

  /* Acceleration measurement not implemented.*/
  pHandle->_Super.hMecAccelUnitP = 0;
  pHandle->BufferFilled = 0U;
  pHandle->CompSpeed = 0;

  /* Initialize speed buffer index */
  pHandle->SpeedFIFOIdx = 0U;
  pHandle->_Super.hElAngle  = 0;
  
  /* Clear speed error counter */
  pHandle->_Super.bSpeedErrorNumber = 0;
  pHandle->IsLoopClosed=false;
  pHandle->IsAlgorithmConverged = false;
  pHandle->ADCRegularLocked=false;
}

/*
 * @brief  Starts bemf ADC conversion of the phase depending on current step
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 * @param  step: current step of the six-step sequence
 */
__weak void BADC_Start(Bemf_ADC_Handle_t *pHandle, uint8_t step, uint8_t LSModArray[])
{
  uint16_t Bemf_Threshold;

  if (LSModArray[step]==1) Bemf_Threshold = pHandle->pSensing_Threshold_LSMod;
  else Bemf_Threshold = pHandle->pSensing_Threshold_HSMod;
  pHandle->ADCRegularLocked=true;

  while (LL_ADC_REG_IsConversionOngoing(ADC1))
  {
    LL_ADC_REG_StopConversion(ADC1);
    while(LL_ADC_REG_IsStopConversionOngoing(ADC1));
  }

/* enable ADC source trigger */
  LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO);
  LL_ADC_REG_SetTriggerEdge (ADC1, LL_ADC_REG_TRIG_EXT_FALLING);
  LL_ADC_SetSamplingTimeCommonChannels (ADC1,  LL_ADC_SAMPLINGTIME_1CYCLE_5);

  switch (step)
  {
    case STEP_1:
    case STEP_3:
    case STEP_5:
      if(pHandle->Direction == 1)
      {
        LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_HIGH, 0xFFF);
        LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_LOW, (uint32_t) (Bemf_Threshold >> 4));
      }
      else
      {
        LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_LOW, 0);
        LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_HIGH, (uint32_t) (Bemf_Threshold >> 4));
      }
      break;

    case STEP_2:
    case STEP_4:
    case STEP_6:
      if(pHandle->Direction == 1)
      {
        LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_LOW, 0);
        LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_HIGH, (uint32_t) (Bemf_Threshold >> 4));
      }
      else
      {
        LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_HIGH, 0xFFF);
        LL_ADC_SetAnalogWDThresholds(ADC1, LL_ADC_AWD_THRESHOLD_LOW, (uint32_t) (Bemf_Threshold >> 4));
      }
      break;

    default:
      break;
  }

  BADC_SelectAdcChannel(pHandle, step);

  /*start regular conversion */
  LL_ADC_ClearFlag_AWD1(ADC1);
  LL_ADC_EnableIT_AWD1(ADC1);
  LL_ADC_REG_StartConversion(ADC1);
}

/*
 * @brief  Stops bemf ADC conversion
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 */
__weak void BADC_Stop(Bemf_ADC_Handle_t *pHandle)
{
  while (LL_ADC_REG_IsConversionOngoing(ADC1))
  {
    LL_ADC_REG_StopConversion(ADC1);
    while(LL_ADC_REG_IsStopConversionOngoing(ADC1));
  }
  /* Disable AWD */
  LL_ADC_DisableIT_AWD1(ADC1);
  /* Clear AWD */
  LL_ADC_ClearFlag_AWD1(ADC1);
}

/*
 * @brief  Disables the speed  loop (low frequency) timer
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 */
__weak void BADC_SpeedMeasureOff(Bemf_ADC_Handle_t *pHandle)
{
  LL_TIM_DisableIT_CC1(pHandle->pParams_str->LfTim);
}

/*
  * @brief  Configures the ADC for the current sampling.
  *         It sets the sampling point via TIM1_Ch4 value, the ADC sequence
  *         and channels.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  pHandlePWMC: handler of the current instance of the PWMC component
  * @param  pHandleSTC: handler of the current instance of the Speed Control component
  */
__weak void BADC_SetSamplingPoint(Bemf_ADC_Handle_t *pHandle, PWMC_Handle_t *pHandlePWMC, BusVoltageSensor_Handle_t *BusVHandle)
{
  uint16_t latest_busConv = VBS_GetAvBusVoltage_d(BusVHandle);
  if (pHandle->DriveMode == VM)
  {
    if (pHandlePWMC->CntPh > pHandle->OnSensingEnThres)
    {
      uint16_t Threshold_Pwm = (uint16_t) (pHandle->Pwm_H_L.AdcThresholdPwmPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor)
        + pHandle->Pwm_H_L.ThresholdCorrectFactor;
      pHandle->IsOnSensingEnabled=true;
      pHandle->pSensing_Point = &(pHandle->Pwm_H_L.SamplingPointOn);
      pHandle->pSensing_Threshold_HSMod = Threshold_Pwm;
      pHandle->pSensing_Threshold_LSMod = Threshold_Pwm;
    }
    else if (pHandlePWMC->CntPh < pHandle->OnSensingDisThres)
    {
      pHandle->IsOnSensingEnabled=false;
      pHandle->pSensing_Point = &(pHandle->Pwm_H_L.SamplingPointOff);
      pHandle->pSensing_Threshold_HSMod = (uint16_t) (pHandle->Pwm_H_L.AdcThresholdLowPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor);
      pHandle->pSensing_Threshold_LSMod = (uint16_t) (pHandle->Pwm_H_L.AdcThresholdHighPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor)
        + pHandle->Pwm_H_L.ThresholdCorrectFactor;
    }
    else if (pHandle->IsOnSensingEnabled == false)
    {
      pHandle->pSensing_Threshold_HSMod = (uint16_t) (pHandle->Pwm_H_L.AdcThresholdLowPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor);
      pHandle->pSensing_Threshold_LSMod = (uint16_t) (pHandle->Pwm_H_L.AdcThresholdHighPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor)
        + pHandle->Pwm_H_L.ThresholdCorrectFactor;
    }
    else
    {
      uint16_t Threshold_Pwm = (uint16_t) (pHandle->Pwm_H_L.AdcThresholdPwmPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor)
        + pHandle->Pwm_H_L.ThresholdCorrectFactor;
      pHandle->pSensing_Threshold_HSMod = Threshold_Pwm;
      pHandle->pSensing_Threshold_LSMod = Threshold_Pwm;
    }
  }
  else
  {
    pHandle->IsOnSensingEnabled=false;
    pHandle->pSensing_Point = &(pHandle->Pwm_H_L.SamplingPointOff);
    pHandle->pSensing_Threshold_HSMod = (uint16_t) (pHandle->Pwm_H_L.AdcThresholdLowPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor);
    pHandle->pSensing_Threshold_LSMod = (uint16_t) (pHandle->Pwm_H_L.AdcThresholdHighPerc * latest_busConv/pHandle->Pwm_H_L.Bus2ThresholdConvFactor)
                                        + pHandle->Pwm_H_L.ThresholdCorrectFactor;
  }
}

/*
 * @brief  Gets last bemf value and checks for zero crossing detection.
 *         Updates speed loop timer and electrical angle accordingly.
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 * @param  pHandlePWMC: handler of the current instance of the PWMC component
 * @retval none
 */
__weak void BADC_IsZcDetected(Bemf_ADC_Handle_t *pHandle, uint8_t step)
{
  uint32_t TimerSpeed_Counter, CounterAutoreload;
  uint32_t CC_Counter = 0U;/* for switch case default before CounterAutoreload comparison */
  uint32_t tStepTime;
  if (pHandle->SpeedTimerState == COMMUTATION)
  {
    if(pHandle->Direction == 1)
    {
      pHandle->MeasuredElAngle = (int16_t)(EL_ANGLE_BASE_CW + step * S16_60_PHASE_SHIFT);
    }
    else
    {
      pHandle->MeasuredElAngle = (int16_t)(EL_ANGLE_BASE_CCW + step * S16_60_PHASE_SHIFT);
    }
    LL_ADC_DisableIT_AWD1(ADC1);
    if (pHandle->ZcEvents > pHandle->StartUpConsistThreshold)
    {
      pHandle->IsAlgorithmConverged = true;
    }
    TimerSpeed_Counter = LL_TIM_GetCounter(pHandle->pParams_str->LfTim);
    CounterAutoreload = LL_TIM_GetAutoReload(pHandle->pParams_str->LfTim) + 1;
    if (TimerSpeed_Counter < pHandle->Last_TimerSpeed_Counter) tStepTime = CounterAutoreload - (pHandle->Last_TimerSpeed_Counter - TimerSpeed_Counter);
    else tStepTime = TimerSpeed_Counter - pHandle->Last_TimerSpeed_Counter;
    pHandle->ZcEvents++;

    switch(step)
    {
      case STEP_1:
      case STEP_3:
      case STEP_5:
        if(pHandle->Direction == 1)
        {
          pHandle->StepTime_Down =  tStepTime;
          if (pHandle->ComputationDelay == 2) tStepTime = pHandle->StepTime_Last;
          CC_Counter = (uint32_t) (TimerSpeed_Counter + (((uint32_t)((pHandle->ZcFalling2CommDelay) * tStepTime)) >> 9));
          pHandle->StepTime_Last = pHandle->StepTime_Down;
        }
        else
        {
          pHandle->StepTime_Up =  tStepTime;
          if (pHandle->ComputationDelay == 2) tStepTime = pHandle->StepTime_Last;
          CC_Counter = (uint32_t) (TimerSpeed_Counter + (((uint32_t)((pHandle->ZcRising2CommDelay) * tStepTime)) >> 9));
          pHandle->StepTime_Last = pHandle->StepTime_Up;
        }
        break;

      case STEP_2:
      case STEP_4:
      case STEP_6:
        if(pHandle->Direction == 1)
        {
          pHandle->StepTime_Up =  tStepTime;
          if (pHandle->ComputationDelay == 2) tStepTime = pHandle->StepTime_Last;
          CC_Counter = (uint32_t) (TimerSpeed_Counter + (((uint32_t)((pHandle->ZcRising2CommDelay) * tStepTime)) >> 9));
          pHandle->StepTime_Last = pHandle->StepTime_Up;
        }
        else
        {
          pHandle->StepTime_Down =  tStepTime;
          if (pHandle->ComputationDelay == 2) tStepTime = pHandle->StepTime_Last;
          CC_Counter = (uint32_t) (TimerSpeed_Counter + (((uint32_t)((pHandle->ZcFalling2CommDelay) * tStepTime)) >> 9));
          pHandle->StepTime_Last = pHandle->StepTime_Down;
        }
        break;
    }
    if (true == pHandle->IsLoopClosed)
    {
      if (CC_Counter >= CounterAutoreload) CC_Counter -= CounterAutoreload;
      LL_TIM_OC_SetCompareCH1(pHandle->pParams_str->LfTim, CC_Counter);
    }
    pHandle->Last_TimerSpeed_Counter = TimerSpeed_Counter;
  }
}

/*
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter hMecSpeedUnit - the rotor average mechanical speed,
  *         expressed in Unit. Average is computed considering a FIFO depth
  *         equal to SpeedBufferSizeUnit. Moreover it also computes and returns
  *         the reliability state of the sensor.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  pMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
  * @retval bool speed sensor reliability, measured with reference to parameters
  *         bMaximumSpeedErrorsNumber, VariancePercentage and SpeedBufferSize
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  */
__weak bool BADC_CalcAvrgMecSpeedUnit(Bemf_ADC_Handle_t *pHandle, int16_t *pMecSpeedUnit)
{
  bool bReliability=true;

  pHandle->_Super.hElSpeedDpp =  pHandle->AvrElSpeedDpp;
  if (0 ==  pHandle->AvrElSpeedDpp)
  {
    /* Speed is too low */
    *pMecSpeedUnit = 0;
  }
  else
  {
    /* Check if speed is not to fast */
    if (pHandle->AvrElSpeedDpp != MAX_PSEUDO_SPEED)
    {
      pHandle->DeltaAngle = pHandle->MeasuredElAngle - pHandle->_Super.hElAngle;
      pHandle->CompSpeed = (int16_t)((int32_t)(pHandle->DeltaAngle) / (int32_t)(pHandle->PWMNbrPSamplingFreq));

      /* Convert el_dpp to MecUnit */
      *pMecSpeedUnit = (int16_t)((pHandle->AvrElSpeedDpp * (int32_t)pHandle->_Super.hMeasurementFrequency
                       * (int32_t)SPEED_UNIT)
                       / ((int32_t)(pHandle->_Super.DPPConvFactor) * (int32_t)(pHandle->_Super.bElToMecRatio)));
    }
    else
    {
      *pMecSpeedUnit = (int16_t)pHandle->SatSpeed;
    }
  }
  if (pHandle->IsLoopClosed==true) bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeedUnit);

  pHandle->_Super.hAvrMecSpeedUnit = *pMecSpeedUnit;

  BADC_CalcAvrgElSpeedDpp (pHandle);
  return (bReliability);
}

/*
  * @brief  Returns false if calculated speed is out of reliability ranges
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval bool: reliability flag
  */
__weak bool BADC_IsSpeedReliable(Bemf_ADC_Handle_t *pHandle)
{
  bool SpeedError = false;

  if (pHandle->_Super.hAvrMecSpeedUnit > pHandle->_Super.hMaxReliableMecSpeedUnit)
  {
    SpeedError = true;
  }

  if (pHandle->_Super.hAvrMecSpeedUnit < pHandle->_Super.hMinReliableMecSpeedUnit)
  {
    SpeedError = true;
  }

  return (SpeedError);
}

/*
  * @brief  Forces the rotation direction
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  direction: imposed direction
  */
__weak void BADC_SetDirection(Bemf_ADC_Handle_t *pHandle, uint8_t direction)
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->Direction = direction;
  }
}

/*
  * @brief  Internally performs a checks necessary to state whether
  *         the bemf algorithm converged. To be periodically called
  *         during motor open-loop ramp-up (e.g. at the same frequency of
  *         SPD_CalcElAngle), it returns true if the estimated angle and speed
  *         can be considered reliable, false otherwise
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  *
  * @retval bool sensor reliability state
  */
__weak bool BADC_IsObserverConverged(Bemf_ADC_Handle_t *pHandle)
{
  return pHandle->IsAlgorithmConverged;
}

/*
  * @brief  Configures the proper ADC channel according to the current 
  *         step corresponding to the floating phase. To be periodically called
  *         at least at every step change. 
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  Phase: Floating phase for bemf acquisition
  */
void BADC_SelectAdcChannel(Bemf_ADC_Handle_t *pHandle, uint8_t step)
{
  /* Regular sequence configuration */
  LL_ADC_REG_SetSequencerChannels(ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->AdcChannel[step]));
  LL_ADC_SetAnalogWDMonitChannels(ADC1, __LL_ADC_ANALOGWD_CHANNEL_GROUP(__LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->AdcChannel[step]),LL_ADC_GROUP_REGULAR));
}

/*
  * @brief  Updates the estimated electrical angle.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval int16_t rotor electrical angle (s16Degrees)
  */
__weak int16_t BADC_CalcElAngle(Bemf_ADC_Handle_t *pHandle)
{
  int16_t retValue;
  if (pHandle->_Super.hElSpeedDpp != MAX_PSEUDO_SPEED)
  {
    if (false == pHandle->IsLoopClosed)
    {
      pHandle->MeasuredElAngle += pHandle->_Super.hElSpeedDpp;
      pHandle->PrevRotorFreq = pHandle->_Super.hElSpeedDpp;
      pHandle->_Super.hElAngle += pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed;
    }
    else
    {
      pHandle->_Super.hElAngle = pHandle->MeasuredElAngle;
    }
  }
  else
  {
    pHandle->_Super.hElAngle += pHandle->PrevRotorFreq;
  }
  retValue = pHandle->_Super.hElAngle;
  return (retValue);
}

/*
  * @brief  Must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and update component
  *         variable hElSpeedDpp that is estimated average electrical speed
  *         expressed in dpp used for instance in observer equations.
  *         Average is computed considering a FIFO depth equal to
  *         bSpeedBufferSizedpp.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  */
void BADC_CalcAvrgElSpeedDpp(Bemf_ADC_Handle_t *pHandle)
{
  uint32_t wCaptBuf;
  
  /* used to validate the average speed measurement */
  if (pHandle->BufferFilled < pHandle->SpeedBufferSize)
  {
    pHandle->BufferFilled++;
  }
  else
  {
    /* Nothing to do */
  }
  if (false == pHandle->IsLoopClosed)
  {
    if (pHandle->VirtualElSpeedDpp == 0) pHandle->Counter_Period = 0xFFFF;
    else pHandle->Counter_Period = (uint32_t)(pHandle->PseudoPeriodConv / ((pHandle->LowFreqTimerPsc + 1) * pHandle->VirtualElSpeedDpp));
  }
  else
  {
    pHandle->Counter_Period =  pHandle->StepTime_Last;
  }
  wCaptBuf = pHandle->Counter_Period * (pHandle->LowFreqTimerPsc + 1);
  /* Filtering to fast speed... could be a glitch  ? */
  /* the MAX_PSEUDO_SPEED is temporary in the buffer, and never included in average computation*/
  if (wCaptBuf < pHandle->MinPeriod)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->ElPeriodSum -= pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx];/* value we gonna removed from the accumulator */
    if (wCaptBuf >= pHandle->MaxPeriod)
    {
      pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx] = (int32_t)pHandle->MaxPeriod * pHandle->Direction;
    }
    else
    {
      pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx] = (int32_t)wCaptBuf;
      pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx] *= pHandle->Direction;
      pHandle->ElPeriodSum += pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx];
    }
    /* Update pointers to speed buffer */
    pHandle->SpeedFIFOIdx++;
    if (pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize)
    {
      pHandle->SpeedFIFOIdx = 0U;
    }
    if (((pHandle->BufferFilled < pHandle->SpeedBufferSize) && (wCaptBuf != 0U))
        || (false == pHandle->IsLoopClosed))
    {
      uint32_t tempReg = (pHandle->PseudoPeriodConv / wCaptBuf) * (uint32_t)pHandle->Direction;
      pHandle->AvrElSpeedDpp = (int16_t)tempReg;
    }
    else
    {
      /* Average speed allow to smooth the mechanical sensors misalignement */
      int32_t  tElPeriodSum = 0;
      uint8_t i;
      for (i=0;i < pHandle->SpeedBufferSize;i++)
      {
        tElPeriodSum += pHandle->SpeedBufferDpp[i];
      }
      pHandle->AvrElSpeedDpp = (int16_t)((int32_t)pHandle->PseudoPeriodConv /
                                               (tElPeriodSum / (int32_t)pHandle->SpeedBufferSize));/* Average value */
    }
  }
}

/*
  * @brief  Used to calculate instant speed during revup, to
  *         initialize parameters at step change and to select proper ADC channel
  *         for next Bemf acquisitions
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  hElSpeedDpp: Mechanical speed imposed by virtual speed component
  * @param  pHandlePWMC: handler of the current instance of the PWMC component
  */
void BADC_StepChangeEvent(Bemf_ADC_Handle_t *pHandle, int16_t hElSpeedDpp)
{
  uint32_t tempReg;
  if (pHandle->IsLoopClosed == false) tempReg = LL_TIM_GetCounter(pHandle->pParams_str->LfTim) + pHandle->DemagCounterThreshold;
  else tempReg = LL_TIM_OC_GetCompareCH1(pHandle->pParams_str->LfTim) + pHandle->DemagCounterThreshold;
  uint32_t CounterAutoreload = LL_TIM_GetAutoReload(pHandle->pParams_str->LfTim) + 1;
  BADC_Stop(pHandle);
  pHandle->SpeedTimerState = DEMAGNETIZATION;
  if (tempReg >= CounterAutoreload) tempReg -= CounterAutoreload;
  LL_TIM_OC_SetCompareCH1(pHandle->pParams_str->LfTim, tempReg);
  LL_TIM_ClearFlag_CC1(pHandle->pParams_str->LfTim);
  LL_TIM_EnableIT_CC1(pHandle->pParams_str->LfTim);

  if (false == pHandle->IsLoopClosed)
  {
    if (hElSpeedDpp < 0)
    {
      pHandle->VirtualElSpeedDpp = - hElSpeedDpp;
    }
    else
    {
      pHandle->VirtualElSpeedDpp = hElSpeedDpp;
    }
  }
  else
  {
    int16_t ElAngleUpdate;
    if(pHandle->Direction == -1)
    {
      ElAngleUpdate = -S16_60_PHASE_SHIFT;
    }
    else
    {
      ElAngleUpdate = S16_60_PHASE_SHIFT;
    }
    pHandle->MeasuredElAngle += ElAngleUpdate;
  }
}

/*
  * @brief  Calculates and stores in the corresponding variable the demagnetization 
  *         time in open loop operation  
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  */
void BADC_CalcRevUpDemagTime(Bemf_ADC_Handle_t *pHandle)
{
  int16_t hSpeed;
  hSpeed = (int16_t)((pHandle->VirtualElSpeedDpp * (int32_t)pHandle->_Super.hMeasurementFrequency
                       * (int32_t)SPEED_UNIT)
                       / ((int32_t)(pHandle->_Super.DPPConvFactor) * (int32_t)(pHandle->_Super.bElToMecRatio)));
  if (hSpeed == 0)
  {
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  }
  else
  {
    pHandle->DemagCounterThreshold = (uint32_t)(pHandle->DemagParams.RevUpDemagSpeedConv / hSpeed);
  }
  if (pHandle->DemagCounterThreshold < pHandle->DemagParams.DemagMinimumThreshold)
  {
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  }
}

/*
  * @brief  Calculates and stores in the corresponding variable the demagnetization 
  *         time in closed loop operation  
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  */
void BADC_CalcRunDemagTime(Bemf_ADC_Handle_t *pHandle)
{
  int16_t hSpeed;
  hSpeed = pHandle->_Super.hAvrMecSpeedUnit;
  if (hSpeed < 0) hSpeed = - hSpeed;
  if (hSpeed < pHandle->DemagParams.DemagMinimumSpeedUnit)
  {
    pHandle->DemagCounterThreshold = (uint32_t)(pHandle->DemagParams.RunDemagSpeedConv / hSpeed);
    if (pHandle->DemagCounterThreshold < pHandle->DemagParams.DemagMinimumThreshold)
    {
      pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
    }
  }
  else
  {
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  }
}

/*
  * @brief  Must be called after switch-over procedure when
  *         virtual speed sensor transition is ended.  
  * @param  pHandle: handler of the current instance of the STO component
  */
void BADC_SetLoopClosed(Bemf_ADC_Handle_t *pHandle)
{
  uint32_t CC_Counter;
  uint32_t CounterAutoreload = LL_TIM_GetAutoReload(pHandle->pParams_str->LfTim) + 1;
  pHandle->IsLoopClosed=true;
  CC_Counter = LL_TIM_OC_GetCompareCH1(pHandle->pParams_str->LfTim);
  CC_Counter = pHandle->Counter_Period + CC_Counter - pHandle->DemagCounterThreshold;
  if (CC_Counter >= CounterAutoreload) CC_Counter -= CounterAutoreload;
  LL_TIM_OC_SetCompareCH1(pHandle->pParams_str->LfTim, CC_Counter);
}

/**
  * @brief  Configures the parameters for bemf sensing during pwm off-time
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  BemfAdcConfig: thresholds and sampling time parameters
  * @param  ZcRising2CommDelay: delay between rising bemf zero crossing parameter and step commutation
  * @param  ZcFalling2CommDelay: delay between falling bemf zero crossing parameter and step commutation
  * @param  bemfAdcDemagConfig: demagnetization parameters
  * @param  OnSensingEnThres: Minimum duty cycle for on-sensing activation
  * @param  OnSensingDisThres: Minimum duty cycle for on-sensing Deactivationg
  * @param  ComputationDelay: Selection of the period for zero crossing to step change delay computation  
  */
void BADC_SetBemfSensorlessParam(Bemf_ADC_Handle_t *pHandle, Bemf_Sensing_Params *BemfAdcConfig, uint16_t *ZcRising2CommDelay, uint16_t *ZcFalling2CommDelay,
                                   Bemf_Demag_Params *bemfAdcDemagConfig, uint16_t *OnSensingEnThres, uint16_t *OnSensingDisThres, uint16_t *ComputationDelay)
{  
  pHandle->Pwm_H_L.AdcThresholdPwmPerc = BemfAdcConfig->AdcThresholdPwmPerc;
  pHandle->Pwm_H_L.AdcThresholdHighPerc = BemfAdcConfig->AdcThresholdHighPerc;
  pHandle->Pwm_H_L.AdcThresholdLowPerc = BemfAdcConfig->AdcThresholdLowPerc;
  pHandle->Pwm_H_L.SamplingPointOff = BemfAdcConfig->SamplingPointOff;
  pHandle->Pwm_H_L.SamplingPointOn = BemfAdcConfig->SamplingPointOn; 
  pHandle->ZcRising2CommDelay = *ZcRising2CommDelay;
  pHandle->ZcFalling2CommDelay = *ZcFalling2CommDelay;
  pHandle->DemagParams.DemagMinimumSpeedUnit = bemfAdcDemagConfig->DemagMinimumSpeedUnit;
  pHandle->DemagParams.DemagMinimumThreshold = bemfAdcDemagConfig->DemagMinimumThreshold;
  pHandle->OnSensingEnThres = *OnSensingEnThres;
  pHandle->OnSensingDisThres = *OnSensingDisThres;
  pHandle->ComputationDelay = (uint8_t) (*ComputationDelay);
}

/**
  * @brief  Gets the parameters for bemf sensing during pwm off-time
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  BemfAdcConfig: thresholds and sampling time parameters
  * @param  ZcRising2CommDelay: delay between rising bemf zero crossing parameter and step commutation
  * @param  ZcFalling2CommDelay: delay between falling bemf zero crossing parameter and step commutation
  * @param  BemfAdcDemagConfig: demagnetization parameters
  * @param  OnSensingEnThres: Minimum duty cycle for on-sensing activation
  * @param  OnSensingDisThres: Minimum duty cycle for on-sensing Deactivationg
  * @param  ComputationDelay: Selection of the period for zero crossing to step change delay computation  
  */
void BADC_GetBemfSensorlessParam(Bemf_ADC_Handle_t *pHandle, Bemf_Sensing_Params *BemfAdcConfig, uint16_t *ZcRising2CommDelay, uint16_t *ZcFalling2CommDelay,
                                   Bemf_Demag_Params *BemfAdcDemagConfig, uint16_t *OnSensingEnThres, uint16_t *OnSensingDisThres, uint16_t *ComputationDelay)
{  
  BemfAdcConfig->AdcThresholdPwmPerc =   pHandle->Pwm_H_L.AdcThresholdPwmPerc;
  BemfAdcConfig->AdcThresholdHighPerc = pHandle->Pwm_H_L.AdcThresholdHighPerc;
  BemfAdcConfig->AdcThresholdLowPerc = pHandle->Pwm_H_L.AdcThresholdLowPerc;
  BemfAdcConfig->SamplingPointOff = pHandle->Pwm_H_L.SamplingPointOff;
  BemfAdcConfig->SamplingPointOn = pHandle->Pwm_H_L.SamplingPointOn;
  *ZcRising2CommDelay = pHandle->ZcRising2CommDelay;
  *ZcFalling2CommDelay = pHandle->ZcFalling2CommDelay;
  BemfAdcDemagConfig->DemagMinimumSpeedUnit = pHandle->DemagParams.DemagMinimumSpeedUnit;
  BemfAdcDemagConfig->DemagMinimumThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  *OnSensingEnThres = pHandle->OnSensingEnThres;
  *OnSensingDisThres = pHandle->OnSensingDisThres;
  *ComputationDelay = (uint16_t) pHandle->ComputationDelay;
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
