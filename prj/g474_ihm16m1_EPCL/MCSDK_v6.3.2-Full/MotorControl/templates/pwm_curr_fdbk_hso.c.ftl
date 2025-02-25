<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Current Feedback component of the Motor Control SDK:
  *
  *           * current sensing
  *           * regular ADC conversion execution
  *           * space vector modulation
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
  * @ingroup pwm_curr_fdbk  
  */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"
#include "parameters_conversion.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup pwm_curr_fdbk PWM & Current Feedback
  *
  * @brief PWM & Current Feedback components of the Motor Control SDK
  *
  * These components fulfill two functions in a Motor Control subsystem:
  *
  * - The generation of the Space Vector Pulse Width Modulation on the motor's phases
  * - The sampling of the actual motor's phases current and voltages
  *
  * Both these features are closely related as the instants when the values of the phase currents
  * and voltages should be sampled by the ADC channels are basically triggered by the timers used to generate
  * the duty cycles for the PWM. For information, please check [PWM modulation documentation](PWM_Modulators.md) 
  *
  * Several implementation of PWM and Current Feedback components are provided by the Motor Control
  * SDK to account for the specificities of the application:
  *
  * - The selected MCU: the number of ADCs available on a given MCU, the presence of internal
  * comparators or OpAmps, for instance, lead to different implementation of this feature
  * - The Current sensing topology also has an impact on the firmware: implementations are provided
  * for Insulated Current Sensors and Three Shunt resistors current sensing topologies
  *
  * All these implementations are built on a base PWM & Current Feedback component that they extend
  * and that provides the functions and data that are common to all of them. This base component is
  * never used directly as it does not provide a complete implementation of the features. Rather,
  * its handle structure (PWMC_Handle) is reused by all the PWM & Current Feedback specific
  * implementations and the functions it provides form the API of the PWM and Current feedback feature.
  * Calling them results in calling functions of the component that actually implement the feature.
  * See PWMC_Handle for more details on this mechanism.
  * @{
  */

static bool CurrentReconstruction(PWMC_Handle_t* pHandle, Currents_Irst_t* pIrst_in_pu, Currents_Irst_t* pIrst_out_pu);

/**
  * @brief  Switches PWM generation off, inactivating the outputs.
  * @param  pHandle Handle on the target instance of the PWMC component
  */
__weak void PWMC_SwitchOffPWM( PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm( pHandle );
}

/**
  * @brief  Switches PWM generation on
  * @param  pHandle Handle on the target instance of the PWMC component
  */
__weak void PWMC_SwitchOnPWM( PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm( pHandle );
}

/**
  * @brief  Calibrates ADC current conversions by reading the offset voltage
  *         present on ADC pins when no motor current is flowing in.
  *
  * This function should be called before each motor start-up.
  *
  * @param  pHandle Handle on the target instance of the PWMC component
  * @param  action Can be #CRC_START to initialize the offset calibration or
  *         #CRC_EXEC to execute the offset calibration.
  * @retval bool true if the current calibration has been completed, false if it is
  *         still ongoing.
  */
__weak bool PWMC_CurrentReadingCalibr( PWMC_Handle_t * pHandle, CRCAction_t action )
{

	bool retVal = false;

	if ( pHandle->OffCalibrWaitTicks == 0u )
	{
		/* Calibration time set to zero, cannot perform offset measurement */

		/* Switch off the PWM, safe default */
		pHandle->pFctSwitchOffPwm( pHandle );

		/* Keep returning false, so the state machine cannot progress, safe default */
		retVal = false;
	}

	if ( action == CRC_START )
	{
		/* Begin the offset measurement process */

		/* Initialize counter, which counts down to zero */
		pHandle->OffCalibrWaitTimeCounter = pHandle->OffCalibrWaitTicks;

  	/* Perform some essential PWM preparation steps */
		pHandle->pFctPreparePWM( pHandle );

		/* Set Offset Measurement mode, initialize default offsets and filter constant,
		 * force PWM duty to 50% */
		pHandle->pFctStartOffsetCalibration( pHandle );

		pHandle->pFctSwitchOnPwm( pHandle );
	}
	else if ( action == CRC_EXEC )
	{
		if ( pHandle->OffCalibrWaitTimeCounter > 0u ) pHandle->OffCalibrWaitTimeCounter--;

		if ( pHandle->OffCalibrWaitTimeCounter == 0u )
		{
			/* PWM is switched off first, since the next step allows FOC to control the PWM signal,
			 * and it does not contain reliable values at this stage */
			pHandle->pFctSwitchOffPwm( pHandle );

			/* Finish offset measurement mode by using the filtered values as the offsets from now on.
			 * PWM is no longer forced to 50% duty */
			pHandle->pFctFinishOffsetCalibration( pHandle );

			/* Tell caller offset calibration is done */
			retVal = true;
		}
	}

	return retVal;

}

/**
  * @brief  Switches power stage Low Sides transistors on.
  *
  * This function is meant for charging boot capacitors of the driving
  * section. It has to be called on each motor start-up when using high
  * voltage drivers.
  *
  * @param  pHandle: handle on the target instance of the PWMC component
  *        ticks:    Timer ticks value to be applied.
  *                  Min value: 0 (low sides ON)
  *                  Max value: PWM_PERIOD_CYCLES/2 (low sides OFF)  
  */
__weak void PWMC_TurnOnLowSides( PWMC_Handle_t * pHandle, const uint32_t ticks )
{
  pHandle->pFctTurnOnLowSides( pHandle, ticks );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/*
  * @brief  Manages HW overcurrent protection.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void *PWMC_OCP_Handler(PWMC_Handle_t *pHandle)
{
  void *tempPointer; //cstat !MISRAC2012-Rule-8.13
#ifdef NULL_PTR_CHECK_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    tempPointer = MC_NULL;
  }
  else
  {
#endif
    if (false == pHandle->BrakeActionLock)
    {
      if (ES_GPIO == pHandle->LowSideOutputs)
      {
        LL_GPIO_ResetOutputPin(pHandle->pwm_en_u_port, pHandle->pwm_en_u_pin);
        LL_GPIO_ResetOutputPin(pHandle->pwm_en_v_port, pHandle->pwm_en_v_pin);
        LL_GPIO_ResetOutputPin(pHandle->pwm_en_w_port, pHandle->pwm_en_w_pin);
      }
      else
      {
        /* Nothing to do */
      }
    }
    else
    {
      /* Nothing to do */
    }
    pHandle->OverCurrentFlag = true;
    tempPointer = &(pHandle->Motor);
#ifdef NULL_PTR_CHECK_PWR_CUR_FDB
  }
#endif
  return (tempPointer);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/*
  * @brief  manages driver protection.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void *PWMC_DP_Handler(PWMC_Handle_t *pHandle)
{
  void *tempPointer; //cstat !MISRAC2012-Rule-8.13
#ifdef NULL_PTR_CHECK_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    tempPointer = MC_NULL;
  }
  else
  {
#endif
    if (false == pHandle->BrakeActionLock)
    {
      if (ES_GPIO == pHandle->LowSideOutputs)
      {
        LL_GPIO_ResetOutputPin(pHandle->pwm_en_u_port, pHandle->pwm_en_u_pin);
        LL_GPIO_ResetOutputPin(pHandle->pwm_en_v_port, pHandle->pwm_en_v_pin);
        LL_GPIO_ResetOutputPin(pHandle->pwm_en_w_port, pHandle->pwm_en_w_pin);
      }
      else
      {
        /* Nothing to do */
      }
    }
    else
    {
      /* Nothing to do */
    }
    pHandle->driverProtectionFlag = true;
    tempPointer = &(pHandle->Motor);
#ifdef NULL_PTR_CHECK_PWR_CUR_FDB
  }
#endif
  return (tempPointer);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/*
  * @brief  Manages HW overvoltage protection.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  *         TIMx: timer used for PWM generation
  */
__weak void *PWMC_OVP_Handler(PWMC_Handle_t *pHandle, TIM_TypeDef *TIMx)
{
  void *tempPointer; //cstat !MISRAC2012-Rule-8.13
#ifdef NULL_PTR_CHECK_PWR_CUR_FDB
  if (MC_NULL == pHandle)
  {
    tempPointer = MC_NULL;
  }
  else
  {
#endif
    TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
    pHandle->OverVoltageFlag = true;
    pHandle->BrakeActionLock = true;
    tempPointer = &(pHandle->Motor);
#ifdef NULL_PTR_CHECK_PWR_CUR_FDB
  }
#endif
  return (tempPointer);
}

/*
  * @brief  Checks if an overcurrent occurred since last call.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Returns #MC_OVER_CURR if an overcurrent has been
  *                  detected since last method call, #MC_NO_FAULTS otherwise.
  */
__weak uint16_t PWMC_IsFaultOccurred(PWMC_Handle_t *pHandle)
{
  uint16_t retVal = MC_NO_FAULTS;

  if (true == pHandle->OverVoltageFlag)
  {
    retVal = MC_OVER_VOLT;
    pHandle->OverVoltageFlag = false;
  }
  else
  {
    /* Nothing to do */
  }

  if (true == pHandle->OverCurrentFlag)
  {
    retVal |= MC_OVER_CURR;
    pHandle->OverCurrentFlag = false;
  }
  else
  {
    /* Nothing to do */
  }

  if (true == pHandle->driverProtectionFlag)
  {
    retVal |= MC_DP_FAULT;
    pHandle->driverProtectionFlag = false;
  }
  else
  {
    /* Nothing to do */
  }

  return (retVal);
}


/**
  * @brief  Sets the over current threshold to be used
  *
  * The value to be set is relative to the VDD_DAC DAC reference voltage with
  * 0 standing for 0 V and 65536 standing for VDD_DAC.
  *
  * @param  pHandle handle on the target instance of the PWMC component
  * @param  hDACVref Value of DAC reference expressed as 16bit unsigned integer
  */
__weak void PWMC_OCPSetReferenceVoltage( PWMC_Handle_t * pHandle, uint16_t hDACVref )
{
  if ( pHandle->pFctOCPSetReferenceVoltage )
  {
    pHandle->pFctOCPSetReferenceVoltage( pHandle, hDACVref );
  }
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation off.
 * @param pCallBack pointer on the callback
 * @param pHandle PWMC handler data structure address
 *
 */
__weak void PWMC_RegisterSwitchOffPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                        PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation on.
 * @param pCallBack pointer on the callback
 * @param pHandle PWMC handler data structure address
 *
 */
__weak void PWMC_RegisterSwitchonPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                       PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to execute a calibration
 *        of the current sensing system.
 * @param pCallBack pointer on the callback
 * @param pHandle PWMC handler data structure address
 *
 */
__weak void PWMC_RegisterReadingCalibrationCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctCurrReadingCalib = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to turn low sides on.
 * @param pCallBack pointer on the callback
 * @param pHandle PWMC handler data structure address
 *
 */
__weak void PWMC_RegisterTurnOnLowSidesCallBack( PWMC_TurnOnLowSides_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctTurnOnLowSides = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to compute ADC sampling point
 * @param pCallBack pointer on the callback
 * @param pHandle PWMC handler data structure address
 *
 */
__weak void PWMC_RegisterSampPointSectXCallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctSetADCSampPointSectX = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to set the reference
 *        voltage for the over current protection
 * @param pHandle PWMC handler data structure address
 *
 */
__weak void PWMC_RegisterOCPSetRefVoltageCallBack( PWMC_SetOcpRefVolt_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctOCPSetReferenceVoltage = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to call PWMC instance IRQ handler
 * @param pHandle PWMC handler data structure address
 *
 * @note this function is deprecated.
 */
__weak void PWMC_RegisterIrqHandlerCallBack( PWMC_IrqHandler_Cb_t pCallBack,
                                      PWMC_Handle_t * pHandle )
{
  pHandle->pFctIrqHandler = pCallBack;
}

/**
 * @brief Retrieves phase voltage and currents measurements from ADCs 
 * 
 * This routine shall be called at the beginning of the current controller task, in order
 * to get the phase voltage and currents measurements converted in *a�* coordinates and 
 * expressed per-unit.
 * 
 * @param pHandle PWMC handler data structure address
 * @param Out phase voltage and current measurements data structure
 *
 */
void PWMC_GetMeasurements( PWMC_Handle_t * pHandle, Measurement_Output_t* Out )
{
	/* get raw currents and voltages from HW*/
  pHandle->pFctGetMeasurements( pHandle, &pHandle->Irst_in_raw_pu, &pHandle->Urst_in_pu);

    /* Perform current reconstruction */
  if ((pHandle->flagEnableCurrentReconstruction) && (PWMC_IsInPwmControl(pHandle) == true )) /* TODO to be generated according to HALL or shunt topology*/
  {
    pHandle->currentsamplingfault = CurrentReconstruction(pHandle, &pHandle->Irst_in_raw_pu, &pHandle->Irst_in_pu);
  }
  else
  {
    /* With current reconstruction disabled, use the samples directly */
    pHandle->Irst_in_pu = pHandle->Irst_in_raw_pu;
  }

  /* Clarke Transformations *************************************************************************/

  /* Clarke transforms */
  pHandle->Iab_in_pu = MCM_Clarke_Current(pHandle->Irst_in_pu);
  pHandle->Uab_in_pu = MCM_Clarke_Voltage(pHandle->Urst_in_pu);
  Out->Iab_pu = pHandle->Iab_in_pu;                /* Measured current vector */
  Out->Uab_pu = pHandle->Uab_in_pu;                /* Measured voltage vector */

}

/**
 * @brief Retrieves auxiliary measurements from ADCs 
 * 
 * This routine is used to get auxliary measurement from ADC, usually it is the potentiometer
 * measurements.
 * 
 * @param pHandle PWMC handler data structure address
 * @param pAuxMeasurement auxliary measurments
 *
 */
void PWMC_GetAuxAdcMeasurement( PWMC_Handle_t * pHandle, fixp30_t* pAuxMeasurement )
{
	pHandle->pFctGetAuxAdcMeasurement( pHandle, pAuxMeasurement );
}

/**
 * @brief Retrieves the reconstruction data structure 
 * 
 * @param pHandle PWMC handler data structure address
 * @param pData reconstruction data strucure adress
 *
 */
uint8_t PWMC_GetCurrentReconstructionData( PWMC_Handle_t *pHandle, CurrentReconstruction_t *pData)
{
	return pHandle->pFctGetCurrentReconstructionData( pHandle, pData);
}

/**
 * @brief Gets the phase currents and voltages offsets calibration value 
 * 
 * @param pHandle PWMC handler data structure address
 * @param PolarizationOffsets offsets calibration value
 *
 */
void PWMC_GetOffsets( PWMC_Handle_t *pHandle, PolarizationOffsets_t *PolarizationOffsets )
{
	pHandle->pFctGetOffsets( pHandle, PolarizationOffsets );
}

/**
 * @brief Sets the phase currents and voltages offsets calibration value 
 * 
 * @param pHandle PWMC handler data structure address
 * @param PolarizationOffsets auxliary measurments
 *
 */
void PWMC_SetOffsets( PWMC_Handle_t *pHandle, PolarizationOffsets_t *PolarizationOffsets )
{
	pHandle->pFctSetOffsets( pHandle, PolarizationOffsets );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
 * @brief Performs the space vector modulation
 * 
 * This routine shall be called at the end of the current controller task 
 * when the *a�* duty cycle vector is available in order to perform the SVPWM. 
 * For more information about PWM modulation type please refer to PWM modulators [documentation](docs/PWM_Modulators.md).
 * 
 * @param pHandle PWMC handler data structure address.
 * @param In input duty cycle vector in *a�* coordinates.
 * @retval uint16_t error.
 *
 */
uint16_t PWMC_SetPhaseDuty( PWMC_Handle_t * pHandle, Duty_Dab_t* In )
{

  // PolPulse Dab_out_pu override
  if (POLPULSE_getOverruleDuty(pHandle->polpulseHandle))
  {
    *In = POLPULSE_getDutyAB(pHandle->polpulseHandle);
  }

  /* Inverse Clarke */
  pHandle->Drst_unmodulated_out_pu = MCM_Inv_Clarke_Duty(In);

  /* Space Vector Modulation */
  pHandle->Drst_out_pu = MCM_Modulate(&pHandle->Drst_unmodulated_out_pu, pHandle->modulationMode);

  int32_t wTimePhA, wTimePhB, wTimePhC;
  /* Takes Drst, which are the duties in fixp30_t, between -1.0 and +1.0 */

  // 0.0 = 50% high, 50% low, the middle point			quarterPeriodCnt
  // +1.0 = 100% high
  // -1.0 = 100% low

  /* Calculate compare values for the phases */
  // Use PWM Mode 2
  // Phases high is in middle of period, where CNT == PRD (PRD is where the counter is reversed, at halfPeriodCnt)
  // CNT goes from 0 to PRD to 0
  // CNT near 0 means mostly high, CNT near PRD means mostly low
  // Difference from neutral is +quarterPeriodCnt and -quarterPeriodCnt
  fixpFmt_t shift = 30 + pHandle->duty_sf.fixpFmt;
  wTimePhA = FIXP_MPY(pHandle->Drst_out_pu.R, pHandle->duty_sf.value, shift) + pHandle->QuarterPeriodCnt;	// ToDo: New FIXP function, naming
  wTimePhB = FIXP_MPY(pHandle->Drst_out_pu.S, pHandle->duty_sf.value, shift) + pHandle->QuarterPeriodCnt;
  wTimePhC = FIXP_MPY(pHandle->Drst_out_pu.T, pHandle->duty_sf.value, shift) + pHandle->QuarterPeriodCnt;

  /* Write to pHandle */
  pHandle->CntPhA = ( uint16_t )FIXP_sat(wTimePhA,(PWM_PERIOD_CYCLES/2),0);
  pHandle->CntPhB = ( uint16_t )FIXP_sat(wTimePhB,(PWM_PERIOD_CYCLES/2),0);
  pHandle->CntPhC = ( uint16_t )FIXP_sat(wTimePhC,(PWM_PERIOD_CYCLES/2),0);

  /* Call SetADCSampPointSingle to set compare values in TIM registers */
  return ( pHandle->pFctSetADCSampPointSectX( pHandle ) );
}

/**
 * @brief  Switch on low sides, switch off high sides, shorting the motor.
 * @param  pHandle Handle on the target instance of the PWMC component
 */
void PWMC_PWMShort( PWMC_Handle_t * pHandle )
{
	pHandle->pFctPwmShort( pHandle );
}

/**
 * @brief  Returns the PWM control status
 * @param  pHandle Handle on the target instance of the PWMC component
 * @retval bool PWM control status;
 *         - true: PWM are in control
 *         - false: PWM are not in control (e.g switched off, forced)
 */
bool PWMC_IsInPwmControl( PWMC_Handle_t * pHandle )
{
	// Returns true if the written duties are being obeyed

	return pHandle->pFctIsInPwmControl( pHandle );
}

/**
 * @brief  Returns PWM enabling status
 * @param  pHandle Handle on the target instance of the PWMC component
 * @retval bool PWM control status;
 *         - true: PWM outputs are enabled
 *         - false: PWM outputs are disabled
 */
bool PWMC_IsPwmEnabled( PWMC_Handle_t * pHandle )
{
	return pHandle->pFctIsPwmEnabled( pHandle );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
static inline bool CurrentReconstruction(PWMC_Handle_t* pHandle, Currents_Irst_t* pIrst_in_pu, Currents_Irst_t* pIrst_out_pu)
{
  CurrentReconstruction_t crData;
  uint8_t numValid = PWMC_GetCurrentReconstructionData(pHandle, &crData);

  bool fault = false;

  /* Copy raw data */
  pIrst_out_pu->R = pIrst_in_pu->R;
  pIrst_out_pu->S = pIrst_in_pu->S;
  pIrst_out_pu->T = pIrst_in_pu->T;

  if (numValid == 3)
  {
    /* All three current samples are valid, current reconstruction is not required */
  }
  else
  {
    /* Current reconstruction is required */
    if (numValid == 2)
    {
      /* Reconstruct */
      if (!crData.validR) pIrst_out_pu->R = - pIrst_in_pu->S - pIrst_in_pu->T;	// Only one of these will be true
      if (!crData.validS) pIrst_out_pu->S = - pIrst_in_pu->R - pIrst_in_pu->T;
      if (!crData.validT) pIrst_out_pu->T = - pIrst_in_pu->R - pIrst_in_pu->S;
    }
    else
    {
      /* One or zero currents could be measured, which should never happen */
      fault = true;
    }
  }
  return fault;
} /* end of CurrentReconstruction() */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

