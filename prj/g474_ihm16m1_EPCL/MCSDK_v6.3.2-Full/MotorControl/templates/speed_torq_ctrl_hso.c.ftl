<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
 ******************************************************************************
 * @file    speed_torq_ctrl_hso.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions of the speed and torque control
 *          HSO component
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
  * @ingroup speed_torq_ctrl_hso
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_torq_ctrl_hso.h"
#include "parameters_conversion.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednTorqCtrl
  * @{
  */

/** @defgroup speed_torq_ctrl_hso Speed & torque control for HSO
  *
  * @brief speed and torque control component for the High Sensitivity Observer
  *
  * The aim of this component is to compute a @f$Iqd@f$ reference in order to maintain 
  * the targeted speed when closed loop speed control is activated or maintain the targeted 
  * torque if the closed loop torque control is activated.
  *
  * For speed regulation it uses the PID speed regulator as shown in the picture below. It 
  * also embeds a [DC bus protection](bus_protection.md) feature that acts on the torque when DC bus reaches
  * the upper or lower DC bus boundaries.
  *  
  * ![](speed_torque_ctrl.png)
  *
  * The speed and torque control component embeds a DC bus protection feature useful during development phase.
  * Iq reference current limitation (minimum and  maximum) are computed dynamically considering the level of measured Bus voltage, the HW board limitations and
  * maximum allowed current. At low speed (below 1Hz), baseline current limit is used. In Close Loop speed, those limitations are used in PI Speed to limit the computed output Iq reference. 
  * In Open loop and Close loop current, used also to limit the Iq reference.
  *
  * As shown in the picture below:
  *  - the acceleration torque will be limited if the measured DC bus is within the range acceleration limit range
  *  - the regenerative torque will be limited if the measured DC bus is within the range regeneration limit range
  *
  *  ![](dc_bus_protection.png)
  *
  * |                   | Iq reference Min                          | Iq Reference Max                         | 
  * | ----------------- | ----------------------------------------- | ---------------------------------------- |
  * | Running forwards  | - (Regenerative factor) * Maximum Current | (Acceleration factor ) * Maximum Current |
  * | Running backwards | -(Acceleration factor )* Maximum Current  | (Regenerative factor) * Maximum Current  |
  *
  * @{
  */
  
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/

/* Performs the CPU load measure of FOC main tasks. */

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/

/* USER CODE BEGIN Private Functions */
<#if MC.M1_BUS_PROTECTION == true>
static void BusProtection(STC_Handle_t* pHandle, fixp30_t i_max_pu, fixp30_t fe_pu, fixp30_t u_dc_pu, fixp30_t *pOut_iq_max, fixp30_t *pOut_iq_min);
static void CurrentLimiting(STC_Handle_t* pHandle, const fixp30_t bus_voltage_pu, fixp30_t* pAccel_curr_factor, fixp30_t* pRegen_curr_factor);
</#if>
static fixp30_t CircleLimitation(fixp30_t reference, const fixp30_t limit);
/* USER CODE END Private Functions */

/**
  * @brief  Initializes speed & torque control component.
  *         It Should be called during Motor control middleware initialization
  * @param  pHandle speed & torque control handler
  * @param  flashParams flash parameters
  */
void STC_Init(STC_Handle_t* pHandle, FLASH_Params_t const *flashParams)
{

  pHandle->regen_volt_high_pu       = FIXP30(flashParams->board.limitRegenHigh / flashParams->scale.voltage);
  pHandle->regen_volt_low_pu        = FIXP30(flashParams->board.limitRegenLow / flashParams->scale.voltage);
  pHandle->regen_volt_oneoverrange  = FIXP24(1.0f / ((flashParams->board.limitRegenHigh - flashParams->board.limitRegenLow) / flashParams->scale.voltage));
  pHandle->accel_volt_high_pu       = FIXP30(flashParams->board.limitAccelHigh / flashParams->scale.voltage);
  pHandle->accel_volt_low_pu        = FIXP30(flashParams->board.limitAccelLow / flashParams->scale.voltage);
  pHandle->accel_volt_oneoverrange  = FIXP24(1.0f / ((flashParams->board.limitAccelHigh - flashParams->board.limitAccelLow) / flashParams->scale.voltage));

  float maxCurrent_A 	= flashParams->motor.maxCurrent;
  fixp30_t current_limit = FIXP30(maxCurrent_A / flashParams->scale.current);
  PIDREG_SPEED_init(&pHandle->PIDSpeed, flashParams->scale.current, flashParams->scale.frequency, TF_REGULATION_RATE);
  PIDREG_SPEED_setKp_si(&pHandle->PIDSpeed, flashParams->PIDSpeed.pidSpdKp);
  PIDREG_SPEED_setKi_si(&pHandle->PIDSpeed, flashParams->PIDSpeed.pidSpdKi);
  /* Speedramp to 1Hz/s */
  fixp24_t hz_pu_psec = FIXP_MPY(1000, FIXP24(1.0f / flashParams->scale.frequency), 0); // 24 = 0 x 24
  pHandle->speed_ramp_pu_per_isr = FIXP24_mpy(hz_pu_psec, FIXP30(1.0f / TF_REGULATION_RATE)); // 30 = 24 x 30 >> 24
  /* Initialize current limiting */
  pHandle->I_max_pu = FIXP30( maxCurrent_A / flashParams->scale.current );
  pHandle->speed_ref_pu = FIXP30( ( (float_t)POLE_PAIR_NUM*(float_t)DEFAULT_TARGET_SPEED_RPM/(float_t)60 ) / flashParams->scale.frequency );
  pHandle->speed_ref_active_pu = pHandle->speed_ref_pu;
  pHandle->Idq_ref_pu.Q = FIXP30( (float_t)DEFAULT_TORQUE_COMPONENT_A / flashParams->scale.current );
  pHandle->Idq_ref_pu.D = FIXP30( (float_t)DEFAULT_FLUX_COMPONENT_A / flashParams->scale.current );
  pHandle->OneOverMaxIq_pu = FIXP24(1.0f / FIXP30_toF(pHandle->I_max_pu));

  PIDREG_SPEED_setOutputLimits(&pHandle->PIDSpeed, current_limit, -current_limit);

}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Performs speed and torque control.
  *         
  * @param  pHandle speed & torque control handler
  * @param  In input data structure 
  */
void STC_Run(STC_Handle_t* pHandle, STC_input_t *In)
{
  /* Speed reference ramping */
  fixp30_t speed_ramp = pHandle->speed_ramp_pu_per_isr;
  fixp30_t speed_delta_pu = pHandle->speed_ref_active_pu - pHandle->speed_ref_ramped_pu;
  pHandle->speed_ref_ramped_pu += FIXP_sat(speed_delta_pu, speed_ramp, -speed_ramp);

  /* speed feedback */
  fixp30_t Fe_pu        = In->emfSpeed; /* Electrical frequency based on EMF magnitude */
  fixp30_t Fe_AngSpd_pu = In->SpeedLP;  /* Electrical frequency based on angle change */

  /* Current Reference Calculation */
  /* (Calculate Id_ref, Iq_ref for the current controller, based on
   * setpoints, speed, bus voltage and maximum current)
   */
  {
    fixp30_t Id_ref_pu;
    fixp30_t I_max_pu = pHandle->I_max_pu;

    /* Limit the D-current reference */
    Id_ref_pu = FIXP_sat(pHandle->Idq_ref_pu.D, I_max_pu, -I_max_pu);

    fixp30_t Iq_max, Iq_min;

<#if MC.M1_BUS_PROTECTION == true>
    /* Bus Protection, calculates limits for Iq reference to protect bus against over/undervoltages */
    BusProtection(pHandle, I_max_pu, Fe_pu, In->Udcbus_in_pu, &Iq_max, &Iq_min);
    
    /* Set integrator limits in speed PID */
    PIDREG_SPEED_setOutputLimits(&pHandle->PIDSpeed, Iq_max, Iq_min);
<#else>
    Iq_max = I_max_pu;
    Iq_min = -I_max_pu;
</#if>

    fixp30_t Iq_ref_spd_pu = FIXP30(0.0f); /* Q-current reference from speed controller */

    if ((pHandle->SpeedControlEnabled) && ( true == In->pwmControlEnable))
    {
      /* Run the speed PID controller */
      if (pHandle->PIDSpeed.Ki_fps.value == 0) PIDREG_SPEED_setUi_pu(&pHandle->PIDSpeed, FIXP30(0.0f)); /* Clear speed integrator if Ki == 0 */
      Iq_ref_spd_pu = PIDREG_SPEED2_run(&pHandle->PIDSpeed, pHandle->speed_ref_ramped_pu - Fe_pu, pHandle->speed_ref_ramped_pu - Fe_AngSpd_pu);
      pHandle->Iq_ref_spd_pu = Iq_ref_spd_pu;
    }
    else
    {
      /* While speed PID is not in control, set the integrator to present q-current*/
      PIDREG_SPEED_setUi_pu(&pHandle->PIDSpeed, pHandle->pCurrCtrl->Idq_in_pu.Q);
      if (true == In->closedLoopEnabled)
      {
        pHandle->Iq_ref_spd_pu = FIXP30(0.0f);
        pHandle->speed_ref_ramped_pu = Fe_AngSpd_pu; /* copy present speed to ramp generator */
      }
    }

    fixp30_t Iq_ref_active_pu = FIXP30(0.0f);

    /* Select the Q-current reference and limit where required */
    if (true == pHandle->SpeedControlEnabled)
    {
      /* Use Q-current reference from speed controller */
      Iq_ref_active_pu = Iq_ref_spd_pu; /* Already limited by Iq_max, Iq_min by PID */
    }
    else
    {
      /* No speed control */
      if (true == In->closedLoopEnabled)
      {
        /* Closed loop current */
        /* Use Q-current reference from MCI or throttle input */
        Iq_ref_active_pu = FIXP_sat(pHandle->Iq_ref_active_pu, Iq_max, Iq_min);
      }
      else
      {
        /* Open loop current */
        /* Use Q-current reference from MCI */
        Iq_ref_active_pu = FIXP_sat(pHandle->Idq_ref_pu.Q, Iq_max, Iq_min);
      }
    }

    /* Calculate Q-current remaining in circle */
    fixp30_t Iq_ref_circle_max = CircleLimitation(Id_ref_pu, I_max_pu);

    /* Actual Q-current reference is the active reference limited by the circle */
    fixp30_t Iq_ref_pu = FIXP_sat(Iq_ref_active_pu, Iq_ref_circle_max, -Iq_ref_circle_max);

    /* Store the limited references */
    pHandle->Idq_ref_limited_pu.D = Id_ref_pu;
    pHandle->Idq_ref_limited_pu.Q = Iq_ref_pu;
  }
} /* end of MotorControl() */

<#if MC.M1_BUS_PROTECTION == true>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
static inline void CurrentLimiting(STC_Handle_t* pHandle, const fixp30_t bus_voltage_pu, fixp30_t* pAccel_curr_factor, fixp30_t* pRegen_curr_factor)
{

  /* active_limits tell the user what limits are active */
  FOC_ActiveCurrentLimits_u active_limits;
  active_limits.u16 = 0;

  /* Calculate maximum allowed forwards and reverse current, implementing the following limits;
   * - Powerstage maximum current
   * - Motor maximum current
   * - Maximum regenerative current
   * - Maximum acceleration current
   * - Maximum forward current
   * - Maximum reverse current
   * - Maximum acceleration current based on bus voltage (less current allowed when battery voltage sags)
   * - Maximum regenerative current based on bus voltage (less current allowed when battery voltage peaks)
   */

  /* regenerative factor */
  fixp30_t regenfactor = FIXP30(0.0f);
  if (bus_voltage_pu < pHandle->regen_volt_low_pu)
  {
    /* Under the lower limit, allow full regenerative current */
    regenfactor = FIXP30(1.0f);
  }
  else if (bus_voltage_pu > pHandle->regen_volt_high_pu)
  {
    /* Over the upper limit, allow no regenerative current */
    regenfactor = FIXP30(0.0f);
    active_limits.bit.FOC_ACL_Regen_Busvolt = 1;
  }
  else
  {
    /* Between lower and upper limit, allow partial regenerative current */
    fixp30_t pos = bus_voltage_pu - pHandle->regen_volt_low_pu;
    fixp24_t factor = FIXP30_mpy(pos, pHandle->regen_volt_oneoverrange);
    factor = FIXP_sat(factor, FIXP24(1.0f), FIXP24(0.0f));
    regenfactor = FIXP30(1.0f) - (factor << (30-24));
    active_limits.bit.FOC_ACL_Regen_Busvolt = 1;
  }
  pHandle->regen_factor = regenfactor;
  *pRegen_curr_factor = regenfactor;

  /* acceleration factor */
  fixp30_t accelfactor = FIXP30(0.0f);
  if (bus_voltage_pu > pHandle->accel_volt_high_pu)
  {
    /* Over the upper limit, allow full acceleration current */
    accelfactor = FIXP30(1.0f);
  }
  else if (bus_voltage_pu < pHandle->accel_volt_low_pu)
  {
    /* Under the lower limit, allow no acceleration current */
    accelfactor = FIXP30(0.0f);
    active_limits.bit.FOC_ACL_Accel_Busvolt = 1;
  }
  else
  {
    /* Between lower and upper limit, allow partial acceleration current */
    fixp30_t pos = bus_voltage_pu - pHandle->accel_volt_low_pu;
    fixp24_t factor = FIXP30_mpy(pos, pHandle->accel_volt_oneoverrange);
    factor = FIXP_sat(factor, FIXP24(1.0f), FIXP24(0.0f));
    accelfactor = factor << (30-24);
    active_limits.bit.FOC_ACL_Accel_Busvolt = 1;
  }
  pHandle->accel_factor = accelfactor;
  *pAccel_curr_factor = accelfactor;

  /* Store active limits for user interface */
  pHandle->active_limits.u16 = active_limits.u16;
} /* end of CurrentLimiting() */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
static inline void BusProtection(STC_Handle_t* pHandle, fixp30_t i_max_pu, fixp30_t fe_pu, fixp30_t u_dc_pu, fixp30_t *pOut_iq_max, fixp30_t *pOut_iq_min)
{
  /* Calculate acceleration and regenerative maximum Iq-current, based on bus voltage and maximum current limit */
  fixp30_t accel_curr_factor;	/* Acceleration current factor, 1.0f while unlimited, reduces when bus voltage drops */
  fixp30_t regen_curr_factor;	/* Regeneration current factor, 1.0f while unlimited, reduces when bus voltage rises */
  CurrentLimiting(pHandle, u_dc_pu, &accel_curr_factor, &regen_curr_factor);

  /* The current limit is ignored around zero speed, below 1.0 Hz */
  //fixp30_t limit =  FIXP30(1.0f / scaleParams->frequency);
  if (FIXP_abs(fe_pu) > pHandle->speedLimit)
  {
    if (fe_pu > 0)
    {
      /* Running forwards */
      *pOut_iq_max = FIXP30_mpy(accel_curr_factor, i_max_pu);	/* Forwards acceleration limit */
      *pOut_iq_min = FIXP30_mpy(-regen_curr_factor, i_max_pu);	/* Forwards regeneration limit */
    }
    else
    {
      /* Running backwards */
      *pOut_iq_max = FIXP30_mpy(regen_curr_factor, i_max_pu);	/* Reverse regeneration limit */
      *pOut_iq_min = FIXP30_mpy(-accel_curr_factor, i_max_pu);	/* Reverse acceleration limit */
    }
  }
  else
  {
    /* Use baseline current limit */
    *pOut_iq_max = i_max_pu;
    *pOut_iq_min = -i_max_pu;
  }
}
</#if>

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
static inline fixp30_t CircleLimitation(fixp30_t reference, const fixp30_t limit)
{
  fixp30_t limit_squared = FIXP30_mpy(limit, limit);
  fixp30_t reference_squared = FIXP30_mpy(reference, reference);
  fixp30_t remainder_squared = limit_squared - reference_squared;
  remainder_squared = FIXP_sat(remainder_squared, FIXP30(1.0f), FIXP30(0.0f));
  return FIXP30_sqrt(remainder_squared);
} /* end of CircleLimitation() */

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
