<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
 ******************************************************************************
 * @file    mc_curr_ctrl.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions of the current controller
 *          component
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
 * @ingroup curr_ctrl
 */

/* Includes ------------------------------------------------------------------*/
#include "mc_curr_ctrl.h"
#include "parameters_conversion.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup curr_ctrl Current controller
  *
  * @brief current controller component 
  *
  * The aim of this component is to perform a current control according to @f$Iqd@f$ reference
  * coming from the [speed and torque](#speed_torq_ctrl) component and the measured current 
  * @f$I\alpha\beta@f$. The current regulation uses a PI current regulator as shown in the picture below. 
  * It can be noticed that, thanks to the VBusCompensation block, the PI regulator takes into account 
  * the DC bus voltage level to mitigate the effect of the voltage ripple. 
  * 
  * When the current controller is deactivated (during observing phase), the estimated Bemf is
  * used to compute and set the PI integral terms. This allows to drastically reduce currents peaks 
  * during flying start.  
  *
  * ![](curr_ctrl.png)
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
static fixp30_t AngleCompensation_run(CurrCtrl_Handle_t* pHandle, fixp30_t angle);
static fixp30_t FOC_BusVoltageCompensation(CurrCtrl_Handle_t* pHandle, const fixp30_t udc_pu);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */

/**
  * @brief  Initializes current controller component.
  *
  * It Should be called during Motor control middleware initialization.
  *
  * @param  pHandle current controller handler address
  * @param  flashParams flash parameters
  */
void MC_currentControllerInit(CurrCtrl_Handle_t* pHandle, FLASH_Params_t const *flashParams)
{

  /* Bus voltage compensation */
  FIXPSCALED_floatToFIXPscaled(1000.0f / TF_REGULATION_RATE, &pHandle->busVoltageFilter); // 1000 (radians/s) filter for bus voltage compensation

  float duty_limit;
  duty_limit = flashParams->board.maxModulationIndex;

  /* Current PID Controllers */

  /* PID Controllers, maximum duty */
  float duty_scale = 2.0f; /* PWM duty ranges from -1 to +1, so the scale is 2.0 */

  /* Calculate current controller default PID parameters, based on Rs and Ls */
  float kp_idq = 0.8f * (0.25f * flashParams->motor.ls * TF_REGULATION_RATE * duty_scale);	/* unit V/A somewhat lower to allow for unoptimized delay by oversampling*/
  float wi_idq = (flashParams->motor.rs / flashParams->motor.ls);					/* unit rad/s */

  /* store Kp in float to be used during RsDC estimation */
  pHandle->pid_IdIqX_obj.Kp = kp_idq;
  /* Cross coupled current controller initialization */
  PIDREGDQX_CURRENT_init(&pHandle->pid_IdIqX_obj,
                         flashParams->scale.current,
                         flashParams->scale.voltage,
                         TF_REGULATION_RATE,
                         flashParams->scale.frequency,
                         duty_limit);

  PIDREGDQX_CURRENT_setKp_si(&pHandle->pid_IdIqX_obj, kp_idq);
  PIDREGDQX_CURRENT_setWi_si(&pHandle->pid_IdIqX_obj, wi_idq);
  PIDREGDQX_CURRENT_setOutputLimitsD(&pHandle->pid_IdIqX_obj, FIXP30(duty_limit*0.95f), FIXP30(-duty_limit*0.95f));
  PIDREGDQX_CURRENT_setOutputLimitsQ(&pHandle->pid_IdIqX_obj, FIXP30(duty_limit), FIXP30(-duty_limit));

  pHandle->maxModulation = FIXP30(duty_limit);
  pHandle->angle_compensation_factor = FIXP24(ANGLE_COMPENSATION_FACTOR);
  pHandle->freq_to_pu_sf = FIXP30(flashParams->scale.frequency / PWM_FREQUENCY);


}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Performs current control.
  *
  * It Should be called during FOC current control.
  *
  * @param  pHandle current controller handler address
  * @param  In current controller input data structure 
  * @param  Out output duties in @f$\alpha\beta@f$ coordinates   
  */
void MC_currentController(CurrCtrl_Handle_t* pHandle, CurrCtrl_Input_t* In, Duty_Dab_t* Out)
{
    /* Angle calculations *****************************************************************************/

  /* Park Transformations ***************************************************************************/

  /* Get cosine/sine of measurement angle */
  FIXP_CosSin_t cossin_park;
  FIXP30_CosSinPU(In->angle, &cossin_park);
  pHandle->angle_park_pu = In->angle; // for tracing purpose

  /* Park transforms */
  pHandle->Idq_in_pu = MCM_Park_Current(In->Iab_in_pu, &cossin_park);

  /* Field oriented control *************************************************************************/

  /* Bus voltage compensation */
  fixp24_t busVoltageComp = FOC_BusVoltageCompensation(pHandle,In->Udcbus_in_pu);
  PIDREGDQX_CURRENT_setCompensation(&pHandle->pid_IdIqX_obj, busVoltageComp);

  /* Current PID controllers */
  if (true == pHandle->currentControlEnabled)
  {
    if (true == In->pwmControlEnable)
    {
      PIDREGDQX_CURRENT_run(
        &pHandle->pid_IdIqX_obj,
        (In->Idq_ref_currentcontrol_pu.D - pHandle->Idq_in_pu.D),
        (In->Idq_ref_currentcontrol_pu.Q - pHandle->Idq_in_pu.Q),
         In->SpeedLP);
      pHandle->Ddq_out_pu.D = PIDREGDQX_CURRENT_getOutD(&pHandle->pid_IdIqX_obj);
      pHandle->Ddq_out_pu.Q = PIDREGDQX_CURRENT_getOutQ(&pHandle->pid_IdIqX_obj);
    }
    else
    {
      /* When the current controller is not in control, the integrators are set to the measured values instead */

      /* While the current controllers are not in control, we set the integrators to the measured values. This makes it possible to start
      * PWM current control at any moment without current spikes */
      /* Perform the Park transformation */

      Voltages_Udq_t	Udq_emf_pu;

      Udq_emf_pu = MCM_Park_Voltage(In->Uab_emf_pu, &cossin_park);

      fixp30_t Id_Ui = FIXP_mpy(Udq_emf_pu.D, (busVoltageComp<<1));
      fixp30_t Iq_Ui = FIXP_mpy(Udq_emf_pu.Q, (busVoltageComp<<1));

      /* Set current controller integrators to (theoretically) correct values */
      PIDREGDQX_CURRENT_setUiD_pu(&pHandle->pid_IdIqX_obj, Id_Ui);
      PIDREGDQX_CURRENT_setUiQ_pu(&pHandle->pid_IdIqX_obj, Iq_Ui);

      /* Store the calculated values */
      pHandle->Ddq_out_pu.D = Id_Ui;
      pHandle->Ddq_out_pu.Q = Iq_Ui;
    }
  }
  else
  {
    /* Current control disabled */

    /* Use the user's duty reference instead */
    pHandle->Ddq_out_pu = pHandle->Ddq_ref_pu;

    /* Clear current PID integrators, since the PIDs are not in control */
    PIDREGDQX_CURRENT_setUiD_pu(&pHandle->pid_IdIqX_obj, FIXP30(0.0f));
    PIDREGDQX_CURRENT_setUiQ_pu(&pHandle->pid_IdIqX_obj, FIXP30(0.0f));
  }

  /* Duty limit */
  {
    /* Limit the duty by the maximum allowed modulation */
    fixp30_t maxModulation = pHandle->maxModulation;
    pHandle->Ddq_out_pu.D = FIXP_sat(pHandle->Ddq_out_pu.D, maxModulation, -maxModulation);
    pHandle->Ddq_out_pu.Q = FIXP_sat(pHandle->Ddq_out_pu.Q, maxModulation, -maxModulation);
  }

  /* make lowpass version for checking delay compensation */
  pHandle->Ddq_out_LP_pu.D += ((pHandle->Ddq_out_pu.D - pHandle->Ddq_out_LP_pu.D) >> 12);
  pHandle->Ddq_out_LP_pu.Q += ((pHandle->Ddq_out_pu.Q - pHandle->Ddq_out_LP_pu.Q) >> 12);

  /* Inverse transformations and output generation **************************************************/

  /* Get cosine/sine of output angle */
  FIXP_CosSin_t cossin_pwm;
  fixp30_t anglePWM;
  anglePWM = AngleCompensation_run(pHandle, In->angle);
  FIXP30_CosSinPU(anglePWM, &cossin_pwm);
<#if MC.MOTOR_PROFILER == true>

  if (false == pHandle->forceZeroPwm)
</#if>  
  {
    /* Inverse Park */
    pHandle->Dab_out_pu = MCM_Inv_Park_Duty(pHandle->Ddq_out_pu, &cossin_pwm);

  } 
<#if MC.MOTOR_PROFILER == true>  
  else
  {
    /* Reset Alpha/Beta duties in case PWM are forced to 0: used in profiling 
       mode. It can be removed in case Profiler is not used */    
    pHandle->Dab_out_pu.A = FIXP30(0.0f);
    pHandle->Dab_out_pu.B = FIXP30(0.0f);

    /* Clear current PID integrators, since the PIDs are not in control */
    PIDREGDQX_CURRENT_setUiD_pu(&pHandle->pid_IdIqX_obj, FIXP30(0.0f));
    PIDREGDQX_CURRENT_setUiQ_pu(&pHandle->pid_IdIqX_obj, FIXP30(0.0f));
  }
  
</#if>  
  *Out = pHandle->Dab_out_pu;
  
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
static inline fixp30_t FOC_BusVoltageCompensation(CurrCtrl_Handle_t* pHandle, const fixp30_t udc_pu)
{
  fixp24_t busVoltageComp = pHandle->busVoltageComp;

  fixp24_t temp = FIXP(1.0f) - FIXP30_mpy(udc_pu, busVoltageComp);
  fixp24_t factor = FIXP_mpyFIXPscaled(busVoltageComp, &pHandle->busVoltageFilter);
  busVoltageComp += FIXP_mpy(temp,factor);
  busVoltageComp = FIXP_sat(busVoltageComp, pHandle->busVoltageCompMax, pHandle->busVoltageCompMin);
  pHandle->busVoltageComp = busVoltageComp;

  if (false == pHandle->busVoltageCompEnable)
  {
    /* Use factor 1.0 when disabled */
    busVoltageComp = FIXP24(1.0f);
  }

  return busVoltageComp;
} 

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Computes the angle compensation factor.
  *         
  * This factor compensates the angle delay between currents measurements 
  *         and duties settings
  * @param  pHandle current controller handler address
  * @param  speed motor electrical speed
  */
void AngleCompensation_runBackground(CurrCtrl_Handle_t* pHandle, fixp30_t speed)
{
  fixp30_t freq_factor;
  
  /* This part of the angle compensation is processed in the MediumFrequencyTask to reduce the
   * calculation time in the HighFrequencyTask. */
  
  /* correct delay of control-loop ang_pu = delaytime x rev/s */
  freq_factor = FIXP24_mpy(pHandle->angle_compensation_factor, pHandle->freq_to_pu_sf);
  
  pHandle->angle_compensation_pu = FIXP30_mpy(speed, freq_factor);
} 

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
static inline fixp30_t AngleCompensation_run(CurrCtrl_Handle_t* pHandle, fixp30_t angle)
{
  /* This part of the angle compensation is processed in the MediumFrequencyTask to reduce the
   * calculation time in the HighFrequencyTask. */

  fixp30_t angle_pwm_pu;
  angle_pwm_pu = angle + pHandle->angle_compensation_pu;
  angle_pwm_pu &= 0x3FFFFFFFL; /* Wrap angle */
  return angle_pwm_pu;
} 


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/