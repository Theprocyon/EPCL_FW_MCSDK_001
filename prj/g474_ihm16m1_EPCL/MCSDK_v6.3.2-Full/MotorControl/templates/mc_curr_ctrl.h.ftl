<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    mc_curr_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          current controller component of the Motor Control SDK.
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
  
#ifndef _MC_CURR_CTRL_H_
#define _MC_CURR_CTRL_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "fixpmath.h"
#include "mc_type.h"
#include "pidregdqx_current.h"
#include "mc_math.h"
#include "flash_parameters.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup curr_ctrl
  * @{
  */
  
/**
  * @brief  current controller input data structure definition
  */  
typedef struct
{
  Currents_Iab_t   Iab_in_pu;                      /*!< @brief input current in ab coordinates expressed in per-unit */
  Voltages_Uab_t   Uab_emf_pu;                     /*!< @brief input Bemf in ab coordinates expressed in per-unit */
  Currents_Idq_t   Idq_ref_currentcontrol_pu;      /*!< @brief current reference in dq coordinates expressed in per-unit */
  fixp30_t angle;                                  /*!< @brief angle used for Park transforamtion */
  fixp30_t SpeedLP;                                /*!< @brief motor electrical frequency */
  fixp30_t Udcbus_in_pu;                           /*!< @brief DC bus level expressed in per-unit */
  bool pwmControlEnable;                           /*!< @brief PWM control flag status */
} CurrCtrl_Input_t;

/**
  * @brief  Current controller data structure definition
  */
typedef struct
{
  PIDREGDQX_CURRENT_s pid_IdIqX_obj;  /*!< @brief PID current regulator data structure */
  Currents_Iab_t Iab_inLP_pu;         /*!< @brief low-pass filtered phase current in ab coordinates expressed in per-unit. */
  Voltages_Uab_t Uab_inLP_pu;         /*!< @brief low-pass filtered phase volatges in ab coordinates expressed in per-unit. */
  Currents_Idq_t Idq_in_pu;           /*!< @brief phase currens in dq coordinates expressed in per-unit */
  Voltages_Udq_t Udq_in_pu;           /*!< @brief phase voltages in dq coordinates expressed in per-unit */
  Duty_Ddq_t Ddq_out_pu;              /*!< @brief PI output duties expressed in per-unit */
  Duty_Ddq_t Ddq_ref_pu;              /*!< @brief duties reference from user expressed in per-unit */
  Duty_Ddq_t Ddq_out_LP_pu;           /*!< @brief low-pass filtered duties for monitoring  */
  Duty_Dab_t Dab_out_pu;              /*!< @brief duties in ab coordinates expressed in per-unit. */
  fixp30_t angle_park_pu;             /*!< @brief angle used for Park transformation */
  fixp30_t angle_pwm_pu;              /*!< @brief compensated angle used in reversa Park transformation */
  fixp30_t angle_compensation_pu;     /*!< @brief angle compensation factor to compensate the angle delay between currents measurements 
                                                  and duties settings in per-unit  */
  fixp24_t angle_compensation_factor; /*!< @brief angle compensation factor to compensate the angle delay between currents measurements 
                                                  and duties settings */
  fixp30_t freq_to_pu_sf;             /*!< @brief  conversion factor */
  fixp30_t maxModulation;             /*!< @brief  maximum modulation index*/
  fixp24_t busVoltageComp;            /*!< @brief  DC bus volatge compensation factor (=1/DC Bus) */
  FIXP_scaled_t busVoltageFilter;     /*!< @brief  low-pass filter of the DC Bus value*/
  fixp24_t busVoltageCompMax;         /*!< @brief  upper limit for bus voltage compensation */
  fixp24_t busVoltageCompMin;         /*!< @brief  lower limit for bus voltage compensation */
  bool busVoltageCompEnable;          /*!< @brief  DC bus voltae compensate enabling flag */
  bool forceZeroPwm;                  /*!< @brief Forces PWM output to zero (50% duty all phases) when enabled */
  bool currentControlEnabled;         /*!< @brief Enable current PID controllers. Uses Ddq_ref_pu when disabled. */
} CurrCtrl_Handle_t;

/* Initializes current controller component. */
void MC_currentControllerInit(CurrCtrl_Handle_t* pHandle, FLASH_Params_t const *flashParams);

/* Performs current control. */
void MC_currentController(CurrCtrl_Handle_t* pHandle, CurrCtrl_Input_t* In, Duty_Dab_t* Out);

/* Computes the angle compensation factor. */
void AngleCompensation_runBackground(CurrCtrl_Handle_t* pHandle, fixp30_t speed);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/