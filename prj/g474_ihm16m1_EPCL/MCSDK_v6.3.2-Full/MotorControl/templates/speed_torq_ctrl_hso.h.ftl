/**
  ******************************************************************************
  * @file    speed_torq_ctrl_hso.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          speed and torque HSO component of the Motor Control SDK.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _SPEED_TORQ_CTRL_HSO_H_
#define _SPEED_TORQ_CTRL_HSO_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pidreg_speed.h"
#include "pwm_curr_fdbk.h"
#include "mc_curr_ctrl.h"
#include "fixpmath.h"
#include "flash_parameters.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednTorqCtrl
  * @{
  */

/** @addtogroup speed_torq_ctrl_hso
  * @{
  */

/* Exported defines ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Speed source definition
  */
typedef enum _FOC_SPEED_Source_e_
{
    FOC_SPEED_SOURCE_Default,   /*!< @brief speed reference commanded by user API */
    FOC_SPEED_SOURCE_Throttle,  /*!< @brief speed reference commandedby throttle/potentiometer */
    numFOC_SPEED_SOURCE
} FOC_SPEED_Source_e;

/**
  * @brief  current limit data structure
  */
typedef union _FOC_ActiveCurrentLimits_u_
{
    uint16_t	u16;
    FOC_ActiveCurrentLimits_Bits bit;
} FOC_ActiveCurrentLimits_u;

/**
  * @brief  speed and torque input data structure definition
  */
typedef struct
{
  fixp30_t emfSpeed;       /*!< @brief emf speed from HSO if closed loop mode enabled
                                       else imposed speed from open loop */
  fixp30_t SpeedLP;        /*!< @brief lowpass filtered speed from HSO if closed loop mode enabled
                                       else imposed speed from open loop */
  fixp30_t Udcbus_in_pu;   /*!< @brief DC bus voltage in per-unit */
  bool closedLoopEnabled;  /*!< @brief closed loop activation status flag */
  bool pwmControlEnable;   /*!< @brief PWM activation status flag */
}STC_input_t;

/**
  * @brief  speed and torque component handler structure definition
  */
typedef struct
{
  PIDREG_SPEED_s PIDSpeed;                  /*!< @brief PID speed controller handler */
  CurrCtrl_Handle_t *pCurrCtrl;             /*!< @brief current controller handler */
  Currents_Idq_t Idq_ref_pu;			    /*!< @brief Current reference (before injection) */
  Currents_Idq_t Idq_ref_limited_pu;	    /*!< @brief Current reference limited */
  Currents_Idq_t Idq_ref_currentcontrol_pu; /*!< @brief Current reference (after injection, passed to current control) */
  fixp30_t speed_ref_pu;			        /*!< @brief User's speed reference */
  fixp30_t speed_ref_active_pu;             /*!< @brief Selected speed reference before ramping */
  fixp30_t speed_ref_ramped_pu;	            /*!< @brief Selected speed reference after ramping */
  fixp30_t speed_ramp_pu_per_isr;           /*!< @brief speed increment to be applied at each interrupt */
  fixp30_t max_accel_pu_per_isr;            /*!< @brief maximum accleration allowed per-unit per interrupt */
  fixp30_t torque_ramp_pu_per_isr;          /*!< @brief torque increment to be applied at each interrupt */
  fixp30_t I_max_pu;                        /*!< @brief maximum current in per-unit */
  fixp30_t Iq_ref_spd_pu;				    /*!< @brief Output of the speed controller */
  fixp30_t Iq_ref_active_pu;			    /*!< @brief Active Q-current reference value (communicated or throttle input) */
  fixp30_t accel_volt_high_pu;              /*!< @brief bus protection, voltage upper limit for acceleration limitation */
  fixp30_t accel_volt_low_pu;               /*!< @brief bus protection, voltage lower limit for acceleration limitation */
  fixp24_t accel_volt_oneoverrange;		    /*!< @brief bus protection, 1/range, to avoid division */
  fixp30_t regen_volt_high_pu;              /*!< @brief bus protection, voltage upper limit for regeneration limitation */
  fixp30_t regen_volt_low_pu;               /*!< @brief bus protection, voltage lower limit for regeneration limitation */
  fixp24_t regen_volt_oneoverrange;         /*!< @brief bus protection, 1/range, to avoid division */
  fixp30_t regen_factor;                    /*!< @brief bus protection, regeneration factor */
  fixp30_t accel_factor;                    /*!< @brief bus protection, acceleration factor */
  fixp30_t speedLimit;                      /*!< @brief speed limit below which the DC bus protection is no more applied */
  fixp24_t OneOverMaxIq_pu;                 /*!< @brief One over maximum current */
  fixp24_t K_InjectBoost;                   /*!< @brief K factor to boost injected current according to Iq current */
  
  FOC_ActiveCurrentLimits_u active_limits;  /*!< @brief status of the limits have been reached */
  FOC_SPEED_Source_e	speedref_source;    /*!< @brief speed source selector, user API or throttle/potentiometer */
  bool SpeedControlEnabled;                 /*!< @brief Enable speed PID controller. Use Idq_ref_pu.Q when disabled. */
  bool Enable_InjectBoost;                  /*!< @brief Enable boost of injected current according to Iq current. */
}STC_Handle_t;

/* Initializes speed & torque control. */
void STC_Init(STC_Handle_t* pHandle, FLASH_Params_t const *flashParams);

/* Performs speed and torque control. */
void STC_Run(STC_Handle_t* pHandle, STC_input_t *In);

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

#endif