/**
  ******************************************************************************
  * @file    mc_potentiometer.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          potentiometer component of the Motor Control SDK.
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
  * @ingroup mc_potentiometer
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MC_POTENTIOMETER_H_
#define _MC_POTENTIOMETER_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pwm_curr_fdbk.h"
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup mc_potentiometer MC Potentiometer
  * @{
  */

/**
  * @brief  Potentiometer application handler definition
  */
typedef struct
{
  PWMC_Handle_t *pPWM;             /*!< @brief todo */
  float speedMaxRPM;               /*!< @brief maximum speed expressed in RPM THROTTLE_SPEED_MAX_RPM */
  float offset;                    /*!< @brief todo */
  float gain;                      /*!< @brief todo */
  int8_t direction;                /*!< @brief todo */
  fixp30_t adcVal;                 /*!< @brief todo */
  fixp30_t position;               /*!< @brief todo */
  fixp30_t speedref_scalefactor;   /*!< @brief Speed at full throttle, in per unit frequency */
} Potentiometer_Handle_t;

void PotentiometerGetPosition(Potentiometer_Handle_t *pHandle );
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _MC_POTENTIOMETER_H_ */
