/**
  ******************************************************************************
  * @file    bus_voltage_sensor.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          BusVoltageSensor component of the Motor Control SDK.
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
  * @ingroup BusVoltageSensor
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BUSVOLTAGESENSOR_H
#define BUSVOLTAGESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor Bus voltage sensor
  * @{
  */

/**
  * @brief  Bus voltage sensor handle definition
  */
typedef struct
{
  fixp30_t Udcbus_convFactor;  
  fixp30_t Udcbus_undervoltage_limit_pu;
  fixp30_t Udcbus_overvoltage_limit_pu;
  fixp30_t Udcbus_in_pu;

} BusVoltageSensor_Handle_t;

/* Exported functions ------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* BUSVOLTAGESENSOR_H */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/