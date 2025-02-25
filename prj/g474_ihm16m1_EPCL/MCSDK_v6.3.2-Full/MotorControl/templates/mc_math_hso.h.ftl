<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    mc_math.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control.
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
  * @ingroup MC_Math
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_MATH_H
#define MC_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MC_Math
  * @{
  */

typedef enum _FOC_ModulationMode_e_
{
  FOC_MODULATIONMODE_Sine,
  FOC_MODULATIONMODE_Centered,
  FOC_MODULATIONMODE_ShiftedCenter,
  FOC_MODULATIONMODE_MinLow,
  FOC_MODULATIONMODE_MaxHigh,
  FOC_MODULATIONMODE_UpDown,
  numFOC_MODULATIONMODE
} FOC_ModulationMode_e;

Currents_Iab_t MCM_Clarke_Current( const Currents_Irst_t Irst );
Voltages_Uab_t MCM_Clarke_Voltage( const Voltages_Urst_t Urst );
Currents_Idq_t MCM_Park_Current( Currents_Iab_t Iab, FIXP_CosSin_t* cosSin );
Voltages_Udq_t MCM_Park_Voltage( Voltages_Uab_t Uab, FIXP_CosSin_t* cosSin );
Duty_Dab_t MCM_Inv_Park_Duty( Duty_Ddq_t Ddq, FIXP_CosSin_t* cosSin );
Duty_Drst_t MCM_Inv_Clarke_Duty(Duty_Dab_t* Dab_pu);
Duty_Drst_t MCM_Modulate(const Duty_Drst_t *pDrst_pu, const FOC_ModulationMode_e mode);


/**
  * @}
  */

/**
  * @}
  */
#endif /* MC_MATH_H*/
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
