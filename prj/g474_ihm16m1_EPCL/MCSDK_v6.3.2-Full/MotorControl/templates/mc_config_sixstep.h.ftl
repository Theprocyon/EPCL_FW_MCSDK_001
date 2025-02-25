<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    mc_config.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler 
  *          structures declarations.
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
  */ 
  
#ifndef MC_CONFIG_H
#define MC_CONFIG_H

#include "speed_ctrl.h"
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true> 
#include "open_loop_sixstep.h"
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true-->
#include "revup_ctrl_sixstep.h"
#include "mc_config_common.h"
#include "pwmc_sixstep.h"
<#if MC.DRIVE_MODE == "CM">
#include "current_ref_ctrl.h"
</#if><#-- MC.DRIVE_MODE == "CM" -->
<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
  <#if CondFamily_STM32F0>
#include "f0xx_bemf_ADC_fdbk.h"
  </#if>
  <#if CondFamily_STM32F3>
#include "f3xx_bemf_ADC_fdbk.h"
  </#if>
  <#if CondFamily_STM32F4>
#include "f4xx_bemf_ADC_fdbk.h"
  </#if>
  <#if CondFamily_STM32G0>
#include "g0xx_bemf_ADC_fdbk.h"
  </#if>
  <#if CondFamily_STM32C0>
#include "c0xx_bemf_ADC_fdbk.h"
  </#if>
  <#if CondFamily_STM32G4>  
#include "g4xx_bemf_ADC_fdbk.h"
  </#if><#-- CondFamily_STM32G4 -->
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
<#if (M1_HALL_SENSOR == true)>
#include "hall_speed_pos_fdbk.h"
</#if><#-- (M1_HALL_SENSOR == true) -->

extern PWMC_Handle_t PWM_Handle_M1;
<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
extern Bemf_ADC_Handle_t Bemf_ADC_M1;
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
<#if MC.DRIVE_MODE == "CM">
extern CurrentRef_Handle_t CurrentRef_M1;
</#if><#-- MC.DRIVE_MODE == "CM" -->
extern SixStepVars_t SixStepVars[NBR_OF_MOTORS];
<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
extern RevUpCtrl_Handle_t RevUpControlM1;
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") 
         || MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
extern SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true> 
extern OpenLoopSixstepCtrl_Handle_t *pOLS[NBR_OF_MOTORS];
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true-->
extern MCI_Handle_t Mci[NBR_OF_MOTORS];
<#if MC.M1_POSITION_CTRL_ENABLING == true>
extern PID_Handle_t PID_PosParamsM1;
extern PosCtrl_Handle_t PosCtrlM1;
</#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->
extern PID_Handle_t PIDSpeedHandle_M1;
<#if M1_HALL_SENSOR == true>
extern HALL_Handle_t HALL_M1;
</#if><#-- M1_HALL_SENSOR == true -->

/* USER CODE BEGIN Additional extern */

/* USER CODE END Additional extern */  

#endif /* MC_CONFIG_H */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
