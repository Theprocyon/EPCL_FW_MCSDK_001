<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/foc_assign.ftl">

/**
  ******************************************************************************
  * @file    mc_tasks.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implementes tasks definition.
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
  * @ingroup MCTasks
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MCTASKS_H
#define MCTASKS_H

/* Includes ------------------------------------------------------------------*/
#include "mc_parameters.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MCTasks Motor Control Tasks
  * 
  * @brief Motor Control subsystem configuration and operation routines.  
  *
  * @{
  */

#define STOPPERMANENCY_MS              ((uint16_t)400)
#define STOPPERMANENCY_MS2             ((uint16_t)400)
#define STOPPERMANENCY_TICKS           (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)  / ((uint16_t)1000))
#define STOPPERMANENCY_TICKS2          (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2) / ((uint16_t)1000))

/* Initializes the Motor subsystem core according to user defined parameters */
void MCboot(MCI_Handle_t *pMCIList[NBR_OF_MOTORS]);

/* Runs all the Tasks of the Motor Control cockpit */
void MC_RunMotorControlTasks(void);

/* Executes the Medium Frequency Task functions for each drive instance */
void MC_Scheduler(void);

/* Executes safety checks (e.g. bus voltage and temperature) for all drive instances */
void TSK_SafetyTask(void);

/* */
void ${MC.M1_DRIVE_TYPE}_Init(void);

/* */
uint8_t ${MC.M1_DRIVE_TYPE}_HighFrequencyTask(uint8_t bMotorNbr);

/* */
void ${MC.M1_DRIVE_TYPE}_Clear(uint8_t bMotor);

/* Executes the Motor Control duties that require a high frequency rate and a precise timing */
<#if HIGH_FREQ_TRIGGER == "TIM_UP_IT">
uint8_t TSK_HighFrequencyTask(void);
<#else>
uint8_t TSK_HighFrequencyTask(uint8_t bMotorNbr);
</#if><#-- HIGH_FREQ_TRIGGER == "TIM_UP_IT" -->

/* */
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);

/* */
bool TSK_ChargeBootCapDelayHasElapsedM1(void);

/* */
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);

/* */
bool TSK_StopPermanencyTimeHasElapsedM1(void);

/* */
void TSK_SetStopPermanencyTimeM2(uint16_t SysTickCount);

/* */
void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount);

/* */
void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount);

/* */
bool TSK_StopPermanencyTimeHasElapsedM2(void);

/* */
bool TSK_ChargeBootCapDelayHasElapsedM2(void);

<#if MC.START_STOP_BTN>
void UI_HandleStartStopButton_cb(void);
</#if><#-- MC.START_STOP_BTN -->

<#if MC.M1_DRIVE_TYPE == "FOC" || MC.M2_DRIVE_TYPE == "FOC">
/* Reserves FOC execution on ADC ISR half a PWM period in advance */
void TSK_DualDriveFIFOUpdate(uint8_t Motor);
</#if><#-- MC.M1_DRIVE_TYPE == "FOC" || MC.M2_DRIVE_TYPE == "FOC" -->

/* Puts the Motor Control subsystem in in safety conditions on a Hard Fault */
void TSK_HardwareFaultTask(void);

 /* Locks GPIO pins used for Motor Control to prevent accidental reconfiguration */
void mc_lock_pins(void);

<#if MC.M1_DRIVE_TYPE == "SIX_STEP" || MC.M2_DRIVE_TYPE == "SIX_STEP">
uint16_t SixStep_StepCommution(void);
  <#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
/* Puts parameters for PWM on sensing configuration. */
void OLS_SetSamplingPoint(Bemf_ADC_Handle_t *pHandle, PWMC_Handle_t *pHandlePWMC, BusVoltageSensor_Handle_t *BusVHandle);
/* Returns the status of the PWM on sensing PWM flag. */
bool OLS_GetOnSensingStatus(Bemf_ADC_Handle_t *pHandle);
    </#if> <#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
  </#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
</#if><#-- MC.M1_DRIVE_TYPE == "SIX_STEP" || MC.M2_DRIVE_TYPE == "SIX_STEP" -->

<#if  MC.M1_SPEED_SENSOR == "HSO" || MC.M1_SPEED_SENSOR == "ZEST">

/* Executes startup procedudure using combine or single PolPules, RsDC measure */
void TSK_PulsesRsDC(StartMode_t Mode);

</#if>

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MCTASKS_H */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
