<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
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
  
#ifndef __MC_CONFIG_H
#define __MC_CONFIG_H

#include "pwm_curr_fdbk.h"
#include "r3_g4xx_pwm_curr_fdbk.h"
#include "oversampling.h"
#include "mc_interface.h"
#include "speed_torq_ctrl_hso.h"    
#include "mc_curr_ctrl.h"
#include "speed_pos_fdbk_hso.h"
#include "mc_polpulse.h"
#include "profiler.h"
#include "rsdc_est.h"
#include "bus_voltage.h"
#include "ntc_temperature_sensor.h"
<#if MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true>
#include "mc_potentiometer.h"
</#if>
#include "regular_conversion_manager.h"
<#if MC.M1_PWM_DRIVER_PN == "STSPIN32G4" >
#include "stspin32g4.h"
</#if><#-- MC.M1_PWM_DRIVER_PN == "STSPIN32G4" -->
<#if MC.MOTOR_SIM == true>
#include "mc_motorsim.h"
</#if>
<#-- ICL feature usage -->
<#if  MC.M1_ICL_ENABLED == true >
  #include "inrush_current_limiter.h"
</#if>

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

extern PWMC_R3_Handle_t PWM_Handle_M1;
extern Oversampling_t oversampling;
extern IMPEDCORR_Obj ImpedCorr_M1;
extern HSO_Obj       HSO_M1;
extern SPD_Handle_t SPD_M1;
extern STC_Handle_t STC_M1;
extern IMPEDCORR_Params ImpedCorr_params_M1;
extern HSO_Params Hso_params_M1;
<#if MC.M1_BUS_VOLTAGE_READING>    
  <#if (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_U) && (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_V) && (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_W)> 
extern RegConv_t VbusRegConv_M1;
  </#if>
</#if>
<#if (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true)>
  <#if (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_U) && (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_V) && (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_W)>
extern RegConv_t TempRegConv_M1;
  </#if>
</#if>
extern NTC_Handle_t TempSensor_M1;
<#if MC.M1_SPEED_SENSOR == "ZEST"> 
extern ZEST_Obj      ZeST_M1;
extern RSEST_Obj     RsEst_M1;
extern ZEST_Params Zest_params_M1;
extern RSEST_Params Rsest_params_M1;
</#if>
extern CurrCtrl_Handle_t CurrCtrl_M1;
extern RsDCEstimation_Handle_t RsDCEst_M1;
extern POLPULSE_Params PolPulse_params_M1;
extern MC_PolPulse_Handle_t MC_PolPulse_M1;
extern PROFILER_Obj	profiler_M1;
extern PROFILER_Params profilerParams_M1;
extern ProfilerMotor_Handle_t profilerMotor_M1;
extern BusVoltageSensor_Handle_t VBus_M1;
<#if MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true>
extern Potentiometer_Handle_t potentiometer_M1;
</#if>
<#if (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_U) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_V) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_W)>
extern RegConv_t PotRegConv_M1;
</#if><#-- (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_U) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_V) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_W) -->
<#if MC.MOTOR_SIM == true>
extern MotorSim_Handle_t MotorSim;
extern MOTORSIM_Obj MotorSim_Obj;
extern MOTORSIM_Params MotorSim_params;
</#if>
#define NBR_OF_MOTORS 1
extern MCI_Handle_t Mci[NBR_OF_MOTORS];
<#if  MC.M1_ICL_ENABLED == true>
extern ICL_Handle_t ICL_M1;
</#if>
<#if MC.M1_ICL_ENABLED == true>
extern DOUT_handle_t ICLDOUTParamsM1;
</#if>
<#if MC.M1_PWM_DRIVER_PN == "STSPIN32G4">
extern STSPIN32G4_HandleTypeDef HdlSTSPING4;
</#if><#-- MC.M1_PWM_DRIVER_PN == "STSPIN32G4" -->
/* USER CODE BEGIN Additional extern */

/* USER CODE END Additional extern */  
#define NBR_OF_MOTORS 1
#endif /* __MC_CONFIG_H */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
