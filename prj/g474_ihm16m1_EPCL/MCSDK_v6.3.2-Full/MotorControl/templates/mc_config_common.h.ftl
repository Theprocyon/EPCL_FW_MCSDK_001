<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    mc_config_common.h 
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
  
#ifndef MC_CONFIG_COMMON_H
#define MC_CONFIG_COMMON_H

<#-- Specific to FOC algorithm usage -->
#include "pid_regulator.h"
#include "virtual_speed_sensor.h"
#include "ntc_temperature_sensor.h"
#include "mc_interface.h"
#include "mc_configuration_registers.h"
<#if (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true) 
  || (MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true)
  || MC.M1_BUS_VOLTAGE_READING == true || MC.M2_BUS_VOLTAGE_READING == true
  ||  MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE >
#include "regular_conversion_manager.h"
</#if><#-- (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true) 
  || (MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true)
  || MC.M1_BUS_VOLTAGE_READING == true || MC.M21_BUS_VOLTAGE_READING == true
  ||  MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE -->
<#if MC.M1_BUS_VOLTAGE_READING == true || MC.M2_BUS_VOLTAGE_READING == true>
#include "r_divider_bus_voltage_sensor.h"
</#if><#-- MC.M1_BUS_VOLTAGE_READING == true || MC.M2_BUS_VOLTAGE_READING == true -->
<#if MC.M1_BUS_VOLTAGE_READING == false || MC.M2_BUS_VOLTAGE_READING == false>
#include "virtual_bus_voltage_sensor.h"
</#if><#-- MC.M1_BUS_VOLTAGE_READING == false || MC.M2_BUS_VOLTAGE_READING == false -->
<#if MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || (MC.M1_HW_OV_CURRENT_PROT_BYPASS == true
  && MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES") || (MC.DRIVE_NUMBER != "1"
  && ((MC.M2_HW_OV_CURRENT_PROT_BYPASS == true && MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES")
  || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE"))>
#include "digital_output.h"
</#if><#-- Resistive break or OC protection bypass on OV -->
<#if MC.M1_PWM_DRIVER_PN == "STSPIN32G4" >
#include "stspin32g4.h"
</#if><#-- MC.M1_PWM_DRIVER_PN == "STSPIN32G4" -->
<#if MC.MOTOR_PROFILER == true>
#include "mp_one_touch_tuning.h"
#include "mp_self_com_ctrl.h"
  <#if  MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR">
#include "mp_hall_tuning.h"
  </#if><#-- MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->
</#if><#-- MC.MOTOR_PROFILER == true -->
<#if MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true >
#include "speed_potentiometer.h"
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true -->
<#if MC.ESC_ENABLE == true>
#include "esc.h"
</#if><#-- MC.ESC_ENABLE == true -->
<#if M1_ENCODER || M2_ENCODER>
#include "enc_align_ctrl.h"
#include "encoder_speed_pos_fdbk.h"
</#if><#-- M1_ENCODER || M2_ENCODER -->

<#if M1_ENCODER>
extern ENCODER_Handle_t ENCODER_M1;
extern EncAlign_Handle_t EncAlignCtrlM1;
</#if><#-- M1_ENCODER -->
<#if M2_ENCODER>
extern ENCODER_Handle_t ENCODER_M2;
extern EncAlign_Handle_t EncAlignCtrlM2;
</#if><#-- M2_ENCODER -->
<#if (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true)>
extern RegConv_t TempRegConv_M1;
</#if><#-- (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true) -->
extern NTC_Handle_t TempSensor_M1;
<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") 
  || MC.M1_VIEW_ENCODER_FEEDBACK == true || (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC") || (MC.M1_DBG_OPEN_LOOP_ENABLE && MC.M1_DRIVE_TYPE != "SIX_STEP")>
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")
        || MC.M1_VIEW_ENCODER_FEEDBACK == true || (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC") -->
<#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")
   || MC.M2_VIEW_ENCODER_FEEDBACK == true || (MC.M2_DBG_OPEN_LOOP_ENABLE && MC.M2_DRIVE_TYPE != "SIX_STEP")>
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM2;
</#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")
         || MC.M2_VIEW_ENCODER_FEEDBACK == true -->
<#if MC.M1_BUS_VOLTAGE_READING == true>
extern RegConv_t VbusRegConv_M1;
extern RDivider_Handle_t BusVoltageSensor_M1;
<#else><#-- MC.M1_BUS_VOLTAGE_READING == false -->
extern VirtualBusVoltageSensor_Handle_t BusVoltageSensor_M1;
</#if><#-- MC.M1_BUS_VOLTAGE_READING == true -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_BUS_VOLTAGE_READING == true>
extern RegConv_t VbusRegConv_M2;
extern RDivider_Handle_t BusVoltageSensor_M2;
  <#else><#-- MC.M2_BUS_VOLTAGE_READING == false -->
extern VirtualBusVoltageSensor_Handle_t BusVoltageSensor_M2;
  </#if><#-- MC.M2_BUS_VOLTAGE_READING == true -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->
<#if DWT_CYCCNT_SUPPORTED>
  <#if MC.DBG_MCU_LOAD_MEASURE == true>
/* Performs the CPU load measure of FOC main tasks */
extern MC_Perf_Handle_t PerfTraces;
  </#if><#--  MC.DBG_MCU_LOAD_MEASURE == true -->
</#if><#-- DWT_CYCCNT_SUPPORTED -->
<#if M1_ENCODER || M2_ENCODER>
extern EncAlign_Handle_t *pEAC[NBR_OF_MOTORS];
</#if><#-- M1_ENCODER || M2_ENCODER -->
extern PWMC_Handle_t *pwmcHandle[NBR_OF_MOTORS];
extern NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
<#if MC.M1_PWM_DRIVER_PN == "STSPIN32G4">
extern STSPIN32G4_HandleTypeDef HdlSTSPING4;
</#if><#-- MC.M1_PWM_DRIVER_PN == "STSPIN32G4" -->
<#if MC.ESC_ENABLE == true>
extern ESC_Handle_t ESC_M1;
</#if><#-- MC.ESC_ENABLE == true -->
<#if MC.M1_POTENTIOMETER_ENABLE == true>
extern SpeedPotentiometer_Handle_t SpeedPotentiometer_M1;
extern RegConv_t PotRegConv_M1;
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true -->

/* USER CODE BEGIN Additional extern */

/* USER CODE END Additional extern */  
 
  

#endif /* MC_CONFIG_COMMON_H */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
