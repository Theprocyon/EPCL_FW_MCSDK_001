<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/*******************************************************************************
  * @file    mc_configuration_registers.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides project configuration information registers.
  *
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
  */
  
#include "mc_type.h"
#include "mc_configuration_registers.h"
<#if MC.MCP_EN>
#include "register_interface.h"
</#if><#-- MC.MCP_EN -->
#include "parameters_conversion.h"

#define FIRMWARE_NAME_STR "ST MC SDK\tVer.6.3.2"


const char_t CTL_BOARD[] = "${MC.CTLBOARD_NAME}";
static const char_t M1_PWR_BOARD[] = "${MC.M1_POWERBOARD_NAME}";
const char_t FIRMWARE_NAME [] = FIRMWARE_NAME_STR;

const GlobalConfig_reg_t globalConfig_reg = 
{
  .SDKVersion     = SDK_VERSION,
  .MotorNumber    = <#if MC.DRIVE_NUMBER == "1"> 1 <#else> 2 </#if>,
  .MCP_Flag       = FLAG_MCP_OVER_STLINK + FLAG_MCP_OVER_UARTA + FLAG_MCP_OVER_UARTB,
  .MCPA_UARTA_LOG = ${MC.MCPA_OVER_UART_A_STREAM},
  .MCPA_UARTB_LOG = ${MC.MCPA_OVER_UART_B_STREAM},
  .MCPA_STLNK_LOG = ${MC.MCPA_OVER_STLINK_STREAM},
};

static const ApplicationConfig_reg_t M1_ApplicationConfig_reg =
{
  .maxMechanicalSpeed = ${MC.M1_MAX_APPLICATION_SPEED},
  .maxReadableCurrent = M1_MAX_READABLE_CURRENT,
  .nominalCurrent     = ${MC.M1_NOMINAL_CURRENT},
  .nominalVoltage     = ${MC.M1_NOMINAL_BUS_VOLTAGE_V},
  .driveType          = DRIVE_TYPE_M1,
};

//cstat !MISRAC2012-Rule-9.2
static const MotorConfig_reg_t M1_MotorConfig_reg =
{
  .polePairs  = ${MC.M1_POLE_PAIR_NUM},
  .ratedFlux  = ${MC.M1_MOTOR_VOLTAGE_CONSTANT},
  .rs         = ${MC.M1_RS},
  .ls         = ${MC.M1_LS}*${MC.M1_LDLQ_RATIO},
  .ld         = ${MC.M1_LS},   <#-- This is not a Typo, WB stores ld value into LS field-->
  .maxCurrent = ${MC.M1_NOMINAL_CURRENT},
  .name = "${MC.M1_MOTOR_NAME}"
};

<#if FOC || ACIM>
static const FOCFwConfig_reg_t M1_FOCConfig_reg =
{
  .primarySensor      = (uint8_t)PRIM_SENSOR_M1,
  .auxiliarySensor    = (uint8_t)AUX_SENSOR_M1,
  .topology           = (uint8_t)TOPOLOGY_M1,
  .FOCRate            = (uint8_t)FOC_RATE_M1,
  .PWMFrequency       = (uint32_t)PWM_FREQ_M1,
  .MediumFrequency    = (uint16_t)MEDIUM_FREQUENCY_TASK_RATE,
  .configurationFlag1 = (uint16_t)configurationFlag1_M1, //cstat !MISRAC2012-Rule-10.1_R6
  .configurationFlag2 = (uint16_t)configurationFlag2_M1, //cstat !MISRAC2012-Rule-10.1_R6
};
</#if><#-- FOC || ACIM -->
<#if SIX_STEP>
static const SixStepFwConfig_reg_t M1_SixStepConfig_reg =
{
  .primarySensor      = (uint8_t)PRIM_SENSOR_M1,
  .topology           = (uint8_t)TOPOLOGY_M1,
  .PWMFrequency       = (uint32_t)PWM_FREQ_M1,
  .MediumFrequency    = (uint16_t)MEDIUM_FREQUENCY_TASK_RATE,
  .configurationFlag1 = (uint16_t)configurationFlag1_M1, //cstat !MISRAC2012-Rule-10.1_R6
  .driveMode          = (uint8_t)DEFAULT_DRIVE_MODE
};
</#if><#-- SIX_STEP -->

<#if MC.DRIVE_NUMBER != "1">
const char_t M2_PWR_BOARD[] = "${MC.M2_POWERBOARD_NAME}";

//cstat !MISRAC2012-Rule-9.2
static const MotorConfig_reg_t M2_MotorConfig_reg =
{
  .polePairs  = ${MC.M2_POLE_PAIR_NUM},
  .ratedFlux  = ${MC.M2_MOTOR_VOLTAGE_CONSTANT},
  .rs         = ${MC.M2_RS},
  .ls         = ${MC.M2_LS}*${MC.M2_LDLQ_RATIO},
  .ld         = ${MC.M2_LS},   <#-- This is not a Typo, if LDLQ_RATIO != 1, WB stores ld value into LS field-->
  .maxCurrent = ${MC.M2_NOMINAL_CURRENT},
  .name       = "${MC.M2_MOTOR_NAME}"
};

static const ApplicationConfig_reg_t M2_ApplicationConfig_reg =
{
  .maxMechanicalSpeed = ${MC.M2_MAX_APPLICATION_SPEED},
  .maxReadableCurrent = M2_MAX_READABLE_CURRENT,
  .nominalCurrent     = ${MC.M2_NOMINAL_CURRENT},
  .nominalVoltage     = ${MC.M2_NOMINAL_BUS_VOLTAGE_V},
  .driveType          = DRIVE_TYPE_M2,
};

static const FOCFwConfig_reg_t M2_FOCConfig_reg =
{
  .primarySensor       = (uint8_t)PRIM_SENSOR_M2,
  .auxiliarySensor    = (uint8_t)AUX_SENSOR_M2,
  .topology           = (uint8_t)TOPOLOGY_M2,
  .FOCRate            = (uint8_t)FOC_RATE_M2,
  .PWMFrequency       = (uint32_t)PWM_FREQ_M2,
  .MediumFrequency    = (uint16_t)MEDIUM_FREQUENCY_TASK_RATE2,
  .configurationFlag1 = (uint16_t)configurationFlag1_M2,
  .configurationFlag2 = (uint16_t)configurationFlag2_M2,
};

const FOCFwConfig_reg_t* FOCConfig_reg[NBR_OF_MOTORS] = {&M1_FOCConfig_reg, &M2_FOCConfig_reg};
const MotorConfig_reg_t* MotorConfig_reg[NBR_OF_MOTORS] = {&M1_MotorConfig_reg, &M2_MotorConfig_reg};
const ApplicationConfig_reg_t* ApplicationConfig_reg[NBR_OF_MOTORS] = {&M1_ApplicationConfig_reg, 
                                                                       &M2_ApplicationConfig_reg};
const char_t * PWR_BOARD_NAME[NBR_OF_MOTORS] = {M1_PWR_BOARD ,M2_PWR_BOARD};
<#else><#-- MC.DRIVE_NUMBER == 1 -->
const char_t * PWR_BOARD_NAME[NBR_OF_MOTORS] = {M1_PWR_BOARD};
  <#if FOC ||  ACIM>
const FOCFwConfig_reg_t* FOCConfig_reg[NBR_OF_MOTORS] = {&M1_FOCConfig_reg};
  </#if><#-- FOC ||  ACIM -->
  <#if SIX_STEP>
const SixStepFwConfig_reg_t* SixStepConfig_reg[NBR_OF_MOTORS] = {&M1_SixStepConfig_reg};
  </#if><#-- SIX_STEP -->
const MotorConfig_reg_t* MotorConfig_reg[NBR_OF_MOTORS] = {&M1_MotorConfig_reg};
const ApplicationConfig_reg_t* ApplicationConfig_reg[NBR_OF_MOTORS] = {&M1_ApplicationConfig_reg};
</#if><#-- MC.DRIVE_NUMBER > 1 -->


/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
