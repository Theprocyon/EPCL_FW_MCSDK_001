<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
<#include "*/ftl/ip_macro.ftl">
/**
  ******************************************************************************
  * @file    mc_config_common.c 
  * @author  Motor Control SDK Team,ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044,the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config_common.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */ 

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

<#if M1_ENCODER>
/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Encoder.
  */
ENCODER_Handle_t ENCODER_M1 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15 * MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED,
    .DPPConvFactor             = DPP_CONV_FACTOR,
  },

  .PulseNumber                 = M1_ENCODER_PPR * 4,
  .SpeedSamplingFreqHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize             = ENC_AVERAGING_FIFO_DEPTH,
  .TIMx                        = ${_last_word(MC.M1_ENC_TIMER_SELECTION)},
  .ICx_Filter                  = M1_ENC_IC_FILTER_LL,
};

/**
  * @brief  Encoder Alignment Controller parameters Motor 1.
  */
EncAlign_Handle_t EncAlignCtrlM1 =
{
  .hEACFrequencyHz = MEDIUM_FREQUENCY_TASK_RATE,
  .hFinalTorque    = FINAL_I_ALIGNMENT,
  .hElAngle        = ALIGNMENT_ANGLE_S16,
  .hDurationms     = M1_ALIGNMENT_DURATION,
  .bElToMecRatio   = POLE_PAIR_NUM,
};

</#if><#-- M1_ENCODER -->
<#if M2_ENCODER>
/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - Encoder.
  */
ENCODER_Handle_t ENCODER_M2 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM2,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15 * MAX_APPLICATION_SPEED_UNIT2),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
    .bMaximumSpeedErrorsNumber = M2_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED2,
    .DPPConvFactor             = DPP_CONV_FACTOR,
  },

  .PulseNumber                 = M2_ENCODER_PPR*4,
  .SpeedSamplingFreqHz         = MEDIUM_FREQUENCY_TASK_RATE2,
  .SpeedBufferSize             = ENC_AVERAGING_FIFO_DEPTH2,
  .TIMx                        = ${_last_word(MC.M2_ENC_TIMER_SELECTION)},
  .ICx_Filter                  = M2_ENC_IC_FILTER_LL,
};

/**
  * @brief  Encoder Alignment Controller parameters Motor 2.
  */
EncAlign_Handle_t EncAlignCtrlM2 =
{
  .hEACFrequencyHz = MEDIUM_FREQUENCY_TASK_RATE2,
  .hFinalTorque    = FINAL_I_ALIGNMENT2,
  .hElAngle        = ALIGNMENT_ANGLE_S162,
  .hDurationms     = M2_ALIGNMENT_DURATION,
  .bElToMecRatio   = POLE_PAIR_NUM2,
};

</#if><#-- M2_ENCODER -->

<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || M1_ENCODER || (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC") || (MC.M1_DBG_OPEN_LOOP_ENABLE && MC.M1_DRIVE_TYPE != "SIX_STEP")>
/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Base Class.
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1 =
{
  
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED,
    .DPPConvFactor             = DPP_CONV_FACTOR,
  },
  
  .hSpeedSamplingFreqHz        = MEDIUM_FREQUENCY_TASK_RATE,
  .hTransitionSteps            = (int16_t)((TF_REGULATION_RATE * TRANSITION_DURATION) / 1000.0),
};

</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || M1_ENCODER || (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC") -->
<#-------------------------------------------------------------------------------------->
<#-- Condition for SpeedNPosition sensor parameters Motor 2 when MC.DRIVE_NUMBER > 1 -->
<#-------------------------------------------------------------------------------------->
  <#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") || M2_ENCODER || (MC.M2_SPEED_SENSOR == "SENSORLESS_ADC")|| (MC.M2_DBG_OPEN_LOOP_ENABLE && MC.M2_DRIVE_TYPE != "SIX_STEP")>
/**
  * @brief  SpeedNPosition sensor parameters Motor 2.
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM2 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM2,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15 * MAX_APPLICATION_SPEED_UNIT2),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
    .bMaximumSpeedErrorsNumber = M2_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED2,
    .DPPConvFactor             = DPP_CONV_FACTOR2,
  },

  .hSpeedSamplingFreqHz        = MEDIUM_FREQUENCY_TASK_RATE2,
  .hTransitionSteps            = (int16_t)(TF_REGULATION_RATE2 * TRANSITION_DURATION2 / 1000.0),
};

</#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") || M2_ENCODER || (MC.M2_SPEED_SENSOR == "SENSORLESS_ADC") -->

<#if (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true)>
/**
  * temperature sensor parameters Motor 1.
  */
RegConv_t TempRegConv_M1 =
{
  .regADC                = ${MC.M1_TEMP_FDBK_ADC},
  .channel               = MC_${MC.M1_TEMP_FDBK_CHANNEL},
  .samplingTime          = M1_TEMP_SAMPLING_TIME,
};  
  
NTC_Handle_t TempSensor_M1 =
{
  .bSensorType             = REAL_SENSOR,

  .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  .hSensitivity            = (int16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)((V0_V * 65536) / ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,
};
<#else><#-- (MC.M1_TEMPERATURE_READING == false  || MC.M1_OV_TEMPERATURE_PROT_ENABLING == false) -->
/**
  * Virtual temperature sensor parameters Motor 1.
  */
NTC_Handle_t TempSensor_M1 =
{
  .bSensorType     = VIRTUAL_SENSOR,
  .hExpectedTemp_d = 555,
  .hExpectedTemp_C = M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE,
};
</#if><#-- (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true) -->

<#if MC.DRIVE_NUMBER != "1">
  <#if (MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true)>
/**
  * temperature sensor parameters Motor 2.
  */
RegConv_t TempRegConv_M2 =
{
  .regADC                = ${MC.M2_TEMP_FDBK_ADC},
  .channel               = MC_${MC.M2_TEMP_FDBK_CHANNEL},
  .samplingTime          = M2_TEMP_SAMPLING_TIME,
};
  
NTC_Handle_t TempSensor_M2 =
{
  .bSensorType             = REAL_SENSOR,
  .hLowPassFilterBW        = M2_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d2),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d2 - OV_TEMPERATURE_HYSTERESIS_d2),
  .hSensitivity            = (int16_t)(ADC_REFERENCE_VOLTAGE/dV_dT2),
  .wV0                     = (uint16_t)((V0_V2 * 65536) / ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C2,
};
  <#else><#-- (MC.M2_TEMPERATURE_READING == false  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == false) -->
/**
  * Virtual temperature sensor parameters Motor 2.
  */
NTC_Handle_t TempSensor_M2 =
{
  .bSensorType = VIRTUAL_SENSOR,
  .hExpectedTemp_d = 555,
  .hExpectedTemp_C = M2_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE,
};
  </#if><#-- (MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true) -->
</#if><#-- (MC.DRIVE_NUMBER > 1 -->

<#if MC.M1_BUS_VOLTAGE_READING == true>
/* Bus voltage sensor value filter buffer */
static uint16_t RealBusVoltageSensorFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 1.
  */
RegConv_t VbusRegConv_M1 =
{
  .regADC                   = ${MC.M1_VBUS_ADC},
  .channel                  = MC_${MC.M1_VBUS_CHANNEL},
  .samplingTime             = M1_VBUS_SAMPLING_TIME,
};
    
RDivider_Handle_t BusVoltageSensor_M1 =
{
  ._Super =
  {
    .SensorType               = REAL_SENSOR,
    .ConversionFactor         = (uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR),
  },
  
  .LowPassFilterBW            =  M1_VBUS_SW_FILTER_BW_FACTOR,
  .OverVoltageThreshold       = OVERVOLTAGE_THRESHOLD_d,
  <#if MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE">
  .OverVoltageThresholdLow    = OVERVOLTAGE_THRESHOLD_LOW_d,
  <#else><#-- MC.M1_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE" -->
  .OverVoltageThresholdLow    = OVERVOLTAGE_THRESHOLD_d,
  </#if><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" -->
  .OverVoltageHysteresisUpDir = true,
  .UnderVoltageThreshold      =  UNDERVOLTAGE_THRESHOLD_d,
  .aBuffer                    = RealBusVoltageSensorFilterBufferM1,
};
<#else><#-- MC.M1_BUS_VOLTAGE_READING == false -->
/**
  * Virtual bus voltage sensor parameters Motor 1.
  */
VirtualBusVoltageSensor_Handle_t BusVoltageSensor_M1 =
{
  ._Super =
  {
    .SensorType       = VIRTUAL_SENSOR,
    .ConversionFactor = 500,
  },

  .ExpectedVbus_d     = 1 + ((NOMINAL_BUS_VOLTAGE_V * 65536) / 500),
};
</#if><#-- MC.M1_BUS_VOLTAGE_READING == true -->

<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_BUS_VOLTAGE_READING == true>
/* Bus voltage sensor value filter buffer */
uint16_t RealBusVoltageSensorFilterBufferM2[M2_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 2.
  */
RegConv_t VbusRegConv_M2 =
{
  .regADC                   = ${MC.M2_VBUS_ADC},
  .channel                  = MC_${MC.M2_VBUS_CHANNEL},
  .samplingTime             = M2_VBUS_SAMPLING_TIME,
};

RDivider_Handle_t BusVoltageSensor_M2 =
{
  ._Super =
  {
    .SensorType               = REAL_SENSOR,
    .ConversionFactor         = (uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR2),
  },

  .LowPassFilterBW            = M2_VBUS_SW_FILTER_BW_FACTOR,
  .OverVoltageThreshold       = OVERVOLTAGE_THRESHOLD_d2,
      <#if MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE">
  .OverVoltageThresholdLow    = OVERVOLTAGE_THRESHOLD_LOW_d2,
      <#else><#-- MC.M2_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE" -->
  .OverVoltageThresholdLow    = OVERVOLTAGE_THRESHOLD_d2,
      </#if><#-- MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" -->
  .OverVoltageHysteresisUpDir = true,
  .UnderVoltageThreshold      = UNDERVOLTAGE_THRESHOLD_d2,
  .aBuffer                    = RealBusVoltageSensorFilterBufferM2,
};
  <#else><#-- MC.M2_BUS_VOLTAGE_READING == false --> 
/**
  * Virtual bus voltage sensor parameters Motor 2.
  */
VirtualBusVoltageSensor_Handle_t BusVoltageSensor_M2 =
{
  ._Super =
  {
    .SensorType       = VIRTUAL_SENSOR,
    .ConversionFactor = 500,
  },

  .ExpectedVbus_d     = 1 + ((NOMINAL_BUS_VOLTAGE_V2 * 65536) / 500),
};
  </#if><#-- MC.M2_BUS_VOLTAGE_READING == true -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

<#if MC.M1_PWM_DRIVER_PN == "STSPIN32G4" >
/**
 * @brief Handler of STSPIN32G4 driver.
 */
STSPIN32G4_HandleTypeDef HdlSTSPING4;
</#if><#-- MC.M1_PWM_DRIVER_PN == "STSPIN32G4" -->

<#if MC.M1_POTENTIOMETER_ENABLE == true>
/** @brief */
uint16_t SpeedPotentiometer_ValuesBuffer_M1[ 1 << POTENTIOMETER_LPF_BANDWIDTH_POW2_M1 ];

/*
 * Speed Potentiometer instance for motor 1
 */
RegConv_t PotRegConv_M1 =
{
  .regADC              = ${MC.POTENTIOMETER_ADC},
  .channel             = MC_${MC.POTENTIOMETER_CHANNEL},
  .samplingTime        = POTENTIOMETER_ADC_SAMPLING_TIME_M1,
};
  
SpeedPotentiometer_Handle_t SpeedPotentiometer_M1 =
{
  .Pot =
  {
    .LPFilterBandwidthPOW2 = POTENTIOMETER_LPF_BANDWIDTH_POW2_M1,
    .PotMeasArray          = (uint16_t *)SpeedPotentiometer_ValuesBuffer_M1
  },

  .pMCI                    = & Mci[0],
  .RampSlope               = (uint32_t)(POTENTIOMETER_RAMP_SLOPE_M1),
  .ConversionFactor        = (uint16_t)((65536.0) / ((POTENTIOMETER_MAX_SPEED_M1) - (POTENTIOMETER_MIN_SPEED_M1))),
  .SpeedAdjustmentRange    = (uint16_t)(POTENTIOMETER_SPEED_ADJUSTMENT_RANGE_M1),
  .MinimumSpeed            = (uint16_t)(POTENTIOMETER_MIN_SPEED_M1),
};
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true -->

<#if MC.DRIVE_NUMBER != "1">
<#if MC.M2_POTENTIOMETER_ENABLE == true>
/** @brief */
uint16_t SpeedPotentiometer_ValuesBuffer_M2[ 1 << POTENTIOMETER_LPF_BANDWIDTH_POW2_M2 ];

/*
 * Speed Potentiometer instance for motor 2
 */
RegConv_t PotRegConv_M2 =
{
  .regADC              = ${MC.POTENTIOMETER_ADC2},
  .channel             = MC_${MC.POTENTIOMETER_CHANNEL2},
  .samplingTime        = POTENTIOMETER_ADC_SAMPLING_TIME_M2,
};
  
SpeedPotentiometer_Handle_t SpeedPotentiometer_M2 =
{
  .Pot =
  {
    .LPFilterBandwidthPOW2 = POTENTIOMETER_LPF_BANDWIDTH_POW2_M2,
    .PotMeasArray          = (uint16_t *)SpeedPotentiometer_ValuesBuffer_M2
  },

  .pMCI                    = & Mci[1],
  .RampSlope               = (uint32_t)(POTENTIOMETER_RAMP_SLOPE_M2),
  .ConversionFactor        = (uint16_t)((65536.0) / ((POTENTIOMETER_MAX_SPEED_M2) - (POTENTIOMETER_MIN_SPEED_M2))),
  .SpeedAdjustmentRange    = (uint16_t)(POTENTIOMETER_SPEED_ADJUSTMENT_RANGE_M2),
  .MinimumSpeed            = (uint16_t)(POTENTIOMETER_MIN_SPEED_M2),
};
</#if><#-- MC.M2_POTENTIOMETER_ENABLE == true -->
</#if><#-- MC.DRIVE_NUMBER != "1" -->

<#if DWT_CYCCNT_SUPPORTED>
  <#if MC.DBG_MCU_LOAD_MEASURE == true>
/* Performs the CPU load measure of FOC main tasks */
MC_Perf_Handle_t PerfTraces;
  </#if><#--  MC.DBG_MCU_LOAD_MEASURE == true -->
</#if><#-- DWT_CYCCNT_SUPPORTED -->

<#if M1_ENCODER || M2_ENCODER>
EncAlign_Handle_t *pEAC[NBR_OF_MOTORS];
</#if><#-- M1_ENCODER || M2_ENCODER -->
PWMC_Handle_t *pwmcHandle[NBR_OF_MOTORS];

<#if MC.ESC_ENABLE == true>
ESC_Handle_t ESC_M1 =
{
  .pESC_params = &ESC_ParamsM1,
} ;
</#if><#-- MC.ESC_ENABLE == true -->

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */ 

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

