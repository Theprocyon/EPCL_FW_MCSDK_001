<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
/**
  ******************************************************************************
  * @file    mc_config.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
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
#include "main.h"
#include "mc_type.h"
#include "mc_parameters.h"
#include "mc_config.h"
#include "parameters_conversion.h"
#include "oversampling.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */ 

#define OFFCALIBRWAIT_MS     100

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */ 

PWMC_R3_Handle_t PWM_Handle_M1 =
{
  {
    .pFctGetMeasurements               = &R3_GetMeasurements,
    .pFctSwitchOffPwm                  = &R3_SwitchOffPWM,             
    .pFctSwitchOnPwm                   = &R3_SwitchOnPWM,              
    .pFctTurnOnLowSides                = &R3_TurnOnLowSides,            
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctSetADCSampPointSectX          = &R3_SetADCSampPointSectX,
    .pFctGetAuxAdcMeasurement          = &R3_GetAuxAdcMeasurement,
    .pFctGetCurrentReconstructionData  = &R3_GetCurrentReconstructionData,
    .pFctGetOffsets                    = &R3_GetOffsets,
    .pFctSetOffsets                    = &R3_SetOffsets,
    .pFctPreparePWM                    = &R3_PreparePWM,
    .pFctStartOffsetCalibration        = &R3_StartOffsetCalibration,
    .pFctFinishOffsetCalibration       = &R3_FinishOffsetCalibration,
	.pFctPwmShort                      = &R3_PwmShort,
	.pFctIsInPwmControl                = &R3_IsInPwmControl,
	.pFctIsPwmEnabled                  = &R3_IsPwmEnabled,

    .LowSideOutputs    = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
<#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO">
  <#if MC.M1_SHARED_SIGNAL_ENABLE == false>
    .pwm_en_u_port     = M1_PWM_EN_U_GPIO_Port,
    .pwm_en_u_pin      = M1_PWM_EN_U_Pin,
    .pwm_en_v_port     = M1_PWM_EN_V_GPIO_Port,
    .pwm_en_v_pin      = M1_PWM_EN_V_Pin,
    .pwm_en_w_port     = M1_PWM_EN_W_GPIO_Port,
    .pwm_en_w_pin      = M1_PWM_EN_W_Pin,
  <#else>
    .pwm_en_u_port     = M1_PWM_EN_UVW_GPIO_Port,
    .pwm_en_u_pin      = M1_PWM_EN_UVW_Pin,
    .pwm_en_v_port     = M1_PWM_EN_UVW_GPIO_Port,
    .pwm_en_v_pin      = M1_PWM_EN_UVW_Pin,
    .pwm_en_w_port     = M1_PWM_EN_UVW_GPIO_Port,
    .pwm_en_w_pin      = M1_PWM_EN_UVW_Pin,
  </#if>  
<#else><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING != "ES_GPIO" -->
    .pwm_en_u_port     = MC_NULL,
    .pwm_en_u_pin      = (uint16_t)0,
    .pwm_en_v_port     = MC_NULL,
    .pwm_en_v_pin      = (uint16_t)0,
    .pwm_en_w_port     = MC_NULL,
    .pwm_en_w_pin      = (uint16_t)0,
</#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" -->     
    .CntPhA = PWM_PERIOD_CYCLES >> 2,
    .CntPhB = PWM_PERIOD_CYCLES >> 2,
    .CntPhC = PWM_PERIOD_CYCLES >> 2,
    .SWerror = 0,
    .TurnOnLowSidesAction = false, 
    .OffCalibrWaitTimeCounter = 0, 
    .Motor = M1,     
    .QuarterPeriodCnt   = PWM_PERIOD_CYCLES >> 2,
    .OffCalibrWaitTicks = (uint16_t)((SPEED_LOOP_FREQUENCY_HZ * OFFCALIBRWAIT_MS)/ 1000),
    
    .flagEnableCurrentReconstruction = true,
<#if MC.MOTOR_PROFILER == true>
<#-- set SW OCP to maxixum current readable by ADC when profiler is used  --> 
  <#if MC.M1_CURRENT_SENSING_TOPO == "THREE_SHUNT">
    .softOvercurrentTripLevel_pu = FIXP30(((float_t)ADC_REFERENCE_VOLTAGE/(float_t)AMPLIFICATION_GAIN/(float_t)RSHUNT) / CURRENT_SCALE),
  <#else>
    .softOvercurrentTripLevel_pu = FIXP30(((float_t)ADC_REFERENCE_VOLTAGE/(float_t)AMPLIFICATION_GAIN) / CURRENT_SCALE),
  </#if><#-- MC.M1_CURRENT_SENSING_TOPO == "THREE_SHUNT" -->     
<#else>
    .softOvercurrentTripLevel_pu = FIXP30(BOARD_SOFT_OVERCURRENT_TRIP / CURRENT_SCALE),
</#if><#-- MC.MOTOR_PROFILER == "true" -->    
    .polpulseHandle = &MC_PolPulse_M1.PolpulseObj,
    .modulationMode = FOC_MODULATIONMODE_Centered, 

  },

  .wPhaseOffsetFilterConst = FIXP31(200.0f / PWM_FREQUENCY),  /* Set filter cutoff frequency to 200 rad/s */

  .maxTicksCurrentReconstruction = (uint16_t)(MAX_TICKS_CURRENT_RECONSTRUCTION_THRESHOLD),

  .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u, 
  .OverCurrentFlag = false,    
  .OverVoltageFlag = false,    
  .BrakeActionLock = false,
  .pParams_str = &R3_ParamsM1
};
  
Oversampling_t oversampling =
{
  .count = OVS_COUNT,
  .divider = REGULATION_EXECUTION_RATE,
  .singleIdx = BOARD_SHUNT_SAMPLE_SELECT, 
  .readBuffer = 1,
  .writeBuffer = 0,
  .Channel =
  {
    /* I_R */
    {
      .adc = OVS_ADC_I_R,
      .rank = OVS_RANK_I_R,
      .tasklen = OVS_${MC.M1_CS_ADC_U}_TASKLENGTH,
<#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT')>	      
      .single = true,
<#else>
      .single = false,
</#if>      
    },
    /* I_S */
    {
      .adc = OVS_ADC_I_S,
      .rank = OVS_RANK_I_S,
      .tasklen = OVS_${MC.M1_CS_ADC_V}_TASKLENGTH,      
<#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT')>	      
      .single = true,
<#else>
      .single = false,
</#if>
    },
    /* I_T */
    {
      .adc = OVS_ADC_I_T,
      .rank = OVS_RANK_I_T,
      .tasklen = OVS_${MC.M1_CS_ADC_W}_TASKLENGTH,      
<#if (MC.M1_CURRENT_SENSING_TOPO=='THREE_SHUNT')>	      
      .single = true,
<#else>
      .single = false,
</#if>   
    },
    /* U_R */
    {
       .adc = OVS_ADC_U_R,
       .rank = OVS_RANK_U_R,
       .tasklen = OVS_${MC.M1_VS_ADC_U}_TASKLENGTH,       
    },
    /* U_S */
    {
      .adc = OVS_ADC_U_S,
      .rank = OVS_RANK_U_S,
      .tasklen = OVS_${MC.M1_VS_ADC_V}_TASKLENGTH,        
    },
    /* U_T */
    {
        .adc = OVS_ADC_U_T,
        .rank = OVS_RANK_U_T,
        .tasklen = OVS_${MC.M1_VS_ADC_W}_TASKLENGTH,        
    },
<#if MC.M1_BUS_VOLTAGE_READING>    
  <#if (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_U) || (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_V) || (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_W)>
    /* U_DC */
    {
        .adc = OVS_ADC_U_DC,
        .rank = OVS_RANK_U_DC,
        .tasklen = OVS_${MC.M1_VBUS_ADC}_TASKLENGTH,         
    },
  </#if>  
</#if>    
<#if MC.M1_POTENTIOMETER_ENABLE> 
  <#if (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_U) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_V) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_W)>
    /* Throttle */
    {
      .adc = OVS_ADC_THROTTLE,
      .rank = OVS_RANK_THROTTLE,
      .tasklen = OVS_${MC.POTENTIOMETER_ADC}_TASKLENGTH,       
    },
</#if>   
</#if>  
<#if MC.M1_TEMPERATURE_READING >
  <#if (MC.M1_TEMP_FDBK_ADC ==  MC.M1_CS_ADC_U) || (MC.M1_TEMP_FDBK_ADC ==  MC.M1_CS_ADC_V) || (MC.M1_TEMP_FDBK_ADC ==  MC.M1_CS_ADC_W)>
    /* Temperature*/
    {
      .adc = OVS_ADC_TEMP,
      .rank = OVS_RANK_TEMP,
      .tasklen = OVS_${MC.M1_TEMP_FDBK_ADC}_TASKLENGTH,       
    },
</#if>  
</#if>   
  }
};

<#if MC.M1_BUS_VOLTAGE_READING>    
  <#if (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_U) && (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_V) && (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_W)>
/**
  * Vbus sensor parameters Motor 1.
  */
RegConv_t VbusRegConv_M1 =
{
  .regADC                = ${MC.M1_VBUS_ADC},
  .channel               = MC_${MC.M1_VBUS_CHANNEL},
  .samplingTime          = M1_VBUS_SAMPLING_TIME,
}; 
</#if>  
</#if>

<#if (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true)>
  <#if (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_U) && (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_V) && (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_W)>
/**
  * temperature sensor parameters Motor 1.
  */
RegConv_t TempRegConv_M1 =
{
  .regADC                = ${MC.M1_TEMP_FDBK_ADC},
  .channel               = MC_${MC.M1_TEMP_FDBK_CHANNEL},
  .samplingTime          = M1_TEMP_SAMPLING_TIME,
};  

  </#if>  
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



<#if MC.M1_ICL_ENABLED == true>

DOUT_handle_t ICLDOUTParamsM1 =
{
  .OutputState       = INACTIVE,                   
  .hDOutputPort      = M1_ICL_SHUT_OUT_GPIO_Port,
  .hDOutputPin       = M1_ICL_SHUT_OUT_Pin,		
  .bDOutputPolarity  = ${MC.M1_ICL_DIGITAL_OUTPUT_POLARITY}		
};

ICL_Handle_t ICL_M1 =
{
  .ICLstate		            	 = ICL_ACTIVE,						
  .hICLTicksCounter	         = M1_ICL_CAPS_CHARGING_DELAY_TICKS,    															
  .hICLSwitchDelayTicks	     = M1_ICL_RELAY_SWITCHING_DELAY_TICKS,
  .hICLChargingDelayTicks	   = M1_ICL_CAPS_CHARGING_DELAY_TICKS,	
  .hICLVoltageThreshold	     = M1_ICL_VOLTAGE_THRESHOLD,
  .hICLUnderVoltageThreshold = UD_VOLTAGE_THRESHOLD_V,
};

</#if> <#-- MC.M1_ICL_ENABLED == true -->


RsDCEstimation_Handle_t RsDCEst_M1 =
{
  .pSTC = &STC_M1,
  .pCurrCtrl = &CurrCtrl_M1,
  .pSPD = &SPD_M1, 
  .pPWM = &PWM_Handle_M1._Super,
  
  .flag_enableRSDCestimate = false,
  .RSDCestimate_AlignOnly = false,
  .RSDCestimate_FilterShift = 10,
  .RSDCestimate_counterInitialValue = RSDC_TOTAL_MEASURE_TIME_TICKS,
  .RSDCestimate_counterMeasureValue = RSDC_SLOP_TICKS,
  .RSDCestimate_Id_ref_pu = FIXP30(NOMINAL_CURRENT_A * 0.5f / CURRENT_SCALE),
<#if MC.M1_SPEED_SENSOR == "ZEST">  
  .RSDCestimate_Fast = false,
  .FastRsDC_measure_duration = (uint32_t) (FAST_RSDC_TOTAL_MEASURE_TIME_MS * SPEED_LOOP_FREQUENCY_HZ /1000),
  .FastRsDC_ramp_duration = (uint32_t) (FAST_RSDC_SLOP_MS * SPEED_LOOP_FREQUENCY_HZ /1000),
</#if>  
};

IMPEDCORR_Obj ImpedCorr_M1;
HSO_Obj       HSO_M1;
<#if MC.M1_SPEED_SENSOR == "ZEST">
ZEST_Obj      ZeST_M1;
RSEST_Obj     RsEst_M1;

</#if>
SPD_Handle_t SPD_M1 =
{
<#if MC.M1_SPEED_SENSOR == "ZEST">
  .zestControl =
  {
    .enableInjection = true,
    .enableCorrection = true,	
    .enableFlip = true,
    .enableRun = true,
    .enableRunBackground = true,
    .enableControlUpdate = true,
    .injectFreq_kHz = FIXP30(MOTOR_ZEST_INJECT_FREQ / 1000.0f),			
    .injectId_A_pu= FIXP30(MOTOR_ZEST_INJECT_D / CURRENT_SCALE),		
    .feedbackGainD= FIXP24(MOTOR_ZEST_GAIN_D),		
    .feedbackGainQ= FIXP24(MOTOR_ZEST_GAIN_Q),		
    .injectFreq_Alt_kHz = FIXP30(MOTOR_ZEST_INJECT_FREQ_ALT / 1000.0f),
    .feedbackGainD_Alt= FIXP24(MOTOR_ZEST_GAIN_D_ALT),
    .feedbackGainQ_Alt= FIXP24(MOTOR_ZEST_GAIN_Q_ALT),
  },
  .zestFeedback = 
  {
    .signalD = 0,
    .signalQ = 0,
    .L = 0,
    .R = 0,
    .thresholdFreq_pu = 0,
  },
  .pZeST = &ZeST_M1,
  .pRsEst = &RsEst_M1,  
</#if>  
  .flagDynamicZestConfig = ${MC.M1_MOTOR_DYNAMICQGAIN?c}, 
  .closedLoopAngleEnabled = true,
  .pImpedCorr = &ImpedCorr_M1,
  .pHSO = &HSO_M1,

};

STC_Handle_t STC_M1 =
{
  .pCurrCtrl = &CurrCtrl_M1,
  .accel_factor				      = FIXP30(1.0f),
  .regen_factor				      = FIXP30(1.0f),
  .active_limits.u16			  = 0,
<#if MC.M1_POTENTIOMETER_ENABLE == true>
  .speedref_source          = FOC_SPEED_SOURCE_Throttle,
<#else><#-- MC.M1_POTENTIOMETER_ENABLE == false -->
  .speedref_source          = FOC_SPEED_SOURCE_Default,
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true -->
  .SpeedControlEnabled = false,
  .Enable_InjectBoost = false,
};

IMPEDCORR_Params ImpedCorr_params_M1=
{
  .voltageFilterPole_rps = VOLTAGE_FILTER_POLE_RPS,
  .cycleFreq_Hz = (PWM_FREQUENCY / REGULATION_EXECUTION_RATE),
};

HSO_Params Hso_params_M1 =
{
  .isrTime_s = 1.0 / TF_REGULATION_RATE,
  .speedPole_rps = SPEED_POLE_RPS,
  .CrossOver_Hz = 20.0f,            /* depends on motor used */
  .flag_FilterAdaptive = true,
  .Filter_adapt_fmin_Hz = 10.0f, /* lowest FO cross-over Hz */
  .Filter_adapt_fmax_Hz = 500.0f, /* highest FO cross-over Hz */
  .CheckDirBW_Hz = 0.5f,  
};

<#if MC.M1_SPEED_SENSOR == "ZEST">

ZEST_Params Zest_params_M1 = 
{ 
  .backgroundFreq_Hz = SPEED_LOOP_FREQUENCY_HZ, /* interval for ZEST_runBackground() */
  .isrFreq_Hz = (PWM_FREQUENCY / REGULATION_EXECUTION_RATE), /* Frequency of ZEST_run() calls */
  .speedPole_rps = SPEED_POLE_RPS,
};

RSEST_Params Rsest_params_M1 =
{
  .F_calculate_Hz     = SPEED_LOOP_FREQUENCY_HZ,
  .AmbientCelcius     = 22.0f,
  .RatedCelcius       = 20.0f,
  .correctionGain     = 0.5f,
  .injectGain         = 0.001f,
};
</#if>

POLPULSE_Params PolPulse_params_M1 =
{
  .Ts                   = 1.0f / TF_REGULATION_RATE,
};

MC_PolPulse_Handle_t MC_PolPulse_M1 =
{
  .pPWM = &PWM_Handle_M1._Super,
  .pSPD = &SPD_M1,
  .TIMx = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},              /*!< timer used for PWM generation.*/
  .pulse_countdown = -1,
  .flagPolePulseActivation = true,
};


CurrCtrl_Handle_t CurrCtrl_M1 =
{
  .busVoltageCompEnable  = true, 
  .busVoltageComp        = FIXP24(1.0f),
  .busVoltageCompMax     = FIXP24(20.0f),
  .busVoltageCompMin     = FIXP24(1.0f),
  .currentControlEnabled = true,
  .forceZeroPwm = false,
  .pid_IdIqX_obj =
  {
    .KpAlign = MOTOR_CURRENT_KP_ALIGN,
  }
    
};

PROFILER_Params profilerParams_M1 =
{
  .isrFreq_Hz = (PWM_FREQUENCY / REGULATION_EXECUTION_RATE),
  .background_rate_hz = SPEED_LOOP_FREQUENCY_HZ,
  .fullScaleFreq_Hz = FREQUENCY_SCALE,
  .fullScaleCurrent_A = CURRENT_SCALE,
  .fullScaleVoltage_V = VOLTAGE_SCALE,
  .voltagefilterpole_rps = VOLTAGE_FILTER_POLE_RPS,
  .PolePairs = POLE_PAIR_NUM,
  .dcac_PowerDC_goal_W = 3.0f,
  .dcac_duty_ramping_time_s = 2.0f,
  .dcac_current_ramping_time_s = 0.5f,
  .dcac_dc_measurement_time_s = 2.0f,
<#if MC.M1_SPEED_SENSOR == "HSO">
  .dcac_ac_measurement_time_s = 5.0f,
<#else>
  .dcac_ac_measurement_time_s = 2.0f,
</#if>  
  .dcac_duty_maxgoal = 0.2f,
  .dcac_ac_freq_Hz = 80.0f,
  .dcac_DCmax_current_A = M1_MAX_READABLE_CURRENT*0.8f,	/* max current, power_goal should come first */
  .glue_dcac_fluxestim_on = true,
  .fluxestim_measurement_time_s = 1.5f,
  .fluxestim_Idref_A = 0.0f,
  .fluxestim_current_ramping_time_s = 0.2f,
  .fluxestim_speed_Hz = 50.0f,
  .fluxestim_speed_ramping_time_s = 2.0f,
};


PROFILER_Obj	profiler_M1;

ProfilerMotor_Handle_t profilerMotor_M1 = 
{
  .pSPD = &SPD_M1,
  .pSTC = &STC_M1,
  .pPolpulse = &MC_PolPulse_M1.PolpulseObj,
  .pCurrCtrl = &CurrCtrl_M1,
  .pPWM = &PWM_Handle_M1._Super,
  .pVBus = &VBus_M1,
};

BusVoltageSensor_Handle_t VBus_M1 =
{
  .Udcbus_convFactor = FIXP30(ADC_BUSVOLTAGE_SCALE/VOLTAGE_SCALE),
  .Udcbus_overvoltage_limit_pu = FIXP30(BOARD_LIMIT_OVERVOLTAGE / VOLTAGE_SCALE), 
  .Udcbus_undervoltage_limit_pu = FIXP30(BOARD_LIMIT_UNDERVOLTAGE / VOLTAGE_SCALE), 
};

<#if MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true>
Potentiometer_Handle_t potentiometer_M1 = 
{
  .pPWM = &PWM_Handle_M1._Super,
  .offset = THROTTLE_OFFSET,
  .gain = THROTTLE_GAIN,
  .speedMaxRPM = THROTTLE_SPEED_MAX_RPM, 
  .direction = 1,	
  .speedref_scalefactor = FIXP30((float_t)THROTTLE_SPEED_MAX_RPM / 60.0f * POLE_PAIR_NUM / (float_t)FREQUENCY_SCALE),		/* Speed at full throttle, in per unit frequency */
};
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true -->

<#if MC.M1_POTENTIOMETER_ENABLE == true>
  <#if (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_U) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_V) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_W)>
/*
 * Speed Potentiometer instance for motor 1
 */
RegConv_t PotRegConv_M1 =
{
  .regADC              = ${MC.POTENTIOMETER_ADC},
  .channel             = MC_${MC.POTENTIOMETER_CHANNEL},
  .samplingTime        = POTENTIOMETER_ADC_SAMPLING_TIME_M1,
};
  </#if><#-- (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_U) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_V) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_W) -->
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true -->

<#if MC.M1_PWM_DRIVER_PN == "STSPIN32G4" >
/**
 * @brief Handler of STSPIN32G4 driver.
 */
STSPIN32G4_HandleTypeDef HdlSTSPING4;
</#if><#-- MC.M1_PWM_DRIVER_PN == "STSPIN32G4" -->

<#if MC.MOTOR_SIM == true>
MOTORSIM_Debug_t gMotorSim_Debug;
MOTORSIM_Obj MotorSim_Obj;

MotorSim_Handle_t MotorSim = 
{
  .pCurrCtrl = &CurrCtrl_M1,
  .pPWM = &PWM_Handle_M1._Super,
  .pDebug = &gMotorSim_Debug,
};

MOTORSIM_Params MotorSim_params =
{
  .cyclefrequency	= TF_REGULATION_RATE,
  
  .polepairs = POLE_PAIR_NUM,
  .Flux_Wb = MOTOR_RATED_FLUX / M_TWOPI,
  .Rs	= RS,
  .Ls	= LS,
  .Jm	= MOTOR_INERTIA,
  .fm	= MOTOR_FRICTION,

  .FullScaleVoltage_V = VOLTAGE_SCALE,
  .FullScaleCurrent_A = CURRENT_SCALE,
  .FullScaleFreq_Hz = FREQUENCY_SCALE,
  .FullScaleTorque_mech_Nm = (1.5f * NOMINAL_CURRENT_A * (MOTOR_RATED_FLUX / M_TWOPI) * POLE_PAIRS) * 2.0f,

  .voltageFilterPole_rps	= VOLTAGE_FILTER_POLE_RPS,

};
</#if>

MCI_Handle_t Mci[NBR_OF_MOTORS];

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */ 

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

