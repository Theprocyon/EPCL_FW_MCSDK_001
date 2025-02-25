<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_macro.ftl">
/**
  ******************************************************************************
  * @file    mc_config.c 
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
#include "mc_config.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */ 

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */ 

/**
  * @brief  PI / PID Speed loop parameters Motor 1.
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
<#if MC.DRIVE_MODE == "VM">
  .wUpperIntegralLimit = (int32_t)(PERIODMAX * SP_KIDIV),
  .wLowerIntegralLimit = 0,
  .hUpperOutputLimit   = (int16_t)PERIODMAX,
  .hLowerOutputLimit   = 0,
<#else>
  .wUpperIntegralLimit = (int32_t)(PERIODMAX_REF * SP_KIDIV),
  .wLowerIntegralLimit = 0,
  .hUpperOutputLimit   = (int16_t)PERIODMAX_REF,
  .hLowerOutputLimit   = 0,
</#if><#-- MC.DRIVE_MODE == "VM" -->
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true> 
/**
  * @brief  Openloop sixstep Controller parameters Motor 1.
  */
OpenLoopSixstepCtrl_Handle_t OpenLoopSixstepCtrllM1 =
{
  .Openloop                   = 0,                                          /* Openloop flag */
  .RevUp                      = 1,                                          /* RevUp flag */
  .DutyCycleRefTable          = {0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U},      /* Max table size initialization */
  .DutyCycleRefCounter        = M1_COMMUTATION_STEP_TIME_BUF_SIZE,          /* DutyCycleRef counter value */
  .DutyCycleRef               = 25U,                                        /* DutyCycleRef */
  .DutyCycleRefTableIndex     = 0U,                                         /* DutyCycleRef table current pointer */
  .DutyCycleRefTableSize      = M1_COMMUTATION_STEP_TIME_BUF_SIZE,          /* DutyCycleRef table size */
  .CurrentFactor              = 50U,                                        /* Curent factor for openloop speed control % of max DutyCycle */
  .VoltageFactor              = 85U,                                        /* Voltage factor for openloop speed control % of max DutyCycle */
  .OnSensing                  = 0,                                          /* OnSensing flag */
};
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true-->

SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz             = MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit = (uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit = (int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit = (int16_t)(-MAX_APPLICATION_SPEED_UNIT),
<#if MC.DRIVE_MODE == "VM">
  .MaxPositiveDutyCycle       = (uint16_t)PERIODMAX / 2U,
<#else><#-- MC.DRIVE_MODE != "VM" -->
  .MaxPositiveDutyCycle       = (uint16_t)PERIODMAX_REF,
</#if><#-- MC.DRIVE_MODE == "VM" -->
  .ModeDefault                = DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault     = (int16_t)(DEFAULT_TARGET_SPEED_UNIT),
 
<#if MC.CURRENT_LIMITER_OFFSET>
  .DutyCycleRefDefault        = (uint16_t)PERIODMAX_REF,
<#else><#-- != MC.CURRENT_LIMITER_OFFSET -->
  .DutyCycleRefDefault        = 0,
</#if><#-- MC.CURRENT_LIMITER_OFFSET -->
};

<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC")>
RevUpCtrl_Handle_t RevUpControlM1 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG)* 65536/360),
  .bFirstAccelerationStage = (ENABLE_SL_ALGO_FROM_PHASE-1u),
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED_UNIT,

<#if MC.DRIVE_MODE == "VM">
  .ParamsData  = 
  {
    {(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_UNIT),
    (uint16_t)PHASE1_VOLTAGE_DPP,&RevUpControlM1.ParamsData[1]},
    {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_UNIT),
    (uint16_t)PHASE2_VOLTAGE_DPP,&RevUpControlM1.ParamsData[2]},
    {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_UNIT),
    (uint16_t)PHASE3_VOLTAGE_DPP,&RevUpControlM1.ParamsData[3]},
    {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_UNIT),
    (uint16_t)PHASE4_VOLTAGE_DPP,&RevUpControlM1.ParamsData[4]},
    {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_UNIT),
    (uint16_t)PHASE5_VOLTAGE_DPP,(void*)MC_NULL},
  },
<#elseif MC.DRIVE_MODE == "CM">
  .ParamsData =
  {
    {(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_UNIT),
    PHASE1_FINAL_CURRENT_REF,&RevUpControlM1.ParamsData[1]},
    {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_UNIT),
    PHASE2_FINAL_CURRENT_REF,&RevUpControlM1.ParamsData[2]},
    {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_UNIT),
    PHASE3_FINAL_CURRENT_REF,&RevUpControlM1.ParamsData[3]},
    {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_UNIT),
    PHASE4_FINAL_CURRENT_REF,&RevUpControlM1.ParamsData[4]},
    {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_UNIT),
    PHASE5_FINAL_CURRENT_REF,(void*)MC_NULL},
  },
<#else><#-- MC.DRIVE_MODE != "VM" ||  MC.DRIVE_MODE != "CM" -->
  .ParamsData =
  {
    {(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_UNIT),(uint16_t)PHASE1_FINAL_CURRENT,&RevUpControlM1.ParamsData[1]},
    {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_UNIT),(uint16_t)PHASE2_FINAL_CURRENT,&RevUpControlM1.ParamsData[2]},
    {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_UNIT),(uint16_t)PHASE3_FINAL_CURRENT,&RevUpControlM1.ParamsData[3]},
    {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_UNIT),(uint16_t)PHASE4_FINAL_CURRENT,&RevUpControlM1.ParamsData[4]},
    {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_UNIT),(uint16_t)PHASE5_FINAL_CURRENT,(void*)MC_NULL},
  },
</#if><#-- MC.M1_DRIVE_TYPE == "SIX_STEP" &&  MC.DRIVE_MODE == "VM" -->
};
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC") -->

<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
Bemf_ADC_Handle_t Bemf_ADC_M1 =
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

  .Pwm_H_L =
  {
    .AdcThresholdPwmPerc       = 10 * BEMF_THRESHOLD_PWM_PERC,
    .AdcThresholdHighPerc      = 10 * BEMF_THRESHOLD_HIGH_PERC,
    .AdcThresholdLowPerc       = 10 * BEMF_THRESHOLD_LOW_PERC,
    .Bus2ThresholdConvFactor   = BEMF_BUS2THRES_FACTOR,
    .ThresholdCorrectFactor    = BEMF_CORRECT_FACTOR,
    .SamplingPointOff          = BEMF_ADC_TRIG_TIME,
  <#if MC.DRIVE_MODE == "VM">
    .SamplingPointOn           = BEMF_ADC_TRIG_TIME_ON,
  </#if><#-- MC.DRIVE_MODE == "VM" -->	
  <#if  CondFamily_STM32G4>
    .AWDfiltering              = ADC_AWD_FILTER_NUMBER + 1,
  </#if><#-- CondFamily_STM32G4 -->	
  },
  <#if  MC.DRIVE_MODE == "VM">
  .OnSensingEnThres            = BEMF_PWM_ON_ENABLE_THRES,
  .OnSensingDisThres           = BEMF_PWM_ON_DISABLE_THRES,
  <#else><#-- MC.DRIVE_MODE != "VM" -->
  .OnSensingEnThres            = PWM_PERIOD_CYCLES,
  .OnSensingDisThres           = PWM_PERIOD_CYCLES,
  </#if><#-- MC.DRIVE_MODE == "VM" -->
  .ComputationDelay            = (uint8_t) (- COMPUTATION_DELAY),
  .ZcRising2CommDelay          = ZCD_RISING_TO_COMM_9BIT,
  .ZcFalling2CommDelay         = ZCD_FALLING_TO_COMM_9BIT,
  .pParams_str                 = &Bemf_ADC_ParamsM1,
  .TIMClockFreq                = SYSCLK_FREQ,
  .PWMFreqScaling              = PWM_FREQ_SCALING,
  .SpeedSamplingFreqHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize             = BEMF_AVERAGING_FIFO_DEPTH,
  .LowFreqTimerPsc             = LF_TIMER_PSC,
  .StartUpConsistThreshold     = NB_CONSECUTIVE_TESTS,

  .DemagParams =
  {
    .DemagMinimumSpeedUnit     = DEMAG_MINIMUM_SPEED,
    .RevUpDemagSpeedConv       = DEMAG_REVUP_CONV_FACTOR,
    .RunDemagSpeedConv         = DEMAG_RUN_CONV_FACTOR,
    .DemagMinimumThreshold     = MIN_DEMAG_COUNTER_TIME,
  },

  .DriveMode                   = DEFAULT_DRIVE_MODE,
};
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->

<#if MC.DRIVE_MODE == "CM">
CurrentRef_Handle_t CurrentRef_M1 =
{
  .pParams_str = &CurrentRef_ParamsM1,
  .PWMperiod   = PWM_PERIOD_CYCLES_REF,
  <#if MC.CURRENT_LIMITER_OFFSET>
  .StartCntPh  = (uint16_t)PWM_PERIOD_CYCLES_REF,
  <#else><#-- MC.CURRENT_LIMITER_OFFSET == false -->
  .StartCntPh  = 0,
  </#if><#-- MC.CURRENT_LIMITER_OFFSET -->
};
</#if><#-- MC.DRIVE_MODE == "CM" -->

PWMC_Handle_t PWM_Handle_M1 =
{
  <#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" && MC.DRIVE_MODE == "CM">
  .StartCntPh                = (uint16_t) (0.8*BEMF_ADC_TRIG_TIME),
  <#else><#-- MC.DRIVE_MODE == "CM" -->
  .StartCntPh                = PWM_PERIOD_CYCLES,
  </#if><#-- MC.DRIVE_MODE == "CM" -->
  .TurnOnLowSidesAction      = false,
  .PWMperiod                 = PWM_PERIOD_CYCLES,
  .OverCurrentFlag  = 0,
  .OverVoltageFlag  = 0,
  .BrakeActionLock  = 0,
  .driverProtectionFlag = false,
  <#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO">
  .TimerCfg = &ThreePwm_TimerCfgM1,
  .QuasiSynchDecay  = false,
  <#else>
  .TimerCfg = &SixPwm_TimerCfgM1,
    <#if  MC.QUASI_SYNC>
  .QuasiSynchDecay  = true,
    <#else>
  .QuasiSynchDecay  = false,
    </#if><#-- MC.QUASI_SYNC -->
  </#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" --> 
    <#if  MC.FAST_DEMAG>
  .LSModArray = {1,0,1,0,1,0},
    <#else>
  .LSModArray = {0,0,0,0,0,0},
    </#if><#-- MC.QUASI_SYNC -->
  .pParams_str = &PWMC_ParamsM1,
  .LowSideOutputs    = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
};

<#if (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR")>
/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL.
  */
HALL_Handle_t HALL_M1 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15 * MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = 0,
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED,
    .DPPConvFactor             = DPP_CONV_FACTOR,
  },

  .SensorPlacement             = HALL_SENSORS_PLACEMENT,
  .PhaseShift                  = (int16_t)(HALL_PHASE_SHIFT * 65536 / 360),
  .SpeedSamplingFreqHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize             = HALL_AVERAGING_FIFO_DEPTH,
  .TIMClockFreq                = HALL_TIM_CLK,
  .TIMx                        = ${_last_word(MC.M1_HALL_TIMER_SELECTION)},
  .ICx_Filter                  = M1_HALL_IC_FILTER_LL,
  .PWMFreqScaling              = PWM_FREQ_SCALING,
  .HallMtpa                    = HALL_MTPA,
  .H1Port                      = M1_HALL_H1_GPIO_Port,
  .H1Pin                       = M1_HALL_H1_Pin,
  .H2Port                      = M1_HALL_H2_GPIO_Port,
  .H2Pin                       = M1_HALL_H2_Pin,
  .H3Port                      = M1_HALL_H3_GPIO_Port,
  .H3Pin                       = M1_HALL_H3_Pin,
};
</#if><#-- (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR") -->

SixStepVars_t SixStepVars[NBR_OF_MOTORS];
<#if MC.DRIVE_NUMBER != "1">
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS]    = {&SpeednTorqCtrlM1 ,&SpeednTorqCtrlM2};
PID_Handle_t *pPIDIq[NBR_OF_MOTORS]             = {&PIDIqHandle_M1 ,&PIDIqHandle_M2};
PID_Handle_t *pPIDId[NBR_OF_MOTORS]             = {&PIDIdHandle_M1 ,&PIDIdHandle_M2};
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS] = {&TempSensor_M1 ,&TempSensor_M2};
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS]  = {&PQD_MotorPowMeasM1,&PQD_MotorPowMeasM2};   
  <#if MC.M1_POSITION_CTRL_ENABLING == true || MC.M2_POSITION_CTRL_ENABLING == true>
PosCtrl_Handle_t *pPosCtrl[NBR_OF_MOTORS]       = {<#if MC.M1_POSITION_CTRL_ENABLING >&PosCtrlM1<#else>MC_NULL</#if>,
                                                   <#if MC.M2_POSITION_CTRL_ENABLING>&PosCtrlM2<#else>MC_NULL</#if>};
  </#if><#-- MC.M1_POSITION_CTRL_ENABLING == false && MC.M2_POSITION_CTRL_ENABLING == false -->
  <#if MC.M1_FLUX_WEAKENING_ENABLING == true || MC.M2_FLUX_WEAKENING_ENABLING == true>
FW_Handle_t *pFW[NBR_OF_MOTORS]                 = {<#if MC.M1_FLUX_WEAKENING_ENABLING>&FW_M1<#else>MC_NULL</#if>,
                                                   <#if MC.M2_FLUX_WEAKENING_ENABLING>&FW_M2<#else>MC_NULL </#if>};
  </#if><#-- MC.M1_POSITION_CTRL_ENABLING == true || MC.M2_POSITION_CTRL_ENABLING == true -->
  <#if MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true || MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true>
FF_Handle_t *pFF[NBR_OF_MOTORS]                 = {<#if MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING>&FF_M1<#else>MC_NULL</#if>,
                                                <#if MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING>&FF_M2<#else>MC_NULL </#if>};
  </#if><#-- MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true || MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true -->
<#else><#-- MC.DRIVE_NUMBER == 1 -->
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS]    = {&SpeednTorqCtrlM1};
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS] = {&TempSensor_M1};
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true> 
OpenLoopSixstepCtrl_Handle_t *pOLS[NBR_OF_MOTORS] = {&OpenLoopSixstepCtrllM1};
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true-->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

MCI_Handle_t Mci[NBR_OF_MOTORS] =
{
  {
    .pSTC = &SpeednTorqCtrlM1,
    .pSixStepVars = &SixStepVars[0],
<#if MC.M1_POSITION_CTRL_ENABLING == true>
    .pPosCtrl = &PosCtrlM1,
</#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->
    .pPWM = &PWM_Handle_M1, 
    .lastCommand = MCI_NOCOMMANDSYET,
    .hFinalSpeed = 0,
    .hFinalTorque = 0,
    .pScale = &scaleParams_M1,
    .hDurationms = 0,
    .DirectCommand = MCI_NO_COMMAND,
    .State = IDLE,
    .CurrentFaults = MC_NO_FAULTS,
    .PastFaults = MC_NO_FAULTS,
    .CommandState = MCI_BUFFER_EMPTY, 
  },

<#if MC.DRIVE_NUMBER != "1">
  {
    .pSTC = &SpeednTorqCtrlM2, 	
    .pSixStepVars = &SixStepVars[1],
<#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
    .pVSS = &VirtualSpeedSensorM2,
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
<#if MC.M2_POSITION_CTRL_ENABLING == true>
    .pPosCtrl = &PosCtrlM2,
</#if><#-- MC.M2_POSITION_CTRL_ENABLING == true -->
    .pPWM = &PWM_Handle_M2,
    .lastCommand = MCI_NOCOMMANDSYET,
    .hFinalSpeed = 0,
    .hFinalTorque = 0,
    .hDurationms = 0,
    .DirectCommand = MCI_NO_COMMAND,
    .State = IDLE,
    .CurrentFaults = MC_NO_FAULTS,
    .PastFaults = MC_NO_FAULTS,
    .CommandState = MCI_BUFFER_EMPTY,
  },
</#if><#-- MC.DRIVE_NUMBER == true -->
};

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */ 

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

