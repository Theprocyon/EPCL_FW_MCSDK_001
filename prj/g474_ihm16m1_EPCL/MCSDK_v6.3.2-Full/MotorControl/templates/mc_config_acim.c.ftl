<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
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
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"
#include "pqd_motor_power_measurement.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
  .ConvFact = PQD_CONVERSION_FACTOR
};

/**
  * @brief  PI / PID Speed loop parameters Motor 1
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
<#if (MC.ACIM_CONFIG == "IFOC" | MC.ACIM_CONFIG == "LSO_FOC" | MC.ACIM_CONFIG == "VF_CL")>
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,
  .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV,
  .hUpperOutputLimit   = (int16_t)IQMAX,
  .hLowerOutputLimit   = -(int16_t)IQMAX,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
</#if>
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

<#if (MC.ACIM_CONFIG == "IFOC" | MC.ACIM_CONFIG == "LSO_FOC")>
/**
  * @brief  PI / PID Iq loop parameters Motor 1
  */
PID_Handle_t PIDIqHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * T_KIDIV,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * T_KIDIV,
  .hUpperOutputLimit   = INT16_MAX,
  .hLowerOutputLimit   = -INT16_MAX,
  .hKpDivisor          = (uint16_t)T_KPDIV,
  .hKiDivisor          = (uint16_t)T_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)T_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 1
  */
PID_Handle_t PIDIdHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * F_KIDIV,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * F_KIDIV,
  .hUpperOutputLimit   = INT16_MAX,
  .hLowerOutputLimit   = -INT16_MAX,
  .hKpDivisor          = (uint16_t)F_KPDIV,
  .hKiDivisor          = (uint16_t)F_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)F_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)F_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

</#if>
/**
  * @brief  SpeednTorque Controller parameters Motor 1
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz             = MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit = (uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit = (int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit = (int16_t)(-MAX_APPLICATION_SPEED_UNIT),
<#if MC.ACIM_CONFIG == "LSO_FOC">
  .MaxPositiveTorque          = (int16_t)NOMINAL_CURRENT,
  .MinNegativeTorque          = -(int16_t)NOMINAL_CURRENT,
</#if><#-- MC.ACIM_CONFIG == "LSO_FOC" -->
  .ModeDefault                = DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault     = (int16_t)(DEFAULT_TARGET_SPEED_UNIT),
<#if MC.ACIM_CONFIG == "LSO_FOC">
  .TorqueRefDefault           = (int16_t)DEFAULT_TORQUE_COMPONENT,
  .IdrefDefault               = (int16_t)DEFAULT_FLUX_COMPONENT_A,
</#if><#-- MC.ACIM_CONFIG == "LSO_FOC" -->
};
PWMC_R3_2_Handle_t PWM_Handle_M1 =
{
  {
    .pFctSetADCSampPointSectX   = &R3_2_SetADCSampPointSectX,
    .pFctGetPhaseCurrents       = &R3_2_GetPhaseCurrents,
    .pFctSwitchOffPwm           = &R3_2_SwitchOffPWM,
    .pFctSwitchOnPwm            = &R3_2_SwitchOnPWM,
    .pFctCurrReadingCalib       = &R3_2_CurrentReadingPolarization,
    .pFctTurnOnLowSides         = &R3_2_TurnOnLowSides,
    .pFctOCPSetReferenceVoltage = MC_NULL,
    .pFctRLDetectionModeEnable  = &R3_2_RLDetectionModeEnable,
    .pFctRLDetectionModeDisable = &R3_2_RLDetectionModeDisable,
    .pFctRLDetectionModeSetDuty = &R3_2_RLDetectionModeSetDuty,
    .hT_Sqrt3                   = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,
    .Sector                     = 0,
    .CntPhA                     = 0,
    .CntPhB                     = 0,
    .CntPhC                     = 0,
    .SWerror                    = 0,
    .TurnOnLowSidesAction       = false,
    .OffCalibrWaitTimeCounter   = 0,
    .Motor                      = M1,
    .RLDetectionMode            = false,
    .Ia                         = 0,
    .Ib                         = 0,
    .Ic                         = 0,
    .PWMperiod                  = PWM_PERIOD_CYCLES,
    .DTCompCnt                  = DTCOMPCNT,
    .Ton                        = TON,
    .Toff                       = TOFF
  },

  .ADCRegularLocked             = false,
  .PhaseAOffset                 = 0,
  .PhaseBOffset                 = 0,
  .PhaseCOffset                 = 0,
  .Half_PWMPeriod               = PWM_PERIOD_CYCLES/2u,
  .pParams_str                  = &R3_2_ParamsM1
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Base Class
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1 =
{

  ._Super = {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED,
    .DPPConvFactor             = DPP_CONV_FACTOR,
    },

  .hSpeedSamplingFreqHz        = MEDIUM_FREQUENCY_TASK_RATE,
  .hTransitionSteps            = (int16_t)(TF_REGULATION_RATE * TRANSITION_DURATION/ 1000.0),

};

<#if M1_ENCODER>
/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Encoder
  */
ENCODER_Handle_t ENCODER_M1 =
{
  ._Super = {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED,
    .DPPConvFactor             = DPP_CONV_FACTOR,
  },

  .PulseNumber                 = M1_ENCODER_PPR*4,
  .RevertSignal                = (FunctionalState)ENC_INVERT_SPEED,
  .SpeedSamplingFreqHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize             = ENC_AVERAGING_FIFO_DEPTH,
  .TIMx                        = TIM2,
  .ICx_Filter                  = M1_ENC_IC_FILTER,

};

/**
  * @brief  Encoder Alignment Controller parameters Motor 1
  */
EncAlign_Handle_t EncAlignCtrlM1 =
{
  .hEACFrequencyHz = MEDIUM_FREQUENCY_TASK_RATE,
  .hFinalTorque    = FINAL_I_ALIGNMENT,
  .hElAngle        = ALIGNMENT_ANGLE_S16,
  .hDurationms     = M1_ALIGNMENT_DURATION,
  .bElToMecRatio   = POLE_PAIR_NUM,
};
</#if>
<#if MC.M1_ICL_ENABLED == true>

ICL_Handle_t ICL_M1 =
{
  .ICLstate		               = ICL_ACTIVE,						
  .hICLTicksCounter          = M1_ICL_CAPS_CHARGING_DELAY_TICKS,
  .hICLSwitchDelayTicks      = M1_ICL_RELAY_SWITCHING_DELAY_TICKS,
  .hICLChargingDelayTicks    = M1_ICL_CAPS_CHARGING_DELAY_TICKS,
  .hICLVoltageThreshold      = M1_ICL_VOLTAGE_THRESHOLD_VOLT,
  .hICLUnderVoltageThreshold = UD_VOLTAGE_THRESHOLD_V,
};
</#if>

/**
  * temperature sensor parameters Motor 1
  */
RegConv_t TempRegConv_M1 =
{
  .regADC       = ${MC.M1_TEMP_FDBK_ADC},
  .channel      = MC_${MC.M1_TEMP_FDBK_CHANNEL},
  .samplingTime = M1_TEMP_SAMPLING_TIME,
};
    
NTC_Handle_t TempSensor_M1 =
{
  .bSensorType             = REAL_SENSOR,
  .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,
};

/* Bus voltage sensor value filter buffer */
uint16_t RealBusVoltageSensorFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 1
  */
RegConv_t VbusRegConv_M1 =
{
    .regADC       = ${MC.M1_VBUS_ADC},
    .channel      = MC_${MC.M1_VBUS_CHANNEL},
    .samplingTime = M1_VBUS_SAMPLING_TIME,
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
<#if   MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE">
  .OverVoltageThresholdLow    = OVERVOLTAGE_THRESHOLD_LOW_d,
<#else>
  .OverVoltageThresholdLow    = OVERVOLTAGE_THRESHOLD_d,
</#if>
  .OverVoltageHysteresisUpDir = true,
  .UnderVoltageThreshold      = UNDERVOLTAGE_THRESHOLD_d,
  .aBuffer                    = RealBusVoltageSensorFilterBufferM1,
};

/** RAMP for Motor1.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 =
{
  .FrequencyHz = TF_REGULATION_RATE
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule = MAX_MODULE,
  .MaxVd     = (uint16_t)(MAX_MODULE * 950 / 1000),
};
<#if MC.M1_ICL_ENABLED == true>

DOUT_handle_t ICLDOUTParamsM1 =
{
  .OutputState      = INACTIVE,
  .hDOutputPort     = M1_ICL_SHUT_OUT_GPIO_Port,
  .hDOutputPin      = M1_ICL_SHUT_OUT_Pin,
  .bDOutputPolarity = ${MC.M1_ICL_DIGITAL_OUTPUT_POLARITY}
};
</#if>
MCI_Handle_t Mci[NBR_OF_MOTORS];
//STM_Handle_t STM[NBR_OF_MOTORS];
<#if (MC.ACIM_CONFIG == "IFOC" | MC.ACIM_CONFIG == "LSO_FOC" | MC.ACIM_CONFIG == "VF_CL")>
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS] = { &SpeednTorqCtrlM1 };
</#if>
<#if (MC.ACIM_CONFIG == "IFOC" | MC.ACIM_CONFIG == "LSO_FOC")>
PID_Handle_t *pPIDIq[NBR_OF_MOTORS] = {&PIDIqHandle_M1};
PID_Handle_t *pPIDId[NBR_OF_MOTORS] = {&PIDIdHandle_M1};
</#if>
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS] = {&TempSensor_M1};
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS] = {&PQD_MotorPowMeasM1};

ACIM_MotorParams_Handle_t ACIM_MotorParams_M1 =
{
 .bPP           = POLE_PAIR_NUM,
<#if (MC.ACIM_CONFIG == "IFOC" | MC.ACIM_CONFIG == "LSO_FOC")>
 .fRs           = RS,
 .fRr           = RR,
 .fLls          = LLS,
 .fLlr          = LLR,
 .fLms          = LMS,
 .fLM           = LM,
 .fLr           = LR,
 .fLs           = LS,
 .fsigma        = SIGMA,
 .ftaur         = TAU_R,
 .ftaus         = TAU_S,
 .fImagn_A      = IMAGN_A,
 .fNominalFrequency_rads = (NOMINAL_FREQ*2.0f*PI)
</#if>
};

#if defined(MAIN_IFOC)
ACIM_IFOC_Handle_t ACIM_IFOC_Component_M1 = 
{
 .pACIM_MotorParams = &ACIM_MotorParams_M1,
 .fCalcAngleExecFreqHz = TF_REGULATION_RATE, /* FOC frequency */
 .fCalcAngleExecTime_s = 1.0f/(float)TF_REGULATION_RATE
};
ACIM_IFOC_Handle_t *pACIM_IFOC_Component_M1 = &ACIM_IFOC_Component_M1;
#endif

#if defined(MAIN_LSO_FOC)
/**
  * @brief  PI LSO parameters Motor 1
  */
PI_Float_Handle_t PI_LSO_Handle_M1 =
{
 .fDefKpGain = LSO_KP,
 .fDefKiGain = LSO_KI,
 .fKs = 0.0f,
 .bAntiWindUpActivation = (FunctionalState)DISABLE,
 .fLowerLimit = -PI_LSO_MAX_OUT,
 .fUpperLimit = PI_LSO_MAX_OUT,
 .fUpperIntegralLimit = (float)4294967296,
 .fLowerIntegralLimit = -(float)4294967296,
 .fExecFrequencyHz = TF_REGULATION_RATE,
};

ACIM_LSO_Handle_t ACIM_LSO_Component_M1 = 
{
  .pACIM_MotorParams = &ACIM_MotorParams_M1,
  .fPI = &PI_LSO_Handle_M1,
  ._SpeedEstimator = {
    .bElToMecRatio             = POLE_PAIR_NUM, /*!< Coefficient used to transform electrical to
                                                     mechanical quantities and viceversa. It usually
                                                     coincides with motor pole pairs number*/
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT), /*!< Maximum value of measured speed that 
                                                                                   is considered to be valid. It's expressed
                                                                                   in tenth of mechanical Hertz.*/
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT), /*!< Minimum value of measured speed that is
                                                                              considered to be valid. It's expressed
                                                                              in tenth of mechanical Hertz.*/
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS, /*!< Maximum value of not valid measurements
                                                                       before an error is reported.*/
    .hMaxReliableMecAccelUnitP = 65535, /*!< Maximum value of measured acceleration
                                             that is considered to be valid. It's
                                             expressed in 01HzP (tenth of Hertz per
                                             speed calculation period)*/
    .hMeasurementFrequency     = TF_REGULATION_RATE, /*!< Frequency on which the user will request
                                                          a measurement of the rotor electrical angle.
                                                          It's also used to convert measured speed from
                                                          tenth of Hz to dpp and viceversa.*/
  },

  .k                           = LSO_K,
  .fCalcAngleExecFreqHz        = TF_REGULATION_RATE, /* FOC frequency */
  .fCalcAngleExecTime_s        = 1.0f / (float)TF_REGULATION_RATE,
  .fMaxObsRotorSpeed_RPM       = PI_LSO_MAX_OUT,
    
#if defined(DEBUG_LSO)
  .fdbg_Flux_K                 = FLUX_K,
#endif
  
};
ACIM_LSO_Handle_t *pACIM_LSO_Component_M1 = &ACIM_LSO_Component_M1;
#endif

#if defined(ACIM_VF)

PWM_Handle_t PWM_ParamsM1 = 
{ 
  .TIMx =  TIM1, /*!< It contains the pointer to the timer used for PWM generation. */
};

/** RAMP for Motor1.
  *
  */
RampExtMngr_Handle_t RampExtMngrVFParamsM1 =
{
  .FrequencyHz = MEDIUM_FREQUENCY_TASK_RATE /*!< Execution frequency expressed in Hz */
    
};

<#if MC.ACIM_CONFIG == "VF_OL" || MC.ACIM_CONFIG == "VF_CL">
/**
  * @brief  PI VF parameters Motor 1
  */
PI_Float_Handle_t PI_VF_Handle_M1 =
{
  /* Medium Frequency Task - Speed loop */
  .fDefKpGain            = PI_VF_SPEED_KP_DEFAULT, /* Tuning performed by GUI, using a Bandwidth w0 = 250 rad/s */
  .fDefKiGain            = PI_VF_SPEED_KI_DEFAULT, /* Tuning performed by GUI, using a Bandwidth w0 = 250 rad/s */
  .fKs                   = 0.0f,
  .bAntiWindUpActivation = (FunctionalState)DISABLE,
  .fLowerLimit           = -PI_VF_SPEED_OUTPUT_LIMIT, /* The output of speed regulator is the max slip frequency expressed in rad/s */
  .fUpperLimit           =  PI_VF_SPEED_OUTPUT_LIMIT, /* The output of speed regulator is the max slip frequency expressed in rad/s */
  .fUpperIntegralLimit   = (float)4294967296,
  .fLowerIntegralLimit   = -(float)4294967296,
  .fExecFrequencyHz      = MEDIUM_FREQUENCY_TASK_RATE
};

</#if>
ACIM_VF_Handle_t ACIM_VF_Component_M1 = 
{
  .pPWM                       = &PWM_ParamsM1,
  .pACIM_MotorParams          = &ACIM_MotorParams_M1,
  .pRMNGR                     = &RampExtMngrVFParamsM1,
  .fFlux_K                    = FLUX_K, /* Vn/fn = to be defined a solid equation */
  .fVoltage_Offset            = VOLTAGE_OFFSET, /* To be defined considering the voltage drop of stator winfding */
  .fMinFreq_TH_Hz             = MIN_FREQUENCY_TH, /* to be defined */
  .fNominalPhaseVoltagePeak_V = (float)NOMINAL_PHASE_VOLTAGE*1.4142f,
  .fCalcAngleExecFreqHz       = TF_REGULATION_RATE,  /*FOC frequency*/ 
  .fCalcAngleExecTime_s       = 1.0f/(float)TF_REGULATION_RATE,
<#if MC.ACIM_CONFIG == "VF_CL">
   
  .fPI = &PI_VF_Handle_M1
</#if>
};
ACIM_VF_Handle_t *pACIM_VF_Component_M1 = &ACIM_VF_Component_M1;
#endif

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

