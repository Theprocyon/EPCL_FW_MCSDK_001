<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
<#include "*/ftl/ip_fct.ftl">
<#include "*/ftl/ip_macro.ftl">
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the 
  *          configuration of the subsystem.
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


/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "parameters_conversion.h"

<#if MC.M1_SPEED_SENSOR != "HSO">
ScaleParams_t scaleParams_M1 =
{
 .voltage = NOMINAL_BUS_VOLTAGE_V/(1.73205 * 32767), /* sqrt(3) = 1.73205 */
 .current = CURRENT_CONV_FACTOR_INV,
 .frequency = (1.15 * MAX_APPLICATION_SPEED_UNIT * U_RPM)/(32768* SPEED_UNIT)
};
</#if> <#-- MC.M1_SPEED_SENSOR != "HSO" -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_SPEED_SENSOR != "HSO">
ScaleParams_t scaleParams_M2 =
{
 .voltage = NOMINAL_BUS_VOLTAGE_V2/(1.73205 * 32767), /* sqrt(3) = 1.73205 */
 .current = CURRENT_CONV_FACTOR_INV2,
 .frequency = (1.15 * MAX_APPLICATION_SPEED_UNIT2 * U_RPM)/(32768* SPEED_UNIT)
};
  </#if> <#-- MC.M2_SPEED_SENSOR != "HSO" -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Current sensor parameters Motor 1 - three shunt - G4 
  */
//cstat !MISRAC2012-Rule-8.4
R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCConfig1 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , (uint32_t)(${getADC("CFG",2,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , (uint32_t)(${getADC("CFG",3,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , (uint32_t)(${getADC("CFG",4,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , (uint32_t)(${getADC("CFG",5,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , (uint32_t)(${getADC("CFG",6,"PHASE_1",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
  .ADCConfig2 = {
                  (uint32_t)(${getADC("CFG",1,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , (uint32_t)(${getADC("CFG",2,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , (uint32_t)(${getADC("CFG",3,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , (uint32_t)(${getADC("CFG",4,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , (uint32_t)(${getADC("CFG",5,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                , (uint32_t)(${getADC("CFG",6,"PHASE_2",1)}U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_${PWM_Timer_M1}_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
 
  .ADCDataReg1 = {
                   ${getADC("IP", 1,"PHASE_1",1)}
                 , ${getADC("IP", 2,"PHASE_1",1)}
                 , ${getADC("IP", 3,"PHASE_1",1)}
                 , ${getADC("IP", 4,"PHASE_1",1)}
                 , ${getADC("IP", 5,"PHASE_1",1)}
                 , ${getADC("IP", 6,"PHASE_1",1)}
                 },
  .ADCDataReg2 = {
                   ${getADC("IP", 1,"PHASE_2",1)}
                 , ${getADC("IP", 2,"PHASE_2",1)}
                 , ${getADC("IP", 3,"PHASE_2",1)}
                 , ${getADC("IP", 4,"PHASE_2",1)}
                 , ${getADC("IP", 5,"PHASE_2",1)}
                 , ${getADC("IP", 6,"PHASE_2",1)}
                 },
 //cstat +MISRAC2012-Rule-12.1 +MISRAC2012-Rule-10.1_R6

  /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter     = REP_COUNTER,
  .Tafter                = TW_AFTER,
  .Tbefore               = TW_BEFORE,
  .Tsampling             = (uint16_t)SAMPLING_TIME,
  .Tcase2                = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3                = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,
  .TIMx                  = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},

/* Internal OPAMP common settings --------------------------------------------*/
<#if MC.M1_USE_INTERNAL_OPAMP>
  .OPAMPParams           = &R3_3_OPAMPParamsM1,
<#else><#-- MC.M1_USE_INTERNAL_OPAMP == false -->
  .OPAMPParams           = MC_NULL,
</#if><#-- MC.M1_USE_INTERNAL_OPAMP -->

/* Internal COMP settings ----------------------------------------------------*/
<#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
  .CompOCPASelection     = ${MC.M1_OCP_COMP_U},
  .CompOCPAInvInput_MODE = ${MC.OCPA_INVERTINGINPUT_MODE},
  .CompOCPBSelection     = ${MC.M1_OCP_COMP_V},
  .CompOCPBInvInput_MODE = ${MC.OCPB_INVERTINGINPUT_MODE},
  .CompOCPCSelection     = ${MC.M1_OCP_COMP_W},
  .CompOCPCInvInput_MODE = ${MC.OCPC_INVERTINGINPUT_MODE},
  <#if MC.M1_OCP_COMP_SRC == "DAC">
  .DAC_OCP_ASelection    = ${MC.M1_OCP_DAC_U},
  .DAC_OCP_BSelection    = ${MC.M1_OCP_DAC_V},
  .DAC_OCP_CSelection    = ${MC.M1_OCP_DAC_W},
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_U, "OUT")},
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_V, "OUT")},
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_${_last_word(MC.M1_OCP_DAC_CHANNEL_W, "OUT")},
  <#else><#-- MC.M1_OCP_COMP_SRC != "DAC" -->
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t)0,
  .DAC_Channel_OCPB      = (uint32_t)0,
  .DAC_Channel_OCPC      = (uint32_t)0,
  </#if><#-- MC.M1_OCP_COMP_SRC == "DAC" -->
<#else><#--  MC.M1_OCP_TOPOLOGY != "EMBEDDED" -->
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t)0,
  .DAC_Channel_OCPB      = (uint32_t)0,
  .DAC_Channel_OCPC      = (uint32_t)0,
</#if><#-- MC.M1_OCP_TOPOLOGY == "EMBEDDED" -->

<#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  .CompOVPSelection      = OVP_SELECTION,
  .CompOVPInvInput_MODE  = OVP_INVERTINGINPUT_MODE,
  <#if MC.DAC_OVP_M1 != "NOT_USED">
  .DAC_OVP_Selection     = ${_first_word(MC.DAC_OVP_M1)},
  .DAC_Channel_OVP       = LL_DAC_CHANNEL_${_last_word(MC.DAC_OVP_M1,"CH")},
  <#else><#-- MC.DAC_OVP_M1 == "NOT_USED" -->
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t)0,
  </#if><#-- MC.DAC_OVP_M1 != "NOT_USED" -->
<#else><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION == false -->
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t)0,
</#if><#-- MC.INTERNAL_OVERVOLTAGEPROTECTION -->

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = ${MC.M1_DAC_CURRENT_THRESHOLD},
  .DAC_OVP_Threshold     = ${MC.M1_OVPREF},

};

/* USER CODE BEGIN Additional parameters */


/* USER CODE END Additional parameters */  

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

