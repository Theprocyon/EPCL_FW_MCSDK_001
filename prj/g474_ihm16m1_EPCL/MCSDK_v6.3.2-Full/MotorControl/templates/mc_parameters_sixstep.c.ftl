<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
<#include "*/ftl/ip_fct.ftl">
<#include "*/ftl/ip_macro.ftl">
<#include "*/ftl/sixstep_assign.ftl">
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
<#if MC.ESC_ENABLE>
#include "esc.h"
</#if><#-- MC.ESC_ENABLE -->
#include "parameters_conversion.h"
#include "pwmc_sixstep.h"
<#if  MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
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
<#if MC.DRIVE_MODE == "CM">
#include "current_ref_ctrl.h"
</#if><#-- MC.DRIVE_MODE == "CM" -->

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */
  <#if MC.PHASE_UH_POLARITY == "H_ACTIVE_HIGH">
#define UH_POLARITY                 (uint16_t)(0x0000)
  <#else>
#define UH_POLARITY                 (uint16_t)(0x0002)
  </#if><#-- MC.PHASE_UH_POLARITY == H_ACTIVE_HIGH -->
  <#if MC.PHASE_VH_POLARITY == "H_ACTIVE_HIGH">
#define VH_POLARITY                 (uint16_t)(0x0000)
  <#else>
#define VH_POLARITY                 (uint16_t)(0x0020)
  </#if><#-- MC.PHASE_VH_POLARITY == H_ACTIVE_HIGH -->
  <#if MC.PHASE_WH_POLARITY == "H_ACTIVE_HIGH">
#define WH_POLARITY                 (uint16_t)(0x0000)
  <#else>
#define WH_POLARITY                 (uint16_t)(0x0200)
  </#if><#-- MC.PHASE_WH_POLARITY == H_ACTIVE_HIGH -->
  <#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
    <#if MC.PHASE_UL_POLARITY == "L_ACTIVE_HIGH">
#define UL_POLARITY                 (uint16_t)(0x0000)
    <#else>
#define UL_POLARITY                 (uint16_t)(0x0008)
    </#if><#-- MC.PHASE_UL_POLARITY == H_ACTIVE_HIGH -->
    <#if MC.PHASE_VL_POLARITY == "L_ACTIVE_HIGH">
#define VL_POLARITY                 (uint16_t)(0x0000)
    <#else>
#define VL_POLARITY                 (uint16_t)(0x0080)
    </#if><#-- MC.PHASE_VL_POLARITY == H_ACTIVE_HIGH -->
    <#if MC.PHASE_WL_POLARITY == "L_ACTIVE_HIGH">
#define WL_POLARITY                 (uint16_t)(0x0000)
    <#else>
#define WL_POLARITY                 (uint16_t)(0x0800)
    </#if><#-- MC.PHASE_WL_POLARITY == H_ACTIVE_HIGH -->

#define CCER_POLARITY_STEP14        UH_POLARITY | UL_POLARITY | VH_POLARITY | VL_POLARITY
#define CCER_POLARITY_STEP25        UH_POLARITY | UL_POLARITY | WH_POLARITY | WL_POLARITY
#define CCER_POLARITY_STEP36        WH_POLARITY | WL_POLARITY | VH_POLARITY | VL_POLARITY
#define CCER_POLARITY_MIDSTEP       UH_POLARITY | UL_POLARITY | VH_POLARITY | VL_POLARITY | WH_POLARITY | WL_POLARITY
  <#else>
#define CCER_POLARITY_STEP14        UH_POLARITY | VH_POLARITY
#define CCER_POLARITY_STEP25        UH_POLARITY | WH_POLARITY
#define CCER_POLARITY_STEP36        WH_POLARITY | VH_POLARITY
#define CCER_POLARITY_MIDSTEP       UH_POLARITY | VH_POLARITY | WH_POLARITY
  </#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER" -->

  <#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO">
#define CCER_UH_VH                  (uint16_t)(0x1011)
#define CCER_UH_WH                  (uint16_t)(0x1101)
#define CCER_VH_WH                  (uint16_t)(0x1110)
#define CCER_UH_VH_WH               (uint16_t)(0x1111)
#define CCER_STEP14                 CCER_UH_VH | CCER_POLARITY_STEP14
#define CCER_STEP25                 CCER_UH_WH | CCER_POLARITY_STEP25
#define CCER_STEP36                 CCER_VH_WH | CCER_POLARITY_STEP36
#define CCER_MIDSTEP                CCER_UH_VH_WH | CCER_POLARITY_MIDSTEP
  <#else>
#define CCER_UH_VH_UL_VL            (uint16_t)(0x1055)
#define CCER_UH_WH_UL_WL            (uint16_t)(0x1505)
#define CCER_VH_WH_VL_WL            (uint16_t)(0x1550)
#define CCER_UH_VH_WH_UL_VL_WL      (uint16_t)(0x1555)
#define CCER_STEP14                 CCER_UH_VH_UL_VL | CCER_POLARITY_STEP14
#define CCER_STEP25                 CCER_UH_WH_UL_WL | CCER_POLARITY_STEP25
#define CCER_STEP36                 CCER_VH_WH_VL_WL | CCER_POLARITY_STEP36
#define CCER_MIDSTEP                CCER_UH_VH_WH_UL_VL_WL | CCER_POLARITY_MIDSTEP
#define CCER_UH_VH_VL               (uint16_t)(0x1051)
#define CCER_UH_WH_WL               (uint16_t)(0x1501)
#define CCER_VH_WH_WL               (uint16_t)(0x1510)
#define CCER_UH_VH_UL               (uint16_t)(0x1015)
#define CCER_UH_WH_UL               (uint16_t)(0x1105)
#define CCER_VH_WH_VL               (uint16_t)(0x1150)
#define CCER_STEP1_QUASISYNC        CCER_UH_VH_VL | CCER_POLARITY_MIDSTEP
#define CCER_STEP2_QUASISYNC        CCER_UH_WH_WL | CCER_POLARITY_MIDSTEP
#define CCER_STEP3_QUASISYNC        CCER_VH_WH_WL | CCER_POLARITY_MIDSTEP
#define CCER_STEP4_QUASISYNC        CCER_UH_VH_UL | CCER_POLARITY_MIDSTEP
#define CCER_STEP5_QUASISYNC        CCER_UH_WH_UL | CCER_POLARITY_MIDSTEP
#define CCER_STEP6_QUASISYNC        CCER_VH_WH_VL | CCER_POLARITY_MIDSTEP
  </#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" -->	

  <#if MC.DRIVE_MODE == "VM">
#define CCMR1_CW_STEP1_MIDALIGN    	(uint16_t)(0x4868) 
#define CCMR1_CW_STEP2_MIDALIGN   	(uint16_t)(0x4868)  
#define CCMR1_CW_STEP3_MIDALIGN   	(uint16_t)(0x6868)  
#define CCMR1_CW_STEP4_MIDALIGN   	(uint16_t)(0x6848)  
#define CCMR1_CW_STEP5_MIDALIGN  	(uint16_t)(0x6848)  
#define CCMR1_CW_STEP6_MIDALIGN  	(uint16_t)(0x4848)
#define CCMR2_CW_STEP1_MIDALIGN    	(uint16_t)(0x6868)  
#define CCMR2_CW_STEP2_MIDALIGN   	(uint16_t)(0x6848)  
#define CCMR2_CW_STEP3_MIDALIGN   	(uint16_t)(0x6848)  
#define CCMR2_CW_STEP4_MIDALIGN   	(uint16_t)(0x6848)  
#define CCMR2_CW_STEP5_MIDALIGN  	(uint16_t)(0x6868)  
#define CCMR2_CW_STEP6_MIDALIGN  	(uint16_t)(0x6868)

#define CCMR1_CCW_STEP1_MIDALIGN   	(uint16_t)(0x4868)  
#define CCMR1_CCW_STEP2_MIDALIGN   	(uint16_t)(0x6868)  
#define CCMR1_CCW_STEP3_MIDALIGN   	(uint16_t)(0x6848)  
#define CCMR1_CCW_STEP4_MIDALIGN   	(uint16_t)(0x6848)  
#define CCMR1_CCW_STEP5_MIDALIGN  	(uint16_t)(0x4848)  
#define CCMR1_CCW_STEP6_MIDALIGN  	(uint16_t)(0x4868)
#define CCMR2_CCW_STEP1_MIDALIGN   	(uint16_t)(0x6848)  
#define CCMR2_CCW_STEP2_MIDALIGN   	(uint16_t)(0x6848)  
#define CCMR2_CCW_STEP3_MIDALIGN   	(uint16_t)(0x6848)  
#define CCMR2_CCW_STEP4_MIDALIGN   	(uint16_t)(0x6868)  
#define CCMR2_CCW_STEP5_MIDALIGN  	(uint16_t)(0x6868)  
#define CCMR2_CCW_STEP6_MIDALIGN  	(uint16_t)(0x6868)

#define CCMR1_STEP1_HSMOD         	(uint16_t)(0x4868)  
#define CCMR1_STEP2_HSMOD        	(uint16_t)(0x0868)  
#define CCMR1_STEP3_HSMOD  	        (uint16_t)(0x6808)  
#define CCMR1_STEP4_HSMOD  	        (uint16_t)(0x6848)  
#define CCMR1_STEP5_HSMOD  	        (uint16_t)(0x0848)  
#define CCMR1_STEP6_HSMOD  	        (uint16_t)(0x4808)
#define CCMR2_STEP1_HSMOD         	(uint16_t)(0x6808)  
#define CCMR2_STEP2_HSMOD        	(uint16_t)(0x6848)  
#define CCMR2_STEP3_HSMOD  	        (uint16_t)(0x6848)  
#define CCMR2_STEP4_HSMOD  	        (uint16_t)(0x6808)  
#define CCMR2_STEP5_HSMOD  	        (uint16_t)(0x6868)  
#define CCMR2_STEP6_HSMOD  	        (uint16_t)(0x6868)

#define CCMR1_STEP1_LSMOD        	(uint16_t)(0x7858)  
#define CCMR1_STEP2_LSMOD        	(uint16_t)(0x0858)  
#define CCMR1_STEP3_LSMOD        	(uint16_t)(0x5808)   
#define CCMR1_STEP4_LSMOD        	(uint16_t)(0x5878)  
#define CCMR1_STEP5_LSMOD        	(uint16_t)(0x0878)  
#define CCMR1_STEP6_LSMOD        	(uint16_t)(0x7808)  
#define CCMR2_STEP1_LSMOD        	(uint16_t)(0x6808)  
#define CCMR2_STEP2_LSMOD        	(uint16_t)(0x6878)  
#define CCMR2_STEP3_LSMOD        	(uint16_t)(0x6878)   
#define CCMR2_STEP4_LSMOD        	(uint16_t)(0x6808)  
#define CCMR2_STEP5_LSMOD        	(uint16_t)(0x6858)  
#define CCMR2_STEP6_LSMOD        	(uint16_t)(0x6858)   
  <#else>
#define CCMR1_CW_STEP1_MIDALIGN    	(uint16_t)(0xC8E8) 
#define CCMR1_CW_STEP2_MIDALIGN   	(uint16_t)(0xC8E8)  
#define CCMR1_CW_STEP3_MIDALIGN   	(uint16_t)(0xE8E8)  
#define CCMR1_CW_STEP4_MIDALIGN   	(uint16_t)(0xE8C8)  
#define CCMR1_CW_STEP5_MIDALIGN  	(uint16_t)(0xE8C8)  
#define CCMR1_CW_STEP6_MIDALIGN  	(uint16_t)(0xC8C8)
#define CCMR2_CW_STEP1_MIDALIGN    	(uint16_t)(0x68E8)  
#define CCMR2_CW_STEP2_MIDALIGN   	(uint16_t)(0x68C8)  
#define CCMR2_CW_STEP3_MIDALIGN   	(uint16_t)(0x68C8)  
#define CCMR2_CW_STEP4_MIDALIGN   	(uint16_t)(0x68C8)  
#define CCMR2_CW_STEP5_MIDALIGN  	(uint16_t)(0x68E8)  
#define CCMR2_CW_STEP6_MIDALIGN  	(uint16_t)(0x68E8)

#define CCMR1_CCW_STEP1_MIDALIGN   	(uint16_t)(0xC8E8)  
#define CCMR1_CCW_STEP2_MIDALIGN   	(uint16_t)(0xE8E8)  
#define CCMR1_CCW_STEP3_MIDALIGN   	(uint16_t)(0xE8C8)  
#define CCMR1_CCW_STEP4_MIDALIGN   	(uint16_t)(0xE8C8)  
#define CCMR1_CCW_STEP5_MIDALIGN  	(uint16_t)(0xC8C8)  
#define CCMR1_CCW_STEP6_MIDALIGN  	(uint16_t)(0xC8E8)
#define CCMR2_CCW_STEP1_MIDALIGN   	(uint16_t)(0x68C8)  
#define CCMR2_CCW_STEP2_MIDALIGN   	(uint16_t)(0x68C8)  
#define CCMR2_CCW_STEP3_MIDALIGN   	(uint16_t)(0x68C8)  
#define CCMR2_CCW_STEP4_MIDALIGN   	(uint16_t)(0x68E8)  
#define CCMR2_CCW_STEP5_MIDALIGN  	(uint16_t)(0x68E8)  
#define CCMR2_CCW_STEP6_MIDALIGN  	(uint16_t)(0x68E8)

#define CCMR1_STEP1_HSMOD         	(uint16_t)(0xC8E8)  
#define CCMR1_STEP2_HSMOD        	(uint16_t)(0x88E8)  
#define CCMR1_STEP3_HSMOD  	        (uint16_t)(0xE888)  
#define CCMR1_STEP4_HSMOD  	        (uint16_t)(0xE8C8)  
#define CCMR1_STEP5_HSMOD  	        (uint16_t)(0x88C8)  
#define CCMR1_STEP6_HSMOD  	        (uint16_t)(0xC888)
#define CCMR2_STEP1_HSMOD         	(uint16_t)(0x6888)  
#define CCMR2_STEP2_HSMOD  	        (uint16_t)(0x68C8)  
#define CCMR2_STEP3_HSMOD  	        (uint16_t)(0x68C8)  
#define CCMR2_STEP4_HSMOD  	        (uint16_t)(0x6888)  
#define CCMR2_STEP5_HSMOD  	        (uint16_t)(0x68E8)  
#define CCMR2_STEP6_HSMOD  	        (uint16_t)(0x68E8)

#define CCMR1_STEP1_LSMOD        	(uint16_t)(0xF8D8)  
#define CCMR1_STEP2_LSMOD        	(uint16_t)(0x88D8)  
#define CCMR1_STEP3_LSMOD        	(uint16_t)(0xD888)   
#define CCMR1_STEP4_LSMOD        	(uint16_t)(0xD8F8)  
#define CCMR1_STEP5_LSMOD        	(uint16_t)(0x88F8)  
#define CCMR1_STEP6_LSMOD        	(uint16_t)(0xF888)  
#define CCMR2_STEP1_LSMOD        	(uint16_t)(0x6888)  
#define CCMR2_STEP2_LSMOD        	(uint16_t)(0x68F8)  
#define CCMR2_STEP3_LSMOD        	(uint16_t)(0x68F8)   
#define CCMR2_STEP4_LSMOD        	(uint16_t)(0x6888)  
#define CCMR2_STEP5_LSMOD        	(uint16_t)(0x68D8)  
#define CCMR2_STEP6_LSMOD        	(uint16_t)(0x68D8)  
  </#if><#-- MC.DRIVE_MODE == "VM" -->

/**
  * @brief  Current sensor parameters Motor 1 - single shunt phase shift
  */
const PWMC_Params_t PWMC_ParamsM1 =
{
/* PWM generation parameters --------------------------------------------------*/
  .TIMx              = ${_last_word(MC.M1_PWM_TIMER_SELECTION)},
  <#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO">
/* PWM Driving signals initialization ----------------------------------------*/
  .pwm_en_u_port     = M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin      = M1_PWM_EN_U_Pin,
  .pwm_en_v_port     = M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin      = M1_PWM_EN_V_Pin,
  .pwm_en_w_port     = M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin      = M1_PWM_EN_W_Pin,
  </#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" -->
};

/**
  * @brief  PWM timer registers Motor 1
  */
  <#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO">
PWMC_TimerCfg_t ThreePwm_TimerCfgM1 =
  <#else>
PWMC_TimerCfg_t SixPwm_TimerCfgM1 =
  </#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" -->
{
  .CCER_cfg = {
                CCER_STEP14, 
                CCER_STEP25, 
                CCER_STEP36,
                CCER_STEP14, 
                CCER_STEP25, 
                CCER_STEP36,
              },
  .CCER_Align_cfg = CCER_MIDSTEP,
  .CCMR1_Standard_cfg = {
                          CCMR1_STEP1_HSMOD, 
                          CCMR1_STEP2_HSMOD, 
                          CCMR1_STEP3_HSMOD,
                          CCMR1_STEP4_HSMOD, 
                          CCMR1_STEP5_HSMOD, 
                          CCMR1_STEP6_HSMOD,
                        },
  .CCMR2_Standard_cfg = {
                          CCMR2_STEP1_HSMOD, 
                          CCMR2_STEP2_HSMOD, 
                          CCMR2_STEP3_HSMOD,
                          CCMR2_STEP4_HSMOD, 
                          CCMR2_STEP5_HSMOD, 
                          CCMR2_STEP6_HSMOD,
                        },
  .CCMR1_CW_Align_cfg = {
                          CCMR1_CW_STEP2_MIDALIGN, 
                          CCMR1_CW_STEP3_MIDALIGN, 
                          CCMR1_CW_STEP4_MIDALIGN,    
                          CCMR1_CW_STEP5_MIDALIGN, 
                          CCMR1_CW_STEP6_MIDALIGN, 
                          CCMR1_CW_STEP1_MIDALIGN,                         
                        },
  .CCMR2_CW_Align_cfg = {
                          CCMR2_CW_STEP2_MIDALIGN, 
                          CCMR2_CW_STEP3_MIDALIGN, 
                          CCMR2_CW_STEP4_MIDALIGN,
                          CCMR2_CW_STEP5_MIDALIGN, 
                          CCMR2_CW_STEP6_MIDALIGN, 
                          CCMR2_CW_STEP1_MIDALIGN,                         
                        },                     
  .CCMR1_CCW_Align_cfg = {
                          CCMR1_CCW_STEP6_MIDALIGN, 
                          CCMR1_CCW_STEP1_MIDALIGN, 
                          CCMR1_CCW_STEP2_MIDALIGN,    
                          CCMR1_CCW_STEP3_MIDALIGN, 
                          CCMR1_CCW_STEP4_MIDALIGN, 
                          CCMR1_CCW_STEP5_MIDALIGN,                         
                        },
  .CCMR2_CCW_Align_cfg = {
                          CCMR2_CCW_STEP6_MIDALIGN, 
                          CCMR2_CCW_STEP1_MIDALIGN, 
                          CCMR2_CCW_STEP2_MIDALIGN,
                          CCMR2_CCW_STEP3_MIDALIGN, 
                          CCMR2_CCW_STEP4_MIDALIGN, 
                          CCMR2_CCW_STEP5_MIDALIGN,                         
                        },  
  .CCMR1_LSMod_cfg = {
                          CCMR1_STEP1_LSMOD, 
                          CCMR1_STEP2_LSMOD, 
                          CCMR1_STEP3_LSMOD,
                          CCMR1_STEP4_LSMOD, 
                          CCMR1_STEP5_LSMOD, 
                          CCMR1_STEP6_LSMOD,
                         },
  .CCMR2_LSMod_cfg = {
                          CCMR2_STEP1_LSMOD, 
                          CCMR2_STEP2_LSMOD, 
                          CCMR2_STEP3_LSMOD,
                          CCMR2_STEP4_LSMOD, 
                          CCMR2_STEP5_LSMOD, 
                          CCMR2_STEP6_LSMOD,
                         },
  <#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER">
  .CCER_QuasiSynch_cfg = {
                          CCER_STEP1_QUASISYNC, 
                          CCER_STEP2_QUASISYNC,
                          CCER_STEP3_QUASISYNC,
                          CCER_STEP4_QUASISYNC,
                          CCER_STEP5_QUASISYNC,
                          CCER_STEP6_QUASISYNC,
                        },
  </#if><#-- MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO" -->
};
  
<#if MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
/**
  * @brief  Current sensor parameters Motor 1 - single shunt phase shift
  */
const Bemf_ADC_Params_t Bemf_ADC_ParamsM1 =
{
  .LfTim                  = ${_last_word(MC.LF_TIMER_SELECTION)},
  .LfTimerChannel         = LL_TIM_CHANNEL_CH1, /*!< Channel of the LF timer used for speed measurement */ 
  <#if MC.BEMF_DIVIDER_AVAILABLE>
  .gpio_divider_available = true,               /*!< Availability of the GPIO port enabling the bemf resistor divider */
  .bemf_divider_port      = M1_BEMF_DIVIDER_GPIO_Port, /*!< GPIO port of OnSensing divider enabler */
  .bemf_divider_pin       = M1_BEMF_DIVIDER_Pin,
  <#else><#-- MC.BEMF_DIVIDER_AVAILABLE == false -->
  .gpio_divider_available = false,              /*!< Availability of the GPIO port enabling the bemf resistor divider */
  </#if><#-- MC.BEMF_DIVIDER_AVAILABLE -->
  /*!< Pointer to the ADC */
  <#if !CondFamily_STM32F0 && !CondFamily_STM32C0 && !CondFamily_STM32G0>
  .pAdc                   = {${MC.PHASE_W_BEMF_ADC}, ${MC.PHASE_V_BEMF_ADC}, ${MC.PHASE_U_BEMF_ADC}, ${MC.PHASE_W_BEMF_ADC}, ${MC.PHASE_V_BEMF_ADC}, ${MC.PHASE_U_BEMF_ADC}},
   </#if><#-- CondFamily_STM32F0 CondFamily_STM32C0 CondFamily_STM32G0 CondFamily_STM32G4 -->
  .AdcChannel             = {MC_${MC.PHASE_W_BEMF_CHANNEL}, MC_${MC.PHASE_V_BEMF_CHANNEL}, MC_${MC.PHASE_U_BEMF_CHANNEL}, MC_${MC.PHASE_W_BEMF_CHANNEL}, MC_${MC.PHASE_V_BEMF_CHANNEL}, MC_${MC.PHASE_U_BEMF_CHANNEL}},   
};
</#if><#-- MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->

<#if MC.DRIVE_MODE == "CM">
const CurrentRef_Params_t CurrentRef_ParamsM1 =
{
  .TIMx            = ${_last_word(MC.CURR_REF_TIMER_SELECTION)}, /*!< It contains the pointer to the timer
                       used for current reference PWM generation. */
  .RefTimerChannel = LL_TIM_CHANNEL_CH1,
};
</#if><#-- MC.DRIVE_MODE == "CM" -->

ScaleParams_t scaleParams_M1 =
{
 .voltage = (1000 * PWM_PERIOD_CYCLES / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V)),
   <#if MC.DRIVE_MODE == "CM">
 .current = PWM_PERIOD_CYCLES_REF * CURRENT_CONV_FACTOR,
   </#if><#-- MC.DRIVE_MODE == "CM" --> 
 .frequency = ((PWM_PERIOD_CYCLES * TIM_CLOCK_DIVIDER ) / (LF_TIMER_PSC + 1)),
};
<#if MC.ESC_ENABLE>
const ESC_Params_t ESC_ParamsM1 =
{
  .Command_TIM        = TIM2,
  .Motor_TIM          = TIM1,
  .ARMING_TIME        = 200,
  .PWM_TURNOFF_MAX    = 500,
  .TURNOFF_TIME_MAX   = 500,
  .Ton_max            = ESC_TON_MAX,               /*!<  Maximum ton value for PWM (by default is 1800 us) */
  .Ton_min            = ESC_TON_MIN,               /*!<  Minimum ton value for PWM (by default is 1080 us) */ 
  .Ton_arming         = ESC_TON_ARMING,            /*!<  Minimum value to start the arming of PWM */ 
  .delta_Ton_max      = ESC_TON_MAX - ESC_TON_MIN,
  .speed_max_valueRPM = MOTOR_MAX_SPEED_RPM,       /*!< Maximum value for speed reference from Workbench */
  .speed_min_valueRPM = 1000,                      /*!< Set the minimum value for speed reference */
  .motor              = M1,
};
</#if><#-- MC.ESC_ENABLE -->

<#if MC.DRIVE_NUMBER != "1">
ScaleParams_t scaleParams_M2 =
{
 .voltage = (1000 * PWM_PERIOD_CYCLES / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V)),
   <#if MC.DRIVE_MODE == "CM">
 .current = PWM_PERIOD_CYCLES_REF * CURRENT_CONV_FACTOR,
   </#if><#-- MC.DRIVE_MODE == "CM" --> 
 .frequency = ((PWM_PERIOD_CYCLES * TIM_CLOCK_DIVIDER ) / (LF_TIMER_PSC + 1)),
};
</#if><#-- MC.DRIVE_NUMBER > 1 -->
/* USER CODE BEGIN Additional parameters */


/* USER CODE END Additional parameters */  

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

