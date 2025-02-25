<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">

<#-- Configuring GPIOs for Testing Environment -->
<#-- <#function _first_word text sep="-"><#return text?split(sep)?first></#function> -->

  <#assign NucleoCond = MC.BOARD?contains("NUCLEO") || 
                        McuName?starts_with("STM32F030R8") ||
                        McuName?starts_with("STM32F072RB") ||
                        McuName?starts_with("STM32G071RB") ||
                        McuName?starts_with("STM32G431RB") ||
                        McuName?starts_with("STM32F103RB") ||
                        McuName?starts_with("STM32F302R8") || 
                        McuName?starts_with("STM32F303RE") ||
                        McuName?starts_with("STM32F401RE") ||
                        McuName?starts_with("STM32F446RE") ||
                        McuName?starts_with("STM32F746ZG") ||
                        McuName?starts_with("STM32L452RE") ||
                        McuName?starts_with("STM32L476RG")
                        >
  <#assign  STM32072B_EVALCond = MC.BOARD == "STM32072B-EVAL" || McuName?starts_with("STM32F072VB")>
  <#assign   STM3210E_EVALCond = MC.BOARD == "STM3210E-EVAL" || McuName?starts_with("STM32F103ZG")>
  <#assign  STM32303E_EVALCond = MC.BOARD == "STM32303E-EVAL" || McuName?starts_with("STM32F303VE")>
  <#assign  STM32446E_EVALCond = MC.BOARD == "STM32446E-EVAL" || McuName?starts_with("STM32F446ZE")>
  <#assign   STM3240G_EVALCond = MC.BOARD == "STM3240G-EVAL" || McuName?starts_with("STM32F407IG")>
  <#assign   STM3241G_EVALCond = MC.BOARD == "STM3241G-EVAL" || McuName?starts_with("STM32F417IG")>
  <#assign STEVAL_IHM039V1Cond = McuName?starts_with("STM32F415ZG")>
  <#assign STM32G081B_EVALCond = MC.BOARD == "STM32G081B-EVAL" || McuName?starts_with("STM32G081RBT")>
  <#assign STM32G474E_EVALCond = McuName?starts_with("STM32G474QE")>
  <#assign STM32L476G_EVALCond = MC.BOARD == "STM32L476G-EVAL" || McuName?starts_with("STM32L476ZG")>
  <#assign STM32F769I_EVALCond = MC.BOARD == "STM32F769I-EVAL" || McuName?starts_with("STM32F769NI")> 

  <#-- 2 Pins need to be configured to drive the WELL board 
       CS stands for Current sensing - Connected to Motor connector DissipativeBrake
       SF stands for Speed feedback  - Connected to Motor connector ICL or MC_NTC (depending on the board)
       PM stands for Performance Measurement - Connected to Motor connector PFC PWM
	   BoardEnable stands for board detection in Automatic test - Connected to Motor connector PFC SYNC
    The connection with Morpho connector is done trough CN10-2 and CN10-4 (from IHM07 schematic)
  -->
  <#if NucleoCond == true >
    <#assign CS_Port = "GPIOC">
    <#assign CS_Pin  = "6">
    <#assign SF_Port = "GPIOC">
    <#assign SF_Pin  = "8">
    <#assign PM_Port = "GPIOC">
    <#assign PM_Pin  = "9">
    <#if McuName?starts_with("STM32G431RB")  >
      <#assign BoardEnable_Port  = "GPIOC">
      <#assign BoardEnable_Pin  = "7"> 
    <#elseif McuName?starts_with("STM32G071RB")>
      <#assign BoardEnable_Port  = "GPIOC">
      <#assign BoardEnable_Pin  = "1"> 
    <#else>
      <#assign BoardEnable_Port  = "GPIOC">
      <#assign BoardEnable_Pin  = "5">
    </#if> 
  <#elseif STM32072B_EVALCond == true>
      <#assign CS_Port = "GPIOB">
      <#assign CS_Pin  = "11">
      <#assign SF_Port = "GPIOE">
      <#assign SF_Pin  = "7">
      <#assign PM_Port = "GPIOC">
      <#assign PM_Pin  = "9">    
      <#assign BoardEnable_Port  = "GPIOC">
      <#assign BoardEnable_Pin  = "8">	   	  
  <#elseif STM3210E_EVALCond == true>
      <#assign CS_Port = "GPIOA">
      <#assign CS_Pin  = "3">
      <#assign SF_Port = "GPIOB">
      <#assign SF_Pin  = "12">
      <#assign PM_Port = "GPIOB">
      <#assign PM_Pin  = "5">   
      <#assign BoardEnable_Port  = "GPIOD">
      <#assign BoardEnable_Pin  = "2">	  	  
  <#elseif STM32303E_EVALCond == true>
    <#assign CS_Port = "GPIOE">
    <#assign CS_Pin  = "5">
    <#assign SF_Port = "GPIOE">
    <#assign SF_Pin  = "4">
    <#assign PM_Port = "GPIOE">
    <#assign PM_Pin  = "3">                
    <#assign BoardEnable_Port  = "GPIOE">
    <#assign BoardEnable_Pin  = "2">
  <#if MC.DRIVE_NUMBER != "1" && MC.M1_ICL_ENABLED == false && MC.M1_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE"> 
      <#assign CS_PortM2 = "GPIOF">
      <#assign CS_PinM2  = "10">
      <#assign SF_PortM2 = "GPIOD">
      <#assign SF_PinM2  = "15">
    </#if> 
  <#elseif STEVAL_IHM039V1Cond == true>
    <#assign CS_Port = "GPIOB">
    <#assign CS_Pin  = "6">
    <#assign SF_Port = "GPIOB">
    <#assign SF_Pin  = "7">
    <#assign PM_Port = "GPIOB">
    <#assign PM_Pin  = "5"> 
    <#assign BoardEnable_Port  = "GPIOB">
    <#assign BoardEnable_Pin  = "4">	
    <#if MC.DRIVE_NUMBER != "1" && MC.M1_ICL_ENABLED == false && MC.M1_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE"> 
      <#assign CS_PortM2 = "GPIOB">
      <#assign CS_PinM2  = "8">
      <#assign SF_PortM2 = "GPIOB">
      <#assign SF_PinM2  = "9">
    </#if>    
  <#elseif STM3240G_EVALCond == true>
      <#assign CS_Port = "GPIOC">
      <#assign CS_Pin  = "8">
      <#assign SF_Port = "GPIOH">
      <#assign SF_Pin  = "8">
      <#assign PM_Port = "GPIOH">
      <#assign PM_Pin  = "12">   
      <#assign BoardEnable_Port  = "GPIOH">
      <#assign BoardEnable_Pin  = "10">		  
  <#elseif STM3241G_EVALCond == true>
      <#assign CS_Port = "GPIOC">
      <#assign CS_Pin  = "8">
      <#assign SF_Port = "GPIOH">
      <#assign SF_Pin  = "8">
      <#assign PM_Port = "GPIOH">
      <#assign PM_Pin  = "12"> 
      <#assign BoardEnable_Port  = "GPIOH">
      <#assign BoardEnable_Pin  = "10">
  <#elseif STM32446E_EVALCond == true>
      <#assign CS_Port = "GPIOD">
      <#assign CS_Pin  = "3">
      <#assign SF_Port = "GPIOG">
      <#assign SF_Pin  = "6">
      <#assign PM_Port = "GPIOA">
      <#assign PM_Pin  = "11">       
      <#assign BoardEnable_Port  = "GPIOA">
      <#assign BoardEnable_Pin  = "8">
  <#elseif STM32F769I_EVALCond == true>
      <#assign CS_Port = "GPIOH">
      <#assign CS_Pin  = "6">
      <#assign SF_Port = "GPIOG">
      <#assign SF_Pin  = "11">
      <#assign PM_Port = "GPIOA">
      <#assign PM_Pin  = "11">             
      <#assign BoardEnable_Port  = "GPIOA">
      <#assign BoardEnable_Pin  = "12">
  <#elseif STM32L476G_EVALCond == true>
      <#assign CS_Port = "GPIOB">
      <#assign CS_Pin  = "2">
      <#assign SF_Port = "GPIOG">
      <#assign SF_Pin  = "6">
      <#assign PM_Port = "GPIOF">
      <#assign PM_Pin  = "10">                   
      <#assign BoardEnable_Port  = "GPIOF">
      <#assign BoardEnable_Pin  = "9">
  <#elseif STM32G081B_EVALCond == true>
      <#assign CS_Port = "GPIOB">
      <#assign CS_Pin  = "15">
      <#assign SF_Port = "GPIOB">
      <#assign SF_Pin  = "9">      
      <#assign PM_Port = "GPIOB">
      <#assign PM_Pin  = "1"> 
      <#assign BoardEnable_Port  = "GPIOD">
      <#assign BoardEnable_Pin  = "0">
  <#elseif STM32G474E_EVALCond == true>
      <#assign CS_Port = "GPIOE">
      <#assign CS_Pin  = "5">
      <#assign SF_Port = "GPIOE">
      <#assign SF_Pin  = "4">      
      <#assign PM_Port = "GPIOE">
      <#assign PM_Pin  = "3">
      <#assign BoardEnable_Port  = "GPIOE">
      <#assign BoardEnable_Pin  = "2">	  
  <#else>
    #error "Cannot generate Test Env code with this control board"
    <#assign BoardEnable_Port  = "">
    <#assign BoardEnable_Pin  = "">	 
    <#assign CS_Port = "">
    <#assign CS_Pin  = "">
    <#assign SF_Port = "">
    <#assign SF_Pin  = "">
    <#assign PM_Port = "">
    <#assign PM_Pin  = "">
    <#assign CS_PortM2 = "">
    <#assign CS_PinM2  = "">
    <#assign SF_PortM2 = "">
    <#assign SF_PinM2  = "">
  </#if>
  /**
  ******************************************************************************
  * @file    mc_testenv.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Definitions used for ST MCSDK internal automatic test system.
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MCTESTENV_H
#define MCTESTENV_H
/* ************************************************************************************** */
  /* ***                               Test Environment Setup                           *** */
  /* ************************************************************************************** */
  /* *** MC.BOARD == ${MC.BOARD} -- McuName == ${McuName} *** */

static inline void mc_testenv_init() //cstat !MISRAC2012-Rule-8.2_a !FUNC-unprototyped-all
#ifdef MC_HAL_IS_USED
  {
    GPIO_InitTypeDef GPIO_InitStruct;
  
    /* Enabling clock for Current Sensing topology control GPIO Port */
    if (__HAL_RCC_${CS_Port}_IS_CLK_DISABLED())
    {
      __HAL_RCC_${CS_Port}_CLK_ENABLE();
    }
  
    /* Enabling clock for Speed Feedback technology control GPIO Port */
    if (__HAL_RCC_${SF_Port}_IS_CLK_DISABLED())
    {
      __HAL_RCC_${SF_Port}_CLK_ENABLE();
    }

    /* Enabling clock for the performance measurement GPIO Port */
    if (__HAL_RCC_${PM_Port}_IS_CLK_DISABLED())
    {
      __HAL_RCC_${PM_Port}_CLK_ENABLE();
    }
    
    <#if (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')>
    /* Current Reading Topology: Single Shunt configuration */
    HAL_GPIO_WritePin( ${CS_Port}, GPIO_PIN_${CS_Pin}, GPIO_PIN_RESET );
    <#else>
    /* Current Reading Topology: Three Shunts or ICS configuration */
    HAL_GPIO_WritePin( ${CS_Port}, GPIO_PIN_${CS_Pin}, GPIO_PIN_SET );
    </#if>
  
    <#if (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")>
    /* Speed Feedback Technology: Encoder configuration */
    HAL_GPIO_WritePin( ${SF_Port}, GPIO_PIN_${SF_Pin}, GPIO_PIN_RESET );
    <#else><#-- (MC.M1_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M1_SPEED_SENSOR != "QUAD_ENCODER_Z") -->
    /* Speed Feedback Technology: Hall sensor or sensorless configuration */
    HAL_GPIO_WritePin( ${SF_Port}, GPIO_PIN_${SF_Pin}, GPIO_PIN_SET );
    </#if><#-- (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") -->

    /* Performance Measurement: initialization */
    HAL_GPIO_WritePin( ${PM_Port}, GPIO_PIN_${PM_Pin}, GPIO_PIN_RESET );

    /*  Test Board Enable */
    HAL_GPIO_WritePin( ${BoardEnable_Port}, GPIO_PIN_${BoardEnable_Pin}, GPIO_PIN_SET );

    /* Configuring Current Sensing topology control GPIO port and pin */  
    GPIO_InitStruct.Pin = GPIO_PIN_${CS_Pin};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ${CS_Port}, &GPIO_InitStruct );
    
    /* Configuring Speed Feedback technology control GPIO port and pin */  
    GPIO_InitStruct.Pin = GPIO_PIN_${SF_Pin};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ${SF_Port}, &GPIO_InitStruct );

    /* Configuring the Performance Measurement GPIO port and pin */  
    GPIO_InitStruct.Pin = GPIO_PIN_${PM_Pin};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ${PM_Port}, &GPIO_InitStruct );

    /* Configuring the Test Board Enable GPIO port and pin */  
    GPIO_InitStruct.Pin = GPIO_PIN_${BoardEnable_Pin};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ${BoardEnable_Port}, &GPIO_InitStruct );

    <#if MC.DRIVE_NUMBER != "1" && MC.M1_ICL_ENABLED == false && MC.M1_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE">
    /* Enabling clock for Current Sensing topology control GPIO Port, Motor 2 */
    if (__HAL_RCC_${CS_PortM2}_IS_CLK_DISABLED())
    {
      __HAL_RCC_${CS_PortM2}_CLK_ENABLE();
    }
  
    /* Enabling clock for Speed Feedback technology control GPIO Port, Motor 2 */
    if (__HAL_RCC_${SF_PortM2}_IS_CLK_DISABLED())
    {
      __HAL_RCC_${SF_PortM2}_CLK_ENABLE();
    }
  
      <#if MC.M2_CURRENT_SENSING_TOPO == "SINGLE_SHUNT2">
    /* Current Reading Topology: Single Shunt configuration, Motor 2 */
    HAL_GPIO_WritePin( ${CS_PortM2}, GPIO_PIN_${CS_PinM2}, GPIO_PIN_RESET );
      <#else>
    /* Current Reading Topology: Three Shunts or ICS configuration, Motor 2 */
    HAL_GPIO_WritePin( ${CS_PortM2}, GPIO_PIN_${CS_PinM2}, GPIO_PIN_SET );
      </#if>
  
      <#if (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")>
    /* Speed Feedback Technology: Encoder configuration, Motor 2 */
    HAL_GPIO_WritePin( ${SF_PortM2}, GPIO_PIN_${SF_PinM2}, GPIO_PIN_RESET );
      <#else><#-- (MC.M2_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M2_SPEED_SENSOR != "QUAD_ENCODER_Z") -->
    /* Speed Feedback Technology: Hall sensor or sensorless configuration, Motor 2 */
    HAL_GPIO_WritePin( ${SF_PortM2}, GPIO_PIN_${SF_PinM2}, GPIO_PIN_SET );
      </#if><#-- (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z") -->

    /* Configuring Current Sensing topology control GPIO port and pin */  
    GPIO_InitStruct.Pin = GPIO_PIN_${CS_PinM2};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ${CS_PortM2}, &GPIO_InitStruct );
    
    /* Configuring Speed Feedback technology control GPIO port and pin */  
    GPIO_InitStruct.Pin = GPIO_PIN_${SF_PinM2};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ${SF_PortM2}, &GPIO_InitStruct );
    </#if>
  }
#else /* MC_HAL_IS_USED */
  {
    LL_GPIO_InitTypeDef GPIO_TestEnvInitStruct;
    
    /* Enabling clock for Current Sensing topology control GPIO Port */
    if (  LL_AHB1_GRP1_IsEnabledClock( LL_AHB1_GRP1_PERIPH_${CS_Port} ) ) 
    {
      /* GPIOC block clock is not enabled. Enable it */
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_${CS_Port});
    }
    
    /* Enabling clock for Speed Feedback technology control GPIO Port */
    if (  LL_AHB1_GRP1_IsEnabledClock( LL_AHB1_GRP1_PERIPH_${SF_Port} ) ) 
    {
      /* GPIOC block clock is not enabled. Enable it */
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_${SF_Port});
    }

    /* Enabling clock for the performance measurement GPIO Port */
    if (  LL_AHB1_GRP1_IsEnabledClock( LL_AHB1_GRP1_PERIPH_${PM_Port} ) ) 
    {
      /* GPIOC block clock is not enabled. Enable it */
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_${PM_Port});
    }
        
    <#if (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')>
    /* Current Reading Topology: Single Shunt configuration */
    LL_GPIO_ResetOutputPin( ${CS_Port}, LL_GPIO_PIN_${CS_Pin} );
    <#else>
    /* Current Reading Topology: Three Shunts or ICS configuration */
    LL_GPIO_SetOutputPin( ${CS_Port}, LL_GPIO_PIN_${CS_Pin} );
    </#if>
    
    <#if (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")>
    /* Speed Feedback Technology: Encoder configuration */
    LL_GPIO_ResetOutputPin( ${SF_Port}, LL_GPIO_PIN_${SF_Pin} );
    <#else><#-- (MC.M1_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M1_SPEED_SENSOR != "QUAD_ENCODER_Z") -->
    /* Speed Feedback Technology: Hall sensor or sensorless configuration */
    LL_GPIO_SetOutputPin( ${SF_Port}, LL_GPIO_PIN_${SF_Pin} );
    </#if><#-- (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") -->

    /* Performance Measurement: initialization */
    LL_GPIO_ResetOutputPin( ${PM_Port}, LL_GPIO_PIN_${PM_Pin} );
    
    /* Configuring Current Sensing topology control GPIO port and pin */  
    GPIO_TestEnvInitStruct.Pin = LL_GPIO_PIN_${CS_Pin};
    GPIO_TestEnvInitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_TestEnvInitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_TestEnvInitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_TestEnvInitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init( ${CS_Port}, &GPIO_TestEnvInitStruct );

    /* Configuring Speed Feedback technology control GPIO port and pin */  
    GPIO_TestEnvInitStruct.Pin = LL_GPIO_PIN_${SF_Pin};
    GPIO_TestEnvInitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_TestEnvInitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_TestEnvInitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_TestEnvInitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init( ${SF_Port}, &GPIO_TestEnvInitStruct );

    /* Configuring the Performance Measurement GPIO port and pin */  
    GPIO_TestEnvInitStruct.Pin = LL_GPIO_PIN_${PM_Pin};
    GPIO_TestEnvInitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_TestEnvInitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_TestEnvInitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_TestEnvInitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init( ${PM_Port}, &GPIO_TestEnvInitStruct );

    <#if MC.DRIVE_NUMBER != "1" && MC.M1_ICL_ENABLED == false && MC.M1_ON_OVER_VOLTAGE != "TURN_ON_R_BRAKE">
    /* Enabling clock for Current Sensing topology control GPIO Port, Motor2 */
    if (  LL_AHB1_GRP1_IsEnabledClock( LL_AHB1_GRP1_PERIPH_${CS_PortM2} ) ) 
    {
      /* GPIOC block clock is not enabled. Enable it */
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_${CS_PortM2});
    }
    
    /* Enabling clock for Speed Feedback technology control GPIO Port, Motor2 */
    if (  LL_AHB1_GRP1_IsEnabledClock( LL_AHB1_GRP1_PERIPH_${SF_PortM2} ) ) 
    {
      /* GPIOC block clock is not enabled. Enable it */
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_${SF_PortM2});
    }
    
      <#if MC.M2_CURRENT_SENSING_TOPO == "SINGLE_SHUNT2">
    /* Current Reading Topology: Single Shunt configuration, Motor2 */
    LL_GPIO_ResetOutputPin( ${CS_PortM2}, LL_GPIO_PIN_${CS_PinM2} );
      <#else>
    /* Current Reading Topology: Three Shunts or ICS configuration, Motor2 */
    LL_GPIO_SetOutputPin( ${CS_PortM2}, LL_GPIO_PIN_${CS_PinM2} );
      </#if>
    
      <#if (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")>
    /* Speed Feedback Technology: Encoder configuration, Motor2 */
    LL_GPIO_ResetOutputPin( ${SF_PortM2}, LL_GPIO_PIN_${SF_PinM2} );
      <#else><#-- (MC.M2_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M2_SPEED_SENSOR != "QUAD_ENCODER_Z") -->
    /* Speed Feedback Technology: Hall sensor or sensorless configuration, Motor2 */
    LL_GPIO_SetOutputPin( ${SF_PortM2}, LL_GPIO_PIN_${SF_PinM2} );
      </#if><#-- (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z") -->
  
    /* Configuring Current Sensing topology control GPIO port and pin, Motor2 */  
    GPIO_TestEnvInitStruct.Pin = LL_GPIO_PIN_${CS_PinM2};
    GPIO_TestEnvInitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_TestEnvInitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_TestEnvInitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_TestEnvInitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init( ${CS_PortM2}, &GPIO_TestEnvInitStruct );

    /* Configuring Speed Feedback technology control GPIO port and pin, Motor2 */  
    GPIO_TestEnvInitStruct.Pin = LL_GPIO_PIN_${SF_PinM2};
    GPIO_TestEnvInitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_TestEnvInitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_TestEnvInitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_TestEnvInitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init( ${SF_PortM2}, &GPIO_TestEnvInitStruct );
    </#if>
 }
 #endif /* MC_HAL_IS_USED */
 
 static inline void start_perf_measure() //cstat !MISRAC2012-Rule-8.2_a !FUNC-unprototyped-all
 {
   /* Performance Measurement: start measure */
  LL_GPIO_SetOutputPin( ${PM_Port}, LL_GPIO_PIN_${PM_Pin} );
 }
 
 static inline void stop_perf_measure() //cstat !MISRAC2012-Rule-8.2_a !FUNC-unprototyped-all
 {
  LL_GPIO_ResetOutputPin( ${PM_Port}, LL_GPIO_PIN_${PM_Pin} );
 }

#endif /* MCTESTENV_H */
/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
