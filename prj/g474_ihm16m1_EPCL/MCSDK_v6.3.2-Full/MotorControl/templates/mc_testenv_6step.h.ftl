<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">

<#-- Configuring GPIOs for Testing Environment -->
<#-- <#function _first_word text sep="-"><#return text?split(sep)?first></#function> -->

  <#assign NucleoCond = McuName?starts_with("STM32F030R8") ||
                        McuName?starts_with("STM32F072RB") ||
                        McuName?starts_with("STM32G071RB")
                        >
  <#assign NucleoCondG4 = McuName?starts_with("STM32G431RB") >							
  <#assign NucleoCondC0 = McuName?starts_with("STM32C031C6") >	
  <#assign STM32072B_EVALCond = (MC.BOARD == "STM32072B-EVAL" || McuName?starts_with("STM32F072VB"))>
  <#assign STM32G081B_EVALCond = (MC.BOARD == "STM32G081B-EVAL" || McuName?starts_with("STM32G081RBT"))>
  <#assign STEVAL_SPIN3202Cond = (MC.BOARD == "STEVAL-SPIN3202")>
  <#assign STEVAL_SPIN3204Cond = (MC.BOARD == "STEVAL-SPIN3204")>
  <#assign STM32G474E_EVALCond = McuName?starts_with("STM32G474QE")>

  <#-- 2 Pins need to be configured to drive the WELL board 
       CS stands for Current sensing - Connected to Motor connector DissipativeBrake
       SF stands for Speed feedback  - Connected to Motor connector ICL or MC_NTC (depending on the board)
       PM stands for Performance Measurement - Connected to Motor connector PFC PWM
	   BoardEnable stands for board detection in Automatic test - Connected to Motor connector PFC SYNC
    The connection with Morpho connector is done trough CN10-2 and CN10-4 (from IHM07 schematic)
  -->
  <#if NucleoCond == true >
      <#assign BoardEnable_Port  = "GPIOC">
      <#assign BoardEnable_Pin  = "6">
  <#elseif NucleoCondG4 == true>
      <#assign BoardEnable_Port  = "GPIOC">
      <#assign BoardEnable_Pin  = "5">	
  <#elseif NucleoCondC0 == true>
      <#assign BoardEnable_Port  = "GPIOB">
      <#assign BoardEnable_Pin  = "8">	
  <#elseif STM32072B_EVALCond == true>
      <#assign BoardEnable_Port  = "GPIOC">
      <#assign BoardEnable_Pin  = "8">	   	  
  <#elseif STM32G081B_EVALCond == true>
      <#assign BoardEnable_Port  = "GPIOD">
      <#assign BoardEnable_Pin  = "0">
  <#elseif STM32G474E_EVALCond == true>
      <#assign BoardEnable_Port  = "GPIOE">
      <#assign BoardEnable_Pin  = "2">	  
  <#elseif STEVAL_SPIN3202Cond == true>
      <#assign BoardEnable_Port  = "GPIOA">
      <#assign BoardEnable_Pin  = "15">	
  <#else>
    #error "Cannot generate Test Env code with this control board"
    <#assign BoardEnable_Port = "">
    <#assign BoardEnable_Pin  = "">
  </#if>
  /**
  ******************************************************************************
  * @file    mc_testenv_6step.h
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
/* ************************************************************************************** */
  /* ***                               Test Environment Setup                           *** */
  /* ************************************************************************************** */
  /* *** MC.BOARD == ${MC.BOARD} -- McuName == ${McuName} *** */

static inline void mc_testenv_init ()
#ifdef MC_HAL_IS_USED
  {
    GPIO_InitTypeDef GPIO_InitStruct;
  
    /* Enabling clock for Board Enable control GPIO Port */
    if (__HAL_RCC_${BoardEnable_Port}_IS_CLK_DISABLED())
    {
      __HAL_RCC_${BoardEnable_Port}_CLK_ENABLE();
    }
  
    /*  Test Board Enable */
    HAL_GPIO_WritePin( ${BoardEnable_Port}, GPIO_PIN_${BoardEnable_Pin}, GPIO_PIN_SET );

    /* Configuring the Test Board Enable GPIO port and pin */  
    GPIO_InitStruct.Pin = GPIO_PIN_${BoardEnable_Pin};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ${BoardEnable_Port}, &GPIO_InitStruct );

  }
#else /* MC_HAL_IS_USED */
  {
    LL_GPIO_InitTypeDef GPIO_TestEnvInitStruct;
    
    /* Enabling clock for Board Enable control GPIO Port */
    if (  LL_AHB1_GRP1_IsEnabledClock( LL_AHB1_GRP1_PERIPH_${BoardEnable_Port} ) ) 
    {
      /* GPIOC block clock is not enabled. Enable it */
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_${BoardEnable_Port});
    }

    LL_GPIO_SetOutputPin( ${BoardEnable_Port}, LL_GPIO_PIN_${BoardEnable_Pin} );    
 
    /* Configuring Current Sensing topology control GPIO port and pin */  
    GPIO_TestEnvInitStruct.Pin = LL_GPIO_PIN_${BoardEnable_Pin};
    GPIO_TestEnvInitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_TestEnvInitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_TestEnvInitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_TestEnvInitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init( ${BoardEnable_Port}, &GPIO_TestEnvInitStruct );

 }
 #endif /* MC_HAL_IS_USED */
 
 static inline void mc_testenv_clear ()
#ifdef MC_HAL_IS_USED
  {
    GPIO_InitTypeDef GPIO_InitStruct;
  
    /* Enabling clock for Board Enable control GPIO Port */
    if (__HAL_RCC_${BoardEnable_Port}_IS_CLK_DISABLED())
    {
      __HAL_RCC_${BoardEnable_Port}_CLK_ENABLE();
    }
  
    /*  Test Board Enable */
    HAL_GPIO_WritePin( ${BoardEnable_Port}, GPIO_PIN_${BoardEnable_Pin}, GPIO_PIN_RESET );

    /* Configuring the Test Board Enable GPIO port and pin */  
    GPIO_InitStruct.Pin = GPIO_PIN_${BoardEnable_Pin};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( ${BoardEnable_Port}, &GPIO_InitStruct );

  }
#else /* MC_HAL_IS_USED */
  {
    LL_GPIO_InitTypeDef GPIO_TestEnvInitStruct;
    
    /* Enabling clock for Board Enable control GPIO Port */
    if (  LL_AHB1_GRP1_IsEnabledClock( LL_AHB1_GRP1_PERIPH_${BoardEnable_Port} ) ) 
    {
      /* GPIOC block clock is not enabled. Enable it */
      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_${BoardEnable_Port});
    }

    LL_GPIO_ResetOutputPin( ${BoardEnable_Port}, LL_GPIO_PIN_${BoardEnable_Pin} );    
 
    /* Configuring Current Sensing topology control GPIO port and pin */  
    GPIO_TestEnvInitStruct.Pin = LL_GPIO_PIN_${BoardEnable_Pin};
    GPIO_TestEnvInitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_TestEnvInitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_TestEnvInitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_TestEnvInitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init( ${BoardEnable_Port}, &GPIO_TestEnvInitStruct );

 }
 #endif /* MC_HAL_IS_USED */
 
/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
