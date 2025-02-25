<#ftl strip_whitespace = true>
<#-- Mode containing the Drive Type for early 6-step integration phase. 
     Contains either MC_Init or MC_SixStep -->
<#assign DriveTypeRoot = IPdatas[0].configModelList[0].configs[0].name>
/**
  ******************************************************************************
  * @file    motorcontrol.c 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem initialization functions.
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
  * @ingroup MCInterface
  */
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_interface.h"
#include "mc_tasks.h"

#include "motorcontrol.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCInterface
  * @{
  */

MCI_Handle_t* pMCI[NBR_OF_MOTORS];


/**
 * @brief Initializes and configures the Motor Control Subsystem
 *
 *  This function initializes and configures all the structures and components needed
 * for the Motor Control subsystem required by the Application. It expects that
 * all the peripherals needed for Motor Control purposes are already configured but
 * that their interrupts are not enabled yet. 
 *
 * CubeMX calls this function after all peripherals initializations and 
 * before the NVIC is configured
 */
__weak void MX_MotorControl_Init(void) 
{
<#if !FREERTOS??>
	<#if isHALUsed??>
  /* Reconfigure the SysTick interrupt to fire every 500 us. */
  (void)HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / SYS_TICK_FREQUENCY);
  HAL_NVIC_SetPriority(SysTick_IRQn, uwTickPrio, 0U); 
		<#else>
  LL_RCC_ClocksTypeDef RCC_Clocks;
  
  /* Reconfigure the SysTick interrupt to fire every 500 us. */
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency/SYS_TICK_FREQUENCY);

	</#if>
</#if>

  /* Initialize the Motor Control Subsystem */
  MCboot(pMCI);
  mc_lock_pins();
}

<#if FREERTOS??>
void vPortSetupTimerInterrupt( void )
{
	<#if isHALUsed??>
  /* Reconfigure the SysTick interrupt to fire every 500 us. */
  (void)HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/SYS_TICK_FREQUENCY);
	<#else>
  LL_RCC_ClocksTypeDef RCC_Clocks;
  
  /* Reconfigure the SysTick interrupt to fire every 500 us. */
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency/SYS_TICK_FREQUENCY);

	</#if> 
}
</#if>
/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
