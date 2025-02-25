<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
 ******************************************************************************
 * @file    oversampling.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions of oversampling 
 *          component
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
  * @ingroup oversampling
  */

/* Includes ------------------------------------------------------------------*/
#include "oversampling.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup oversampling
  *
  * @brief oversampling component 
  *
  * This component is in charge to retrieve the samples that have been transfered from ADC to memory 
  * buffer by the DMA and scaled them in order to be processed by the current controller.
  *  
  * @{
  */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
<#if MC.M1_CURRENT_SENSING_TOPO=="THREE_SHUNT">
static inline int32_t OVERSAMPLING_getOversampledSumCurr(DMA_ADC_Buffer_t* pBuffer, uint8_t rank);
</#if>
static inline int32_t OVERSAMPLING_getOversampledSum(DMA_ADC_Buffer_t* pBuffer, uint8_t rank);

<#if MC.M1_CURRENT_SENSING_TOPO=="THREE_SHUNT" >
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Gets the phase current sum.
  */
inline int32_t OVERSAMPLING_getOversampledSumCurr(DMA_ADC_Buffer_t* pBuffer, uint8_t rank)
{
  int32_t sum = 0;
  uint8_t n;							/* Rank determines starting index in buffer */	
  uint8_t i;
		
  n = rank + (BOARD_SHUNT_SAMPLE_SELECT * OVS_LONGEST_TASK);    
  for (i = 0; i < REGULATION_EXECUTION_RATE; i++)
  {
    sum += (*pBuffer)[n];
    n += OVS_LONGEST_TASK * OVS_COUNT;
  }

  return sum;
}
</#if>

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * Gets the phase voltage sum.
  */
inline int32_t OVERSAMPLING_getOversampledSum(DMA_ADC_Buffer_t* pBuffer, uint8_t rank)
{
  int32_t sum = 0;
  uint8_t n;         /* Rank determines starting index in buffer */	
  uint8_t i;

  n = rank;
  for (i = 0; i < (OVS_COUNT * REGULATION_EXECUTION_RATE); i++)
  {
    sum += (*pBuffer)[n];
    n += OVS_LONGEST_TASK;
  }

  return sum;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Gets the phase voltages and currents values
  *
  * This routine retrieves currents and voltages sampled by ADCs and scaled
  * them.
  *  
  * @param pCurrents_Irst output phase currents data structure address
  * @param pVoltages_Urst output phase voltages data structure address 
  */
void OVERSAMPLING_getMeasurements(Currents_Irst_t* pCurrents_Irst, Voltages_Urst_t* pVoltages_Urst)
{
	Oversampling_t* pOvs = &oversampling;
  DMA_ADC_Buffer_t*		pBuffer;
  int readBuffer = oversampling.readBuffer;
  
  pBuffer = &oversampling.DMABuffer[readBuffer][OVS_ADC_I_R];
  pCurrents_Irst->R = FIXP_MPY(OVERSAMPLING_getOversampledSum${currentSuffix}(pBuffer,OVS_RANK_I_R), pOvs->current_factor, ADC_FIXPFMT);
  
  pBuffer = &oversampling.DMABuffer[readBuffer][OVS_ADC_I_S];
  pCurrents_Irst->S = FIXP_MPY(OVERSAMPLING_getOversampledSum${currentSuffix}(pBuffer,OVS_RANK_I_S), pOvs->current_factor, ADC_FIXPFMT);
  
  pBuffer = &oversampling.DMABuffer[readBuffer][OVS_ADC_I_T];
  pCurrents_Irst->T = FIXP_MPY(OVERSAMPLING_getOversampledSum${currentSuffix}(pBuffer,OVS_RANK_I_T), pOvs->current_factor, ADC_FIXPFMT);
  
  pBuffer = &oversampling.DMABuffer[readBuffer][OVS_ADC_U_R];
  pVoltages_Urst->R = FIXP_MPY(OVERSAMPLING_getOversampledSum(pBuffer,OVS_RANK_U_R), pOvs->voltage_factor, ADC_FIXPFMT);
  
  pBuffer = &oversampling.DMABuffer[readBuffer][OVS_ADC_U_S];
  pVoltages_Urst->S = FIXP_MPY(OVERSAMPLING_getOversampledSum(pBuffer,OVS_RANK_U_S), pOvs->voltage_factor, ADC_FIXPFMT);
  
  pBuffer = &oversampling.DMABuffer[readBuffer][OVS_ADC_U_T];
  pVoltages_Urst->T = FIXP_MPY(OVERSAMPLING_getOversampledSum(pBuffer,OVS_RANK_U_T), pOvs->voltage_factor, ADC_FIXPFMT);

} 

<#if MC.M1_POTENTIOMETER_ENABLE == true>
  <#if (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_U) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_V) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_W)>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Gets potentiometer value
  *    
  * @param  value potentiometer value 
  */
void OVERSAMPLING_getPotentiometerMeasurements( fixp30_t *value)
{
  DMA_ADC_Buffer_t*		pBuffer;
  int readBuffer = oversampling.readBuffer;

  pBuffer = &oversampling.DMABuffer[readBuffer][OVS_ADC_THROTTLE];
  *value = OVERSAMPLING_getOversampledSum(pBuffer,OVS_RANK_THROTTLE);

}
  </#if><#-- (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_U) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_V) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_W) -->
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true -->

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Gets DC bus value
  *    
  * This routine retrieves DC bus voltages sampled by ADC and scaled
  * it.
  *  
  * @param  value DC bus value
  */
void OVERSAMPLING_getVbusMeasurements( fixp30_t *value)
{
<#if MC.M1_BUS_VOLTAGE_READING>
  <#if (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_U) || (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_V) || (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_W)>
  Oversampling_t* pOvs = &oversampling;  
  DMA_ADC_Buffer_t*		pBuffer;
  int readBuffer = oversampling.readBuffer;

  pBuffer = &oversampling.DMABuffer[readBuffer][OVS_ADC_U_DC];
  *value = FIXP_MPY(OVERSAMPLING_getOversampledSum(pBuffer, OVS_RANK_U_DC), pOvs->busvoltage_factor, ADC_FIXPFMT);
  </#if>
<#else>
  *value = FIXP_MPY(FIXP16(NOMINAL_BUS_VOLTAGE_V*VBUS_PARTITIONING_FACTOR/ADC_REFERENCE_VOLTAGE), FIXP30(ADC_BUSVOLTAGE_SCALE/VOLTAGE_SCALE) , ADC_FIXPFMT);
</#if>

}


<#if MC.M1_TEMPERATURE_READING == true>
<#if (MC.M1_TEMP_FDBK_ADC ==  MC.M1_CS_ADC_U) || (MC.M1_TEMP_FDBK_ADC ==  MC.M1_CS_ADC_V) || (MC.M1_TEMP_FDBK_ADC ==  MC.M1_CS_ADC_W)>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Gets temperature value
  *    
  * This routine retrieves temperature sampled by ADC 
  *  
  * @param  value temperature value
  */
void OVERSAMPLING_getTempMeasurements( fixp30_t *value)
{
  DMA_ADC_Buffer_t*		pBuffer;
  int readBuffer = oversampling.readBuffer;

  pBuffer = &oversampling.DMABuffer[readBuffer][OVS_ADC_TEMP];
  *value = OVERSAMPLING_getOversampledSum(pBuffer,OVS_RANK_TEMP) / (OVS_COUNT * REGULATION_EXECUTION_RATE);

}
</#if>
</#if>
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
