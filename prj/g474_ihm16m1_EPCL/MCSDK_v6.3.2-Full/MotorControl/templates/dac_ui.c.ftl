<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    dac_ui.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the dac component of the Motor Control SDK:
  *           + dac or virtual dac initialization
  *           + dac or virtual dac execution
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
#include "mc_type.h"
#include "dac_ui.h"
#include "register_interface.h"

#define DACOFF (uint32_t)32768

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCUI
  * @{
  */

/**
 * @defgroup DAC_UserInterface DAC User Interface
 *
 * @brief DAC User Interface
 *
 * @todo Complete Documentation. 
 * @{
 */

DAC_Handle_t DAC_Handle;

/**
  * @brief  Hardware and software initialization of the DAC object.
  * @param  pHandle pointer on related component instance.
  */
__weak void DAC_Init(DAC_Handle_t *pHandle)
{
<#if MC.DEBUG_DAC_TIMER_CH1 != "NOT_USED">
#ifdef NULL_PTR_CHECK_DAC_UI
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* CH1 configuration  */
    /* By default send Ia motor 1 */
    (void)RI_GetPtrReg((MC_REG_I_A + 0x1U), (void *)&pHandle->ptrDataCh[DAC_CH1]); //cstat !MISRAC2012-Rule-11.5 
    LL_TIM_CC_EnableChannel(${MC.DEBUG_DAC_TIMER_SELECTION}, LL_TIM_CHANNEL_${MC.DEBUG_DAC_TIMER_CH1});
    LL_TIM_OC_DisablePreload(${MC.DEBUG_DAC_TIMER_SELECTION}, LL_TIM_CHANNEL_${MC.DEBUG_DAC_TIMER_CH1});
#ifdef NULL_PTR_CHECK_DAC_UI
  }
#endif
<#elseif MC.DEBUG_DAC_CH1_EN == true >
#ifdef NULL_PTR_CHECK_DAC_UI
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* Enable DAC Channel1 */
    /* By default send Ia motor 1 */
    (void)RI_GetPtrReg((MC_REG_I_A + 0x1U), (void *)&pHandle->ptrDataCh[DAC_CH1]); //cstat !MISRAC2012-Rule-11.5
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
#ifdef NULL_PTR_CHECK_DAC_UI
  }
  #endif
</#if><#-- MC.DEBUG_DAC_TIMER_CH1 != "NOT_USED" -->
<#if MC.DEBUG_DAC_TIMER_CH2 != "NOT_USED">
#ifdef NULL_PTR_CHECK_DAC_UI
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* CH2 configuration  */
    /* By default send Ib motor 1 */
    (void)RI_GetPtrReg((MC_REG_I_B + 0x1U), (void *)&pHandle->ptrDataCh[DAC_CH2]); //cstat !MISRAC2012-Rule-11.5
    LL_TIM_CC_EnableChannel(${MC.DEBUG_DAC_TIMER_SELECTION}, LL_TIM_CHANNEL_${MC.DEBUG_DAC_TIMER_CH2});
    LL_TIM_OC_DisablePreload(${MC.DEBUG_DAC_TIMER_SELECTION}, LL_TIM_CHANNEL_${MC.DEBUG_DAC_TIMER_CH2});
#ifdef NULL_PTR_CHECK_DAC_UI
  }
#endif
<#elseif MC.DEBUG_DAC_CH2_EN == true >
#ifdef NULL_PTR_CHECK_DAC_UI
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* Enable DAC Channel2 */
    /* By default send Ib motor 1 */
    (void)RI_GetPtrReg((MC_REG_I_B + 0x1U), (void *)&pHandle->ptrDataCh[DAC_CH2]); //cstat !MISRAC2012-Rule-11.5
    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);
#ifdef NULL_PTR_CHECK_DAC_UI
  }
#endif
</#if><#-- MC.DEBUG_DAC_TIMER_CH2 != "NOT_USED" -->

<#if (MC.DEBUG_DAC_TIMER_CH1 != "NOT_USED") || (MC.DEBUG_DAC_TIMER_CH2 != "NOT_USED")>
   /* start Timer */
   LL_TIM_EnableCounter(${MC.DEBUG_DAC_TIMER_SELECTION});
</#if><#-- (MC.DEBUG_DAC_TIMER_CH1 != "NOT_USED") || (MC.DEBUG_DAC_TIMER_CH2 != "NOT_USED") -->
}

/**
  * @brief  This method is used to update the DAC outputs. The selected
  *         variables will be provided in the related output channels.
  * @param  pHandle pointer on related component instance.
  */

<#if MC.DRIVE_NUMBER == "1">
__weak void DAC_Exec(DAC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_DAC_UI
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
  <#if MC.DEBUG_DAC_TIMER_CH1 != "NOT_USED" >
    LL_TIM_OC_SetCompare${MC.DEBUG_DAC_TIMER_CH1}(${MC.DEBUG_DAC_TIMER_SELECTION},
                          (uint16_t)((int16_t)((*((int16_t *)pHandle->ptrDataCh[DAC_CH1]) + 32768) / 32)));

  <#elseif MC.DEBUG_DAC_CH1_EN == true > 
    uint32_t temp1;
    temp1 = DACOFF + (uint32_t)*((uint16_t*) pHandle->ptrDataCh[DAC_CH1]); //cstat !MISRAC2012-Rule-11.3
    LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_1, temp1);
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
  </#if>
  <#if MC.DEBUG_DAC_TIMER_CH2 != "NOT_USED" >
    LL_TIM_OC_SetCompare${MC.DEBUG_DAC_TIMER_CH2}(${MC.DEBUG_DAC_TIMER_SELECTION},
                          (uint16_t)((int16_t)((*((int16_t *)pHandle->ptrDataCh[DAC_CH2]) + 32768) / 32)));
  
  <#elseif MC.DEBUG_DAC_CH2_EN == true >  
    uint32_t temp2;
    temp2 = DACOFF + (uint32_t)*((uint16_t*) pHandle->ptrDataCh[DAC_CH2]); //cstat !MISRAC2012-Rule-11.3
    LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_2, temp2);
    LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);
  </#if>
#ifdef NULL_PTR_CHECK_DAC_UI
  }
#endif
}
<#else> <#-- DUAL Drive use case -->
__weak void DAC_Exec(DAC_Handle_t *pHandle, uint8_t motorID)
{
#ifdef NULL_PTR_CHECK_DAC_UI
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
  <#if MC.DEBUG_DAC_TIMER_CH1 != "NOT_USED">


    if (motorID == pHandle->motorCh[DAC_CH1])
    {
      LL_TIM_OC_SetCompare$ {MC.DEBUG_DAC_TIMER_CH1}(${MC.DEBUG_DAC_TIMER_SELECTION},
                            (uint16_t)((int16_t)((*((int16_t *)pHandle->ptrDataCh[DAC_CH1]) + 32768) / 32)));
    }
  <#elseif MC.DEBUG_DAC_CH1_EN == true> 
    if (motorID == pHandle->motorCh[DAC_CH1])
    {
      uint32_t temp1;
      temp1 = DACOFF + (uint32_t)*((uint16_t*)pHandle->ptrDataCh[DAC_CH1]); //cstat !MISRAC2012-Rule-11.3
      LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_1, temp1);
      LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_1);
    }
  </#if><#-- MC.DEBUG_DAC_TIMER_CH1 != "NOT_USED" -->
  <#if MC.DEBUG_DAC_TIMER_CH2 != "NOT_USED">
    if (motorID == pHandle->motorCh[DAC_CH2])
    {
      LL_TIM_OC_SetCompare${MC.DEBUG_DAC_TIMER_CH2}(${MC.DEBUG_DAC_TIMER_SELECTION},
                           (uint16_t)((int16_t)((*((int16_t *)pHandle->ptrDataCh[DAC_CH2]) + 32768) / 32))); 
    }  
  <#elseif MC.DEBUG_DAC_CH2_EN == true>
    if (motorID == pHandle->motorCh[DAC_CH2])
    {
      uint32_t temp2;
      temp2 = DACOFF + (uint32_t)*((uint16_t*)pHandle->ptrDataCh[DAC_CH2]); //cstat !MISRAC2012-Rule-11.3
      LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_2, temp2);
      LL_DAC_TrigSWConversion(DAC1, LL_DAC_CHANNEL_2);
    }
  </#if><#-- MC.DEBUG_DAC_TIMER_CH2 != "NOT_USED" -->
#ifdef NULL_PTR_CHECK_DAC_UI
  }
#endif
}
</#if><#-- MC.DRIVE_NUMBER == 1 -->

/**
  * @brief  Set up the DAC outputs. The selected
  *         variables will be provided in the related output channels after next
  *         DACExec.
  * @param  user interface handle.
  * @param  bChannel the DAC channel to be programmed. It must be one of the
  *         exported channels Ex. DAC_CH1.
  * @param  regID the variables to be provided in out through the selected
  *         channel. It must be one of the exported UI register Ex.
  *         MC_PROTOCOL_REG_I_A.
  */
__weak void DAC_SetChannelConfig(DAC_Handle_t *pHandle, DAC_Channel_t bChannel, uint16_t regID)
{
#ifdef NULL_PTR_CHECK_DAC_UI
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->dataCh[bChannel] = regID;
    (void)RI_GetPtrReg(regID, (void *)&pHandle->ptrDataCh[bChannel]); //cstat !MISRAC2012-Rule-11.5
<#if MC.DRIVE_NUMBER != "1">
    pHandle->motorCh[bChannel] = EXTRACT_MOTORID(regID);
</#if><#-- MC.DRIVE_NUMBER != 1 -->
#ifdef NULL_PTR_CHECK_DAC_UI
  }
#endif
}

uint16_t DAC_GetChannelConfig(DAC_Handle_t *pHandle, DAC_Channel_t bChannel)
{
#ifdef NULL_PTR_CHECK_DAC_UI
  return ((NULL == pHandle) ? 0U : pHandle->dataCh[bChannel]);
#else
  return (pHandle->dataCh[bChannel]);
#endif
}



/**
  * @}
  */

/**
  * @}
  */


/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
