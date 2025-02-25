<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    regular_conversion_manager.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the regular_conversion_manager component of the Motor Control SDK:
  *           Register conversion with or without callback
  *           Execute regular conv directly from Temperature and VBus sensors
  *           Execute user regular conversion scheduled by medium frequency task
  *           Manage user conversion state machine
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
#include "regular_conversion_manager.h"
#include "mc_config.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup RCM Regular Conversion Manager 
  * @brief Regular Conversion Manager component of the Motor Control SDK
  *
  * MotorControl SDK makes an extensive usage of ADCs. Some conversions are timing critical
  * like current reading, and some have less constraints. If an ADC offers both Injected and Regular,
  * channels, critical conversions will be systematically done on Injected channels, because they 
  * interrupt any ongoing regular conversion so as to be executed without delay.
  * Others conversions, mainly Bus voltage, and Temperature sensing are performed with regular channels.
  * If users wants to perform ADC conversions with an ADC already used by MC SDK, they must use regular
  * conversions. It is forbidden to use Injected channel on an ADC that is already in use for current reading.
  * As usera and MC-SDK may share ADC regular scheduler, this component intents to manage all the 
  * regular conversions.
  * 
  * If users wants to execute their own conversion, they first have to register it through the 
  * RCM_RegisterRegConv_WithCB() or RCM_RegisterRegConv() APIs. Multiple conversions can be registered, 
  * but only one can be scheduled at a time .
  *
  * A requested user regular conversion will be executed by the medium frequency task after the 
  * MC-SDK regular safety conversions: Bus voltage and Temperature.
  *
  * If a callback is registered, particular care must be taken with the code executed inside the CB.
  * The callback code is executed under Medium frequency task IRQ context (Systick).
  *
  * If the Users do not register a callback, they must poll the RCM state machine to know if
  * a conversion is ready to be read, scheduled, or free to be scheduled. This is performed through 
  * the RCM_GetUserConvState() API.
  *
  * If the state is #RCM_USERCONV_IDLE, a conversion is ready to be scheduled.
  * if a conversion is already scheduled, the returned value is #RCM_USERCONV_REQUESTED.
  * if a conversion is ready to be read, the returned value is #RCM_USERCONV_EOC.
  * In #RCM_USERCONV_EOC state, a call to RCM_GetUserConv will consume the value, and set the state machine back
  * to #RCM_USERCONV_IDLE state. It implies that a second call without new conversion performed,
  * will send back 0xffff which is an error value meaning that the data is not available.
  * If a conversion request is executed, but the previous conversion has not been completed, nor consumed, 
  * the request is discarded and the RCM_RequestUserConv() return false.
  * 
  * If a callback is registered, the data read is sent back to the callback parameters, and therefor consumed.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
<#if NoInjectedChannel>
/**
  * @brief Document as stated in template.h
  *   
  * ...
  */
typedef enum
{
  notvalid,
  ongoing,
  valid
} RCM_status_t;
 
typedef struct 
{
  bool enable;
  RCM_status_t status;
  uint16_t value;
  uint8_t prev;
  uint8_t next;
} RCM_NoInj_t;
</#if><#-- NoInjectedChannel -->

typedef struct 
{
  RCM_exec_cb_t cb;
  void *data;
} RCM_callback_t;

/* Private defines -----------------------------------------------------------*/
/**
  * @brief Number of regular conversion allowed By default.
  *  
  * In single drive configuration, it is defined to 4. 2 of them are consumed by 
  * Bus voltage and temperature reading. This leaves 2 handles available for 
  * user conversions
  *
  * In dual drives configuration, it is defined to 6. 2 of them are consumed by 
  * Bus voltage and temperature reading for each motor. This leaves 2 handles 
  * available for user conversion.
  *
  <#if MC.DRIVE_NUMBER == "1">
  * Defined to 4 here. 
  <#else><#-- MC.DRIVE_NUMBER != 1 -->
  * Defined to 6 here. 
  </#if><#-- MC.DRIVE_NUMBER == 1 -->
  */
#define RCM_MAX_CONV <#if MC.DRIVE_NUMBER == "1" > 4U <#else> 6U </#if>


/* Global variables ----------------------------------------------------------*/

static RegConv_t *RCM_handle_array[RCM_MAX_CONV];
static RCM_callback_t RCM_CB_array[RCM_MAX_CONV];

<#if NoInjectedChannel>
static RCM_NoInj_t RCM_NoInj_array[RCM_MAX_CONV];
static uint8_t RCM_currentHandle;
</#if><#-- NoInjectedChannel -->
static uint16_t RCM_UserConvValue;
static RCM_UserConvState_t RCM_UserConvState;
static RegConv_t* RCM_UserConvHandle;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Registers a regular conversion, and attaches a callback.
  * 
  * This function registers a regular ADC conversion that can be later scheduled for execution. It
  * returns a handle that uniquely identifies the conversion. This handle is used in the other API
  * of the Regular Converion Manager to reference the registered conversion.
  * 
  * A regular conversion is defined by an ADC + ADC channel pair. If a registration already exists 
  * for the requested ADC + ADC channel pair, the same handle will be reused.
  *
  * The regular conversion is registered along with a callback that is executed each time the
  * conversion has completed. The callback is invoked with two parameters:
  *
  * - the handle of the regular conversion
  * - a data pointer, supplied by uthe users at registration time.
  * 
  * The registration may fail if there is no space left for additional conversions. The 
  * maximum number of regular conversion that can be registered is defined by #RCM_MAX_CONV.
  *  
  * @note   Users who do not want a callback to be executed at the end of the conversion, 
  *         should use RCM_RegisterRegConv() instead.
  *
  * @param  regConv Pointer to the regular conversion parameters. 
  *         Contains ADC, Channel and sampling time to be used.
  *
  * @param  fctCB Function called once the regular conversion is executed.
  *
  * @param  Data Used to save a user context. this parameter will be send back by 
  *               the fctCB function. @b Note: This parameter can be NULL if not used.
  *
  */
void RCM_RegisterRegConv_WithCB (RegConv_t *regConv, RCM_exec_cb_t fctCB, void *data)
{
  
  RCM_RegisterRegConv(regConv);
  if (regConv->convHandle < RCM_MAX_CONV)
  {
    RCM_CB_array [regConv->convHandle].cb = fctCB;
    RCM_CB_array [regConv->convHandle].data = data;
  }
  else
  {
    /* Nothing to do */
  }
}

/**
  * @brief  Registers a regular conversion.
  * 
  * This function registers a regular ADC conversion that can be later scheduled for execution. It
  * returns a handle that uniquely identifies the conversion. This handle is used in the other API
  * of the Regular Converion Manager to reference the registered conversion.
  * 
  * A regular conversion is defined by an ADC + ADC channel pair. If a registration already exists 
  * for the requested ADC + ADC channel pair, the same handle will be reused.
  *
  * The registration may fail if there is no space left for additional conversions. The 
  * maximum number of regular conversion that can be registered is defined by #RCM_MAX_CONV.
  *  
  * @note   Users who do not want a callback to be executed at the end of the conversion, 
  *         should use RCM_RegisterRegConv() instead.
  *
  * @param  regConv Pointer to the regular conversion parameters. 
  *         Contains ADC, Channel and sampling time to be used.
  *
  */
void RCM_RegisterRegConv(RegConv_t *regConv)
{
  uint8_t handle = 255U;
#ifdef NULL_PTR_CHECK_REG_CON_MNG 
  if (MC_NULL == regConv)
  {
    handle = 0U;
  }
  else
  {
#endif
    uint8_t i = 0;
    
    /* Parse the array to be sure that same 
     * conversion does not already exist*/
    while (i < RCM_MAX_CONV)
    { 
      if ((0 == RCM_handle_array [i]) && (handle > RCM_MAX_CONV))  
      {
        handle = i; /* First location available, but still looping to check that this config does not already exist */
      }
      else 
      {
        /* Nothing to do */
      }
      /* Ticket 64042 : If RCM_handle_array [i] is null access to data member will cause Memory Fault */
      if (RCM_handle_array [i] != 0)
      {
        if ((RCM_handle_array [i]->channel == regConv->channel) 
         && (RCM_handle_array [i]->regADC == regConv->regADC))
        {
          handle = i; /* Reuse the same handle */
          i = RCM_MAX_CONV; /* We can skip the rest of the loop */
        }
        else 
        {
          /* Nothing to do */
        }
      }
      else 
      {
        /* Nothing to do */
      }
      i++;
    }    
    if (handle < RCM_MAX_CONV)
    {
      RCM_handle_array [handle] = regConv;
      RCM_CB_array [handle].cb = NULL; /* If a previous callback was attached, it is cleared */
      if (0U == LL_ADC_IsEnabled(regConv->regADC))
      {
<#if CondFamily_STM32F0>
<#-- useless as there is only one ADC -->
        <#elseif  CondFamily_STM32F3 || CondFamily_STM32L4 || CondFamily_STM32G4 || CondFamily_STM32H5>
        LL_ADC_DisableIT_EOC(regConv->regADC);
        LL_ADC_ClearFlag_EOC(regConv->regADC);
        LL_ADC_DisableIT_JEOC(regConv->regADC);
        LL_ADC_ClearFlag_JEOC(regConv->regADC);
        <#elseif  CondFamily_STM32F4 || CondFamily_STM32F7>
        LL_ADC_DisableIT_EOCS(regConv->regADC);
        LL_ADC_ClearFlag_EOCS(regConv->regADC);
        LL_ADC_DisableIT_JEOS(regConv->regADC);
        LL_ADC_ClearFlag_JEOS(regConv->regADC);
</#if><#-- CondFamily_STM32F0 -->

<#if !CondFamily_STM32F4 && !CondFamily_STM32L4 && !CondFamily_STM32F7>
  <#if CondFamily_STM32F0 || CondFamily_STM32G0 || CondFamily_STM32C0>
        LL_ADC_StartCalibration( regConv->regADC);
  <#elseif CondFamily_STM32H7>
        LL_ADC_StartCalibration(regConv->regADC, LL_ADC_CALIB_OFFSET_LINEARITY, LL_ADC_SINGLE_ENDED);
  <#else><#-- CondFamily_STM32F0 == false || CondFamily_STM32G0 == false || CondFamily_STM32C0 == false && CondFamily_STM32H7 == false -->
        LL_ADC_StartCalibration(regConv->regADC, LL_ADC_SINGLE_ENDED);
  </#if><#-- CondFamily_STM32F0 || CondFamily_STM32G0 || CondFamily_STM32C0 -->
        while (1U == LL_ADC_IsCalibrationOnGoing(regConv->regADC))  
        {
          /* Nothing to do */
        }
</#if><#-- !CondFamily_STM32F4 && !CondFamily_STM32L4 && !CondFamily_STM32F7 -->
<#if CondFamily_STM32F3 || CondFamily_STM32G4 || CondFamily_STM32H5>
        <#-- This is done only for G4 because clock ratio 1/10 flag this issue -->
        /* ADC Enable (must be done after calibration) */
        /* ADC5-140924: Enabling the ADC by setting ADEN bit soon after polling ADCAL=0 
        * following a calibration phase, could have no effect on ADC 
        * within certain AHB/ADC clock ratio
        */
        while (0U == LL_ADC_IsActiveFlag_ADRDY(regConv->regADC))  
        { 
          LL_ADC_Enable(regConv->regADC);
        }

<#else><#-- CondFamily_STM32G4 == false -->
        LL_ADC_Enable(regConv->regADC);
</#if><#-- CondFamily_STM32G4 -->
      }
      else 
      {
        /* Nothing to do */
      }
<#if NoInjectedChannel>
      /* Conversion handler is created, will be enabled by the first call to RCM_ExecRegularConv */
      RCM_NoInj_array[handle].enable = false;
      RCM_NoInj_array[handle].next = handle;
      RCM_NoInj_array[handle].prev = handle;
  <#if CondFamily_STM32G0 || CondFamily_STM32C0><#-- MCU using 2 common sampling groups ; should be removed after CubeMx 6.12, see ticket 179556 -->
      LL_ADC_SetChannelSamplingTime(regConv->regADC, __LL_ADC_DECIMAL_NB_TO_CHANNEL(regConv->channel), LL_ADC_SAMPLINGTIME_COMMON_2);
  </#if><#-- CondFamily_STM32G0 || CondFamily_STM32C0 -->
  <#if G4_Cut2_2_patch>
      /* Reset regular conversion sequencer length set by cubeMX */
      LL_ADC_REG_SetSequencerLength(regConv->regADC, LL_ADC_REG_SEQ_SCAN_DISABLE);
      /* Configure the sampling time (should already be configured by for non user conversions) */
      LL_ADC_SetChannelSamplingTime(regConv->regADC, __LL_ADC_DECIMAL_NB_TO_CHANNEL(regConv->channel),
                                    regConv->samplingTime);
  </#if><#-- G4_Cut2_2_patch -->
<#else><#-- NoInjectedChannel == false -->
      /* Reset regular conversion sequencer length set by cubeMX */
      LL_ADC_REG_SetSequencerLength(regConv->regADC, LL_ADC_REG_SEQ_SCAN_DISABLE);
      /* Configure the sampling time (should already be configured by for non user conversions) */
      LL_ADC_SetChannelSamplingTime (regConv->regADC, __LL_ADC_DECIMAL_NB_TO_CHANNEL(regConv->channel),
                                     regConv->samplingTime);
</#if><#-- NoInjectedChannel -->
    }
    else
    {
      /* Nothing to do handle is already set to error value : 255 */
    }
#ifdef NULL_PTR_CHECK_REG_CON_MNG
  }
#endif
  regConv->convHandle = handle;  
}

/*
 * This function is used to read the result of a regular conversion.
<#if NoInjectedChannel>
 * Depending of the MC state machine, this function can poll on the ADC end of conversion or not.
 * If the ADC is already in use for currents sensing, the regular conversion can not
 * be executed instantaneously but have to be scheduled in order to be executed after currents sensing
 * inside HF task.
 * This function takes care of inserting the handle into the scheduler.
 * If it is possible to execute the conversion instantaneously, it will be executed, and result returned.
 * Otherwise, the latest stored conversion result will be returned.
<#else>
 * This function polls on the ADC end of conversion.
 * As ADC have injected channels for currents sensing, 
 * There is no issue to execute regular conversion asynchronously.
</#if>
 *
 * NOTE: This function is not part of the public API and users should not call it. 
 */
uint16_t RCM_ExecRegularConv (RegConv_t *regConv)
{
  uint16_t retVal;
  uint8_t handle = regConv->convHandle;
<#if NoInjectedChannel>
  uint8_t formerNext;
  uint8_t i=0;
  uint8_t LastEnable = RCM_MAX_CONV;

  if (false == RCM_NoInj_array [handle].enable)
  {
    /* Find position in the list */
    while (i < RCM_MAX_CONV)
    {
      if (true == RCM_NoInj_array [i].enable)
      {
        if (RCM_NoInj_array[i].next > handle)
        /* We found a previous reg conv to link with */
        {
          formerNext = RCM_NoInj_array [i].next;
          RCM_NoInj_array[handle].next = formerNext;
          RCM_NoInj_array[handle].prev = i;
          RCM_NoInj_array[i].next = handle;
          RCM_NoInj_array[formerNext].prev = handle;
          i = RCM_MAX_CONV; /* Stop the loop, handler inserted */
        }
        else
        { /* We found an enabled regular conv, 
           * but do not know yet if it is the one we have to be linked to */
          LastEnable = i;
        }
      }
      else
      { 
        /* Nothing to do */
      }
      i++;
      if (RCM_MAX_CONV == i) 
      /* We reach end of the array without handler inserted */
      {
       if (LastEnable != RCM_MAX_CONV )
       /* We find a regular conversion with smaller position to be linked with */
       {
         formerNext = RCM_NoInj_array[LastEnable].next;
         RCM_NoInj_array[handle].next = formerNext;
         RCM_NoInj_array[handle].prev = LastEnable;
         RCM_NoInj_array[LastEnable].next = handle;
         RCM_NoInj_array[formerNext].prev = handle;
       }
       else 
       { /* The current handle is the only one in the list */
         /* Previous and next are already pointing to itself (done at registerRegConv) */
         RCM_currentHandle = handle;
       }       
      }
      else 
      {
        /* Nothing to do we are parsing the array, nothing inserted yet */
      }
    }
    /* The handle is now linked with others, we can set the enable flag */
    RCM_NoInj_array[handle].enable = true;
    RCM_NoInj_array[handle].status = notvalid;
    if (RCM_NoInj_array[RCM_currentHandle].status != ongoing)
    {/* Select the new conversion to be the next scheduled only if a conversion is not ongoing */
      RCM_currentHandle = handle;
    }
    else
    {
      /* Nothing to do */
    } 
  }
  else
  {
    /* Nothing to do the current handle is already scheduled */
  }
  <#if ACIM>
  if (false == PWM_Handle_M1.ADCRegularLocked)
  </#if><#-- ACIM -->
  <#if FOC>
    <#if MC.DRIVE_NUMBER == "1">
  if (false == PWM_Handle_M1.ADCRegularLocked)
    <#else><#-- MC.DRIVE_NUMBER == 1 -->
  if ((false == PWM_Handle_M1.ADCRegularLocked) && (false == PWM_Handle_M2.ADCRegularLocked))
    </#if><#-- MC.DRIVE_NUMBER == 1 -->
  </#if><#-- FOC -->
  <#if SIX_STEP && MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
  if (false == Bemf_ADC_M1.ADCRegularLocked)  
  </#if><#-- SIX_STEP && MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
  /* The ADC is free to be used asynchronously */
  {
  <#if G4_Cut2_2_patch>
    LL_ADC_REG_SetSequencerRanks(RCM_handle_array[handle]->regADC,
                                 LL_ADC_REG_RANK_1,
                                 __LL_ADC_DECIMAL_NB_TO_CHANNEL(RCM_handle_array[handle]->channel));

    (void)LL_ADC_REG_ReadConversionData12L(RCM_handle_array[handle]->regADC);
  <#else><#-- G4_Cut2_2_patch == false -->
    LL_ADC_REG_SetDMATransfer(RCM_handle_array[handle]->regADC, LL_ADC_REG_DMA_TRANSFER_NONE);
  
    /* ADC STOP condition requested to write CHSELR is true because of the ADCSTOP is set by hardware
       at the end of A/D conversion if the external Trigger of ADC is disabled */
  
    /* By default it is ADSTART = 0, then at the first time the CFGR1 can be written */
  
    /* Disabling External Trigger of ADC */
    LL_ADC_REG_SetTriggerSource(RCM_handle_array[handle]->regADC, LL_ADC_REG_TRIG_SOFTWARE);
  
    /* Set Sampling time and channel */
    <#if CondFamily_STM32G0 || CondFamily_STM32C0>
    LL_ADC_SetSamplingTimeCommonChannels(RCM_handle_array[handle]->regADC, LL_ADC_SAMPLINGTIME_COMMON_2,
                                         RCM_handle_array[handle]->samplingTime);
    <#else><#-- CondFamily_STM32G0 == false && CondFamily_STM32G0 == false -->
    LL_ADC_SetSamplingTimeCommonChannels(RCM_handle_array[handle]->regADC, RCM_handle_array[handle]->samplingTime);
    </#if><#-- CondFamily_STM32G0 -->
    LL_ADC_REG_SetSequencerChannels(RCM_handle_array[handle]->regADC,
                                    __LL_ADC_DECIMAL_NB_TO_CHANNEL(RCM_handle_array[handle]->channel));

    /* Clear EOC */
    LL_ADC_ClearFlag_EOC(RCM_handle_array[handle]->regADC);

  </#if><#-- G4_Cut2_2_patch -->
    /* Start ADC conversion */
    LL_ADC_REG_StartConversion(RCM_handle_array[handle]->regADC);

    /* Wait EOC */
    while ( 0U == LL_ADC_IsActiveFlag_EOC(RCM_handle_array[handle]->regADC))
    {
      /* Nothing to do */
    }

    /* Read the "Regular" conversion (Not related to current sampling) */
    RCM_NoInj_array[handle].value = LL_ADC_REG_ReadConversionData12L(RCM_handle_array[handle]->regADC);
  <#if !G4_Cut2_2_patch>
    LL_ADC_REG_SetDMATransfer(RCM_handle_array[RCM_currentHandle]->regADC, LL_ADC_REG_DMA_TRANSFER_LIMITED);
  </#if><#-- !G4_Cut2_2_patch -->
    RCM_currentHandle = RCM_NoInj_array[handle].next;
    RCM_NoInj_array[handle].status = valid;
  }
  <#if FOC>
  else
  {
    /* Nothing to do */
  }
  </#if><#-- FOC -->
  <#if SIX_STEP && MC.M1_SPEED_SENSOR == "SENSORLESS_ADC">
  else
  {
    /* Nothing to do */
  }
  </#if><#-- SIX_STEP && MC.M1_SPEED_SENSOR == "SENSORLESS_ADC" -->
  retVal = RCM_NoInj_array[handle].value;
<#else><#-- We do have injected channel, can do the sampling asynchronously -->
  LL_ADC_REG_SetSequencerRanks(RCM_handle_array[handle]->regADC,
                               LL_ADC_REG_RANK_1,
                               __LL_ADC_DECIMAL_NB_TO_CHANNEL(RCM_handle_array[handle]->channel));

  (void)LL_ADC_REG_ReadConversionData12L(RCM_handle_array[handle]->regADC);


  <#if CondFamily_STM32F4><#-- F4 requires explicitly bitbanding access otherwise dual drive 1 shunt fails -->
  /* Bit banding access equivalent to LL_ADC_REG_StartConversionSWStart */
  BB_REG_BIT_SET(&RCM_handle_array[handle]->regADC->CR2, ADC_CR2_SWSTART_Pos);
  /* Wait until end of regular conversion */
  while (LL_ADC_IsActiveFlag_EOCS(RCM_handle_array[handle]->regADC) == 0u)
  {
    /* Nothing to do */
  }
  <#elseif CondFamily_STM32F7>
  LL_ADC_REG_StartConversionSWStart (RCM_handle_array[handle]->regADC);
  /* Wait until end of regular conversion */
  while (LL_ADC_IsActiveFlag_EOCS(RCM_handle_array[handle]->regADC) == 0u)
  {
    /* Nothing to do */
  }
  <#else><#--  CondFamily_STM32F4 == false &&  CondFamily_STM32F7 == false -->
  LL_ADC_REG_StartConversion(RCM_handle_array[handle]->regADC);
  /* Wait until end of regular conversion */
  while (LL_ADC_IsActiveFlag_EOC(RCM_handle_array[handle]->regADC) == 0u)
  {
    /* Nothing to do */
  }
  </#if><#-- CondFamily_STM32F4 -->
  retVal = LL_ADC_REG_ReadConversionData12L(RCM_handle_array[handle]->regADC);
</#if><#-- NoInjectedChannel -->
  return (retVal);
}

/**
 * @brief Schedules a regular conversion for execution.
 *  
 * This function requests the execution of the user-defined regular conversion identified
 * by @p handle. All user defined conversion requests must be performed inside routines with the
 * same priority level. If a previous regular conversion request is pending this function has no 
 * effect, for this reason is better to call RCM_GetUserConvState() and check if the state is 
 * #RCM_USERCONV_IDLE before calling RCM_RequestUserConv().
 *
 * @param  handle used for the user conversion.
 *
 * @return True if the regular conversion could be scheduled and false otherwise.
 */
bool RCM_RequestUserConv(RegConv_t *regConv)
{
  bool retVal = false;
  if (RCM_USERCONV_IDLE == RCM_UserConvState)
  {
    RCM_UserConvHandle = regConv;
    /* must be done last so that RCM_UserConvHandle already has the right value */
    RCM_UserConvState = RCM_USERCONV_REQUESTED;
    retVal = true;
  }
  else
  {
    /* Nothing to do */
  }
  return (retVal);
}

/**
 * @brief  Returns the last user-defined regular conversion that was executed.
 *
 * This function returns a valid result if the state returned by
 * RCM_GetUserConvState is #RCM_USERCONV_EOC.
 *
 * @retval uint16_t The converted value or 0xFFFF in case of conversion error.
 */
uint16_t RCM_GetUserConv(void)
{
  uint16_t hRetVal = 0xFFFFu;
  if (RCM_USERCONV_EOC == RCM_UserConvState)
  {
    hRetVal = RCM_UserConvValue;
    RCM_UserConvState = RCM_USERCONV_IDLE;
  }
  else
  {
    /* Nothing to do */
  }
  return (hRetVal);
}

/*
 *  This function must be scheduled by mc_task.
 *  It executes the current user conversion that has been selected by the 
 *  latest call to RCM_RequestUserConv.
 *
 * NOTE: This function is not part of the public API and users should not call it. 
 */
void RCM_ExecUserConv()
{
  uint8_t handle;
  if (RCM_UserConvHandle != NULL)
  {
    handle = RCM_UserConvHandle->convHandle;
    if (RCM_USERCONV_REQUESTED == RCM_UserConvState)
    {
      RCM_UserConvValue = RCM_ExecRegularConv(RCM_UserConvHandle);
<#if NoInjectedChannel>
      /* Regular conversion is read from RCM_NoInj_array but we must take care that first conversion is done */
      /* Status could also be ongoing, but decision is taken to provide previous conversion
       * instead of waiting for RCM_NoInj_array [handle].status == valid */
      if (RCM_NoInj_array [handle].status != notvalid) 
      {
        RCM_UserConvState = RCM_USERCONV_EOC;
      }
      else
      {
        /* Nothing to do */
      }
<#else><#-- NoInjectedChannel == false -->
      RCM_UserConvState = RCM_USERCONV_EOC;
</#if><#-- NoInjectedChannel -->
      if (RCM_CB_array[handle].cb != NULL)
      {
        RCM_UserConvState = RCM_USERCONV_IDLE;
        RCM_CB_array[handle].cb(RCM_UserConvHandle, RCM_UserConvValue,
                                            RCM_CB_array[handle].data);
      }
      else
      {
        /* Nothing to do */
      }
    }
  }
  else
  {
     /* Nothing to do */
  }
}

/**
 * @brief  Returns the status of the last requested regular conversion.
 *
 * It can be one of the following values:
 
 * - UDRC_STATE_IDLE no regular conversion request pending.
 * - UDRC_STATE_REQUESTED regular conversion has been requested and not completed.
 * - UDRC_STATE_EOC regular conversion has been completed but not readed from the user.
 *
 * @retval The state of the last user-defined regular conversion.
 */
RCM_UserConvState_t RCM_GetUserConvState(void)
{
  return (RCM_UserConvState);
}


<#if NoInjectedChannel>
/**
 * @brief  Un-schedules a regular conversion
 *
 * This function does not poll ADC read and is meant to be used when 
 * ADCs do not support injected channels.
 *
 * In such configurations, once a regular conversion has been executed once,
 * It is continuously scheduled in HF task after current reading.
 *
 * This function remove the handle from the scheduling.
 *
 * @note Note that even though, in such configurations, regular conversions are 
 *       continuously scheduled after having been requested once, the results of
 *       subsequent conversions are not made available unless the users invoke
 *       RCM_RequestUserConv() again.   
 *
 */
bool RCM_PauseRegularConv(RegConv_t *regConv)
{
  bool retVal;
  uint8_t Prev;
  uint8_t Next;
  uint8_t handle = regConv->convHandle;
  
  if (handle < RCM_MAX_CONV)
  {
    retVal = true;
    if (true == RCM_NoInj_array [handle].enable)
    {
      RCM_NoInj_array [handle].enable = false;
      RCM_NoInj_array [handle].status = notvalid;
      Prev = RCM_NoInj_array [handle].prev;
      Next = RCM_NoInj_array [handle].next;
      RCM_NoInj_array [Prev].next = RCM_NoInj_array [handle].next;
      RCM_NoInj_array [Next].prev = RCM_NoInj_array [handle].prev;
    }
    else 
    {
      /* Nothing to do */
    }
  }
  else 
  {
    retVal = false;
  }
  return (retVal);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/*
 * Starts the next scheduled regular conversion
 *
 * This function does not poll on ADC read and is foreseen to be used inside
 * high frequency task where ADC are shared between currents reading
 * and user conversion.
 *
 * NOTE: This function is not part of the public API and users should not call it. 
 */
void RCM_ExecNextConv(void)
{
  if (true == RCM_NoInj_array [RCM_currentHandle].enable) 
  {
    /* When this function is called, the ADC conversions triggered by External
       event for current reading has been completed. 
       ADC is therefore ready to be started because already stopped */
       
    /* Clear EOC */
    LL_ADC_ClearFlag_EOC(RCM_handle_array[RCM_currentHandle]->regADC);
  <#if G4_Cut2_2_patch>
    LL_ADC_REG_SetSequencerRanks(RCM_handle_array[RCM_currentHandle]->regADC,
                                 LL_ADC_REG_RANK_1,
                                 __LL_ADC_DECIMAL_NB_TO_CHANNEL(RCM_handle_array[RCM_currentHandle]->channel));

    (void)LL_ADC_REG_ReadConversionData12L(RCM_handle_array[RCM_currentHandle]->regADC);
    
  <#else><#-- G4_Cut2_2_patch == flase -->
    /* Disabling ADC DMA request */
    LL_ADC_REG_SetDMATransfer(RCM_handle_array[RCM_currentHandle]->regADC, LL_ADC_REG_DMA_TRANSFER_NONE);
    
    /* Disabling External Trigger of ADC */
    LL_ADC_REG_SetTriggerSource(RCM_handle_array[RCM_currentHandle]->regADC, LL_ADC_REG_TRIG_SOFTWARE);
      
    /* Set Sampling time and channel of ADC for Regular Conversion */
      <#if CondFamily_STM32G0 || CondFamily_STM32C0>
    LL_ADC_SetSamplingTimeCommonChannels(RCM_handle_array[RCM_currentHandle]->regADC, LL_ADC_SAMPLINGTIME_COMMON_2,
                                         RCM_handle_array[RCM_currentHandle]->samplingTime);
      <#else><#-- CondFamily_STM32G0 == false -->
    LL_ADC_SetSamplingTimeCommonChannels(RCM_handle_array[RCM_currentHandle]->regADC,
                                         RCM_handle_array[RCM_currentHandle]->samplingTime);
      </#if><#-- CondFamily_STM32G0 -->
    (void)LL_ADC_REG_SetSequencerChannels(RCM_handle_array[RCM_currentHandle]->regADC,
                                       __LL_ADC_DECIMAL_NB_TO_CHANNEL(RCM_handle_array[RCM_currentHandle]->channel));
  </#if><#-- G4_Cut2_2_patch -->

    /* Start ADC for regular conversion */
    LL_ADC_REG_StartConversion(RCM_handle_array[RCM_currentHandle]->regADC);
    RCM_NoInj_array[RCM_currentHandle].status = ongoing;
  }
  else
  {
    /* Nothing to do, conversion not enabled have already notvalid status */
  }
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/*
 * Reads the result of the ongoing regular conversion
 *
 * This function is foreseen to be used inside
 * high frequency task where ADC are shared between current reading
 * and user conversion.
 *
 * NOTE: This function is not part of the public API and users should not call it. 
 */
void RCM_ReadOngoingConv(void)
{
  uint32_t result;
  RCM_status_t status;

  if (true == RCM_NoInj_array [RCM_currentHandle].enable)
  {
    status = RCM_NoInj_array[RCM_currentHandle].status;
    result = LL_ADC_IsActiveFlag_EOC(RCM_handle_array[RCM_currentHandle]->regADC);
    if (( valid == status ) || ( notvalid == status ) || ( 0U == result ))
    {
      /* Nothing to do */
    }
    else
    {
      /* Reading of ADC Converted Value */
      RCM_NoInj_array[RCM_currentHandle].value
                    = LL_ADC_REG_ReadConversionData12L(RCM_handle_array[RCM_currentHandle]->regADC);
      RCM_NoInj_array[RCM_currentHandle].status = valid;
      /* Restore back DMA configuration */
    <#if !G4_Cut2_2_patch>
      LL_ADC_REG_SetDMATransfer( RCM_handle_array[RCM_currentHandle]->regADC, LL_ADC_REG_DMA_TRANSFER_LIMITED );
    </#if><#-- !G4_Cut2_2_patch -->
    }

    /* Prepare next conversion */
    RCM_currentHandle = RCM_NoInj_array [RCM_currentHandle].next;
    }
    else
    {
      /* Nothing to do */
    }
}
</#if><#-- NoInjectedChannel -->

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/




