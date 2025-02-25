<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    r3_g4xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
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
  * @ingroup R3_G4XX_pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "r3_g4xx_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"
#include "parameters_conversion.h"
#include "drive_parameters.h"
#include "oversampling.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup R3_G4XX_pwm_curr_fdbk G4 PWM & Current Feedback
 *
 * @brief STM32G4 3-Shunt (2 or 3ADCs), ICS, PWM & Current Feedback implementation
 *
 * This component gathers all the routines that does hardware access related to PWM 
 * generation and phase voltages and currents reading. It will complete the peripheral 
 * initialization and configuration done by STM32CubeMX and serve the commands coming 
 * from the upper layer.
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/
/** @brief  Used to enable or disable all PWM channels */
#define TIMxCCER_MASK_CH123        ((uint16_t)  (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                                 LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                                 LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))
#define MIDDLE (PWM_PERIOD_CYCLES/OVS_COUNT/2)
#define DT_TN  (TNOISE+TDEAD)
#define DT_TN_TS (TDEAD+TNOISE+TW_BEFORE)
#define DT_TR_TS (TDEAD+TRISE+TW_BEFORE)
#define SAMPLE_OFFSET_TICKS TW_BEFORE

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
__STATIC_INLINE uint16_t R3_WriteTIMRegisters( PWMC_Handle_t * pHdl, uint16_t hCCR4Reg  );
static void R3_ADCxInit( ADC_TypeDef * ADCx );
static void R3_TIMxInit( TIM_TypeDef * TIMx, TIM_TypeDef * TIMx_Oversample, PWMC_Handle_t * pHdl );
static void R3_DMAChannelsInit( PWMC_R3_Handle_t* pHandle );
static void R3_SetDmaTargetBuffer(PWMC_R3_Handle_t * pHandle, int bufferIndex);
static void R3_OversamplingInit( PWMC_R3_Handle_t* pHandle, const scaleFlashParams_t *pScale );
<#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
static void R3_SetAOReferenceVoltage(uint32_t DAC_Channel, DAC_TypeDef *DACx, uint16_t hDACVref);
</#if>

/**
  * @brief  Initializes TIMx, ADC, OPAMP and DMA for current and voltage reading
  * @param  pHandle: handler of the current instance of the PWM component
  * @param  scaleFlashParams_t: address of the structure which stores the scaling parameters
  */
__weak void R3_Init( PWMC_R3_Handle_t * pHandle, const scaleFlashParams_t *pScale )
{
  /*
   * Before timers are started
   * Reconfigures peripherals for oversampling operation on regular samples
   */

<#if MC.M1_CS_OPAMP_NUM != "0">
  R3_3_OPAMPParams_t * OPAMPParams = pHandle->pParams_str->OPAMPParams;
</#if>
<#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
  COMP_TypeDef *COMP_OCPAx = pHandle->pParams_str->CompOCPASelection;
  COMP_TypeDef *COMP_OCPBx = pHandle->pParams_str->CompOCPBSelection;
  COMP_TypeDef *COMP_OCPCx = pHandle->pParams_str->CompOCPCSelection;
  DAC_TypeDef *DAC_OCPAx = pHandle->pParams_str->DAC_OCP_ASelection;
  DAC_TypeDef *DAC_OCPBx = pHandle->pParams_str->DAC_OCP_BSelection;
  DAC_TypeDef *DAC_OCPCx = pHandle->pParams_str->DAC_OCP_CSelection;
</#if>
<#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
  COMP_TypeDef *COMP_OVPx = pHandle->pParams_str->CompOVPSelection;
  DAC_TypeDef *DAC_OVPx = pHandle->pParams_str->DAC_OVP_Selection;
</#if>
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  TIM_TypeDef *TIMx_Oversample = pHandle->pParams_str->TIMx_Oversample;
  ADC_TypeDef *ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef *ADCx_2 = pHandle->pParams_str->ADCx_2;
<#if MC.M1_CS_ADC_NUM == "3"> 
  ADC_TypeDef *ADCx_3 = pHandle->pParams_str->ADCx_3;  
</#if>

  /*Check that _Super is the first member of the structure PWMC_R3_Handle_t */
  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {
    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOC( ADCx_1 );
    LL_ADC_ClearFlag_EOC( ADCx_1 );
    LL_ADC_DisableIT_JEOC( ADCx_1 );
    LL_ADC_ClearFlag_JEOC( ADCx_1 );
    LL_ADC_DisableIT_EOC( ADCx_2 );
    LL_ADC_ClearFlag_EOC( ADCx_2 );
    LL_ADC_DisableIT_JEOC( ADCx_2 );
    LL_ADC_ClearFlag_JEOC( ADCx_2 );
<#if MC.M1_CS_ADC_NUM == "3">
    LL_ADC_DisableIT_EOC( ADCx_3 );
    LL_ADC_ClearFlag_EOC( ADCx_3 );
    LL_ADC_DisableIT_JEOC( ADCx_3 );
    LL_ADC_ClearFlag_JEOC( ADCx_3 );      
</#if>
    /* TIM1 and TIM8 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM1_STOP );
    LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM8_STOP );
    LL_DBGMCU_APB1_GRP1_FreezePeriph( LL_DBGMCU_APB1_GRP1_TIM3_STOP );
    
    /* Enable the OPAMPs */
<#if MC.M1_CS_OPAMP_NUM != "0">
    if ( OPAMPParams != NULL )
    {
       /* Testing of all OPAMP one by one is required as 2 or 3 OPAMPS cfg may exist*/
       if (OPAMPParams -> OPAMPx_1 != NULL )
       {
         LL_OPAMP_Enable( OPAMPParams->OPAMPx_1 );
       }
       if (OPAMPParams -> OPAMPx_2 != NULL )
       {
         LL_OPAMP_Enable( OPAMPParams->OPAMPx_2 );
       }
       if (OPAMPParams -> OPAMPx_3 != NULL )
       {
         LL_OPAMP_Enable( OPAMPParams->OPAMPx_3 );
       }
    }
</#if>

<#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
    /* Overcurrent protection via the COMPs */
    /* Over current protection phase A */
    if (COMP_OCPAx != NULL)
    {
      /* Inverting input */
      if ((pHandle->pParams_str->CompOCPAInvInput_MODE != EXT_MODE) && (DAC_OCPAx != MC_NULL))
      {
        R3_SetAOReferenceVoltage(pHandle->pParams_str->DAC_Channel_OCPA, DAC_OCPAx,
                                   (uint16_t)(pHandle->pParams_str->DAC_OCP_Threshold));
      }
      else
      {
        /* Nothing to do */
      }
      /* Output */
      LL_COMP_Enable(COMP_OCPAx);
      LL_COMP_Lock(COMP_OCPAx);
    }
    else
    {
      /* Nothing to do */
    }
    
    /* Over current protection phase B */
    if (COMP_OCPBx != NULL)
    {
      if ((pHandle->pParams_str->CompOCPBInvInput_MODE != EXT_MODE) && (DAC_OCPBx != MC_NULL))
      {
        R3_SetAOReferenceVoltage(pHandle->pParams_str->DAC_Channel_OCPB, DAC_OCPBx,
                                   (uint16_t)(pHandle->pParams_str->DAC_OCP_Threshold));
      }
      else
      {
        /* Nothing to do */
      }
      LL_COMP_Enable(COMP_OCPBx);
      LL_COMP_Lock(COMP_OCPBx);
    }
    else
    {
      /* Nothing to do */
    }
    
    /* Over current protection phase C */
    if (COMP_OCPCx != NULL)
    {
      if ((pHandle->pParams_str->CompOCPCInvInput_MODE != EXT_MODE)  && (DAC_OCPCx != MC_NULL))
      {
        R3_SetAOReferenceVoltage(pHandle->pParams_str->DAC_Channel_OCPC, DAC_OCPCx,
                                   (uint16_t)(pHandle->pParams_str->DAC_OCP_Threshold));
      }
      else
      {
        /* Nothing to do */
      }
      LL_COMP_Enable(COMP_OCPCx);
      LL_COMP_Lock(COMP_OCPCx);
    }
    else
    {
      /* Nothing to do */
    }
</#if>
<#if MC.INTERNAL_OVERVOLTAGEPROTECTION>
    /* Overvoltage protection via the COMPs */
    if (COMP_OVPx != NULL)
    {
      /* Inverting input */
      if ((pHandle->pParams_str->CompOVPInvInput_MODE != EXT_MODE) && (DAC_OVPx != MC_NULL))
      {
        R3_2_SetAOReferenceVoltage(pHandle->pParams_str->DAC_Channel_OVP, DAC_OVPx,
                                   (uint16_t)(pHandle->pParams_str->DAC_OVP_Threshold));
      }
      else
      {
        /* Nothing to do */
      }
      /* Output */
      LL_COMP_Enable(COMP_OVPx);
      LL_COMP_Lock(COMP_OVPx);
    }
    else
    {
      /* Nothing to do */
    }
</#if>
    /* Initialize ADCs */
    R3_ADCxInit(ADCx_1);
    R3_ADCxInit(ADCx_2);
<#if MC.M1_CS_ADC_NUM == "3"> 
    R3_ADCxInit(ADCx_3);      
</#if>
    /* ADC interrupt left disabled intentionally */
    
    /* Initialize PWM TIMx */
    R3_TIMxInit( TIMx, TIMx_Oversample, &pHandle->_Super );
    
    /* Initialize the DMAs */
    R3_DMAChannelsInit( pHandle );
    
    /* Initialize oversampling data structure */
    R3_OversamplingInit(pHandle, pScale);
    
    pHandle->oversampling_count = OVS_COUNT;
    
    /* Convert a quarter period to scalefactor */
    FIXPSCALED_floatToFIXPscaled((float) pHandle->_Super.QuarterPeriodCnt, &pHandle->_Super.duty_sf);
  }
  else
  {
    /* _Super is not the first member of the structure PWMC_R3_Handle_t, so we did no initialization */
  }

}

/**
  * @brief  Configures ADC current reading
  * @param  ADCx ADC peripheral to be configured 
  */
void R3_ADCxInit( ADC_TypeDef * ADCx )
{
  /* Same startup as R3_ADCxInit(), but injected channels removed */

  /* - Exit from deep-power-down mode */
  LL_ADC_DisableDeepPowerDown(ADCx);

  if ( LL_ADC_IsInternalRegulatorEnabled(ADCx) == 0u)
  {
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADCx);

    /* Wait for Regulator Startup time, once for both */
    /* Note: Variable divided by 2 to compensate partially              */
    /*       CPU processing cycles, scaling in us split to not          */
    /*       exceed 32 bits register capacity and handle low frequency. */
    volatile uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));
    while(wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }
  }

  LL_ADC_StartCalibration( ADCx, LL_ADC_SINGLE_ENDED );
  while ( LL_ADC_IsCalibrationOnGoing( ADCx) == 1u)
  {}
  /* ADC Enable (must be done after calibration) */
  /* ADC5-140924: Enabling the ADC by setting ADEN bit soon after polling ADCAL=0
  * following a calibration phase, could have no effect on ADC
  * within certain AHB/ADC clock ratio.
  */
  while (  LL_ADC_IsActiveFlag_ADRDY( ADCx ) == 0u)
  {
    LL_ADC_Enable(  ADCx );
  }

  /* Clear JSQR from CubeMX setting to avoid not wanting conversion*/
  LL_ADC_INJ_StopConversion(ADCx);

  /* Enable DMA on ADC */
  ADCx->CFGR = (ADCx->CFGR & ~(ADC_CFGR_DMAEN | ADC_CFGR_DMACFG)) | ADC_CFGR_DMAEN;

} /* end of R3_ADCxInit() */

/**
  * @brief  Configures Timer for PWM generation and ADC triggering
  * @param  TIMx Timer peripheral used to PWM generation   
  * @param  TIMx_Oversample Timer peripheral used ADC triggering
  * @param  pHdl: handler of the current instance of the PWM component
  */
void R3_TIMxInit( TIM_TypeDef * TIMx, TIM_TypeDef * TIMx_Oversample, PWMC_Handle_t * pHdl )
{

#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  uint32_t Brk2Timeout = 1000;

  /* disable main TIM counter to ensure
  * a synchronous start by TIM2 trigger */
  LL_TIM_DisableCounter( TIMx );
  /* Disable oversample counter as well */
  LL_TIM_DisableCounter( TIMx_Oversample );

  /* Enables the TIMx Preload on CC1 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH1 );
  /* Enables the TIMx Preload on CC2 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
  /* Enables the TIMx Preload on CC3 Register */
  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH3 );
        
  /* Enables the TIMx_Oversample Preload on CC1 Register */
  LL_TIM_OC_EnablePreload( TIMx_Oversample, LL_TIM_CHANNEL_CH1 );

  /* Only 1 motor supported, ratio set using DMA transfer length */
  LL_TIM_SetCounter( TIMx, 0 ); /* Counter reset to 0 */

  LL_TIM_ClearFlag_BRK( TIMx ); /* Not used */

  while ((LL_TIM_IsActiveFlag_BRK2 (TIMx) == 1u) && (Brk2Timeout != 0u) )
    {
      LL_TIM_ClearFlag_BRK2( TIMx );
      Brk2Timeout--;
    }
  LL_TIM_EnableIT_BRK( TIMx );

  /* trigger point to raise an Irq middle of PWM period (for pole pulse) */
  LL_TIM_OC_SetCompareCH4(TIMx, ((uint32_t) pHandle->Half_PWMPeriod - (uint32_t) 20));

  /* counter sart value of the auxiliary timer according to ovsersampling setting */
  if (OVS_COUNT != 1)
  {  
    LL_TIM_SetCounter( TIMx_Oversample, M1_AUX_TIM_PERIOD_CYCLES>>1 );
  }
  else
  {
    LL_TIM_SetCounter( TIMx_Oversample, 0 );
    LL_TIM_ClearFlag_UPDATE(TIMx);
    LL_TIM_EnableIT_UPDATE(TIMx);
  }

  /* initialization of Valid R,S,T flags, all currents should be valid after after boot  */
  R3_SetCurrentReconstructionData(pHdl);
  
} /* end of R3_TIMxInit() */

/**
  * @brief  Configures DMA channel X to transfer the samples from ADC to memory buffer.
  * @param  DMA_ADCx DMA peripheral used for data transfer.
  * @param  adcindex ADC peripheral used for sampling.
  * @param  sequencelength: ADC sequence length.
  */
void R3_DMAChannelxInit( DMA_Channel_TypeDef* DMA_ADCx, ADC_TypeDef* hadc, int adcindex, int sequencelength )
{
  /* Uses direct register access to the DMA channel, because there are no LL functions for DMA channels, only DMA peripherals */
  DMA_Channel_TypeDef * hdmach = DMA_ADCx;

  /* Set peripheral/source address for DMA transfer */
  hdmach->CPAR = (uint32_t) &hadc->DR;

  /* Set target address for DMA transfer */
  hdmach->CMAR = (uint32_t) &oversampling.DMABuffer[0][adcindex][0];
  /* Not critical, will be overwritten later in initialization sequence */

  /* Set number of samples in transfer */
  hdmach->CNDTR = sequencelength * OVS_COUNT * REGULATION_EXECUTION_RATE;

  /* Disable DMA Transfer Complete Interrupt */
  hdmach->CCR &= ~DMA_CCR_TCIE;  /* Only one DMA should generate an interrupt */

  // ToDo Confirm (non-)initialization in LL/HAL
  // It seems the generated code does not set these DMA configuration items
  // MSIZE_0 = 16-bit
  // PSIZE_0 = 16-bit
  // MINC = increment memory
  // CIRC = circular mode
  hdmach->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC;

  /* Enable DMA */
  hdmach->CCR |= DMA_CCR_EN;

} /* end of R3_DMAChannelxInit() */

/**
  * @brief  Configures all DMA channels to transfer the samples from all used ADCs to memory buffer.
  * @param  pHandle: handler of the current instance of the PWM component
  */
static void R3_DMAChannelsInit( PWMC_R3_Handle_t* pHandle )
{
  DMA_Channel_TypeDef * DMA_ADCx_1 = pHandle->pParams_str->DMA_ADCx_1;
  DMA_Channel_TypeDef * DMA_ADCx_2 = pHandle->pParams_str->DMA_ADCx_2;
<#if MC.M1_CS_ADC_NUM == "3">   
  DMA_Channel_TypeDef * DMA_ADCx_3 = pHandle->pParams_str->DMA_ADCx_3;
  ADC_TypeDef * ADCx_3 = pHandle->pParams_str->ADCx_3;    
</#if>
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  R3_DMAChannelxInit( DMA_ADCx_1, ADCx_1, 0, OVS_${MC.M1_CS_ADC_U}_TASKLENGTH );
  R3_DMAChannelxInit( DMA_ADCx_2, ADCx_2, 1, OVS_<#if MC.M1_CS_ADC_U == MC.M1_CS_ADC_V>${MC.M1_CS_ADC_W}<#else>${MC.M1_CS_ADC_V}</#if>_TASKLENGTH );
<#if MC.M1_CS_ADC_NUM == "3">   
  R3_DMAChannelxInit( DMA_ADCx_3, ADCx_3, 2, OVS_${MC.M1_CS_ADC_W}_TASKLENGTH );  
</#if>

  /* Enable DMA Transfer Complete Interrupt */
  /* All the ADCs are forced to the same length then enabling DMA_ADCx_1 Irq is enough  */
  DMA_ADCx_1->CCR |= DMA_CCR_TCIE;

} 

/**
  * @brief  Configures oversampling data structure.
  * @param  scaleFlashParams_t: address of the structure which stores the scaling parameters
  */
static void R3_OversamplingInit( PWMC_R3_Handle_t* pHandle, const scaleFlashParams_t* pScale )
{
  /* This function initializes the oversampling data structure
   * This determines which ADC channel is related to which oversampling channel
   */
  
  Oversampling_t* pOvs = &oversampling;
  
  /* Initialization of data-driven method of gathering the samples */
  
  /* count is the number of samples taken in a single PWM period */
  pOvs->count = OVS_COUNT;
  
  /* divider is the number of PWM periods per motor control interrupt */
  pOvs->divider = REGULATION_EXECUTION_RATE;
  
  /* Set DMA buffering to a logical starting point */
  pOvs->writeBuffer = 0;
  R3_SetDmaTargetBuffer(pHandle, 0);  /* New samples will be stored in buffer 0 */
  pOvs->readBuffer = 1;                  /* while this cycle we will read from buffer 1 */
  
  pOvs->current_factor = OVERSAMPLING_FIXP_CURRENT_FACTOR(pScale->current);
  pOvs->voltage_factor = OVERSAMPLING_FIXP_VOLTAGE_FACTOR(pScale->voltage);
  pOvs->busvoltage_factor = OVERSAMPLING_FIXP_BUSVOLTAGE_FACTOR(pScale->voltage);

} /* end of R3_OversamplingInit() */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  Configures ADC sampling instant.
  *         Computes sampling instant according to phase A,B and duties,
  *         and timing constraint like deatime, Tnoise, Trise and sampling time.
  *         the result will be then applied thanks to R3_WriteTIMRegisters().
  *         This routine has to be called at the end the current controller execution.
  * @param  pHdl handler of the current instance of the PWM component
  * @retval Error status
  */
uint16_t R3_SetADCSampPointSectX( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */

  uint16_t SamplingPoint;
<#if MC.M1_CURRENT_SENSING_TOPO != "ICS_SENSORS" && MC.M1_CURRENT_SENSING_TOPO != "THREE_SHUNT">
  uint16_t mid;
  uint16_t min;
  uint16_t delta;

  if(pHandle->_Super.CntPhA > pHandle->_Super.CntPhB)
  {
    if (pHandle->_Super.CntPhA > pHandle->_Super.CntPhC)
    {
      // A>B && A>C
      if (pHandle->_Super.CntPhB > pHandle->_Super.CntPhC)
      {
        // A>B && A>C && B>C
        mid = pHandle->_Super.CntPhB;
        min = pHandle->_Super.CntPhA;
      }
      else
      {
        // A>B && A>C && C>B
        mid = pHandle->_Super.CntPhC;
        min = pHandle->_Super.CntPhA;
      }
    }
    else // A>B && C>A
    {
      mid = pHandle->_Super.CntPhA;
      min = pHandle->_Super.CntPhC;
    }
  }
  else // B>A
  {
    if (pHandle->_Super.CntPhB > pHandle->_Super.CntPhC)
    {
      // B>A  && B>C
      if (pHandle->_Super.CntPhA > pHandle->_Super.CntPhC)
      {
        // B>A  && B>C && A>C 
        mid = pHandle->_Super.CntPhA;
        min = pHandle->_Super.CntPhB;
      }
      else
      {
        // B>A  && B>C && C>A
        mid = pHandle->_Super.CntPhC;
        min = pHandle->_Super.CntPhB;
      }
    }
    else // B>A && C>B
    {
        mid = pHandle->_Super.CntPhB;
        min = pHandle->_Super.CntPhC;
    }
  }
 
  delta = pHandle->Half_PWMPeriod - min;
  if (delta > DT_TN)
  {
    SamplingPoint =(uint16_t) MIDDLE;
  }
  else
  {
    if ((delta > DT_TN_TS/2) && (delta <= DT_TN_TS))/*((min - mid) < DT_TR_TS))*/
    {
      SamplingPoint = (uint16_t) MIDDLE + delta - SAMPLE_OFFSET_TICKS;
    }
    else
    {
      if  ((delta <= DT_TN_TS/2) && ((min - mid) >= DT_TR_TS))
      {
        SamplingPoint = (uint16_t) MIDDLE - delta - SAMPLE_OFFSET_TICKS;
      }
      else
      {
        SamplingPoint = (uint16_t) MIDDLE;
      }
    }
  }
<#else> /* CUSTOM_SAMPLING_DELAY */
  /* set sampling  point trigger so that a current sample is taken in the middle of PWM period */
  SamplingPoint =(uint16_t) MIDDLE;
</#if>

  return R3_WriteTIMRegisters( &pHandle->_Super, SamplingPoint );
}


#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  Write new duty cycles and sampling point into peripheral registers.
  * @param  pHandle handler of the current instance of the PWM component
  * @param  SamplingPoint new sampling point value.
  * @retval Error status
  */
__STATIC_INLINE uint16_t R3_WriteTIMRegisters( PWMC_Handle_t * pHdl, uint16_t SamplingPoint )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
TIM_TypeDef * TIMx_Oversample = pHandle->pParams_str->TIMx_Oversample;
  uint16_t Aux;

  /* Previous overwriting previous CC, save off validitiy for reconstruction */
  R3_SetCurrentReconstructionData( pHdl );

  if (pHandle->isShorting)
  {
    /* Force fixed duty on all channels, shorting the motor windings */  
    LL_TIM_OC_SetCompareCH1 ( TIMx, 0UL );
    LL_TIM_OC_SetCompareCH2 ( TIMx, 0UL );
    LL_TIM_OC_SetCompareCH3 ( TIMx, 0UL );
    SamplingPoint = MIDDLE;
  }
  else if (pHandle->isMeasuringOffset)
  {
    /* Force 50% duty on all channels, giving 0V difference, zero current */
    uint16_t ticksFor50PercentDuty = pHandle->Half_PWMPeriod >> 1;
    LL_TIM_OC_SetCompareCH1 ( TIMx, ticksFor50PercentDuty );
    LL_TIM_OC_SetCompareCH2 ( TIMx, ticksFor50PercentDuty );
    LL_TIM_OC_SetCompareCH3 ( TIMx, ticksFor50PercentDuty );
    SamplingPoint = MIDDLE;
  }
  else
  {
    LL_TIM_OC_SetCompareCH1 ( TIMx, (uint32_t) pHandle->_Super.CntPhA );
    LL_TIM_OC_SetCompareCH2 ( TIMx, (uint32_t) pHandle->_Super.CntPhB );
    LL_TIM_OC_SetCompareCH3 ( TIMx, (uint32_t) pHandle->_Super.CntPhC );
  }

  LL_TIM_OC_SetCompareCH1( TIMx_Oversample, (uint32_t) SamplingPoint );
  /* Limit for update event */

  if (LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 1UL)
  {
    Aux = MC_DURATION;
  }
  else
  {
    Aux = MC_NO_FAULTS;
  }
  return Aux;
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers.
  * @param  pHdl: handler of the current instance of the PWM component
  * @param  ticks: Timer ticks value to be applied.
  *                Min value: 0 (low sides ON)
  *                Max value: PWM_PERIOD_CYCLES/2 (low sides OFF)  
  */
__weak void R3_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks)
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* Clear shorting mode */
  pHandle->isShorting = false;

  pHandle->_Super.TurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1( TIMx, ticks );
  LL_TIM_OC_SetCompareCH2( TIMx, ticks );
  LL_TIM_OC_SetCompareCH3( TIMx, ticks );

  /* Wait until next update */
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0u )
  {}

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);    

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
  }
  return;
}

/**
  * @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
  *         bit
  * @param  pHdl: handler of the current instance of the PWM component
  */
__weak void R3_SwitchOnPWM_Internal( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = false;

  // Do not update duty when switching on PWM, to allow flying start

  /* Main PWM Output Enable */  
  LL_TIM_EnableAllOutputs ( TIMx );
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);  

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    if ( ( TIMx->CCER & TIMxCCER_MASK_CH123 ) != 0u )
    {
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
  }
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

}

/**
  * @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
  *         bit
  * @param  pHdl: handler of the current instance of the PWM component
  */
__weak void R3_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;

  R3_SwitchOnPWM_Internal(pHdl);

  /*  */
  pHandle->isShorting = false;
}

/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  * @param  pHdl: handler of the current instance of the PWM component
  */
__weak void R3_SwitchOffPWM_Internal( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = false;
  
  /* Main PWM Output Disable */
  LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);    
  LL_TIM_DisableAllOutputs( TIMx );

  if ( pHandle->_Super.BrakeActionLock == true )
  {
  }
  else
  {
    if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
  }
}

/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  * @param  pHdl: handler of the current instance of the PWM component
  */
__weak void R3_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;

  R3_SwitchOffPWM_Internal(pHdl);

  /*  */
  pHandle->isShorting = false;
}


/**
  * @brief  It contains the TIMx Update event interrupt
  * @param  pHandle: handler of the current instance of the PWM component
  */
__weak void * R3_TIMx_UP_IRQHandler( PWMC_R3_Handle_t * pHandle )
{
  return &( pHandle->_Super.Motor );
}

/**
  * @brief  Sets DMA target buffer for all used DMA.
  * @param  bufferIndex index of the destination buffer 
  */
static void R3_SetDmaTargetBuffer(PWMC_R3_Handle_t * pHandle, int bufferIndex)
{
  /* Set DMA target memory addresses to indicated buffer. This is done while DMA channel is disabled
   * This is done immediately after a DMA transfer completes, well before the next oversampling trigger */

  DMA_Channel_TypeDef * DMA_ADCx_1 = pHandle->pParams_str->DMA_ADCx_1;
  DMA_Channel_TypeDef * DMA_ADCx_2 = pHandle->pParams_str->DMA_ADCx_2;
<#if MC.M1_CS_ADC_NUM == "3"> 
  DMA_Channel_TypeDef * DMA_ADCx_3 = pHandle->pParams_str->DMA_ADCx_3;
</#if>

  // Disable before updating the target memory address, then re-enable

  // May only be called when we are certain we will not miss DMA transfers

  CLEAR_BIT(DMA_ADCx_1->CCR, DMA_CCR_EN);
  DMA_ADCx_1->CMAR = (uint32_t) oversampling.DMABuffer[bufferIndex][0];
  SET_BIT(DMA_ADCx_1->CCR, DMA_CCR_EN);

  CLEAR_BIT(DMA_ADCx_2->CCR, DMA_CCR_EN);
  DMA_ADCx_2->CMAR = (uint32_t) oversampling.DMABuffer[bufferIndex][1];
  SET_BIT(DMA_ADCx_2->CCR, DMA_CCR_EN);
<#if MC.M1_CS_ADC_NUM == "3"> 
  CLEAR_BIT(DMA_ADCx_3->CCR, DMA_CCR_EN);
  DMA_ADCx_3->CMAR = (uint32_t) oversampling.DMABuffer[bufferIndex][2];
  SET_BIT(DMA_ADCx_3->CCR, DMA_CCR_EN);  
</#if>

} /* end of R3_SetDmaTargetBuffer() */

/**
  * @brief  Swaps DMA target buffer for all DMA involved in current and voltage reading.
  *         This routine is called when DMA transfer is complete in order to handle
  *         double buffering mechanism.  
  * @param  bufferIndex index of the destination buffer 
  */
void R3_SwapDmaBuffer( PWMC_R3_Handle_t * pHandle )
{
  /* Swap the sampling DMA buffer (the one the DMA is writing to) and the reading DMA buffer (the one we are reading from)
   * This is done at the start of the motor control interrupt, before processing the data, and should be finished
   * well before the next ADC sample is taken
   */

  // The last reading buffer becomes the new sampling buffer
  int samplingDmaBufferIndex = oversampling.writeBuffer;
  oversampling.writeBuffer = oversampling.readBuffer;
  oversampling.readBuffer = samplingDmaBufferIndex;
  
  // Set the DMA to point to the new sampling buffer
  R3_SetDmaTargetBuffer(pHandle, oversampling.writeBuffer);

} /* end of R3_SwapDmaBuffer() */

/**
  * @brief  Gets phase voltage and currents measurements 
  *         This routine is called at the beginning of the current controller.
  * @param  pCurrents_Irst measured phase currents
  * @param  pVoltages_Urst measured phase voltages
  */
void R3_GetMeasurements(PWMC_Handle_t * pHdl, Currents_Irst_t* pCurrents_Irst, Voltages_Urst_t* pVoltages_Urst)
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  Currents_Irst_t Irst_in_pu;
  Voltages_Urst_t Urst_in_pu;

  OVERSAMPLING_getMeasurements(&Irst_in_pu, &Urst_in_pu);
  
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  if (!pHandle->isMeasuringOffset)
  {
    pCurrents_Irst->R = Irst_in_pu.R - pHandle->Offset_I_R;
    pCurrents_Irst->S = Irst_in_pu.S - pHandle->Offset_I_S;
    pCurrents_Irst->T = Irst_in_pu.T - pHandle->Offset_I_T;
    pVoltages_Urst->R = Urst_in_pu.R - pHandle->Offset_U_R;
    pVoltages_Urst->S = Urst_in_pu.S - pHandle->Offset_U_S;
    pVoltages_Urst->T = Urst_in_pu.T - pHandle->Offset_U_T;
  }
  else
  {
    fixp31_t filtconst = pHandle->wPhaseOffsetFilterConst;
    pHandle->Offset_I_R += FIXP31_mpy((Irst_in_pu.R - pHandle->Offset_I_R), filtconst);
    pHandle->Offset_I_S += FIXP31_mpy((Irst_in_pu.S - pHandle->Offset_I_S), filtconst);
    pHandle->Offset_I_T += FIXP31_mpy((Irst_in_pu.T - pHandle->Offset_I_T), filtconst);
    pHandle->Offset_U_R += FIXP31_mpy((Urst_in_pu.R - pHandle->Offset_U_R), filtconst);
    pHandle->Offset_U_S += FIXP31_mpy((Urst_in_pu.S - pHandle->Offset_U_S), filtconst);
    pHandle->Offset_U_T += FIXP31_mpy((Urst_in_pu.T - pHandle->Offset_U_T), filtconst);

    pCurrents_Irst->R = 0;
    pCurrents_Irst->S = 0;
    pCurrents_Irst->T = 0;
    pVoltages_Urst->R = 0;
    pVoltages_Urst->S = 0;
    pVoltages_Urst->T = 0;
  }

  return;
} /* end of R3_GetMeasurements() */

/**
  * @brief  Gets DC bus measurements 
  *         This routine is called at the beginning of the current controller.
  * @param  pBusVoltage measured DC bus
  */
void R3_GetVbusMeasurements(PWMC_Handle_t * pHdl, fixp30_t* pBusVoltage)
{
  OVERSAMPLING_getVbusMeasurements(pBusVoltage);
  return;
} /* end of R3_GetMeasurements() */

/**
  * @brief  Gets auxiliary measurements.
  *         In this case, auxiliary measurements is related to
  *         potentiometer/throttle
  * @param  pAuxMeasurement auxiliary measurement
  */
void R3_GetAuxAdcMeasurement( PWMC_Handle_t * pHdl, fixp30_t* pAuxMeasurement )
{
<#if MC.M1_POTENTIOMETER_ENABLE == true >
  <#if (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_U) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_V) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_W)>
  int32_t sample_Aux;
  OVERSAMPLING_getPotentiometerMeasurements(&sample_Aux);
  *pAuxMeasurement = FIXP_MPY(sample_Aux, FIXP30(1.0/(OVS_COUNT * REGULATION_EXECUTION_RATE)), ADC_FIXPFMT);
  <#else>
  /* No Aux */
  *pAuxMeasurement = 0;
  </#if><#-- (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_U) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_V) || (MC.POTENTIOMETER_ADC ==  MC.M1_CS_ADC_W) -->
<#else>
  /* No Aux */
  *pAuxMeasurement = 0;
</#if>
  return;
} /* end of R3_GetAuxAdcMeasurement() */

/**
  * @brief  Determines the number of valid currents and their validity status
  *         In this case, auxiliary measurements is related to
  *         potentiometer/throttle
  * @param  pHdl: handler of the current instance of the PWM component
  */
void R3_SetCurrentReconstructionData( PWMC_Handle_t * pHdl )
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  uint32_t maxduty = pHandle->maxTicksCurrentReconstruction;
  uint8_t numValid = 0;

  pHandle->reconPrevious.validR = (LL_TIM_OC_GetCompareCH1(TIMx) <= maxduty)? true:false;
  if (pHandle->reconPrevious.validR) { numValid++; }
  
  pHandle->reconPrevious.validS = (LL_TIM_OC_GetCompareCH2(TIMx) <= maxduty)? true:false;
  if (pHandle->reconPrevious.validS) { numValid++; }

  pHandle->reconPrevious.validT = (LL_TIM_OC_GetCompareCH3(TIMx) <= maxduty)? true:false;
  if (pHandle->reconPrevious.validT) { numValid++; }
  
  pHandle->reconPrevious.numValid = numValid;
}

/**
  * @brief  Exports the number of valid currents and their validity status
  * @param  pHdl: handler of the current instance of the PWM component
  * @param  pData: data structure will store the reconstruction status
  * @retval number of valid current
  */
uint8_t R3_GetCurrentReconstructionData( PWMC_Handle_t * pHdl, CurrentReconstruction_t * pData )
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;

  *pData = pHandle->reconPrevious;

  return pData->numValid;
} /* end of R3_GetCurrentReconstructionData() */

/**
  * @brief  Gets phase voltage and currents offset measurements.
  *         In this case, auxiliary measurements is related to
  *         potentiometer/throttle
  * @param  pHdl: handler of the current instance of the PWM component
  * @param  PolarizationOffsets phase current and voltage measured offsets
  */
void R3_GetOffsets( PWMC_Handle_t * pHdl, PolarizationOffsets_t * PolarizationOffsets )
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;

  PolarizationOffsets->CurrentOffsets.R = pHandle->Offset_I_R;
  PolarizationOffsets->CurrentOffsets.S = pHandle->Offset_I_S;
  PolarizationOffsets->CurrentOffsets.T = pHandle->Offset_I_T;

  PolarizationOffsets->VoltageOffsets.R = pHandle->Offset_U_R;
  PolarizationOffsets->VoltageOffsets.S = pHandle->Offset_U_S;
  PolarizationOffsets->VoltageOffsets.T = pHandle->Offset_U_T;

  return;
} /* end of R3_GetOffsets() */

/**
  * @brief  Sets phase voltage and currents offset measurements.
  * @param  pHdl: handler of the current instance of the PWM component
  * @param  PolarizationOffsets phase current and voltage offsets
  *         to be set
  */
void R3_SetOffsets( PWMC_Handle_t * pHdl, PolarizationOffsets_t * PolarizationOffsets )
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;

  pHandle->Offset_I_R = PolarizationOffsets->CurrentOffsets.R;
  pHandle->Offset_I_S = PolarizationOffsets->CurrentOffsets.S;
  pHandle->Offset_I_T = PolarizationOffsets->CurrentOffsets.T;

  pHandle->Offset_U_R = PolarizationOffsets->VoltageOffsets.R;
  pHandle->Offset_U_S = PolarizationOffsets->VoltageOffsets.S;
  pHandle->Offset_U_T = PolarizationOffsets->VoltageOffsets.T;

  /* To skip offsets measurements */
  pHandle->isMeasuringOffset = false;

  return;
} /* end of R3_SetOffsets() */

/**
  * @brief  Starts the offset calibration process
  *         Set hardware to offset calibration mode:
  *         - PWM enabled and set to 50% duty, so the voltage on all three phases is half the bus voltage
  *         - Do not obey whatever duty is being written, just keep at 50%
  *         - Clear the offsets, set to zero
  *         - Measure/filter the sampled values, to be used at the end of the offset measurement
  * @param  pHdl: handler of the current instance of the PWM component
  */
void R3_StartOffsetCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;

  pHandle->Offset_I_R = 0;
  pHandle->Offset_I_S = 0;
  pHandle->Offset_I_T = 0;
  pHandle->Offset_U_R = 0;
  pHandle->Offset_U_S = 0;
  pHandle->Offset_U_T = 0;

  /* Set sampling mode to Offset mode. This implies PWM duty is held at 50% and
   * the read-samples function filters the signal into the offset variables */
  pHandle->isMeasuringOffset = true;

  return;
} /* end of R3_StartOffsetCalibration() */

/**
  * @brief  Declares end of offset calibration process
  *         Since the offset variables contain a filtered version of the measured signals,
  *         and the offset will be subtracted from the measured signal from now on, this
  *         function does not need to do any calculations.
  * @param  pHdl: handler of the current instance of the PWM component
  */
void R3_FinishOffsetCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;

  /* Sampling mode back to normal. Normal PWM function, normal sample reading */
  pHandle->isMeasuringOffset = false;

  return;
} /* end of R3_FinishOffsetCalibration() */

/**
  * @brief  Prepares the PWM for offset calibration.
  *         PWM duties are set to 50% and all PWM channels are enabled.
  * @param  pHdl: handler of the current instance of the PWM component
  */
void R3_PreparePWM( PWMC_Handle_t * pHdl )
{
  /* Critical PWM setup from R3_CurrentReadingPolarization() */

  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  // Set initial duty in PreparePWM, as this is no longer done in SwitchOnPWM

  /* Set all duty to 50% */
  LL_TIM_OC_SetCompareCH1(TIMx, ((uint32_t) pHandle->Half_PWMPeriod / (uint32_t) 2));
  LL_TIM_OC_SetCompareCH2(TIMx, ((uint32_t) pHandle->Half_PWMPeriod / (uint32_t) 2));
  LL_TIM_OC_SetCompareCH3(TIMx, ((uint32_t) pHandle->Half_PWMPeriod / (uint32_t) 2));

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);

} /* end of R3_PreparePWM() */

/**
  * @brief  Shorts the motor by forcing 0% duty on all phases
  * @param  pHdl: handler of the current instance of the PWM component
  */
void R3_PwmShort( PWMC_Handle_t * pHdl )
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
  /* Set PWM to shorted mode */
  pHandle->isShorting = true;

  // Write the shorted duty to the timer compare registers
  R3_WriteTIMRegisters(pHdl, (uint16_t) MIDDLE);

  R3_SwitchOnPWM_Internal(pHdl);
} /* end of R3_PwmShort() */

/**
  * @brief  Checks if PWM are in control
  * @retval PWM control status. 
  *         - true: PWM are controlled 
  *      - false: PWM are not controlled. This happens when the timer Main Output is disabled,
  *         when the phases are shorted or when offset calibration is on going        
  */
bool R3_IsInPwmControl( PWMC_Handle_t * pHdl )
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  bool retval = false;
      
  // PWM output must be enabled
  if (!LL_TIM_IsEnabledAllOutputs(TIMx) || (pHandle->isShorting == true) || (pHandle->isMeasuringOffset == true)) 
    {
      retval = false;
    }
    else
    {
      retval = true;
    }
    
  return retval;
}

/**
  * @brief  Gets Main Output Enable status of the timer that manages 
  *         the PWM generation  
  * @param  pHdl: handler of the current instance of the PWM component
  */
bool R3_IsPwmEnabled( PWMC_Handle_t * pHdl )
{
  PWMC_R3_Handle_t * pHandle = ( PWMC_R3_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  return LL_TIM_IsEnabledAllOutputs(TIMx);
}

<#if MC.M1_OCP_TOPOLOGY == "EMBEDDED">
/*
  * @brief  Configures the analog output used for protection thresholds.
  *
  * @param  DAC_Channel: the selected DAC channel.
  *          This parameter can be:
  *            @arg DAC_Channel_1: DAC Channel1 selected.
  *            @arg DAC_Channel_2: DAC Channel2 selected.
  * @param  DACx: DAC to be configured.
  * @param  hDACVref: Value of DAC reference expressed as 16bit unsigned integer.
  *         Ex. 0 = 0V 65536 = VDD_DAC.
  */
static void R3_SetAOReferenceVoltage(uint32_t DAC_Channel, DAC_TypeDef *DACx, uint16_t hDACVref)
{
  LL_DAC_ConvertData12LeftAligned(DACx, DAC_Channel, hDACVref);

  /* Enable DAC Channel */
  LL_DAC_TrigSWConversion(DACx, DAC_Channel);

  if (1U == LL_DAC_IsEnabled(DACx, DAC_Channel))
  {
    /* If DAC is already enable, we wait LL_DAC_DELAY_VOLTAGE_SETTLING_US */
    volatile uint32_t wait_loop_index = ((LL_DAC_DELAY_VOLTAGE_SETTLING_US) * (SystemCoreClock / (1000000UL * 2UL)));
    while (wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }
  }
  else
  {
    /* If DAC is not enabled, we must wait LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US */
    LL_DAC_Enable(DACx, DAC_Channel);
    volatile uint32_t wait_loop_index = ((LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US)
                                         * (SystemCoreClock / (1000000UL * 2UL)));
    while (wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }
  }
}
</#if>

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
