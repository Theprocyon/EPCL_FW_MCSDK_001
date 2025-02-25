<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
/**
  ******************************************************************************
  * @file    r1_ps_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r1_ps_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup r1_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef R1_PS_PWMCURRFDBK_H
#define R1_PS_PWMCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup r1_ps_pwm_curr_fdbk
  * @{
  */

/* Exported constants --------------------------------------------------------*/
#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))
#define DAC_MODE  ((uint8_t)(0x03))

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  Parameters structure of the r1_ps_pwm_curr_fdbk Component.
 *
 */
typedef struct
{
  /* HW IP involved -----------------------------*/
  ADC_TypeDef * ADCx;            /*!< First ADC peripheral to be used.*/
  TIM_TypeDef * TIMx;              /*!< timer used for PWM generation.*/
  DMA_TypeDef * DMAx;              /*!< timer used for PWM generation.*/  
<#if MC.M1_USE_INTERNAL_OPAMP == true || MC.M2_USE_INTERNAL_OPAMP == true>  
  OPAMP_TypeDef * OPAMP_Selection;
</#if>  
<#if MC.M1_OCP_TOPOLOGY == "EMBEDDED" || MC.M2_OCP_TOPOLOGY == "EMBEDDED">
  COMP_TypeDef * CompOCPSelection; /*!< Internal comparator used for Phases protection.*/
</#if>  
<#if MC.INTERNAL_OVERVOLTAGEPROTECTION == true || MC.INTERNAL_OVERVOLTAGEPROTECTION2 == true>   
  COMP_TypeDef * CompOVPSelection;  /*!< Internal comparator used for Over Voltage protection.*/
</#if>

  uint32_t DMAChannelX;            /*!< DMA channel used to modify CCR on the fly */                          
  uint32_t DMASamplingPtChannelX;  /*!< DMA channel used to modify sampling point on the fly */  
  uint32_t AdcExtTrigger;          /*!< ADC trigger */   
<#if HAS_ADC_INJ == false>
  uint32_t DMA_ADC_DR_ChannelX;    /*!< DMA channel used to transfer ADC data to memory buffer */  
</#if>

 /* PWM generation parameters --------------------------------------------------*/

  uint16_t TMin;                   
  uint16_t TSample;  
  uint16_t hTADConv;
 
  /* DAC settings --------------------------------------------------------------*/
<#if MC.M1_OCP_TOPOLOGY == "EMBEDDED" || MC.M2_OCP_TOPOLOGY == "EMBEDDED">  
  uint16_t DAC_OCP_Threshold;        /*!< Value of analog reference expressed
                                           as 16bit unsigned integer.
                                           Ex. 0 = 0V 65536 = VDD_DAC.*/
</#if>                                           
<#if MC.INTERNAL_OVERVOLTAGEPROTECTION == true || MC.INTERNAL_OVERVOLTAGEPROTECTION2 == true>                                            
  uint16_t DAC_OVP_Threshold;        /*!< Value of analog reference expressed
                                           as 16bit unsigned integer.
                                           Ex. 0 = 0V 65536 = VDD_DAC.*/   
</#if>                                           
  /* PWM Driving signals initialization ----------------------------------------*/
  uint8_t  IChannel;                                                
<#if CondFamily_STM32F0 || CondFamily_STM32G0  >
  uint8_t ISamplingTime;
</#if>  
  uint8_t  RepetitionCounter;         /*!< It expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again. In
                                            particular:
                                            RepetitionCounter= (2* #PWM periods)-1*/
                                           
  /* Internal COMP settings ----------------------------------------------------*/      
<#if MC.M1_OCP_TOPOLOGY == "EMBEDDED" || MC.M2_OCP_TOPOLOGY == "EMBEDDED"> 
  uint8_t       CompOCPInvInput_MODE;    /*!< COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */         
</#if>                                                                   
<#if MC.INTERNAL_OVERVOLTAGEPROTECTION == true || MC.INTERNAL_OVERVOLTAGEPROTECTION2 == true> 
  uint8_t       CompOVPInvInput_MODE;     /*!< COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
</#if>  
  /* Dual MC parameters --------------------------------------------------------*/
  uint8_t  FreqRatio;             /*!< It is used in case of dual MC to
                                        synchronize TIM1 and TIM8. It has
                                        effect only on the second instanced
                                        object and must be equal to the
                                        ratio between the two PWM frequencies
                                        (higher/lower). Supported values are
                                        1, 2 or 3 */
  uint8_t  IsHigherFreqTim;       /*!< When bFreqRatio is greather than 1
                                        this param is used to indicate if this
                                        instance is the one with the highest
                                        frequency. Allowed value are: HIGHER_FREQ
                                        or LOWER_FREQ */                                           

} R1_Params_t;

/**
  * @brief  Handle structure of the r1_ps_pwm_curr_fdbk Component
  */
typedef struct
{
  PWMC_Handle_t _Super;            /*!< Offset of current sensing network  */
  uint16_t DmaBuffCCR[6];          /*!< Buffer used for PWM phase shift points */
  uint16_t DmaBuffCCR_latch[6];    /*!< Buffer used to latch PWM phase shift points */  
  uint32_t PhaseOffset;            /*!< Offset of Phase current sensing network  */ 
  uint32_t AdcExtTrigger;          /*!< external ADC trigger */
  uint16_t aShiftval[3];           /*!< shift value to be applied  */               
  uint16_t DmaBuffCCR_ADCTrig[3];  /*!< Buffer used to store sampling point values */
<#if HAS_ADC_INJ == false>
  uint16_t CurConv[2];             /*!< Buffer used to store sampled currents */
</#if>
  uint16_t Half_PWMPeriod;     /* Half PWM Period in timer clock counts */
  uint16_t CntSmp1;            /*!< First sampling point express in timer counts*/
  uint16_t CntSmp2;            /*!< Second sampling point express in timer counts*/
  uint8_t sampCur1;            /*!< Current sampled in the first sampling point*/
  uint8_t sampCur2;            /*!< Current sampled in the second sampling point*/
  int16_t CurrAOld;            /*!< Previous measured value of phase A current*/
  int16_t CurrBOld;            /*!< Previous measured value of phase B current*/
  volatile uint8_t  Index;     /*!< Number of conversions performed during the
                                   calibration phase*/
  uint8_t iflag;
  uint8_t TCCnt;    

  bool UpdateFlagBuffer;       /*!< buffered version of Timer update IT flag */  
  bool FOCDurationFlag;        /*!< This flag is used to detect FOC duration error.*/
  bool TCDoneFlag;             /*!< This flag is used to indicate that last DMA TC of the period is done.*/
<#if HAS_ADC_INJ == false>
  bool ADCRegularLocked;
</#if>
  R1_Params_t const * pParams_str;

} PWMC_R1_Handle_t;

/**
  * It performs the initialization of the MCU peripherals required for
  * the PWM generation and current sensing. this initialization is dedicated
  * to one shunt topology and F3 family
  */
void R1_Init( PWMC_R1_Handle_t * pHandle );

/**
  * It disables PWM generation on the proper Timer peripheral acting on
  * MOE bit
  */
void R1_SwitchOffPWM( PWMC_Handle_t * pHdl );

/**
  * It enables PWM generation on the proper Timer peripheral acting on MOE
  * bit
  */
void R1_SwitchOnPWM( PWMC_Handle_t * pHdl );

/**
  * It turns on low sides switches. This function is intended to be
  * used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers
  */
void R1_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks );

/**
  * It computes and return latest converted motor phase currents motor
  */
void R1_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );

/**
  * It contains the TIMx Break2 event interrupt
  */
void * R1_BRK2_IRQHandler( PWMC_R1_Handle_t * pHdl );

/**
  * It contains the TIMx Break1 event interrupt
  */
void * R1_BRK_IRQHandler( PWMC_R1_Handle_t * pHdl );

/**
  * It stores into pHandle the offset voltage read onchannels when no
  * current is flowing into the motor
  */
void R1_CurrentReadingCalibration( PWMC_Handle_t * pHdl );

/**
  * Implementation of the single shunt algorithm to setup the
  * TIM1 register and DMA buffers values for the next PWM period.
  */
uint16_t R1_CalcDutyCycles( PWMC_Handle_t * pHdl );

/**
  * This function handles motor DMAx TC interrupt request. 
  */
void *R1_DMAx_TC_IRQHandler( PWMC_R1_Handle_t * pHandle );

/**
  * This function handles motor DMAx HT interrupt request. 
  */
void *R1_DMAx_HT_IRQHandler( PWMC_R1_Handle_t * pHandle );

/**
  * It contains the TIM1 Update event interrupt
  */
void * R1_TIM1_UP_IRQHandler( PWMC_R1_Handle_t * pHandle );

/**
  * It contains the TIM8 Update event interrupt
  */
void * R1_TIM8_UP_IRQHandler( PWMC_R1_Handle_t * pHandle );

/**
  * It sets the calibrated offset. In single, only Phase A member is used
  * to set the offset.  
  */
void R1_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/**
  * It reads the calibrated offsets.In single, offset is written
  * in phase A member.  
  */
void R1_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__R1_PS_F30X_PWMCURRFDBK_H*/

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
