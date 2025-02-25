<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    r3_g4xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          R3_G4XX_pwm_curr_fdbk component of the Motor Control SDK.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef R3_G4XX_PWMNCURRFDBK_H
#define R3_G4XX_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"
#include "flash_parameters.h"

/* Exported defines --------------------------------------------------------*/
#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t) 1)
#define NO_SHIFTED_TIMs   ((uint8_t) 0)

#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup R3_G4XX_pwm_curr_fdbk
  * @{
  */

/* Exported types ------------------------------------------------------- */

/**
  * @brief  R3_G4XX_pwm_curr_fdbk component OPAMP parameters definition
  */
typedef const struct
{
  /* First OPAMP settings ------------------------------------------------------*/
   OPAMP_TypeDef * OPAMPx_1; /*!< @brief OPAMP used for phase A */
   OPAMP_TypeDef * OPAMPx_2; /*!< @brief OPAMP used for phase B */
   OPAMP_TypeDef * OPAMPx_3; /*!< @brief OPAMP used for phase C */
} R3_3_OPAMPParams_t;

/**
  * @brief  R3_G4XX_pwm_curr_fdbk component parameters definition
  */
typedef const struct
{
  /* HW IP involved -----------------------------*/
  ADC_TypeDef * ADCx_1;            /*!< @brief First ADC peripheral to be used.*/
  ADC_TypeDef * ADCx_2;            /*!< @brief Second ADC peripheral to be used.*/
  ADC_TypeDef * ADCx_3;            /*!< @brief third ADC peripheral to be used.*/
  TIM_TypeDef * TIMx;              /*!< @brief timer used for PWM generation.*/
  TIM_TypeDef * TIMx_Oversample;   /*!< @brief timer used for oversampling generation.*/
  DMA_Channel_TypeDef * DMA_ADCx_1;
  DMA_Channel_TypeDef * DMA_ADCx_2;
  DMA_Channel_TypeDef * DMA_ADCx_3;
  R3_3_OPAMPParams_t * OPAMPParams;  /*!< @brief Pointer to the OPAMP params struct.
                                       It must be MC_NULL if internal OPAMP are not used.*/
  COMP_TypeDef * CompOCPASelection; /*!< @brief Internal comparator used for Phase A protection.*/
  COMP_TypeDef * CompOCPBSelection; /*!< @brief Internal comparator used for Phase B protection.*/
  COMP_TypeDef * CompOCPCSelection; /*!< @brief Internal comparator used for Phase C protection.*/
  COMP_TypeDef * CompOVPSelection;  /*!< @brief Internal comparator used for Over Voltage protection.*/
  DAC_TypeDef  * DAC_OCP_ASelection; /*!< @brief DAC used for Phase A protection.*/
  DAC_TypeDef  * DAC_OCP_BSelection; /*!< @brief DAC used for Phase B protection.*/
  DAC_TypeDef  * DAC_OCP_CSelection; /*!< @brief DAC used for Phase C protection.*/
  DAC_TypeDef  * DAC_OVP_Selection; /*!< @brief DAC used for Over Voltage protection.*/
  uint32_t DAC_Channel_OCPA;      /*!< @brief DAC channel used for Phase A current protection.*/
  uint32_t DAC_Channel_OCPB;      /*!< @brief DAC channel used for Phase B current protection.*/
  uint32_t DAC_Channel_OCPC;      /*!< @brief DAC channel used for Phase C current protection.*/
  uint32_t DAC_Channel_OVP;       /*!< @brief DAC channel used for Over Voltage protection.*/


 /* PWM generation parameters --------------------------------------------------*/

  /* DAC settings --------------------------------------------------------------*/
  uint16_t DAC_OCP_Threshold;        /*!< @brief Value of analog reference expressed
                                           as 16bit unsigned integer.
                                           Ex. 0 = 0V 65536 = VDD_DAC.*/
  uint16_t DAC_OVP_Threshold;        /*!< @brief Value of analog reference expressed
                                           as 16bit unsigned integer.
                                           Ex. 0 = 0V 65536 = VDD_DAC.*/
  /* PWM Driving signals initialization ----------------------------------------*/


  uint8_t  RepetitionCounter;         /*!< @brief Expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again. In
                                            particular:
                                            RepetitionCounter= (2* #PWM periods)-1*/

  /* Internal COMP settings ----------------------------------------------------*/
  uint8_t       CompOCPAInvInput_MODE;    /*!< @brief COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
  uint8_t       CompOCPBInvInput_MODE;    /*!< @brief COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
  uint8_t       CompOCPCInvInput_MODE;    /*!< @brief COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
  uint8_t       CompOVPInvInput_MODE;     /*!< @brief COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */

} R3_Params_t, *pR3_Params_t;

/**
  * @brief  This structure is used to handle an instance of the
  *         r3_g4xx_pwm_curr_fdbk component.
  */
typedef struct
{
  PWMC_Handle_t _Super;     /*!< @brief address of   */
  uint16_t Half_PWMPeriod;  /*!< @brief Half PWM Period in timer clock counts */
  bool OverCurrentFlag;     /*!< @brief This flag is set when an overcurrent occurs.*/
  bool OverVoltageFlag;     /*!< @brief This flag is set when an overvoltage occurs.*/
  bool BrakeActionLock;     /*!< @brief This flag is set to avoid that brake action is
                                 interrupted.*/
  int32_t	Offset_I_R;     /*!< @brief R phase current offset */
  int32_t	Offset_I_S;     /*!< @brief S phase current offset */
  int32_t	Offset_I_T;     /*!< @brief T phase current offset */
  int32_t	Offset_U_R;     /*!< @brief R phase voltage offset */
  int32_t	Offset_U_S;     /*!< @brief S phase voltage offset */ 
  int32_t	Offset_U_T;     /*!< @brief T phase voltage offset */
  bool		isMeasuringOffset; /*!< @brief This flage is set during offset calibration process */
  fixp31_t	wPhaseOffsetFilterConst; /*!< @brief filter constant used to filter ossfet calibration measurements */
  uint32_t	maxTicksCurrentReconstruction;	/*!< @brief if TIMx.CRRy is above this number, sample is not valid */	
  uint8_t	oversampling_count; /*!< @brief stores the oversampling value */	
  CurrentReconstruction_t reconPrevious; /*!< @brief stores the sample validity of each phase */	
  pR3_Params_t pParams_str; /*!< @brief R3 parameters data structure address */	
  bool		isShorting; /*!< @brief this flag is set when motor phases are shorted */	
  bool ADCRegularLocked; /*!< This flag is set when regular conversions are locked. Specific to G4XX. */     /* Cut 2.2 patch*/
} PWMC_R3_Handle_t;

/* Exported functions ------------------------------------------------------- */

/* Initializes TIMx, ADC, OPAMP and DMA for current and voltage reading */
void R3_Init( PWMC_R3_Handle_t * pHandle, const scaleFlashParams_t *pScale );

/* It turns on low sides switches. */
void R3_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks);

/* It enables PWM generation on the proper Timer peripheral acting on MOE bit */
void R3_SwitchOnPWM( PWMC_Handle_t * pHdl );

/* It disables PWM generation on the proper Timer peripheral acting on MOE bit */
void R3_SwitchOffPWM( PWMC_Handle_t * pHdl );

/* Configures ADC sampling instant.*/
uint16_t R3_SetADCSampPointSectX( PWMC_Handle_t * pHdl );

/* It contains the TIMx Update event interrupt */
void * R3_TIMx_UP_IRQHandler( PWMC_R3_Handle_t * pHdl );

/* It contains the TIMx Break2 event interrupt */
void *R3_OCP_IRQHandler( PWMC_R3_Handle_t * pHdl );

/* It contains the TIMx Break1 event interrupt */
void *R3_OVP_IRQHandler( PWMC_R3_Handle_t * pHdl );

/* Swaps DMA target buffer for all DMA involved in current and voltage reading. */
void R3_SwapDmaBuffer( PWMC_R3_Handle_t * pHandle );

/* Gets phase voltage and currents measurements */
void R3_GetMeasurements(PWMC_Handle_t * pHdl, Currents_Irst_t* pCurrents_Irst, Voltages_Urst_t* pVoltages_Urst);

/* Gets DC bus measurements */
void R3_GetVbusMeasurements(PWMC_Handle_t * pHdl, fixp30_t* pBusVoltage);

/*Gets auxiliary measurements */ 
void R3_GetAuxAdcMeasurement( PWMC_Handle_t * pHdl, fixp30_t* pAuxMeasurement );

/* Determines the number of valid currents and their validity status */
void R3_SetCurrentReconstructionData( PWMC_Handle_t * pHdl );

/* Exports the number of valid currents and their validity status */
uint8_t R3_GetCurrentReconstructionData( PWMC_Handle_t * pHdl, CurrentReconstruction_t * pData );

/* Gets phase voltage and currents offset measurements. */
void R3_GetOffsets( PWMC_Handle_t * pHdl, PolarizationOffsets_t * PolarizationOffsets );

/* Sets phase voltage and currents offset measurements. */
void R3_SetOffsets( PWMC_Handle_t * pHdl, PolarizationOffsets_t * PolarizationOffsets );

/* Starts the offset calibration process */
void R3_StartOffsetCalibration( PWMC_Handle_t * pHdl );

/* Declares end of affset calibration process */
void R3_FinishOffsetCalibration( PWMC_Handle_t * pHdl );

/* Prepares the PWM for offset calibration. */
void R3_PreparePWM( PWMC_Handle_t * pHdl );

/* Shorts the motor by forcing 0% duty on all phases */
void R3_PwmShort( PWMC_Handle_t * pHdl );

/* Checks if PWM are in control */
bool R3_IsInPwmControl( PWMC_Handle_t * pHdl );

/* Gets Main Output Enable status of the timer that manages */
bool R3_IsPwmEnabled( PWMC_Handle_t * pHdl );

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

#endif /*R3_G4XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
