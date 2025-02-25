<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          PWM & Current Feedback component of the Motor Control SDK.
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
  * @ingroup pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PWMNCURRFDBK_H
#define PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_math.h"
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/* Exported defines ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/** @brief PWM & Current Sensing component handle type */
typedef struct PWMC_Handle PWMC_Handle_t;

/** @brief Measurements data structure definition  */
typedef struct
{
  Currents_Iab_t      Iab_pu;      /*!< @brief Measured current vector in *aß* */
  Voltages_Uab_t      Uab_pu;      /*!< @brief Measured voltage vector in *aß* */
}Measurement_Output_t;

/**
  * @brief Pointer on callback functions used by PWMC components
  *
  * This type is needed because the actual functions to use can change at run-time.
  *
  * See the following items:
  * - PWMC_Handle::pFctSwitchOffPwm
  * - PWMC_Handle::pFctSwitchOnPwm
  * - PWMC_Handle::pFctCurrReadingCalib
  * - PWMC_Handle::pFctTurnOnLowSides
  */
typedef void ( *PWMC_Generic_Cb_t )( PWMC_Handle_t * pHandle );

// Same but with bool return value
typedef bool ( *PWMC_Bool_Cb_t )( PWMC_Handle_t * pHandle );

/**
  * @brief Pointer on the interrupt handling function of the PWMC component instance.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctIrqHandler).
  *
  */
typedef void * ( *PWMC_IrqHandler_Cb_t )( PWMC_Handle_t * pHandle, unsigned char flag );

/**
  * @brief Pointer on the function provided by the PMWC component instance to set the reference
  *        voltage for the over current protection.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctOCPSetReferenceVoltage).
  *
  */
typedef void ( *PWMC_SetOcpRefVolt_Cb_t )( PWMC_Handle_t * pHandle, uint16_t hDACVref );

/**
  * @brief Pointer on the functions provided by the PMWC component instance to set the ADC sampling
  *        point for each sectors.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctSetADCSampPointSectX).
  *
  */
typedef uint16_t ( *PWMC_SetSampPointSectX_Cb_t )( PWMC_Handle_t * pHandle);

/**
  * @brief Pointer on the function provided by the PMWC component instance to set low sides ON.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctTurnOnLowSides).
  *
  */
typedef void (*PWMC_TurnOnLowSides_Cb_t)(PWMC_Handle_t *pHandle, const uint32_t ticks);

/* New function pointer type to fetch samples */
/**
  * @brief Pointer on the function provided by the PMWC component instance to get the phase voltage and currents measurements.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctGetMeasurements).
  *
  */
typedef void ( *PWMC_GetMeasurements_Cb_t )( PWMC_Handle_t * pHandle, Currents_Irst_t* pCurrents_Irst, Voltages_Urst_t* pVoltages_Urst);

/**
  * @brief Pointer on the function provided by the PMWC component instance to get the auxiliary measurement.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctGetMeasurements).
  *
  */
typedef void ( *PWMC_GetAuxAdcMeas_Cb_t )( PWMC_Handle_t * pHandle, fixp30_t* pAuxMeasurement );

/**
  * @brief Pointer on the function provided by the PMWC component instance to get the phase voltage and currents measurements.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctGetAuxAdcMeasurement).
  *
  */
typedef uint8_t ( *PWMC_GetCurrentReconstructionData_Cb_t )( PWMC_Handle_t * pHdl, CurrentReconstruction_t * pData );

/**
  * @brief Pointer on the function provided by the PMWC component instance to get the offset calibration values.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctGetOffsets).
  *
  */
typedef void ( *PWMC_GetOffsets_Cb_t )( PWMC_Handle_t * pHandle, PolarizationOffsets_t * PolarizationOffsets );

/**
  * @brief Pointer on the function provided by the PMWC component instance to set the offset calibration values.
  *
  * This type is needed because the actual function to use can change at run-time
  * (See PWMC_Handle::pFctSetOffsets).
  *
  */
typedef void ( *PWMC_SetOffsets_Cb_t )( PWMC_Handle_t * pHandle, PolarizationOffsets_t * PolarizationOffsets );

/**
  * @brief This structure is used to handle the data of an instance of the PWM & Current Feedback component
  *
  */
struct PWMC_Handle
{
  /** @{ */
  PWMC_IrqHandler_Cb_t                    pFctIrqHandler;                   /*!< @brief pointer on the interrupt handling function. */
  PWMC_Generic_Cb_t                       pFctSwitchOffPwm;                 /*!< @brief pointer on the function the component instance uses to switch PWM off */
  PWMC_Generic_Cb_t                       pFctSwitchOnPwm;                  /*!< @brief pointer on the function the component instance uses to switch PWM on */
  PWMC_Generic_Cb_t                       pFctCurrReadingCalib;             /*!< @brief pointer on the function the component instance uses to calibrate the current reading ADC(s) */
  PWMC_TurnOnLowSides_Cb_t                pFctTurnOnLowSides;               /*!< @brief pointer on the function the component instance uses to turn low sides on */
  PWMC_SetSampPointSectX_Cb_t             pFctSetADCSampPointSectX;         /*!< @brief pointer on the function the component instance uses to set the ADC sampling point  */
  PWMC_SetOcpRefVolt_Cb_t                 pFctOCPSetReferenceVoltage;       /*!< @brief pointer on the function the component instance uses to set the over current reference voltage */
  /** @} */
  uint16_t                                CntPhA;                           /*!< @brief PWM Duty cycle for phase A */
  uint16_t                                CntPhB;                           /*!< @brief PWM Duty cycle for phase B */
  uint16_t                                CntPhC;                           /*!< @brief PWM Duty cycle for phase C */
  uint16_t                                SWerror;                          /*!< @brief Contains status about SW error */
  bool                                    TurnOnLowSidesAction;             /*!< @brief true if TurnOnLowSides action is active, false otherwise. */
  uint16_t                                OffCalibrWaitTimeCounter;         /*!< @brief Counter to wait fixed time before motor
                                                                                 current measurement offset calibration. */
  uint8_t                                 Motor;                            /*!< @brief Motor reference number */
  uint16_t                                QuarterPeriodCnt;                 /*!< @brief Quarter period. This is the compare value for 50% duty */
  FIXP_scaled_t                           duty_sf;                          /*!< @brief Scale factor of QuarterPeriodCnt */
  uint16_t                                OffCalibrWaitTicks;               /*!< @brief Wait time duration before current reading
                                                                             *  calibration expressed in number of calls
                                                                             *  of PWMC_CurrentReadingCalibr() with action
                                                                             *  #CRC_EXEC */
  GPIO_TypeDef                            *pwm_en_u_port;                   /*!< @brief Channel 1N (low side) GPIO output */
  GPIO_TypeDef                            *pwm_en_v_port;                   /*!< @brief Channel 2N (low side) GPIO output*/
  GPIO_TypeDef                            *pwm_en_w_port;                   /*!< @brief Channel 3N (low side)  GPIO output */
  uint16_t                                pwm_en_u_pin;                     /*!< @brief Channel 1N (low side) GPIO output pin */
  uint16_t                                pwm_en_v_pin;                     /*!< @brief Channel 2N (low side) GPIO output pin */
  uint16_t                                pwm_en_w_pin;                     /*!< @brief Channel 3N (low side)  GPIO output pin */
  LowSideOutputsFunction_t                LowSideOutputs;                   /*!< @brief Low side or enabling signals
                                                                                 generation method are defined
                                                                                 here.*/
  fixp30_t                                softOvercurrentTripLevel_pu;      /*!< @brief software overcurrent trip limit*/
  Currents_Irst_t                         Irst_in_pu;                       /*!< @brief input phase currents after reconstruction */
  Currents_Irst_t                         Irst_in_raw_pu;                   /*!< @brief raw input phase currents */
  Voltages_Urst_t                         Urst_in_pu;                       /*!< @brief input phase voltage */
  Currents_Iab_t                          Iab_in_pu;                        /*!< @brief Measured current vector in *aß* coordinates */
  Voltages_Uab_t                          Uab_in_pu;                        /*!< @brief Measured voltage vector in *aß* coordinates */
  bool                                    flagEnableCurrentReconstruction;  /*!< @brief reconstruction activation flag */
  bool                                    currentsamplingfault;             /*!< @brief current sampling fault. 
                                                                             *  - true: only one current has been sampled. reconstruction is impossible 
                                                                             *  - false: at least teo current have been sampled */ 
  bool                                    OverCurrentFlag;                  /* This flag is set when an overcurrent occurs.*/
  bool                                    OverVoltageFlag;                  /* This flag is set when an overvoltage occurs.*/
  bool                                    driverProtectionFlag;             /* This flag is set when a driver protection occurs.*/  
  bool                                    BrakeActionLock;                  /* This flag is set to avoid that brake action is interrupted.*/

  POLPULSE_Handle                         polpulseHandle;                   /*!< @brief PolPulse handler */
  Duty_Drst_t                             Drst_unmodulated_out_pu;          /*!< @brief three phase duty cycles before modulation */
  Duty_Drst_t                             Drst_out_pu;                      /*!< @brief three phase duty cycles after modulation */
  FOC_ModulationMode_e                    modulationMode;                   /*!< @brief modulation type selector */
  PWMC_GetMeasurements_Cb_t               pFctGetMeasurements;              /*!< @brief pointer on the function the component instance uses to 
                                                                             * to get the phase voltage and currents measurements  */
  PWMC_GetAuxAdcMeas_Cb_t                 pFctGetAuxAdcMeasurement;         /*!< @brief pointer on the function the component instance uses to 
                                                                             * to get the auxiliary measurement*/
  PWMC_GetCurrentReconstructionData_Cb_t  pFctGetCurrentReconstructionData; /*!< @brief pointer on the function the component instance uses to 
                                                                             * to get the reconstration data structure */
  PWMC_GetOffsets_Cb_t                    pFctGetOffsets;                   /*!< @brief pointer on the function the component instance uses to 
                                                                             * to get the offset calibration values */
  PWMC_Generic_Cb_t                       pFctPreparePWM;                   /*!< @brief pointer on the function the component instance uses to 
                                                                             * to prepare the PWM for offset calibration process */
  PWMC_Generic_Cb_t                       pFctStartOffsetCalibration;       /*!< @brief pointer on the function the component instance uses to 
                                                                             * to start the offset calibration process */
  PWMC_Generic_Cb_t                       pFctFinishOffsetCalibration;      /*!< @brief pointer on the function the component instance uses to 
                                                                             * to finish the offset calibration process */
  PWMC_SetOffsets_Cb_t                    pFctSetOffsets;                   /*!< @brief pointer on the function the component instance uses to 
                                                                             * to set offset calibration values */
  PWMC_Generic_Cb_t                       pFctPwmShort;                     /*!< @brief pointer on the function the component instance uses to 
                                                                             * to short the motor phases */
  PWMC_Bool_Cb_t                          pFctIsInPwmControl;               /*!< @brief pointer on the function the component instance uses to 
                                                                             * to get PWM control status */
  PWMC_Bool_Cb_t                          pFctIsPwmEnabled;                 /*!< @brief pointer on the function the component instance uses to 
                                                                             * to get PWM output enabling status */
};

/**
  * @brief  Duty cycles data structure definition
  */
typedef struct
{
  Duty_Dab_t      Dab_pu;      /*!< @brief duty cycle vector *aß* coordinates */
}PhaseDuty_Input_t;

/**
  * @brief  Current reading calibration status definition
  */
typedef enum CRCAction
{
  CRC_START, /*!< @brief Initialize the current reading calibration.*/
  CRC_EXEC   /*!< @brief Execute the current reading calibration.*/
} CRCAction_t;

/* Switches the PWM generation off, setting the outputs to inactive */
void PWMC_SwitchOffPWM( PWMC_Handle_t * pHandle );

/* Switches the PWM generation on */
void PWMC_SwitchOnPWM( PWMC_Handle_t * pHandle );

/* Calibrates ADC current conversions by reading the offset voltage
 * present on ADC pins when no motor current is flowing.
 * This function should be called before each motor start-up */
bool PWMC_CurrentReadingCalibr( PWMC_Handle_t * pHandle,
                                CRCAction_t action );

/* Manages HW overcurrent protection. */
void *PWMC_OCP_Handler(PWMC_Handle_t *pHandle);

/* Manages driver protection. */
void *PWMC_DP_Handler(PWMC_Handle_t *pHandle);

/* Manages HW overvoltage protection. */
void *PWMC_OVP_Handler(PWMC_Handle_t *pHandle, TIM_TypeDef *TIMx);

/* Checks if a fault (OCP, DP or OVP) occurred since last call. */
uint16_t PWMC_IsFaultOccurred(PWMC_Handle_t *pHandle);


/* Sets the over current threshold through the DAC reference voltage. */
void PWMC_OCPSetReferenceVoltage( PWMC_Handle_t * pHandle,
                                  uint16_t hDACVref );

/* Sets the Callback that the PWMC component shall invoke to switch off PWM
 *        generation. */
void PWMC_RegisterSwitchOffPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                        PWMC_Handle_t * pHandle );

/* Sets the Callback that the PWMC component shall invoke to switch on PWM
 *        generation. */
void PWMC_RegisterSwitchonPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                       PWMC_Handle_t * pHandle );

/* Sets the Callback that the PWMC component shall invoke to execute a calibration
 * of the current sensing system. */
void PWMC_RegisterReadingCalibrationCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );

/* Sets the Callback that the PWMC component shall invoke to turn on low sides. */
void PWMC_RegisterTurnOnLowSidesCallBack( PWMC_TurnOnLowSides_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );

/* Sets the Callback that the PWMC component shall invoke to ADC sampling point for sector 1. */
void PWMC_RegisterSampPointSectXCallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );
  
/* Sets the Callback that the PWMC component shall invoke to set the reference voltage for the over current protection */
void PWMC_RegisterOCPSetRefVoltageCallBack( PWMC_SetOcpRefVolt_Cb_t pCallBack,
    PWMC_Handle_t * pHandle );

/* Sets the Callback that the PWMC component shall invoke to call PWMC instance IRQ handler */
void PWMC_RegisterIrqHandlerCallBack( PWMC_IrqHandler_Cb_t pCallBack,
                                      PWMC_Handle_t * pHandle );

/* Retrieves phase voltage and currents measurements from ADCs. */
void PWMC_GetMeasurements( PWMC_Handle_t * pHandle, Measurement_Output_t* Out );

/* Retrieves auxiliary measurements from ADCs. */
void PWMC_GetAuxAdcMeasurement( PWMC_Handle_t * pHandle, fixp30_t* pAuxMeasurement );

/* Retrieves the reconstruction data structure  */
uint8_t PWMC_GetCurrentReconstructionData( PWMC_Handle_t * pHandle, CurrentReconstruction_t* pData);

/* Gets the phase currents and voltages offsets calibration value. */
void PWMC_GetOffsets( PWMC_Handle_t * pHandle, PolarizationOffsets_t * PolarizationOffsets );

/* Sets the phase currents and voltages offsets calibration value. */
void PWMC_SetOffsets( PWMC_Handle_t * pHandle, PolarizationOffsets_t * PolarizationOffsets );

/* Performs the space vector modulation. */
uint16_t PWMC_SetPhaseDuty( PWMC_Handle_t * pHandle, Duty_Dab_t* In  );

/* Switch on low sides, switch off high sides, shorting the motor. */
void PWMC_PWMShort( PWMC_Handle_t * pHandle );

/* Returns the PWM control status. */
bool PWMC_IsInPwmControl( PWMC_Handle_t * pHandle );

/* Returns PWM enabling status. */
bool PWMC_IsPwmEnabled( PWMC_Handle_t * pHandle );

/**
  * @brief  Returns the status of TurnOnLowSides action.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval bool It returns the state of TurnOnLowSides action:
  *         true if TurnOnLowSides action is active, false otherwise.
  */
/** @brief Returns the status of the "TurnOnLowSide" action on the power stage
 *         controlled by the @p pHandle PWMC component: true if it
 *         is active, false otherwise*/
static inline bool PWMC_GetTurnOnLowSidesAction( PWMC_Handle_t * pHandle )
{
  return pHandle->TurnOnLowSidesAction;
}

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* PWMNCURRFDBK_H */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
