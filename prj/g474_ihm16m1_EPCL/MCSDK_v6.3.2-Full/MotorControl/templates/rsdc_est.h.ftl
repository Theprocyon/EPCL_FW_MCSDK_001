/**
  ******************************************************************************
  * @file    rsdc_est.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Rs DC estimation of the Motor Control SDK.
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
  * @ingroup rsdcest
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_RSDCEST_H
#define __MC_RSDCEST_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_curr_ctrl.h"
#include "speed_pos_fdbk_hso.h"
#include "speed_torq_ctrl_hso.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup rsdcest
  * @{
  */

/* Exported defines ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
  * @brief Lists all possible Rs DC estimation states
  */
typedef enum {
    RSDC_EST_RESET = 0,    /*!< @brief No polarization offset have been provided to or measured by
                            *          the Motor Control application since the application was started */
    RSDC_EST_ONGOING = 1,     /*!< @brief The polarization offset measurement procedure is on-going. The Motor
                               *          cannot be controlled yet. */
    RSDC_EST_COMPLETED = 2    /*!< @brief The polarization offsets measurement procedure has completed or
                               *          polarization offsets have been provided with MC_SetPolarizationOffsetsMotor1(). */
} RsDC_estimationState_t;

/**
  * @brief Rs DC estimation handler structure
  */
typedef struct
{
  STC_Handle_t *pSTC;                        /*!< @brief pointer on speed and torque component.*/
  CurrCtrl_Handle_t *pCurrCtrl;              /*!< @brief pointer on current controller component.*/
  SPD_Handle_t *pSPD;              /*!< @brief pointer on sensorless component.*/
  PWMC_Handle_t *pPWM;                       /*!< @brief pointer on PWM component.*/
  Currents_Iab_t Iab_inLP_pu;	             /*!< @brief low-passed measured current vector */
  Voltages_Uab_t Uab_inLP_pu;	             /*!< @brief low-passed measured voltage vector */
  uint32_t RSDCestimate_counter;             /*!< @brief counter used for Id current ramp up/down */
  uint32_t RSDCestimate_counterInitialValue; /*!< @brief initial value of  RsDCEstimation_Handle_t::RSDCestimate_counter */
  uint32_t RSDCestimate_counterMeasureValue; /*!< @brief todo */
  fixp30_t RSDCestimate_Id_ref_pu;           /*!< @brief todo */
  fixp30_t RSDCestimate_rampRate;            /*!< @brief todo */
  fixp30_t RSDCestimate_IqRefStore;          /*!< @brief todo */
  uint32_t  FastRsDC_ramp_duration;           /*!< @brief ramp duration */
  uint32_t  FastRsDC_measure_duration;        /*!< @brief total time allowed for fast RsDC measure */
  uint16_t RSDCestimate_FilterShift;         /*!< @brief Bit shift filter */
  RsDC_estimationState_t RSDCestimate_state; /*!< @brief todo */
  bool flag_enableRSDCestimate;              /*!< @brief todo */
  bool RSDCestimate_AlignOnly;               /*!< @brief todo */
  bool RSDCestimate_Fast;                    /*!< @brief combine both polPulses and RsDc */
} RsDCEstimation_Handle_t;

void RsDC_estimationInit(FOCVars_t *pFOCVars, RsDCEstimation_Handle_t *pRsDCEst);
void RsDC_estimationRun(FOCVars_t *pFOCVars, RsDCEstimation_Handle_t *pRsDCEst);
void RsDC_estimationFiltering(RsDCEstimation_Handle_t *pRsDCEst,   Currents_Iab_t   *Iab_in_pu,  Voltages_Uab_t   *Uab_in_pu);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/