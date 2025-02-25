<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    profiler_dcac.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          profiler DC/AC component of the Motor Control SDK.
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
  *
  */

#ifndef _PROFILER_DCAC_H_
#define _PROFILER_DCAC_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "profiler_types.h"
<#if MC.M1_SPEED_SENSOR == "HSO">
#include "profiler_impedest.h"
</#if>
  
/**
  * @brief  todo
  */ 
typedef enum _PROFILER_DCAC_State_e_
{
  PROFILER_DCAC_STATE_Idle,
  PROFILER_DCAC_STATE_RampUp,          /*!< @brief Duty is ramped up until power goal (or another limit) is reached */
  PROFILER_DCAC_STATE_DC_Measuring,    /*!< @brief Measure DC resistance */
  PROFILER_DCAC_STATE_AC_Measuring,    /*!< @brief Measure AC impedance using ZEST */
  PROFILER_DCAC_STATE_DConly,          /*!< @brief ToDo: Slight delay to let injection zero settle */
  PROFILER_DCAC_STATE_RampDown,        /*!< @brief Duty is ramped back down to zero */
  PROFILER_DCAC_STATE_Complete,        /*!< @brief todo */
  PROFILER_DCAC_STATE_Error,           /*!< @brief todo */
} PROFILER_DCAC_State_e;

/**
  * @brief  todo
  */
typedef enum _PROFILER_DCAC_Error_e_
{
  PROFILER_DCAC_ERROR_None,               /*!< @brief todo */
  PROFILER_DCAC_ERROR_ResistanceAC,       /*!< @brief todo */
  PROFILER_DCAC_ERROR_Inductance,         /*!< @brief todo */
} PROFILER_DCAC_Error_e;

/**
  * @brief  todo
  */
typedef struct _PROFILER_DCAC_Obj_
{
  PROFILER_DCAC_State_e  state;            /*!< @brief todo */
  PROFILER_DCAC_Error_e  error;            /*!< @brief todo */
<#if MC.M1_SPEED_SENSOR == "HSO">
  PROFILER_IMPEDEST_Obj impedestObj;
</#if>
  /* outputs */
  float_t        Rs_dc;                    /*!< @brief todo */
  float_t        Rs_inject;                /*!< @brief todo */
  float_t        Ls_inject;                /*!< @brief todo */

  /* configuration */
  float_t        background_rate_hz;       /*!< @brief todo */
  float_t        dc_measurement_time_sec;
  uint32_t       dc_measurement_counts;
  float_t        ac_measurement_time_sec;
  uint32_t       ac_measurement_counts;
  fixp30_t       ac_freqkHz_pu;            /*!< @brief todo */
  fixp30_t       ac_duty_pu;               /*!< @brief todo */
  fixp30_t       Id_dc_ref_pu;             /*!< @brief Current used during DC measurement */
  fixp30_t       Id_dc_max_pu;             /*!< @brief todo */
  fixp30_t       CurToRamp_sf_pu;          /*!< @brief todo */

  fixp30_t       freq_ac_ref_pu;           /*!< @brief todo */
  fixp30_t       Imag_pu;                  /*!< @brief todo */
  fixp30_t       Umag_pu;                  /*!< @brief todo */
  fixp30_t       ramp_rate_duty_per_cycle; /*!< @brief todo */
  float_t        fullScaleCurrent_A;       /*!< @brief todo */
  float_t        fullScaleVoltage_V;       /*!< @brief todo */
  float_t        voltagefilterpole_rps;    /*!< @brief todo */

    /* Targets */
  fixp30_t       duty_maxgoal_pu;          /*!< @brief Target duty */
  float_t        PowerDC_goal_W;           /*!< @brief Target power */
  fixp30_t       PowerDC_goal_pu;          /*!< @brief Target power (per unit) */

    /* Levels reached */
  float_t        PowerDC_W;                /*!< @brief Power reached during duty ramp-up and DC measurement */
  fixp30_t       dc_duty_pu;               /*!< @brief Duty reached */

  /* RS state data */
  Voltages_Udq_t UdqLP;                     /*!< @brief Low pass filtered DQ voltage vector */
  Currents_Idq_t IdqLP;                     /*!< @brief Low pass filtered DQ current vector */
  uint32_t       counter;                   /*!< @brief todo */
  bool           glue_dcac_fluxestim_on;    /*!< @brief todo */
} PROFILER_DCAC_Obj;

/**
  * @brief  todo
  */
typedef PROFILER_DCAC_Obj* PROFILER_DCAC_Handle;

void PROFILER_DCAC_init(PROFILER_DCAC_Handle handle);

void PROFILER_DCAC_setParams(PROFILER_DCAC_Handle handle, PROFILER_Params* pParams);

void PROFILER_DCAC_run(PROFILER_DCAC_Handle handle, MOTOR_Handle motor);

void PROFILER_DCAC_runBackground(PROFILER_DCAC_Handle handle, MOTOR_Handle motor);

void PROFILER_DCAC_reset(PROFILER_DCAC_Handle handle, MOTOR_Handle motor);

/* Accessors */

float_t PROFILER_DCAC_getDcMeasurementTime(PROFILER_DCAC_Handle handle);
float_t PROFILER_DCAC_getAcMeasurementTime(PROFILER_DCAC_Handle handle);
void PROFILER_DCAC_setIdDCMax(PROFILER_DCAC_Handle handle, const float_t id_dc_max);
void PROFILER_DCAC_setImpedCorr_RsLs(PROFILER_DCAC_Handle handle, MOTOR_Handle motor);
void PROFILER_DCAC_setDcMeasurementTime(PROFILER_DCAC_Handle handle, const float_t time_seconds);
void PROFILER_DCAC_setAcMeasurementTime(PROFILER_DCAC_Handle handle, const float_t time_seconds);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _PROFILER_DCAC_H_ */

/* end of profiler_dcac.h */
