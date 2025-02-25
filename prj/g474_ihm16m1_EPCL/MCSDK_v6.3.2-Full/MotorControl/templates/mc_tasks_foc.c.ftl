<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
/**
  ******************************************************************************
  * @file    mc_tasks_foc.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
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
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1 
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
<#if MC.MCP_EN == true>
#include "mcp_config.h"
</#if><#--  MC.MCP_EN == true -->
<#if MC.DEBUG_DAC_FUNCTIONALITY_EN>
#include "dac_ui.h"
</#if><#--  MC.DEBUG_DAC_FUNCTIONALITY_EN -->
#include "mc_app_hooks.h"
<#if MC.TESTENV == true>
  <#if MC.PFC_ENABLED == false >
#include "mc_testenv.h"
  </#if>
</#if>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/
<#if MC.M1_MTPA_ENABLING == true || MC.M2_MTPA_ENABLING == true>
static MTPA_Handle_t *pMaxTorquePerAmpere[NBR_OF_MOTORS] = {<#list 1..(MC.DRIVE_NUMBER?number) as NUM>MC_NULL<#sep>,</#sep></#list>};
</#if><#-- MC.M1_MTPA_ENABLING == true || MC.M2_MTPA_ENABLING == true -->
<#if MC.DRIVE_NUMBER != "1" && (MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true)>
OpenLoop_Handle_t *pOpenLoop[2] = {MC_NULL,MC_NULL};  /* Only if M1 or M2 has OPEN LOOP */
<#elseif (MC.DRIVE_NUMBER == "1" &&  MC.M1_DBG_OPEN_LOOP_ENABLE == true)>
OpenLoop_Handle_t *pOpenLoop[1] = {MC_NULL};          /* Only if M1 has OPEN LOOP */
</#if><#-- (MC.DRIVE_NUMBER > 1 &&  MC.M2_DBG_OPEN_LOOP_ENABLE == true) -->

static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);
<#if MC.DRIVE_NUMBER != "1">
static volatile uint16_t hMFTaskCounterM2 = ((uint16_t)0);
static volatile uint16_t hBootCapDelayCounterM2 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM2 = ((uint16_t)0);
</#if><#-- MC.DRIVE_NUMBER > 1 -->
<#if (MC.M1_ICL_ENABLED == true)>
static volatile bool ICLFaultTreatedM1 = true;
</#if><#-- (MC.M1_ICL_ENABLED == true) -->
<#if (MC.M2_ICL_ENABLED == true)>
static volatile bool ICLFaultTreatedM2 = true;
</#if><#-- (MC.M2_ICL_ENABLED == true) -->


<#if CHARGE_BOOT_CAP_ENABLING == true>
#define M1_CHARGE_BOOT_CAP_TICKS          (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)${MC.M1_PWM_CHARGE_BOOT_CAP_MS}) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)${MC.M1_PWM_CHARGE_BOOT_CAP_DUTY_CYCLES}\
                                      * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
</#if><#-- CHARGE_BOOT_CAP_ENABLING == true -->
<#if CHARGE_BOOT_CAP_ENABLING2 == true>
#define M2_CHARGE_BOOT_CAP_TICKS         (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)${MC.M2_PWM_CHARGE_BOOT_CAP_MS}) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)${MC.M2_PWM_CHARGE_BOOT_CAP_DUTY_CYCLES}\
                                      * ((uint32_t)PWM_PERIOD_CYCLES2 / 2U))
</#if><#-- CHARGE_BOOT_CAP_ENABLING2 == true -->

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
void TSK_MF_StopProcessing(uint8_t motor);

MCI_Handle_t *GetMCI(uint8_t bMotor);
  <#if MC.MOTOR_PROFILER != true>
static uint16_t FOC_CurrControllerM1(void);
    <#if MC.DRIVE_NUMBER != "1">
static uint16_t FOC_CurrControllerM2(void);
    </#if><#-- MC.DRIVE_NUMBER > 1 -->
  <#else><#-- MC.MOTOR_PROFILER == true -->
bool SCC_DetectBemf( SCC_Handle_t * pHandle );
  </#if><#-- MC.MOTOR_PROFILER != true -->

<#if MC.M1_ON_OVER_VOLTAGE == "TURN_OFF_PWM" || MC.M2_ON_OVER_VOLTAGE == "TURN_OFF_PWM">
void TSK_SafetyTask_PWMOFF(uint8_t motor);
</#if><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_OFF_PWM" || MC.M2_ON_OVER_VOLTAGE == "TURN_OFF_PWM" -->
<#if MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE">
void TSK_SafetyTask_RBRK(uint8_t motor);
</#if><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" -->
<#if MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES">
void TSK_SafetyTask_LSON(uint8_t motor);
</#if><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" -->
<#if MC.PFC_ENABLED == true>
void PFC_Scheduler(void);
</#if><#-- MC.PFC_ENABLED == true -->

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  */
__weak void FOC_Init(void)
{

  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */
  
<#if MC.TESTENV == true && MC.PFC_ENABLED == false>
    mc_testenv_init();
</#if><#-- MC.TESTENV == true && MC.PFC_ENABLED == false -->
   
<#if MC.M1_MTPA_ENABLING == true>
    pMaxTorquePerAmpere[M1] = &MTPARegM1;
</#if><#-- MC.M1_MTPA_ENABLING == true -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_MTPA_ENABLING == true>
    pMaxTorquePerAmpere[M2] = &MTPARegM2;
  </#if><#-- MC.M2_MTPA_ENABLING == true -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    ${M1_CS_TOPO}Init(&PWM_Handle_M1);
<#if MC.DRIVE_NUMBER != "1">
    pwmcHandle[M2] = &PWM_Handle_M2._Super;
    ${M2_CS_TOPO}Init(&PWM_Handle_M2);
</#if><#-- MC.DRIVE_NUMBER > 1 -->

    /* USER CODE BEGIN MCboot 1 */
  
    /* USER CODE END MCboot 1 */

  <#if !CondFamily_STM32F0 && !CondFamily_STM32G0 && !CondFamily_STM32C0>
    /**************************************/
    /*    Start timers synchronously      */
    /**************************************/
    startTimers();
  </#if><#-- !CondFamily_STM32F0 && !CondFamily_STM32G -->

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);
    
    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    ${SPD_init_M1} (${SPD_M1});
	
  <#if (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")>
    /******************************************************/
    /*   Main encoder alignment component initialization  */
    /******************************************************/
    EAC_Init(&EncAlignCtrlM1,pSTC[M1],&VirtualSpeedSensorM1,${SPD_M1});
    pEAC[M1] = &EncAlignCtrlM1;
  </#if><#-- (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") -->
  
  <#if MC.M1_POSITION_CTRL_ENABLING == true>
    /******************************************************/
    /*   Position Control component initialization        */
    /******************************************************/
    PID_HandleInit(&PID_PosParamsM1);
    TC_Init(&PosCtrlM1, &PID_PosParamsM1, &SpeednTorqCtrlM1, &ENCODER_M1);
  </#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M1],&PIDSpeedHandle_M1, ${SPD_M1}._Super);

   <#if AUX_SPEED_FDBK_M1  == true>
    /******************************************************/
    /*   Auxiliary speed sensor component initialization  */
    /******************************************************/
    ${SPD_aux_init_M1} (${SPD_AUX_M1});
    
     <#if (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z")>
    /***********************************************************/
    /*   Auxiliary encoder alignment component initialization  */
    /***********************************************************/
    EAC_Init(&EncAlignCtrlM1,pSTC[M1],&VirtualSpeedSensorM1,${SPD_AUX_M1});
    pEAC[M1] = &EncAlignCtrlM1;
     </#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z") -->
   </#if><#-- AUX_SPEED_FDBK_M1  == true -->

<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") >
    /**************************************/
    /*   Rev-up component initialization  */
    /**************************************/
    RUC_Init(&RevUpControlM1, pSTC[M1], &VirtualSpeedSensorM1, &STO_M1, pwmcHandle[M1]);
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") -->

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M1]->pVBS = &(BusVoltageSensor_M1._Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];

  <#if MC.M1_FLUX_WEAKENING_ENABLING == true>
    /*******************************************************/
    /*   Flux weakening component initialization           */
    /*******************************************************/
    PID_HandleInit(&PIDFluxWeakeningHandle_M1);
    FW_Init(pFW[M1],&PIDSpeedHandle_M1,&PIDFluxWeakeningHandle_M1);
  </#if><#-- MC.M1_FLUX_WEAKENING_ENABLING == true -->

  <#if MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true>
    /*******************************************************/
    /*   Feed forward component initialization             */
    /*******************************************************/
    FF_Init(pFF[M1],&(BusVoltageSensor_M1._Super),pPIDId[M1],pPIDIq[M1]);
  </#if><#-- MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true -->

  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
    OL_Init(&OpenLoop_ParamsM1, &VirtualSpeedSensorM1);     /* Only if M1 has open loop */
    pOpenLoop[M1] = &OpenLoop_ParamsM1;
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->

    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);

  <#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true>
    SCC.pPWMC = pwmcHandle[M1];
    SCC.pVBS = &BusVoltageSensor_M1;
    SCC.pFOCVars = &FOCVars[M1];
    SCC.pMCI = &Mci[M1];
    SCC.pVSS = &VirtualSpeedSensorM1;
    SCC.pCLM = &CircleLimitationM1;
    SCC.pPIDIq = pPIDIq[M1];
    SCC.pPIDId = pPIDId[M1];
    SCC.pRevupCtrl = &RevUpControlM1;
    SCC.pSTO = &STO_PLL_M1;
    SCC.pSTC = &SpeednTorqCtrlM1;
    SCC.pOTT = &OTT;
    <#if MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR">
    SCC.pHT = &HT;
    <#else><#-- MC.M1_AUXILIARY_SPEED_SENSOR != "HALL_SENSOR" -->
    SCC.pHT = MC_NULL;
    </#if><#-- MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->
    SCC_Init(&SCC);

    OTT.pSpeedSensor = &STO_PLL_M1._Super;
    OTT.pFOCVars = &FOCVars[M1];
    OTT.pPIDSpeed = &PIDSpeedHandle_M1;
    OTT.pSTC = &SpeednTorqCtrlM1;
    OTT_Init(&OTT);
  </#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true -->

    FOC_Clear(M1);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
    MCI_SetSpeedMode(&Mci[M1]);
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->

<#if MC.M1_DEFAULT_CONTROL_MODE == "STC_TORQUE_MODE">
    MCI_ExecTorqueRamp(&Mci[M1], STC_GetDefaultIqdref(pSTC[M1]).q, 0);
<#else><#-- MC.M1_DEFAULT_CONTROL_MODE != "STC_TORQUE_MODE" -->
    MCI_ExecSpeedRamp(&Mci[M1],
    STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /* First command to STC */
</#if><#-- MC.M1_DEFAULT_CONTROL_MODE == "STC_TORQUE_MODE"-->

<#if MC.MOTOR_PROFILER && (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR")>
    HT.pOTT = &OTT;
    HT.pMCI = &Mci[M1];
    HT.pHALL_M1 = &HALL_M1;
    HT.pSTO_PLL_M1 = &STO_PLL_M1;
    HT_Init(&HT, false);
</#if><#-- MC.MOTOR_PROFILER && (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR") -->
   
<#if MC.DRIVE_NUMBER != "1">
    /******************************************************/
    /*   Motor 2 features initialization                  */
    /******************************************************/
    
    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/  
    PID_HandleInit(&PIDSpeedHandle_M2);
    
    /***********************************************************/
    /*   Main speed  sensor initialization: speed regulation   */
    /***********************************************************/ 
    ${SPD_init_M2} (${SPD_M2});
    
  <#if (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")>
    /******************************************************/
    /*   Main encoder alignment component initialization  */
    /******************************************************/  
    EAC_Init(&EncAlignCtrlM2,pSTC[M2],&VirtualSpeedSensorM2,${SPD_M2});
    pEAC[M2] = &EncAlignCtrlM2;
  </#if><#-- (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z") -->
  
  <#if MC.M2_POSITION_CTRL_ENABLING == true>
    /******************************************************/
    /*   Position Control component initialization        */
    /******************************************************/
    PID_HandleInit(&PID_PosParamsM2);
    TC_Init(&PosCtrlM2, &PID_PosParamsM2, &SpeednTorqCtrlM2, ${SPD_M2});
  </#if><#-- MC.M2_POSITION_CTRL_ENABLING == true -->

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M2], &PIDSpeedHandle_M2, ${SPD_M2}._Super);

  <#if AUX_SPEED_FDBK_M2>
    /***********************************************************/
    /*   Auxiliary speed sensor component initialization       */
    /***********************************************************/ 
    ${SPD_aux_init_M2} (${SPD_AUX_M2});

    <#if (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z")>
    /***********************************************************/
    /*   Auxiliary encoder alignment component initialization  */
    /***********************************************************/ 
    EAC_Init(&EncAlignCtrlM2,pSTC[M2],&VirtualSpeedSensorM2,${SPD_AUX_M2});
    pEAC[M2] = &EncAlignCtrlM2;
    </#if><#-- (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z") -->
  </#if><#-- AUX_SPEED_FDBK_M2 -->

  <#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")>
    /****************************************************/
    /*   Rev-up component initialization                */
    /****************************************************/ 
    RUC_Init(&RevUpControlM2, pSTC[M2], &VirtualSpeedSensorM2, &STO_M2, pwmcHandle[M2]); /* Only if sensorless */
  </#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") -->

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M2);
    PID_HandleInit(&PIDIdHandle_M2);
 
    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M2]->pVBS = &(BusVoltageSensor_M2._Super);
    pMPM[M2]->pFOCVars = &FOCVars[M2];

  <#if MC.M2_FLUX_WEAKENING_ENABLING == true>
    /*************************************************/
    /*   Flux weakening component initialization     */
    /*************************************************/
    PID_HandleInit(&PIDFluxWeakeningHandle_M2);
    FW_Init(pFW[M2], &PIDSpeedHandle_M2, &PIDFluxWeakeningHandle_M2); /* Only if M2 has FW */
  </#if><#-- MC.M2_FLUX_WEAKENING_ENABLING == true -->

  <#if MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true>
    /*************************************************/
    /*   Feed forward component initialization       */
    /*************************************************/
    FF_Init(pFF[M2], &(BusVoltageSensor_M2._Super), pPIDId[M2], pPIDIq[M2]); /* Only if M2 has FF */
  </#if><#-- MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true -->

  <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
    OL_Init(&OpenLoop_ParamsM2, &VirtualSpeedSensorM2); /* Only if M2 has open loop */
    pOpenLoop[M2] = &OpenLoop_ParamsM2;
  </#if><#-- MC.M2_DBG_OPEN_LOOP_ENABLE == true -->

    pREMNG[M2] = &RampExtMngrHFParamsM2;
    REMNG_Init(pREMNG[M2]);
    FOC_Clear(M2);
    FOCVars[M2].bDriveInput = EXTERNAL;
    FOCVars[M2].Iqdref = STC_GetDefaultIqdref(pSTC[M2]);
    FOCVars[M2].UserIdref = STC_GetDefaultIqdref(pSTC[M2]).d;
  <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
    MCI_SetSpeedMode(&Mci[M2]);
  </#if><#-- MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
  <#if MC.M2_DEFAULT_CONTROL_MODE == 'STC_TORQUE_MODE'>
    MCI_ExecTorqueRamp(&Mci[M2], STC_GetDefaultIqdref(pSTC[M2]).q, 0);
  <#else><#-- MC.M2_DEFAULT_CONTROL_MODE != "STC_TORQUE_MODE" -->
    MCI_ExecSpeedRamp(&Mci[M2],
    STC_GetMecSpeedRefUnitDefault(pSTC[M2]),0); /* First command to STC */
  </#if><#-- MC.M2_DEFAULT_CONTROL_MODE == 'STC_TORQUE_MODE -->
	
</#if><#-- MC.DRIVE_NUMBER > !1 -->

<#if MC.M1_ICL_ENABLED == true>
    ICL_Init(&ICL_M1, &ICLDOUTParamsM1);
    Mci[M1].State = ICLWAIT;
</#if><#-- MC.M1_ICL_ENABLED == true -->
<#if MC.M2_ICL_ENABLED == true>
    ICL_Init(&ICL_M2, &ICLDOUTParamsM2);
    Mci[M2].State = ICLWAIT;
</#if><#-- MC.M2_ICL_ENABLED == true -->

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
}

/**
 * @brief Performs stop process and update the state machine.This function 
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
  <#if MC.DRIVE_NUMBER != "1">
  if (M1 == motor)
  {
    ${M1_CS_TOPO}SwitchOffPWM(pwmcHandle[motor]);
  }
  else
  {
    ${M2_CS_TOPO}SwitchOffPWM(pwmcHandle[motor]);
  }
  <#else>
    ${M1_CS_TOPO}SwitchOffPWM(pwmcHandle[motor]);
  </#if>

<#if MC.MOTOR_PROFILER == true && MC.DRIVE_NUMBER == "1">
  SCC_Stop(&SCC);
  OTT_Stop(&OTT);
</#if><#-- MC.MOTOR_PROFILER == true && MC.DRIVE_NUMBER == 1 -->
  FOC_Clear(motor);

<#if MC.M1_DISCONTINUOUS_PWM == true  || MC.M2_DISCONTINUOUS_PWM == true>
  /* Disable DPWM mode */
  PWMC_DPWM_ModeDisable(pwmcHandle[motor]);
</#if><#-- MC.M1_DISCONTINUOUS_PWM == true  || MC.M2_DISCONTINUOUS_PWM == true -->
<#if MC.DRIVE_NUMBER != "1">
 if (M1 == motor)
  {
    TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  }
  else
  {
    TSK_SetStopPermanencyTimeM2(STOPPERMANENCY_TICKS);
  }
<#else>
  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
</#if>  
  Mci[motor].State = STOP;
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the 
  * present state of its state machine. In particular, duties requiring a periodic 
  * execution at a medium frequency rate (such as the speed controller for instance) 
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  int16_t wAux = 0;
<#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") && (MC.M1_OTF_STARTUP == true)>
  PWMC_${M1_CS_TOPO}Handle_t *p_Handle = (PWMC_${M1_CS_TOPO}Handle_t *)pwmcHandle[M1];
</#if><#-- (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") && (MC.M1_OTF_STARTUP == true) -->
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
  MC_ControlMode_t mode;
  
  mode = MCI_GetControlMode(&Mci[M1]);
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
<#if MC.M1_ICL_ENABLED == true>
  uint16_t Vbus_M1 = VBS_GetAvBusVoltage_V(&(BusVoltageSensor_M1._Super));
  ICL_State_t ICLstate = ICL_Exec(&ICL_M1, Vbus_M1);
</#if><#-- MC.M1_ICL_ENABLED == true -->
<#if AUX_SPEED_FDBK_M1 == true>
  (void)${SPD_aux_calcAvrgMecSpeedUnit_M1}(${SPD_AUX_M1}, &wAux);
</#if><#-- AUX_SPEED_FDBK_M1 == true -->
<#if MC.M1_SPEED_FEEDBACK_CHECK == true || (MC.M1_SPEED_SENSOR == "HALL_SENSOR")>
  bool IsSpeedReliable = ${SPD_calcAvrgMecSpeedUnit_M1}(${SPD_M1}, &wAux);
<#else><#-- MC.M1_SPEED_FEEDBACK_CHECK = false || (MC.M1_SPEED_SENSOR != "HALL_SENSOR") -->
  (void)${SPD_calcAvrgMecSpeedUnit_M1}(${SPD_M1}, &wAux);
</#if><#-- MC.M1_SPEED_FEEDBACK_CHECK == true || (MC.M1_SPEED_SENSOR == "HALL_SENSOR") -->
  PQD_CalcElMotorPower(pMPM[M1]);
  
<#if MC.M1_ICL_ENABLED == true>
  if ( !ICLFaultTreatedM1 && (ICLstate == ICL_ACTIVE))
  {
    ICLFaultTreatedM1 = true;
  }
  else
  {
    /* Nothing to do */
  }
</#if><#-- MC.M1_ICL_ENABLED == true -->

<#if MC.M1_ICL_ENABLED == true>
  if ((MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS) && ICLFaultTreatedM1)
<#else><#-- MC.M1_ICL_ENABLED == false -->
  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
</#if><#-- MC.M1_ICL_ENABLED == true -->
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {
<#if MC.M1_ICL_ENABLED == true>
        case ICLWAIT:
        {
          if (ICL_INACTIVE == ICLstate)
          {
            /* If ICL is Inactive, move to IDLE */
            Mci[M1].State = IDLE;
          }
          break;
        }
</#if><#-- MC.M1_ICL_ENABLED == true -->
        
        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") >
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
            if ( mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE) 
            {
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
              RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
            }
            else
            {
              /* Nothing to do */
            }
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") -->
            if (pwmcHandle[M1]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
              Mci[M1].State = OFFSET_CALIB;
            }
            else
            {
              /* Calibration already done. Enables only TIM channels */
              pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);
<#if CHARGE_BOOT_CAP_ENABLING == true>
  <#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE")>
    <#if MC.M1_DP_DESTINATION == "TIM_BKIN">
              LL_TIM_DisableBRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    <#elseif MC.M1_DP_DESTINATION == "TIM_BKIN2">
              LL_TIM_DisableBRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    </#if><#-- MC.M1_DP_DESTINATION == "TIM_BKIN" -->
  </#if><#-- (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") -->
  <#if (MC.M1_OTF_STARTUP == true)>
              ${M1_CS_TOPO}TurnOnLowSides(pwmcHandle[M1],p_Handle->Half_PWMPeriod-1);
  <#else><#-- !M1_OTF_STARTUP -->
              ${M1_CS_TOPO}TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
  </#if><#-- M1_OTF_STARTUP -->
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
<#else><#-- CHARGE_BOOT_CAP_ENABLING == false -->
<#-- test sensorless -->
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);
              ${SPD_clear_M1}(${SPD_M1});
              FOC_Clear(M1);
              ${M1_CS_TOPO}SwitchOnPWM(pwmcHandle[M1]);
              Mci[M1].State = START;
</#if><#-- CHARGE_BOOT_CAP_ENABLING == true -->
            }
<#if (MC.MOTOR_PROFILER == true) || (MC.ONE_TOUCH_TUNING == true)>
            OTT_Clear(&OTT);
</#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true -->
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
            {
              if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
              {
                FOC_Clear(M1);
                Mci[M1].DirectCommand = MCI_NO_COMMAND;
                Mci[M1].State = IDLE;
              }
              else
              {
  <#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true>
                Mci[M1].State = WAIT_STOP_MOTOR;
  <#else><#-- MC.MOTOR_PROFILER == false || MC.ONE_TOUCH_TUNING == false -->
    <#if CHARGE_BOOT_CAP_ENABLING == true>
      <#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE")>
        <#if MC.M1_DP_DESTINATION == "TIM_BKIN">
                LL_TIM_DisableBRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
        <#elseif MC.M1_DP_DESTINATION == "TIM_BKIN2">
                LL_TIM_DisableBRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
        </#if><#-- MC.M1_DP_DESTINATION == "TIM_BKIN" -->
      </#if><#-- (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") -->
      <#if (MC.M1_OTF_STARTUP == true)>
                ${M1_CS_TOPO}TurnOnLowSides(pwmcHandle[M1],p_Handle->Half_PWMPeriod-1);
      <#else><#-- !M1_OTF_STARTUP -->
                ${M1_CS_TOPO}TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
      </#if><#-- M1_OTF_STARTUP -->
                TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                Mci[M1].State = CHARGE_BOOT_CAP;
    <#else><#-- CHARGE_BOOT_CAP_ENABLING == false -->
                FOCVars[M1].bDriveInput = EXTERNAL;
                STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);
                ${SPD_clear_M1}(${SPD_M1});
                FOC_Clear(M1);
      <#if MC.M1_DISCONTINUOUS_PWM == true>
                /* Enable DPWM mode before Start */
                PWMC_DPWM_ModeEnable(pwmcHandle[M1]);
      </#if><#-- MC.M1_DISCONTINUOUS_PWM == true -->
                ${M1_CS_TOPO}SwitchOnPWM(pwmcHandle[M1]);
                Mci[M1].State = START;
    </#if><#-- CHARGE_BOOT_CAP_ENABLING == true -->
  </#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true -->
              }
            }
            else
            {
              /* Nothing to be done, FW waits for offset calibration to finish */
            }
          }  
          break;
        }


<#if (CHARGE_BOOT_CAP_ENABLING == true)>
  
        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              ${M1_CS_TOPO}SwitchOffPWM(pwmcHandle[M1]);
  <#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE")>
    <#if MC.M1_DP_DESTINATION == "TIM_BKIN">
              LL_TIM_ClearFlag_BRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
              LL_TIM_EnableBRK(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    <#elseif MC.M1_DP_DESTINATION == "TIM_BKIN2">
              LL_TIM_ClearFlag_BRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
              LL_TIM_EnableBRK2(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    </#if><#-- MC.M1_DP_DESTINATION == "TIM_BKIN" -->
  </#if><#-- (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") -->
<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || M1_ENCODER >
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || M1_ENCODER -->


              ${SPD_clear_M1}(${SPD_M1});
<#if AUX_SPEED_FDBK_M1 == true>
              ${SPD_aux_clear_M1}(${SPD_AUX_M1});
</#if><#-- AUX_SPEED_FDBK_M1 == true -->
<#if MC.M1_OVERMODULATION == true>
              PWMC_Clear(pwmcHandle[M1]);
</#if><#-- MC.M1_OVERMODULATION == true -->
<#if MC.M1_DISCONTINUOUS_PWM == true>
              /* Enable DPWM mode before Start */
              PWMC_DPWM_ModeEnable( pwmcHandle[M1]);
</#if><#-- MC.M1_DISCONTINUOUS_PWM == true -->

              FOC_Clear( M1 );
  <#if (MC.M1_OTF_STARTUP == true)>
              ${M1_CS_TOPO}SwitchOnPWM(pwmcHandle[M1]);
  </#if><#-- MC.M1_OTF_STARTUP == true -->

	  
<#if (MC.MOTOR_PROFILER == true)>
        SCC_Start(&SCC);
              /* The generic function needs to be called here as the undelying   
               * implementation changes in time depending on the Profiler's state 
               * machine. Calling the generic function ensures that the correct
               * implementation is invoked */
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
              Mci[M1].State = START;
<#else><#-- MC.MOTOR_PROFILER == false -->
 
  <#if (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")>
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE>
              if (mode == MCM_OPEN_LOOP_VOLTAGE_MODE || mode == MCM_OPEN_LOOP_CURRENT_MODE)
              { 
                MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp even in OL */
                Mci[M1].State = RUN;              
              }
              else
              {
    </#if>  			  
              if (EAC_IsAligned(&EncAlignCtrlM1) == false)
              {
                EAC_StartAlignment(&EncAlignCtrlM1);
                Mci[M1].State = ALIGNMENT;
              }
              else
              {
                STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
                STC_SetSpeedSensor(pSTC[M1], &ENCODER_M1._Super);
                FOC_InitAdditionalMethods(M1);
                FOC_CalcCurrRef(M1);
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
                Mci[M1].State = RUN;
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE >
              }
    </#if>
              }
  <#elseif  (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z")>
              if (EAC_IsAligned(&EncAlignCtrlM1) == false)
              {
                EAC_StartAlignment(&EncAlignCtrlM1);
                Mci[M1].State = ALIGNMENT;
              }
              else
              {
                VSS_Clear(&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
                FOC_Clear(M1);
                Mci[M1].State = START;
              }
  <#elseif MC.M1_SPEED_SENSOR == "HALL_SENSOR">
      <#if MC.M1_DBG_OPEN_LOOP_ENABLE> 
              if (mode == MCM_OPEN_LOOP_VOLTAGE_MODE || mode == MCM_OPEN_LOOP_CURRENT_MODE)
              { /* In open loop mode, angle comes from virtual sensor */
                STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );
              }
              else 
              {
                STC_SetSpeedSensor( pSTC[M1], &HALL_M1._Super );
      </#if> <#-- M1_DBG_OPEN_LOOP_ENABLE -->
                FOC_InitAdditionalMethods(M1);
                FOC_CalcCurrRef(M1);
                STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1]); /* Init the reference speed to current speed */
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE>
              }
    </#if>				
              MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
              Mci[M1].State = RUN;		  
  <#else><#-- sensorless mode only -->
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
              if (MCM_OPEN_LOOP_VOLTAGE_MODE == mode || MCM_OPEN_LOOP_CURRENT_MODE == mode)
              {
                Mci[M1].State = RUN;
              }
              else
              {
    </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
                Mci[M1].State = START;
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
              }
    </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
  </#if><#-- (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") -->
</#if><#-- MC.MOTOR_PROFILER == true -->
<#if (MC.M1_OTF_STARTUP == false)>
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
</#if><#-- (MC.M1_OTF_STARTUP == false) -->
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }
</#if><#-- CHARGE_BOOT_CAP_ENABLING == true -->

<#if M1_ENCODER><#-- only for encoder -->
        case ALIGNMENT:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            bool isAligned = EAC_IsAligned(&EncAlignCtrlM1);
            bool EACDone = EAC_Exec(&EncAlignCtrlM1);
            if ((isAligned == false)  && (EACDone == false))
            {
              qd_t IqdRef;
              IqdRef.q = 0;
              IqdRef.d = STC_CalcTorqueReference(pSTC[M1]);
              FOCVars[M1].Iqdref = IqdRef;
            }
            else
            {
              ${M1_CS_TOPO}SwitchOffPWM( pwmcHandle[M1] );
  <#if (MC.M1_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M1_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER_Z")>
              STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
              STC_SetSpeedSensor(pSTC[M1], &ENCODER_M1._Super);
  </#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M1_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER_Z") -->
              FOC_Clear(M1);
  <#if (CHARGE_BOOT_CAP_ENABLING == true)>
              ${M1_CS_TOPO}TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
  </#if><#-- CHARGE_BOOT_CAP_ENABLING == true -->
              TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
              Mci[M1].State = WAIT_STOP_MOTOR;
              /* USER CODE BEGIN MediumFrequencyTask M1 EndOfEncAlignment */
            
              /* USER CODE END MediumFrequencyTask M1 EndOfEncAlignment */
            }
          }
          break;
        }
</#if><#-- M1_ENCODER -->

<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") >
        case START:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
          <#-- only for sensor-less control -->
            /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
            int16_t hForcedMecSpeedUnit;
            qd_t IqdRef;
            bool ObserverConverged;

            /* Execute the Rev Up procedure */
  <#if MC.M1_OTF_STARTUP == true>
            if (! RUC_OTF_Exec(&RevUpControlM1))
  <#else><#-- MC.M1_OTF_STARTUP == false -->
            if(! RUC_Exec(&RevUpControlM1))
  </#if><#-- MC.M1_OTF_STARTUP == true -->
            {
            /* The time allowed for the startup sequence has expired */
  <#if MC.MOTOR_PROFILER == true>
              /* However, no error is generated when OPEN LOOP is enabled 
               * since then the system does not try to close the loop... */
  <#else><#-- MC.MOTOR_PROFILER == false -->
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
  </#if><#-- MC.MOTOR_PROFILER == true -->
            }
            else
            {
              /* Execute the torque open loop current start-up ramp:
               * Compute the Iq reference current as configured in the Rev Up sequence */
              IqdRef.q = STC_CalcTorqueReference(pSTC[M1]);
              IqdRef.d = FOCVars[M1].UserIdref;
              /* Iqd reference current used by the High Frequency Loop to generate the PWM output */
              FOCVars[M1].Iqdref = IqdRef;
           }
          
            (void)VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);
         
  <#if MC.M1_OTF_STARTUP == false && MC.MOTOR_PROFILER == false>
            /* Check that startup stage where the observer has to be used has been reached */
            if (true == RUC_FirstAccelerationStageReached(&RevUpControlM1))
            {
  </#if><#-- MC.M1_OTF_STARTUP == false) && (MC.MOTOR_PROFILER == false -->
  <#if MC.M1_SPEED_SENSOR == "STO_PLL">
              ObserverConverged = STO_PLL_IsObserverConverged(&STO_PLL_M1, &hForcedMecSpeedUnit);
              STO_SetDirection(&STO_PLL_M1, (int8_t)MCI_GetImposedMotorDirection(&Mci[M1]));
  <#elseif MC.M1_SPEED_SENSOR == "STO_CORDIC">
              ObserverConverged = STO_CR_IsObserverConverged(&STO_CR_M1, hForcedMecSpeedUnit);
  </#if> <#-- MC.M1_SPEED_SENSOR == "STO_PLL"     -->
              (void)VSS_SetStartTransition(&VirtualSpeedSensorM1, ObserverConverged);
  <#if MC.M1_OTF_STARTUP == false && MC.MOTOR_PROFILER == false>
            }
            else
            {
              ObserverConverged = false;
            }
  </#if><#-- MC.M1_OTF_STARTUP == false) && (MC.MOTOR_PROFILER == false -->
            if (ObserverConverged)
            {
              qd_t StatorCurrent = MCM_Park(FOCVars[M1].Ialphabeta, SPD_GetElAngle(${SPD_M1}._Super));

              /* Start switch over ramp. This ramp will transition from the revup to the closed loop FOC */
              REMNG_Init(pREMNG[M1]);
              (void)REMNG_ExecRamp(pREMNG[M1], FOCVars[M1].Iqdref.q, 0);
              (void)REMNG_ExecRamp(pREMNG[M1], StatorCurrent.q, TRANSITION_DURATION);

              Mci[M1].State = SWITCH_OVER;
            }
          }
          break;
        }

        case SWITCH_OVER:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            bool LoopClosed;
            int16_t hForcedMecSpeedUnit;

  <#if MC.MOTOR_PROFILER == false><#-- No need to call RUC_Exec() when in MP in this state -->
    <#if MC.M1_OTF_STARTUP == true>
            if (! RUC_OTF_Exec(&RevUpControlM1))
    <#else><#-- MC.M1_OTF_STARTUP == false -->
            if (! RUC_Exec(&RevUpControlM1))
    </#if><#-- MC.M1_OTF_STARTUP == true -->
            {
              /* The time allowed for the startup sequence has expired */
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
            } 
            else
            {
  </#if><#--  MC.MOTOR_PROFILER == false -->
              /* Compute the virtual speed and positions of the rotor. 
                 The function returns true if the virtual speed is in the reliability range */
              LoopClosed = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);
              /* Check if the transition ramp has completed. */
              bool tempBool;
              tempBool = VSS_TransitionEnded(&VirtualSpeedSensorM1);
              LoopClosed = LoopClosed || tempBool;
              
              /* If any of the above conditions is true, the loop is considered closed. 
                 The state machine transitions to the RUN state */
              if (true ==  LoopClosed) 
              {
#if PID_SPEED_INTEGRAL_INIT_DIV == 0
                PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
                PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                    (((int32_t)FOCVars[M1].Iqdref.q * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1)) 
                                    / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
  <#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true>
                OTT_SR(&OTT);
  </#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true -->
                /* USER CODE BEGIN MediumFrequencyTask M1 1 */

                /* USER CODE END MediumFrequencyTask M1 1 */ 
                STC_SetSpeedSensor(pSTC[M1], ${SPD_M1}._Super); /* Observer has converged */
                FOC_InitAdditionalMethods(M1);
                FOC_CalcCurrRef(M1);
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
                Mci[M1].State = RUN;
              }
  <#if MC.MOTOR_PROFILER == false>
            }
  </#if><#--  MC.MOTOR_PROFILER == false -->
          }
          break;
        }
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") -->

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */
            
            /* USER CODE END MediumFrequencyTask M1 2 */
       
<#if  MC.M1_POSITION_CTRL_ENABLING == true >
            TC_PositionRegulation(pPosCtrl[M1]);
</#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->
            MCI_ExecBufferedCommands(&Mci[M1]);
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
            if (mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE)
            {
</#if> <#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
              FOC_CalcCurrRef(M1);
<#if MC.M1_SPEED_FEEDBACK_CHECK == true || (MC.M1_SPEED_SENSOR == "HALL_SENSOR")>
              if(!IsSpeedReliable)
              {
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
              }
              else
              {
                /* Nothing to do */
              }
</#if><#-- MC.M1_SPEED_FEEDBACK_CHECK == true || (MC.M1_SPEED_SENSOR == "HALL_SENSOR") -->
<#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
            }
            else
            {
              int16_t hForcedMecSpeedUnit;
              /* Open Loop */
              VSS_CalcAvrgMecSpeedUnit( &VirtualSpeedSensorM1, &hForcedMecSpeedUnit);
              OL_Calc(pOpenLoop[M1]);
            }   
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
<#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true>
            OTT_MF(&OTT);
</#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true -->
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {

<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC")>
            STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);    /* Sensor-less */
            VSS_Clear(&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC")  -->
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */
    
            /* USER CODE END MediumFrequencyTask M1 5 */
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        
        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        
<#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true || M1_ENCODER>
        case WAIT_STOP_MOTOR:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
  <#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true>
            if (0 == SCC_DetectBemf(&SCC))
            {
              /* In a sensorless configuration. Initiate the Revup procedure */
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);
               ${SPD_clear_M1}(${SPD_M1});
              FOC_Clear(M1);
              SCC_Start(&SCC);
              /* The generic function needs to be called here as the undelying   
               * implementation changes in time depending on the Profiler's state 
               * machine. Calling the generic function ensures that the correct
               * implementation is invoked */
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
              Mci[M1].State = START;
            }
            else
            {
              /* Nothing to do */
            }
  <#elseif (MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z")>
            if (TSK_StopPermanencyTimeHasElapsedM1())
            {
              ENC_Clear(&ENCODER_M1);
              ${M1_CS_TOPO}SwitchOnPWM(pwmcHandle[M1]);
    <#if MC.M1_POSITION_CTRL_ENABLING == true>
              TC_EncAlignmentCommand(pPosCtrl[M1]);
    </#if><#-- MC.M1_POSITION_CTRL_ENABLING == true -->
              FOC_InitAdditionalMethods(M1);
              STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
              MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
              FOC_CalcCurrRef(M1);
              Mci[M1].State = RUN;
            } 
            else
            {
              /* Nothing to do */
            }
  <#elseif (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z")>
            if (TSK_StopPermanencyTimeHasElapsedM1())
            {
              RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
              ${SPD_clear_M1}(${SPD_M1});
              ENC_Clear(&ENCODER_M1);
              VSS_Clear(&VirtualSpeedSensorM1);
              ${M1_CS_TOPO}SwitchOnPWM(pwmcHandle[M1]);
              Mci[M1].State = START;
            } 
            else
            {
              /* Nothing to do */
            }
  </#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true -->
          }
          break;
        }
</#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true || M1_ENCODER -->

        default:
          break;
       }
    }  
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
<#if MC.MOTOR_PROFILER>
   <#if MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR">
  HT_MF(&HT);
   </#if><#-- MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->
  SCC_MF(&SCC);
</#if><#-- MC.MOTOR_PROFILER == true -->
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
  MC_ControlMode_t mode;
  
  mode = MCI_GetControlMode( &Mci[bMotor] );
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
    
  ab_t NULL_ab = {((int16_t)0), ((int16_t)0)};
  qd_t NULL_qd = {((int16_t)0), ((int16_t)0)};
  alphabeta_t NULL_alphabeta = {((int16_t)0), ((int16_t)0)};
  
  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
  if ( mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE)
  {
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
    FOCVars[bMotor].Iqdref = NULL_qd;
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
  }
  else
  {
    /* Nothing to do */
  }
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], ((int32_t)0));
  PID_SetIntegralTerm(pPIDId[bMotor], ((int32_t)0));

  STC_Clear(pSTC[bMotor]);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  <#if (MC.M1_FLUX_WEAKENING_ENABLING == true) || (MC.M2_FLUX_WEAKENING_ENABLING == true)>
  if (NULL == pFW[bMotor])
  {
    /* Nothing to do */
  }
  else
  {
    FW_Clear(pFW[bMotor]);
  }
  </#if><#-- (MC.M1_FLUX_WEAKENING_ENABLING == true) || (MC.M2_FLUX_WEAKENING_ENABLING == true) -->
  <#if (MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true) || ( MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true)>
  if (NULL == pFF[bMotor])
  {
    /* Nothing to do */
  }
  else
  {
    FF_Clear(pFF[bMotor]);
  }
  </#if><#-- (MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true) || ( MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true) -->

  <#if DWT_CYCCNT_SUPPORTED>
    <#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Clear(&PerfTraces,bMotor);
    </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
  </#if><#-- DWT_CYCCNT_SUPPORTED -->
  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor) //cstat !RED-func-no-effect
{
    if (M_NONE == bMotor)
    {
      /* Nothing to do */
    }
    else
    {
  <#if (MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true) || ( MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true)>
      if (NULL == pFF[bMotor])
      {
        /* Nothing to do */
      }
      else
      {
        FF_InitFOCAdditionalMethods(pFF[bMotor]);
      }
  </#if><#-- (MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true) || ( MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true) -->
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
    }
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
  qd_t IqdTmp;

  /* Enter critical section */ 
  /* Disable interrupts to avoid any interruption during Iqd reference latching */
  /* to avoid MF task writing them while HF task reading them */
  __disable_irq();
  IqdTmp = FOCVars[bMotor].Iqdref;

  /* Exit critical section */
  __enable_irq();
   
  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true>
  MC_ControlMode_t mode;
  
  mode = MCI_GetControlMode( &Mci[bMotor] );
  if (INTERNAL == FOCVars[bMotor].bDriveInput
               && (mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE))
  <#else><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == false && MC.M2_DBG_OPEN_LOOP_ENABLE == false -->
  if (INTERNAL == FOCVars[bMotor].bDriveInput)
  </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true || MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    IqdTmp.q = FOCVars[bMotor].hTeref;
  <#if (MC.M1_MTPA_ENABLING == true) || (MC.M2_MTPA_ENABLING == true)>
    if (0 == pMaxTorquePerAmpere[bMotor])
    {
      /* Nothing to do */
    }
    else
    {
      MTPA_CalcCurrRefFromIq(pMaxTorquePerAmpere[bMotor], &IqdTmp);
    }
  </#if><#-- (MC.M1_MTPA_ENABLING == true) || (MC.M2_MTPA_ENABLING == true) -->

  <#if (MC.M1_FLUX_WEAKENING_ENABLING == true) || (MC.M2_FLUX_WEAKENING_ENABLING == true)>
    if (NULL == pFW[bMotor])
    {
      /* Nothing to do */
    }
    else
    {
    <#if MC.DRIVE_NUMBER == "1">
      <#if (MC.M1_MTPA_ENABLING == true)>
      IqdTmp = FW_CalcCurrRef(pFW[bMotor], IqdTmp);
      <#else><#-- (MC.M1_MTPA_ENABLING == false) -->
      IqdTmp.d = FOCVars[bMotor].UserIdref;
      IqdTmp = FW_CalcCurrRef(pFW[bMotor], IqdTmp);
      </#if><#-- (MC.M1_MTPA_ENABLING == true) -->
    <#else><#-- MC.DRIVE_NUMBER == 1 -->
      <#if (MC.M1_MTPA_ENABLING == true) &&  (MC.M2_MTPA_ENABLING == true)>
      IqdTmp = FW_CalcCurrRef(pFW[bMotor], IqdTmp);
      <#elseif (MC.M1_MTPA_ENABLING == false) && (MC.M2_MTPA_ENABLING == false)>
      IqdTmp.d = FOCVars[bMotor].UserIdref;
      IqdTmp = FW_CalcCurrRef(pFW[bMotor], IqdTmp);
      <#else><#-- ((MC.M1_MTPA_ENABLING == true) &&  (MC.M2_MTPA_ENABLING == false)
                || (MC.M1_MTPA_ENABLING == false) && (MC.M2_MTPA_ENABLING == true)) -->
      if (0 == pMaxTorquePerAmpere[bMotor])
      {
        IqdTmp.d = FOCVars[bMotor].UserIdref;
      }
      else
      {
        /* Use Id reference from MTPA */
      }     
      IqdTmp = FW_CalcCurrRef(pFW[bMotor], IqdTmp);
      </#if><#-- (MC.M1_MTPA_ENABLING == true) &&  (MC.M2_MTPA_ENABLING == true) -->
    </#if><#-- MC.DRIVE_NUMBER == 1 -->
    }
  </#if><#-- (MC.M1_FLUX_WEAKENING_ENABLING == true) || (MC.M2_FLUX_WEAKENING_ENABLING == true) -->
  <#if (MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true) || ( MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true)>
    if (NULL == pFF[bMotor])
    {
      /* Nothing to do */
    }
    else
    {
      FF_VqdffComputation(pFF[bMotor], IqdTmp, pSTC[bMotor]);
    }
  </#if><#-- (MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true) || ( MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true) -->
  }
  else
  {
    /* Nothing to do */
  }

  /* Enter critical section */ 
  /* Disable interrupts to avoid any interruption during Iqd reference restoring */
  __disable_irq();
  FOCVars[bMotor].Iqdref = IqdTmp;

  /* Exit critical section */
  __enable_irq();
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}

<#if MC.DRIVE_NUMBER != "1">
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 2 according to the 
  * present state of its state machine. In particular, duties requiring a periodic 
  * execution at a medium frequency rate (such as the speed controller for instance) 
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM2(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M2 0 */

  /* USER CODE END MediumFrequencyTask M2 0 */

  int16_t wAux = 0;
<#if (MC.M2_PWM_DRIVER_PN == "STDRIVE101") && (MC.M2_DP_TOPOLOGY != "NONE") && (MC.M2_OTF_STARTUP == true)>
  PWMC_${M2_CS_TOPO}Handle_t *p_Handle = (PWMC_${M2_CS_TOPO}Handle_t *)pwmcHandle[M2];
</#if><#-- (MC.M2_PWM_DRIVER_PN == "STDRIVE101") && (MC.M2_DP_TOPOLOGY != "NONE") && (MC.M2_OTF_STARTUP == true) -->
<#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
  MC_ControlMode_t mode;
  mode = MCI_GetControlMode(&Mci[M2]);
</#if><#-- MC.M2_DBG_OPEN_LOOP_ENABLE == true -->

<#if MC.M2_ICL_ENABLED == true>
  uint16_t Vbus_M2 = VBS_GetAvBusVoltage_V(&(BusVoltageSensor_M2._Super));
  ICL_State_t ICLstate = ICL_Exec(&ICL_M2, Vbus_M2);
</#if><#-- MC.M2_ICL_ENABLED == true -->
<#if AUX_SPEED_FDBK_M2 == true>
  (void)${SPD_aux_calcAvrgMecSpeedUnit_M2}(${SPD_AUX_M2}, &wAux);
</#if><#-- AUX_SPEED_FDBK_M2 == true -->
<#if MC.M2_SPEED_FEEDBACK_CHECK == true || MC.M2_SPEED_SENSOR == "HALL_SENSOR">
  bool IsSpeedReliable = ${SPD_calcAvrgMecSpeedUnit_M2}(${SPD_M2}, &wAux);
<#else><#-- MC.M2_SPEED_FEEDBACK_CHECK = false || MC.M2_SPEED_SENSOR != "HALL_SENSOR" -->
  (void)${SPD_calcAvrgMecSpeedUnit_M2}(${SPD_M2}, &wAux);
</#if><#-- MC.M2_SPEED_FEEDBACK_CHECK == true || MC.M2_SPEED_SENSOR == "HALL_SENSOR" -->
  PQD_CalcElMotorPower(pMPM[M2]);

<#if MC.M2_ICL_ENABLED == true>
  if ( !ICLFaultTreatedM2 && (ICLstate == ICL_ACTIVE))
  {
    ICLFaultTreatedM2 = true;
  }
  else
  {
    /* Nothing to do */
  }
</#if><#-- MC.M2_ICL_ENABLED == true -->

<#if MC.M2_ICL_ENABLED == true>
  if ((MCI_GetCurrentFaults(&Mci[M2]) == MC_NO_FAULTS) && ICLFaultTreatedM2)
<#else><#-- MC.M2_ICL_ENABLED == false -->
  if (MCI_GetCurrentFaults(&Mci[M2]) == MC_NO_FAULTS)
</#if><#-- MC.M2_ICL_ENABLED == true -->
  {
    if (MCI_GetOccurredFaults(&Mci[M2]) == MC_NO_FAULTS)
    {
      switch (Mci[M2].State)
      {
<#if MC.M2_ICL_ENABLED == true>
        case ICLWAIT:
        {
          if (ICL_INACTIVE == ICLstate)
          {
            /* If ICL is Inactive, move to IDLE */
            Mci[M2].State = IDLE;
          }
          break;
        }
</#if><#-- MC.M2_ICL_ENABLED == true -->
        
        case IDLE:
        {
          if ((MCI_START == Mci[M2].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M2].DirectCommand))
          {
<#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")>
  <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
            if ( mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE) 
            {
  </#if><#-- MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
              RUC_Clear(&RevUpControlM2, MCI_GetImposedMotorDirection(&Mci[M2]));
  <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
            }
            else
            {
              /* Nothing to do */
            }
  </#if><#-- MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
</#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")-->

            if (pwmcHandle[M2]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M2], CRC_START);
              Mci[M2].State = OFFSET_CALIB;
            }
            else
            {
             /* Calibration already done. Enables only TIM channels */
             pwmcHandle[M2]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M2], CRC_EXEC);

<#if CHARGE_BOOT_CAP_ENABLING2 == true>
  <#if (MC.M2_PWM_DRIVER_PN == "STDRIVE101") && (MC.M2_DP_TOPOLOGY != "NONE")>  
    <#if MC.M2_DP_DESTINATION == "TIM_BKIN">
              LL_TIM_DisableBRK(${_last_word(MC.M2_PWM_TIMER_SELECTION)});
    <#elseif MC.M2_DP_DESTINATION == "TIM_BKIN2" >
              LL_TIM_DisableBRK2(${_last_word(MC.M2_PWM_TIMER_SELECTION)});
    </#if><#-- MC.M2_DP_DESTINATION == "TIM_BKIN" -->
  </#if> <#-- (MC.M2_PWM_DRIVER_PN == "STDRIVE101") && (MC.M2_DP_TOPOLOGY != "NONE") -->
  <#if (MC.M2_OTF_STARTUP == true)>
              ${M2_CS_TOPO}TurnOnLowSides(pwmcHandle[M2],p_Handle->Half_PWMPeriod-1);
  <#else><#-- !M2_OTF_STARTUP -->
              ${M2_CS_TOPO}TurnOnLowSides(pwmcHandle[M2],M2_CHARGE_BOOT_CAP_DUTY_CYCLES);
  </#if><#-- M2_OTF_STARTUP -->
              TSK_SetChargeBootCapDelayM2(M2_CHARGE_BOOT_CAP_TICKS);
              Mci[M2].State = CHARGE_BOOT_CAP;
<#else><#-- CHARGE_BOOT_CAP_ENABLING2 == false -->
<#-- test sensorless -->
              FOCVars[M2].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor(pSTC[M2], &VirtualSpeedSensorM2._Super);
              ${SPD_clear_M2}(${SPD_M2});
              FOC_Clear(M2);
              ${M2_CS_TOPO}SwitchOnPWM(pwmcHandle[M2]);
              Mci[M2].State = START;
</#if><#-- CHARGE_BOOT_CAP_ENABLING2 == true -->
            }
<#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true>
            OTT_Clear(&OTT);
</#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true -->
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
            if (PWMC_CurrentReadingCalibr(pwmcHandle[M2], CRC_EXEC))
            {
              if (MCI_MEASURE_OFFSETS == Mci[M2].DirectCommand)
              {
                FOC_Clear(M2);
                Mci[M2].DirectCommand = MCI_NO_COMMAND;
                Mci[M2].State = IDLE;
              }
              else
                {
  <#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true>
                Mci[M2].State = WAIT_STOP_MOTOR;
  <#else><#-- MC.MOTOR_PROFILER == false || MC.ONE_TOUCH_TUNING == false -->
    <#if CHARGE_BOOT_CAP_ENABLING2 == true>
      <#if (MC.M2_PWM_DRIVER_PN == "STDRIVE101") && (MC.M2_DP_TOPOLOGY != "NONE")>
        <#if MC.M2_DP_DESTINATION == "TIM_BKIN">
                LL_TIM_DisableBRK(${_last_word(MC.M2_PWM_TIMER_SELECTION)});
        <#elseif MC.M2_DP_DESTINATION == "TIM_BKIN2" >
                LL_TIM_DisableBRK2(${_last_word(MC.M2_PWM_TIMER_SELECTION)});
        </#if><#-- MC.M2_DP_DESTINATION == "TIM_BKIN" -->
      </#if>   <#-- (MC.M2_PWM_DRIVER_PN == "STDRIVE101") && (MC.M2_DP_TOPOLOGY != "NONE") -->
      <#if (MC.M2_OTF_STARTUP == true)>
                ${M2_CS_TOPO}TurnOnLowSides(pwmcHandle[M2],p_Handle->Half_PWMPeriod-1);
      <#else><#-- !M2_OTF_STARTUP -->
                ${M2_CS_TOPO}TurnOnLowSides(pwmcHandle[M2],M2_CHARGE_BOOT_CAP_DUTY_CYCLES);
      </#if><#-- M2_OTF_STARTUP -->
                TSK_SetChargeBootCapDelayM2(M2_CHARGE_BOOT_CAP_TICKS);
                Mci[M2].State = CHARGE_BOOT_CAP;
    <#else><#-- CHARGE_BOOT_CAP_ENABLING2 == true -->
                FOCVars[M2].bDriveInput = EXTERNAL;
                STC_SetSpeedSensor(pSTC[M2], &VirtualSpeedSensorM2._Super);
                ${SPD_clear_M2}(${SPD_M2});
                FOC_Clear(M2);
        <#if MC.M2_DISCONTINUOUS_PWM == true>
                /* Enable DPWM mode before Start */
                PWMC_DPWM_ModeEnable(pwmcHandle[M2]);
        </#if><#-- MC.M2_DISCONTINUOUS_PWM == true -->
                ${M2_CS_TOPO}SwitchOnPWM(pwmcHandle[M2]);
                Mci[M2].State = START;
    </#if><#-- CHARGE_BOOT_CAP_ENABLING2 == true -->
  </#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true -->
              }
            }
            else
            {
              /* Nothing to be done, FW waits for offset calibration to finish */
            }
          }  
          break;
        }

<#if (CHARGE_BOOT_CAP_ENABLING2 == true)>
        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM2())
            {
              ${M2_CS_TOPO}SwitchOffPWM(pwmcHandle[M2]);
  <#if (MC.M2_PWM_DRIVER_PN == "STDRIVE101") && (MC.M2_DP_TOPOLOGY != "NONE")>
    <#if MC.M2_DP_DESTINATION == "TIM_BKIN">
              LL_TIM_ClearFlag_BRK(${_last_word(MC.M2_PWM_TIMER_SELECTION)});
              LL_TIM_EnableBRK(${_last_word(MC.M2_PWM_TIMER_SELECTION)});
    <#elseif MC.M2_DP_DESTINATION == "TIM_BKIN2">
              LL_TIM_ClearFlag_BRK2(${_last_word(MC.M2_PWM_TIMER_SELECTION)});
              LL_TIM_EnableBRK2(${_last_word(MC.M2_PWM_TIMER_SELECTION)});
    </#if><#-- MC.M2_DP_DESTINATION == "TIM_BKIN" -->
  </#if><#-- (MC.M2_PWM_DRIVER_PN == "STDRIVE101") && (MC.M2_DP_TOPOLOGY != "NONE") -->
<#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") || M2_ENCODER>
              FOCVars[M2].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M2], &VirtualSpeedSensorM2._Super );
</#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") || M2_ENCODER -->
              ${SPD_clear_M2}(${SPD_M2});
<#if AUX_SPEED_FDBK_M2 == true>
              ${SPD_aux_clear_M2}(${SPD_AUX_M2});
</#if><#-- AUX_SPEED_FDBK_M2 == true -->
<#if MC.M1_OVERMODULATION == true>
              PWMC_Clear(pwmcHandle[M2]);
</#if><#-- MC.M1_OVERMODULATION == true -->
<#if MC.M2_DISCONTINUOUS_PWM == true>
              /* Enable DPWM mode before Start */
              PWMC_DPWM_ModeEnable( pwmcHandle[M2]);
</#if><#-- MC.M2_DISCONTINUOUS_PWM == true -->
		FOC_Clear( M2 );
    <#if (MC.M2_OTF_STARTUP == true)>
              ${M2_CS_TOPO}SwitchOnPWM(pwmcHandle[M2]);
    </#if><#-- MC.M2_OTF_STARTUP == true -->
     
<#if (MC.MOTOR_PROFILER == true)>
        SCC_Start(&SCC);
              /* The generic function needs to be called here as the undelying   
               * implementation changes in time depending on the Profiler's state 
               * machine. Calling the generic function ensures that the correct
               * implementation is invoked */
              PWMC_SwitchOnPWM(pwmcHandle[M2]);
              Mci[M2].State = START;
<#else><#-- MC.MOTOR_PROFILER == false -->
 
  <#if (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")>
    <#if MC.M2_DBG_OPEN_LOOP_ENABLE>
              if (mode == MCM_OPEN_LOOP_VOLTAGE_MODE || mode == MCM_OPEN_LOOP_CURRENT_MODE)
              { 
                MCI_ExecBufferedCommands(&Mci[M2]); /* Exec the speed ramp even in OL*/
                Mci[M2].State = RUN;              
              }
              else
              {
    </#if>  
              if (EAC_IsAligned(&EncAlignCtrlM2) == false)
              {
                EAC_StartAlignment(&EncAlignCtrlM2);  
                Mci[M2].State = ALIGNMENT;
              }
              else
              {
                STC_SetControlMode(pSTC[M2], MCM_SPEED_MODE);
                STC_SetSpeedSensor(pSTC[M2], &ENCODER_M2._Super);
                FOC_InitAdditionalMethods(M2);
                FOC_CalcCurrRef(M2);
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M2]); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M2]); /* Exec the speed ramp after changing of the speed sensor */
                Mci[M2].State = RUN;
    <#if MC.M2_DBG_OPEN_LOOP_ENABLE >
              }
    </#if>				
              }
  <#elseif  (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z")>
              if (EAC_IsAligned(&EncAlignCtrlM2) == false)
              {
                EAC_StartAlignment(&EncAlignCtrlM2);
                Mci[M2].State = ALIGNMENT;
              }
              else
              {
                VSS_Clear(&VirtualSpeedSensorM2); /* Reset measured speed in IDLE */
                FOC_Clear(M2);
                Mci[M2].State = START;
              }
  <#elseif MC.M2_SPEED_SENSOR == "HALL_SENSOR">
       <#if MC.M2_DBG_OPEN_LOOP_ENABLE> 
              if (mode == MCM_OPEN_LOOP_VOLTAGE_MODE || mode == MCM_OPEN_LOOP_CURRENT_MODE)
              { /* In open loop mode, angle comes from virtual sensor */
                STC_SetSpeedSensor( pSTC[M2], &VirtualSpeedSensorM2._Super );
              }
              else 
              {
                STC_SetSpeedSensor( pSTC[M1], &HALL_M2._Super );
      </#if> <#-- M1_DBG_OPEN_LOOP_ENABLE -->
                FOC_InitAdditionalMethods(M2);
                FOC_CalcCurrRef(M2);
                STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M2]); /* Init the reference speed to current speed */
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE>
              }
    </#if>				
              MCI_ExecBufferedCommands(&Mci[M2]); /* Exec the speed ramp after changing of the speed sensor */
              Mci[M2].State = RUN;		  
  <#else><#-- sensorless mode only -->
    <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
              if (MCM_OPEN_LOOP_VOLTAGE_MODE == mode || MCM_OPEN_LOOP_CURRENT_MODE == mode)
              {
                Mci[M2].State = RUN;
              }
              else
              {
    </#if><#-- MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
                Mci[M2].State = START;
    <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
              }
    </#if><#-- MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
  </#if><#-- (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z") -->
       
</#if><#-- MC.MOTOR_PROFILER == true -->

<#if (MC.M2_OTF_STARTUP == false)>
              PWMC_SwitchOnPWM(pwmcHandle[M2]);

</#if><#-- (MC.M2_OTF_STARTUP == false) -->
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }
</#if><#-- CHARGE_BOOT_CAP_ENABLING2 == true -->

<#if M2_ENCODER><#-- only for encoder -->
        case ALIGNMENT:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
            bool isAligned = EAC_IsAligned(&EncAlignCtrlM2);
            bool EACDone = EAC_Exec(&EncAlignCtrlM2);
            if ((isAligned == false)  && (EACDone == false))
            {
              qd_t IqdRef;
              IqdRef.q = 0;
              IqdRef.d = STC_CalcTorqueReference(pSTC[M2]);
              FOCVars[M2].Iqdref = IqdRef;
            }
            else
            {
              ${M2_CS_TOPO}SwitchOffPWM( pwmcHandle[M2] );
  <#if (MC.M2_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M2_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER_Z")>
              STC_SetControlMode(pSTC[M2], MCM_SPEED_MODE);
              STC_SetSpeedSensor(pSTC[M2], &ENCODER_M2._Super);
  </#if><#-- (MC.M2_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER") && (MC.M2_AUXILIARY_SPEED_SENSOR != "QUAD_ENCODER_Z") -->
              FOC_Clear(M2);
              TSK_SetStopPermanencyTimeM2(STOPPERMANENCY_TICKS);
  <#if (CHARGE_BOOT_CAP_ENABLING == true)>
              ${M2_CS_TOPO}TurnOnLowSides(pwmcHandle[M2],M2_CHARGE_BOOT_CAP_DUTY_CYCLES);
  </#if><#-- CHARGE_BOOT_CAP_ENABLING2 == true -->
              Mci[M2].State = WAIT_STOP_MOTOR;
              /* USER CODE BEGIN MediumFrequencyTask M2 EndOfEncAlignment */
            
              /* USER CODE END MediumFrequencyTask M2 EndOfEncAlignment */
            }
          }
          break;
        }
</#if><#-- M2_ENCODER -->

<#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")>
        case START:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
          <#-- only for sensor-less control -->
            /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
            int16_t hForcedMecSpeedUnit;
            qd_t IqdRef;
            bool ObserverConverged;

            /* Execute the Rev Up procedure */
  <#if MC.M2_OTF_STARTUP == true>
            if (! RUC_OTF_Exec(&RevUpControlM2))
  <#else><#-- MC.M2_OTF_STARTUP == false -->
            if(! RUC_Exec(&RevUpControlM2))
  </#if><#-- MC.M2_OTF_STARTUP == true -->
            {
            /* The time allowed for the startup sequence has expired */
  <#if MC.MOTOR_PROFILER == true>
              /* However, no error is generated when OPEN LOOP is enabled 
               * since then the system does not try to close the loop... */
  <#else><#-- MC.MOTOR_PROFILER == false -->
              MCI_FaultProcessing(&Mci[M2], MC_START_UP, 0);
  </#if><#-- MC.MOTOR_PROFILER == true -->
            }
            else
            {
              /* Execute the torque open loop current start-up ramp:
               * Compute the Iq reference current as configured in the Rev Up sequence */
              IqdRef.q = STC_CalcTorqueReference(pSTC[M2]);
              IqdRef.d = FOCVars[M2].UserIdref;
              /* Iqd reference current used by the High Frequency Loop to generate the PWM output */
              FOCVars[M2].Iqdref = IqdRef;
           }
          
            (void)VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM2, &hForcedMecSpeedUnit);
         
  <#if MC.M2_OTF_STARTUP == false && MC.MOTOR_PROFILER == false>
            /* Check that startup stage where the observer has to be used has been reached */
            if (true == RUC_FirstAccelerationStageReached(&RevUpControlM2))
            {
  </#if><#-- MC.M2_OTF_STARTUP == false) && (MC.MOTOR_PROFILER == false -->
  <#if (MC.M2_SPEED_SENSOR == "STO_PLL")>
              ObserverConverged = STO_PLL_IsObserverConverged(&STO_PLL_M2, &hForcedMecSpeedUnit);
              STO_SetDirection(&STO_PLL_M2, (int8_t)MCI_GetImposedMotorDirection(&Mci[M2]));
  <#elseif MC.M2_SPEED_SENSOR == "STO_CORDIC">
              ObserverConverged = STO_CR_IsObserverConverged(&STO_CR_M2, hForcedMecSpeedUnit);
  </#if> <#-- (MC.M2_SPEED_SENSOR == "STO_PLL")     -->
              (void)VSS_SetStartTransition(&VirtualSpeedSensorM2, ObserverConverged);
  <#if MC.M2_OTF_STARTUP == false && MC.MOTOR_PROFILER == false>
            }
            else
            {
              ObserverConverged = false;
            }
  </#if><#-- MC.M2_OTF_STARTUP == false) && (MC.MOTOR_PROFILER == false -->
            if (ObserverConverged)
            {
              qd_t StatorCurrent = MCM_Park(FOCVars[M2].Ialphabeta, SPD_GetElAngle(${SPD_M2}._Super));

              /* Start switch over ramp. This ramp will transition from the revup to the closed loop FOC */
              REMNG_Init(pREMNG[M2]);
              (void)REMNG_ExecRamp(pREMNG[M2], FOCVars[M2].Iqdref.q, 0);
              (void)REMNG_ExecRamp(pREMNG[M2], StatorCurrent.q, TRANSITION_DURATION);

              Mci[M2].State = SWITCH_OVER;
            }
          }
          break;
        }

        case SWITCH_OVER:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
            bool LoopClosed;
            int16_t hForcedMecSpeedUnit;

  <#if MC.MOTOR_PROFILER == false><#-- No need to call RUC_Exec() when in MP in this state -->
    <#if MC.M2_OTF_STARTUP == true>
            if (! RUC_OTF_Exec(&RevUpControlM2))
    <#else><#-- MC.M2_OTF_STARTUP == false -->
            if (! RUC_Exec(&RevUpControlM2))
    </#if><#-- MC.M2_OTF_STARTUP == true -->
            {
              /* The time allowed for the startup sequence has expired */
              MCI_FaultProcessing(&Mci[M2], MC_START_UP, 0);
            } 
            else
            {
  </#if><#--  MC.MOTOR_PROFILER == false -->
              /* Compute the virtual speed and positions of the rotor. 
                 The function returns true if the virtual speed is in the reliability range */
              LoopClosed = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM2, &hForcedMecSpeedUnit);
              /* Check if the transition ramp has completed. */
              bool tempBool;
              tempBool = VSS_TransitionEnded(&VirtualSpeedSensorM2);
              LoopClosed = LoopClosed || tempBool;
              
              /* If any of the above conditions is true, the loop is considered closed. 
                 The state machine transitions to the RUN state */
              if (true ==  LoopClosed) 
              {
#if PID_SPEED_INTEGRAL_INIT_DIV == 0
                PID_SetIntegralTerm(&PIDSpeedHandle_M2, 0);
#else
                PID_SetIntegralTerm(&PIDSpeedHandle_M2,
                                    (((int32_t)FOCVars[M2].Iqdref.q * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M2)) 
                                    / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
  <#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true>
                OTT_SR(&OTT);
  </#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true -->
                /* USER CODE BEGIN MediumFrequencyTask M2 1 */

                /* USER CODE END MediumFrequencyTask M2 1 */ 
                STC_SetSpeedSensor(pSTC[M2], ${SPD_M2}._Super); /* Observer has converged */
                FOC_InitAdditionalMethods(M2);
                FOC_CalcCurrRef(M2);
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M2]); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M2]); /* Exec the speed ramp after changing of the speed sensor */
                Mci[M2].State = RUN;
              }
  <#if MC.MOTOR_PROFILER == false>
            }
  </#if><#--  MC.MOTOR_PROFILER == false -->
          }
          break;
        }
</#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")-->

        case RUN:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M2 2 */
            
            /* USER CODE END MediumFrequencyTask M2 2 */
       
<#if  MC.M2_POSITION_CTRL_ENABLING == true >
            TC_PositionRegulation(pPosCtrl[M2]);
</#if><#-- MC.M2_POSITION_CTRL_ENABLING == true -->
            MCI_ExecBufferedCommands(&Mci[M2]);
<#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
            if (mode != MCM_OPEN_LOOP_VOLTAGE_MODE && mode != MCM_OPEN_LOOP_CURRENT_MODE)
            {
</#if> <#-- MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
              FOC_CalcCurrRef(M2);
         
<#if MC.M2_SPEED_FEEDBACK_CHECK == true || (MC.M2_SPEED_SENSOR == "HALL_SENSOR")>
              if(!IsSpeedReliable)
              {
                MCI_FaultProcessing(&Mci[M2], MC_SPEED_FDBK, 0);
              }
              else
              {
                /* Nothing to do */
              }
</#if><#-- MC.M2_SPEED_FEEDBACK_CHECK == true || (MC.M2_SPEED_SENSOR == "HALL_SENSOR") -->
<#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
            }
            else
            {
              int16_t hForcedMecSpeedUnit;
              /* Open Loop */
              VSS_CalcAvrgMecSpeedUnit( &VirtualSpeedSensorM2, &hForcedMecSpeedUnit);
              OL_Calc(pOpenLoop[M2]);
            }   
</#if><#-- MC.M2_DBG_OPEN_LOOP_ENABLE == true -->
<#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true>
            OTT_MF(&OTT);
</#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true -->
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM2())
          {

<#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")>
            STC_SetSpeedSensor(pSTC[M2], &VirtualSpeedSensorM2._Super);    /* Sensor-less */
            VSS_Clear(&VirtualSpeedSensorM2); /* Reset measured speed in IDLE */
</#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") -->
            /* USER CODE BEGIN MediumFrequencyTask M2 5 */
    
            /* USER CODE END MediumFrequencyTask M2 5 */
            Mci[M2].DirectCommand = MCI_NO_COMMAND;
            Mci[M2].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M2].DirectCommand)
          {
            Mci[M2].DirectCommand = MCI_NO_COMMAND;
            Mci[M2].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        
        case FAULT_NOW:
        {
          Mci[M2].State = FAULT_OVER;
          break;
        }

        
<#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true || M2_ENCODER>
        case WAIT_STOP_MOTOR:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
  <#if MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true>
            if (0 == SCC_DetectBemf(&SCC))
            {
              /* In a sensorless configuration. Initiate the Revup procedure */
              FOCVars[M2].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor(pSTC[M2], &VirtualSpeedSensorM2._Super);
               ${SPD_clear_M2}(${SPD_M2});
              FOC_Clear(M2);
              SCC_Start(&SCC);
              /* The generic function needs to be called here as the undelying   
               * implementation changes in time depending on the Profiler's state 
               * machine. Calling the generic function ensures that the correct
               * implementation is invoked */
              PWMC_SwitchOnPWM(pwmcHandle[M2]);
              Mci[M2].State = START;
            }
            else
            {
              /* Nothing to do */
            }
  <#elseif (MC.M2_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_SPEED_SENSOR == "QUAD_ENCODER_Z")>
            if (TSK_StopPermanencyTimeHasElapsedM2())
            {
              ENC_Clear(&ENCODER_M2);
              ${M2_CS_TOPO}SwitchOnPWM(pwmcHandle[M2]);
    <#if MC.M2_POSITION_CTRL_ENABLING == true>
              TC_EncAlignmentCommand(pPosCtrl[M2]);
    </#if><#-- MC.M2_POSITION_CTRL_ENABLING == true -->
              FOC_InitAdditionalMethods(M2);
              STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M2]); /* Init the reference speed to current speed */
              MCI_ExecBufferedCommands(&Mci[M2]); /* Exec the speed ramp after changing of the speed sensor */
              FOC_CalcCurrRef(M2);
              Mci[M2].State = RUN;
            } 
            else
            {
              /* Nothing to do */
            }
  <#elseif (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M2_AUXILIARY_SPEED_SENSOR == "QUAD_ENCODER_Z")>
            if (TSK_StopPermanencyTimeHasElapsedM2())
            {
              RUC_Clear(&RevUpControlM2, MCI_GetImposedMotorDirection(&Mci[M2]));
              ${SPD_clear_M2}(${SPD_M2});
              ENC_Clear(&ENCODER_M2);
              VSS_Clear(&VirtualSpeedSensorM2);
              ${M2_CS_TOPO}SwitchOnPWM(pwmcHandle[M2]);
              Mci[M2].State = START;
            } 
            else
            {
              /* Nothing to do */
            }
  </#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true -->
          }
          break;
        }
</#if><#-- MC.MOTOR_PROFILER == true || MC.ONE_TOUCH_TUNING == true || M2_ENCODER -->

        default:
          break;
       }
    }  
    else
    {
      Mci[M2].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M2].State = FAULT_NOW;
  }
<#if MC.MOTOR_PROFILER>
   <#if MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR">
  HT_MF(&HT);
   </#if><#-- MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->
  SCC_MF(&SCC);
</#if><#-- MC.MOTOR_PROFILER == true -->
  /* USER CODE BEGIN MediumFrequencyTask M2 6 */

  /* USER CODE END MediumFrequencyTask M2 6 */
}

</#if><#-- MC.DRIVE_NUMBER > 1 -->

<#if MC.MOTOR_PROFILER == false>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing.
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control 
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */
__weak uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
  uint16_t hFOCreturn;
<#if MC.DRIVE_NUMBER != "1">
  <#if (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||  (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||
      (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") ||  (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
  Observer_Inputs_t STO_aux_Inputs;
  </#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||  (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||
            (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") ||  (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->
</#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||  (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||
            (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") ||  (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->			
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */

<#if MC.TESTENV == true && MC.PFC_ENABLED == false >
  /* Performance Measurement: start measure */
  start_perf_measure();
</#if><#-- MC.TESTENV == true && MC.PFC_ENABLED == false -->

<#if MC.DRIVE_NUMBER == "1">
  <#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC")>
  Observer_Inputs_t STO_Inputs; /* Only if sensorless main */
  </#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") -->
  <#if (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
  Observer_Inputs_t STO_aux_Inputs; /* Only if sensorless aux */
  STO_aux_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* Only if sensorless */
  </#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") || (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->

  <#if M1_ENCODER>
  (void)ENC_CalcAngle(&ENCODER_M1);   /* If not sensorless then 2nd parameter is MC_NULL */
  </#if><#-- M1_ENCODER -->
  <#if (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR")>
  (void)HALL_CalcElAngle(&HALL_M1);
  </#if><#-- (MC.M1_SPEED_SENSOR == "HALL_SENSOR") || (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR") -->

  <#if ((MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC"))>
  STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* Only if sensorless */
  if (SWITCH_OVER == Mci[M1].State)
  {
    if (!REMNG_RampCompleted(pREMNG[M1]))
    {
      FOCVars[M1].Iqdref.q = (int16_t)REMNG_Calc(pREMNG[M1]);
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
  </#if><#-- ((MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC")) -->
  <#if ((MC.M1_OTF_STARTUP == true))>
  if(!RUC_Get_SCLowsideOTF_Status(&RevUpControlM1))
  {
    hFOCreturn = FOC_CurrControllerM1();
  }
  else
  {
    hFOCreturn = MC_NO_ERROR;
  }
  <#else><#-- ((MC.M1_OTF_STARTUP == false)) -->
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
  hFOCreturn = FOC_CurrControllerM1();
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
  </#if><#-- ((MC.M1_OTF_STARTUP == true)) -->
  if(hFOCreturn == MC_DURATION)
  {
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }
  else
  {
  <#if MC.M1_SPEED_SENSOR == "STO_PLL">
    bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1);
  </#if>
  <#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC")>
    STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /* Only if sensorless */
    STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /* Only for sensorless */
    (void)${SPD_calcElAngle_M1}(${SPD_M1}, &STO_Inputs);
    ${SPD_calcAvergElSpeedDpp_M1}(${SPD_M1}); /* Only in case of Sensor-less */
    <#if MC.M1_SPEED_SENSOR == "STO_PLL">
    if (false == IsAccelerationStageReached)
    {
      STO_ResetPLL(&STO_PLL_M1);
    }
    else
    {
      /* Nothing to do */
    }
    </#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") -->
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
    /* Only for sensor-less or open loop */
    if((START == Mci[M1].State) || (SWITCH_OVER == Mci[M1].State) || (RUN == Mci[M1].State))
        <#else><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == false -->
    /* Only for sensor-less */
    if((START == Mci[M1].State) || (SWITCH_OVER == Mci[M1].State))
    </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
    {
    <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
      if (START == Mci[M1].State )
      {
        if (0U == RUC_IsAlignStageNow(&RevUpControlM1))
        {
          PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
        }
        else
        {
          PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 1);
        }
      }
      else
      {
        PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
      }
    </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
      int16_t hObsAngle = SPD_GetElAngle(${SPD_M1}._Super);
      (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
    }
    <#else><#-- (MC.M1_SPEED_SENSOR != "STO_PLL") && (MC.M1_SPEED_SENSOR != "STO_CORDIC") -->
        <#if ((MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") || (MC.M1_SPEED_SENSOR == "HALL_SENSOR")) && MC.M1_DBG_OPEN_LOOP_ENABLE == true>
    if(RUN == Mci[M1].State)
    { 
      int16_t hObsAngle = SPD_GetElAngle(${SPD_M1}._Super);
      (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
    }
    else
    {
      /* Nothing to do */
    }
    </#if><#-- ((MC.M1_SPEED_SENSOR == "QUAD_ENCODER") || (MC.M1_SPEED_SENSOR == "QUAD_ENCODER_Z") || (MC.M1_SPEED_SENSOR == "HALL_SENSOR")) && MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
  </#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") -->
   <#if (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") || (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
    STO_aux_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /* Only if sensorless */
    STO_aux_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /* Only for sensorless */
    (void)${SPD_aux_calcElAngle_M1} (${SPD_AUX_M1}, &STO_aux_Inputs);
    ${SPD_aux_calcAvrgElSpeedDpp_M1} (${SPD_AUX_M1});
  </#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") || (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */  
  }

<#else><#-- MC.DRIVE_NUMBER > 1 -->
  <#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_SPEED_SENSOR == "STO_PLL")
        || (MC.M2_SPEED_SENSOR == "STO_CORDIC") >
  Observer_Inputs_t STO_Inputs; /* Only if sensorless main */
  </#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || (MC.M2_SPEED_SENSOR == "STO_PLL")
              || (MC.M2_SPEED_SENSOR == "STO_CORDIC") -->

  if (M1 == bMotorNbr)
  {
  <#if (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
    STO_aux_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* Only if sensorless */
  </#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->

  <#if M1_ENCODER>
    (void)ENC_CalcAngle(&ENCODER_M1);
  </#if><#-- M1_ENCODER -->
  <#if (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR")||( MC.M1_SPEED_SENSOR == "HALL_SENSOR")>
    (void)HALL_CalcElAngle(&HALL_M1);
  </#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR")||( MC.M1_SPEED_SENSOR == "HALL_SENSOR") -->
  <#if ((MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC"))>
    STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;        /* Only if motor0 is sensorless */
    if (SWITCH_OVER == Mci[M1].State)
    {
      if (!REMNG_RampCompleted(pREMNG[M1]))
      {
        FOCVars[M1].Iqdref.q = (int16_t)REMNG_Calc(pREMNG[M1]);
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
      </#if><#-- ((MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC")) -->
      <#if (true == MC.M1_OTF_STARTUP)>
    if(!RUC_Get_SCLowsideOTF_Status(&RevUpControlM1))
    {
      hFOCreturn = FOC_CurrControllerM1();
    }
    else
    {
      hFOCreturn = MC_NO_ERROR;
    }
      <#else><#-- (false == MC.M1_OTF_STARTUP) -->
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_1 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_1 */
    hFOCreturn = FOC_CurrControllerM1();
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_2 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_2 */
      </#if><#-- (true == MC.M1_OTF_STARTUP) -->
  }
  else /* bMotorNbr != M1 */
  {
      <#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")>
    STO_Inputs.Valfa_beta = FOCVars[M2].Valphabeta;      /* Only if motor2 is sensorless */
      </#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") -->
      <#if ((MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL")||(  MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC"))>
    STO_aux_Inputs.Valfa_beta = FOCVars[M2].Valphabeta;   /* Only if motor2 is sensorless */
      </#if><#-- MC.DRIVE_NUMBER == 1 -->
      <#if M2_ENCODER>
    (void)ENC_CalcAngle(&ENCODER_M2);
      </#if><#-- M2_ENCODER -->
      <#if (MC.M2_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR")||( MC.M2_SPEED_SENSOR == "HALL_SENSOR")>
    (void)HALL_CalcElAngle(&HALL_M2);
      </#if><#-- (MC.M2_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR")||( MC.M2_SPEED_SENSOR == "HALL_SENSOR") -->
      <#if ((MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC"))>
    if (SWITCH_OVER == Mci[M2].State)
    {
      if (!REMNG_RampCompleted(pREMNG[M2]))
      {
        FOCVars[M2].Iqdref.q = ( int16_t )REMNG_Calc(pREMNG[M2]);
      }
    }
      </#if><#-- ((MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")) -->
      <#if ((MC.M2_OTF_STARTUP == true))>
    if(!RUC_Get_SCLowsideOTF_Status(&RevUpControlM2))
    {
      hFOCreturn = FOC_CurrControllerM2();
    }
    else
    {
      hFOCreturn = MC_NO_ERROR;
    }
      <#else><#-- ((MC.M2_OTF_STARTUP == false)) -->
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_3 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_3 */
    hFOCreturn = FOC_CurrControllerM2();
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_4 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_4 */
      </#if><#-- ((MC.M2_OTF_STARTUP == true)) -->
  }
  if(MC_DURATION == hFOCreturn)
  {
    MCI_FaultProcessing(&Mci[bMotorNbr], MC_DURATION, 0);
  }
  else
  {
    if (M1 == bMotorNbr)
    {
      <#if MC.M1_SPEED_SENSOR == "STO_PLL">
    bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1);
      </#if><#-- MC.M1_SPEED_SENSOR == "STO_PLL" -->
      <#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC")>
      STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /* Only if sensorless*/
      STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /* Only for sensorless*/
      (void)${SPD_calcElAngle_M1}(${SPD_M1}, &STO_Inputs);
      ${SPD_calcAvergElSpeedDpp_M1}(${SPD_M1}); /* Only in case of Sensor-less */
        <#if MC.M1_SPEED_SENSOR == "STO_PLL">
      if (false == IsAccelerationStageReached)
      {
        STO_ResetPLL(&STO_PLL_M1);
      }
      else
      {
        /* Nothing to do */
      }
        </#if><#-- MC.M1_SPEED_SENSOR == "STO_PLL" -->

      /* Only for sensor-less */
      if((START == Mci[M1].State) || (SWITCH_OVER == Mci[M1].State) || (RUN == Mci[M1].State))
      {
        <#if ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
        if (START == Mci[M1].State)
        {
          if (0U == RUC_IsAlignStageNow(&RevUpControlM1))
          {
            PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
          }
          else
          {
            PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 1);
          }
        }
        else
        {
          PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
        }
        </#if><#-- ((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
        int16_t hObsAngle = SPD_GetElAngle(${SPD_M1}._Super);
        (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
      }
      else
      {
        /* Nothing to do */
      }
      </#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") -->
      <#if (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
      STO_aux_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /* Only if sensorless */
      STO_aux_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /* Only for sensorless */
      (void)${SPD_aux_calcElAngle_M1}(${SPD_AUX_M1}, &STO_aux_Inputs);
      ${SPD_aux_calcAvrgElSpeedDpp_M1}(${SPD_AUX_M1});
      </#if><#-- (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") -->
    }
    else /* bMotorNbr != M1 */
    {
      <#if MC.M2_SPEED_SENSOR == "STO_PLL">
      bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM2);
      </#if><#-- MC.M2_SPEED_SENSOR == "STO_PLL" -->
      <#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC")>
      STO_Inputs.Ialfa_beta = FOCVars[M2].Ialphabeta; /* Only if sensorless */
      STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M2._Super)); /* Only for sensorless */
      (void)${SPD_calcElAngle_M2}(${SPD_M2}, &STO_Inputs);
      ${SPD_calcAvergElSpeedDpp_M2}(${SPD_M2}); /* Only in case of Sensor-less */
        <#if MC.M2_SPEED_SENSOR == "STO_PLL">
      if (false == IsAccelerationStageReached)
      {
        STO_ResetPLL(&STO_PLL_M2);
      }
      else
      {
        /* Nothing to do */
      }
        </#if><#-- MC.M2_SPEED_SENSOR == "STO_PLL" -->

      /* Only for sensor-less */
      if((START == Mci[M2].State) || (SWITCH_OVER == Mci[M2].State) || (RUN == Mci[M2].State))
      {
        <#if ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN'))>
        if (START == Mci[M2].State)
        {
          if (0U == RUC_IsAlignStageNow(&RevUpControlM2))
          {
            PWMC_SetAlignFlag(&PWM_Handle_M2._Super, 0);
          }
          else
          {
            PWMC_SetAlignFlag(&PWM_Handle_M2._Super, 1);
          }
        }
        else
        {
          PWMC_SetAlignFlag(&PWM_Handle_M2._Super, 0);
        }
        </#if><#-- ((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) -->
        int16_t hObsAngle = SPD_GetElAngle(${SPD_M2}._Super);
        (void)VSS_CalcElAngle(&VirtualSpeedSensorM2, &hObsAngle);
      }
      else
      {
        /* Nothing to do */
      }
      </#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") -->
  
      <#if (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC")>
      STO_aux_Inputs.Ialfa_beta = FOCVars[M2].Ialphabeta; /* Only if sensorless */
      STO_aux_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M2._Super)); /* Only for sensorless */
      (void)${SPD_aux_calcElAngle_M2}(${SPD_AUX_M2}, &STO_aux_Inputs);
      ${SPD_aux_calcAvrgElSpeedDpp_M2}(${SPD_AUX_M2});
      </#if><#-- (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_PLL") || (MC.M2_AUXILIARY_SPEED_SENSOR == "STO_CORDIC") --->
    }
  }
  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */
</#if>
  <#if MC.TESTENV == true  && MC.PFC_ENABLED == false >
  /* Performance Measurement: end measure */
  stop_perf_measure();
    </#if><#-- MC.TESTENV == true && MC.PFC_ENABLED == false -->
   
  return (bMotorNbr);


}
</#if><#-- MC.MOTOR_PROFILER == false -->

<#if (MC.MOTOR_PROFILER == true)>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Motor control profiler HF task
  * @param  None
  * @retval uint8_t It return always 0.
  */
__weak uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
  ab_t Iab;
  
  <#if MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR">
  HALL_CalcElAngle (&HALL_M1);
  </#if><#-- MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->
  
  if (SWITCH_OVER == Mci[M1].State)
  {
    if (!REMNG_RampCompleted(pREMNG[M1]))
    {
      FOCVars[M1].Iqdref.q = (int16_t)REMNG_Calc(pREMNG[M1]);
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
  <#if G4_Cut2_2_patch == true>
  RCM_ReadOngoingConv();
  RCM_ExecNextConv();
  </#if><#-- G4_Cut2_2_patch == true -->
  /* The generic function needs to be called here as the undelying   
   * implementation changes in time depending on the Profiler's state 
   * machine. Calling the generic function ensures that the correct
   * implementation is invoked */
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  FOCVars[M1].Iab = Iab;
  SCC_SetPhaseVoltage(&SCC);
  <#if MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR">
  HT_GetPhaseShift(&HT);
  </#if><#-- MC.M1_AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->
  
  return (0); /* Single motor only */
}
</#if><#-- (MC.MOTOR_PROFILER == true) -->

<#if MC.MOTOR_PROFILER == false>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise
  */
inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  int16_t hElAngle;
  uint16_t hCodeError;
  SpeednPosFdbk_Handle_t *speedHandle;
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
  MC_ControlMode_t mode;
  
  mode = MCI_GetControlMode( &Mci[M1] );
    </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);
    <#if (MC.M1_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_SPEED_SENSOR == "STO_CORDIC")>
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*PARK_ANGLE_COMPENSATION_FACTOR;
    </#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") ||  (MC.M1_SPEED_SENSOR == "STO_CORDIC") -->
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
    <#if NoInjectedChannel>
      <#if G4_Cut2_2_patch>
  RCM_ReadOngoingConv();
      </#if><#-- G4_Cut2_2_patch -->
  RCM_ExecNextConv();
    </#if><#-- NoInjectedChannel -->
    <#if (MC.M1_AMPLIFICATION_GAIN?number <0)>
  /* As the Gain is negative, we invert the current read */
  Iab.a = -Iab.a;
  Iab.b = -Iab.b;
    </#if><#-- (MC.M1_AMPLIFICATION_GAIN?number <0) -->
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
  Vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
    <#if MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true>
  Vqd = FF_VqdConditioning(pFF[M1],Vqd);
    </#if><#-- MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true -->
    <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
  if (mode == MCM_OPEN_LOOP_VOLTAGE_MODE)
  {
    Vqd = OL_VqdConditioning(pOpenLoop[M1]);
  }
  else
  {
    /* Nothing to do */
  }
    </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
  Vqd = Circle_Limitation(&CircleLimitationM1, Vqd);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
    <#if NoInjectedChannel &&  !G4_Cut2_2_patch>
  RCM_ReadOngoingConv();
    </#if><#-- NoInjectedChannel &&  !G4_Cut2_2_patch -->
  hCodeError = PWMC_SetPhaseVoltage${OVM}(pwmcHandle[M1], Valphabeta);
    <#if (((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || MC.M1_OVERMODULATION == true) && ((MC.M1_CURRENT_SENSING_TOPO != 'ICS_SENSORS'))>
  PWMC_CalcPhaseCurrentsEst(pwmcHandle[M1],Iqd, hElAngle);
    </#if><#-- (((MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M1_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || MC.M1_OVERMODULATION == true) && (MC.M1_CURRENT_SENSING_TOPO != 'ICS_SENSORS') -->

  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;

    <#if MC.M1_FLUX_WEAKENING_ENABLING == true>
  FW_DataProcess(pFW[M1], Vqd);
    </#if><#-- MC.M1_FLUX_WEAKENING_ENABLING == true -->
    <#if MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true>
  FF_DataProcess(pFF[M1]);
    </#if><#-- MC.M1_FEED_FORWARD_CURRENT_REG_ENABLING == true -->
  return (hCodeError);
}

  <#if (MC.DRIVE_NUMBER != "1")>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation of motor 2. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted.
  * @param This related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise.
  */
inline uint16_t FOC_CurrControllerM2(void)
{
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  qd_t Iqd, Vqd;
  int16_t hElAngle;
  uint16_t hCodeError;
  SpeednPosFdbk_Handle_t *speedHandle;

<#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
  MC_ControlMode_t mode;
  mode = MCI_GetControlMode( &Mci[M2] );
</#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->

  speedHandle = STC_GetSpeedSensor(pSTC[M2]);
  hElAngle = SPD_GetElAngle(speedHandle);
    <#if (MC.M2_SPEED_SENSOR == "STO_PLL") ||  (MC.M2_SPEED_SENSOR == "STO_CORDIC")>
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle) * PARK_ANGLE_COMPENSATION_FACTOR2;
    </#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") ||  (MC.M2_SPEED_SENSOR == "STO_CORDIC") -->
  PWMC_GetPhaseCurrents(pwmcHandle[M2], &Iab);
    <#if NoInjectedChannel>
      <#if G4_Cut2_2_patch>
  RCM_ReadOngoingConv();
      </#if><#-- G4_Cut2_2_patch -->
  RCM_ExecNextConv();
    </#if><#-- NoInjectedChannel -->
    <#if (MC.M2_AMPLIFICATION_GAIN?number <0)>
  /* As the Gain is negative, we invert the current read */
  Iab.a = -Iab.a;
  Iab.b = -Iab.b;
    </#if><#-- (MC.M2_AMPLIFICATION_GAIN?number <0) -->
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  Vqd.q = PI_Controller(pPIDIq[M2], (int32_t)(FOCVars[M2].Iqdref.q) - Iqd.q);
  Vqd.d = PI_Controller(pPIDId[M2], (int32_t)(FOCVars[M2].Iqdref.d) - Iqd.d);
    <#if MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true>
  Vqd = FF_VqdConditioning(pFF[M2],Vqd);
    </#if><#-- MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true -->
  
    <#if MC.M2_DBG_OPEN_LOOP_ENABLE == true>
  if (mode == MCM_OPEN_LOOP_VOLTAGE_MODE)
  {
    Vqd = OL_VqdConditioning(pOpenLoop[M2]);
  }
  else
  {
    /* Nothing to do */
  }
    </#if><#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true -->
  Vqd = Circle_Limitation(&CircleLimitationM2, Vqd);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle) * REV_PARK_ANGLE_COMPENSATION_FACTOR2;
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
  hCodeError = PWMC_SetPhaseVoltage${OVM2}(pwmcHandle[M2], Valphabeta);
    <#if (((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || MC.M2_OVERMODULATION == true) && (MC.M2_CURRENT_SENSING_TOPO != 'ICS_SENSORS')>
  PWMC_CalcPhaseCurrentsEst(pwmcHandle[M2],Iqd, hElAngle);
    </#if><#-- (((MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_PHASE_SHIFT') || (MC.M2_CURRENT_SENSING_TOPO == 'SINGLE_SHUNT_ACTIVE_WIN')) 
    || MC.M2_OVERMODULATION == true) && (MC.M2_CURRENT_SENSING_TOPO != 'ICS_SENSORS') -->
    <#if NoInjectedChannel && !G4_Cut2_2_patch>
  RCM_ReadOngoingConv();
    </#if><#-- NoInjectedChannel && !G4_Cut2_2_patch -->
  FOCVars[M2].Vqd = Vqd;
  FOCVars[M2].Iab = Iab;
  FOCVars[M2].Ialphabeta = Ialphabeta;
  FOCVars[M2].Iqd = Iqd;
  FOCVars[M2].Valphabeta = Valphabeta;
  FOCVars[M2].hElAngle = hElAngle;
    <#if MC.M2_FLUX_WEAKENING_ENABLING == true>
  FW_DataProcess(pFW[M2], Vqd);
    </#if><#-- MC.M2_FLUX_WEAKENING_ENABLING == true -->
    <#if MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true>
  FF_DataProcess(pFF[M2]);
    </#if><#-- MC.M2_FEED_FORWARD_CURRENT_REG_ENABLING == true -->
  return(hCodeError);
}
  </#if><#-- MC.DRIVE_NUMBER > 1 -->
</#if><#--  MC.MOTOR_PROFILER == false -->

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
