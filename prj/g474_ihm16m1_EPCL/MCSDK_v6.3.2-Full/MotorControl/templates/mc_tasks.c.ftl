<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/foc_assign.ftl">

/**
  ******************************************************************************
  * @file    mc_tasks.c
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
<#if MC.RTOS == "FREERTOS">
#include "cmsis_os.h"
</#if><#-- MC.RTOS == "FREERTOS" -->
#include "mc_interface.h"
#include "digital_output.h"
<#if FOC>
#include "pwm_common.h"
</#if><#-- FOC -->
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
  <#if FOC && MC.PFC_ENABLED == false >
#include "mc_testenv.h"
  </#if>
  <#if SIX_STEP>
#include "mc_testenv_6step.h"
  </#if>
</#if>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/
/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

<#if FOC>
</#if><#-- FOC -->
/* USER CODE END Private define */

<#if MC.M1_OV_TEMPERATURE_PROT_ENABLING == true &&  MC.M1_UV_VOLTAGE_PROT_ENABLING == true
  && MC.M1_OV_VOLTAGE_PROT_ENABLING == true>
#define VBUS_TEMP_ERR_MASK (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)
<#else><#-- MC.M1_OV_TEMPERATURE_PROT_ENABLING == false ||  MC.M1_UV_VOLTAGE_PROT_ENABLING == false
|| MC.M1_OV_VOLTAGE_PROT_ENABLING == false -->
  <#if MC.M1_UV_VOLTAGE_PROT_ENABLING == false>
<#assign UV_ERR = "MC_UNDER_VOLT">
  <#else><#-- MC.M1_UV_VOLTAGE_PROT_ENABLING == true -->
<#assign UV_ERR = "0">
  </#if><#--  MC.M1_UV_VOLTAGE_PROT_ENABLING == false -->
  <#if MC.M1_OV_VOLTAGE_PROT_ENABLING == false>
<#assign OV_ERR = "MC_OVER_VOLT">
  <#else><#-- MC.M1_OV_VOLTAGE_PROT_ENABLING == true -->
<#assign OV_ERR = "0">
  </#if><#-- MC.M1_OV_VOLTAGE_PROT_ENABLING == false -->
  <#if MC.M1_OV_TEMPERATURE_PROT_ENABLING == false>
<#assign OT_ERR = "MC_OVER_TEMP">
  <#else><#-- MC.M1_OV_TEMPERATURE_PROT_ENABLING == true -->
<#assign OT_ERR = "0">
  </#if><#-- MC.M1_OV_TEMPERATURE_PROT_ENABLING == false -->
#define VBUS_TEMP_ERR_MASK ~(${OV_ERR} | ${UV_ERR} | ${OT_ERR})
</#if><#-- MC.M1_OV_TEMPERATURE_PROT_ENABLING == true &&  MC.M1_UV_VOLTAGE_PROT_ENABLING == true
        && MC.M1_OV_VOLTAGE_PROT_ENABLING == true -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_OV_TEMPERATURE_PROT_ENABLING == true &&  MC.M2_UV_VOLTAGE_PROT_ENABLING == true
    && MC.M2_OV_VOLTAGE_PROT_ENABLING == true>
#define VBUS_TEMP_ERR_MASK2 (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)
  <#else><#-- MC.M2_OV_TEMPERATURE_PROT_ENABLING == false ||  MC.M2_UV_VOLTAGE_PROT_ENABLING == false
         || MC.M2_OV_VOLTAGE_PROT_ENABLING == false -->
    <#if MC.M2_UV_VOLTAGE_PROT_ENABLING == false>
<#assign UV_ERR2 = "MC_UNDER_VOLT">
    <#else><#-- MC.M2_UV_VOLTAGE_PROT_ENABLING == true -->
<#assign UV_ERR2 = "0">
    </#if><#-- MC.M2_UV_VOLTAGE_PROT_ENABLING == false -->
    <#if MC.M2_OV_VOLTAGE_PROT_ENABLING == false>
<#assign OV_ERR2 = "MC_OVER_VOLT">
    <#else><#-- MC.M2_OV_VOLTAGE_PROT_ENABLING == true -->
<#assign OV_ERR2 = "0">
    </#if><#-- MC.M2_OV_VOLTAGE_PROT_ENABLING == false -->
    <#if MC.M1_OV_TEMPERATURE_PROT_ENABLING == false>
<#assign OT_ERR2 = "MC_OVER_TEMP">
    <#else><#-- MC.M1_OV_TEMPERATURE_PROT_ENABLING == true -->
<#assign OT_ERR2 = "0">
    </#if><#-- MC.M1_OV_TEMPERATURE_PROT_ENABLING == false -->
#define VBUS_TEMP_ERR_MASK2 ~(${OV_ERR2} | ${UV_ERR2} | ${OT_ERR2})
  </#if><#-- MC.M2_OV_TEMPERATURE_PROT_ENABLING == true &&  MC.M2_UV_VOLTAGE_PROT_ENABLING == true
          && MC.M2_OV_VOLTAGE_PROT_ENABLING == true -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->
/* Private variables----------------------------------------------------------*/

<#if MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE == true>
  <#if MC.DRIVE_NUMBER == "1">
static uint16_t nominalBusd[1] = {0u};
static uint16_t ovthd[1] = {OVERVOLTAGE_THRESHOLD_d};
  <#else><#-- MC.DRIVE_NUMBER != 1 -->
static uint16_t nominalBusd[2] = {0u,0u};
static uint16_t ovthd[2] = {OVERVOLTAGE_THRESHOLD_d,OVERVOLTAGE_THRESHOLD_d2};
  </#if><#-- MC.DRIVE_NUMBER == 1 -->
</#if><#-- MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE == true -->

<#if (MC.M1_HW_OV_CURRENT_PROT_BYPASS == true) &&  (MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES")
 || (MC.M1_HW_OV_CURRENT_PROT_BYPASS == true)>
static DOUT_handle_t *pOCPDisabling[NBR_OF_MOTORS];
</#if><#-- (MC.M1_HW_OV_CURRENT_PROT_BYPASS == true) &&  (MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES")
        || (MC.M1_HW_OV_CURRENT_PROT_BYPASS == true) -->

static uint16_t hMFTaskCounterM1 = 0; //cstat !MISRAC2012-Rule-8.9_a
static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);
<#if MC.DRIVE_NUMBER != "1">
static volatile uint16_t hMFTaskCounterM2 = ((uint16_t)0);
static volatile uint16_t hBootCapDelayCounterM2 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM2 = ((uint16_t)0);
</#if><#-- MC.DRIVE_NUMBER > 1 -->
static volatile uint8_t bMCBootCompleted = ((uint8_t)0);


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
<#if SIX_STEP>
#define S16_90_PHASE_SHIFT             (int16_t)(65536/4)
</#if><#-- SIX_STEP -->

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void TSK_MF_StopProcessing(uint8_t motor);
MCI_Handle_t *GetMCI(uint8_t bMotor);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
<#if MC.M1_ON_OVER_VOLTAGE == "TURN_OFF_PWM" || MC.M2_ON_OVER_VOLTAGE == "TURN_OFF_PWM">
void TSK_SafetyTask_PWMOFF(uint8_t motor);
</#if><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_OFF_PWM" || MC.M2_ON_OVER_VOLTAGE == "TURN_OFF_PWM" -->
<#if MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE">
void TSK_SafetyTask_RBRK(uint8_t motor);
</#if><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" -->
<#if MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES">
void TSK_SafetyTask_LSON(uint8_t motor);
</#if><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" -->
<#if MC.DRIVE_NUMBER != "1">
void TSK_MediumFrequencyTaskM2(void);
void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM2(void);
void TSK_SetStopPermanencyTimeM2(uint16_t SysTickCount);
bool TSK_StopPermanencyTimeHasElapsedM2(void);

<#if HIGH_FREQ_TRIGGER == "TIM_UP_IT">
#define FOC_ARRAY_LENGTH 2
static uint8_t FOC_array[FOC_ARRAY_LENGTH]={0, 0};
static uint8_t FOC_array_head = 0; /* Next obj to be executed */
static uint8_t FOC_array_tail = 0; /* Last arrived */
</#if><#-- HIGH_FREQ_TRIGGER == "TIM_UP_IT" -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->
<#if MC.PFC_ENABLED == true>
void PFC_Scheduler(void);
</#if><#-- MC.PFC_ENABLED == true -->

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  */
__weak void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */
  
  if (MC_NULL == pMCIList)
  {
    /* Nothing to do */
  }
  else
  {
<#if MC.USE_STGAP1S>
    /**************************************/
    /*    STGAP1AS initialization         */
    /**************************************/
    if (false == GAP_Configuration(&STGAP_M1))
    {
      MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0);
    }
    else
    {
      /* Nothing to do */
    }
</#if><#-- MC.USE_STGAP1S -->

    bMCBootCompleted = (uint8_t )0;
<#if MC.M1_HW_OV_CURRENT_PROT_BYPASS == true &&  MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES">
    pOCPDisabling[M1] = &DOUT_OCPDisablingParamsM1;
    DOUT_SetOutputState(pOCPDisabling[M1],INACTIVE);
</#if><#-- MC.M1_HW_OV_CURRENT_PROT_BYPASS == true &&  MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" -->
<#if MC.DRIVE_NUMBER != "1">
  <#if MC.M2_HW_OV_CURRENT_PROT_BYPASS == true &&  MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES">
    pOCPDisabling[M2] = &DOUT_OCPDisablingParamsM2;
    DOUT_SetOutputState(pOCPDisabling[M2],INACTIVE);
  </#if><#-- MC.M2_HW_OV_CURRENT_PROT_BYPASS == true &&  MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" == true -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->

    /*************************************************/
    /*    ${MC.M1_DRIVE_TYPE} initialization         */
    /*************************************************/
    pMCIList[M1] = &Mci[M1];
    ${MC.M1_DRIVE_TYPE}_Init();

	
<#if MC.MCP_OVER_UART_A_EN>
    ASPEP_start(&aspepOverUartA);
</#if><#-- MC.MCP_OVER_UART_A_EN -->
<#if MC.MCP_OVER_UART_B_EN>
    ASPEP_start(&aspepOverUartB);
</#if><#-- MC.MCP_OVER_UART_B_EN -->
<#if MC.MCP_OVER_STLNK_EN>
    STLNK_init(&STLNK);
</#if><#-- MC.MCP_OVER_STLNK_EN -->
    /* USER CODE BEGIN MCboot 1 */
  
    /* USER CODE END MCboot 1 */
 
    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

<#if (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || M1_ENCODER
  || (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC")>
    /****************************************************/
    /*   Virtual speed sensor component initialization  */
    /****************************************************/ 
    VSS_Init(&VirtualSpeedSensorM1);
</#if><#-- (MC.M1_SPEED_SENSOR == "STO_PLL") || (MC.M1_SPEED_SENSOR == "STO_CORDIC") || M1_ENCODER || (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC") -->

<#if MC.M1_BUS_VOLTAGE_READING == true>
    /********************************************************/
    /*   Bus voltage sensor component initialization        */
    /********************************************************/
    (void)RCM_RegisterRegConv(&VbusRegConv_M1);
    RVBS_Init(&BusVoltageSensor_M1);
<#else><#-- MC.M1_BUS_VOLTAGE_READING == false -->
    /**********************************************************/
    /*   Virtual bus voltage sensor component initialization  */
    /**********************************************************/
    VVBS_Init(&BusVoltageSensor_M1);
</#if><#-- MC.M1_BUS_VOLTAGE_READING == true -->

    /*******************************************************/
    /*   Temperature measurement component initialization  */
    /*******************************************************/
<#if (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true)>
    (void)RCM_RegisterRegConv(&TempRegConv_M1);
</#if>
    NTC_Init(&TempSensor_M1);

<#if DWT_CYCCNT_SUPPORTED>
  <#if MC.DBG_MCU_LOAD_MEASURE == true>
    Mci[M1].pPerfMeasure = &PerfTraces;
    MC_Perf_Measure_Init(&PerfTraces);
  </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
</#if><#-- DWT_CYCCNT_SUPPORTED -->

<#if MC.DRIVE_NUMBER != "1">
    /******************************************************/
    /*   Motor 2 features initialization                  */
    /******************************************************/
    pMCIList[M2] = &Mci[M2];
    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/  
    PID_HandleInit(&PIDSpeedHandle_M2);
    
  <#if (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") || M2_ENCODER>
    /****************************************************/
    /*   Virtual speed sensor component initialization  */
    /****************************************************/ 
    VSS_Init(&VirtualSpeedSensorM2);
  </#if><#-- (MC.M2_SPEED_SENSOR == "STO_PLL") || (MC.M2_SPEED_SENSOR == "STO_CORDIC") || M2_ENCODER -->

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M2);
    PID_HandleInit(&PIDIdHandle_M2);

  <#if MC.M2_BUS_VOLTAGE_READING == true>
    /**********************************************************/
    /*   Bus voltage sensor component initialization          */
    /**********************************************************/
    (void)RCM_RegisterRegConv(&VbusRegConv_M2);
    RVBS_Init(&BusVoltageSensor_M2);
  <#else><#-- MC.M2_BUS_VOLTAGE_READING == false -->
    /**********************************************************/
    /*   Virtual bus voltage sensor component initialization  */
    /**********************************************************/
    VVBS_Init(&BusVoltageSensor_M2);
  </#if><#-- MC.M2_BUS_VOLTAGE_READING == true -->

    /*******************************************************/
    /*   Temperature measurement component initialization  */
    /*******************************************************/
  <#if (MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true)>
    (void)RCM_RegisterRegConv(&TempRegConv_M2);
  </#if>
    NTC_Init(&TempSensor_M2);

</#if><#-- MC.DRIVE_NUMBER > !1 -->

<#if MC.PFC_ENABLED == true>
    /* Initializing the PFC component */
    PFC_Init(&PFC);
</#if><#-- MC.PFC_ENABLED == true -->

<#if MC.DEBUG_DAC_FUNCTIONALITY_EN>
    DAC_Init(&DAC_Handle);
</#if><#-- MC.PFC_ENABLED == true -->

<#if MC.M1_PWM_DRIVER_PN == "STSPIN32G4">
    /*************************************************/
    /*   STSPIN32G4 driver component initialization  */
    /*************************************************/
    STSPIN32G4_init(&HdlSTSPING4);
    STSPIN32G4_reset(&HdlSTSPING4);
    STSPIN32G4_setVCC(&HdlSTSPING4, (STSPIN32G4_confVCC){.voltage = _12V,
                                                         .useNFAULT = true,
                                                         .useREADY = false });
    STSPIN32G4_setVDSP(&HdlSTSPING4, (STSPIN32G4_confVDSP){.deglitchTime = _4us,
                                                           .useNFAULT = true });
    STSPIN32G4_clearFaults(&HdlSTSPING4);
</#if><#-- MC.M1_PWM_DRIVER_PN == "STSPIN32G4" -->

    /* Applicative hook in MCBoot() */
    MC_APP_BootHook();

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
  
    bMCBootCompleted = 1U;
  }
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is 
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors.
 * - Safety Task.
 * - Power Factor Correction Task (if enabled).
 * - User Interface task.
 */
__weak void MC_RunMotorControlTasks(void)
{
  if (0U == bMCBootCompleted)
  {
    /* Nothing to do */
  }
  else
  {
    /* ** Medium Frequency Tasks ** */
/* USER CODE BEGIN MC_Scheduler 0 */

/* USER CODE END MC_Scheduler 0 */

    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
<#if DWT_CYCCNT_SUPPORTED>
  <#if MC.DBG_MCU_LOAD_MEASURE == true>
      MC_BG_Perf_Measure_Start(&PerfTraces, MEASURE_TSK_MediumFrequencyTaskM1);
  </#if><#-- DWT_CYCCNT_SUPPORTED -->
</#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
      TSK_MediumFrequencyTaskM1();
	  
<#if DWT_CYCCNT_SUPPORTED>
  <#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_BG_Perf_Measure_Stop(&PerfTraces, MEASURE_TSK_MediumFrequencyTaskM1);
  </#if><#-- DWT_CYCCNT_SUPPORTED -->
</#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
      /* Applicative hook at end of Medium Frequency for Motor 1 */
      MC_APP_PostMediumFrequencyHook_M1();

<#if MC.MCP_OVER_UART_A_EN>

      MCP_Over_UartA.rxBuffer = MCP_Over_UartA.pTransportLayer->fRXPacketProcess(MCP_Over_UartA.pTransportLayer, 
                                                                                &MCP_Over_UartA.rxLength);
      if ( 0U == MCP_Over_UartA.rxBuffer)
      {
        /* Nothing to do */
      }
      else
      {
        /* Synchronous answer */
        if (0U == MCP_Over_UartA.pTransportLayer->fGetBuffer(MCP_Over_UartA.pTransportLayer, 
                                                     (void **) &MCP_Over_UartA.txBuffer, //cstat !MISRAC2012-Rule-11.3
                                                     MCTL_SYNC)) 
        {
          /* No buffer available to build the answer ... should not occur */
        }
        else
        {
          MCP_ReceivedPacket(&MCP_Over_UartA);
          MCP_Over_UartA.pTransportLayer->fSendPacket(MCP_Over_UartA.pTransportLayer, MCP_Over_UartA.txBuffer, 
                                                      MCP_Over_UartA.txLength, MCTL_SYNC);
          /* No buffer available to build the answer ... should not occur */
        }
      }
</#if><#-- MC.MCP_OVER_UART_A_EN -->
<#if MC.MCP_OVER_UART_B_EN>

      MCP_Over_UartB.rxBuffer = MCP_Over_UartB.pTransportLayer->fRXPacketProcess(MCP_Over_UartB.pTransportLayer,
                                                                                 &MCP_Over_UartB.rxLength);
      if (MCP_Over_UartB.rxBuffer)
      {
        /* Synchronous answer */
        if (MCP_Over_UartB.pTransportLayer->fGetBuffer(MCP_Over_UartB.pTransportLayer,
                                                       (void **) &MCP_Over_UartB.txBuffer, MCTL_SYNC))
        {
          MCP_ReceivedPacket(&MCP_Over_UartB);
          MCP_Over_UartB.pTransportLayer->fSendPacket(MCP_Over_UartB.pTransportLayer, MCP_Over_UartB.txBuffer,
                                                      MCP_Over_UartB.txLength, MCTL_SYNC);
        }
        else 
        {
          /* No buffer available to build the answer ... should not occur */
        }
      }
</#if><#-- MC.MCP_OVER_UART_B_EN -->
<#if MC.MCP_OVER_STLNK_EN>
      MCP_Over_STLNK.rxBuffer = MCP_Over_STLNK.pTransportLayer->fRXPacketProcess( MCP_Over_STLNK.pTransportLayer,
                                                                                  &MCP_Over_STLNK.rxLength);
      if (0U == MCP_Over_STLNK.rxBuffer)
      {
        /* Nothing to do */
      }
      else
      {
        /* Synchronous answer */
        if (0U == MCP_Over_STLNK.pTransportLayer->fGetBuffer(MCP_Over_STLNK.pTransportLayer,
                                      (void **) &MCP_Over_STLNK.txBuffer, MCTL_SYNC)) //cstat !MISRAC2012-Rule-11.3
        {
          /* No buffer available to build the answer ... should not occur */
        }
        else 
        {
          MCP_ReceivedPacket(&MCP_Over_STLNK);
          MCP_Over_STLNK.pTransportLayer->fSendPacket (MCP_Over_STLNK.pTransportLayer, MCP_Over_STLNK.txBuffer,
                                                       MCP_Over_STLNK.txLength, MCTL_SYNC);
        }
      }
</#if><#-- MC.MCP_OVER_STLNK_EN -->

      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */

      hMFTaskCounterM1 = (uint16_t)MF_TASK_OCCURENCE_TICKS;
    }
<#if MC.DRIVE_NUMBER != "1">
    if(hMFTaskCounterM2 > ((uint16_t )0))
    {
      hMFTaskCounterM2--;
    }
    else
    {
      TSK_MediumFrequencyTaskM2();

      /* Applicative hook at end of Medium Frequency for Motor 2 */
      MC_APP_PostMediumFrequencyHook_M2();

      /* USER CODE BEGIN MC_Scheduler MediumFrequencyTask M2 */

      /* USER CODE END MC_Scheduler MediumFrequencyTask M2 */
      hMFTaskCounterM2 = MF_TASK_OCCURENCE_TICKS2;
    }
</#if><#-- MC.DRIVE_NUMBER > 1 -->
    if(hBootCapDelayCounterM1 > 0U)
    {
      hBootCapDelayCounterM1--;
    }
    else
    {
      /* Nothing to do */
    }
    if(hStopPermanencyCounterM1 > 0U)
    {
      hStopPermanencyCounterM1--;
    }
    else
    {
      /* Nothing to do */
    }
<#if MC.DRIVE_NUMBER != "1">
    if(hBootCapDelayCounterM2 > 0U)
    {
      hBootCapDelayCounterM2--;
    }
    else
    {
      /* Nothing to do */
    }
    if(hStopPermanencyCounterM2 > 0U)
    {
      hStopPermanencyCounterM2--;
    }
    else
    {
      /* Nothing to do */
    }
</#if><#-- MC.DRIVE_NUMBER > 1 -->

  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
<#if MC.RTOS == "NONE">

    /* Safety task is run after Medium Frequency task so that
     * it can overcome actions they initiated if needed */
    TSK_SafetyTask();
    
</#if><#-- MC.RTOS == "NONE" -->
<#if MC.PFC_ENABLED == true>
    /* ** Power Factor Correction Task ** */ 
    PFC_Scheduler();
</#if><#-- MC.PFC_ENABLED == true -->
  }
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1.
  * @param  hTickCount number of ticks to be counted.
  * @retval void
  */
__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed.
  * @param  none
  * @retval bool true if time has elapsed, false otherwise.
  */
__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hBootCapDelayCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1.
  * @param  hTickCount number of ticks to be counted.
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed.
  * @param  none
  * @retval bool true if time is elapsed, false otherwise.
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hStopPermanencyCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

<#if MC.DRIVE_NUMBER != "1">
/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 2.
  * @param  hTickCount number of ticks to be counted.
  * @retval void
  */
__weak void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount)
{
   hBootCapDelayCounterM2 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 2 has elapsed.
  * @param  none
  * @retval bool true if time has elapsed, false otherwise.
  */
__weak bool TSK_ChargeBootCapDelayHasElapsedM2(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM2 == ((uint16_t )0))
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 2.
  * @param  hTickCount number of ticks to be counted.
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM2(uint16_t hTickCount)
{
  hStopPermanencyCounterM2 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 2 has elapsed.
  * @param  none
  * @retval bool true if time is elapsed, false otherwise.
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM2(void)
{
  bool retVal = false;
  if (0U == hStopPermanencyCounterM2)
  {
    retVal = true;
  }
  return (retVal);
}
</#if><#-- MC.DRIVE_NUMBER > 1 -->

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
  <#if HIGH_FREQ_TRIGGER == "TIM_UP_IT">
__weak uint8_t TSK_HighFrequencyTask(void)
{
  uint8_t bMotorNbr;
    <#if MC.DRIVE_NUMBER != "1">
  bMotorNbr = FOC_array[FOC_array_head];
    <#else>
  bMotorNbr = 0;
    </#if><#-- MC.DRIVE_NUMBER > 1 -->

  <#else><#-- HIGH_FREQ_TRIGGER == "ADC_IT" -->
__weak uint8_t TSK_HighFrequencyTask(uint8_t bMotorNbr)
{
  </#if><#-- HIGH_FREQ_TRIGGER == "TIM_UP_IT" -->

  
<#if DWT_CYCCNT_SUPPORTED>
  <#if MC.DBG_MCU_LOAD_MEASURE == true>
    <#if MC.DRIVE_NUMBER != "1">
  if(M1 == bMotorNbr){
    MC_Perf_Measure_Start(&PerfTraces, MEASURE_TSK_HighFrequencyTaskM1);
  }
  else{
    MC_Perf_Measure_Start(&PerfTraces, MEASURE_TSK_HighFrequencyTaskM2);
  }
    <#else>
  MC_Perf_Measure_Start(&PerfTraces, MEASURE_TSK_HighFrequencyTaskM1);
    </#if><#-- MC.DRIVE_NUMBER > 1 -->
  </#if><#-- MC.DBG_MCU_LOAD_MEASURE == true -->
</#if><#-- DWT_CYCCNT_SUPPORTED -->
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */
  ${MC.M1_DRIVE_TYPE}_HighFrequencyTask(bMotorNbr);
  <#if HIGH_FREQ_TRIGGER == "TIM_UP_IT">  
    <#if MC.DRIVE_NUMBER != "1">
  FOC_array_head++;
  if (FOC_array_head == FOC_ARRAY_LENGTH)
  {
    FOC_array_head = 0;
  }
  else
  {
    /* Nothing to do */
  }
    </#if><#-- MC.DRIVE_NUMBER == "1" -->
  </#if><#--  HIGH_FREQ_TRIGGER == "TIM_UP_IT" -->
<#if MC.DEBUG_DAC_FUNCTIONALITY_EN == true>
  <#if MC.DRIVE_NUMBER != "1">
  DAC_Exec(&DAC_Handle, bMotorNbr);
  <#else><#-- MC.DRIVE_NUMBER == "1" -->
  DAC_Exec(&DAC_Handle);
  </#if><#-- MC.DRIVE_NUMBER != "1" -->
</#if><#-- MC.DEBUG_DAC_FUNCTIONALITY_EN == true -->

  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */

    <#if MC.MCP_ASYNC_EN>
  GLOBAL_TIMESTAMP++;
    </#if><#-- MC.MCP_ASYNC_EN -->
    <#if MC.MCP_ASYNC_OVER_UART_A_EN>
  if (0U == MCPA_UART_A.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_UART_A);
  }
    </#if><#-- MC.MCP_ASYNC_OVER_UART_A_EN -->
    <#if MC.MCP_ASYNC_OVER_UART_B_EN>
  if (0U == MCPA_UART_BM_1.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_UART_B);
  }
    </#if><#-- MC.MCP_ASYNC_OVER_UART_B_EN -->
    <#if MC.MCP_ASYNC_OVER_STLNK_EN>
  if (0U == MCPA_STLNK.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_STLNK);
  }
    </#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->

    <#if DWT_CYCCNT_SUPPORTED>
      <#if MC.DBG_MCU_LOAD_MEASURE == true>
        <#if MC.DRIVE_NUMBER != "1">
  if(M1 == bMotorNbr){
    MC_Perf_Measure_Stop(&PerfTraces, MEASURE_TSK_HighFrequencyTaskM1);
  }
  else{
    MC_Perf_Measure_Stop(&PerfTraces, MEASURE_TSK_HighFrequencyTaskM2);
  }
        <#else>
  MC_Perf_Measure_Stop(&PerfTraces, MEASURE_TSK_HighFrequencyTaskM1);
        </#if><#-- MC.DRIVE_NUMBER > 1 -->
      </#if><#-- MC.DG_MCU_LOAD_MEASURE == true -->
    </#if><#-- DWT_CYCCNT_SUPPORTED -->
  return (bMotorNbr);



<#if HIGH_FREQ_TRIGGER == "TIM_UP_IT">
}
<#else><#-- HIGH_FREQ_TRIGGER == "ADC_IT" -->
}
</#if><#-- HIGH_FREQ_TRIGGER == "TIM_UP_IT"-->

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances. 
  *
  * Faults flags are updated here.
  */
__weak void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (1U == bMCBootCompleted)
  {
<#if (MC.MOTOR_PROFILER == true)>
    SCC_CheckOC_RL(&SCC);
</#if><#-- (MC.MOTOR_PROFILER == true) -->
<#if ( MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE")>
    TSK_SafetyTask_RBRK(M1);
<#elseif ( MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES")>
    TSK_SafetyTask_LSON(M1);
<#else><#-- (MC.M1_ON_OVER_VOLTAGE == "TURN_OFF_PWM") or not defined -->
    TSK_SafetyTask_PWMOFF(M1);
</#if><#-- ( MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE") -->
<#if (MC.DRIVE_NUMBER != "1")>
    /* Second drive */
  <#if ( MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE")>
    TSK_SafetyTask_RBRK(M2);
  <#elseif ( MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES")>
    TSK_SafetyTask_LSON(M2);
  <#else> <#-- (MC.M2_ON_OVER_VOLTAGE == "TURN_OFF_PWM") or not defined -->
    TSK_SafetyTask_PWMOFF(M2);
  </#if><#-- ( MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE") -->
</#if><#-- (MC.DRIVE_NUMBER > 1) -->
    /* User conversion execution */
    RCM_ExecUserConv();
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
  else
  {
    /* Nothing to do */
  }
}

<#if MC.M1_ON_OVER_VOLTAGE == "TURN_OFF_PWM" || MC.M2_ON_OVER_VOLTAGE == "TURN_OFF_PWM">
/**
  * @brief  Safety task implementation if  MC.M1_ON_OVER_VOLTAGE == TURN_OFF_PWM.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink.
  */
__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */
  uint16_t CodeReturn = MC_NO_ERROR;
  uint8_t lbmotor = M1;
  <#if MC.M1_BUS_VOLTAGE_READING == true || (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true) || (MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true)>
    <#if MC.DRIVE_NUMBER != "1">
  const uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK, VBUS_TEMP_ERR_MASK2};
    <#else><#-- MC.DRIVE_NUMBER == 1 -->
  const uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
    </#if><#-- MC.DRIVE_NUMBER > 1 -->
  </#if><#-- MC.M1_BUS_VOLTAGE_READING == true || (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true) || (MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true) -->
  /* Check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
<#if (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true)>
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 = RCM_ExecRegularConv(&TempRegConv_M1);
    CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(&TempSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }  
</#if>  
<#if (MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true)>
  if (M2 == bMotor)
  {
    uint16_t rawValueM2 = RCM_ExecRegularConv(&TempRegConv_M2);
    CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(&TempSensor_M2, rawValueM2);
  }
  else
  {
    /* Nothing to do */
  }  
</#if>

/* Due to warning array subscript 1 is above array bounds of PWMC_Handle_t *[1] [-Warray-bounds] */
<#if MC.DRIVE_NUMBER != "1">
  if (M2 == bMotor)
  {
    lbmotor = M2;
  }
</#if><#--  MC.DRIVE_NUMBER != "1" -->
   CodeReturn |= PWMC_IsFaultOccurred(pwmcHandle[lbmotor]);     /* check for fault. It return MC_OVER_CURR or MC_NO_FAULTS
                                                     (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */


  <#if MC.M1_BUS_VOLTAGE_READING == true>
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 =  RCM_ExecRegularConv(&VbusRegConv_M1);
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }
  <#else><#-- MC.M1_BUS_VOLTAGE_READING == false -->
  <#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
  </#if><#-- MC.M1_BUS_VOLTAGE_READING == true -->
  <#if MC.M2_BUS_VOLTAGE_READING == true>
  if (M2 == bMotor)
  {
    uint16_t rawValueM2 =  RCM_ExecRegularConv(&VbusRegConv_M2);
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M2, rawValueM2);
  }
  else
  {
    /* Nothing to do */
  }
  <#else><#-- MC.M2_BUS_VOLTAGE_READING == false -->
<#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
  </#if><#-- MC.M2_BUS_VOLTAGE_READING == true -->
  MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn); /* Process faults */

  <#if (MC.M1_ICL_ENABLED == true)>
  if ((M1 == bMotor) && (MC_UNDER_VOLT == (CodeReturn & MC_UNDER_VOLT)) && ICLFaultTreatedM1){
    ICLFaultTreatedM1 = false;
  }
  else
  {
    /* Nothing to do */
  }
  </#if><#-- (MC.M1_ICL_ENABLED == true) -->

  <#if (MC.M2_ICL_ENABLED == true)>
  if ((M2 == bMotor) && (MC_UNDER_VOLT == (CodeReturn & MC_UNDER_VOLT)) && ICLFaultTreatedM2){
    ICLFaultTreatedM2 = false;
  }
  else
  {
    /* Nothing to do */
  }
  </#if><#-- (MC.M2_ICL_ENABLED == true) -->

  if (MCI_GetFaultState(&Mci[bMotor]) != (uint32_t)MC_NO_FAULTS)
  {
  <#if (MC.MOTOR_PROFILER == true)>
      SCC_Stop(&SCC);
      OTT_Stop(&OTT);
  </#if><#-- (MC.MOTOR_PROFILER == true) -->
  <#if M1_ENCODER || M2_ENCODER>
    /* Reset Encoder state */
    if (pEAC[bMotor] != MC_NULL)
    {
      EAC_SetRestartState(pEAC[bMotor], false);
    }
    else
    {
      /* Nothing to do */
    }
  </#if><#-- M1_ENCODER || M2_ENCODER -->
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
  <#if MC.MCP_ASYNC_OVER_UART_A_EN>
    if (MCPA_UART_A.Mark != 0U)
    {
      MCPA_flushDataLog (&MCPA_UART_A);
    }
    else
    {
      /* Nothing to do */
    }
  </#if><#-- MC.MCP_ASYNC_OVER_UART_A_EN -->
  <#if MC.MCP_ASYNC_OVER_UART_B_EN>
    if (MCPA_UART_B.Mark != 0)
    {
      MCPA_flushDataLog (&MCPA_UART_B);
    }
    else
    {
      /* Nothing to do */
    }
  </#if><#-- C.MCP_DATALOG_OVER_UART_B -->
  <#if MC.MCP_ASYNC_OVER_STLNK_EN>
    if (MCPA_STLNK.Mark != 0)
    {
      MCPA_flushDataLog (&MCPA_STLNK);
    }
    else
    {
      /* Nothing to do */
    }
  </#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
    ${MC.M1_DRIVE_TYPE}_Clear(bMotor);
    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
  }
  else
  {
    /* No errors */
  }
  <#if  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE == true>
  /* Smooth braking action on overvoltage */
  if(M1 == bMotor)
  {
    busd = (uint16_t)VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super));
  }
  else if(M2 == bMotor)
  {
    busd = (uint16_t)VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M2._Super));
  }

  if ((Mci[bMotor].State == IDLE)||
     ((Mci[bMotor].State== RUN)&&(FOCVars[bMotor].Iqdref.q>0)))
  {
    nominalBusd[bMotor] = busd;
  }
  else
  {
    if((Mci[bMotor].State == RUN) && (FOCVars[bMotor].Iqdref.q<0))
    {
      if (busd > ((ovthd[bMotor] + nominalBusd[bMotor]) >> 1))
      {
        FOCVars[bMotor].Iqdref.q = 0;
        FOCVars[bMotor].Iqdref.d = 0;
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
  }
    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF SMOOTH_BREAKING */

    /* USER CODE END TSK_SafetyTask_PWMOFF SMOOTH_BREAKING */
  </#if><#-- MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE -->
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}
</#if><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_OFF_PWM" || MC.M2_ON_OVER_VOLTAGE == "TURN_OFF_PWM" -->
<#if MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE">
/**
  * @brief  Safety task implementation if  MC.M1_ON_OVER_VOLTAGE == TURN_ON_R_BRAKE.
  * @param  motor Motor reference number defined
  *         \link Motors_reference_number here \endlink.
  */
__weak void TSK_SafetyTask_RBRK(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_RBRK 0 */

  /* USER CODE END TSK_SafetyTask_RBRK 0 */
  uint16_t CodeReturn = MC_NO_ERROR;
  uint16_t BusVoltageFaultsFlag = MC_OVER_VOLT;
  <#if (MC.M1_BUS_VOLTAGE_READING == true) || (MC.M2_BUS_VOLTAGE_READING == true)>
    <#if MC.DRIVE_NUMBER != "1">
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK, VBUS_TEMP_ERR_MASK2};
    <#else><#-- MC.DRIVE_NUMBER == 1 -->
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
    </#if><#-- MC.DRIVE_NUMBER > 1 -->
  </#if>
  /* Brake resistor management */
  <#if MC.M1_BUS_VOLTAGE_READING == true>
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 =  RCM_ExecRegularConv(&VbusRegConv_M1);
    BusVoltageFaultsFlag =  errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }
  <#else><#--MC.M1_BUS_VOLTAGE_READING == true -->
 <#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
  </#if><#-- MC.M1_BUS_VOLTAGE_READING == true -->
  <#if MC.M2_BUS_VOLTAGE_READING == true>
  if (M2 == bMotor)
  {
    uint16_t rawValueM2 =  RCM_ExecRegularConv(&VbusRegConv_M2);
    BusVoltageFaultsFlag = errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M2, rawValueM2);
  }
  else
  {
    /* Nothing to do */
  }
  <#else><#-- MC.M2_BUS_VOLTAGE_READING == false -->
<#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
  </#if><#-- MC.M2_BUS_VOLTAGE_READING == true -->
  if (MC_OVER_VOLT == BusVoltageFaultsFlag)
  {
    DOUT_SetOutputState(pR_Brake[bMotor], ACTIVE);
  }
  else
  {
    DOUT_SetOutputState(pR_Brake[bMotor], INACTIVE);
  }
  /* Check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
<#if (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true)>
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 = RCM_ExecRegularConv(&TempRegConv_M1);
    CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(&TempSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }  
</#if>  
<#if (MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true)>
  if (M2 == bMotor)
  {
    uint16_t rawValueM2 = RCM_ExecRegularConv(&TempRegConv_M2);
    CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(&TempSensor_M2, rawValueM2);
  }
  else
  {
    /* Nothing to do */
  }  
</#if>
  CodeReturn |= PWMC_IsFaultOccurred(pwmcHandle[bMotor]);     /* Check for fault. It return MC_OVER_CURR or MC_NO_FAULTS
                                                    (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  CodeReturn |= (BusVoltageFaultsFlag & MC_UNDER_VOLT);  /* MC_UNDER_VOLT generates fault if FW protection is activated,
                                                                                  MC_OVER_VOLT doesn't generate fault */
  MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn);  /* Update the STM according error code */

  <#if (MC.M1_ICL_ENABLED == true)>
  if ((M1 == bMotor) && (MC_UNDER_VOLT == (BusVoltageFaultsFlag & MC_UNDER_VOLT)) && ICLFaultTreatedM1)
  {
    ICLFaultTreatedM1 = false;
  }
  else
  {
    /* Nothing to do */
  }
  </#if><#-- (MC.M1_ICL_ENABLED == true) -->

  <#if (MC.M2_ICL_ENABLED == true)>
  if ((M2 == bMotor) && (MC_UNDER_VOLT == (BusVoltageFaultsFlag & MC_UNDER_VOLT)) && ICLFaultTreatedM2)
  {
    ICLFaultTreatedM2 = false;
  }
  else
  {
    /* Nothing to do */
  }
  </#if><#-- (MC.M2_ICL_ENABLED == true) -->
  
  if (MCI_GetFaultState(&Mci[bMotor]) != (uint32_t)MC_NO_FAULTS)
  {
  <#if (MC.MOTOR_PROFILER == true)>
      SCC_Stop(&SCC);
      OTT_Stop(&OTT);
  </#if><#-- (MC.MOTOR_PROFILER == true) -->
  <#if M1_ENCODER || M2_ENCODER>
      /* Reset Encoder state */
      if (pEAC[bMotor] != MC_NULL)
      {
        EAC_SetRestartState( pEAC[bMotor], false );
      }
      else
      {
        /* Nothing to do */
      }
  </#if><#-- M1_ENCODER || M2_ENCODER -->
      PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
  <#if MC.MCP_ASYNC_OVER_UART_A_EN>
    if (MCPA_UART_A.Mark != 0U)
    { /* Dual motor not yet supported */
      MCPA_flushDataLog (&MCPA_UART_A);
    }
    else
    {
      /* Nothing to do */
    }
  </#if><#-- MC.MCP_ASYNC_OVER_UART_A_EN -->
  <#if MC.MCP_ASYNC_OVER_UART_B_EN>
    if (MCPA_UART_B.Mark != 0)
    { /* Dual motor not yet supported */
      MCPA_flushDataLog (&MCPA_UART_B);
    }
    else
    {
      /* Nothing to do */
    }
  </#if><#-- MC.MCP_ASYNC_OVER_UART_B_EN -->
  <#if MC.MCP_ASYNC_OVER_STLNK_EN>
    if (MCPA_STLNK.Mark != 0)
    { /* Dual motor not yet supported */
      MCPA_flushDataLog (&MCPA_STLNK);
    }
    else
    {
      /* Nothing to do */
    }
  </#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
    ${MC.M1_DRIVE_TYPE}_Clear(bMotor);
    /* USER CODE BEGIN TSK_SafetyTask_RBRK 1 */

    /* USER CODE END TSK_SafetyTask_RBRK 1 */
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN TSK_SafetyTask_RBRK 2 */

  /* USER CODE END TSK_SafetyTask_RBRK 2 */
}
</#if><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_R_BRAKE" -->
<#if MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES">
/**
  * @brief  Safety task implementation if  MC.M1_ON_OVER_VOLTAGE == TURN_ON_LOW_SIDES.
  * @param  motor Motor reference number defined
  *         \link Motors_reference_number here \endlink.
  */
__weak void TSK_SafetyTask_LSON(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_LSON 0 */

  /* USER CODE END TSK_SafetyTask_LSON 0 */
  uint16_t CodeReturn = MC_NO_ERROR;
  <#if MC.DRIVE_NUMBER != "1">
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK, VBUS_TEMP_ERR_MASK2};
  <#else><#-- MC.DRIVE_NUMBER == 1 -->
  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
  </#if><#-- MC.DRIVE_NUMBER > 1 -->
  bool TurnOnLowSideAction;
  
  TurnOnLowSideAction = PWMC_GetTurnOnLowSidesAction(pwmcHandle[bMotor]);
  /* Check for fault if FW protection is activated */
<#if (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true)>
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 = RCM_ExecRegularConv(&TempRegConv_M1);
    CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(&TempSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }  
</#if>  
<#if (MC.M2_TEMPERATURE_READING == true  && MC.M2_OV_TEMPERATURE_PROT_ENABLING == true)>
  if (M2 == bMotor)
  {
    uint16_t rawValueM2 = RCM_ExecRegularConv(&TempRegConv_M2);
    CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(&TempSensor_M2, rawValueM2);
  }
  else
  {
    /* Nothing to do */
  }  
</#if>
  CodeReturn |= PWMC_IsFaultOccurred(pwmcHandle[bMotor]); /* For fault. It return MC_OVER_CURR or MC_NO_FAULTS
                                                   (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  /* USER CODE BEGIN TSK_SafetyTask_LSON 1 */

  /* USER CODE END TSK_SafetyTask_LSON 1 */
  <#if MC.M1_BUS_VOLTAGE_READING == true>
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 =  RCM_ExecRegularConv(&VbusRegConv_M1);  
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }
  <#else><#-- MC.M1_BUS_VOLTAGE_READING == false -->
  <#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
  </#if><#-- MC.M1_BUS_VOLTAGE_READING == true -->
  <#if MC.M2_BUS_VOLTAGE_READING == true>
  if (M2 == bMotor)
  {
    uint16_t rawValueM2 =  RCM_ExecRegularConv(&VbusRegConv_M2);
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M2, rawValueM2);
  }
  else
  {
    /* Nothing to do */
  }
  <#else><#-- MC.M2_BUS_VOLTAGE_READING == false -->
  <#-- Nothing to do here the virtual voltage does not need computations nor measurement and it cannot fail... -->
  </#if><#-- MC.M2_BUS_VOLTAGE_READING == true -->
  MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  
  <#if (MC.M1_ICL_ENABLED == true)>
  if ((M1 == bMotor) && (MC_UNDER_VOLT == (CodeReturn & MC_UNDER_VOLT)) && ICLFaultTreatedM1){
    ICLFaultTreatedM1 = false;
  }
  else
  {
    /* Nothing to do */
  }
  </#if><#-- (MC.M1_ICL_ENABLED == true) -->

  <#if (MC.M2_ICL_ENABLED == true)>
  if ((M2 == bMotor) && (MC_UNDER_VOLT == (CodeReturn & MC_UNDER_VOLT)) && ICLFaultTreatedM2)
  {
    ICLFaultTreatedM2 = false;
  }
  else
  {
    /* Nothing to do */
  }
  </#if><#-- (MC.M2_ICL_ENABLED == true) -->
  
  if ((MC_OVER_VOLT == (CodeReturn & MC_OVER_VOLT)) && (false == TurnOnLowSideAction))
  {
  <#if M1_ENCODER || M2_ENCODER>
    /* Reset Encoder state */
    if (pEAC[bMotor] != MC_NULL)
    {
      EAC_SetRestartState(pEAC[bMotor], false);
    }
    else
    {
      /* Nothing to do */
    }
  </#if><#-- M1_ENCODER || M2_ENCODER -->
    /* Start turn on low side action */
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]); /* Required before PWMC_TurnOnLowSides */
    ${MC.M1_DRIVE_TYPE}_Clear(bMotor);
  <#if (MC.M1_HW_OV_CURRENT_PROT_BYPASS == true)>
    DOUT_SetOutputState(pOCPDisabling[bMotor], ACTIVE); /* Disable the OCP */
  </#if><#-- (MC.M1_HW_OV_CURRENT_PROT_BYPASS == true) -->
    /* USER CODE BEGIN TSK_SafetyTask_LSON 2 */

    /* USER CODE END TSK_SafetyTask_LSON 2 */
    PWMC_TurnOnLowSides(pwmcHandle[bMotor], 0UL); /* Turn on Low side switches */
  }
  else
  {
    switch (Mci[bMotor].State) /* Is state equal to FAULT_NOW or FAULT_OVER */
    {
    
      case IDLE:
      {
        /* After a OV occurs the turn on low side action become active. It is released just after a fault acknowledge
         * -> state == IDLE */
        if (true == TurnOnLowSideAction)
        {
          /* End of TURN_ON_LOW_SIDES action */
  <#if (MC.M1_HW_OV_CURRENT_PROT_BYPASS == true)>
          DOUT_SetOutputState(pOCPDisabling[bMotor], INACTIVE); /* Re-enable the OCP */
  </#if><#-- (MC.M1_HW_OV_CURRENT_PROT_BYPASS == true) -->
          PWMC_SwitchOffPWM(pwmcHandle[bMotor]);  /* Switch off the PWM */
        }
        else
        {
          /* Nothing to do */
        }
        /* USER CODE BEGIN TSK_SafetyTask_LSON 3 */

        /* USER CODE END TSK_SafetyTask_LSON 3 */
        break;
      }
      
      case FAULT_NOW:
      {
        if (TurnOnLowSideAction == false)
        {
  <#if M1_ENCODER || M2_ENCODER>
          /* Reset Encoder state */
          if (pEAC[bMotor] != MC_NULL)
          {
            EAC_SetRestartState(pEAC[bMotor], false);
          }
          else
          {
            /* Nothing to do */
          }
  </#if><#-- M1_ENCODER || M2_ENCODER -->
          /* Switching off the PWM if fault occurs must be done just if TURN_ON_LOW_SIDES action is not in place */
          PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
  <#if MC.MCP_ASYNC_OVER_UART_A_EN>
          if (MCPA_UART_A.Mark != 0U)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_UART_A);
          }
          else
          {
            /* Nothing to do */
          }
  </#if><#-- MC.MCP_ASYNC_OVER_UART_A_EN -->
  <#if MC.MCP_ASYNC_OVER_UART_B_EN>
          if (MCPA_UART_B.Mark != 0)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_UART_B);
          }
          else
          {
            /* Nothing to do */
          }
  </#if><#-- MC.MCP_ASYNC_OVER_UART_B_EN -->
  <#if MC.MCP_ASYNC_OVER_STLNK_EN>
          if (MCPA_STLNK.Mark != 0)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_STLNK);
          }
          else
          {
            /* Nothing to do */
          }
  </#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
          ${MC.M1_DRIVE_TYPE}_Clear(bMotor);
        }
        /* USER CODE BEGIN TSK_SafetyTask_LSON 4 */

        /* USER CODE END TSK_SafetyTask_LSON 4 */
        break;
      }
      
      case FAULT_OVER:
      {
        if (TurnOnLowSideAction == false)
        {
          /* Switching off the PWM if fault occurs must be done just if TURN_ON_LOW_SIDES action is not in place */
          PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
  <#if MC.MCP_ASYNC_OVER_UART_A_EN>
          if (MCPA_UART_A.Mark != 0U)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_UART_A);
          }
          else
          {
            /* Nothing to do */
          }
  </#if><#-- MC.MCP_ASYNC_OVER_UART_A_EN -->
  <#if MC.MCP_ASYNC_OVER_UART_B_EN>
          if (MCPA_UART_B.Mark != 0)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_UART_B);
          }
          else
          {
            /* Nothing to do */
          }
  </#if><#-- MC.MCP_ASYNC_OVER_UART_B_EN -->
  <#if MC.MCP_ASYNC_OVER_STLNK_EN>
          if (MCPA_STLNK.Mark != 0)
          { /* Dual motor not yet supported */
            MCPA_flushDataLog (&MCPA_STLNK);
          }
          else
          {
            /* Nothing to do */
          }
  </#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
        }
        /* USER CODE BEGIN TSK_SafetyTask_LSON 5 */

        /* USER CODE END TSK_SafetyTask_LSON 5 */
        break;
      }
      
      default:
        break;
    }
  }
  /* USER CODE BEGIN TSK_SafetyTask_LSON 6 */

  /* USER CODE END TSK_SafetyTask_LSON 6 */
}
</#if><#-- MC.M1_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" || MC.M2_ON_OVER_VOLTAGE == "TURN_ON_LOW_SIDES" -->

<#if MC.DRIVE_NUMBER != "1">
#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Reserves FOC execution on ADC ISR half a PWM period in advance.
  *
  *  This function is called by TIMx_UP_IRQHandler in case of dual MC and
  * it allows to reserve half PWM period in advance the FOC execution on
  * ADC ISR.
  * @param  pDrive Pointer on the FOC Array.
  */
__weak void TSK_DualDriveFIFOUpdate(uint8_t Motor)
{
<#if HIGH_FREQ_TRIGGER == "TIM_UP_IT">
  FOC_array[FOC_array_tail] = Motor;
  FOC_array_tail++;
  if (FOC_ARRAY_LENGTH == FOC_array_tail)
  {
    FOC_array_tail = 0;
  }
  else
  {
    /* Nothing to do */
  }
</#if><#-- HIGH_FREQ_TRIGGER == "TIM_UP_IT"-->  
}
</#if><#-- MC.DRIVE_NUMBER > 1 -->

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected  
  * by the microcontroller and is used to put the system in safety condition.
  */
__weak void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */
<#if FOC>
  <#if (MC.MOTOR_PROFILER == true)>
  SCC_Stop(&SCC);
  OTT_Stop(&OTT);
  </#if><#-- (MC.MOTOR_PROFILER == true) -->
</#if><#-- FOC -->
   ${MC.M1_DRIVE_TYPE}_Clear(M1);
  MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0);
<#if MC.DRIVE_NUMBER != "1">
   ${MC.M1_DRIVE_TYPE}_Clear(M2);
  MCI_FaultProcessing(&Mci[M2], MC_SW_ERROR, 0);
</#if><#-- MC.DRIVE_NUMBER > 1 -->

  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}
<#if MC.PFC_ENABLED == true>

/**
  * @brief  Executes the PFC Task.
  */
void PFC_Scheduler(void)
{
  PFC_Task(&PFC);
}
</#if><#-- MC.PFC_ENABLED == true -->
<#if MC.RTOS == "FREERTOS">

/* startMediumFrequencyTask function */
void startMediumFrequencyTask(void const * argument)
{
  <#if MC.CUBE_MX_VER == "xxx">
  /* Init code for MotorControl */
  MX_MotorControl_Init();
  <#else><#-- MC.CUBE_MX_VER != "xxx" -->
<#assign cubeVersion = MC.CUBE_MX_VER?replace(".","") >
    <#if cubeVersion?number < 540>
  /* Init code for MotorControl */
  MX_MotorControl_Init();
    </#if><#-- cubeVersion?number < 540 -->
  </#if><#-- MC.CUBE_MX_VER == "xxx" -->
  /* USER CODE BEGIN MF task 1 */

  /* Infinite loop */
  for(;;)
  {
    /* Delay of 500us */
    vTaskDelay(1);

  <#if MCP_DMA>
    /* Buffer is ready by the HW layer to be processed */
    /* NO DMA interrupt */
    <#if MC.MCP_OVER_UART_A_EN>
    if (LL_DMA_IsActiveFlag_TC(DMA_RX_A, DMACH_RX_A))
    {
      LL_DMA_ClearFlag_TC(DMA_RX_A, DMACH_RX_A);
      ASPEP_HWDataReceivedIT(&aspepOverUartA);
    }
    else
    {
      /* Nothing to do */
    }
    </#if><#--  MC.MCP_OVER_UART_A_EN -->
    <#if MC.MCP_OVER_UART_B_EN>
    if (LL_DMA_IsActiveFlag_TC(DMA_RX_B, DMACH_RX_B))
    {
      LL_DMA_ClearFlag_TC(DMA_RX_B, DMACH_RX_B);
      ASPEP_HWDataReceivedIT(&aspepOverUartB);
    }
    else
    {
      /* Nothing to do */
    }
    </#if><#--  MC.MCP_OVER_UART_B_EN -->
  <#else><#-- !MCP_DMA -->
    <#if MC.MCP_OVER_UART_A_EN>
    if(aspepOverUartA.rxComplete == true)
    {
      ASPEP_HWDataReceivedIT(&aspepOverUartA);
      aspepOverUartA.rxComplete = false;
    }
    else if(aspepOverUartA.txComplete == true)
    {
      ASPEP_HWDataTransmittedIT(&aspepOverUartA);
      aspepOverUartA.txComplete = false;
    }
    </#if><#-- MC.MCP_OVER_UART_A_EN -->
  </#if><#-- MCP_DMA -->

    MC_RunMotorControlTasks();
  }
  /* USER CODE END MF task 1 */
}

/* startSafetyTask function */
void StartSafetyTask(void const * argument)
{
  /* USER CODE BEGIN SF task 1 */
  /* Infinite loop */
  for(;;)
  {
    /* Delay of 500us */
    vTaskDelay(1);
    TSK_SafetyTask();
  }
  /* USER CODE END SF task 1 */ 
}

</#if><#-- MC.RTOS == "FREERTOS" -->

<#if MC.START_STOP_BTN == true>
__weak void UI_HandleStartStopButton_cb (void)
{
/* USER CODE BEGIN START_STOP_BTN */
  if (IDLE == MC_GetSTMStateMotor1())
  {
    /* Ramp parameters should be tuned for the actual motor */
    (void)MC_StartMotor1();
  }
  else
  {
    (void)MC_StopMotor1();
  }
/* USER CODE END START_STOP_BTN */
}
</#if><#-- MC.START_STOP_BTN == true -->

 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration.
  */
__weak void mc_lock_pins (void)
{
<#list configs as dt>
<#list dt.peripheralGPIOParams.values() as io>
<#list io.values() as ipIo>
<#list ipIo.entrySet() as e>
<#if (e.getKey().equals("GPIO_Label")) && (e.getValue()?matches("^M[0-9]+_.*$"))>
LL_GPIO_LockPin(${e.getValue()}_GPIO_Port, ${e.getValue()}_Pin);
</#if><#-- (e.getKey().equals("GPIO_Label")) && (e.getValue()?matches("^M[0-9]+_.*$")) -->
</#list>
</#list>
</#list>
</#list>
}
/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
