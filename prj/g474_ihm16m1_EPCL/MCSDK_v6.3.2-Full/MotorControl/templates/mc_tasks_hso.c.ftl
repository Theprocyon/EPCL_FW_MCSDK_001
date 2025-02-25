<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
/**
 ******************************************************************************
 * @file    mc_tasks.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file implements tasks routines
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
 * @ingroup MCTasks_hso
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "mc_interface.h"
#include "pwm_common.h"
<#if MC.DEBUG_DAC_FUNCTIONALITY_EN == true> 
#include "dac_ui.h"
</#if>
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "register_interface.h"
<#if MC.MCP_EN == true>
#include "mcp_config.h"
</#if>
#include "mc_curr_ctrl.h"
#include "speed_pos_fdbk_hso.h"
#include "speed_torq_ctrl_hso.h"   
#include "rsdc_est.h"
#include "mc_polpulse.h"
#ifdef EEPROM_EMULATION
#include "eeprom_emul.h"
#endif /* EEPROM_EMULATION */

#include "fixpmath.h"
#include "oversampling.h"  
<#if (MC.M1_POTENTIOMETER_ENABLE == true)>
  <#if (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_U) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_V) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_W)>
#include "mc_app_hooks.h"
  </#if><#-- (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_U) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_V) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_W) -->
</#if><#-- (MC.M1_POTENTIOMETER_ENABLE == true) -->
<#if MC.MOTOR_PROFILER == true>
#include "profiler.h"
</#if>
/** @defgroup MCTasks_hso Motor Control Tasks HSO
  *
  * @brief Motor Control subsystem configuration and operation routines.
  *
  * Implements Motor Control subsystem application. For more information please
  * refer to [FOC documentation](docs/zest_architecture.md)
  *
  * @{
  */
  
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */
/*! @brief  defines the type of error to check in the safety task */
#define VBUS_TEMP_ERR_MASK (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)

#define M1_CHARGE_BOOT_CAP_TICKS          (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)${MC.M1_PWM_CHARGE_BOOT_CAP_MS}) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)${MC.M1_PWM_CHARGE_BOOT_CAP_DUTY_CYCLES}\
                                      * ((uint32_t)PWM_PERIOD_CYCLES / 2U))

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t * pMCInterface[NBR_OF_MOTORS];
BusVoltageSensor_Handle_t *pVBus[NBR_OF_MOTORS];
PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
STC_Handle_t *pSTC[NBR_OF_MOTORS];
CurrCtrl_Handle_t *pCurrCtrlhandle[NBR_OF_MOTORS];
<#if MC.M1_SPEED_SENSOR == "ZEST">
ZEST_Obj *pZeST[NBR_OF_MOTORS];
</#if>
SPD_Handle_t *pSPD[NBR_OF_MOTORS];
<#if MC.M1_POTENTIOMETER_ENABLE == true>
Potentiometer_Handle_t *pPotentiometer[NBR_OF_MOTORS];
</#if>
static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;

uint8_t bMCBootCompleted = 0;

<#if MC.MOTOR_PROFILER == true>
PROFILER_Obj	profilerObj[NBR_OF_MOTORS];
</#if>

/* Performs the CPU load measure of FOC main tasks. */
<#if MC.DBG_MCU_LOAD_MEASURE == true>
MC_Perf_Handle_t PerfTraces;
</#if>

<#if (MC.M1_ICL_ENABLED == true)>
static volatile bool ICLFaultTreatedM1 = true; 
</#if>

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/

void TSK_MediumFrequencyTaskM1(void);
void FOC_Clear(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
void TSK_SafetyTask_PWMOFF(uint8_t motor);
void FOC_ReferenceSelection(uint8_t bMotor);
void MC_init(void);
uint16_t ErrorDetection(uint8_t bMotor);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */

#ifdef EEPROM_EMULATION
void GetFloatParamFromEEPROM(int EEPROM_Addr,float *param);
void GetUint32ParamFromEEPROM(int EEPROM_Addr,uint32_t *param);
#endif /* EEPROM_EMULATION */

__weak void MCboot_init( MCI_Handle_t* pMCIList[NBR_OF_MOTORS] );

/* Reference to pMCI and pMCT to re-run init without LoadConfiguration */
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];

/**
  * @brief  Initializes the whole motor control subsystem according to user 
  *         configuration and parameters.
  * @param  pMCIList motor control interface data structure that will be 
  *         initialized. 
  */
__weak void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */
 
  /**************************************/
  /*    State machine initialization    */
  /**************************************/
  bMCBootCompleted = 0;

  /**********************************************************/
  /*    PWM and current sensing component initialization    */
  /**********************************************************/
  pwmcHandle[M1] = &PWM_Handle_M1._Super;
  R3_Init(&PWM_Handle_M1, scaleParams);
<#if MC.MCP_EN == true>
  ASPEP_start (&aspepOverUartA);

  MCP_RegisterCallBack (0,flashWrite_cb);
  MCP_RegisterCallBack (1,flashRead_cb);

</#if>

  /* USER CODE BEGIN MCboot 1 */

  /* USER CODE END MCboot 1 */

  /******************************************************/
  /*   Main speed sensor component initialization       */
  /******************************************************/
  pMCInterface[M1] = &Mci[M1];

  MCI_Init(pMCInterface[M1], &FOCVars[M1] );
  pMCIList[M1] = pMCInterface[M1];

  /* USER CODE BEGIN MCboot 2 */

  /* USER CODE END MCboot 2 */

  /*******************************************************/
  /*    speed and torque controller initialization       */
  /*******************************************************/  
  STC_Init(&STC_M1, &flashParams);
  pSTC[M1] = &STC_M1;
    
  /*******************************************************/
  /*   Sensor less controller initialization             */
  /*******************************************************/
  SPD_Init(&SPD_M1,&flashParams);
  pSPD[M1] = &SPD_M1;
<#if MC.M1_SPEED_SENSOR == "ZEST">
  pZeST[M1] = &ZeST_M1;
</#if>  

  /*******************************************************/
  /*   Current controller initialization                 */
  /*******************************************************/  
  MC_currentControllerInit(&CurrCtrl_M1, &flashParams);
  pCurrCtrlhandle[M1] = &CurrCtrl_M1;

<#if MC.M1_BUS_VOLTAGE_READING>    
  <#if (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_U) && (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_V) && (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_W)>
  /*******************************************************/
  /*   Vbus reading initialization                       */
  /*******************************************************/
  (void)RCM_RegisterRegConv(&VbusRegConv_M1);
  
  </#if>
</#if> 
  /*******************************************************/
  /*   Temperature measurement component initialization  */
  /*******************************************************/
<#if (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true)>
  <#if (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_U) && (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_V) && (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_W)>
  (void)RCM_RegisterRegConv(&TempRegConv_M1);
  </#if>
</#if>
  NTC_Init(&TempSensor_M1);  
    
  /*******************************************************/
  /*   PolPulse component initialization                 */
  /*******************************************************/
  MC_POLPULSE_init(&MC_PolPulse_M1, &PolPulse_params_M1,&flashParams);

<#if MC.MOTOR_PROFILER == true>
  /*******************************************************/
  /*   Profiler component initialization                 */
  /*******************************************************/
  PROFILER_init(&profiler_M1, &profilerParams_M1, &flashParams);
  profilerMotor_M1.pFocVars = &FOCVars[M1];

</#if>    
  Mci[M1].pPWM = pwmcHandle[M1];
  Mci[M1].pSPD = &SPD_M1;
  Mci[M1].pSTC = &STC_M1;
  Mci[M1].pCurrCtrl = &CurrCtrl_M1;
  Mci[M1].pRsDCEst = &RsDCEst_M1;
  Mci[M1].pPolPulse = &MC_PolPulse_M1;
  Mci[M1].pProfiler = &profiler_M1;
  Mci[M1].pVBus = &VBus_M1;
<#if MC.DBG_MCU_LOAD_MEASURE == true>
  Mci[M1].pPerfMeasure = &PerfTraces;
</#if>
<#if MC.M1_POTENTIOMETER_ENABLE == true>
  Mci[M1].pPotentiometer = &potentiometer_M1;
  pPotentiometer[M1] = &potentiometer_M1;
</#if>  
  /* Perform additional initialization */
  MC_init();

  FOC_Clear(M1);
<#if (MC.M1_DBG_OPEN_LOOP_ENABLE == false) && (MC.MOTOR_PROFILER == false)> 
  
</#if> 
<#if MC.DEBUG_DAC_FUNCTIONALITY_EN == true>  
  DAC_Init(&DAC_Handle);
</#if>  
<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Measure_Init(&PerfTraces);
</#if>

<#if  MC.M1_ICL_ENABLED == true>
  ICL_Init(&ICL_M1, &ICLDOUTParamsM1);
  Mci[M1].State = ICLWAIT;
</#if>

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

<#if (MC.M1_POTENTIOMETER_ENABLE == true)>
  <#if (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_U) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_V) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_W)>
  /* Applicative hook in MCBoot() */
  MC_APP_BootHook();
  </#if><#-- (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_U) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_V) && (MC.POTENTIOMETER_ADC !=  MC.M1_CS_ADC_W) -->
</#if><#-- (MC.M1_POTENTIOMETER_ENABLE == true) -->

  LL_TIM_ClearFlag_UPDATE(TIM3);
  /* start timers synchronously */
  startTimers();

#if (OVS_COUNT !=1)
  while(LL_TIM_IsActiveFlag_UPDATE(TIM3) == 0)
  {}
#endif

  /* Start conversions */
  LL_ADC_REG_StartConversion(${ADCX});
  LL_ADC_REG_StartConversion(${ADCY});
<#if MC.M1_CS_ADC_NUM == "3">
  LL_ADC_REG_StartConversion(${ADCZ});
</#if>

  bMCBootCompleted = 1;
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task
 * @param pHandle Motor Control Intercafe data structure
 * @param motor Motor ID
 */
void TSK_MF_StopProcessing(  MCI_Handle_t * pHandle, uint8_t motor)
{
  R3_SwitchOffPWM(pwmcHandle[motor]);
  FOC_Clear(motor);
  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
  return;
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is called periodically at least at the Medium Frequency task
 * rate. Exact invocation rate is the Speed regulator execution rate.
 *
 * It executes  the Medium Frequency Tasks  processes communication 
 * TX/RX packets. 
 */
__weak void MC_RunMotorControlTasks(void)
{
  if ( bMCBootCompleted ) {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();
  }
}

/**
 * @brief  Schedules background routines.
 *
 * MC_scheduler executes Medium frequency task and [motor control protocol]() 
 * communication at a frequency defined by SPEED_LOOP_FREQUENCY_HZ.
 *
 */
__weak void MC_Scheduler(void)
{
  /* USER CODE BEGIN MC_Scheduler 0 */

  /* USER CODE END MC_Scheduler 0 */

  if (bMCBootCompleted == 1)
  {    
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      TSK_MediumFrequencyTaskM1();
<#if MC.MCP_EN == true>
      MCP_Over_UartA.rxBuffer = MCP_Over_UartA.pTransportLayer->fRXPacketProcess ( MCP_Over_UartA.pTransportLayer,  &MCP_Over_UartA.rxLength);
      if (MCP_Over_UartA.rxBuffer)
      {
        /* Synchronous answer */
        if (MCP_Over_UartA.pTransportLayer->fGetBuffer (MCP_Over_UartA.pTransportLayer, (void **) &MCP_Over_UartA.txBuffer, MCTL_SYNC))
        {
          MCP_ReceivedPacket(&MCP_Over_UartA);
          MCP_Over_UartA.pTransportLayer->fSendPacket (MCP_Over_UartA.pTransportLayer, MCP_Over_UartA.txBuffer, MCP_Over_UartA.txLength, MCTL_SYNC);
        }
        else
        {
          /* no buffer available to build the answer ... should not occur */
        }
      }

</#if>
      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
      hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
    }
    if(hBootCapDelayCounterM1 > 0u)
    {
      hBootCapDelayCounterM1--;
    }
    if(hStopPermanencyCounterM1 > 0u)
    {
      hStopPermanencyCounterM1--;
    } 
  }
  else
  {
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
 * @brief Executes medium frequency tasks
 *
 * This function manages the motor control [state machine](docs/ZeST_StateMachine.md) and executes 
 * background routines that requires heavy computation.
 *
 */
__weak void TSK_MediumFrequencyTaskM1(void)
{

<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_BG_Perf_Measure_Start(&PerfTraces, MEASURE_TSK_MediumFrequencyTaskM1);
</#if>

  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */
  FOCVars_t* pFOCVars = &FOCVars[M1];
  fixp30_t emfSpeed;
  fixp30_t speedLP;

<#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") >
  /* check if BRK or BRK2 has been disabled after a PWM switch on to prevent false 
     error detection during wakeup phase */
  if (READ_BIT(${_last_word(MC.M1_PWM_TIMER_SELECTION)}->BDTR, TIM_BDTR_${brkEnBit}) != (TIM_BDTR_${brkEnBit}))
  {
    /* check if nFault signal level is back to high */
    if (1UL == LL_GPIO_IsInputPinSet(M1_DP_GPIO_Port, M1_DP_Pin))
    {
      /* nFault signal level back to high, wakeup phase is over */
      LL_TIM_ClearFlag_${brkNb}(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
      LL_TIM_Enable${brkNb}(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
    }
  }
</#if>
   
  <#if  MC.M1_ICL_ENABLED == true>
  uint16_t Vbus = (uint16_t)(FIXP30_toF(VBus_M1.Udcbus_in_pu) * VOLTAGE_SCALE);
  ICL_State_t ICLstate = ICL_Exec(&ICL_M1, Vbus);
  </#if>

<#if MC.M1_SPEED_SENSOR == "ZEST">
  if (STC_M1.Enable_InjectBoost == true)
  {
    STC_M1.K_InjectBoost = FIXP24(1.0) + FIXP30_mpy(FIXP_abs(STC_M1.pCurrCtrl->Idq_in_pu.Q), MCI_GetOneOverMaxIq(pMCI[M1]));
  }
  else
  {
    STC_M1.K_InjectBoost = FIXP24(1.0);  // No boost applied 
  }

  /* Update ZEST controls and feedback */
  if (SPD_M1.zestControl.enableControlUpdate)
  {
    SPD_M1.Idq_in_pu.Q = STC_M1.pCurrCtrl->Idq_in_pu.Q; 
    MC_ZEST_Control_Update(&SPD_M1);
  }  

  /* ZEST Background task is run at 1 kHz, as specified in the initialization */

  if (SPD_M1.zestControl.enableRunBackground)
  {

    fixp30_t thresholdFreq_pu = ZEST_getThresholdFreq_pu(&ZeST_M1);
    fixp30_t speed_pu = FIXP_abs(HSO_getEmfSpeed_pu(&HSO_M1));
    if ( speed_pu > (thresholdFreq_pu << 1) )
    {
      /* Speed clamped to zest threshold freq value to avoid zero the fraction without
         fade-out when speed is over 2x threshold */
      speed_pu = thresholdFreq_pu;  
    }
    ZEST_runBackground(&ZeST_M1, speed_pu);

    /* If we detect that the sensorless algorithm has locked on in the negative D direction, we flip the
     * angle in the flux observer */
    fixp30_t DirCheck = HSO_getCheckDir(&HSO_M1);
    if (DirCheck > SPD_M1.DirCheck_threshold_pu)
    {
      if (STC_M1.Idq_ref_pu.Q > 0)
      {
        HSO_adjustAngle_pu(&HSO_M1, FIXP30(0.25f));    // 90 degrees
      }
      else {
        HSO_adjustAngle_pu(&HSO_M1, FIXP30(-0.25f));   // -90 degrees
      }
    }

    if (ZEST_getInjectFactor(&ZeST_M1) < FIXP(0.8f))
    {
      HSO_setMinCrossOver_Hz(&HSO_M1, 10.0f);
    }
    else
    {
      HSO_setMinCrossOver_Hz(&HSO_M1, SPD_M1.MinCrossOverAtLowSpeed_Hz);
    }

  }

  /* Resistance update during RUN state*/
  {
    RSEST_runBackground(&RsEst_M1);
    /* execute float calculations at lower rate */
    if (RSEST_getDoBackGround(&RsEst_M1))
    {    
      fixp30_t CheckRs = ZEST_getCheckRs(&ZeST_M1);
      if (ZEST_getInjectFactor(&ZeST_M1) < FIXP(0.8f))
      {
        CheckRs = 0;   /* reset during transient phase */
      }
      RSEST_runBackSlowed(&RsEst_M1, ZEST_getFlagInjectActive(&ZeST_M1), CheckRs, ZEST_getR(&ZeST_M1));
    }

    switch (SPD_M1.Rs_update_select)
    {
      case FOC_Rs_update_Auto:
      {
        if ((RUN == Mci[M1].State) && CurrCtrl_M1.currentControlEnabled)
        {
          /* Motor is running closed loop, we can estimate the resistance */
          RSEST_setUpdate_ON(&RsEst_M1, true);
        }
        else
        {
          /* Motor is not running closed loop and open loop current, go back to rated resistance */
          RSEST_setUpdate_ON(&RsEst_M1, false);
          RSEST_setRsToRated(&RsEst_M1);
        }     
        pFOCVars->Ls_Est_H = FIXP24(SPD_M1.zestFeedback.L);
      }
      break;
      case FOC_Rs_update_None:
      {
        RSEST_setRsToRated(&RsEst_M1);
      }
      break;

      default:
        break;
    }
  }
</#if>
  /* Angle compensation */
  SPD_GetSpeed(&SPD_M1, &emfSpeed, &speedLP);
  AngleCompensation_runBackground(&CurrCtrl_M1, emfSpeed);

<#if MC.MOTOR_PROFILER == true>
  /* Profiler */
  PROFILER_runBackground(&profiler_M1, &profilerMotor_M1);
</#if>

<#if (MC.M1_ICL_ENABLED == true)>
  if ( !ICLFaultTreatedM1 && (ICLstate == ICL_ACTIVE)){
    ICLFaultTreatedM1 = true; 
  }
</#if>

<#if (MC.M1_ICL_ENABLED == true)>
  if ((MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS) && ICLFaultTreatedM1)
<#else> 
  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
</#if>
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {
<#if (MC.M1_ICL_ENABLED == true)>
        case ICLWAIT:
        {
          if (ICL_INACTIVE == ICLstate)
          {
            /* If ICL is Inactive, move to IDLE */
            Mci[M1].State = IDLE; 
          }
          break;
        }
        
</#if>
        case IDLE:
        {
          if (MCI_START == Mci[M1].DirectCommand)
          {
  <#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") >
            /* deactivate driver protection input while STDRIVE101 exiting standby mode */
            LL_TIM_Disable${brkNb}(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
  </#if>
            if ((pFOCVars->flagStartWithCalibration == true) || (pFOCVars->PolarizationState == NOT_DONE))
            {
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              R3_PwmShort(pwmcHandle[M1]);  
              Mci[M1].State = OFFSET_CALIB;
            }
            else
            {
<#if MC.M1_OTF_STARTUP == false >
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              R3_PwmShort(pwmcHandle[M1]);  
              Mci[M1].State = CHARGE_BOOT_CAP;    
<#else> <#-- MC.M1_OTF_STARTUP = true -->
              FOC_Clear(M1);                                
              Mci[M1].State = RUN;  /* Proceed on RUN  */
</#if><#-- MC.M1_OTF_STARTUP = false -->
            }
          }
          else if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
          {
            TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
            R3_PwmShort(pwmcHandle[M1]);  
            Mci[M1].State = OFFSET_CALIB;
          }
        break;
        }
        
        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {
<#if MC.M1_OTF_STARTUP == false >
            if ( TSK_ChargeBootCapDelayHasElapsedM1() )
</#if>
            {
                FOC_Clear(M1);
             
                {                                
                  Mci[M1].State = RUN;  /* Proceed on RUN  */
                }

        /*   USER CODE BEGIN MediumFrequencyTask M1 Charge BootCap elapsed */
            
        /*   USER CODE END MediumFrequencyTask M1 Charge BootCap elapsed */
            }
<#if MC.M1_OTF_STARTUP == false || MC.M1_PWM_DRIVER_PN == "STDRIVE101">
            else
            {
              /* wait for boot capacitor charge */
            }
</#if>
          }
          break;
        }
        
      case OFFSET_CALIB:
      {
        if (MCI_STOP == Mci[M1].DirectCommand)
        {
          TSK_MF_StopProcessing(&Mci[M1], M1);
        }
        else
        {
          if ( TSK_ChargeBootCapDelayHasElapsedM1() )
          {
            if ((pFOCVars->PolarizationState == NOT_DONE))
            {
<#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") >
              /* deactivate driver protection input while STDRIVE101 exiting standby mode */
              LL_TIM_Disable${brkNb}(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
</#if>
              PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_START );
              pFOCVars->PolarizationState = ONGOING;
            }
            else if ( pFOCVars->PolarizationState == ONGOING )
            {
              if ( PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_EXEC ) )
              {
                pFOCVars->PolarizationState = COMPLETED;
                if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
                {
                  FOC_Clear(M1);
                  Mci[M1].DirectCommand = MCI_NO_COMMAND;
                  Mci[M1].State = IDLE;  /* Back to Idle state after Calibration procedure */
                }
                else if (pFOCVars->flagStartWithCalibration)
                {
                  pFOCVars->flagStartWithCalibration = false;
<#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") >
                  /* deactivate driver protection input while STDRIVE101 exiting standby mode */
                  LL_TIM_Disable${brkNb}(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
</#if>
                  MCI_SetControlMode(&Mci[M1],FOCVars[M1].controlMode);
                  TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                  R3_PwmShort(pwmcHandle[M1]);   
                  Mci[M1].State = CHARGE_BOOT_CAP;  /* Proceed on RUN  */
                }
                else
                {
                   Mci[M1].State = IDLE;  /* Back to Idle state after Calibration procedure */
                }
              }
              else
              {
                /* wait for offset calibration end */
              }
            }
            else
            {
              Mci[M1].State = IDLE;  /* Back to Idle in cases */
            }
          } 
          else
          {
            /* wait for boostrap capacitor to be charged */
          }
        }
        break;
      }
        
      case RUN:
      {
        /* USER CODE BEGIN MediumFrequencyTask M1 2 */
        
        /* USER CODE END MediumFrequencyTask M1 2 */
        if (MCI_STOP == Mci[M1].DirectCommand)
        {
          TSK_MF_StopProcessing(&Mci[M1], M1);
        }
        else
        {
<#if MC.M1_SPEED_SENSOR == "ZEST">
          if ( (RsDCEst_M1.RSDCestimate_Fast && RsDCEst_M1.RSDCestimate_state != RSDC_EST_COMPLETED) &&
               (pFOCVars->controlMode == MCM_SPEED_MODE || pFOCVars->controlMode == MCM_TORQUE_MODE) &&
               (pFOCVars->controlMode_prev != pFOCVars->controlMode) )
          {
              TSK_PulsesRsDC(FAST_RSDC);
          }  
          else if ( (RsDCEst_M1.flag_enableRSDCestimate && RsDCEst_M1.RSDCestimate_state != RSDC_EST_COMPLETED) &&
                    (pFOCVars->controlMode == MCM_SPEED_MODE || pFOCVars->controlMode == MCM_TORQUE_MODE) &&
                    (pFOCVars->controlMode_prev != pFOCVars->controlMode) )
<#else>
          if ( (RsDCEst_M1.flag_enableRSDCestimate && RsDCEst_M1.RSDCestimate_state != RSDC_EST_COMPLETED) &&
               (pFOCVars->controlMode == MCM_SPEED_MODE || pFOCVars->controlMode == MCM_TORQUE_MODE) &&
               (pFOCVars->controlMode_prev != pFOCVars->controlMode) )
</#if>
          {
              TSK_PulsesRsDC(RSDC_ONLY);
          }
          else if ( (MC_PolPulse_M1.flagPolePulseActivation) && (POLPULSE_getState(&MC_PolPulse_M1.PolpulseObj) != POLPULSE_STATE_PostcalcsComplete) &&
                    (pFOCVars->controlMode == MCM_SPEED_MODE || pFOCVars->controlMode == MCM_TORQUE_MODE) &&
                    (pFOCVars->controlMode_prev != pFOCVars->controlMode) )
          {
              TSK_PulsesRsDC(POL_PULSE_ONLY);
          }
          else
          {
            if (pFOCVars->controlMode_prev != pFOCVars->controlMode)
            {
              pFOCVars->controlMode_prev = pFOCVars->controlMode;
<#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") >
              
              if (false == PWMC_IsPwmEnabled(pwmcHandle[M1]))
              {  
                /* deactivate driver protection input while STDRIVE101 exiting standby mode */
                LL_TIM_Disable${brkNb}(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
              }
</#if>
              __disable_irq();  // << Start of critical zone: write global variables read by HighFrequencyTask
          
              // During RUN we can switch MODE which may enable/disable PWM etc
              switch (pFOCVars->controlMode)
              {
                case MCM_OBSERVING_MODE:
                  /* PWM output is off, HSO is active */
                  STC_M1.SpeedControlEnabled = false;
                  CurrCtrl_M1.currentControlEnabled = true; // NOTE PWM off, current control tracking the duty
                  SPD_M1.closedLoopAngleEnabled = true;
                  SPD_M1.flagHsoAngleEqualsOpenLoopAngle = false;
<#if MC.M1_SPEED_SENSOR == "ZEST">            
                  ZEST_setInjectMode(&ZeST_M1,ZEST_INJECTMODE_Auto );
</#if>              
                  CurrCtrl_M1.forceZeroPwm = false;
                  POLPULSE_resetState(&MC_PolPulse_M1.PolpulseObj);
                  // Disable PWM while in Observing mode
                  if (PWMC_IsPwmEnabled(pwmcHandle[M1]))
                  {
                    PWMC_SwitchOffPWM(pwmcHandle[M1]);
                  }
                  break;
          
                case MCM_SPEED_MODE:
                  /* Closed loop angle and speed PID active */
                  STC_M1.SpeedControlEnabled = true;
                  CurrCtrl_M1.currentControlEnabled = true;
                  SPD_M1.closedLoopAngleEnabled = true;
                  SPD_M1.flagHsoAngleEqualsOpenLoopAngle = false;
                  CurrCtrl_M1.forceZeroPwm = false;
                  PIDREG_SPEED_setUi_pu(&pSTC[M1]->PIDSpeed, FIXP30(0.0f));
<#if MC.M1_SPEED_SENSOR == "ZEST">              
                  ZEST_setInjectMode(&ZeST_M1,ZEST_INJECTMODE_Auto );
</#if>              
                  // Enable PWM in these modes
                  PWMC_SwitchOnPWM(pwmcHandle[M1]);
                  HSO_setFlag_EnableOffsetUpdate(&HSO_M1, true);
                  break;
          
                case MCM_TORQUE_MODE:
                  /* Closed loop angle with speed PID disabled */
                  STC_M1.SpeedControlEnabled = false;
                  CurrCtrl_M1.currentControlEnabled = true;
                  SPD_M1.closedLoopAngleEnabled = true;
                  SPD_M1.flagHsoAngleEqualsOpenLoopAngle = false;
                  CurrCtrl_M1.forceZeroPwm = false;
<#if MC.M1_SPEED_SENSOR == "ZEST">            
                  ZEST_setInjectMode(&ZeST_M1,ZEST_INJECTMODE_Auto );
</#if>              
                  // Enable PWM in these modes
                  PWMC_SwitchOnPWM(pwmcHandle[M1]);
                  HSO_setFlag_EnableOffsetUpdate(&HSO_M1, true);
                  break;
          
                case MCM_OPEN_LOOP_CURRENT_MODE:
                  /* Open loop angle, with current controllers active */
                  STC_M1.SpeedControlEnabled = false;
                  CurrCtrl_M1.currentControlEnabled = true;
                  SPD_M1.closedLoopAngleEnabled = false;
<#if MC.MOTOR_SIM ==true>
                  /* Disabled to allow HSO tracking in open loop mode */
                  SPD_M1.flagHsoAngleEqualsOpenLoopAngle = false;
<#else>
                  SPD_M1.flagHsoAngleEqualsOpenLoopAngle = true;
</#if>
                  CurrCtrl_M1.forceZeroPwm = false;
<#if MC.M1_SPEED_SENSOR == "ZEST">     
<#if MC.MOTOR_SIM ==true>
                  /* Disabled to allow injection to fade out when motor is moving */
                  ZEST_setInjectMode(&ZeST_M1,ZEST_INJECTMODE_Auto );
<#else>
                  ZEST_setInjectMode(&ZeST_M1,ZEST_INJECTMODE_Always );
</#if>
</#if>
                  // Enable PWM in these modes
                  PWMC_SwitchOnPWM(pwmcHandle[M1]);
                  HSO_setFlag_EnableOffsetUpdate(&HSO_M1, true);
                  break;
          
                case MCM_OPEN_LOOP_VOLTAGE_MODE:
                  /* Open loop angle, using duty reference Ddq_ref_pu */
                  STC_M1.SpeedControlEnabled = false;
                  CurrCtrl_M1.currentControlEnabled = false;
                  SPD_M1.closedLoopAngleEnabled = false;
<#if MC.MOTOR_SIM ==true>
                  /* Disabled to allow HSO tracking in open loop mode */
                  SPD_M1.flagHsoAngleEqualsOpenLoopAngle = false;
<#else>
                  SPD_M1.flagHsoAngleEqualsOpenLoopAngle = true;
</#if>
                  CurrCtrl_M1.forceZeroPwm = false;
<#if MC.M1_SPEED_SENSOR == "ZEST">
                  ZEST_setInjectMode(&ZeST_M1,ZEST_INJECTMODE_Auto );
</#if>
                  // Enable PWM in these modes
                  PWMC_SwitchOnPWM(pwmcHandle[M1]);
                  HSO_setFlag_EnableOffsetUpdate(&HSO_M1, true);
                  break;

<#if (MC.MOTOR_PROFILER == true)>   
                case MCM_PROFILING_MODE:
                  /* HSO modes depend on Profiling state */
                  MCI_SetMaxCurrent(&Mci[M1],FIXP16(0.8f*M1_MAX_READABLE_CURRENT));
                  STC_M1.SpeedControlEnabled = false;
                  CurrCtrl_M1.currentControlEnabled = false;
                  SPD_M1.closedLoopAngleEnabled = false;
                  CurrCtrl_M1.forceZeroPwm = true;
                  // Enable PWM in these modes
                  PWMC_SwitchOnPWM(pwmcHandle[M1]);
                  HSO_setFlag_EnableOffsetUpdate(&HSO_M1, true);
                  break;
</#if>       
                case MCM_SHORTED_MODE:
                  /* PWM output is on, HSO is active, duty is being forced the same on all phases, shorting the motor */
                  STC_M1.SpeedControlEnabled = false;
                  CurrCtrl_M1.currentControlEnabled = true;
                  SPD_M1.closedLoopAngleEnabled = true;
                  SPD_M1.flagHsoAngleEqualsOpenLoopAngle = false;
                  CurrCtrl_M1.forceZeroPwm = false;
<#if MC.M1_SPEED_SENSOR == "ZEST">
                  ZEST_setInjectMode(&ZeST_M1,ZEST_INJECTMODE_Auto );
</#if>
                  POLPULSE_resetState(&MC_PolPulse_M1.PolpulseObj);
                  // Short PWM while in Shorted mode
                  PWMC_PWMShort(pwmcHandle[M1]);
                  break;
          
                default:
                  /* No action */
                  break;
              }
              __enable_irq();  // >> End of critical zone
            }
          }
        }
        /* USER CODE BEGIN MediumFrequencyTask M1 3 */
        
        /* USER CODE END MediumFrequencyTask M1 3 */
        break;
      }
        
      case STOP:
      {
        if (TSK_StopPermanencyTimeHasElapsedM1())
        {  
          Mci[M1].DirectCommand = MCI_NO_COMMAND;
          Mci[M1].State = IDLE;        
          PWMC_SwitchOffPWM(pwmcHandle[M1]);
          HSO_setFlag_EnableOffsetUpdate(&HSO_M1, false);
        }
        else
        {
          /* nothing to do, FW waits for to stop */
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
          /* nothing to do, FW stays in FAULT_OVER state until acknowledgement */
        }
        break;
      }

     
      case FAULT_NOW:
      {
        Mci[M1].State = FAULT_OVER;
        break;
      } 

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

  FOC_ReferenceSelection(M1); 
  
  /* PolPulse background handling */
  {
	  fixp_t oneOverUdc_pu = pCurrCtrlhandle[M1]->busVoltageComp;
	  POLPULSE_runBackground(&MC_PolPulse_M1.PolpulseObj, oneOverUdc_pu, VBus_M1.Udcbus_in_pu);
  }

  <#if MC.MOTOR_SIM == true>MC_MotorSim_Debug(&MotorSim);</#if>

  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */

<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_BG_Perf_Measure_Stop(&PerfTraces, MEASURE_TSK_MediumFrequencyTaskM1);
</#if>

}

/**
 * @brief  Clears variables for the motor identified by @p bMotor. 
 *         It must be called before each motor restart.
 */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */
  
  /* Initialize additional FOCVars members */
  FOCVars_t* pFOCVars = &FOCVars[bMotor];
  pFOCVars->controlMode_prev = MCM_OBSERVING_MODE;
  pSTC[bMotor]->SpeedControlEnabled = false;
  pCurrCtrlhandle[bMotor]->currentControlEnabled = true;
  pCurrCtrlhandle[bMotor]->forceZeroPwm = false;  
  pSPD[bMotor]->closedLoopAngleEnabled = true;
  pSPD[bMotor]->flagHsoAngleEqualsOpenLoopAngle = false;
  RsDCEst_M1.RSDCestimate_state = RSDC_EST_RESET;
  
  PIDREG_SPEED_setUi_pu(&pSTC[bMotor]->PIDSpeed, FIXP30(0.0f));
  PIDREGDQX_CURRENT_setUiD_pu(&pCurrCtrlhandle[bMotor]->pid_IdIqX_obj, FIXP30(0.0f));
  PIDREGDQX_CURRENT_setUiQ_pu(&pCurrCtrlhandle[bMotor]->pid_IdIqX_obj, FIXP30(0.0f));
<#if MC.M1_SPEED_SENSOR == "ZEST">
  ZEST_setInjectMode(pZeST[bMotor],ZEST_INJECTMODE_Auto );
</#if>
  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  POLPULSE_stopPulse(&MC_PolPulse_M1.PolpulseObj);

  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Initializes @p hTickCount counter for charge boot capacitors of motor 1
  */
__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
  hBootCapDelayCounterM1 = hTickCount;
}

/**
 * @brief  Monitors the motor1 counter for charge boot
 *         capacitors.
 * @retval bool true if time has elapsed, false otherwise
 */
__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

/**
 * @brief  Initalizes the @p hTickCount counter used for counting the permanency
 *         time in STOP state for motor 1
 */
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
 * @brief  Monitors the counter of the permanency time in STOP state
 *         for motor 1
 * @retval bool true if time is elapsed, false otherwise
 */
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM1 == 0)
  {
    retVal = true;
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
/**
 * @brief  Executes High frequency task.
 *
 *  This is mainly the FOC current control loop. 
 *
 * @retval Motor ID.
 */
__weak uint8_t TSK_HighFrequencyTask(void)
{
  uint16_t hFOCreturn;

<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Measure_Start(&PerfTraces, MEASURE_TSK_HighFrequencyTaskM1);
</#if>

  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */

  /* Simplified controller */
  if (bMCBootCompleted != 0)
  {
    hFOCreturn = FOC_CurrControllerM1();

    if(hFOCreturn == MC_DURATION)
    {
      MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
    }

    TSK_SafetyTask();

  }

  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */

<#if MC.DBG_MCU_LOAD_MEASURE == true>
  MC_Perf_Measure_Stop(&PerfTraces, MEASURE_TSK_HighFrequencyTaskM1);
</#if>

  return M1;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
 * @brief Executes the core of FOC  that is the controllers for Iqd
 *        currents regulation. Reference frame transformations are carried out
 *        accordingly to the active speed sensor. 
 * @retval uint16_t: It returns MC_NO_FAULTS if the FOC has been ended before
 *         next PWM Update event, MC_DURATION otherwise
 */
inline uint16_t FOC_CurrControllerM1(void)
{

  uint16_t hCodeError = 0;
  Measurement_Output_t measuredVal;
  Sensorless_Input_t SensorlessInp;
  Sensorless_Output_t SensorlessOut;
  CurrCtrl_Input_t   CurrCtrlInp;
  STC_input_t STCInp;
  Duty_Dab_t dutyAB;
  fixp30_t angle_park_pu; 
  bool pwmControlEnable;

  pwmControlEnable = R3_IsInPwmControl(pwmcHandle[M1]);

<#if MC.M1_BUS_VOLTAGE_READING>    
  <#if (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_U) || (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_V) || (MC.M1_VBUS_ADC ==  MC.M1_CS_ADC_W)>
  R3_GetVbusMeasurements(pwmcHandle[M1], &VBus_M1.Udcbus_in_pu);
  </#if>
</#if>
  /* High level motor control */
  SPD_GetSpeed(&SPD_M1,
               &STCInp.emfSpeed, 
               &STCInp.SpeedLP);  
  STCInp.pwmControlEnable = pwmControlEnable;
  STCInp.Udcbus_in_pu = VBus_M1.Udcbus_in_pu;
  STCInp.closedLoopEnabled = SPD_M1.closedLoopAngleEnabled;
  STC_Run(&STC_M1, &STCInp);

  if (false == RsDCEst_M1.flag_enableRSDCestimate || RSDC_EST_ONGOING != RsDCEst_M1.RSDCestimate_state)
  {
    /* always update open loop angle unless RSDC estimation is on going */
    SPD_UpdtOpenLoopAngle(&SPD_M1, STC_M1.speed_ref_ramped_pu);
  } 

<#if MC.MOTOR_SIM == true> 
  /* - Retrieve phase current and voltage from motor simulation,
     - outputs currents and voltages expressed in alphaBeta coordinates */
  MotorSim.pwmControlEnable = pwmControlEnable;
  MotorSim.Udc_pu = VBus_M1.Udcbus_in_pu;
  MotorSim.LoadTorque = FIXP30(0.0f);	                 	// Todo: apply simulated load
  MC_MotorSim_Measurements(&MotorSim, &measuredVal);
<#else>
  /* - Retrieve measurements from hardware, 
     - perform Clarke transformation
     - outputs currents and voltages expressed in alphaBeta coordinates */
  PWMC_GetMeasurements(&PWM_Handle_M1._Super, &measuredVal);
</#if>

  MC_POLPULSE_run(&MC_PolPulse_M1, 0); // angle optional and currently not used in PolPulse
  
  /* filters  alpha/beta currents and voltages: used by Rs DC estimator 
     can be removed if Rs DC estimation is not used */  
  RsDC_estimationFiltering(&RsDCEst_M1, &measuredVal.Iab_pu, &measuredVal.Uab_pu);
 
   /* Sensorless position/speed estimator */

  SensorlessInp.Iab_pu = measuredVal.Iab_pu;
  SensorlessInp.Uab_pu = measuredVal.Uab_pu;
  SPD_Run(&SPD_M1, &SensorlessInp, &SensorlessOut);


  /* FOC current control 
     Prepare park and pwm angles for current control 
     get angle from sensorless Use closed loop estimated angle or open loop angle */
  SPD_GetAngle(&SPD_M1, &angle_park_pu);

  /* FOC current control */

  /* get sensorless speed */
  SPD_GetSpeed(&SPD_M1,
               &STCInp.emfSpeed,       /* dummy */
               &CurrCtrlInp.SpeedLP);
    
  /* Calculate current references */
  CurrCtrlInp.Idq_ref_currentcontrol_pu = STC_M1.Idq_ref_limited_pu;
<#if MC.M1_SPEED_SENSOR == "ZEST">
  if (true == SPD_M1.zestControl.enableInjection) 
  {
    CurrCtrlInp.Idq_ref_currentcontrol_pu.D += FIXP24_mpy(SensorlessOut.Idq_inject_pu.D, STC_M1.K_InjectBoost);
    CurrCtrlInp.Idq_ref_currentcontrol_pu.Q += FIXP24_mpy(SensorlessOut.Idq_inject_pu.Q, STC_M1.K_InjectBoost);
  }
</#if>  
  CurrCtrlInp.Udcbus_in_pu = VBus_M1.Udcbus_in_pu;
  CurrCtrlInp.Iab_in_pu = measuredVal.Iab_pu;	  
  CurrCtrlInp.Uab_emf_pu = SensorlessOut.Uab_emf_pu;
  CurrCtrlInp.angle = angle_park_pu;
  CurrCtrlInp.pwmControlEnable = pwmControlEnable;
  MC_currentController(&CurrCtrl_M1, 
                       &CurrCtrlInp, 
                       &dutyAB);

  /* generates PWM duty cycles from alpha beta duty values  */
  hCodeError = PWMC_SetPhaseDuty(&PWM_Handle_M1._Super, &dutyAB);

<#if MC.MOTOR_PROFILER == true>

  /* Udq park transform (profiler and trace) */
  FIXP_CosSin_t cossin_park; 
  FIXP30_CosSinPU(angle_park_pu, &cossin_park);
  CurrCtrl_M1.Udq_in_pu = MCM_Park_Voltage(measuredVal.Uab_pu, &cossin_park);
  
  /* Profiler */
  PROFILER_run(&profiler_M1, &profilerMotor_M1);
</#if>

  /* Data logging */
<#if MC.MCP_EN == true>
  GLOBAL_TIMESTAMP++;
  if (MCPA_UART_A.Mark != 0)
  {
    MCPA_dataLog (&MCPA_UART_A);
  }
<#if MC.DEBUG_DAC_FUNCTIONALITY_EN == true>

  DAC_Exec(&DAC_Handle);

</#if>
</#if>

  return hCodeError;
}

/**
 * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances.
 *
 * Faults flags are updated here.
 */
__weak void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (bMCBootCompleted == 1)
  {  
    TSK_SafetyTask_PWMOFF(M1);
    /* User conversion execution */
    /* USER CODE BEGIN TSK_SafetyTask 1 */

    /* USER CODE END TSK_SafetyTask 1 */
  }
}

/**
 * @brief  Safety task implementation, of the motor identified by @p bMotor, if  MC.M1_ON_OVER_VOLTAGE == TURN_OFF_PWM
 *         
 */
__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */

  uint16_t CodeReturn = MC_NO_FAULTS;

  CodeReturn |= PWMC_IsFaultOccurred(pwmcHandle[bMotor]);                    /* check for fault. It return MC_OVER_CURR or MC_NO_FAULTS 
                                                                                 (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  
<#if MC.M1_BUS_VOLTAGE_READING>    
  <#if (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_U) && (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_V) && (MC.M1_VBUS_ADC !=  MC.M1_CS_ADC_W)> 
  uint16_t rawValueVbusM1 = RCM_ExecRegularConv(&VbusRegConv_M1);
  VBus_M1.Udcbus_in_pu = FIXP16_mpy(VBus_M1.Udcbus_convFactor, rawValueVbusM1);
  </#if>
</#if>
<#if (MC.M1_TEMPERATURE_READING == true  && MC.M1_OV_TEMPERATURE_PROT_ENABLING == true)>
  <#if (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_U) && (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_V) && (MC.M1_TEMP_FDBK_ADC !=  MC.M1_CS_ADC_W)>
  uint16_t rawValueTempM1 = RCM_ExecRegularConv(&TempRegConv_M1);
  <#else>
  fixp30_t rawValueTempM1;
  OVERSAMPLING_getTempMeasurements(&rawValueTempM1);
  </#if>
  CodeReturn |= NTC_CalcAvTemp(&TempSensor_M1, (uint16_t)rawValueTempM1);
</#if>
      
  CodeReturn |= ErrorDetection(bMotor);

  MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
  
  <#if (MC.M1_ICL_ENABLED == true)>
  if ((M1 == bMotor) && (MC_UNDER_VOLT == (CodeReturn & MC_UNDER_VOLT)) && ICLFaultTreatedM1){
    ICLFaultTreatedM1 = false; 
  }
  </#if>
  
  if (MCI_GetFaultState(&Mci[bMotor]) != (uint32_t)MC_NO_FAULTS) /* Acts on PWM outputs in case of faults */
  {
      PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
      FOC_Clear(bMotor);
      /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

      /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
  }
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}

/**
 * @brief  Puts the Motor Control subsystem in safety conditions on a Hard Fault
 *
 *  This function is to be executed when a general hardware failure has been detected
 * by the microcontroller and is used to put the system in safety condition.
 */
__weak void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */

  R3_SwitchOffPWM(pwmcHandle[M1]);
  MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0);
  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}

/**
 * @brief  Executes startup procedudure using combine or single PolPules, RsDC measure
 */
void TSK_PulsesRsDC(StartMode_t Mode)
{
  FOCVars_t* pFOCVars = &FOCVars[M1];

  switch (Mode)
  {
<#if MC.M1_SPEED_SENSOR == "ZEST">
    case FAST_RSDC:
      if (POLPULSE_getState(&MC_PolPulse_M1.PolpulseObj) == POLPULSE_STATE_Idle)
      {
        __disable_irq();  // << Start of critical zone: write global variables read by HighFrequencyTask
<#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") >
             /* deactivate driver protection input while STDRIVE101 exiting standby mode */
             LL_TIM_Disable${brkNb}(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
</#if>
        /* Onpen loop, with current controllers disabled during pulses sequence */
        STC_M1.SpeedControlEnabled = false;
        CurrCtrl_M1.currentControlEnabled = false;
        SPD_M1.closedLoopAngleEnabled = false;
        SPD_M1.flagHsoAngleEqualsOpenLoopAngle = false;
        CurrCtrl_M1.forceZeroPwm = false;
               
        ZEST_setInjectMode(&ZeST_M1,ZEST_INJECTMODE_None );
	
        POLPULSE_trigger(&MC_PolPulse_M1.PolpulseObj);
        // Enable PWM in these modes
        PWMC_SwitchOnPWM(pwmcHandle[M1]);
        HSO_setFlag_EnableOffsetUpdate(&HSO_M1, true);

        __enable_irq();  // >> End of critical zone
      }
      else if ( (POLPULSE_getState(&MC_PolPulse_M1.PolpulseObj) == POLPULSE_STATE_PostcalcsComplete) && 
                (RsDCEst_M1.RSDCestimate_state != RSDC_EST_COMPLETED) )
      {       
        if (RSDC_EST_RESET == RsDCEst_M1.RSDCestimate_state)
        {
          RsDC_estimationInit(pFOCVars, &RsDCEst_M1);
        }
        else if (RSDC_EST_ONGOING == RsDCEst_M1.RSDCestimate_state)
        {
          RsDC_estimationRun(pFOCVars, &RsDCEst_M1);
        }
        else
        {
          /* nothing to do, Rs DC estimation complete */
        }
      }
      break;
</#if>      
    case RSDC_ONLY:
        if (RSDC_EST_RESET == RsDCEst_M1.RSDCestimate_state)
        {
<#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") >
             /* deactivate driver protection input while STDRIVE101 exiting standby mode */
             LL_TIM_Disable${brkNb}(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
</#if>          
          RsDC_estimationInit(pFOCVars, &RsDCEst_M1);
        }
        else if (RSDC_EST_ONGOING == RsDCEst_M1.RSDCestimate_state)
        {
          RsDC_estimationRun(pFOCVars, &RsDCEst_M1);
        }
        else
        {
          /* nothing to do, Rs DC estimation complete */
        }
      break;
    case POL_PULSE_ONLY:
      if (POLPULSE_getState(&MC_PolPulse_M1.PolpulseObj) == POLPULSE_STATE_Idle)
      {
        __disable_irq();  // << Start of critical zone: write global variables read by HighFrequencyTask
<#if (MC.M1_PWM_DRIVER_PN == "STDRIVE101") && (MC.M1_DP_TOPOLOGY != "NONE") >
             /* deactivate driver protection input while STDRIVE101 exiting standby mode */
             LL_TIM_Disable${brkNb}(${_last_word(MC.M1_PWM_TIMER_SELECTION)});
</#if>
        /* Onpen loop, with current controllers disabled during pulses sequence */
        STC_M1.SpeedControlEnabled = false;
        CurrCtrl_M1.currentControlEnabled = false;
        SPD_M1.closedLoopAngleEnabled = false;
        SPD_M1.flagHsoAngleEqualsOpenLoopAngle = false;
        CurrCtrl_M1.forceZeroPwm = false;
<#if MC.M1_SPEED_SENSOR == "ZEST">                 
        ZEST_setInjectMode(&ZeST_M1,ZEST_INJECTMODE_None );
</#if>	
        POLPULSE_trigger(&MC_PolPulse_M1.PolpulseObj);
        // Enable PWM in these modes
        PWMC_SwitchOnPWM(pwmcHandle[M1]);
        HSO_setFlag_EnableOffsetUpdate(&HSO_M1, true);

        __enable_irq();  // >> End of critical zone
      }
      break;
      default:
        break;
  }
}

/**
 * @brief  Initializes miscellaneous montrol control core variables.
 *
 * Initializes mainly FOCVars data structure.
 */
void MC_init(void)
{
<#if CondFamily_STM32G4 == false>  
  FIXPMATH_init();
</#if>

  FOCVars_t* pFOCVars = &FOCVars[M1];

  /* Initialize Calibration status */
  pFOCVars->PolarizationState = NOT_DONE;
  pFOCVars->flagStartWithCalibration = false;

  /* Pre-calculated values */
  fixp24_t ls_henry_fixp24 = FIXP24(motorParams->ls);
  pFOCVars->Ls_Rated_H = ls_henry_fixp24;
  pFOCVars->Ls_Active_H = ls_henry_fixp24;
  
  float_t fs_l = (scaleParams->voltage / scaleParams->current / (float_t)VOLTAGE_FILTER_POLE_RPS);
  float_t ls_pu_flt = motorParams->ls / fs_l;
  FIXPSCALED_floatToFIXPscaled(ls_pu_flt, &pFOCVars->Ls_Rated_pu_fps);
  FIXPSCALED_floatToFIXPscaled(ls_pu_flt, &pFOCVars->Ls_Active_pu_fps);

  pFOCVars->Rs_Rated_Ohm = FIXP20(motorParams->rs);
  
  pFOCVars->Flux_Rated_VpHz = FIXP24(motorParams->ratedFlux);

  /* References */
  pCurrCtrlhandle[M1]->Ddq_ref_pu.D = 0;
  pCurrCtrlhandle[M1]->Ddq_ref_pu.Q = 0;

  pFOCVars->controlMode = DEFAULT_CONTROL_MODE;
  pFOCVars->controlMode_prev = MCM_OBSERVING_MODE;
  pFOCVars->polePairs = motorParams->polePairs; 
  pFOCVars->Kt = (1.0f / (1.5f * (double)motorParams->polePairs * ((double)motorParams->ratedFlux / M_TWOPI))); /* Pre-computation of cst Kt in Ampere/Nm */

<#if MC.MOTOR_SIM == true>
  /* MotorSim Initialization */
  MC_MotorSim_init(&MotorSim);
</#if>

} /* end of MC_init() */

/**
 * @brief  Monitors fault error for motor identified by @p bMotor.
 *
 * This routines needs to be called at least in the safety task
 *
 * @retval uint16_t error status
 *  
 */
uint16_t ErrorDetection(uint8_t bMotor)
{
  uint16_t errorCode = 0;
  /* Software overcurrent limit tripped */
  if (RUN == pMCI[bMotor]->State)
  {
    fixp30_t overcurrent_trip_pu = pwmcHandle[bMotor]->softOvercurrentTripLevel_pu;
    if (	FIXP_abs(pwmcHandle[bMotor]->Irst_in_pu.R) > overcurrent_trip_pu ||
        FIXP_abs(pwmcHandle[bMotor]->Irst_in_pu.S) > overcurrent_trip_pu ||
        FIXP_abs(pwmcHandle[bMotor]->Irst_in_pu.T) > overcurrent_trip_pu)
    {
      errorCode |= MC_OVERCURR_SW;
    }
  }

  /* Software over/under voltage limit tripped */
  if (Mci[bMotor].pVBus->Udcbus_in_pu > Mci[bMotor].pVBus->Udcbus_overvoltage_limit_pu)
  {
    errorCode |= MC_OVER_VOLT;
  }
  else if (Mci[bMotor].pVBus->Udcbus_in_pu < Mci[bMotor].pVBus->Udcbus_undervoltage_limit_pu)
  {
    errorCode |= MC_UNDER_VOLT;
  }

  /* Current sampling fault, related to current reconstruction */
  if (pwmcHandle[bMotor]->currentsamplingfault)
  {
    errorCode |= MC_SAMPLEFAULT;
  }

  return errorCode;
}

/**
 * @brief  Selects the reference for speed or torque for the motor identified by @p bMotor.
 * 
 * The reference can be define by APIs or throttle/potentiometer position 
 *
 */
void FOC_ReferenceSelection(uint8_t bMotor)
{
<#if MC.M1_POTENTIOMETER_ENABLE == true>
  /* Read throttle position *************************************************************************/
  PotentiometerGetPosition(pPotentiometer[bMotor]);
</#if>
  /* Q-current reference source *********************************************************************/

  /* Speed source selection */
<#if MC.M1_POTENTIOMETER_ENABLE == true>
  if (pSTC[bMotor]->speedref_source == FOC_SPEED_SOURCE_Throttle)
  {
    pSTC[bMotor]->speed_ref_active_pu = FIXP30_mpy(potentiometer_M1.position, potentiometer_M1.speedref_scalefactor);
  }
  else
  {
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true -->
    /* Reference from user via MCI */
    pSTC[bMotor]->speed_ref_active_pu = pSTC[bMotor]->speed_ref_pu;
<#if MC.M1_POTENTIOMETER_ENABLE == true>
  }
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true -->

  /* Speed reference source *************************************************************************/

  /* For torque/current mode, select the reference based on speedref_source */
<#if MC.M1_POTENTIOMETER_ENABLE == true>
  if (pSTC[bMotor]->speedref_source == FOC_SPEED_SOURCE_Throttle)
  {
    /* Reference from throttle input */
    pSTC[bMotor]->Iq_ref_active_pu = FIXP30_mpy(potentiometer_M1.position, pSTC[bMotor]->I_max_pu);
  }
  else
  {
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true -->
    /* Reference from user via MCI */
    pSTC[bMotor]->Iq_ref_active_pu = pSTC[bMotor]->Idq_ref_pu.Q;
<#if MC.M1_POTENTIOMETER_ENABLE == true>
  }
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true -->
}

<#if MC.START_STOP_BTN == true>
__weak void UI_HandleStartStopButton_cb (void)
{
/* USER CODE BEGIN START_STOP_BTN */
  if (IDLE == MC_GetSTMStateMotor1())
  {
    /* Ramp parameters should be tuned for the actual motor */
    (void)MC_StartWithPolarizationMotor1();
  }
  else
  {
    (void)MC_StopMotor1();
  }
/* USER CODE END START_STOP_BTN */
}
</#if><#-- MC.START_STOP_BTN == true -->

#ifdef EEPROM_EMULATION

void GetFloatParamFromEEPROM(int EEPROM_Addr,float *param)
{
  uint32_t val;
  EE_Status ee_status = EE_OK;

  ee_status = EE_ReadVariable32bits(EEPROM_Addr,&val);
  //if EEPROM value present get it
  if(ee_status == EE_OK)
  {
    CONVERT_u convert;
    convert.u32 = val;
    *param = convert.flt;
  }
  //if EEPROM value not present create it with default value
  else
  {
    CONVERT_u convert;
    convert.flt = *param;
    ee_status = EE_WriteVariable32bits(EEPROM_Addr,convert.u32);
  }
} /* end of GetFloatParamFromEEPROM() function */

void GetUint32ParamFromEEPROM(int EEPROM_Addr,uint32_t *param)
{
  uint32_t val;
  EE_Status ee_status = EE_OK;

  ee_status = EE_ReadVariable32bits(EEPROM_Addr,&val);
  //if EEPROM value present get it
  if(ee_status == EE_OK)
  {

    *param = val;
  }
  //if EEPROM value not present create it with default value
  else
  {
    ee_status = EE_WriteVariable32bits(EEPROM_Addr,*param);
  }
} /* end of GetUint32ParamFromEEPROM() function */

#endif /* EEPROM_EMULATION */

 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration 
  */
__weak void mc_lock_pins (void)
{
<#list configs as dt>
<#list dt.peripheralGPIOParams.values() as io>
<#list io.values() as ipIo>
<#list ipIo.entrySet() as e>
<#if (e.getKey().equals("GPIO_Label")) && (e.getValue()?matches("^M[0-9]+_.*$"))>
LL_GPIO_LockPin(${e.getValue()}_GPIO_Port, ${e.getValue()}_Pin);
</#if>
</#list>
</#list>
</#list>
</#list>
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
