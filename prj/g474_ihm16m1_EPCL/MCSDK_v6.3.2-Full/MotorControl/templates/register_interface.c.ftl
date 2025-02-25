<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
/**
  ******************************************************************************
  * @file    register_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the register access for the MCP protocol
  *
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

#include "mc_type.h"
#include "string.h"
#include "register_interface.h"
#include "mc_config.h"
<#if MC.MCP_EN>
#include "mcp.h"
#include "mcp_config.h"
  <#if MC.MCP_ASYNC_EN>
#include "mcpa.h"
  </#if>
</#if>
<#if MC.DEBUG_DAC_FUNCTIONALITY_EN>
#include "dac_ui.h"
</#if>
<#if MC.MOTOR_PROFILER == true>
#include "mp_one_touch_tuning.h"
#include "mp_self_com_ctrl.h"
</#if>
#include "mc_configuration_registers.h"
<#if SIX_STEP >
  <#if MC.M1_DBG_OPEN_LOOP_ENABLE == true>
#include "mc_tasks.h"
  </#if> <#-- MC.M1_DBG_OPEN_LOOP_ENABLE == true-->
</#if> <#-- SIX_STEP -->

<#if M1_IS_SENSORLESS || (MC.M1_SPEED_SENSOR == "SENSORLESS_ADC") || (MC.M1_SPEED_SENSOR == "SENSORLESS_COMP")>
</#if><#-- M1_IS_SENSORLESS || MC.SPEED_SENSOR == "SENSORLESS_ADC" -->

uint8_t RI_SetRegisterGlobal(uint16_t regID, uint8_t typeID, uint8_t *data, uint16_t *size, int16_t dataAvailable)
{
  uint8_t retVal = MCP_CMD_OK;
  switch(typeID)
  {
    case TYPE_DATA_8BIT:
    {
      switch (regID)
      {
        case MC_REG_STATUS:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

<#if MC.PFC_ENABLED == true>
        case MC_REG_PFC_STATUS:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_PFC_ENABLED:
          break;

</#if><#-- MC.PFC_ENABLED == true -->

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 1;
      break;
    }

    case TYPE_DATA_16BIT:
    {
<#if (MEMORY_FOOTPRINT_REG || MEMORY_FOOTPRINT_REG2)>
  <#if MC.DEBUG_DAC_FUNCTIONALITY_EN>
      uint16_t regdata16 = *(uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
  </#if><#-- MC.DEBUG_DAC_FUNCTIONALITY_EN -->
</#if><#-- MEMORY_FOOTPRINT_REG || MEMORY_FOOTPRINT_REG2 -->
      switch (regID) 
      {

        case MC_REG_BUS_VOLTAGE:
        case MC_REG_HEATS_TEMP:
        case MC_REG_MOTOR_POWER:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
          
<#if MEMORY_FOOTPRINT_REG2>
  <#if MC.DEBUG_DAC_FUNCTIONALITY_EN>
        case MC_REG_DAC_OUT1:
        {
          DAC_SetChannelConfig(&DAC_Handle , DAC_CH1, regdata16);
          break;
        }
          
        case MC_REG_DAC_OUT2:
        {
          DAC_SetChannelConfig(&DAC_Handle , DAC_CH2, regdata16);
          break;
        }
  </#if><#-- MC.DEBUG_DAC_FUNCTIONALITY_EN -->
</#if><#-- MEMORY_FOOTPRINT_REG2 -->

<#if MEMORY_FOOTPRINT_REG2>
        case MC_REG_DAC_USER1:
        case MC_REG_DAC_USER2:
        break;
</#if><#-- MEMORY_FOOTPRINT_REG2 -->

<#if MC.PFC_ENABLED>
        case MC_REG_PFC_DCBUS_REF:
        case MC_REG_PFC_DCBUS_MEAS:
        case MC_REG_PFC_ACBUS_FREQ:
        case MC_REG_PFC_ACBUS_RMS:
  <#if MEMORY_FOOTPRINT_REG>
        case MC_REG_PFC_I_KP:
        case MC_REG_PFC_I_KI:
        case MC_REG_PFC_I_KD:
  </#if><#-- MEMORY_FOOTPRINT_REG -->
        case MC_REG_PFC_V_KP:
        case MC_REG_PFC_V_KI:
        case MC_REG_PFC_V_KD:
        case MC_REG_PFC_STARTUP_DURATION:
        case MC_REG_PFC_I_KP_DIV:
        case MC_REG_PFC_I_KI_DIV:
        case MC_REG_PFC_I_KD_DIV:
        case MC_REG_PFC_V_KP_DIV:
        case MC_REG_PFC_V_KI_DIV:
        case MC_REG_PFC_V_KD_DIV:
        break;
</#if><#-- MC.PFC_ENABLED -->

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 2;
      break;
    }

    case TYPE_DATA_32BIT:
    {

      switch (regID)
      {
        case MC_REG_FAULTS_FLAGS:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

<#if MC.PFC_ENABLED>
        case MC_REG_PFC_FAULTS:
          break;
</#if><#-- MC.PFC_ENABLED -->

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 4;
      break;
    }

<#if MEMORY_FOOTPRINT_REG>
    case TYPE_DATA_STRING:
    {
      const char_t *charData = (const char_t *)data;
      char_t *dummy = (char_t *)data;
      retVal = MCP_ERROR_RO_REG;
      /* Used to compute String length stored in RXBUFF even if Reg does not exist */
      /* It allows to jump to the next command in the buffer */
      (void)RI_MovString(charData, dummy, size, dataAvailable);
      break;
    }
</#if><#-- MEMORY_FOOTPRINT_REG -->

    case TYPE_DATA_RAW:
    {
      uint16_t rawSize = *(uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
      /* The size consumed by the structure is the structure size + 2 bytes used to store the size */
      *size = rawSize + 2U;
      uint8_t *rawData = data; /* rawData points to the first data (after size extraction) */
      rawData++;
      rawData++;

      if (*size > (uint16_t)dataAvailable)
      { 
        /* The decoded size of the raw structure can not match with transmitted buffer, error in buffer
           construction */
        *size = 0;
        retVal = MCP_ERROR_BAD_RAW_FORMAT; /* This error stop the parsing of the CMD buffer */
      }
      else
      {
        switch (regID)
        {
          case MC_REG_APPLICATION_CONFIG:
<#if MEMORY_FOOTPRINT_REG>
          case MC_REG_MOTOR_CONFIG:
</#if><#-- MEMORY_FOOTPRINT_REG -->
          case MC_REG_GLOBAL_CONFIG:
          case MC_REG_FOCFW_CONFIG:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }

      }
      break;
    }

    default:
    {
      retVal = MCP_ERROR_BAD_DATA_TYPE;
      *size =0; /* From this point we are not able anymore to decode the RX buffer */
      break;
    }
  }
  return (retVal);
}

<#macro SetRegsFct MOTOR_NUM > 
uint8_t RI_SetRegisterMotor${MOTOR_NUM}(uint16_t regID, uint8_t typeID, uint8_t *data, uint16_t *size, int16_t dataAvailable)
{
  uint8_t retVal = MCP_CMD_OK;
  uint8_t motorID=${MOTOR_NUM - 1};
  MCI_Handle_t *pMCIN = &Mci[motorID];

  switch(typeID)
  {
    case TYPE_DATA_8BIT:
    {
      switch (regID)
      {
        case MC_REG_STATUS:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }
<#if (.vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || (.vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "ACIM")>
        case MC_REG_CONTROL_MODE:
        {
          uint8_t regdata8 = *data;

          if ((uint8_t)MCM_TORQUE_MODE == regdata8)
          {

            MCI_ExecTorqueRamp(pMCIN, MCI_GetTeref(pMCIN), 0);
          }
          else
          {
            /* Nothing to do */
          }

          if ((uint8_t)MCM_SPEED_MODE == regdata8)
          {
  <#if .vars.MC["M"+ MOTOR_NUM?c + "_DBG_OPEN_LOOP_ENABLE"] == false> 
            MCI_ExecSpeedRamp(pMCIN, MCI_GetMecSpeedRefUnit(pMCIN), 0);
  <#else><#-- DBG_OPEN_LOOP_ENABLE == true -->
            MCI_SetSpeedMode(pMCIN);
  </#if><#-- DBG_OPEN_LOOP_ENABLE == false -->
          }
          else
          {
            /* Nothing to do */
          }
<#if .vars.MC["M"+ MOTOR_NUM?c + "_DBG_OPEN_LOOP_ENABLE"] == true>

          if ((uint8_t)regdata8 == MCM_OPEN_LOOP_CURRENT_MODE)
          {
            MCI_SetOpenLoopCurrentMode(pMCIN);
          }
          else
          {
            /* Nothing to do */
          }
            
          if ((uint8_t)regdata8 == MCM_OPEN_LOOP_VOLTAGE_MODE)
          {
            MCI_SetOpenLoopVoltageMode(pMCIN);
          }
          else
          {
            /* Nothing to do */
          }
  </#if><#-- DBG_OPEN_LOOP_ENABLE == true-->
          break;
        }

</#if><#-- FOC or ACIM -->
<#if (.vars["M" + MOTOR_NUM?c + "_IS_SENSORLESS"] == true) || (.vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_ADC")>
        case MC_REG_RUC_STAGE_NBR:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
</#if><#-- IS_SENSORLESS -->
<#if MC.MOTOR_PROFILER == true>
        case MC_REG_SC_PP:
        { 
          uint8_t regdataU8 = *(uint8_t *)data;
  <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_PLL"]>
          SPD_SetElToMecRatio(&STO_PLL_M${MOTOR_NUM}._Super, regdataU8);
  </#if><#-- IS_STO_PLL -->
  <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_CORDIC"]>
          SPD_SetElToMecRatio(&STO_CR_M${MOTOR_NUM}._Super, regdataU8);
  </#if><#-- IS_STO_CORDIC -->
          SPD_SetElToMecRatio(&VirtualSpeedSensorM${MOTOR_NUM}._Super, regdataU8);
  <#if .vars.MC["M" + MOTOR_NUM?c + "_AUXILIARY_SPEED_SENSOR"] == "HALL_SENSOR">
          SPD_SetElToMecRatio(&HALL_M${MOTOR_NUM}._Super, regdataU8);
  </#if><#-- AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->
          SCC_SetPolesPairs(&SCC, regdataU8);
          OTT_SetPolesPairs(&OTT, regdataU8);
          break;
        }

        case MC_REG_SC_CHECK:
        case MC_REG_SC_STATE:
        case MC_REG_SC_STEPS:
        case MC_REG_SC_FOC_REP_RATE:
        case MC_REG_SC_COMPLETED:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

  <#if .vars.MC["M" + MOTOR_NUM?c + "_AUXILIARY_SPEED_SENSOR"] == "HALL_SENSOR">
        case MC_REG_HT_MECH_WANTED_DIRECTION:
        {
          uint8_t regdataU8 = *(uint8_t *)data;
          HT_SetMechanicalWantedDirection(&HT,regdataU8);
          break;
        }
  </#if><#-- AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->
  
          case MC_REG_PB_CHARACTERIZATION:
        {
          uint8_t regdataU8 = *(uint8_t *)data;
          SCC_SetPBCharacterization(&SCC,regdataU8);
          break;
        }
</#if><#-- MOTOR_PROFILER -->

<#if .vars.MC["M" + MOTOR_NUM?c + "_POSITION_CTRL_ENABLING"] == true>
  <#if MEMORY_FOOTPRINT_REG == true>
        case MC_REG_POSITION_CTRL_STATE:
        case MC_REG_POSITION_ALIGN_STATE:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
  </#if><#-- MEMORY_FOOTPRINT_REG -->
</#if><#-- POSITION_CTRL_ENABLING -->

<#if .vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "SIX_STEP">
        case MC_REG_LOWSIDE_MODULATION:
        {
          uint8_t regdataU8 = *(uint8_t *)data;
          PWMC_SetLSModConfig(&PWM_Handle_M1, regdataU8);  
          break;
        }

        case MC_REG_QUASI_SYNCH:
        {
          uint8_t regdata8 = *data;
          PWMC_SetQuasiSynchState(&PWM_Handle_M${MOTOR_NUM}, regdata8);
          break;
        }

  <#if .vars.MC["M" + MOTOR_NUM?c + "_DBG_OPEN_LOOP_ENABLE"] == true>
        case MC_REG_COMMUTATION_STEPBUFSIZE:
        {
          OLS_SetDutyCycleRefTableSize(pOLS[0], *data);
          break;
        }
        
        case MC_REG_OPENLOOP:
        {
          OLS_SetOpenLoopFlag(pOLS[0], *data);
          break;
        }
        
        case MC_REG_OPENLOOP_DC:
        {
          OLS_SetDutyCycleRef(pOLS[0],*data);
          break;
        }
        
        case MC_REG_OPENLOOP_REVUP:
        {
          OLS_SetRevUpFlag(pOLS[0], *data);
          break;
        }

        case MC_REG_OPENLOOP_VOLTFACTOR:
        {
          OLS_SetVoltageFactor(pOLS[0], *data);
          break;
        }
    <#if .vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_ADC">
        case MC_REG_OPENLOOP_SENSING:
        {
          OLS_SetOnSensing(pOLS[0], *data);
          break;
        }
    </#if><#-- SENSORLESS_ADC -->

        case MC_REG_CONTROL_MODE:
        {
          uint8_t regdata8 = *data;

          if ((uint8_t)MCM_SPEED_MODE == regdata8)
          {
            MCI_ExecSpeedRamp(pMCIN, MCI_GetMecSpeedRefUnit(pMCIN), 0);
          }
          else
          {
            /* Nothing to do */
          }
          break;
        }
  </#if><#-- DBG_OPEN_LOOP_ENABLE -->
</#if><#-- 6STEP == true -->

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 1;
      break;
    }

    case TYPE_DATA_16BIT:
    {
<#if (MEMORY_FOOTPRINT_REG || MEMORY_FOOTPRINT_REG2)>
      uint16_t regdata16 = *(uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
</#if><#-- MEMORY_FOOTPRINT_REG || MEMORY_FOOTPRINT_REG2 -->
      switch (regID) 
      {
<#if .vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "SIX_STEP">
  <#if .vars.MC["M" + MOTOR_NUM?c + "_DBG_OPEN_LOOP_ENABLE"] == true>
        case MC_REG_OPENLOOP_CURRFACTOR:
        {
          OLS_SetCurrentFactor(pOLS[0], regdata16);
          break;
        }
  </#if><#-- DBG_OPEN_LOOP_ENABLE -->
</#if><#-- DRIVE_TYPE == "SIX_STEP" -->
<#if MEMORY_FOOTPRINT_REG2 && ((.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] != "ACIM") || (MC.ACIM_CONFIG == "LSO_FOC"))>
        case MC_REG_SPEED_KP:
        {
          PID_SetKP(&PIDSpeedHandle_M${MOTOR_NUM}, (int16_t)regdata16);
          break;
        }

        case MC_REG_SPEED_KI:
        {
          PID_SetKI(&PIDSpeedHandle_M${MOTOR_NUM}, (int16_t)regdata16);
          break;
        }

        case MC_REG_SPEED_KD:
        {
          PID_SetKD(&PIDSpeedHandle_M${MOTOR_NUM}, (int16_t)regdata16);
          break;
        }
</#if><#-- MEMORY_FOOTPRINT_REG2 && ((MC.DRIVE_TYPE != "ACIM") || (MC.ACIM_CONFIG == "LSO_FOC")) -->

<#if (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "ACIM")>
  <#if MEMORY_FOOTPRINT_REG && ((.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] != "ACIM") || (MC.ACIM_CONFIG == "LSO_FOC"))>
        case MC_REG_I_Q_KP:
        {
          PID_SetKP(&PIDIqHandle_M${MOTOR_NUM}, (int16_t)regdata16);
          break;
        }

        case MC_REG_I_Q_KI:
        {
          PID_SetKI(&PIDIqHandle_M${MOTOR_NUM}, (int16_t)regdata16);
          break;
        }

        case MC_REG_I_Q_KD:
        {
          PID_SetKD(&PIDIqHandle_M${MOTOR_NUM}, (int16_t)regdata16);
          break;
        }

        case MC_REG_I_D_KP:
        {
          PID_SetKP(&PIDIdHandle_M${MOTOR_NUM}, (int16_t)regdata16);
          break;
        }

        case MC_REG_I_D_KI:
        {
          PID_SetKI(&PIDIdHandle_M${MOTOR_NUM}, (int16_t)regdata16);
          break;
        }

        case MC_REG_I_D_KD:
        {
          PID_SetKD(&PIDIdHandle_M${MOTOR_NUM}, (int16_t)regdata16);
          break;
        }
  </#if><#-- (MEMORY_FOOTPRINT_REG && (MC.DRIVE_TYPE != "ACIM")) || (MC.ACIM_CONFIG == "LSO_FOC") -->

  <#if MEMORY_FOOTPRINT_REG2>
    <#if .vars.MC["M" + MOTOR_NUM?c + "_FLUX_WEAKENING_ENABLING"]>
        case MC_REG_FLUXWK_KP:
        {
          PID_SetKP(&PIDFluxWeakeningHandle_M${MOTOR_NUM}, (int16_t)regdata16);
          break;
        }

        case MC_REG_FLUXWK_KI:
        {
          PID_SetKI(&PIDFluxWeakeningHandle_M${MOTOR_NUM}, (int16_t)regdata16);
          break;
        }

        case MC_REG_FLUXWK_BUS:
        {
          FW_SetVref(&FW_M${MOTOR_NUM}, regdata16);
          break;
        }
    </#if> <#-- M1_FLUX_WEAKENING_ENABLING -->
  </#if><#-- MEMORY_FOOTPRINT_REG2 -->
</#if><#-- (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "ACIM") -->

        case MC_REG_BUS_VOLTAGE:
        case MC_REG_HEATS_TEMP:
        case MC_REG_MOTOR_POWER:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

<#if (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "ACIM")>
  <#if MEMORY_FOOTPRINT_REG2>
    <#if MEMORY_FOOTPRINT_REG>
        case MC_REG_I_A:
        case MC_REG_I_B:
        case MC_REG_I_ALPHA_MEAS:
        case MC_REG_I_BETA_MEAS:
        case MC_REG_I_Q_MEAS:
        case MC_REG_I_D_MEAS:
    </#if><#-- MEMORY_FOOTPRINT_REG -->
        case MC_REG_FLUXWK_BUS_MEAS:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
  </#if><#-- MEMORY_FOOTPRINT_REG2 -->

  <#if MEMORY_FOOTPRINT_REG>
        case MC_REG_I_Q_REF:
        {
          qd_t currComp;
          currComp = MCI_GetIqdref(pMCIN);
          currComp.q = (int16_t)regdata16;
          MCI_SetCurrentReferences(pMCIN,currComp);
          break;
        }

        case MC_REG_I_D_REF:
        {
          qd_t currComp;
          currComp = MCI_GetIqdref(pMCIN);
          currComp.d = (int16_t)regdata16;
          MCI_SetCurrentReferences(pMCIN,currComp);
          break;
        }
  </#if><#-- MEMORY_FOOTPRINT_REG -->

  <#if MEMORY_FOOTPRINT_REG2>  
        case MC_REG_V_Q:
        case MC_REG_V_D:
        case MC_REG_V_ALPHA:
        case MC_REG_V_BETA:
    <#if .vars["M" + MOTOR_NUM?c + "_IS_ENCODER"]>
        case MC_REG_ENCODER_EL_ANGLE:
        case MC_REG_ENCODER_SPEED:
    </#if><#-- IS_ENCODER -->
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
  </#if><#-- MEMORY_FOOTPRINT_REG2 -->
</#if><#-- (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "ACIM") -->

<#if MEMORY_FOOTPRINT_REG2>
  <#if .vars["M" + MOTOR_NUM?c + "_IS_HALL_SENSOR"]>
    <#if MEMORY_FOOTPRINT_REG>
        case MC_REG_HALL_EL_ANGLE:
        case MC_REG_HALL_SPEED:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
    </#if><#-- MEMORY_FOOTPRINT_REG -->
  </#if><#-- IS_HALL_SENSOR -->

  <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_PLL"]>
        case MC_REG_STOPLL_C1:
        {
          int16_t hC1;
          int16_t hC2;
          STO_PLL_GetObserverGains(&STO_PLL_M${MOTOR_NUM}, &hC1, &hC2);
          STO_PLL_SetObserverGains(&STO_PLL_M${MOTOR_NUM}, (int16_t)regdata16, hC2);
          break;
        }

        case MC_REG_STOPLL_C2:
        {
          int16_t hC1;
          int16_t hC2;
          STO_PLL_GetObserverGains(&STO_PLL_M${MOTOR_NUM}, &hC1, &hC2);
          STO_PLL_SetObserverGains(&STO_PLL_M${MOTOR_NUM}, hC1, (int16_t)regdata16);
          break;
        }

        case MC_REG_STOPLL_KI:
        {
          PID_SetKI (&(&STO_PLL_M${MOTOR_NUM})->PIRegulator, (int16_t)regdata16);
          break;
        }

        case MC_REG_STOPLL_KP:
        {
          PID_SetKP (&(&STO_PLL_M${MOTOR_NUM})->PIRegulator, (int16_t)regdata16);
          break;
        }

        case MC_REG_STOPLL_EL_ANGLE:
        case MC_REG_STOPLL_ROT_SPEED:
    <#if MEMORY_FOOTPRINT_REG>
        case MC_REG_STOPLL_I_ALPHA:
        case MC_REG_STOPLL_I_BETA:
    </#if><#-- MEMORY_FOOTPRINT_REG -->
        case MC_REG_STOPLL_BEMF_ALPHA:
        case MC_REG_STOPLL_BEMF_BETA:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
  </#if><#-- IS_STO_PLL -->

  <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_CORDIC"]>
        case MC_REG_STOCORDIC_EL_ANGLE:
        case MC_REG_STOCORDIC_ROT_SPEED:
    <#if MEMORY_FOOTPRINT_REG>
         case MC_REG_STOCORDIC_I_ALPHA:
         case MC_REG_STOCORDIC_I_BETA:
    </#if><#-- MEMORY_FOOTPRINT_REG -->
        case MC_REG_STOCORDIC_BEMF_ALPHA:
        case MC_REG_STOCORDIC_BEMF_BETA:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_STOCORDIC_C1:
        {
          int16_t hC1,hC2;
          STO_CR_GetObserverGains(&STO_CR_M${MOTOR_NUM}, &hC1,&hC2);
          STO_CR_SetObserverGains(&STO_CR_M${MOTOR_NUM}, (int16_t)regdata16, hC2);
           break;
        }
          
        case MC_REG_STOCORDIC_C2:
        {
          int16_t hC1,hC2;
          STO_CR_GetObserverGains(&STO_CR_M${MOTOR_NUM}, &hC1, &hC2);
          STO_CR_SetObserverGains(&STO_CR_M${MOTOR_NUM}, hC1, (int16_t)regdata16);
          break;
        }
  </#if><#-- IS_STO_CORDIC -->

        case MC_REG_DAC_USER1:
        case MC_REG_DAC_USER2:
          break;

  <#if .vars.MC["M" + MOTOR_NUM?c + "_FEED_FORWARD_CURRENT_REG_ENABLING"]>
        case MC_REG_FF_VQ:
        case MC_REG_FF_VD:
        case MC_REG_FF_VQ_PIOUT:
        case MC_REG_FF_VD_PIOUT:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
  </#if><#-- FEED_FORWARD_CURRENT_REG_ENABLING -->


  <#if MC.MOTOR_PROFILER>
        case MC_REG_OVERVOLTAGETHRESHOLD:
        {
          SCC_SetOverVoltageThreshold(&SCC,regdata16);
          break;
        }
          
        case MC_REG_UNDERVOLTAGETHRESHOLD:
        {
          SCC_SetUnderVoltageThreshold(&SCC,regdata16);
          break;
        }
        
        case MC_REG_SC_PWM_FREQUENCY:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
        
  </#if><#-- MOTOR_PROFILER -->

  <#if .vars.MC["M" + MOTOR_NUM?c + "_POSITION_CTRL_ENABLING"]>
        case MC_REG_POSITION_KP:
        {
          PID_SetKP(&PID_PosParamsM${MOTOR_NUM}, regdata16);
          break;
        }

        case MC_REG_POSITION_KI:
        {
          PID_SetKI(&PID_PosParamsM${MOTOR_NUM}, regdata16);
          break;
        }

        case MC_REG_POSITION_KD:
        {
          PID_SetKD(&PID_PosParamsM${MOTOR_NUM}, regdata16);
          break;
        }
  </#if><#-- POSITION_CTRL_ENABLING -->

  <#if MEMORY_FOOTPRINT_REG && ((.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] != "ACIM") || (MC.ACIM_CONFIG == "LSO_FOC"))>
        case MC_REG_SPEED_KP_DIV:
        {
          PID_SetKPDivisorPOW2(&PIDSpeedHandle_M${MOTOR_NUM}, regdata16);
          break;
        } 

        case MC_REG_SPEED_KI_DIV:
        {
          PID_SetKIDivisorPOW2(&PIDSpeedHandle_M${MOTOR_NUM}, regdata16);
          break;
        }

        case MC_REG_SPEED_KD_DIV:
        {
          PID_SetKDDivisorPOW2(&PIDSpeedHandle_M${MOTOR_NUM}, regdata16);
          break;
        }
  </#if><#-- (MEMORY_FOOTPRINT_REG && (MC.DRIVE_TYPE != "ACIM")) || (MC.ACIM_CONFIG == "LSO_FOC") -->
</#if><#-- MEMORY_FOOTPRINT_REG2 -->

<#if MEMORY_FOOTPRINT_REG>
  <#if (.vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || ((.vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "ACIM") && MC.ACIM_CONFIG == "LSO_FOC")>
        case MC_REG_I_D_KP_DIV:
        {
          PID_SetKPDivisorPOW2(&PIDIdHandle_M${MOTOR_NUM}, regdata16);
          break;
        }

        case MC_REG_I_D_KI_DIV:
        {
          PID_SetKIDivisorPOW2(&PIDIdHandle_M${MOTOR_NUM}, regdata16);
          break;
        }
          
        case MC_REG_I_D_KD_DIV:
        {
          PID_SetKDDivisorPOW2(&PIDIdHandle_M${MOTOR_NUM}, regdata16);
          break;
        }
          
        case MC_REG_I_Q_KP_DIV:
        {
          PID_SetKPDivisorPOW2(&PIDIqHandle_M${MOTOR_NUM}, regdata16);
          break;
        }
          
        case MC_REG_I_Q_KI_DIV:
        {
          PID_SetKIDivisorPOW2(&PIDIqHandle_M${MOTOR_NUM}, regdata16);
          break;
        }
          
        case MC_REG_I_Q_KD_DIV:
        {
          PID_SetKDDivisorPOW2(&PIDIqHandle_M${MOTOR_NUM}, regdata16);
          break;
        }
  </#if><#-- (.vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || ((.vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "ACIM") && MC.ACIM_CONFIG == "LSO_FOC") -->
  
  <#if .vars.MC["M" + MOTOR_NUM?c + "_POSITION_CTRL_ENABLING"]>
        case MC_REG_POSITION_KP_DIV:
        {
          PID_SetKPDivisorPOW2(&PID_PosParamsM${MOTOR_NUM}, regdata16);
          break;
        }

        case MC_REG_POSITION_KI_DIV:
        {
          PID_SetKIDivisorPOW2(&PID_PosParamsM${MOTOR_NUM}, regdata16);
          break;
        }

        case MC_REG_POSITION_KD_DIV:
        {
          PID_SetKDDivisorPOW2(&PID_PosParamsM${MOTOR_NUM}, regdata16);
          break;
        }
  </#if><#-- POSITION_CTRL_ENABLING -->

  <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_PLL"]>
        case MC_REG_STOPLL_KI_DIV:
        {
          PID_SetKIDivisorPOW2 (&(&STO_PLL_M${MOTOR_NUM})->PIRegulator,regdata16);
          break;
        }

        case MC_REG_STOPLL_KP_DIV:
        {
          PID_SetKPDivisorPOW2 (&(&STO_PLL_M${MOTOR_NUM})->PIRegulator,regdata16);
          break;
        }
  </#if><#-- IS_STO_PLL -->

  <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC" && .vars.MC["M"+ MOTOR_NUM?c + "_DBG_OPEN_LOOP_ENABLE"]>
        case MC_REG_FOC_VQREF:
        {
          OL_UpdateVoltage(&OpenLoop_ParamsM${MOTOR_NUM}, ((regdata16 * 32767) / 100));
          break;
        }
  </#if><#-- .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC" && .vars.MC["M"+ MOTOR_NUM?c + "_DBG_OPEN_LOOP_ENABLE"] -->

  <#if .vars.MC["M" + MOTOR_NUM?c + "_FLUX_WEAKENING_ENABLING"]>
        case MC_REG_FLUXWK_KP_DIV: 
        {
          PID_SetKPDivisorPOW2 (&PIDFluxWeakeningHandle_M${MOTOR_NUM},regdata16);
          break;
        }

        case MC_REG_FLUXWK_KI_DIV:
        {
          PID_SetKIDivisorPOW2 (&PIDFluxWeakeningHandle_M${MOTOR_NUM},regdata16);
          break;
        }
  </#if><#-- M1_FLUX_WEAKENING_ENABLING -->

  <#if .vars["M" + MOTOR_NUM?c + "_IS_SENSORLESS"] || (.vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_ADC")|| (.vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_COMP")>
        case MC_REG_PULSE_VALUE:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
  </#if><#-- IS_SENSORLESS || MC.SPEED_SENSOR == "SENSORLESS_ADC"-->
</#if><#-- MEMORY_FOOTPRINT_REG -->

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 2;
      break;
    }

    case TYPE_DATA_32BIT:
    {
<#if UNALIGNMENT_SUPPORTED>
      uint32_t regdata32 = *(uint32_t *)data; //cstat !MISRAC2012-Rule-11.3
<#else><#-- UNALIGNMENT_NOT_SUPPORTED -->
      uint32_t regdata32 = (((uint32_t)(*(uint16_t *)&data[2])) << 16) | *(uint16_t *)data;
</#if><#-- UNALIGNMENT_SUPPORTED -->

      switch (regID)
      {
        case MC_REG_FAULTS_FLAGS:
        case MC_REG_SPEED_MEAS:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_SPEED_REF:
        {
          MCI_ExecSpeedRamp(pMCIN,((((int16_t)regdata32) * ((int16_t)SPEED_UNIT)) / (int16_t)U_RPM), 0);
          break;
        }

<#if MEMORY_FOOTPRINT_REG2>
  <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_PLL"]>
        case MC_REG_STOPLL_EST_BEMF:
        case MC_REG_STOPLL_OBS_BEMF:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
  </#if><#-- IS_STO_PLL -->
  
  <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_CORDIC"]>
        case MC_REG_STOCORDIC_EST_BEMF:
        case MC_REG_STOCORDIC_OBS_BEMF:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
  </#if><#-- IS_STO_CORDIC -->

  <#if .vars.MC["M" + MOTOR_NUM?c + "_FEED_FORWARD_CURRENT_REG_ENABLING"]>
        case MC_REG_FF_1Q:
        {
          FF_M${MOTOR_NUM}.wConstant_1Q = (int32_t)regdata32;
          break;
        }

        case MC_REG_FF_1D:
        {
          FF_M${MOTOR_NUM}.wConstant_1D = (int32_t)regdata32;
          break;
        }

        case MC_REG_FF_2:
        {
          FF_M${MOTOR_NUM}.wConstant_2 = (int32_t)regdata32;
          break;
        }
  </#if><#-- FEED_FORWARD_CURRENT_REG -->
</#if><#-- MEMORY_FOOTPRINT_REG2 -->

  <#if MC.MOTOR_PROFILER>
        case MC_REG_SC_RS:
        case MC_REG_SC_LS:
        case MC_REG_SC_KE:
        case MC_REG_SC_VBUS:
        case MC_REG_SC_MEAS_NOMINALSPEED:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
        
        case MC_REG_SC_CURRENT:
        { /* Profiler is supported only by series supporting unaligned access */
          if(SCC.sm_state==SCC_IDLE || SCC_CALIBRATION_END==SCC.sm_state)
          {
            float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
            SCC_SetNominalCurrent(&SCC, fregdata);
          }
          else{
              retVal = MCP_ERROR_RO_REG;
          }
          break;
        }
        
        case MC_REG_SC_SPDBANDWIDTH:
        { 
          float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
          OTT_SetSpeedRegulatorBandwidth(&OTT, fregdata);
          break;
        }
        
        case MC_REG_SC_LDLQRATIO:
        { 
          float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
          SCC_SetLdLqRatio(&SCC, fregdata);
          break;
        }
        
        case MC_REG_SC_NOMINAL_SPEED:
        {
          SCC_SetNominalSpeed (&SCC, (int32_t) regdata32);
          break;
        }
        
        case MC_REG_SC_CURRBANDWIDTH:
        { 
          float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
          SCC_SetCurrentBandwidth(&SCC, fregdata);
          break;
        }
        case MC_REG_RESISTOR_OFFSET:
        {
          float fregdata = *(float*)data;
          SCC_SetResistorOffset(&SCC,fregdata);
          break;
        }
        case MC_REG_SC_J:
        case MC_REG_SC_F:
        case MC_REG_SC_MAX_CURRENT:
        case MC_REG_SC_STARTUP_SPEED:
        case MC_REG_SC_STARTUP_ACC:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
  </#if><#-- MOTOR_PROFILER -->

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 4;
      break;
    }

<#if MEMORY_FOOTPRINT_REG>
    case TYPE_DATA_STRING:
    {
      const char_t *charData = (const char_t *)data;
      char_t *dummy = (char_t *)data;
      retVal = MCP_ERROR_RO_REG;
      /* Used to compute String length stored in RXBUFF even if Reg does not exist */
      /* It allows to jump to the next command in the buffer */
      (void)RI_MovString(charData, dummy, size, dataAvailable);
      break;
    }
</#if><#-- MEMORY_FOOTPRINT_REG -->

    case TYPE_DATA_RAW:
    {
      uint16_t rawSize = *(uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
      /* The size consumed by the structure is the structure size + 2 bytes used to store the size */
      *size = rawSize + 2U;
      uint8_t *rawData = data; /* rawData points to the first data (after size extraction) */
      rawData++;
      rawData++;

      if (*size > (uint16_t)dataAvailable)
      { 
        /* The decoded size of the raw structure can not match with transmitted buffer, error in buffer
           construction */
        *size = 0;
        retVal = MCP_ERROR_BAD_RAW_FORMAT; /* This error stop the parsing of the CMD buffer */
      }
      else
      {
        switch (regID)
        {
          case MC_REG_APPLICATION_CONFIG:
<#if MEMORY_FOOTPRINT_REG>
          case MC_REG_MOTOR_CONFIG:
</#if><#-- MEMORY_FOOTPRINT_REG -->
          case MC_REG_GLOBAL_CONFIG:
          case MC_REG_FOCFW_CONFIG:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

          case MC_REG_SPEED_RAMP:
          {
            int32_t rpm;
            uint16_t duration;

<#if UNALIGNMENT_SUPPORTED >
            rpm = *(int32_t *)rawData; //cstat !MISRAC2012-Rule-11.3
<#else><#-- UNALIGNMENT_NOT_SUPPORTED -->
            /* 32 bits access are splited into 2x16 bits access */
            rpm = (((int32_t)(*(int16_t *)&rawData[2])) << 16) | *(uint16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
</#if><#-- UNALIGNMENT_SUPPORTED -->
            duration = *(uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            MCI_ExecSpeedRamp(pMCIN, (int16_t)((rpm * SPEED_UNIT) / U_RPM), duration);
            break;
          }

<#if MEMORY_FOOTPRINT_REG>
  <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC">
          case MC_REG_TORQUE_RAMP:
          {
            uint32_t torque;
            uint16_t duration;

    <#if UNALIGNMENT_SUPPORTED >
            torque = *(uint32_t *)rawData; //cstat !MISRAC2012-Rule-11.3
    <#else><#-- UNALIGNMENT_NOT_SUPPORTED -->
            /* 32 bits access are splited into 2x16 bits access */
            //cstat !MISRAC2012-Rule-11.3
            torque = ((uint32_t)(*(int16_t *)&rawData[2]))<<16 | *(uint16_t *)rawData;
    </#if><#-- UNALIGNMENT_SUPPORTED -->
            duration = *(uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            MCI_ExecTorqueRamp(pMCIN, (int16_t)torque, duration);
            break;
          }

    <#if .vars.MC["M" + MOTOR_NUM?c + "_POSITION_CTRL_ENABLING"]>
          case MC_REG_POSITION_RAMP:
          {
            FloatToU32 Position;
            FloatToU32 Duration;
            /* 32 bits access are split into 2x16 bits access */
            Position.U32_Val = ((int32_t)(*(int16_t *)&rawData[2]))<<16 | *(uint16_t *)rawData;
            /* 32 bits access are split into 2x16 bits access */
            Duration.U32_Val = ((int32_t)(*(int16_t *)&rawData[6]))<<16 | *(uint16_t *)&rawData[4];
            MCI_ExecPositionCommand(pMCIN, Position.Float_Val, Duration.Float_Val);
            break;
          }
    </#if><#-- POSITION_CTRL -->
  </#if><#-- .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC" -->

  <#if .vars["M" + MOTOR_NUM?c + "_IS_SENSORLESS"] || (.vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_ADC")|| (.vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_COMP")>
          case MC_REG_REVUP_DATA:
          {
            int32_t rpm;
            RevUpCtrl_PhaseParams_t revUpPhase;
            uint8_t i;
            uint8_t nbrOfPhase = (((uint8_t)rawSize) / 8U);
              
            if (((0U != ((rawSize) % 8U))) || ((nbrOfPhase > RUC_MAX_PHASE_NUMBER) != 0))
            {
              retVal = MCP_ERROR_BAD_RAW_FORMAT;
            }
            else
            {
              for (i = 0; i <nbrOfPhase; i++)
              {
    <#if UNALIGNMENT_SUPPORTED>
              rpm = *(int32_t *) &rawData[i * 8U]; //cstat !MISRAC2012-Rule-11.3
    <#else><#-- UNALIGNMENT_NOT_SUPPORTED -->
              /* &rawData is guarantee to be 16 bits aligned, so 32 bits access are to be splitted */
              //cstat !MISRAC2012-Rule-11.3
              rpm = ((int32_t)(*(int16_t *)&rawData[2+i*8]))<<16 | *(uint16_t *)&rawData[i*8];
    </#if><#-- UNALIGNMENT_SUPPORTED -->
              revUpPhase.hFinalMecSpeedUnit = (((int16_t)rpm) * ((int16_t)SPEED_UNIT)) / ((int16_t)U_RPM);
    <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC">
              revUpPhase.hFinalTorque = *((int16_t *) &rawData[4U + (i * 8U)]); //cstat !MISRAC2012-Rule-11.3
    </#if><#-- DRIVE_TYPE == "FOC" -->
    <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "SIX_STEP">
              revUpPhase.hFinalPulse = *((int16_t *) &rawData[4U + (i * 8U)]); //cstat !MISRAC2012-Rule-11.3
    </#if><#-- DRIVE_TYPE == "SIX_STEP" -->
              revUpPhase.hDurationms  = *((uint16_t *) &rawData[6U +(i * 8U)]); //cstat !MISRAC2012-Rule-11.3
              (void)RUC_SetPhase(&RevUpControlM${MOTOR_NUM}, i, &revUpPhase);
              }
            }
            break;
          }

    <#if SIX_STEP >
          case MC_REG_BEMF_ADC_CONF:
          {
            Bemf_Sensing_Params BemfAdcConfig;
            Bemf_Demag_Params bemfAdcDemagConfig;
            uint16_t ZcRising2CommDelay, ZcFalling2CommDelay, OnSensingEnThres, OnSensingHisteresys, ConvertedData, computationdelayChoice;

            BemfAdcConfig.AdcThresholdHighPerc = *(uint16_t *)rawData; //cstat !MISRAC2012-Rule-11.3

            BemfAdcConfig.AdcThresholdPwmPerc = *(uint16_t *)&rawData[2]; //cstat !MISRAC2012-Rule-11.3

            BemfAdcConfig.AdcThresholdLowPerc = *(uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            
            ConvertedData = *(uint16_t *)&rawData[6]; //cstat !MISRAC2012-Rule-11.3
            BemfAdcConfig.SamplingPointOff = (uint16_t) ((PWM_PERIOD_CYCLES * ConvertedData) / 100);

            ConvertedData = *(uint16_t *)&rawData[8]; //cstat !MISRAC2012-Rule-11.3
            BemfAdcConfig.SamplingPointOn = (uint16_t) ((PWM_PERIOD_CYCLES * ConvertedData) / 100);
            
            ConvertedData = *(uint16_t *)&rawData[10]; //cstat !MISRAC2012-Rule-11.3
            ZcRising2CommDelay = (uint16_t) ((ConvertedData * 512) / 60);

            ConvertedData = *(uint16_t *)&rawData[12]; //cstat !MISRAC2012-Rule-11.3
            ZcFalling2CommDelay = (uint16_t) ((ConvertedData * 512) / 60);
      
            bemfAdcDemagConfig.DemagMinimumThreshold = *(uint16_t *)&rawData[14]; //cstat !MISRAC2012-Rule-11.3
            
            ConvertedData = *(uint16_t *)&rawData[16]; //cstat !MISRAC2012-Rule-11.3
            bemfAdcDemagConfig.DemagMinimumSpeedUnit = (uint16_t) ((ConvertedData * SPEED_UNIT) / U_RPM); //cstat !MISRAC2012-Rule-11.3
            
            ConvertedData = *(uint16_t *)&rawData[18]; //cstat !MISRAC2012-Rule-11.3
            OnSensingEnThres = (uint16_t) ((PWM_PERIOD_CYCLES * ConvertedData) / 100);
            
            ConvertedData = *(uint16_t *)&rawData[20]; //cstat !MISRAC2012-Rule-11.3
            OnSensingHisteresys = (uint16_t) ((PWM_PERIOD_CYCLES * ConvertedData) / 100);
            
            BemfAdcConfig.AWDfiltering = *(uint16_t *)&rawData[22]; //cstat !MISRAC2012-Rule-11.3
			
            computationdelayChoice = *(uint16_t *)&rawData[24];

            (void)BADC_SetBemfSensorlessParam(&Bemf_ADC_M1, &BemfAdcConfig,&ZcRising2CommDelay,&ZcFalling2CommDelay,&bemfAdcDemagConfig,
                                              &OnSensingEnThres,&OnSensingHisteresys, &computationdelayChoice);
            
            break;
          }
    </#if><#-- DRIVE_TYPE == "SIX_STEP" -->
  </#if><#-- IS_SENSORLESS || MC.SPEED_SENSOR == "SENSORLESS_ADC" -->

  <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC">
          case MC_REG_CURRENT_REF:
          {
            qd_t currComp;
            currComp.q = *((int16_t *) rawData); //cstat !MISRAC2012-Rule-11.3
            currComp.d = *((int16_t *) &rawData[2]); //cstat !MISRAC2012-Rule-11.3
            MCI_SetCurrentReferences(pMCIN, currComp);
            break;
          }
  </#if><#-- DRIVE_TYPE == "FOC" -->

  <#if MC.MCP_ASYNC_OVER_UART_A_EN>
          case MC_REG_ASYNC_UARTA:
          {
            retVal =  MCPA_cfgLog (&MCPA_UART_A, rawData);
            break;
          }
  </#if><#-- MC.MCP_ASYNC_OVER_UART_A_EN -->

  <#if MC.MCP_ASYNC_OVER_UART_B_EN>
          case MC_REG_ASYNC_UARTB:
          {
            retVal =  MCPA_cfgLog (&MCPA_UART_B, rawData);
            break;
          }
  </#if><#-- MC.MCP_ASYNC_OVER_UART_B_EN -->

  <#if MC.MCP_ASYNC_OVER_STLNK_EN>
          case MC_REG_ASYNC_STLNK:
          {
            retVal =  MCPA_cfgLog (&MCPA_STLNK, rawData);
            break;
          }
  </#if><#-- MC.MCP_ASYNC_OVER_STLNK_EN -->
</#if><#-- MEMORY_FOOTPRINT_REG -->

          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
      }
      break;
    }

    default:
    {
      retVal = MCP_ERROR_BAD_DATA_TYPE;
      *size =0; /* From this point we are not able anymore to decode the RX buffer */
      break;
    }
  }
  return (retVal);
}
</#macro>

<#list 1..(MC.DRIVE_NUMBER?number) as NUM>

   <@SetRegsFct MOTOR_NUM=NUM />
  
</#list>

uint8_t RI_GetRegisterGlobal(uint16_t regID,uint8_t typeID,uint8_t * data,uint16_t *size,int16_t freeSpace){
    uint8_t retVal = MCP_CMD_OK;
<#if (MEMORY_FOOTPRINT_REG2 == true) && (DWT_CYCCNT_SUPPORTED == true) && (MC.DBG_MCU_LOAD_MEASURE == true)>
    uint8_t motorID=0;
    MCI_Handle_t *pMCIN = &Mci[motorID];
</#if><#-- (MEMORY_FOOTPRINT_REG2 == true) && (DWT_CYCCNT_SUPPORTED == true) && (MC.DBG_MCU_LOAD_MEASURE == true) -->
    switch (typeID)
    {
      case TYPE_DATA_8BIT:
      {
        if (freeSpace > 0)
        {
          switch (regID)
          {
<#if MC.PFC_ENABLED>
            case MC_REG_PFC_STATUS:
            case MC_REG_PFC_ENABLED:
              break;
</#if><#-- MC.PFC_ENABLED -->
            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 1;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_16BIT:
      {
 <#if MEMORY_FOOTPRINT_REG2>
  <#if MC.DEBUG_DAC_FUNCTIONALITY_EN>
       int16_t *regdata16 = (int16_t *) data; //cstat !MISRAC2012-Rule-11.3
  </#if><#-- MC.DEBUG_DAC_FUNCTIONALITY_EN -->
</#if><#-- MEMORY_FOOTPRINT_REG2 -->
        if (freeSpace >= 2)
        {
          switch (regID) 
          {
<#if MEMORY_FOOTPRINT_REG2>
  <#if MC.DEBUG_DAC_FUNCTIONALITY_EN>
            case MC_REG_DAC_OUT1:
            {
              *regdata16 = (int16_t)DAC_GetChannelConfig(&DAC_Handle , DAC_CH1);
              break;
            }

            case MC_REG_DAC_OUT2:
            {
              *regdata16 = (int16_t)DAC_GetChannelConfig(&DAC_Handle , DAC_CH2);
              break;
            }
  </#if><#-- MC.DEBUG_DAC_FUNCTIONALITY_EN -->
            case MC_REG_DAC_USER1:
            case MC_REG_DAC_USER2:
              break;
</#if><#-- MEMORY_FOOTPRINT_REG2 -->

<#if MC.PFC_ENABLED>
            case MC_REG_PFC_DCBUS_REF:
            case MC_REG_PFC_DCBUS_MEAS:
            case MC_REG_PFC_ACBUS_FREQ:
            case MC_REG_PFC_ACBUS_RMS:
  <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_PFC_I_KP:
            case MC_REG_PFC_I_KI:
            case MC_REG_PFC_I_KD:
  </#if><#-- MEMORY_FOOTPRINT_REG -->
            case MC_REG_PFC_V_KP:
            case MC_REG_PFC_V_KI:
            case MC_REG_PFC_V_KD:
            case MC_REG_PFC_STARTUP_DURATION:
            case MC_REG_PFC_I_KP_DIV:
            case MC_REG_PFC_I_KI_DIV:
            case MC_REG_PFC_I_KD_DIV:
            case MC_REG_PFC_V_KP_DIV:
            case MC_REG_PFC_V_KI_DIV:
            case MC_REG_PFC_V_KD_DIV:
              break;
</#if><#-- MC.PFC_ENABLED -->
            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 2;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_32BIT:
      {
<#if (MEMORY_FOOTPRINT_REG2 == true) && (DWT_CYCCNT_SUPPORTED == true) && (MC.DBG_MCU_LOAD_MEASURE == true)> 
        uint32_t *regdataU32 = (uint32_t *)data; //cstat !MISRAC2012-Rule-11.3
</#if><#-- (MEMORY_FOOTPRINT_REG2 == true) && (DWT_CYCCNT_SUPPORTED == true) && (MC.DBG_MCU_LOAD_MEASURE == true) -->
        if (freeSpace >= 4)
        {
          switch (regID)
          {
<#if (MEMORY_FOOTPRINT_REG2 == true) && (DWT_CYCCNT_SUPPORTED == true) && (MC.DBG_MCU_LOAD_MEASURE == true)>
            case MC_REG_PERF_CPU_LOAD:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = MC_Perf_GetCPU_Load(pMCIN->pPerfMeasure);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_PERF_MIN_CPU_LOAD:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = MC_Perf_GetMinCPU_Load(pMCIN->pPerfMeasure);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_PERF_MAX_CPU_LOAD:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = MC_Perf_GetMaxCPU_Load(pMCIN->pPerfMeasure);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
</#if><#-- (MEMORY_FOOTPRINT_REG2 == true) && (DWT_CYCCNT_SUPPORTED == true) && (MC.DBG_MCU_LOAD_MEASURE == true) -->

<#if MC.PFC_ENABLED>
            case MC_REG_PFC_FAULTS:
              break;
</#if><#-- MC.PFC_ENABLED-->
            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 4;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_STRING:
      {
<#if MEMORY_FOOTPRINT_REG>
        char_t *charData = (char_t *)data;
</#if><#-- MEMORY_FOOTPRINT_REG -->
        switch (regID)
        {
<#if MEMORY_FOOTPRINT_REG>
          case MC_REG_FW_NAME:
            retVal = RI_MovString (FIRMWARE_NAME ,charData, size, freeSpace);
            break;

          case MC_REG_CTRL_STAGE_NAME:
          {
            retVal = RI_MovString (CTL_BOARD ,charData, size, freeSpace);
            break;
          }
</#if><#-- MEMORY_FOOTPRINT_REG -->
          default:
          {

            retVal = MCP_ERROR_UNKNOWN_REG;
            *size= 0 ; /* */

            break;
          }
        }
        break;

      }
      case TYPE_DATA_RAW:
      {
        /* First 2 bytes of the answer is reserved to the size */
        uint16_t *rawSize = (uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
        uint8_t * rawData = data;
        rawData++;
        rawData++;

        switch (regID)
        {
          case MC_REG_GLOBAL_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(GlobalConfig_reg_t);
            if (((*rawSize) + 2U) > (uint16_t)freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else 
            {
              (void)memcpy(rawData, &globalConfig_reg, sizeof(GlobalConfig_reg_t));
            }
            break;
          }
          case MC_REG_ASYNC_UARTA:
          case MC_REG_ASYNC_UARTB:
          case MC_REG_ASYNC_STLNK:
          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }

        /* Size of the answer is size of the data + 2 bytes containing data size */
        *size = (*rawSize) + 2U;
        break;
      }

      default:
      {
        retVal = MCP_ERROR_BAD_DATA_TYPE;
        break;
      }
    }
  return (retVal);
}

<#macro GetRegFct MOTOR_NUM > 
  uint8_t RI_GetRegisterMotor${MOTOR_NUM}(uint16_t regID,uint8_t typeID,uint8_t * data,uint16_t *size,int16_t freeSpace) {
    uint8_t retVal = MCP_CMD_OK;
    uint8_t motorID=${MOTOR_NUM-1};
    MCI_Handle_t *pMCIN = &Mci[motorID];
    BusVoltageSensor_Handle_t* BusVoltageSensor= &BusVoltageSensor_M${MOTOR_NUM}._Super;
    switch (typeID)
    {
      case TYPE_DATA_8BIT:
      {
        if (freeSpace > 0)
        {
          switch (regID)
          {
            case MC_REG_STATUS:
            {
              *data = (uint8_t)MCI_GetSTMState(pMCIN);
              break;
            }

            case MC_REG_CONTROL_MODE:
            {
              *data = (uint8_t)MCI_GetControlMode(pMCIN);
              break;
            }

<#if .vars["M" + MOTOR_NUM?c + "_IS_SENSORLESS"] || (.vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_ADC")>
            case MC_REG_RUC_STAGE_NBR:
            {
              *data = (uint8_t)RUC_GetNumberOfPhases(&RevUpControlM${MOTOR_NUM});
              break;
            }
</#if><#-- IS_SENSORLESS -->


<#if MC.MOTOR_PROFILER>
            case MC_REG_SC_CHECK:
            {
              *data = (uint8_t) 1u;
              break;
            }

            case MC_REG_SC_STATE:
            {
              uint8_t state ;
              state = SCC_GetState(&SCC);
              state += OTT_GetState (&OTT);
              *data = state;
              break;
            }

            case MC_REG_SC_STEPS:
            {
              uint8_t steps ;
              steps = SCC_GetSteps(&SCC);
              steps += OTT_GetSteps (&OTT);
              *data = steps-1u;
              break;
            }

            case MC_REG_SC_PP:
            {
              *data = SPD_GetElToMecRatio(&STO_PLL_M1._Super);
              break;
            }

            case MC_REG_SC_FOC_REP_RATE:
            {
              *data = SCC_GetFOCRepRate(&SCC);
              break;
            }

            case MC_REG_SC_COMPLETED:
            {
              *data = OTT_IsMotorAlreadyProfiled(&OTT);
              break;
             }

  <#if .vars.MC["M" + MOTOR_NUM?c + "_AUXILIARY_SPEED_SENSOR"] == "HALL_SENSOR">
            case MC_REG_HT_STATE:
            {
              *data = (uint8_t) HT.sm_state;
              break;
            }

            case MC_REG_HT_PROGRESS:
            {
              *data = HT.bProgressPercentage;
              break;
            }

            case MC_REG_HT_PLACEMENT:
            {
              *data = HT.bPlacement;
              break;
            }
  </#if><#-- AUXILIARY_SPEED_SENSOR == HALL_SENSOR -->
</#if> <#-- MOTOR_PROFILER -->

<#if .vars.MC["M" + MOTOR_NUM?c + "_POSITION_CTRL_ENABLING"]> 
  <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_POSITION_CTRL_STATE:
            {
              *data = (uint8_t) TC_GetControlPositionStatus(&PosCtrlM${MOTOR_NUM});
              break;
            }

            case MC_REG_POSITION_ALIGN_STATE:
            {
              *data = (uint8_t) TC_GetAlignmentStatus(&PosCtrlM${MOTOR_NUM});
              break;
            }
  </#if><#-- MEMORY_FOOTPRINT_REG -->
</#if><#-- POSITION_CTRL_ENABLING -->

<#if .vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "SIX_STEP">
            case MC_REG_LOWSIDE_MODULATION:
            {
              *data = PWMC_GetLSModConfig(&PWM_Handle_M1);
              break;
            }

            case MC_REG_QUASI_SYNCH:  
            {
              *data = PWMC_GetQuasiSynchState(&PWM_Handle_M${MOTOR_NUM});			
              break;
            }

  <#if .vars.MC["M" + MOTOR_NUM?c + "_DBG_OPEN_LOOP_ENABLE"] == true>
           case MC_REG_OPENLOOP:
           {
             *data = (uint8_t)OLS_GetOpenLoopFlag(pOLS[0]);
             break;
           }
           
           case MC_REG_COMMUTATION_STEPBUFSIZE:
           {
             *data = OLS_GetDutyCycleRefTableSize(pOLS[0]);
              break;
            }
            
           case MC_REG_OPENLOOP_DC:
           {
             *data = OLS_GetDutyCycleRef(pOLS[0]);
             break;
           }
           
           case MC_REG_OPENLOOP_REVUP:
           {
             *data = OLS_GetRevUpFlag(pOLS[0]);
             break;
           }

           case MC_REG_OPENLOOP_VOLTFACTOR:
           {
             *data = OLS_GetVoltageFactor(pOLS[0]);
             break;
           }

    <#if .vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_ADC">
           case MC_REG_OPENLOOP_SENSING:
           {
             *data = (uint8_t)OLS_GetOnSensing(pOLS[0]) | (uint8_t)OLS_GetOnSensingStatus(&Bemf_ADC_M1);
             break;
           }
    </#if><#-- SENSORLESS_ADC -->
  </#if><#-- DBG_OPEN_LOOP_ENABLE -->
</#if><#-- DRIVE_TYPE == "SIX_STEP" -->

            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 1;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_16BIT:
      {
        uint16_t *regdataU16 = (uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
        int16_t *regdata16 = (int16_t *) data; //cstat !MISRAC2012-Rule-11.3

        if (freeSpace >= 2)
        {
          switch (regID) 
          {
<#if .vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "SIX_STEP">
  <#if .vars.MC["M" + MOTOR_NUM?c + "_DBG_OPEN_LOOP_ENABLE"] == true>
           case MC_REG_OPENLOOP_CURRFACTOR:
           {
             *regdata16 = OLS_GetCurrentFactor(pOLS[0]);
             break;
           }
  </#if><#-- DBG_OPEN_LOOP_ENABLE -->
</#if><#-- DRIVE_TYPE == "SIX_STEP" -->
<#if MEMORY_FOOTPRINT_REG2 && ((.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] != "ACIM") || (MC.ACIM_CONFIG == "LSO_FOC"))>
            case MC_REG_SPEED_KP:
            {
              *regdata16 = PID_GetKP(&PIDSpeedHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_SPEED_KI:
            {
              *regdata16 = PID_GetKI(&PIDSpeedHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_SPEED_KD:
            {
              *regdata16 = PID_GetKD(&PIDSpeedHandle_M${MOTOR_NUM});
              break;
            }

  <#if (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || (MC.ACIM_CONFIG == "LSO_FOC")>
    <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_I_Q_KP:
            {
              *regdata16 = PID_GetKP(&PIDIqHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_I_Q_KI:
            {
              *regdata16 = PID_GetKI(&PIDIqHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_I_Q_KD:
            {
              *regdata16 = PID_GetKD(&PIDIqHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_I_D_KP:
            {
              *regdata16 = PID_GetKP(&PIDIdHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_I_D_KI:
            {
              *regdata16 = PID_GetKI(&PIDIdHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_I_D_KD:
            {
              *regdata16 = PID_GetKD(&PIDIdHandle_M${MOTOR_NUM});
              break;
            }
    </#if><#-- MEMORY_FOOTPRINT_REG -->
  </#if><#-- (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || (MC.ACIM_CONFIG == "LSO_FOC") -->

  <#if .vars.MC["M" + MOTOR_NUM?c + "_FLUX_WEAKENING_ENABLING"]>
            case MC_REG_FLUXWK_KP:
            {
              *regdata16 = PID_GetKP(&PIDFluxWeakeningHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_FLUXWK_KI:
            {
              *regdata16 = PID_GetKI(&PIDFluxWeakeningHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_FLUXWK_BUS:
            {
              *regdataU16 = FW_GetVref(&FW_M${MOTOR_NUM});
              break;
            }

            case MC_REG_FLUXWK_BUS_MEAS:
            {
              *regdata16 = (int16_t)FW_GetAvVPercentage(&FW_M${MOTOR_NUM});
              break;
            }
  </#if><#-- M1_FLUX_WEAKENING_ENABLING -->
</#if><#-- MEMORY_FOOTPRINT_REG2 && ((.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] != "ACIM") || (MC.ACIM_CONFIG == "LSO_FOC")) -->

            case MC_REG_BUS_VOLTAGE:
            {
              *regdataU16 = VBS_GetAvBusVoltage_V(BusVoltageSensor);
              break;
            }

            case MC_REG_HEATS_TEMP:
            {
              *regdata16 = NTC_GetAvTemp_C(&TempSensor_M${MOTOR_NUM});
              break;
            }

<#if MEMORY_FOOTPRINT_REG2>
  <#if (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "ACIM")>
    <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_I_A:
            {
              *regdata16 = MCI_GetIab(pMCIN).a;
              break;
            }

            case MC_REG_I_B:
            {
              *regdata16 = MCI_GetIab(pMCIN).b;
              break;
            }

            case MC_REG_I_ALPHA_MEAS:
            {
              *regdata16 = MCI_GetIalphabeta(pMCIN).alpha;
              break;
            }

            case MC_REG_I_BETA_MEAS:
            {
              *regdata16 = MCI_GetIalphabeta(pMCIN).beta;
              break;
            }

            case MC_REG_I_Q_MEAS:
            {
              *regdata16 = MCI_GetIqd(pMCIN).q;
              break;
            }

            case MC_REG_I_D_MEAS:
            {
              *regdata16 = MCI_GetIqd(pMCIN).d;
              break;
            }

            case MC_REG_I_Q_REF:
            {
              *regdata16 = MCI_GetIqdref(pMCIN).q;
              break;
            }

            case MC_REG_I_D_REF:
            {
              *regdata16 = MCI_GetIqdref(pMCIN).d;
              break;
            }
    </#if><#-- MEMORY_FOOTPRINT_REG -->

            case MC_REG_V_Q:
            {
              *regdata16 = MCI_GetVqd(pMCIN).q;
              break;
            }

            case MC_REG_V_D:
            {
              *regdata16 = MCI_GetVqd(pMCIN).d;
              break;
            }

            case MC_REG_V_ALPHA:
            {
              *regdata16 = MCI_GetValphabeta(pMCIN).alpha;
              break;
            }

            case MC_REG_V_BETA:
            {
              *regdata16 = MCI_GetValphabeta(pMCIN).beta;
              break;
            }
  </#if><#-- (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || (.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] == "ACIM") -->

  <#if .vars["M" + MOTOR_NUM?c + "_IS_ENCODER"]>
            case MC_REG_ENCODER_EL_ANGLE:
            {
              *regdata16 = SPD_GetElAngle ((SpeednPosFdbk_Handle_t*) &ENCODER_M${MOTOR_NUM}); //cstat !MISRAC2012-Rule-11.3
              break;
            }

            case MC_REG_ENCODER_SPEED:
            {
              *regdata16 = SPD_GetS16Speed ((SpeednPosFdbk_Handle_t*) &ENCODER_M${MOTOR_NUM}); //cstat !MISRAC2012-Rule-11.3
              break;
            }
  </#if><#-- IS_ENCODER_M -->

  <#if .vars["M" + MOTOR_NUM?c + "_IS_HALL_SENSOR"]>
    <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_HALL_EL_ANGLE:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetElAngle ((SpeednPosFdbk_Handle_t*) &HALL_M${MOTOR_NUM});
              break;
            }

            case MC_REG_HALL_SPEED:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetS16Speed ((SpeednPosFdbk_Handle_t*) &HALL_M${MOTOR_NUM});
              break;
            }
    </#if><#-- MEMORY_FOOTPRINT_REG -->
  </#if><#-- IS_HALL_SENSOR -->
  
  <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_PLL"]>
            case MC_REG_STOPLL_EL_ANGLE:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetElAngle((SpeednPosFdbk_Handle_t *)&STO_PLL_M${MOTOR_NUM});
              break;
            }

            case MC_REG_STOPLL_ROT_SPEED:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetS16Speed((SpeednPosFdbk_Handle_t *)&STO_PLL_M${MOTOR_NUM});
              break;
            }

    <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_STOPLL_I_ALPHA:
            {
              *regdata16 = STO_PLL_GetEstimatedCurrent(&STO_PLL_M${MOTOR_NUM}).alpha;
              break;
            }

            case MC_REG_STOPLL_I_BETA:
            {
              *regdata16 = STO_PLL_GetEstimatedCurrent(&STO_PLL_M${MOTOR_NUM}).beta;
              break;
            }
    </#if><#-- MEMORY_FOOTPRINT_REG -->

            case MC_REG_STOPLL_BEMF_ALPHA:
            {
              *regdata16 = STO_PLL_GetEstimatedBemf(&STO_PLL_M${MOTOR_NUM}).alpha;
              break;
            }

            case MC_REG_STOPLL_BEMF_BETA:
            {
              *regdata16 = STO_PLL_GetEstimatedBemf(&STO_PLL_M${MOTOR_NUM}).beta;
              break;
            }

            case MC_REG_STOPLL_C1:
            {
              int16_t hC1;
              int16_t hC2;
              STO_PLL_GetObserverGains(&STO_PLL_M${MOTOR_NUM}, &hC1, &hC2);
              *regdata16 = hC1;
              break;
            }

            case MC_REG_STOPLL_C2:
            {
              int16_t hC1;
              int16_t hC2;
              STO_PLL_GetObserverGains(&STO_PLL_M${MOTOR_NUM}, &hC1, &hC2);
              *regdata16 = hC2;
              break;
            }

            case MC_REG_STOPLL_KI:
            {
              *regdata16 = PID_GetKI (&(&STO_PLL_M${MOTOR_NUM})->PIRegulator);
              break;
            }

            case MC_REG_STOPLL_KP:
            {
              *regdata16 = PID_GetKP (&(&STO_PLL_M${MOTOR_NUM})->PIRegulator);
              break;
            }
  </#if><#-- IS_STO_PLL -->

  <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_CORDIC"]>
            case MC_REG_STOCORDIC_EL_ANGLE:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetElAngle((SpeednPosFdbk_Handle_t *)&STO_CR_M${MOTOR_NUM});
              break;
            }

    <#if MEMORY_FOOTPRINT_REG>
            case MC_REG_STOCORDIC_ROT_SPEED:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetS16Speed((SpeednPosFdbk_Handle_t*) &STO_CR_M${MOTOR_NUM});
              break;
            }

            case MC_REG_STOCORDIC_I_ALPHA:
            {
              *regdata16 = STO_CR_GetEstimatedCurrent(&STO_CR_M${MOTOR_NUM}).alpha;
              break;
            }

            case MC_REG_STOCORDIC_I_BETA:
            {
              *regdata16 = STO_CR_GetEstimatedCurrent(&STO_CR_M${MOTOR_NUM}).beta;
              break;
            }
    </#if><#-- MEMORY_FOOTPRINT_REG -->

            case MC_REG_STOCORDIC_BEMF_ALPHA:
            {
              *regdata16 = STO_CR_GetEstimatedBemf(&STO_CR_M${MOTOR_NUM}).alpha;
              break;
            }

            case MC_REG_STOCORDIC_BEMF_BETA:
            {
              *regdata16 = STO_CR_GetEstimatedBemf(&STO_CR_M${MOTOR_NUM}).beta;
              break;
            }

            case MC_REG_STOCORDIC_C1:
            {
              int16_t hC1;
              int16_t hC2;
              STO_CR_GetObserverGains(&STO_CR_M${MOTOR_NUM}, &hC1, &hC2);
              *regdata16 = hC1;
              break;
            }

            case MC_REG_STOCORDIC_C2:
            {
              int16_t hC1;
              int16_t hC2;
              STO_CR_GetObserverGains(&STO_CR_M${MOTOR_NUM}, &hC1, &hC2);
              *regdata16 = hC2;
              break;
            }
  </#if><#-- IS_STO_CORDIC -->

            case MC_REG_DAC_USER1:
            case MC_REG_DAC_USER2:
              break;

  <#if .vars.MC["M"+ MOTOR_NUM?c + "_FEED_FORWARD_CURRENT_REG_ENABLING"]>
            case MC_REG_FF_VQ:
            {
              *regdata16 = FF_GetVqdff(&FF_M${MOTOR_NUM}).q;
              break;
            }

            case MC_REG_FF_VD:
            {
              *regdata16 = FF_GetVqdff(&FF_M${MOTOR_NUM}).d;
              break;
            }

            case MC_REG_FF_VQ_PIOUT:
            {
              *regdata16 = FF_GetVqdAvPIout(&FF_M${MOTOR_NUM}).q;
              break;
            }

            case MC_REG_FF_VD_PIOUT:
            {
              *regdata16 = FF_GetVqdAvPIout(&FF_M${MOTOR_NUM}).d;
              break;
            }
  </#if><#-- FEED_FORWARD_CURRENT_REG_ENABLING -->


  <#if MC.MOTOR_PROFILER>
            case MC_REG_SC_PWM_FREQUENCY:
              *regdataU16 = SCC_GetPWMFrequencyHz(&SCC);
              break;
            case MC_REG_OVERVOLTAGETHRESHOLD:
            {
              *regdataU16 = SCC_GetOverVoltageThreshold(&SCC);
              break;
            }

            case MC_REG_UNDERVOLTAGETHRESHOLD:
              {
                *regdata16 = SCC_GetUnderVoltageThreshold(&SCC);
              break;
            }
  </#if><#-- MC.MOTOR_PROFILER -->
  <#if .vars.MC["M" + MOTOR_NUM?c + "_POSITION_CTRL_ENABLING"]>
            case MC_REG_POSITION_KP:
            {
              *regdata16 = PID_GetKP( &PID_PosParamsM${MOTOR_NUM});
              break;
            }

            case MC_REG_POSITION_KI:
            {
              *regdata16 = PID_GetKI( &PID_PosParamsM${MOTOR_NUM});
              break;
            }

            case MC_REG_POSITION_KD:
            {
              *regdata16 = PID_GetKD( &PID_PosParamsM${MOTOR_NUM});
              break;
            }
  </#if><#-- POSITION_CTRL_ENABLING -->
  
  <#if MEMORY_FOOTPRINT_REG && ((.vars.MC["M" + MOTOR_NUM?c + "_DRIVE_TYPE"] != "ACIM") || (MC.ACIM_CONFIG == "LSO_FOC"))>
            case MC_REG_SPEED_KP_DIV:
            {
              *regdataU16 = (uint16_t)PID_GetKPDivisorPOW2(&PIDSpeedHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_SPEED_KI_DIV:
            {
              *regdataU16 = (uint16_t)PID_GetKIDivisorPOW2(&PIDSpeedHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_SPEED_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(&PIDSpeedHandle_M${MOTOR_NUM});
              break;
            }
    <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC" || (MC.ACIM_CONFIG == "LSO_FOC")>
            case MC_REG_I_D_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(&PIDIdHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_I_D_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(&PIDIdHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_I_D_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(&PIDIdHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_I_Q_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(&PIDIqHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_I_Q_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(&PIDIqHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_I_Q_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(&PIDIqHandle_M${MOTOR_NUM});
              break;
            }
    </#if><#-- DRIVE_TYPE == "FOC" -->

    <#if .vars.MC["M" + MOTOR_NUM?c + "_POSITION_CTRL_ENABLING"]>
            case MC_REG_POSITION_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(&PID_PosParamsM${MOTOR_NUM});
              break;
            }

            case MC_REG_POSITION_KI_DIV: 
            {
              *regdataU16 = PID_GetKIDivisorPOW2(&PID_PosParamsM${MOTOR_NUM});
              break;
            }

            case MC_REG_POSITION_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(&PID_PosParamsM${MOTOR_NUM});
              break;
            }
    </#if><#-- POSITION_CTRL_ENABLING -->

    <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_PLL"]>
            case MC_REG_STOPLL_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(&(&STO_PLL_M${MOTOR_NUM})->PIRegulator);
              break;
            }

            case MC_REG_STOPLL_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(&(&STO_PLL_M${MOTOR_NUM})->PIRegulator);
              break;
            }
    </#if><#-- IS_STO_PLL -->
  </#if><#-- MEMORY_FOOTPRINT_REG -->

  <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC" && .vars.MC["M"+ MOTOR_NUM?c + "_DBG_OPEN_LOOP_ENABLE"]>
            case MC_REG_FOC_VQREF:
            {
            *regdata16 = ((OL_GetVoltage(&OpenLoop_ParamsM${MOTOR_NUM})*100)/32767);
              break;
            }
  </#if><#-- .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC" && .vars.MC["M"+ MOTOR_NUM?c + "_DBG_OPEN_LOOP_ENABLE"]-->

  <#if MEMORY_FOOTPRINT_REG>
    <#if .vars.MC["M" + MOTOR_NUM?c + "_FLUX_WEAKENING_ENABLING"]>
            case MC_REG_FLUXWK_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(&PIDFluxWeakeningHandle_M${MOTOR_NUM});
              break;
            }

            case MC_REG_FLUXWK_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(&PIDFluxWeakeningHandle_M${MOTOR_NUM});
              break;
            }
    </#if><#--M1_FLUX_WEAKENING_ENABLING-->

    <#if (.vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_ADC") || (.vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_COMP")>
            case MC_REG_PULSE_VALUE:
            {
              *regdataU16 = MCI_GetDutyCycleRef(pMCIN);
              break;
            }  
    </#if><#-- MC.SPEED_SENSOR == "SENSORLESS_ADC" -->
  </#if><#-- MEMORY_FOOTPRINT_REG -->
</#if><#-- MEMORY_FOOTPRINT_REG2 -->

            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 2;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_32BIT:
      {
        uint32_t *regdataU32 = (uint32_t *)data; //cstat !MISRAC2012-Rule-11.3
        int32_t *regdata32 = (int32_t *)data; //cstat !MISRAC2012-Rule-11.3

        if (freeSpace >= 4)
        {
          switch (regID)
          {
            case MC_REG_FAULTS_FLAGS:
            {
              *regdataU32 = MCI_GetFaultState(pMCIN);
              break;
            }
            case MC_REG_SPEED_MEAS:
            {
              *regdata32 = (((int32_t)MCI_GetAvrgMecSpeedUnit(pMCIN) * U_RPM) / SPEED_UNIT);
              break;
            }

            case MC_REG_SPEED_REF:
            { 
              *regdata32 = (((int32_t)MCI_GetMecSpeedRefUnit(pMCIN) * U_RPM) / SPEED_UNIT);
              break;
            }

<#if MEMORY_FOOTPRINT_REG2> 
  <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_PLL"]>
            case MC_REG_STOPLL_EST_BEMF:
            {
              *regdata32 = STO_PLL_GetEstimatedBemfLevel(&STO_PLL_M${MOTOR_NUM});
              break;
            }

            case MC_REG_STOPLL_OBS_BEMF:
            {
              *regdata32 = STO_PLL_GetObservedBemfLevel(&STO_PLL_M${MOTOR_NUM});
              break;
            }
  </#if><#-- IS_STO_PLL -->
  
  <#if .vars["M" + MOTOR_NUM?c + "_IS_STO_CORDIC"]>
            case MC_REG_STOCORDIC_EST_BEMF:
            {
              *regdata32 = STO_CR_GetEstimatedBemfLevel(&STO_CR_M${MOTOR_NUM});
              break;
            }

            case MC_REG_STOCORDIC_OBS_BEMF:
            {
              *regdata32 = STO_CR_GetObservedBemfLevel(&STO_CR_M${MOTOR_NUM});
              break;
            }
  </#if><#-- IS_STO_CORDIC -->
  
  <#if .vars.MC["M" + MOTOR_NUM?c + "_FEED_FORWARD_CURRENT_REG_ENABLING"]>
            case MC_REG_FF_1Q:
            {
              *regdata32 = FF_M${MOTOR_NUM}.wConstant_1Q;
              break;
            }

            case MC_REG_FF_1D:
            {
              *regdata32 = FF_M${MOTOR_NUM}.wConstant_1D;
              break;
            }

            case MC_REG_FF_2:
            {
              *regdata32 = FF_M${MOTOR_NUM}.wConstant_2;
              break;
            }
  </#if><#-- FEED_FORWARD_CURRENT_REG_ENABLING-->

  <#if .vars.MC["M" + MOTOR_NUM?c + "_POSITION_CTRL_ENABLING"]>
            case MC_REG_CURRENT_POSITION:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = MCI_GetCurrentPosition(pMCIN);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }
  </#if><#-- POSITION_CTRL_ENABLING -->
</#if><#-- MEMORY_FOOTPRINT_REG2 -->

<#if (.vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || (.vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "ACIM")><#-- TODO: These registers should also be supported identically in 6S configurations in public releases... and some be made optional -->
            case MC_REG_MOTOR_POWER:
            {
              FloatToU32 ReadVal; //cstat !MISRAC2012-Rule-19.2
              ReadVal.Float_Val = PQD_GetAvrgElMotorPowerW(pMPM[M${MOTOR_NUM}]);
              *regdataU32 = ReadVal.U32_Val; //cstat !UNION-type-punning
              break;
            }
</#if><#-- DRIVE_TYPE == "FOC" -->

<#if MC.MOTOR_PROFILER>
            case MC_REG_SC_RS:
            {
              *regdataU32 = SCC_GetRs(&SCC);
              break;
            }

            case MC_REG_SC_LS:
            {
              *regdataU32 = SCC_GetLs(&SCC);
              break;
            }

            case MC_REG_SC_KE:
            {
              *regdataU32 = SCC_GetKe(&SCC);
              break;
            }

            case MC_REG_SC_VBUS:
            {
              *regdataU32 = SCC_GetVbus(&SCC);
              break;
            }

            case MC_REG_SC_MEAS_NOMINALSPEED:
            {
              *regdataU32 = OTT_GetNominalSpeed(&OTT);
              break;
            }

            case MC_REG_SC_CURRENT:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetNominalCurrent(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_SPDBANDWIDTH:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = OTT_GetSpeedRegulatorBandwidth(&OTT);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_LDLQRATIO:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetLdLqRatio(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_NOMINAL_SPEED:
            {
              *regdata32 = SCC_GetNominalSpeed(&SCC);
              break;
            }

            case MC_REG_SC_CURRBANDWIDTH:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetCurrentBandwidth(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_J:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = OTT_GetJ(&OTT);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_F:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = OTT_GetF(&OTT);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_MAX_CURRENT:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetStartupCurrentAmp(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_STARTUP_SPEED:
            {
              *regdata32 = SCC_GetEstMaxOLSpeed(&SCC);
              break;
            }

            case MC_REG_SC_STARTUP_ACC:
            {
              *regdata32 = SCC_GetEstMaxAcceleration(&SCC);
              break;
            }
            case MC_REG_RESISTOR_OFFSET:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val=SCC_GetResistorOffset(&SCC);
              *regdata32 =ReadVal.U32_Val;
              break;
            }
</#if><#-- MOTOR_PROFILER -->

            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 4;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_STRING:
      {
<#if MEMORY_FOOTPRINT_REG>
        char_t *charData = (char_t *)data;
</#if><#-- MEMORY_FOOTPRINT_REG -->
        switch (regID)
        {
<#if MEMORY_FOOTPRINT_REG>
          case MC_REG_PWR_STAGE_NAME:
          {
            retVal = RI_MovString (PWR_BOARD_NAME[motorID], charData, size, freeSpace);
            break;
          }

          case MC_REG_MOTOR_NAME:
          {
            retVal = RI_MovString (MotorConfig_reg[motorID]->name ,charData, size, freeSpace);
            break;
          }
</#if><#-- MEMORY_FOOTPRINT_REG -->

          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            *size= 0 ; /* */
            break;
          }
        }
        break;
      }

      case TYPE_DATA_RAW:
      {
        /* First 2 bytes of the answer is reserved to the size */
        uint16_t *rawSize = (uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
        uint8_t * rawData = data;
        rawData++;
        rawData++;

        switch (regID)
        {
          case MC_REG_APPLICATION_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(ApplicationConfig_reg_t);
            if (((*rawSize) + 2U) > (uint16_t)freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else 
            {
              ApplicationConfig_reg_t const *pApplicationConfig_reg = ApplicationConfig_reg[motorID];
              (void)memcpy(rawData, (const uint8_t *)pApplicationConfig_reg, sizeof(ApplicationConfig_reg_t));
            }
            break;
          }

  <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_MOTOR_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(MotorConfig_reg_t);
            if (((*rawSize) + 2U) > (uint16_t)freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else 
            { 
              MotorConfig_reg_t const *pMotorConfig_reg = MotorConfig_reg[motorID];
              (void)memcpy(rawData, (const uint8_t *)pMotorConfig_reg, sizeof(MotorConfig_reg_t));
            }
            break;
          }
  </#if><#-- MEMORY_FOOTPRINT_REG -->
          

          case MC_REG_FOCFW_CONFIG:
          {
  <#if (.vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC") || (.vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "ACIM")><#-- TODO: Create an equivalent register for 6S -->
            *rawSize = (uint16_t)sizeof(FOCFwConfig_reg_t);
            if (((*rawSize) + 2U) > (uint16_t)freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else 
            { 
              FOCFwConfig_reg_t const *pFOCConfig_reg = FOCConfig_reg[motorID];
              (void)memcpy(rawData, (const uint8_t *)pFOCConfig_reg, sizeof(FOCFwConfig_reg_t));
            }

  </#if><#-- (DRIVE_TYPE == "FOC") || (DRIVE_TYPE == "ACIM" -->
  <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "SIX_STEP"><#-- TODO: Create an equivalent register for 6S -->
            *rawSize = (uint16_t)sizeof(SixStepFwConfig_reg_t);
            if (((*rawSize) + 2U) > (uint16_t)freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else 
            { 
              SixStepFwConfig_reg_t const *pSixStepConfig_reg = SixStepConfig_reg[motorID];
              (void)memcpy(rawData, (const uint8_t *)pSixStepConfig_reg, sizeof(SixStepFwConfig_reg_t));
            }
  </#if><#-- DRIVE_TYPE == "SIX_STEP" -->
            break;
          }
          case MC_REG_SCALE_CONFIG:
          {
            *rawSize = 12;
            if ((*rawSize) +2U > (uint16_t)freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              memcpy(rawData, &scaleParams_M${MOTOR_NUM}, sizeof(ScaleParams_t) ); 
            }
            break;
          }
          case MC_REG_SPEED_RAMP:
          {
<#if UNALIGNMENT_SUPPORTED >
            int32_t *rpm = (int32_t *)rawData; //cstat !MISRAC2012-Rule-11.3 
            uint16_t *duration = (uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            *rpm = (((int32_t)MCI_GetLastRampFinalSpeed(pMCIN) * U_RPM) / (int32_t)SPEED_UNIT);
<#else><#-- UNALIGNMENT_NOT_SUPPORTED -->
            uint16_t *rpm16p = (uint16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *duration = (uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            int32_t rpm32 = ((int32_t)(MCI_GetLastRampFinalSpeed(pMCIN) * U_RPM) / (int32_t)SPEED_UNIT);
            *rpm16p = (uint16_t)rpm32;
            *(rpm16p+1) = (uint16_t)(rpm32>>16);
</#if><#-- UNALIGNMENT_SUPPORTED -->
            *duration = MCI_GetLastRampFinalDuration(pMCIN);
            *rawSize = 6;
            break;
          }

<#if MEMORY_FOOTPRINT_REG>
  <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC"><#-- TODO: Create an equivalent register for 6S -->
          case MC_REG_TORQUE_RAMP:
          {
            int16_t *torque = (int16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *duration = (uint16_t *)&rawData[2]; //cstat !MISRAC2012-Rule-11.3
            
            *rawSize = 4;
            *torque = MCI_GetLastRampFinalTorque(pMCIN);
            *duration = MCI_GetLastRampFinalDuration(pMCIN) ;
            break;
          }
  </#if><#-- DRIVE_TYPE == "FOC" -->
  <#if .vars["M" + MOTOR_NUM?c + "_IS_SENSORLESS"] || (.vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_ADC")|| (.vars.MC["M" + MOTOR_NUM?c + "_SPEED_SENSOR"] == "SENSORLESS_COMP")>

          case MC_REG_REVUP_DATA:
          {
    <#if UNALIGNMENT_SUPPORTED>
            int32_t *rpm;
    <#else><#-- UNALIGNMENT_NOT_SUPPORTED -->
            int32_t rpm32;
            int16_t *rpm16p;
    </#if><#-- UNALIGNMENT_SUPPORTED -->
    <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC">
            uint16_t *finalTorque;
    </#if><#-- DRIVE_TYPE == "FOC" -->
    <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "SIX_STEP">
            uint16_t *finalPulse;
    </#if><#-- DRIVE_TYPE == "SIX_STEP" -->    
            uint16_t *durationms;
            RevUpCtrl_PhaseParams_t revUpPhase;
            uint8_t i;

            *rawSize = (uint16_t)RUC_MAX_PHASE_NUMBER*8U;
            if (((*rawSize) + 2U) > (uint16_t)freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              for (i = 0; i <RUC_MAX_PHASE_NUMBER; i++)
              {
                (void)RUC_GetPhase( &RevUpControlM${MOTOR_NUM} ,i, &revUpPhase);
    <#if UNALIGNMENT_SUPPORTED>
                rpm = (int32_t *)&data[2U + (i * 8U)];  //cstat !MISRAC2012-Rule-11.3
                *rpm = (((int32_t)revUpPhase.hFinalMecSpeedUnit) * U_RPM) / SPEED_UNIT; //cstat !MISRAC2012-Rule-11.3
    <#else><#-- UNALIGNMENT_NOT_SUPPORTED -->
                rpm32 = (((int32_t)revUpPhase.hFinalMecSpeedUnit) * U_RPM) / SPEED_UNIT; //cstat !MISRAC2012-Rule-11.3
                rpm16p = (int16_t *)&data[2U + (i * 8U)]; //cstat !MISRAC2012-Rule-11.3
                *rpm16p = (uint16_t)rpm32; /* 16 LSB access */ //cstat !MISRAC2012-Rule-11.3
                *(rpm16p+1) = ((uint16_t)(rpm32 >> 16)); /* 16 MSB access */ //cstat !MISRAC2012-Rule-11.3
    </#if><#-- UNALIGNMENT_SUPPORTED -->
    <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC">
                finalTorque = (uint16_t *)&data[6U + (i * 8U)]; //cstat !MISRAC2012-Rule-11.3
                *finalTorque = (uint16_t)revUpPhase.hFinalTorque; //cstat !MISRAC2012-Rule-11.3
    </#if><#-- DRIVE_TYPE == "FOC" -->
    <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "SIX_STEP">
                finalPulse = (uint16_t *)&data[6U + (i * 8U)]; //cstat !MISRAC2012-Rule-11.3
                *finalPulse = (uint16_t)revUpPhase.hFinalPulse; //cstat !MISRAC2012-Rule-11.3
    </#if><#-- DRIVE_TYPE == "SIX_STEP" -->
                durationms  = (uint16_t *)&data[8U + (i * 8U)]; //cstat !MISRAC2012-Rule-11.3
                *durationms  = revUpPhase.hDurationms;
              }
            }
            break;
          }

    <#if SIX_STEP >
          case MC_REG_BEMF_ADC_CONF:
          {
            Bemf_Sensing_Params bemfAdcConfig;
            Bemf_Demag_Params bemfAdcDemagConfig;
            uint16_t ZcRising2CommDelay,ZcFalling2CommDelay,OnSensingEnThres, OnSensingDisThres, computationdelayChoice;
            uint16_t *ConvertedData;

            (void)BADC_GetBemfSensorlessParam(&Bemf_ADC_M1, &bemfAdcConfig,&ZcRising2CommDelay,&ZcFalling2CommDelay,&bemfAdcDemagConfig,
                                              &OnSensingEnThres,&OnSensingDisThres,&computationdelayChoice);
            ConvertedData = (uint16_t *)rawData;  //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = bemfAdcConfig.AdcThresholdHighPerc;

            ConvertedData = (uint16_t *)&rawData[2]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = bemfAdcConfig.AdcThresholdPwmPerc;

            ConvertedData = (uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = bemfAdcConfig.AdcThresholdLowPerc;

            ConvertedData = (uint16_t *)&rawData[6]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = (uint16_t)((100 * bemfAdcConfig.SamplingPointOff) / PWM_PERIOD_CYCLES) + 1U;

            ConvertedData = (uint16_t *)&rawData[8]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = (uint16_t)((100 * bemfAdcConfig.SamplingPointOn) / PWM_PERIOD_CYCLES) + 1U;

            ConvertedData = (uint16_t *)&rawData[10]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = ((ZcRising2CommDelay * 0.12) < 1) ? 1 : (uint16_t)(ZcRising2CommDelay * 0.12);

            ConvertedData = (uint16_t *)&rawData[12]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = ((ZcFalling2CommDelay * 0.12) < 1) ? 1 : (uint16_t)(ZcFalling2CommDelay * 0.12);

            ConvertedData = (uint16_t *)&rawData[14]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = bemfAdcDemagConfig.DemagMinimumThreshold;

            ConvertedData = (uint16_t *)&rawData[16]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = (uint16_t) ((bemfAdcDemagConfig.DemagMinimumSpeedUnit * U_RPM) / SPEED_UNIT) ;

            ConvertedData = (uint16_t *)&rawData[18]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = (uint16_t)((100 * OnSensingEnThres) / PWM_PERIOD_CYCLES) + 1U;

            ConvertedData = (uint16_t *)&rawData[20]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = (uint16_t)((100 * OnSensingDisThres) / PWM_PERIOD_CYCLES) + 1U;

            ConvertedData = (uint16_t *)&rawData[22]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = bemfAdcConfig.AWDfiltering;

            ConvertedData = (uint16_t *)&rawData[24]; //cstat !MISRAC2012-Rule-11.3
            *ConvertedData = computationdelayChoice;

            *rawSize = 26;

            break;
          }
    </#if><#-- DRIVE_TYPE == "SIX_STEP" -->
  </#if><#-- IS_SENSORLESS || MC.SPEED_SENSOR == "SENSORLESS_ADC" -->

  <#if .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC">
          case MC_REG_CURRENT_REF:
          {
            uint16_t *iqref = (uint16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *idref = (uint16_t *)&rawData[2]; //cstat !MISRAC2012-Rule-11.3

            *rawSize = 4;
            *iqref = (uint16_t)MCI_GetIqdref(pMCIN).q;
            *idref = (uint16_t)MCI_GetIqdref(pMCIN).d;
            break;
          }
  </#if><#-- DRIVE_TYPE == "FOC" -->
  
  <#if .vars.MC["M"+ MOTOR_NUM?c + "_POSITION_CTRL_ENABLING"]>
          case MC_REG_POSITION_RAMP:
          {
            float Position;
            float Duration;

            *rawSize = 8;
            Position = TC_GetMoveDuration(&PosCtrlM${MOTOR_NUM});   /* Does this duration make sense ? */
            Duration = TC_GetTargetPosition(&PosCtrlM${MOTOR_NUM});
            (void)memcpy(rawData, &Position, 4);
            (void)memcpy(&rawData[4], &Duration, 4);
            break;
          }
  </#if><#-- POSITION_CTRL_ENABLING -->
</#if><#-- MEMORY_FOOTPRINT_REG -->  
<#if (MC.MOTOR_PROFILER == true) && .vars.MC["M"+ MOTOR_NUM?c + "_DRIVE_TYPE"] == "FOC" && .vars.MC["M" + MOTOR_NUM?c + "_AUXILIARY_SPEED_SENSOR"] == "HALL_SENSOR">
          case MC_REG_HT_HEW_PINS:
            {
              *rawSize = 4;
              *rawData = HT.bNewH1;
              *(rawData+1) = HT.bNewH2;
              *(rawData+2) = HT.bNewH3;
              *(rawData+3) = (uint8_t) 0U; /* Padding Raw structure must be 16 bits aligned */
              break;
            }
            
          case MC_REG_HT_CONNECTED_PINS:
            {
              *rawSize = 4;
              *rawData = HT.H1Connected;
              *(rawData+1) = HT.H2Connected;
              *(rawData+2) = HT.H3Connected;
              *(rawData+3) = (uint8_t) 0U; /* Padding Raw structure must be 16 bits aligned */
              break;
            }
            
          case MC_REG_HT_PHASE_SHIFT:
            {
              int16_t *rawData16 = (int16_t *) rawData;
              *rawSize = 28; /*14 16 bits values */
              *rawData16 = HT.hPhaseShiftCircularMean;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean5_1;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean1_3;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean3_2;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean2_6;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean6_4;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean4_5;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMeanNeg;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean5_4;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean4_6;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean6_2;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean2_3;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean3_1;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean1_5;
              break;
            }
</#if> <#-- MOTOR_PROFILER && DRIVE_TYPE == "FOC" && _AUXILIARY_SPEED_SENSOR == "HALL_SENSOR" -->

          case MC_REG_ASYNC_UARTA:
          case MC_REG_ASYNC_UARTB:
          case MC_REG_ASYNC_STLNK:
          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }

        /* Size of the answer is size of the data + 2 bytes containing data size */
        *size = (*rawSize) + 2U;
        break;
      }

      default:
      {
        retVal = MCP_ERROR_BAD_DATA_TYPE;
        break;
      }
    }
    return (retVal);
  }
</#macro>

<#list 1..(MC.DRIVE_NUMBER?number) as NUM>

   <@GetRegFct MOTOR_NUM=NUM />
  
</#list>

<#if MEMORY_FOOTPRINT_REG>
uint8_t RI_MovString(const char_t *srcString, char_t *destString, uint16_t *size, int16_t maxSize)
{
  uint8_t retVal = MCP_CMD_OK;
  const char_t *tempsrcString = srcString;
  char_t *tempdestString = destString;
  *size= 1U ; /* /0 is the min String size */

  while ((*tempsrcString != (char_t)0) && (*size < (uint16_t)maxSize))
  {
    *tempdestString = *tempsrcString;
    tempdestString++;
    tempsrcString++;
    *size = *size + 1U;
  }
  
  if (*tempsrcString != (char_t)0)
  { /* Last string char must be 0 */
    retVal = MCP_ERROR_STRING_FORMAT;
  }
  else
  {
    *tempdestString = (int8_t)0;
  }
  return (retVal);
}
</#if><#-- MEMORY_FOOTPRINT_REG -->

uint8_t RI_GetIDSize(uint16_t dataID)
{
  uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;
  uint8_t result;

  switch (typeID)
  {
    case TYPE_DATA_8BIT:
    {
      result = 1;
      break;
    }

    case TYPE_DATA_16BIT:
    {
      result = 2;
      break;
    }

    case TYPE_DATA_32BIT:
    {
      result = 4;
      break;
    }

    default:
    {
      result=0;
      break;
    }
  }
  
  return (result);
}

<#macro GetPtrReg MOTOR_NUM>

  uint8_t retVal = MCP_CMD_OK;
  static uint16_t nullData16=0;

#ifdef NULL_PTR_CHECK_REG_INT  
  if (MC_NULL == dataPtr)
  {
    retVal = MCP_CMD_NOK;
  }
  else
  {
#endif

    MCI_Handle_t *pMCIN = &Mci[${MOTOR_NUM-1}];
    uint16_t regID = dataID & REG_MASK;
    uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;

    switch (typeID)
    {
      case TYPE_DATA_16BIT:
      {
        switch (regID)
        {
<#if FOC || ACIM>
  <#if MEMORY_FOOTPRINT_REG>
          case MC_REG_I_A:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iab.a);
             break;
          }

          case MC_REG_I_B:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iab.b);
            break;
          }

          case MC_REG_I_ALPHA_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Ialphabeta.alpha);
            break;
          }

          case MC_REG_I_BETA_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Ialphabeta.beta);
            break;
          }

          case MC_REG_I_Q_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqd.q);
            break;
          }

          case MC_REG_I_D_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqd.d);
            break;
          }

          case MC_REG_I_Q_REF:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqdref.q);
            break;
          }

          case MC_REG_I_D_REF:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqdref.d);
            break;
          }
  </#if><#-- MEMORY_FOOTPRINT_REG -->
  
          case MC_REG_V_Q:
          {
            *dataPtr = &(pMCIN->pFOCVars->Vqd.q);
            break;
          }

          case MC_REG_V_D:
          {
            *dataPtr = &(pMCIN->pFOCVars->Vqd.d);
            break;
          }

          case MC_REG_V_ALPHA:
          {
            *dataPtr = &(pMCIN->pFOCVars->Valphabeta.alpha);
            break;
          }

          case MC_REG_V_BETA:
          {
            *dataPtr = &(pMCIN->pFOCVars->Valphabeta.beta);
            break;
          }
</#if><#-- FOC -->


<#if SIX_STEP>
          case MC_REG_PULSE_VALUE:
          {
            *dataPtr = &(pMCIN->pSixStepVars->DutyCycleRef);
            break;
          }
</#if><#-- SIX_STEP -->

<#if MEMORY_FOOTPRINT_REG>
  <#if .vars["M"+ MOTOR_NUM?c + "_IS_HALL_SENSOR"]>
          case MC_REG_HALL_SPEED:
          {
            *dataPtr = &((&HALL_M${MOTOR_NUM})->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_HALL_EL_ANGLE:
          {
            *dataPtr = &((&HALL_M${MOTOR_NUM})->_Super.hElAngle);
            break;
          }
  </#if><#-- IS_HALL_SENSOR -->
</#if><#-- MEMORY_FOOTPRINT_REG -->

<#if .vars["M"+ MOTOR_NUM?c + "_IS_ENCODER"]>
          case MC_REG_ENCODER_SPEED:
          {
            *dataPtr = &((&ENCODER_M${MOTOR_NUM})->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_ENCODER_EL_ANGLE:
          {
            *dataPtr = &((&ENCODER_M${MOTOR_NUM})->_Super.hElAngle);
            break;
          }
</#if><#-- IS_ENCODER -->

<#if .vars["M"+ MOTOR_NUM?c + "_IS_STO_PLL"]>
          case MC_REG_STOPLL_ROT_SPEED:
          {
            *dataPtr = &((&STO_PLL_M${MOTOR_NUM})->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_STOPLL_EL_ANGLE:
          {
            *dataPtr = &((&STO_PLL_M${MOTOR_NUM})->_Super.hElAngle);
            break;
          }
          
#ifdef NOT_IMPLEMENTED /* Not yet implemented */
          case MC_REG_STOPLL_I_ALPHA:
          case MC_REG_STOPLL_I_BETA:
            break;
#endif

          case MC_REG_STOPLL_BEMF_ALPHA:
          {
            *dataPtr = &((&STO_PLL_M${MOTOR_NUM})->hBemf_alfa_est);
            break;
          }

          case MC_REG_STOPLL_BEMF_BETA:
          {
            *dataPtr = &((&STO_PLL_M${MOTOR_NUM})->hBemf_beta_est);
            break;
          }
</#if><#-- IS_STO_PLL -->

<#if .vars["M"+ MOTOR_NUM?c + "_IS_STO_CORDIC"]>
          case MC_REG_STOCORDIC_ROT_SPEED:
          {
            *dataPtr = &((&STO_CR_M${MOTOR_NUM})->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_STOCORDIC_EL_ANGLE:
          {
            *dataPtr = &((&STO_CR_M${MOTOR_NUM})->_Super.hElAngle);
            break;
          }
#ifdef NOT_IMPLEMENTED /* Not yet implemented */
          case MC_REG_STOCORDIC_I_ALPHA:
          case MC_REG_STOCORDIC_I_BETA:
            break;
#endif
          case MC_REG_STOCORDIC_BEMF_ALPHA:
          {
            *dataPtr = &((&STO_CR_M${MOTOR_NUM})->hBemf_alfa_est);
            break;
          }

          case MC_REG_STOCORDIC_BEMF_BETA:
          {
            *dataPtr = &((&STO_CR_M${MOTOR_NUM})->hBemf_beta_est);
            break;
          }
</#if><#-- IS_STO_CORDIC -->

          default:
          {
            *dataPtr = &nullData16;
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
        break;
      }

      default:
      {
        *dataPtr = &nullData16;
        retVal = MCP_ERROR_UNKNOWN_REG;
        break;
      }
    }
#ifdef NULL_PTR_CHECK_REG_INT
  }  
#endif
  return (retVal);
</#macro>

<#if (MC.DRIVE_NUMBER?number) gt 1>
__weak uint8_t RI_GetPtrReg(uint16_t dataID, void **dataPtr)
{
  uint8_t retVal;
  
  uint8_t motorID = (uint8_t)((dataID & MOTOR_MASK) - 1U);
  uint8_t (*GetPtrRegFcts[NBR_OF_MOTORS])(uint16_t, void**) = {&RI_GetPtrRegMotor1<#list 2..(MC.DRIVE_NUMBER?number) as NUM>, &RI_GetPtrRegMotor${NUM}</#list>};

  retVal = GetPtrRegFcts[motorID](dataID, dataPtr);

  return(retVal); 
}
<#list 1..(MC.DRIVE_NUMBER?number) as NUM>
__weak uint8_t RI_GetPtrRegMotor${NUM}(uint16_t dataID, void **dataPtr)
{
   <@GetPtrReg MOTOR_NUM=NUM />
}
</#list>

<#else><#-- MC.DRIVE_NUMBER <= 1 -->
__weak uint8_t RI_GetPtrReg(uint16_t dataID, void **dataPtr)
{
   <@GetPtrReg MOTOR_NUM=1 />
}
</#if><#-- MC.DRIVE_NUMBER > 1 -->
/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
