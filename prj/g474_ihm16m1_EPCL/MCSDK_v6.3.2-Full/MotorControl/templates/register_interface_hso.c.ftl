<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
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
#include "stdint.h"
#include "string.h"
#include "register_interface.h"
#include "mc_config.h"
#include "mc_parameters.h"
#include "mcp.h"
#include "mcp_config.h"
#include "mcpa.h"
#include "mc_configuration_registers.h"
#include "impedcorr.h"
#include "mc_curr_ctrl.h"
#include "speed_pos_fdbk_hso.h"
#include "speed_torq_ctrl_hso.h"
//#include "dac_ui.h"

#define RSDC_ESTIMATION_SHIFT     0
#define DYNAMIC_ZEST_CONFIG_SHIFT 1
#define OFFSET_MEASUREMENTS_SHIFT 2
#define POLE_PULSE_ACTIVATION_SHIFT 3
#define RSDC_ESTIMATION_FLAG     (1 << RSDC_ESTIMATION_SHIFT)
#define DYNAMIC_ZEST_CONFIG_FLAG (1 << DYNAMIC_ZEST_CONFIG_SHIFT)
#define OFFSET_MEASUREMENTS_FLAG (1 << OFFSET_MEASUREMENTS_SHIFT)
#define POLE_PULSE_ACTIVATION_FLAG (1 << POLE_PULSE_ACTIVATION_SHIFT)

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

  <#if MC.DEBUG_DAC_FUNCTIONALITY_EN>
      uint16_t regdata16 = *(uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
  </#if><#-- MC.DEBUG_DAC_FUNCTIONALITY_EN -->

      switch (regID) 
      {
         
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

<#if MC.PFC_ENABLED>
        case MC_REG_PFC_DCBUS_REF:
        case MC_REG_PFC_DCBUS_MEAS:
        case MC_REG_PFC_ACBUS_FREQ:
        case MC_REG_PFC_ACBUS_RMS:
        case MC_REG_PFC_I_KP:
        case MC_REG_PFC_I_KI:
        case MC_REG_PFC_I_KD:
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

          case MC_REG_GLOBAL_CONFIG:
          case MC_REG_APPLICATION_CONFIG:
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

uint8_t RI_SetRegisterMotor1(uint16_t regID, uint8_t typeID, uint8_t *data, uint16_t *size, int16_t dataAvailable)
{
  uint8_t motorID = 0;
  uint8_t retVal = MCP_CMD_OK;
  motorID = 0;

  MCI_Handle_t * pMCI = &Mci[motorID];

  switch (typeID)
  {
  case TYPE_DATA_8BIT:
    {
    uint8_t regdata8 = *data;

    switch (regID)
    {
    case MC_REG_STATUS:
      retVal = MCP_ERROR_RO_REG;
      break;
    case MC_REG_FOC_CONTROLMODE:
      MCI_SetControlMode(pMCI, (MC_ControlMode_t) (regdata8));
        break;
    case MC_REG_FOC_MODULATIONMODE:
      {
        uint8_t mode = regdata8;
        mode = mode < numFOC_MODULATIONMODE ? mode : FOC_MODULATIONMODE_Sine;
        pMCI->pPWM->modulationMode = (FOC_ModulationMode_e) mode;
        MCI_SetMaxModulation(pMCI, FIXP30(BOARD_MAX_MODULATION));
      }
        break;
    case MC_REG_FOC_SPEED_SOURCE_SELECT:
      {
        uint8_t mode = regdata8;
        mode = mode < numFOC_SPEED_SOURCE ? mode : FOC_SPEED_SOURCE_Default;
        pMCI->pSTC->speedref_source = (FOC_SPEED_Source_e)mode;
      }
        break;
    case MC_REG_FOC_RS_UPDATE_SELECT:
      {
        uint8_t mode = regdata8;
        mode = mode < numFOC_Rs_update_Select ? mode : FOC_Rs_update_None;
        pMCI->pSPD->Rs_update_select = (FOC_Rs_update_Select_e)mode;
      }
        break;
    case MC_REG_OVS_SAMPLESELECT:
      oversampling.singleIdx = regdata8;
        break;
    case MC_REG_FOC_CURRENTRECONSTRUCTION:
      pMCI->pPWM->flagEnableCurrentReconstruction = (regdata8 != 0);
        break;
<#if MC.MOTOR_PROFILER == true>        
    case MC_REG_PROFILER_COMMAND:
      MCI_SetProfilerCommand(pMCI, regdata8);
        break;
</#if>        
    default:
      retVal = MCP_ERROR_UNKNOWN_REG;
    }
    *size = 1;
    break;
    }
  case TYPE_DATA_16BIT:
    {
    uint16_t regdata16 = *(uint16_t *)data;
    switch (regID)
    {
    case MC_REG_FOC_CURRENTRECONSTRUCTION_LIMIT: /*TODO: Hardcoded to Motor 1 !*/
      {
         fixp15_t perc = regdata16;
         float_t headroom = 1.0f - FIXP15_toF(perc);
         PWM_Handle_M1.maxTicksCurrentReconstruction = (uint16_t) (PWM_Handle_M1.Half_PWMPeriod * (1.0f - (headroom / 2.0f)));
         break;
      }
    case MC_REG_START_CONFIGURATION:
      {
        pMCI->pRsDCEst->flag_enableRSDCestimate  = regdata16 & RSDC_ESTIMATION_FLAG;
        pMCI->pFOCVars->flag_enableOffsetMeasurement  = regdata16 & OFFSET_MEASUREMENTS_FLAG;
        pMCI->pSPD->flagDynamicZestConfig = regdata16 & DYNAMIC_ZEST_CONFIG_FLAG;
        pMCI->pPolPulse->flagPolePulseActivation = (regdata16 & POLE_PULSE_ACTIVATION_FLAG) >> POLE_PULSE_ACTIVATION_SHIFT;
        pMCI->pRsDCEst->RSDCestimate_Fast = (pMCI->pRsDCEst->flag_enableRSDCestimate) && (pMCI->pPolPulse->flagPolePulseActivation);
        break;
      }
    case MC_REG_DAC_OUT1:
      {
        //DAC_SetChannelConfig(&DAC_Handle , DAC_CH1, regdata16);
        break;
       }
    case MC_REG_FOC_PULSE_DURATION:
      {
        MCI_SetPulsePeriods(pMCI, regdata16);
        break;
      }
    case MC_REG_FOC_PULSE_DECAY_DURATION:
      {
        MCI_SetDecayDuration(pMCI, regdata16);
        break;
      }
    case MC_REG_HEATS_TEMP:
      {
        retVal = MCP_ERROR_RO_REG;
        break;
      }
      
    default:
      retVal = MCP_ERROR_UNKNOWN_REG;

    }
    *size = 2;
    }
    break;

  case TYPE_DATA_32BIT:
    {
    uint32_t regdata32 = *(uint32_t *)data;
    switch (regID)
    {
    case MC_REG_FAULTS_FLAGS:
      break;
    case MC_REG_FOC_OPENLOOP_ANGLE:
      pMCI->pSPD->OpenLoopAngle =  regdata32;
        break;
    case MC_REG_FOC_VDREF:
      pMCI->pCurrCtrl->Ddq_ref_pu.D = regdata32;
        break;
    case MC_REG_I_Q:
    case MC_REG_I_D:
      retVal = MCP_ERROR_RO_REG;
        break;
    case MC_REG_FOC_TORQUE_REF_NM:
      MCI_SetTorqueReference(pMCI, regdata32);
        break;
    case MC_REG_RS: /* DEPRECATED */
        break;
    case MC_REG_LS: /* DEPRECATED */
        break;
    case MC_REG_FOC_KP_IDQ:
      {
        CONVERT_u convert;
        convert.u32 = regdata32;
        float_t Kp_si_flt = convert.flt;
        MCI_SetPidCurrentKp(pMCI, Kp_si_flt);
      }
      break;
    case MC_REG_FOC_WI_IDQ:
      {
        CONVERT_u convert;
        convert.u32 = regdata32;
        float_t Wi_si_flt = convert.flt;
        MCI_SetPidCurrentWi(pMCI, Wi_si_flt);
      }
        break;
    case MC_REG_FOC_KP_SPD:
      {
        CONVERT_u convert;
        convert.u32 = regdata32;
        float_t Kp_si_flt = convert.flt;
        MCI_SetPidSpeedKp(pMCI, Kp_si_flt);
      }
        break;
    case MC_REG_FOC_KI_SPD:
      {
        CONVERT_u convert;
        convert.u32 = regdata32;
        float_t Ki_si_flt = convert.flt;
        MCI_SetPidSpeedKi(pMCI, Ki_si_flt);
      }
        break;
    case MC_REG_FOC_SPEEDREF_FREQ_PU:
      MCI_SetSpeedReference_PU( pMCI, regdata32 );
        break;
    case MC_REG_FOC_SPEEDREF_RAMPED_FREQ_PU:
      pMCI->pSTC->speed_ref_ramped_pu = regdata32;
        break;
    case MC_REG_FOC_SPEEDRAMP:
      MCI_SetSpeedRamp(pMCI, regdata32);
        break;
    case MC_REG_FOC_TORQUERAMP:
        break;
<#if MC.M1_SPEED_SENSOR == "ZEST">      
    case MC_REG_FOC_ID_INJECT_A:
      pMCI->pSPD->zestControl.injectId_A_pu = FIXP16_mpy(regdata32, FIXP30(1.0f / scaleParams->current));
        break;
    case MC_REG_FOC_INJECTFREQ_HZ:
      pMCI->pSPD->zestControl.injectFreq_kHz = FIXP16_mpy(regdata32, FIXP30(0.001));
        break;
    case MC_REG_FOC_GAIN_D:
      pMCI->pSPD->zestControl.feedbackGainD = regdata32;
        break;
    case MC_REG_FOC_GAIN_Q:
      pMCI->pSPD->zestControl.feedbackGainQ = regdata32;
        break;    
<#else>       
    case MC_REG_FOC_ID_INJECT_A:
    case MC_REG_FOC_INJECTFREQ_HZ:
    case MC_REG_FOC_GAIN_D:
    case MC_REG_FOC_GAIN_Q:
    retVal = MCP_ERROR_UNKNOWN_REG;
        break;  
</#if>           
    case MC_REG_FOC_RS_EST: /* DEPRECATED */
    case MC_REG_FOC_LS_EST:
        break;
    case MC_REG_FOC_USER_CURRENT_LIMIT:
      MCI_SetMaxCurrent(pMCI, regdata32);
        break;
    case MC_REG_FOC_FLUX_RATED_VPHZ:
      MCI_SetRatedFlux(pMCI, regdata32);
        break;
    case MC_REG_FOC_RS_RATED:
        /* MCI updates RsEst */
        MCI_SetRatedRs(pMCI, regdata32);
        MCI_SetActiveRs(pMCI, regdata32);
        /* This updates current PID settings */
        MCI_SetCurrentPIDByRsLs(pMCI, FIXP20_toF(regdata32), FIXP24_toF(MCI_GetRatedLs(pMCI)));
        break;
    case MC_REG_FOC_RS_ACTIVE:
      MCI_SetActiveRs(pMCI, regdata32);
        break;
    case MC_REG_FOC_LS_RATED:
        MCI_SetRatedLs(pMCI, regdata32);
        MCI_SetActiveLs(pMCI, regdata32);
        // ToDo Review SetReg FOC_LS_RATED
        MCI_SetCurrentPIDByRsLs(pMCI, FIXP20_toF(MCI_GetRatedRs(pMCI)), FIXP24_toF(regdata32));
        break;
    case MC_REG_FOC_LS_ACTIVE:
      MCI_SetActiveLs(pMCI, regdata32);
        break;
    case MC_REG_MAXMODULATION:
        /* fixp30_t, up to 1.15, =~ 2/sqrt(3) */
        MCI_SetMaxModulation(pMCI, regdata32);
        break;
<#if MC.M1_SPEED_SENSOR == "ZEST">         
    case MC_REG_FOC_ZEST_OUTD:
      retVal = MCP_ERROR_RO_REG;
        break;
    case MC_REG_FOC_ZEST_OUTQ:
      retVal = MCP_ERROR_RO_REG;
        break;
</#if>        
    case MC_REG_FOC_ANGLE_COMP_FACTOR:
      pMCI->pCurrCtrl->angle_compensation_factor = regdata32;
        break;
    case MC_REG_FOC_MAX_ACC: /* Seems to be not implemented*/
      retVal = MCP_ERROR_UNKNOWN_REG;
        break;
    case MC_REG_FOC_IDREF:
      MCI_SetIdRef(pMCI, regdata32);
        break;
    case MC_REG_FOC_IQREF:
      MCI_SetIqRef(pMCI, regdata32);
        break;
    case MC_REG_MOTOR_POLEPAIRS:
      {
        CONVERT_u convert;
        convert.u32 = regdata32;
        MCI_SetPolePairs(pMCI, convert.flt);
      }
      break;

      case MC_REG_FOC_KSAMPLE_DELAY:
      {
        CONVERT_u convert;
        convert.u32 = regdata32;
        IMPEDCORR_setKSampleDelay(pMCI->pSPD->pImpedCorr, convert.flt);
      }
      break; 
      
<#if MC.MOTOR_PROFILER == true>      
    case MC_REG_PROFILER_POWERGOAL_W:
      {
        CONVERT_u convert;
        convert.u32 = regdata32;
        MCI_SetProfilerPowerGoal_W(pMCI, convert.flt);
      }
      break;
    case MC_REG_PROFILER_FLUXESTFREQ_HZ:
      {
        CONVERT_u convert;
        convert.u32 = regdata32;
        MCI_SetProfilerFluxEstFreq_Hz(pMCI, convert.flt);
      }
      break;
</#if>  
    case MC_REG_FOC_PULSE_CURRENT_GOAL:
      {
        CONVERT_u convert;
        convert.u32 = regdata32;
        MCI_SetPulseCurrentGoal(pMCI, convert.flt);
      }  
      break;     

    case MC_REG_FOC_SKINFACTOR:
      {
<#if MC.M1_SPEED_SENSOR == "ZEST">  
        CONVERT_u convert;
        convert.u32 = regdata32;
        RSEST_setSkinFactor(pMCI->pSPD->pRsEst, convert.flt);
</#if>  
      }
      break;  
    default:
      retVal = MCP_ERROR_UNKNOWN_REG;
    }
    *size = 4;
    break;
    }
  case TYPE_DATA_STRING:
    {
      const char_t *charData = (const char_t *) data;
      char_t *dummy = (char_t *) data ;
      retVal = MCP_ERROR_RO_REG;
      /* Used to compute String length stored in RXBUFF even if Reg does not exist*/
      /* It allows to jump to the next command in the buffer */
      RI_MovString (charData, dummy, size, dataAvailable);
    }
    break;
  case TYPE_DATA_RAW:
      {
        uint16_t rawSize = *(uint16_t *) data; //cstat !MISRAC2012-Rule-11.3
        /* The size consumed by the structure is the structure size + 2 bytes used to store the size*/
        *size = rawSize + 2U;
        uint8_t *rawData = data; /* rawData points to the first data (after size extraction) */
        rawData++;
        rawData++;

        if (*size > dataAvailable )
        { /* The decoded size of the raw structure can not match with transmitted buffer, error in buffer construction*/
          *size = 0;
          retVal = MCP_ERROR_BAD_RAW_FORMAT; /* this error stop the parsing of the CMD buffer */
        }
        else
        {
          switch (regID)
          {
            case MC_REG_GLOBAL_CONFIG:
            case MC_REG_APPLICATION_CONFIG:
            case MC_REG_MOTOR_CONFIG:
            case MC_REG_FOCFW_CONFIG:
            {
              retVal = MCP_ERROR_RO_REG;
              break;
            }
            case MC_REG_ASYNC_UARTA:
            {
              retVal =  MCPA_cfgLog (&MCPA_UART_A, rawData);
              break;
            }
            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
        }
      }
    break;
  default:
    retVal = MCP_ERROR_BAD_DATA_TYPE;
    *size =0; /* From this point we are not able anymore to decode the RX buffer*/
  }
  return retVal;
}

uint8_t RI_GetRegisterMotor1 (uint16_t regID, uint8_t typeID, uint8_t * data, uint16_t *size, int16_t freeSpace)
{
  uint8_t motorID = 0;
  uint8_t retVal = MCP_CMD_OK;

  MCI_Handle_t * pMCI = &Mci[motorID];
  switch (typeID)
  {
  case TYPE_DATA_8BIT:
    {
      if (freeSpace > 0 )
      {
        switch (regID)
        {
        case MC_REG_STATUS:
          *data = MCI_GetSTMState(pMCI);
          break;
        case MC_REG_FOC_CONTROLMODE:
           *data = MCI_GetFocControlMode(pMCI);
           break;
        case MC_REG_FOC_CONTROLMODE_NEW:
          *data = MCI_GetControlMode(pMCI);
          break;
        case MC_REG_FOC_MODULATIONMODE:
          *data = pMCI->pPWM->modulationMode;
          break;
        case MC_REG_FOC_SPEED_SOURCE_SELECT:
          *data = pMCI->pSTC->speedref_source;
          break;
        case MC_REG_FOC_RS_UPDATE_SELECT:
          *data = pMCI->pSPD->Rs_update_select;
          break;
        case MC_REG_OVS_SAMPLESELECT:
          *data = oversampling.singleIdx;
          break;
        case MC_REG_FOC_CURRENTRECONSTRUCTION:
          *data = (pMCI->pPWM->flagEnableCurrentReconstruction ? 1 : 0);
          break;
<#if MC.MOTOR_PROFILER == true>          
        case MC_REG_PROFILER_COMMAND:
          retVal = MCP_ERROR_WO_REG;
          break;
        case MC_REG_PROFILER_STATE:
          *data = MCI_GetProfilerState(pMCI);
          break;
</#if>          
        default:
          retVal = MCP_ERROR_UNKNOWN_REG;
        }
        *size = 1;
      }
      else
      {
        retVal = MCP_ERROR_NO_TXSYNC_SPACE;
      }
    }
    break;
  case TYPE_DATA_16BIT:
    {
      // uint16_t * regdataU16 = (uint16_t *)data;
      int16_t * regdata16 = (int16_t *) data;
      if (freeSpace >= 2 )
      {
        switch (regID)
        {
        case MC_REG_FOC_CURRENTRECONSTRUCTION_LIMIT: /* TODO: HARDECODED to Motor 1*/
          {
            uint32_t ticks = PWM_Handle_M1.maxTicksCurrentReconstruction;
            uint32_t halfperiod = PWM_Handle_M1.Half_PWMPeriod;
            uint32_t quarterperiod = halfperiod >> 1;
            float_t limit = (ticks - quarterperiod) / (float_t) quarterperiod;
            *regdata16 = FIXP15(limit);
          }
          break;
        case MC_REG_START_CONFIGURATION:
          {
            uint16_t startFlags;

            startFlags = pMCI->pRsDCEst->flag_enableRSDCestimate << RSDC_ESTIMATION_SHIFT;
            startFlags |= pMCI->pFOCVars->flag_enableOffsetMeasurement << OFFSET_MEASUREMENTS_SHIFT;
            startFlags |= pMCI->pSPD->flagDynamicZestConfig << DYNAMIC_ZEST_CONFIG_SHIFT;
            startFlags |= pMCI->pPolPulse->flagPolePulseActivation << POLE_PULSE_ACTIVATION_SHIFT;
            *regdata16 = startFlags;
          }
          break;
         case MC_REG_FOC_PULSE_DURATION:
           {
             *regdata16 = MCI_GetPulsePeriods(pMCI);
             break;
           }
         case MC_REG_FOC_PULSE_DECAY_DURATION:
           {
             *regdata16 = MCI_GetDecayDuration(pMCI);
             break;
           }

         case MC_REG_HEATS_TEMP:
           {
             *regdata16 = NTC_GetAvTemp_C(&TempSensor_M1);
             break;
           }
            
        default:
          retVal = MCP_ERROR_UNKNOWN_REG;
        }
        *size = 2;
      }
      else
      {
        retVal = MCP_ERROR_NO_TXSYNC_SPACE;
      }
    }
    break;
  case TYPE_DATA_32BIT:
    {
      uint32_t *regdataU32 = (uint32_t *) data;
      int32_t *regdata32 = (int32_t *) data;
      if ( freeSpace >= 4)
      {
        switch (regID)
        {
        case MC_REG_FAULTS_FLAGS:
          *regdataU32 = MCI_GetFaultState(pMCI);
          break;
        case MC_REG_ADC_I_U:
          *regdata32 = (int32_t) pMCI->pPWM->Irst_in_pu.R;
          break;
        case MC_REG_ADC_I_V:
          *regdata32 = (int32_t) pMCI->pPWM->Irst_in_pu.S;
          break;
        case MC_REG_ADC_I_W:
          *regdata32 = (int32_t) pMCI->pPWM->Irst_in_pu.T;
          break;
        case MC_REG_ADC_U_U:
          *regdata32 = (int32_t) pMCI->pPWM->Urst_in_pu.R;
          break;
        case MC_REG_ADC_U_V:
          *regdata32 = (int32_t) pMCI->pPWM->Urst_in_pu.S;
          break;
        case MC_REG_ADC_U_W:
          *regdata32 = (int32_t) pMCI->pPWM->Urst_in_pu.T;
          break;
        case MC_REG_ADC_U_DC:
          *regdata32 = (int32_t) pMCI->pVBus->Udcbus_in_pu;
          break;
        case MC_REG_ADC_I_U_OFFSET:
          {
            PolarizationOffsets_t CalibIUrstOffsets;
            if (MCI_GetCalibratedOffsetsMotor( pMCI, &CalibIUrstOffsets) == MC_SUCCESS)
              {
          *regdata32 = CalibIUrstOffsets.CurrentOffsets.R;
              }
          }
        break;
        case MC_REG_ADC_I_V_OFFSET:
          {
            PolarizationOffsets_t CalibIUrstOffsets;
            if (MCI_GetCalibratedOffsetsMotor( pMCI, &CalibIUrstOffsets) == MC_SUCCESS)
              {
          *regdata32 = CalibIUrstOffsets.CurrentOffsets.S;
              }
          }
        break;
        case MC_REG_ADC_I_W_OFFSET:
          {
            PolarizationOffsets_t CalibIUrstOffsets;
            if (MCI_GetCalibratedOffsetsMotor( pMCI, &CalibIUrstOffsets) == MC_SUCCESS)
              {
          *regdata32 = CalibIUrstOffsets.CurrentOffsets.T;
              }
          }
        break;
        case MC_REG_ADC_U_U_OFFSET:
          {
            PolarizationOffsets_t CalibIUrstOffsets;
            if (MCI_GetCalibratedOffsetsMotor( pMCI, &CalibIUrstOffsets) == MC_SUCCESS)
              {
                 *regdata32 = CalibIUrstOffsets.VoltageOffsets.R;
              }
          }
        break;
        case MC_REG_ADC_U_V_OFFSET:
          {
            PolarizationOffsets_t CalibIUrstOffsets;
            if (MCI_GetCalibratedOffsetsMotor( pMCI, &CalibIUrstOffsets) == MC_SUCCESS)
              {
                 *regdata32 = CalibIUrstOffsets.VoltageOffsets.S;
              }
          }
        break;
        case MC_REG_ADC_U_W_OFFSET:
          {
            PolarizationOffsets_t CalibIUrstOffsets;
            if (MCI_GetCalibratedOffsetsMotor( pMCI, &CalibIUrstOffsets) == MC_SUCCESS)
              {
                 *regdata32 = CalibIUrstOffsets.VoltageOffsets.T;
              }
          }
        break;
        case MC_REG_HSO_FLUX_A:
          *regdata32 = pMCI->pSPD->Flux_ab.A ;
            break;
        case MC_REG_HSO_FLUX_B:
          *regdata32 = pMCI->pSPD->Flux_ab.B ;
            break;
        case MC_REG_FOC_OPENLOOP_ANGLE:
            *regdata32 = pMCI->pSPD->OpenLoopAngle;
          break;
        case MC_REG_FOC_VDREF:
          *regdata32 = pMCI->pCurrCtrl->Ddq_ref_pu.D;
          break;
        case MC_REG_I_Q:
          *regdata32 = MCI_GetCurrent_PU(pMCI).Q;
          break;
        case MC_REG_I_D:
          *regdata32 = MCI_GetCurrent_PU(pMCI).D;
          break;
        case MC_REG_FOC_TORQUE_REF_NM:
          *regdata32 = MCI_GetTorqueReference(pMCI);
          break;
<#if MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true>
        case MC_REG_THROTTLE_ADC:
          *regdata32 = pMCI->pPotentiometer->adcVal;
          break;
        case MC_REG_THROTTLE_POS:
          *regdata32 = pMCI->pPotentiometer->position;
           break;
</#if><#-- MC.M1_POTENTIOMETER_ENABLE == true || MC.M2_POTENTIOMETER_ENABLE == true -->
        case MC_REG_RS:
          /* Deprecated, returns the active resistance for now */
          *regdata32 = MCI_GetActiveRs(pMCI) << 4; // REG_RS in fixp24_t, Rs_Active in fixp20_t
          break;
        case MC_REG_LS:
          /* Deprecated, returns the active inductance for now */
          *regdata32 = MCI_GetActiveLs(pMCI) << 6; // REG_LS in fixp30_t, Ls_Active in fixp24_t
          break;
        case MC_REG_MECHANICAL_SPEED_RPM:
          {
            CONVERT_u convert;
            convert.flt  = FIXP30_toF(HSO_getEmfSpeed_pu(pMCI->pSPD->pHSO)) * 60.0f * scaleParams->frequency / pMCI->pFOCVars->polePairs;
            *regdataU32 = convert.u32;
          }
          break;

        case MC_REG_FOC_KP_IDQ:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetPidCurrentKp(pMCI);
            *regdataU32 = convert.u32;
          }
          break;
        case MC_REG_FOC_WI_IDQ:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetPidCurrentWi(pMCI);
            *regdataU32 = convert.u32;
          }
        break;
        case MC_REG_FOC_KP_SPD:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetPidSpeedKp(pMCI);
            *regdataU32 = convert.u32;
          }
        break;
        case MC_REG_FOC_KI_SPD:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetPidSpeedKi(pMCI);
            *regdataU32 = convert.u32;
          }
          break;
        case MC_REG_FOC_SPEEDREF_FREQ_PU:
          *regdata32 = MCI_GetSpeedReference_PU(pMCI);
          break;
        case MC_REG_FOC_SPEEDREF_RAMPED_FREQ_PU:
          *regdata32 = pMCI->pSTC->speed_ref_ramped_pu;
          break;
        case MC_REG_FOC_SPEEDRAMP:
          *regdata32 = MCI_GetSpeedRamp(pMCI);
           break;
        case MC_REG_FOC_TORQUERAMP:
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        case MC_REG_FOC_TORQUE_NM:
          *regdata32 = MCI_GetTorque_Nm(pMCI);
          break;
        case MC_REG_FOC_FELEC_HZ:
          *regdata32 = MCI_GetAvrgSpeed(pMCI);
          break;
<#if MC.M1_SPEED_SENSOR == "ZEST">           
        case MC_REG_FOC_ID_INJECT_A:
          *regdata32 = FIXP30_mpy(pMCI->pSPD->zestControl.injectId_A_pu, FIXP16(scaleParams->current));
          break;
        case MC_REG_FOC_INJECTFREQ_HZ:
          *regdata32 = FIXP30_mpy(ZEST_getInjectFreqkHz(pMCI->pSPD->pZeST), FIXP16(1000.0f)); /* bRetVal = FIXP16(Hz) */
          break;
        case MC_REG_FOC_GAIN_D:
          *regdata32 = ZEST_getGainD(pMCI->pSPD->pZeST);
          break;
        case MC_REG_FOC_GAIN_Q:
          *regdata32 = ZEST_getGainQ(pMCI->pSPD->pZeST);
          break;
        case MC_REG_FOC_ZEST_OUTD:
          *regdata32 = pMCI->pSPD->zestOutD;
          break;
        case MC_REG_FOC_ZEST_OUTQ:
          *regdata32 = pMCI->pSPD->zestOutQ;
          break;      
<#else>       
    case MC_REG_FOC_ID_INJECT_A:
    case MC_REG_FOC_INJECTFREQ_HZ:
    case MC_REG_FOC_GAIN_D:
    case MC_REG_FOC_GAIN_Q:
    case MC_REG_FOC_ZEST_OUTD:
    case MC_REG_FOC_ZEST_OUTQ:
         *regdata32 = 0;
         retVal = MCP_ERROR_UNKNOWN_REG;
        break;                
</#if>                            
        case MC_REG_FOC_RS_EST:
          /* Deprecated, fixp16_t Ohm */
          *regdata32 = MCI_GetActiveRs(pMCI) >> (20 - 16);
          break;
        case MC_REG_FOC_USER_CURRENT_LIMIT:
          *regdata32 = MCI_GetMaxCurrent(pMCI);
          break;
        case MC_REG_FOC_LS_EST:
          /* Deprecated, fixp30_t Henry */
          *regdata32 = MCI_GetActiveLs(pMCI) << (30 - 24);
          break;
        case MC_REG_FOC_FLUX_RATED_VPHZ:
          *regdata32 = MCI_GetRatedFlux(pMCI);
          break;
        case MC_REG_FOC_FLUX_VPHZ:
          *regdata32 = MCI_GetActiveFlux(pMCI);
          break;
        case MC_REG_FOC_DUTYCYCLE_D:
          *regdata32 = pMCI->pCurrCtrl->Ddq_out_LP_pu.D;
          break;
        case MC_REG_FOC_DUTYCYCLE_Q:
          *regdata32 = pMCI->pCurrCtrl->Ddq_out_LP_pu.Q;
          break;
        case MC_REG_FOC_ANGLE_PU:
          *regdata32 = pMCI->pCurrCtrl->angle_park_pu;
          break;
        case MC_REG_FOC_RS_RATED:
          *regdata32 = MCI_GetRatedRs(pMCI);
          break;
        case MC_REG_FOC_RS_ACTIVE:
          *regdata32 = MCI_GetActiveRs(pMCI);
          break;
        case MC_REG_FOC_LS_RATED: /* TODO: Check why set and get in the same CMD ? */
          {
            fixp24_t ls_rated_fixp24 = MCI_GetRatedLs(pMCI);
            pMCI->pFOCVars->Ls_Rated_H = ls_rated_fixp24;
            *regdata32 = ls_rated_fixp24;
          }
          break;
        case MC_REG_FOC_LS_ACTIVE: /* TODO: Check why set and get in the same CMD ? */
          {
            fixp24_t ls_active_fixp24 = MCI_GetActiveLs(pMCI);
            pMCI->pFOCVars->Ls_Active_H = ls_active_fixp24;
            *regdata32 = ls_active_fixp24;
          }
          break;
        case MC_REG_MAXMODULATION:
          *regdata32 = MCI_GetMaxModulation(pMCI);
          break;
        case MC_REG_FOC_ANGLE_COMP_FACTOR:
          *regdata32 = pMCI->pCurrCtrl->angle_compensation_factor;
          break;
        case MC_REG_FOC_MAX_ACC:
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        case MC_REG_FOC_IQ_REF_SPEED:
          *regdata32 = MCI_GetIqRefSpeed(pMCI);
          break;
        case MC_REG_FOC_IDREF:
          *regdata32 = MCI_GetIdRef(pMCI);
          break;
        case MC_REG_FOC_IQREF:
          *regdata32 = MCI_GetIqRef(pMCI);
          break;
        case MC_REG_MOTOR_POLEPAIRS:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetPolePairs(pMCI);
            *regdataU32 = convert.u32;
          }
          break;
          
        case MC_REG_FOC_KSAMPLE_DELAY:
        {
           CONVERT_u convert;
           convert.flt = IMPEDCORR_getKSampleDelay(pMCI->pSPD->pImpedCorr);
           *regdata32 = convert.u32;
        }
        break; 
<#if MC.MOTOR_PROFILER == true>          
        case MC_REG_PROFILER_RS_DC:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetProfilerRs_DC(pMCI);
            *regdataU32 = convert.u32;
          }
          break;
        case MC_REG_PROFILER_RS_AC:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetProfilerRs_AC(pMCI);
            *regdataU32 = convert.u32;
          }
          break;
        case MC_REG_PROFILER_LD_H:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetProfilerLd_H(pMCI);
            *regdataU32 = convert.u32;
          }
          break;
        case MC_REG_PROFILER_FLUX_WB:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetProfilerFlux_Wb(pMCI);
            *regdataU32 = convert.u32;
          }
          break;
        case MC_REG_PROFILER_POWERGOAL_W:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetProfilerPowerGoal_W(pMCI);
            *regdataU32 = convert.u32;
          }
          break;
        case MC_REG_PROFILER_FLUXESTFREQ_HZ:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetProfilerFluxEstFreq_Hz(pMCI);
            *regdataU32 = convert.u32;
          }
          break;
</#if>          
        case MC_REG_FOC_PULSE_CURRENT_GOAL:
          {
            CONVERT_u convert;
            convert.flt = MCI_GetPulseCurrentGoal(pMCI);
            *regdataU32 = convert.u32;
          }
          break;
        case MC_REG_FOC_PULSE_DUTY:
          {
            *regdata32 = MCI_GetPulseDuty(pMCI);
          }
          break;
        case MC_REG_FOC_PULSE_EST_ANGLE:
          {
            *regdataU32 = MCI_GetPulseAngleEstimated(pMCI);
          }
          break;
        case MC_REG_RSEST_RSOHM:
          {
            CONVERT_u convert;
<#if MC.M1_SPEED_SENSOR == "ZEST">                
            convert.flt = RSEST_getRsOhm(pMCI->pSPD->pRsEst);
<#else>            
            convert.flt = 0.0f;
</#if>            
            *regdataU32 = convert.u32;
          }
          break;
        case MC_REG_FOC_SKINFACTOR:
          {
            CONVERT_u convert;
<#if MC.M1_SPEED_SENSOR == "ZEST">          
            convert.flt = FIXP_toF(RSEST_getSkinFactor(pMCI->pSPD->pRsEst));
<#else>  
            convert.flt = 0.0f;
</#if>     
            *regdataU32 = convert.u32;
          }
          break; 
        case MC_REG_RSEST_RSPOWEROHM:
          {
            CONVERT_u convert;
<#if MC.M1_SPEED_SENSOR == "ZEST">
            convert.flt = RSEST_getRsPowerOhm(pMCI->pSPD->pRsEst);
<#else>            
            convert.flt = 0.0f;
</#if>  
            *regdataU32 = convert.u32;
          }
          break;
        case MC_REG_FOC_ZEST_CORRECTION:
          {
<#if MC.M1_SPEED_SENSOR == "ZEST">
            *regdataU32 = ZEST_getAngleIncrement(pMCI->pSPD->pZeST);
<#else>  
            *regdataU32 = 0UL;
</#if>   
          }
          break;
        default:
          retVal = MCP_ERROR_UNKNOWN_REG;
        }
        *size = 4;
      }
      else
      {
        retVal = MCP_ERROR_NO_TXSYNC_SPACE;
      }
    }
    break;
  case TYPE_DATA_STRING:
    {
      char_t *charData = (char_t *) data;
      switch (regID)
      {
      case MC_REG_FW_NAME:
        retVal = RI_MovString (FIRMWARE_NAME ,charData, size, freeSpace);
        break;
      case MC_REG_CTRL_STAGE_NAME:
        retVal = RI_MovString (CTL_BOARD ,charData, size, freeSpace);
        break;
      case MC_REG_PWR_STAGE_NAME:
        retVal = RI_MovString (PWR_BOARD_NAME[motorID] ,charData, size, freeSpace);
        break;
      case MC_REG_MOTOR_NAME:
        retVal = RI_MovString ( motorParams->name ,charData, size, freeSpace);
        break;
      default:
        retVal = MCP_ERROR_UNKNOWN_REG;
        *size= 0 ; /* */
      }
    }
    break;
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
            if (((*rawSize) + 2U) > freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              (void)memcpy(rawData, &globalConfig_reg, sizeof(GlobalConfig_reg_t));
            }
            break;
          }
          
          case MC_REG_APPLICATION_CONFIG:
          {
            *rawSize = sizeof(ApplicationConfig_reg_t);
            if ((*rawSize) +2  > freeSpace) 
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else 
            { 
              memcpy(rawData, ApplicationConfig_reg[motorID], sizeof(ApplicationConfig_reg_t));
            }
            break;
          }  
           
          case MC_REG_MOTOR_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(MotorConfig_reg_t);
            if (((*rawSize) + 2U) > freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              MotorConfig_reg_t const *pMotorConfig_reg = MotorConfig_reg[motorID];
              (void)memcpy(rawData, (uint8_t *)pMotorConfig_reg, sizeof(MotorConfig_reg_t));
            }
          }
      break;

          case MC_REG_FOCFW_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(FOCFwConfig_reg_t);
            if (((*rawSize) + 2U) > freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              FOCFwConfig_reg_t const *pFOCConfig_reg = FOCConfig_reg[motorID];
              (void)memcpy(rawData, (uint8_t *)pFOCConfig_reg, sizeof(FOCFwConfig_reg_t));
            }
            break;
          }
      case MC_REG_SCALE_CONFIG:
        *rawSize = 12; 
        if ((*rawSize) +2  > freeSpace)
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        else
        {
          memcpy(rawData, scaleParams, 12 );  /* Copy just Voltage, Current and frequency scale*/
        }
      break;

      case MC_REG_ADC_OFFSET:
          *rawSize = 4*6; /* 6 int32 values */
          if ((*rawSize) +2  > freeSpace)
          {
            retVal = MCP_ERROR_NO_TXSYNC_SPACE;
          }
          else
            {
              PolarizationOffsets_t CalibIUrstOffsets;
        if (MCI_GetCalibratedOffsetsMotor( pMCI, &CalibIUrstOffsets) == MC_SUCCESS)
        {
    uint32_t *rawData32 = (uint32_t *)rawData;
    rawData32[0] = CalibIUrstOffsets.CurrentOffsets.R;
    rawData32[1] = CalibIUrstOffsets.CurrentOffsets.S;
    rawData32[2] = CalibIUrstOffsets.CurrentOffsets.T;
    rawData32[3] = CalibIUrstOffsets.VoltageOffsets.R;
    rawData32[4] = CalibIUrstOffsets.VoltageOffsets.S;
    rawData32[5] = CalibIUrstOffsets.VoltageOffsets.T;
        }
        else
        {
          /*Prepare Error response in case of reading offset failure. */
    retVal = MCP_ERROR_REGISTER_ACCESS;
        }
            }
        break;
      case MC_REG_ASYNC_UARTA:
      case MC_REG_ASYNC_UARTB:
      case MC_REG_ASYNC_STLNK:
      default:
        retVal = MCP_ERROR_UNKNOWN_REG;
      }

      /* Size of the answer is size of the data + 2 bytes containing data size*/
      *size = (*rawSize)+2;
    }
    break;

  default:
    retVal = MCP_ERROR_BAD_DATA_TYPE;
    break;
  }
  return retVal;
}

uint8_t RI_MovString (const char_t * srcString, char_t * destString, uint16_t *size, int16_t maxSize)
{
  uint8_t retVal = MCP_CMD_OK;
  *size= 1 ; /* /0 is the min String size */
  while ((*srcString != 0) && (*size < maxSize) )
  {
    *destString = *srcString ;
    srcString = srcString+1;
    destString = destString+1;
    *size=*size+1;
  }
  if (*srcString != 0)
  { /* Last string char_t must be 0 */
    retVal = MCP_ERROR_STRING_FORMAT;
  }
  else
  {
    *destString = 0;
  }
return retVal;
}

uint8_t RI_GetIDSize (uint16_t dataID)
{
  uint8_t typeID = dataID & TYPE_MASK;
  uint8_t result;
  switch (typeID)
  {
    case TYPE_DATA_8BIT:
      result = 1;
      break;
    case TYPE_DATA_16BIT:
      result = 2;
      break;
    case TYPE_DATA_32BIT:
      result = 4;
      break;
    default:
      result=0;
      break;
  }
  return result;
}

uint8_t RI_GetRegisterGlobal(uint16_t regID,uint8_t typeID,uint8_t * data,uint16_t *size,int16_t freeSpace)
{
  uint8_t retVal = MCP_CMD_OK;

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
  <#if MC.DEBUG_DAC_FUNCTIONALITY_EN>
     int16_t *regdata16 = (int16_t *) data; //cstat !MISRAC2012-Rule-11.3
  </#if><#-- MC.DEBUG_DAC_FUNCTIONALITY_EN -->
      if (freeSpace >= 2)
      {
        switch (regID) 
        {
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

<#if MC.PFC_ENABLED>
          case MC_REG_PFC_DCBUS_REF:
          case MC_REG_PFC_DCBUS_MEAS:
          case MC_REG_PFC_ACBUS_FREQ:
          case MC_REG_PFC_ACBUS_RMS:
          case MC_REG_PFC_I_KP:
          case MC_REG_PFC_I_KI:
          case MC_REG_PFC_I_KD:
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
<#if MC.DBG_MCU_LOAD_MEASURE == true>      
      uint32_t *regdataU32 = (uint32_t *)data; //cstat !MISRAC2012-Rule-11.3
      MCI_Handle_t * pMCI = &Mci[0];
</#if>        
      if (freeSpace >= 4)
      {
        switch (regID)
        {
<#if MC.DBG_MCU_LOAD_MEASURE == true>
          case MC_REG_PERF_CPU_LOAD:
          {
            CONVERT_u ReadVal;
            ReadVal.flt = MC_Perf_GetCPU_Load(pMCI->pPerfMeasure);
            *regdataU32 = ReadVal.u32;
          }
          break;
          case MC_REG_PERF_MIN_CPU_LOAD:
          {
            CONVERT_u ReadVal;
            ReadVal.flt = MC_Perf_GetMinCPU_Load(pMCI->pPerfMeasure);
            *regdataU32 = ReadVal.u32;
          }
          break;
          case MC_REG_PERF_MAX_CPU_LOAD:
          {
            CONVERT_u ReadVal;
            ReadVal.flt = MC_Perf_GetMaxCPU_Load(pMCI->pPerfMeasure);
            *regdataU32 = ReadVal.u32;
          }
          break;
</#if>

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
      char_t *charData = (char_t *)data;
      switch (regID)
      {
        case MC_REG_FW_NAME:
          retVal = RI_MovString (FIRMWARE_NAME ,charData, size, freeSpace);
          break;

        case MC_REG_CTRL_STAGE_NAME:
        {
          retVal = RI_MovString (CTL_BOARD ,charData, size, freeSpace);
          break;
        }

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

__weak uint8_t RI_GetPtrReg (uint16_t dataID, void ** dataPtr)
{
  uint8_t typeID = dataID & TYPE_MASK;
  uint8_t motorID = (dataID & MOTOR_MASK)-1;
  uint16_t regID = dataID & REG_MASK;
  uint8_t retVal = MCP_CMD_OK;

  MCI_Handle_t * pMCI = &Mci[motorID];

  switch (typeID)
  {
  case TYPE_DATA_32BIT:
    {
      switch (regID)
      {
      case MC_REG_ADC_I_U:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Irst_in_pu.R)) + 2UL);
        break;
      case MC_REG_ADC_I_V:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Irst_in_pu.S)) + 2UL);
        break;
      case MC_REG_ADC_I_W:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Irst_in_pu.T)) + 2UL);
        break;
      case MC_REG_ADC_U_U:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Urst_in_pu.R)) + 2UL);
        break;
      case MC_REG_ADC_U_V:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Urst_in_pu.S)) + 2UL);
        break;
      case MC_REG_ADC_U_W:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Urst_in_pu.T)) + 2UL);
        break;
      case MC_REG_I_ALPHA:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Iab_in_pu.A)) + 2UL);
        break;
      case MC_REG_I_BETA:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Iab_in_pu.B)) + 2UL);
        break;
      case MC_REG_U_ALPHA:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Uab_in_pu.A)) + 2UL);
        break;
      case MC_REG_U_BETA:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Uab_in_pu.B)) + 2UL);
        break;
      case MC_REG_I_D:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pCurrCtrl->Idq_in_pu.D)) + 2UL);
        break;
      case MC_REG_I_Q:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pCurrCtrl->Idq_in_pu.Q)) + 2UL);
        break;
      case MC_REG_U_D:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pCurrCtrl->Udq_in_pu.D)) + 2UL);
        break;
      case MC_REG_U_Q:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pCurrCtrl->Udq_in_pu.Q)) + 2UL);
        break;
      case MC_REG_ADC_I_RAW_U:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Irst_in_raw_pu.R)) + 2UL);
        break;
      case MC_REG_ADC_I_RAW_V:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Irst_in_raw_pu.S)) + 2UL);
        break;
      case MC_REG_ADC_I_RAW_W:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Irst_in_raw_pu.T)) + 2UL);
        break;
      case MC_REG_FE_HSO:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pSPD->Fe_hso_pu)) + 2UL);
        break;
<#if MC.M1_SPEED_SENSOR == "ZEST">  
      case MC_REG_FOC_ZEST_CORRECTION:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pSPD->ZestcorrectionShifted)) + 2UL);
        break;
      case MC_REG_ANG_SPEED:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pSPD->SpeedLP_pu)) + 2UL);
        break;
      case MC_REG_RSEST_RSOHM:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pSPD->RsEstimated)) + 2UL);
        break;
      case MC_REG_ZEST_CORR_LP:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pSPD->CheckDir)) + 2UL);
        break;
      case MC_REG_DEMOD_EQDQRIP_D:    
        *dataPtr = (void *)((uint32_t)(&(pMCI->pSPD->qEQdqrip_D)) + 2UL);
        break; 
      case MC_REG_DEMOD_EQDQRIP_Q:    
        *dataPtr = (void *)((uint32_t)(&(pMCI->pSPD->qEQdqrip_Q)) + 2UL);
        break; 
      case MC_REG_ZEST_FRACTION:    
        *dataPtr = (void *)((uint32_t)(&(pMCI->pSPD->fraction)) + 2UL);
        break;	    
</#if>      
      case MC_REG_HSO_FLUX_A:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pSPD->Flux_ab.A)) + 2UL);
        break;
      case MC_REG_HSO_FLUX_B:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pSPD->Flux_ab.B)) + 2UL);
        break;
      case MC_REG_FOC_ANGLE_PU:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pCurrCtrl->angle_park_pu)) + 2UL);
        break;
      case MC_REG_FOC_DUTYCYCLE_R:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Drst_out_pu.R)) + 2UL);
        break;
      case MC_REG_FOC_DUTYCYCLE_S:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Drst_out_pu.S)) + 2UL);
        break;
      case MC_REG_FOC_DUTYCYCLE_T:
        *dataPtr = (void *)((uint32_t)(&(pMCI->pPWM->Drst_out_pu.T)) + 2UL);
        break;
      default:
        break;
      }
    }
      break;
    default:

      break;
  }

  return retVal;
}



uint8_t flashWrite_cb (uint16_t rxLength, uint8_t *rxBuffer, int16_t txFreeSpace, uint16_t *txLength, uint8_t *txBuffer)
{
  uint64_t data;
  FLASH_EraseInitTypeDef flashEraseInit;
  HAL_StatusTypeDef status;
  uint32_t PageError;
  uint32_t paramsAddress = (uint32_t) &flashParams;
  uint8_t i,result;

  flashEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  flashEraseInit.Banks = FLASH_BANK_1;
  flashEraseInit.Page = ((uint32_t)&flashParams-FLASH_BASE) /(2*1024);
  flashEraseInit.NbPages = 1;
  status = HAL_FLASH_Unlock();
  status |= HAL_FLASHEx_Erase(&flashEraseInit, &PageError);
  for (i=0; i<rxLength; i=i+8)
  {
    memcpy (&data,&rxBuffer[i],8);
    status |= HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, paramsAddress+i, data);
  }
  status |= HAL_FLASH_Lock();
  if (status == HAL_OK)
  {
    *txLength = 0;
    result = MCP_CMD_OK;
  }
  else
  {
    *txLength = 4;
    *((uint32_t *) txBuffer) = HAL_FLASH_GetError();
    result = MCP_CMD_NOK;
  }
  return result;
}

uint8_t flashRead_cb (uint16_t rxLength, uint8_t *rxBuffer, int16_t txFreeSpace, uint16_t *txLength, uint8_t *txBuffer)
{
  uint8_t result = MCP_CMD_OK;
  
  if (FLASH_PARAMS_SIZE <= txFreeSpace)
  {
    memcpy (txBuffer, &flashParams, FLASH_PARAMS_SIZE);
    *txLength = FLASH_PARAMS_SIZE;
  }
  else
  {
    *txBuffer = MCP_ERROR_NO_TXSYNC_SPACE;
    *txLength = 1;
    result = MCP_ERROR_NO_TXSYNC_SPACE;
  }
  return result;
}

