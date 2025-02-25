<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    flash_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file includes the type definition of data aimed to be written by the application.
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FLASH_PARAMETERS_H
#define FLASH_PARAMETERS_H
#include "mc_type.h"
#include "mc_configuration_registers.h"

typedef struct
{
  float zestThresholdFreqHz; // MOTOR_ZEST_THRESHOLD_FREQ_HZ
  float zestInjectFreq; // MOTOR_ZEST_INJECT_FREQ
  float zestInjectD; // MOTOR_ZEST_INJECT_D
  float zestInjectQ; // MOTOR_ZEST_INJECT_Q
  float zestGainD; // MOTOR_ZEST_GAIN_D
  float zestGainQ; //MOTOR_ZEST_GAIN_Q
} zestFlashParams_t; 
  
typedef struct
{
  float pidSpdKp; //PID_SPD_KP
  float pidSpdKi; //PID_SPD_KI
} PIDSpeedFlashParams_t;  
      
typedef struct
{
  float voltage;
  float current;
  float frequency;
  float padding [1];
} scaleFlashParams_t; // 3 useful words + padding

typedef struct
{
  uint16_t        N;                   
  uint16_t        Nd;                  
  float           PulseCurrentGoal;
  float padding [1];
} polPulseFlashParams_t; // 2 useful words + 2 padding

typedef struct
{
  float limitOverVoltage;
  float limitRegenHigh;
  float limitRegenLow;
  float limitAccelHigh;
  float limitAccelLow;
  float limitUnderVoltage;
  float maxModulationIndex;
  float softOverCurrentTrip;
  float padding [1];
} boardFlashParams_t; // 8 useful words + padding

/** 
 * @brief Size of the significant part of the #FLASH_Params_t structure in bytes
 *
 * The #FLASH_Params_t structure is instanciated once per motor in the application.
 * Its start address must be aligned on a FLASH page border and its size needs to 
 * be a multiple of this FLASH page size. This is needed so that it can be erased 
 * and overwritten without impacting the rest of the application. 
 *
 * Actually, its size does not really need to be that big. But the there must be 
 * nothing else in the same FLASH page as this structure. The only way to ensure 
 * that without modifying the linker configuration is to align the structure on a 
 * FLASH page size and to make the strucure big enough. 
 *
 * However, the #FLASH_Params_t is also meant to be transferred across the MCP, as
 * a register. But, the MCP limist the size of the registers values it can transmit.
 * And the typical limit is quite inferior to a FLASH page size. 
 *
 * This symbol is used by the MCP to only transfer that portion of the structure that
 * contains significatn data. Hoperfully, this size fits into MCP limits...
 */
#define FLASH_PARAMS_SIZE (  sizeof( MotorConfig_reg_t )     \
                           + sizeof( zestFlashParams_t )     \
                           + sizeof( PIDSpeedFlashParams_t ) \
                           + sizeof( boardFlashParams_t )    \
                           + sizeof( float )                 \
                           + sizeof( throttleParams_t )      \
                           + sizeof( scaleFlashParams_t )    \
                           + sizeof( polPulseFlashParams_t ) )

typedef __attribute__((aligned(FLASH_PAGE_SIZE))) struct
{
  MotorConfig_reg_t motor;
  zestFlashParams_t zest;
  PIDSpeedFlashParams_t PIDSpeed; 
  boardFlashParams_t board;
  float KSampleDelay;
  throttleParams_t throttle;
  scaleFlashParams_t scale;
  polPulseFlashParams_t polPulse;
  uint8_t padding[ FLASH_PAGE_SIZE - FLASH_PARAMS_SIZE ];
} FLASH_Params_t;

#endif // FLASH_PARAMETERS_H