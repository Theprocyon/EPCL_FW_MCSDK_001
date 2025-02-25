<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    register_interface.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware registers definitions used by MCP protocol
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


#ifndef REGISTER_INTERFACE_H
#define REGISTER_INTERFACE_H
#include "mcp.h"
#include "mc_type.h"

/*
   MCP_ID definition :
   | Element Identifier 10 bits  |  Type  | Motor #|
   |                             |        |        |
   |15|14|13|12|11|10|09|08|07|06|05|04|03|02|01|00|

   Type definition :
   0	Reserved
   1	8-bit data
   2	16-bit data
   3	32-bit data
   4	Character string
   5	Raw Structure
   6	Reserved
   7	Reserved
*/
#define MCP_ID_SIZE 2 /* Number of byte */
#define ELT_IDENTIFIER_POS 6
#define TYPE_POS 3
#define TYPE_MASK 0x38
#define MOTOR_MASK 0x7
#define REG_MASK 0xFFF8

#define MOTORID(dataID) ((dataID & MOTOR_MASK)-1)

#define TYPE_DATA_8BIT    (1 << TYPE_POS)
#define TYPE_DATA_16BIT   (2 << TYPE_POS)
#define TYPE_DATA_32BIT   (3 << TYPE_POS)
#define TYPE_DATA_STRING  (4 << TYPE_POS)
#define TYPE_DATA_RAW     (5 << TYPE_POS)

/* TYPE_DATA_8BIT registers definition */
//#define  MC_REG_TARGET_MOTOR           ((0  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_STATUS                    ((1  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_FOC_CONTROLMODE           ((2  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_FOC_CONTROLMODE_NEW       ((3  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_FOC_MODULATIONMODE        ((4  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_DEBUGPIN_1                ((5  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_DEBUGPIN_2                ((6  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_FOC_SPEED_SOURCE_SELECT   ((7  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_FOC_RS_UPDATE_SELECT      ((8  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_OVS_SAMPLESELECT          ((9  << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_FOC_CURRENTRECONSTRUCTION ((10 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_PROFILER_COMMAND          ((11 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )
#define MC_REG_PROFILER_STATE            ((12 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT )


/* TYPE_DATA_16BIT registers definition */
#define MC_REG_SPEED_KP                		    ((1   << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define MC_REG_FOC_CURRENTRECONSTRUCTION_LIMIT	((2   << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define MC_REG_START_CONFIGURATION		        ((3   << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define MC_REG_DAC_OUT1				            ((4   << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define MC_REG_DAC_OUT2				            ((5   << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define MC_REG_FOC_PULSE_DURATION	            ((6  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define MC_REG_FOC_PULSE_DECAY_DURATION 	    ((7  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )
#define MC_REG_HEATS_TEMP                       ((8  << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT )

/* TYPE_DATA_32BIT registers definition */
#define MC_REG_FAULTS_FLAGS		((0  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_I_U			((1  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_I_V			((2  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_I_W			((3  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_U_U			((4  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_U_V			((5  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_U_W			((6  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_I_ALPHA			((7  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_I_BETA                   ((8  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_U_ALPHA                  ((9  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_U_BETA                   ((10  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_I_D                      ((11  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_I_Q                      ((12  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_U_D                      ((13  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_U_Q                      ((14  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_I_RAW_U		((15  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_I_RAW_V		((16  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_I_RAW_W		((17  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FE_HSO			((18  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_HSO_FLUX_A		((19  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_HSO_FLUX_B		((20  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_U_DC			((30  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_I_U_OFFSET		((31  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_I_V_OFFSET		((32  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_I_W_OFFSET		((33  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_U_U_OFFSET		((34  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_U_V_OFFSET		((35  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ADC_U_W_OFFSET     	((36  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_OPENLOOP_ANGLE	((37  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_MECHANICAL_SPEED_RPM ((38  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_VDREF		((39  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_TORQUE_REF_NM	((42  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_THROTTLE_ADC		((43  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_THROTTLE_POS      	((44  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_RS			((45  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_LS			((46  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_KP_IDQ		((47  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_WI_IDQ		((48  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_KP_SPD		((49  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_KI_SPD		((50  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_SPEEDREF_FREQ_PU	((51  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_SPEEDREF_RAMPED_FREQ_PU ((52  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_SPEEDRAMP		((53  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_TORQUERAMP		((54  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_TORQUE_NM		((55  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_FELEC_HZ      	((56  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_ID_INJECT_A		((57  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_RS_EST		((58  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_USER_CURRENT_LIMIT	((59  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_LS_EST		((60  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_FLUX_RATED_VPHZ	((61  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_FLUX_VPHZ		((62  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_INJECTFREQ_HZ	((63  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_GAIN_D		((64  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_GAIN_Q		((65  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_DUTYCYCLE_D		((66  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_DUTYCYCLE_Q		((67  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_ANGLE_PU      	((68  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_RS_RATED		((69  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_RS_ACTIVE		((70  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_LS_RATED		((71  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_LS_ACTIVE		((72  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_MAXMODULATION		((73  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_ZEST_OUTD		((74  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_ZEST_OUTQ		((75  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_ANGLE_COMP_FACTOR	((76  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_MAX_ACC		((77  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_IQ_REF_SPEED		((78  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_IDREF		((79  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_IQREF		((80  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_MOTOR_POLEPAIRS		((81  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_PROFILER_RS_DC		((82  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_PROFILER_RS_AC		((83  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_PROFILER_LD_H		((84  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_PROFILER_FLUX_WB      	((85  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_PROFILER_POWERGOAL_W	((86  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_PROFILER_FLUXESTFREQ_HZ	((87  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
<#if MC.DBG_MCU_LOAD_MEASURE == true>
  #define MC_REG_PERF_CPU_LOAD		((88  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
  #define MC_REG_PERF_MIN_CPU_LOAD	((89  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
  #define MC_REG_PERF_MAX_CPU_LOAD	((90  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
</#if>
#define MC_REG_FOC_DUTYCYCLE_R	((106  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_DUTYCYCLE_S	((107  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_DUTYCYCLE_T	((108  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )

#define MC_REG_FOC_PULSE_CURRENT_GOAL ((110  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_PULSE_DUTY         ((111  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_PULSE_EST_ANGLE    ((112  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_KSAMPLE_DELAY      ((113  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_RSEST_RSOHM            ((114  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_RSEST_RSPOWEROHM       ((115  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_SKINFACTOR         ((117  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_FOC_ZEST_CORRECTION    ((118  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ANG_SPEED              ((119  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ZEST_CORR_LP           ((120  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_DEMOD_EQDQRIP_D        ((121  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_DEMOD_EQDQRIP_Q        ((122  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )
#define MC_REG_ZEST_FRACTION          ((123  << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT )

/* TYPE_DATA_STRING registers definition */
#define  MC_REG_FW_NAME               ((0  << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING )
#define  MC_REG_CTRL_STAGE_NAME       ((1  << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING )
#define  MC_REG_PWR_STAGE_NAME        ((2  << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING )
#define  MC_REG_MOTOR_NAME            ((3  << ELT_IDENTIFIER_POS) | TYPE_DATA_STRING )

/* TYPE_DATA_RAW registers definition */
#define  MC_REG_GLOBAL_CONFIG            ((0  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_MOTOR_CONFIG             ((1  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_APPLICATION_CONFIG       ((2 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_FOCFW_CONFIG             ((3 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_SCALE_CONFIG             ((4  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_ADC_OFFSET	             ((5  << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_ASYNC_UARTA           ((20 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_ASYNC_UARTB           ((21 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_ASYNC_STLNK           ((22 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_SHIFT_UARTA           ((25 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_SHIFT_UARTB           ((26 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )
#define  MC_REG_SHIFT_STLNK           ((27 << ELT_IDENTIFIER_POS) | TYPE_DATA_RAW )

uint8_t RI_SetRegisterGlobal(uint16_t regID, uint8_t typeID, uint8_t *data, uint16_t *size, int16_t dataAvailable);
<#list 1..(MC.DRIVE_NUMBER?number) as NUM>
uint8_t RI_SetRegisterMotor${NUM}(uint16_t regID,  uint8_t typeID,uint8_t *data, uint16_t *size, int16_t dataAvailable);
</#list>
uint8_t RI_GetRegisterGlobal(uint16_t regID, uint8_t typeID, uint8_t * data, uint16_t *size, int16_t freeSpace);
<#list 1..(MC.DRIVE_NUMBER?number) as NUM>
uint8_t RI_GetRegisterMotor${NUM}(uint16_t regID, uint8_t typeID, uint8_t * data, uint16_t *size, int16_t freeSpace);
</#list>
uint8_t RI_MovString(const char_t * srcString, char_t * destString, uint16_t *size, int16_t maxSize);
uint8_t RI_GetPtrReg (uint16_t dataID, void ** dataPtr);
uint8_t RI_GetIDSize (uint16_t ID);

uint8_t flashWrite_cb(uint16_t rxSize, uint8_t *rxBuffer, int16_t txFreeSpace, uint16_t *txSize, uint8_t *txBuffer);
uint8_t flashRead_cb(uint16_t rxSize, uint8_t *rxBuffer, int16_t txFreeSpace, uint16_t *txSize, uint8_t *txBuffer);

#endif /* REGISTER_INTERFACE_H */


/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
