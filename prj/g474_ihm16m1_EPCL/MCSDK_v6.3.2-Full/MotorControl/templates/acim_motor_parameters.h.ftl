<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    acim_motor_parameters.h
  * @author  
  * @version 
  * @date    
  * @project 
  * @path    
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure the ACIM FOC speed and current control. 
  *      It is created by the ACIM GUI solving compatibility issues with the official release.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ACIM_MOTOR_PARAMS_H
#define __ACIM_MOTOR_PARAMS_H

/* Private defines ------------------------------------------------------------*/
#define RAD3DIV3        0.577350269f

/* Motor parameters ----------------------------------------------------------*/
<#if (MC.ACIM_CONFIG == "IFOC" | MC.ACIM_CONFIG == "LSO_FOC")>
#define IMAGN_A         ((float)(${MC.IMAGN_A})) /* Magnetizing Current (No-Load current) expressed in Ampere */

#define RR              ((float)(${MC.RR})) /* Rotor resistance referred to stator equivalent circuit, (Ohm)*/
#define LLS             ((float)(${MC.LLS})) /* Stator leakage inductance, (H)*/
#define LLR             ((float)(${MC.LLR})) /* Rotor leakage  inductance referred to the stator equivalent circuit, (H)*/
#define LMS             ((float)(${MC.LMS})) /* Motor Magnetizing inductance ((Steady-state model), (H) */

#define LM              ((float)(${MC.LM})) /* Motor Magnetizing inductance (Dynamic model), (H) */     
#define LR              ((float)(${MC.LR})) /* Global rotor inductance (Dynamic model), Lr = Llr + LM, (H)*/
#define LS              ((float)(${MC.M1_LS})) /* Global stator inductance (Dynamic model), Ls = Lls + LM, (H)*/
#define TAU_R           ((float)(${MC.TAU_R})) /* Rotor Time constant, Lr/Rr, (s) */
#define TAU_S           ((float)(${MC.TAU_S})) /* Statot Time constant, Ls/Rs, (s) */

#define SIGMA           ((float)(${MC.SIGMA})) /* Total leakage factor, (dimensionless)*/

</#if>
#define POLE_PAIR_NUM   ${MC.ACIM_POLE_PAIR_NUM} /* Number of motor pole pairs */
#define RS              ${MC.ACIM_RS} /* Stator resistance , ohm*/
#define NOMINAL_CURRENT ${MC.ACIM_NOMINAL_CURRENT}
   
/* Required by V/f control */   
#define NOMINAL_PHASE_VOLTAGE  ${MC.NOMINAL_PHASE_VOLTAGE} /* Nominal stator phase-to-neutral voltage, Vrms */ 

/* K = Vn/fn , it should be defined considering the pulsation we instead the frequency */ 
#define FLUX_K          ((float)(${MC.FLUX_K}))

#define NOMINAL_FREQ    ${MC.NOMINAL_FREQ} /* Nominal frequency, Hz */

<#if (MC.ACIM_CONFIG == "IFOC" | MC.ACIM_CONFIG == "VF_CL")>
/*** Quadrature encoder ***/
#define M1_ENCODER_PPR     ${MC.M1_ENCODER_PPR} /*!< Number of pulses per revolution */   

</#if>
#define MOTOR_MAX_SPEED_RPM    ${MC.M1_MOTOR_MAX_SPEED_RPM} /*!< Maximum rated speed  */

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*__ACIM_MOTOR_PARAMS_H*/

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
