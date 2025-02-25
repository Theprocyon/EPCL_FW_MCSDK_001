<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    mc_math.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control.
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
#include "mc_math.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MC_Math Motor Control Math functions
  * @brief Motor Control Mathematic functions of the Motor Control SDK
  *
  * @todo Document the Motor Control Math "module".
  *
  * @{
  */

#define SQRT_3_OVER_2				(0.86602540378f)
#define DCOMSHIFT					FIXP30(1.0f - SQRT_3_OVER_2)

/* Private macro -------------------------------------------------------------*/

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/* Three phase input Clarke transformation
 *
 * Ialpha = 2/3(Ia) - 1/3(Ib+Ic)
 * Ibeta  = 1/sqrt(3) (Ib-Ic)
 *        = (Ib - Ic) * 1/sqrt(3)
 *
 * All inputs must be normalized to a maximum value of (+/-) 1.0 in fixp30_t
 */
Currents_Iab_t MCM_Clarke_Current( const Currents_Irst_t Irst )
{
	Currents_Iab_t Iab;

	Iab.A = FIXP30_mpy(Irst.R, FIXP30(2.0f / 3.0f)) - FIXP30_mpy(Irst.S + Irst.T, FIXP30(1.0f / 3.0f));
	Iab.B = FIXP30_mpy((Irst.S - Irst.T), FIXP30(0.577350269));

	return (Iab);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/* Three phase input Clarke transformation
 *
 * Ialpha = 2/3(Ia) - 1/3(Ib+Ic)
 * Ibeta  = 1/sqrt(3) (Ib-Ic)
 *        = (Ib - Ic) * 1/sqrt(3)
 *
 * All inputs must be normalized to a between 0.0 and 1.0 (inclusive) in fixp30_t
 */
Voltages_Uab_t MCM_Clarke_Voltage( const Voltages_Urst_t Urst )
{
	Voltages_Uab_t Uab;

	Uab.A   = FIXP30_mpy(Urst.R, FIXP30(2.0f / 3.0f)) - FIXP30_mpy(Urst.S + Urst.T, FIXP30(1.0f / 3.0f));
	Uab.B   = FIXP30_mpy((Urst.S - Urst.T), FIXP30(0.577350269));

	return (Uab);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
Currents_Idq_t MCM_Park_Current( Currents_Iab_t Iab, FIXP_CosSin_t* cosSin )
{
	Currents_Idq_t Idq;

//	Idq.D = Iab.A * cos(theta) + Iab.B * sin(theta);
	Idq.D = FIXP30_mpy(Iab.A, cosSin->cos) + FIXP30_mpy(Iab.B, cosSin->sin);

//	Idq.Q = Iab.B * cos(theta) - Iab.A * sin(theta);
	Idq.Q = FIXP30_mpy(Iab.B, cosSin->cos) - FIXP30_mpy(Iab.A, cosSin->sin);

	return (Idq);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
Voltages_Udq_t MCM_Park_Voltage( Voltages_Uab_t Uab, FIXP_CosSin_t* cosSin )
{
	// The Current and Voltage variants of the Park function are the same, except for the data types
	Voltages_Udq_t Udq;

//	Idq.D = Iab.A * cos(theta) + Iab.B * sin(theta);
	Udq.D = FIXP30_mpy(Uab.A, cosSin->cos) + FIXP30_mpy(Uab.B, cosSin->sin);

//	Idq.Q = Iab.B * cos(theta) - Iab.A * sin(theta);
	Udq.Q = FIXP30_mpy(Uab.B, cosSin->cos) - FIXP30_mpy(Uab.A, cosSin->sin);

	return (Udq);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
Duty_Dab_t MCM_Inv_Park_Duty( Duty_Ddq_t Ddq, FIXP_CosSin_t* cosSin )
{
	Duty_Dab_t Dab;

	Dab.A = FIXP30_mpy(Ddq.D, cosSin->cos) - FIXP30_mpy(Ddq.Q, cosSin->sin);
	Dab.B = FIXP30_mpy(Ddq.Q, cosSin->cos) + FIXP30_mpy(Ddq.D, cosSin->sin);

	return (Dab);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/* Da	= Dalpha
 *
 * Db	= (-Dalpha + sqrt(3) * Dbeta) / 2
 * 		= -Dalpha/2 + sqrt(3) * Dbeta / 2
 * 		= -Dalpha/2 + sqrt(3)/2 * Dbeta
 *
 * Dc	= (-Dalpha - sqrt(3) * Dbeta) / 2		<-- Only change is + to -
 * 		= ...
 * 		= -Dalpha/2 - sqrt(3)/2 * Dbeta
 *
 * We calculate the following factors because we use them twice
 * D_factor_a = -Dalpha/2
 * D_factor_b = sqrt(3)/2 * Dbeta
 *
 */
Duty_Drst_t MCM_Inv_Clarke_Duty(Duty_Dab_t* Dab_pu)
{
	Duty_Drst_t Drst_pu;

	// D_factor_a = -Dalpha/2
	fixp30_t D_factor_a = -(Dab_pu->A >> 1);

	// D_factor_b = sqrt(3)/2 * Dbeta
	fixp30_t D_factor_b = FIXP30_mpy(FIXP30(SQRT_3_OVER_2), Dab_pu->B);

	Drst_pu.R = Dab_pu->A;
	Drst_pu.S = D_factor_a + D_factor_b;
	Drst_pu.T = D_factor_a - D_factor_b;

	return (Drst_pu);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
Duty_Drst_t MCM_Modulate(const Duty_Drst_t *pDrst_pu, const FOC_ModulationMode_e mode)
{
	/* Space Vector Modulation subtracts the average of the highest and lowest
	 * duties, centering the signals in the middle of the available voltage range */


	fixp30_t Dr, Ds, Dt, Dmin, Dmax, Dcom, De;

	/* Load into local variables for efficiency */
	Dr = pDrst_pu->R;
	Ds = pDrst_pu->S;
	Dt = pDrst_pu->T;

	/* Find lowest and highest values */
	Dmin = (Dr > Ds ? Ds : Dr);
	Dmin = (Dmin > Dt ? Dt : Dmin);

	Dmax = (Dr < Ds ? Ds : Dr);
	Dmax = (Dmax < Dt ? Dt : Dmax);

	switch(mode)
	{
	case FOC_MODULATIONMODE_Sine:
		Dcom = 0;
		break;
	default:
	case FOC_MODULATIONMODE_Centered:
		Dcom = (Dmin + Dmax) >> 1;
		break;

	case FOC_MODULATIONMODE_ShiftedCenter:
		Dcom = (Dmin + Dmax) >> 1;
		De = Dmin - Dcom - DCOMSHIFT + FIXP30(1.0f);
		if (De < 0) Dcom += De; /* add to all phases by adding to Dcom */
		Dcom += DCOMSHIFT; /* always lower zero-level by Dcomshift */
		break;
	case FOC_MODULATIONMODE_MinLow:
		Dcom = Dmin + FIXP30(1.0f);
		break;
	case FOC_MODULATIONMODE_MaxHigh:
		Dcom = Dmax - FIXP30(1.0f);
		break;
	case FOC_MODULATIONMODE_UpDown:
		if (Dmax > -Dmin)
		{
			Dcom = Dmax - FIXP30(1.0f);
		}
		else
		{
			Dcom = Dmin + FIXP30(1.0f);
		}
		break;
	}
	Duty_Drst_t Drst_out_pu;
	/* Subtract the average from the duties */
	Drst_out_pu.R = Dr - Dcom;
	Drst_out_pu.S = Ds - Dcom;
	Drst_out_pu.T = Dt - Dcom;

	return Drst_out_pu;
}



/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
