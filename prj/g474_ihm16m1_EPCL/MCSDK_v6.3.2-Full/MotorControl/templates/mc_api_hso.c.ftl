<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
/**
  ******************************************************************************
  * @file    mc_api.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements the high level interface of the Motor Control SDK.
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
  * @ingroup MCIAPIHSO
  */

#include "mc_interface.h"
#include "mc_api.h"
#include "mc_config.h"
#include "mcp.h"
/** @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup HSOAI Application Programming Interface for High Sensitivity Observer 
  * @brief Interface for application using the High Sensitivity Observer
  *
  * Motor Control Applications using the High Sensitivity Observer are offered a 
  * set of specific interfaces, that differ from the classic Interfaces provided 
  * by ST Motor Control SDK.
  *
  * @{
  */

/** @defgroup MCIAPIHSO Motor Control API for HSO
  * @brief High level Programming Interface of the Motor Control SDK for HSO applications
  *
  *  This interface allows for performing basic operations on the motor driven by a
  *  Motor Control SDK based application. With it, motors can be started and stopped, speed or
  *  torque ramps can be programmed and executed and information on the state of the motor can
  *  be retrieved, among others.
  *
  *  These functions aims at being the main interface used by an application to control the motor.
  * 
  * @{
  */

extern MCI_Handle_t * pMCI[NBR_OF_MOTORS];

/**
  * @brief  Initiates the start-up procedure for Motor 1
  *
  *  The MC_StartMotor1() function triggers the start-up procedure of the motor
  * and returns immediately. It returns #MC_SUCCESS if successful, or an error
  * code identifying the reason for failure if not.
  *
  *  This function is not blocking: it only initiates the start-up procedure. When
  * the procedure completes, the state machine of the motor goes to the #RUN state.
  * The state of the motor can be queried with the MC_GetSTMStateMotor1() function.
  *
  *  At the end of the start-up procedure, the motor is ready to be controlled by the
  * application.
  *
  *  The state machine of the motor must be in the #IDLE state for this function to
  * succeed. If it is not the case, the start-up procedure is not triggered and
  * the #MC_WRONG_STATE_ERROR status is returned.
  *
  *  Also, polarization offsets must have been either measured by or provided to the
  * Motor Control subsystem before this function is called. Otherwise, the start-up
  * procedure is not triggered and the #MC_NO_POLARIZATION_OFFSETS_ERROR error code
  * is returned. See MC_StartPolarizationOffsetsMeasurementMotor1() for the polarization
  * offsets measurements procedure and the MC_SetPolarizationOffsetsMotor1() function.
  *
  * @retval returns #MC_SUCCESS if successfully executed, or an error code otherwise.
  */
MC_RetStatus_t MC_StartMotor1(void)
{
	return ((MCI_StartMotor(pMCI[M1]) == true) ? MC_SUCCESS : MC_WRONG_STATE_ERROR);
}

/**
  * @brief  Initiates the start-up procedure for Motor 1 and the polarization offsets
  *         measurement procedure if needed
  *
  *  The MC_StartMotor1() function triggers the start-up procedure of the motor
  * and returns immediately. It returns #MC_SUCCESS if successful, or an error
  * code identifying the reason for failure if not.
  *
  *  This function is not blocking: it only initiates the start-up procedure. When
  * the procedure completes, the state machine of the motor goes to the #RUN state.
  * The state of the motor can be queried with the MC_GetSTMStateMotor1() function.
  *
  *  At the end of the start-up procedure, the motor is ready to be controlled by the
  * application.
  *
  *  The state machine of the motor must be in the #IDLE state for this function to
  * succeed. If it is not the case, the start-up procedure is not triggered and
  * the #MC_WRONG_STATE_ERROR status is returned.
  *
  *  If the polarization offsets have not been either measured by or provided to the
  * Motor Control subsystem before this function is called, the polarization offsets
  * measurement procedure is executed before the motor is started. See
  * MC_StartPolarizationOffsetsMeasurementMotor1() for the polarization offsets
  * measurements procedure and the MC_SetPolarizationOffsetsMotor1() function.
  *
  * @retval returns #MC_SUCCESS if successfully executed, or an error code otherwise.
  */
MC_RetStatus_t MC_StartWithPolarizationMotor1(void)
{
  return ((MCI_StartWithPolarizationMotor(pMCI[M1]) == true) ? MC_SUCCESS : MC_WRONG_STATE_ERROR);
}

/**
  * @brief  Initiates the polarization offsets measurement procedure
  *
  *  The MC_StartOffsetMeasurmentsMotor1() function triggers the polarization offsets 
  * measurement procedure and returns immediately. It returns #MC_SUCCESS if successful, 
  * or an error code identifying the reason for failure if not.
  *
  *  This function is not blocking: it only initiates the polarization offsets measurement. 
  * When the procedure completes, the state machine of the motor goes to the #IDLE state.
  * The state of the motor can be queried with the MC_GetSTMStateMotor1() function.
  *
  *  The state machine of the motor must be in the #IDLE state for this function to
  * succeed. If it is not the case, the start-up procedure is not triggered and
  * the #MC_WRONG_STATE_ERROR status is returned.
  *
  * @retval returns #MC_SUCCESS if successfully executed, or an error code otherwise.
  */
MC_RetStatus_t MC_StartOffsetMeasurmentsMotor1(void)
{
  return ((MCI_StartOffsetMeasurments(pMCI[M1]) == true) ? MC_SUCCESS : MC_WRONG_STATE_ERROR);
}

/**
  * @brief  Initiates the stop procedure for Motor 1.
  *
  *  The MC_StopMotor1() function triggers the stop procedure of the motor
  * and returns immediately. It returns MC_SUCCESS if successful, or an error
  * code identifying the reason for failure if not.
  *
  *  This function is not blocking: it only initiates the stop procedure. When
  * this procedure completes, the state machine of the motor goes to the #IDLE state.
  * The state of the motor can be queried with the MC_GetSTMStateMotor1() function.
  *
  *  At the end of the stop procedure, the motor is not under control anymore.
  *
  *  The state machine of the motor must be in the #RUN, #CHARGE_BOOT_CAP,
  * #OFFSET_CALIB or #STATE_RSDCESTIMATE states for this function to succeed. If it is 
  * not the case, the stop procedure is not triggered and the #MC_WRONG_STATE_ERROR 
  * status is returned.
  *
  * @retval returns #MC_SUCCESS if successfully executed, or an error code otherwise.
  */
MC_RetStatus_t MC_StopMotor1(void)
{
  return ((MCI_StopMotor(pMCI[M1]) == true) ? MC_SUCCESS : MC_WRONG_STATE_ERROR);
}

/**
  * @brief Sets the acceleration to apply on Motor 1 when its speed reference is changed
  *
  * When setting the speed reference to a new value with MC_SetSpeedReferenceMotor1(), MC_SetSpeedReferenceMotor1_F() or
  * MC_SetSpeedReferenceMotor1_PU(), @p Acceleration is used to compute the speed ramp that is applied to reach the new speed
  * value.
  *
  * @param Acceleration Electrical acceleration of the motor expressed in Hertz/s.
  */
void MC_SetAccelerationMotor1( fixp16_t Acceleration )
{
    MCI_SetSpeedRamp( pMCI[M1], Acceleration );
}

/// Same as MC_SetAccelerationMotor1() with Acceleration in float
void MC_SetAccelerationMotor1_F( float Acceleration )
{
    MCI_SetSpeedRamp_F( pMCI[M1], Acceleration );
}

/**
  * @brief Sets the electrical acceleration to apply on Motor 1 when its speed reference is changed
  *        expressed in "Per Unit" and per PWM period.
  *
  * This function is very similar to MC_SetAccelerationMotor1. The differences are:
  *
  * - @p Acceleration is expressed as a "per unit" and per PWM period value in the [-1.0 , 1.0] range.
  *   @p Acceleration is to be multiplied by the Frequency Scale parameter (the max application speed)
  *      and by the PWM frequency in order to get the acceleration in Hz/s.
  *   @p Acceleration is a fixp30_t value instead of a fixp16_t one.
  *   Note
  *      The fixp30_t allows for values in the [-2.0 , 2.0[ range. However, values outside
  *      of the [-1.0 , 1.0] should not be used.
  * @param Acceleration Electrical acceleration of the motor expressed in fraction of the Frequency Scale per PWM period.
  */
void MC_SetAccelerationMotor1_PU( fixp30_t Acceleration  )
{
    MCI_SetSpeedRamp_PU( pMCI[M1], Acceleration );
}

/**
 * @brief Returns the electrical acceleration applied on Motor 1 when its speed reference is changed
 *
 * The format of the returned acceleration is fixed point and the value in Hz/s.
 */
fixp16_t MC_GetAccelerationMotor1(void)
{
    return FIXP16(MCI_GetSpeedRamp_F( pMCI[M1] ));
}

/**
 * @brief Returns the electrical acceleration applied on Motor 1 when its speed reference is changed
 *
 * The format of the returned acceleration is float and the value in Hz/s.
 */
float MC_GetAccelerationMotor1_F(void)
{
    return MCI_GetSpeedRamp_F( pMCI[M1] );
}

/**
 * @brief Returns the electrical acceleration applied on Motor 1 when its speed reference is changed.
 *
 * The format of the returned acceleration is fixed point and the value in "Per Unit" and per PWM period.
 */
fixp30_t MC_GetAccelerationMotor1_PU(void)
{
    return MCI_GetSpeedRamp_PU( pMCI[M1] );
}

/**
  * @brief Sets the electrical rotation speed of Motor 1.
  *
  * If the motor's state machine is in the #RUN state, the electrical speed of the
  * motor is set to @p Speed, otherwise the function does nothing.
  *
  *  The new Speed is reached with the acceleration set thanks to the MC_SetAccelerationMotor1()
  * or MC_SetAccelerationPUMotor1() functions. If no acceleration was set, the default one, set in
  * the application's parameters is used.
  *
  * The sign of the @p Speed parameter indicates the direction of the rotation.
  *
  * @param  Speed Electrical rotor speed reference. Expressed in Hz.
  */
void MC_SetSpeedReferenceMotor1( fixp16_t Speed )
{
    MCI_SetSpeedReference( pMCI[M1], Speed );
}

/// Same as MC_SetSpeedReferenceMotor1() with Speed in float
void MC_SetSpeedReferenceMotor1_F( float Speed )
{
    MCI_SetSpeedReference_F( pMCI[M1], Speed );
}

/**
  * @brief Sets the electrical rotation speed of Motor 1 expressed in "Per-Unit"
  *
  * This function is very similar to MC_SetSpeedRampMotor1. The differences are:
  *
  * - @p Speed is expressed as a "per unit" value in the [-2.0 , 2.0[ range. @p Speed is to
  *   be multiplied by the Frequency Scale parameter (the max application speed) in order
  *   to get the speed in Hz.
  * - @p Speed is a fixp30_t value instead of a fixp16_t one.
  *
  * @param  Speed Electrical rotor speed reference. Expressed as a fraction of the
  *         Speed Scale of the application.
  */
void MC_SetSpeedReferenceMotor1_PU( fixp30_t Speed )
{
    MCI_SetSpeedReference_PU( pMCI[M1], Speed );
}

/**
 *  @brief Returns the electrical rotor speed reference set for Motor 1.
 *
 * The format of the returned speed is fixed point, and the value is in Hz.
 */
fixp16_t MC_GetSpeedReferenceMotor1(void)
{
    return MCI_GetSpeedReference( pMCI[M1] );
}

/**
 *  @brief Returns the electrical rotor speed reference set for Motor 1.
 *
 * The format of the returned speed is float, and the value is in Hz.
 */
float MC_GetSpeedReferenceMotor1_F(void)
{
    return MCI_GetSpeedReference_F( pMCI[M1] );
}

/**
 *  @brief Returns the electrical rotor speed reference set for Motor 1.
 *
 * The format of the returned speed is fixed point, and the value is in "Per Unit".
 */
fixp30_t MC_GetSpeedReferenceMotor1_PU(void)
{
    return MCI_GetSpeedReference_PU( pMCI[M1] );
}

/**
 * @brief Returns the average electrical rotor speed for Motor 1.
 *
 * The format of the returned speed is fixed point, and the value is in Hz.
 */
fixp16_t MC_GetSpeedMotor1(void)
{
    return MCI_GetAvrgSpeed( pMCI[M1] );
}

/**
 * @brief Returns the average electrical rotor speed for Motor 1
 *
 * The format of the returned speed is float, and the value is in Hz.
 */
float MC_GetSpeedMotor1_F(void)
{
    return MCI_GetAvrgSpeed_F( pMCI[M1] );
}

/**
 * @brief Returns the average electrical rotor speed for Motor 1
 *
 * The format of the returned speed is fixed point, and the value is in "Per Unit".
 */
fixp30_t MC_GetSpeedMotor1_PU(void)
{
    return MCI_GetAvrgSpeed_PU( pMCI[M1] );
}

/**
 * @brief Returns the magnitude of EMF Electro Motive Force for Motor 1
 *
 * The format of the returned EMF is float, and the value is in Volts.
 */
float MC_GetEmfMagnitudeMotor1_F(void)
{
    return MCI_GetEmfMagnitude_F( pMCI[M1] );
}

/**
 * @brief Returns the magnitude of EMF Electro Motive Force for Motor 1
 *
 * The format of the returned EMF is fixed point, and the value is in "Per Unit".
 */
fixp30_t MC_GetEmfMagnitudeMotor1_PU(void)
{
    return MCI_GetEmfMagnitude_PU( pMCI[M1] );
}

/**
 * @brief Sets the track angle flag for Motor 1
 *
 * Allow to fix or track the angle provided by the High Sensitivity Observer.
 */
void MC_SetFlagFreezeTrackAngle(bool value)
{
    MCI_SetFlagFreezeTrackAngle( pMCI[M1], value );
}

/**
 * @brief Sets the rated flux for Motor 1
 *
 * @param  flux_VpHz Rated flux, expressed in V/Hz.
 */
void MC_SetRatedFluxMotor1(fixp24_t flux_VpHz)
{
    MCI_SetRatedFlux( pMCI[M1], flux_VpHz );
}

/**
  * @brief Sets the torque produced on the shaft of Motor 1.
  *
  * If the motor's state machine is in the #RUN state, the mechanical torque of the
  * motor is set to @p Torque, otherwise the function does nothing.
  *
  * The sign of the @p Torque parameter indicates the direction of the torque to produce.
  *
  * @param Torque Mechanical torque reference. Expressed in Nm.
  */
void MC_SetTorqueReferenceMotor1( fixp16_t Torque )
{
    MCI_SetTorqueReference( pMCI[M1], Torque );
}

/// Same as MC_SetTorqueReferenceMotor1() with Torque in float
void MC_SetTorqueReferenceMotor1_F( float Torque )
{
    MCI_SetTorqueReference_F( pMCI[M1], Torque );
}

/**
  * @brief Sets the mechanical torque to produce on the shaft of Motor 1 expressed in "Per Unit".
  *
  * This function is very similar to MC_SetTorqueReferenceMotor1. The differences are:
  *
  * - @p Torque torque  expressed as a "per unit" value in the [-2.0 , 2.0[ range.
  *   Multiply it by 1.5 times the Current Scale parameter (the max application current)
  *   to get the value in Nm.
  * - @p Torque is a fixp30_t value instead of a fixp16_t one.
  *
  * @param Torque Mechanical torque reference. Expressed in fraction of the maximum torque.
  */
void MC_SetTorqueReferenceMotor1_PU( fixp30_t Torque )
{
    MCI_SetTorqueReference_PU( pMCI[M1], Torque );
}

/**
 * @brief Returns the torque reference for Motor 1.
 *
 * The format of the returned torque is fixed point, and the value is in Nm.
 *
 *  This function may return invalid results if the parameters of the application
 * results in a possible torque range that that exceeds the [-65536, 65536[ Nm
 * possibility of the fixp16_t type. If the actual value does not fit in a fixp16_t
 * the returned value will either be 65,535.9999847412109375 ((2^32-1)/(2^16)) or
 * -65536 (-2^32/2^16) depending on the sign of the actual value.
 *
 * It is left to the responsibility of users to determine whether they can use this
 * function. This depends on the maximum torque their application can produce. An
 * absolute value of 65,536 Nm for the torque produced by the motor is considered
 * high enough for most applications.
 */
fixp16_t MC_GetTorqueReferenceMotor1(void)
{
    return MCI_GetTorqueReference( pMCI[M1] );
}

/**
 * @brief Returns the torque reference for Motor 1.
 *
 * The format of the returned torque is float, and the value is in Nm.
 *
 */
float MC_GetTorqueReferenceMotor1_F(void)
{
    return MCI_GetTorqueReference_F( pMCI[M1] );
}

/**
 * @brief Returns the torque reference for Motor 1.
 *
 * The format of the returned torque is fixed point, and the value is in "Per Unit".
 */
fixp30_t MC_GetTorqueReferenceMotor1_PU(void)
{
    return MCI_GetTorqueReference_PU( pMCI[M1] );
}

/**
 * @brief Returns the torque currently produced on the shaft of Motor 1.
 *
 * The format of the returned torque is fixed point, and the value is in Nm.
 *
 *  This function may return invalid results if the parameters of the application
 * results in a possible torque range that that exceeds the [-65536, 65536[ Nm
 * possibility of the fixp16_t type. If the actual value does not fit in a fixp16_t
 * the returned value will either be 65,535.9999847412109375 ((2^32-1)/(2^16)) or
 * -65536 (-2^32/2^16) depending on the sign of the actual value.
 *
 * It is left to the responsibility of users to determine whether they can use this
 * function. This depends on the maximum torque their application can produce. An
 * absolute value of 65,536 Nm for the torque produced by the motor is considered
 * high enough for most applications.
 */
fixp16_t MC_GetTorqueMotor1( void )
{
    return MCI_GetTorque_Nm( pMCI[M1] );
}

/**
 * @brief Returns the torque currently produced on the shaft of Motor 1.
 *
 * The format of the returned torque is float, and the value is in Nm.
 *
 */
float MC_GetTorqueMotor1_F(void)
{
    return MCI_GetTorque_Nm_F( pMCI[M1] );
}

/**
 * @brief Returns the torque currently produced on Motor 1.
 *
 * The format of the returned torque is fixed point, and the value is in "Per Unit".
 */
fixp30_t MC_GetTorqueMotor1_PU(void)
{
    return MCI_GetTorque_PU( pMCI[M1] );
}

/**
  * @brief Programs the current reference to Motor 1 for later or immediate execution.
  *
  *  The current reference to consider is made of the Id and Iq current components.
  *
  *  Invoking the MC_SetCurrentReferenceMotor1() function programs a current reference
  * with the provided parameters. The programmed reference is executed immediately if
  * the state machine of the motor is in the #RUN states. Otherwise, the command is buffered
  * and will be executed when the state machine reaches the #RUN state.
  *
  * @param  Idqref current reference in the Direct-Quadratic reference frame. Each component
  *         is expressed in Ampere.
  */
void MC_SetCurrentReferenceMotor1( dq_fixp16_t Idqref )
{
    MCI_SetCurrentReferences( pMCI[M1], Idqref );
}

/// Same as MC_SetCurrentReferenceMotor1() with current references expresses in float
void MC_SetCurrentReferenceMotor1_F( dq_float_t Idqref )
{
    MCI_SetCurrentReferences_F( pMCI[M1], Idqref );
}

/**
  * @brief Programs the current reference for Motor 1 for later or immediate execution, in "Per Unit".
  *
  * This function is very similar to MC_SetCurrentReferenceMotor1. The differences are:
  *
  * - @p Idqref currents are expressed as "per unit" values in the [-2.0 , 2.0[ range.
  *   Multiply them by the Current Scale parameter (the max application current) to get
  *   values in A.
  * - @p Idqref currents are fixp30_t values instead of fixp16_t ones.
  *
  * @param  Idqref current reference in the Direct-Quadratic reference frame. Each component
  *         is expressed in fraction of the Maximum current .
  */
void MC_SetCurrentReferenceMotor1_PU( dq_fixp30_t Idqref )
{
    MCI_SetCurrentReferences_PU( pMCI[M1], Idqref );
}

/**
 * @brief Returns the Current reference set for motor 1
 *
 * The current reference is returned in the D-Q frame. The format of each component is fixed point and
 * values are in Ampere.
 */
dq_fixp16_t MC_GetCurrentReferenceMotor1(void)
{
    return MCI_GetCurrentReferences( pMCI[M1] );
}

/**
 * @brief Returns the Current reference set for motor 1
 *
 * The current reference is returned in the D-Q frame. The format of each component is float and
 * values are in Ampere.
 */
dq_float_t MC_GetCurrentReferenceMotor1_F(void)
{
    return MCI_GetCurrentReferences_F( pMCI[M1] );
}

/**
 * @brief Returns the Current measured for motor 1
 *
 * The current reference is returned in the D-Q frame. The format of each component is fixed point and
 * values are in "Per Unit".
 */
dq_fixp30_t MC_GetCurrentReferenceMotor1_PU(void)
{
    return MCI_GetCurrentReferences_PU( pMCI[M1] );
}

/**
 * @brief Returns the Current measured for motor 1
 *
 * The current measure is returned in the D-Q frame. The format of each component is fixed point and
 * values are in Ampere.
 */
dq_fixp16_t MC_GetCurrentMotor1(void)
{
    return MCI_GetCurrent( pMCI[M1] );
}

/**
 * @brief Returns the Current reference set for motor 1
 *
 * The current measure is returned in the D-Q frame. The format of each component is float and
 * values are in Ampere.
 */
dq_float_t MC_GetCurrentMotor1_F(void)
{
    return MCI_GetCurrent_F( pMCI[M1] );
}

/**
 * @brief Returns the Current reference set for motor 1
 *
 * The current measure is returned in the D-Q frame. The format of each component is fixed point and
 * values are in "Per Unit".
 */
dq_fixp30_t MC_GetCurrentMotor1_PU(void)
{
    return MCI_GetCurrent_PU( pMCI[M1] );
}

/**
 * @brief Specifies the duty cycles to apply on the phase voltages in the DQ frame
 *
 * @param DutyCycle Duty Cycle to apply
 */
void MC_SetDutyCycleMotor1_PU( dq_fixp30_t DutyCycle )
{
    MCI_SetDutyCycle_PU( pMCI[M1], DutyCycle );
}

/**
 * @brief Returns the duty cycles applied on the phase voltages in the DQ frame.
 *
 * duty cycles are returned in the D-Q frame. The format of each component is fixed point and
 * values are in "Per Unit".
 */
dq_fixp30_t MC_GetDutyCycleMotor1_PU(void)
{
    return MCI_GetDutyCycle_PU( pMCI[M1] );
}

/**
 * @brief Returns the Control Mode used for Motor 1.
 */
MC_ControlMode_t MC_GetControlModeMotor1(void)
{
    return MCI_GetControlMode( pMCI[M1] );
}

/**
 * @brief Sets the control mode for Motor 1 to @p mode and returns true on success
 *
 * If the control mode of Motor 1 cannot be set to @p mode, false is returned and the
 * control mode remains unchanged.
 */
bool MC_SetControlModeMotor1( MC_ControlMode_t mode )
{
    return MCI_SetControlMode( pMCI[M1], mode );
}

/**
  * @brief Sets the polarization offset values to use for Motor 1
  *
  * The Motor Control algorithm relies on a number of current and voltage measures. The hardware
  * parts that make these measurements need to be characterized at least once in the course of
  * product life, prior to its first activation. This characterization consists in measuring the
  * voltage presented to the ADC channels when either no current flows into the phases of the motor
  * or no voltage is applied to them. This characterization is named polarization offsets measurement
  * and its results are the polarization offsets.
  *
  * The Motor Control Firmware can performs this polarization offsets measurement procedure which
  * results in a number of offset values that the application can store in a non volatile memory and
  * then set into the Motor Control subsystem at power-on or after a reset.
  *
  * The application uses this function to set the polarization offset values that the Motor Control
  * subsystem is to use in the current session. This function can only be used when the state machine
  * of the motor is in the #IDLE state in which case it returns #MC_SUCCESS. Otherwise, it does nothing
  * and returns the #MC_WRONG_STATE_ERROR error code.
  *
  *  The Motor Control subsystem needs to know the polarization offsets before the motor can be controlled.
  * The MC_SetPolarizationOffsetsMotor1() function provides a way to set these offsets. Alternatively, the
  * application can either:
  *
  *  * Execute the polarization offsets measurement procedure with a call to
  *    MC_StartPolarizationOffsetsMeasurementMotor1() after a reset or a power on;
  *  * Start the motor control with the MC_StartWithPolarizationMotor1() that will execute the procedure
  *    before actually starting the motor, on the first time it is called after a reset or a power on.
  *
  * When this function completes successfully, the state of the polarization offsets measurement procedure
  * is set to #COMPLETED. See MC_GetPolarizationState().
  *
  * @param PolarizationOffsets an pointer on a structure containing the offset values
  */
MC_RetStatus_t MC_SetPolarizationOffsetsMotor1( PolarizationOffsets_t * PolarizationOffsets )
{
  return ((MCI_SetCalibratedOffsetsMotor(pMCI[M1], PolarizationOffsets) == true) ? MC_SUCCESS : MC_WRONG_STATE_ERROR);
}

/**
  * @brief Returns the polarization offset values measured or set for Motor 1
  *
  *  See MC_SetPolarizationOffsetsMotor1() for more details.
  *
  *  If the Motor Control Firmware knows the polarization offset values, they are copied into the
  * @p PolarizationOffsets structure and #MC_SUCCESS is returned. Otherwise, nothing is done and
  * #MC_NO_POLARIZATION_OFFSETS_ERROR is returned.
  *
  * @param PolarizationOffsets an pointer on the structure into which the polarization offsets will be
  *        copied
  * @return #MC_SUCCESS if calibration data were present and could be copied into @p PolarizationOffsets,
  *         #MC_NO_POLARIZATION_OFFSETS_ERROR otherwise.
  */
MC_RetStatus_t MC_GetPolarizationOffsetsMotor1( PolarizationOffsets_t * PolarizationOffsets )
{
   return ( MCI_GetCalibratedOffsetsMotor( pMCI[M1], PolarizationOffsets) );
}

/**
  * @brief Starts the polarization offsets measurement procedure.
  *
  * See MC_SetPolarizationOffsetsMotor1() for more details.
  *
  * If the Motor Control Firmware is in the #IDLE state, the procedure is started, the state machine
  * of the motor switches to #OFFSET_CALIB and #MC_SUCCESS is returned. Otherwise, nothing is done
  * and the #MC_WRONG_STATE_ERROR error code is returned.
  *
  * The polarization offsets measurement procedure is only triggered by this function and it is has not
  * completed when this function returns. The application can use the MC_GetPolarizationState()
  * function to query the state of the procedure.
  *
  * @see MC_GetPolarizationState()
  */
MC_RetStatus_t MC_StartPolarizationOffsetsMeasurementMotor1( void )
{
  return ((MCI_StartOffsetsCalibrationMotor(pMCI[M1]) == true) ? MC_SUCCESS : MC_WRONG_STATE_ERROR);
}

/**
  * @brief Returns the state of the polarization offsets measurement procedure.
  *
  * @see MC_SetPolarizationOffsetsMotor1
  */
PolarizationState_t MC_GetPolarizationState( void )
{
   return ( MCI_GetOffsetsCalibrationState( pMCI[M1] ) );
}

/**
 * @brief Acknowledges Motor Control faults that occurred on Motor 1.
 *
 *  This function informs the state machine of the motor that the Application has taken
 * the error condition that occurred into account. If no error condition exists when
 * the function is called, nothing is done and false is returned. Otherwise, true is
 * returned.
 */
bool MC_AcknowledgeFaultsMotor1( void )
{
    return MCI_FaultAcknowledged( pMCI[M1] );
}

/**
 * @brief Returns a bit-field showing non acknowledged faults that occurred on Motor 1.
 *
 * This function returns a 16 bit fields containing the Motor Control faults
 * that have occurred on Motor 1 since the last call to MC_AcknowledgeFaultsMotor1().
 *
 * See @ref fault_codes "Motor Control Faults" for a list of 
 * of all possible faults codes.
 */
uint32_t MC_GetOccurredFaultsMotor1(void)
{
    return MCI_GetOccurredFaults( pMCI[M1] );
}

/**
 * @brief Returns a bit-field showing any currently active fault on Motor 1.
 *
 * This function returns a 16 bit fields containing the Motor Control faults
 * which trigger condition are currently true. Calling MC_AcknowledgeFaultsMotor1()
 * does not clear these faults.
 *
 * See @ref fault_codes "Motor Control Faults" for a list of 
 * of all possible faults codes.
 */
uint32_t MC_GetCurrentFaultsMotor1(void)
{
    return MCI_GetCurrentFaults( pMCI[M1] );
}

/**
 * @brief returns the current state of Motor 1 state machine
 */
MCI_State_t  MC_GetSTMStateMotor1(void)
{
   return MCI_GetSTMState( pMCI[M1] );
}

/**
 * @brief sets HSO minimum crossover frequency
 *
 * @param frequency minimum crossover frequency in Hz
 */
void MC_SetMinHSOCrossoverFreq(float frequency)
{
  MCI_SetMinHSOMinCrossoverFreqHz(pMCI[M1], frequency);
}

<#if MC.M1_SPEED_SENSOR == "ZEST">
/**
 * @brief sets alternate HSO minimum crossover frequency
 *
 * @param frequency minimum crossover frequency in Hz
 */
void MC_SetAltMinHSOCrossoverFreq(float frequency)
{
  MCI_SetAltMinHSOMinCrossoverFreqHz(pMCI[M1], frequency);
}
</#if>

/**
 * @brief sets HSO crossover frequency
 *
 * @param frequency crossover frequency in Hz
 */
void MC_SetHSOCrossoverFreq(float frequency)
{
  MCI_SetHSOCrossoverFreqHz(pMCI[M1], frequency);
}

/**
  * @brief Configures RsDC estimation procedure, full procedure (Rs
  *        estimation and motor aligment) or only motor aligment
  *
  * @param value if true RsDC estimation performs only a motor alignment else
  *              RsDC estimation performs a motor aligment and a RsDC estimation
  */
void MC_SetRsDCEstimationAlignOnlyMotor1(bool value)
{
  MCI_SetRsDCEstimationAlignOnly(pMCI[M1], value);
}

/**
  * @brief Enables RsDC estimation procedure. to be done before doing an FOC mode
  *        transition
  */
void MC_SetRsDCEstimationEnableMotor1(void)
{
  MCI_SetRsDCEstimationEnable(pMCI[M1]);
}

/**
  * @brief Disables RsDC estimation procedure. to be done before doing an FOC mode
  *        transition
  */
void MC_SetRsDCEstimationDisableMotor1(void)
{
  MCI_SetRsDCEstimationDisable(pMCI[M1]);
}

/**
  * @brief Returns the state of the RsDC estimation procedure
  */
RsDC_estimationState_t MC_GetRsDCEstimationStateMotor1(void)
{
  return MCI_GetRsDCEstimationState(pMCI[M1]);
}

/**
  * @brief Enables angle estimation procedure at startup. To be done before doing a FOC mode
  *        transition.
  */
void MC_EnablePulseMotor1(void)
{
  MCI_EnablePulse(pMCI[M1]);
}

/**
  * @brief Disables angle estimation procedure at startup. To be done before doing a FOC mode
  *        transition.
  */
void MC_DisablePulseMotor1(void)
{
  MCI_DisablePulse(pMCI[M1]);
}

/**
  * @brief Indicates whether pulses processing is completed.
  * @retval return true if pulse processing is busy.
  */
bool MC_IsPulseBusyMotor1(void)
{
  return MCI_IsPulseBusy(pMCI[M1]);
}

/**
 * @brief Not implemented MC_Profiler function.
 *  */
uint8_t MC_ProfilerCommand (uint16_t rxLength, uint8_t *rxBuffer, int16_t txSyncFreeSpace, uint16_t *txLength, uint8_t *txBuffer)
{
  return MCP_CMD_UNKNOWN;
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

