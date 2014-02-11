/*******************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Quirl
*******************************************************************************/
/** \file mot.h
 *  
 *  Motor Driver
 *  
 *  Provides the motor-control for the Quirl-drive
 *  \version 1.0
 */

#ifndef MOT_H
#define MOT_H


double yl_I; ///< Integral output component of left PI Controller
double yr_I; ///< Integral output component of right PI Controller

///Initializes the Registers that control the motors.
void mot_init(void);

/** Sets the PWM Output signal for the right motor.
 * \param vel Speed value (positive or negative).
 */
void set_right(int16_t vel);

/** Sets the PWM Output signal for the left motor.
 * \param vel Speed output value (positive or negative).
 */
void set_left(int16_t vel);

/** Provides a PI Controller which with the input from odometry sets the output values.
 * to maintain the speed given in the parameters.
 * \param soll_l Desired speed of the left motor.
 * \param soll_r Desired speed of the right motor.
 */
void mot_ctrl(int16_t soll_l, int16_t soll_r);

/**Virtual wheel control
 * Allows control of the motion of the tablobot by setting the parameters of a
 * virtual wheel.
 * \param soll_v Desired speed of the virtual wheel.
 * \param v_phi Desired angle of the virtual wheel.
 */
void vrad_ctrl(int8_t soll_v, double v_phi);

#endif
