/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Raser 
****************************************************************************
****************************************************************************
***  Autor: Georg Arbeiter      
***************************************************************************/

#ifndef ODO_H
#define ODO_H

/** \file odo.h
 *
 * Functions to read the wheel encoders.
 *
 *  \author Georg Arbeiter
 */

#define DIST_PER_INC 1.719e-5		///< Distance per encoder increase.
#define WHEEL_BASE 0.176			///< Diameter between wheels

uint16_t enc_left;					///< Encoder count of left wheel
uint16_t enc_right;					///< Encoder count of right wheel
uint8_t  odo_reset;					///< Reset odometry count
//int32_t ix,iy;
//int16_t iphi;
double iphi;						///< Global variable with Tablobot's angle
double ix;							///< Global variable with Tablobot's X coordinate
double iy;							///< Global variable with Tablobot's Y coordinate

/** Calculates the difference between left and right enconder and adds them up.*/
void odo_cycle(void);

/** Counts the pulses on the left encoder.*/
void enc_cycle_left(void);

/** Counts the pulses on the right encoder.*/
void enc_cycle_right(void);

#endif
