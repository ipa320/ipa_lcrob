/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Raser 
****************************************************************************
****************************************************************************
***  Autor: Georg Arbeiter      
***************************************************************************/
/** \file odo.c
 *
 * Functions to read the wheel encoders.
 *
 *  \author Georg Arbeiter
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <math.h>
#include "interfaces.h"
#include "mot.h"
#include "odo.h"
#include "usart.h"

void odo_cycle(void) 
{
	static double x, y, phi;
	static uint16_t left_old, right_old;
	int16_t delta_left, delta_right;
	double moved;

	if(odo_reset) 
	{
		
	}
	else
	{cli();
		delta_left  = enc_left - left_old;					//Left difference calculation
		delta_right = enc_right - right_old;				//Right difference calculation
		left_old = enc_left;								//New value becomes old value
		right_old = enc_right;
	       
		moved = 0.5 * (1000 * DIST_PER_INC) * (delta_right + delta_left);	//Calculate movement
		
		phi += DIST_PER_INC / WHEEL_BASE * (delta_right - delta_left);		//Calculate rotation angle

		x += moved * cos(phi);							//Determine x and y coordinates
		y += moved * sin(phi);
		
		if(phi >= M_PI)
			phi -= 2 * M_PI;
		if(phi < -M_PI)
			phi += 2 * M_PI;                        //Limit angle between -PI <= phi <= PI
	sei();
	}

	cli();
	//ix = (int32_t)floor(x);				//x abrunden
	//iy = (int32_t)floor(y);				//y abrunden
	//iphi = (int16_t)floor(32768. / M_PI * phi);		//phi abrunden
	ix=x;
	iy=y;
	iphi = phi;
	
	sei();
}

void enc_cycle_left(void) 
{

	// first time only left
	static uint8_t ab_old;
	uint8_t ab, change;

	ab = LEFT_PORT & LEFT_MASK;		// Read status of odometry left AND concatenate with the left mask
	change = ab ^ ab_old;			// XOR, i.e. changes
	if(change) 						// If there is a change...
	{
		
		if(change == LEFT_MASK_B) 		// ...and this change looks like mask B...
		{
			
			if(ab == 0x00 || ab == LEFT_MASK)	// ...either 0 or left mask
			{
				enc_left++;			// increase left encoder
			}
			else
				enc_left--;			// otherwise decrease left encoder
		}
		else if(change == LEFT_MASK_A) 		// ...and this change looks like mask A...
		{
			if(ab == 0x00 || ab == LEFT_MASK)	// ...either 0 or left mask
				enc_left--;			// decrease left encoder
			else
				enc_left++;			// otherwise increase left encoder
		}

	}
	ab_old = ab;
}

void enc_cycle_right(void) 
{
	static uint8_t ab_old;
	uint8_t ab, change;

	ab = RIGHT_PORT & RIGHT_MASK;		// Read status of odometry right AND concatenate with the right mask
	change = ab ^ ab_old;				// XOR, i.e. changes
	if(change)							// If there is a change...
	{
		
		if(change == RIGHT_MASK_B) 		// ...and this change looks like mask B...
		{
			
			if(ab == 0x00 || ab == RIGHT_MASK)	// ...either 0 or right mask
			{
				enc_right--;			// decrease right encoder
				
			}			
			else
			{
				enc_right++;			// otherwise increase right encoder
			}
		}
		else if(change == RIGHT_MASK_A) 	// ...and this change looks like mask A...
		{
			if(ab == 0x00 || ab == RIGHT_MASK)	// ...either 0 or right mask
				enc_right++;			// increase right encoder
			else
				enc_right--;			// otherwise decrease right encoder
		}
	}
	ab_old = ab;

}
