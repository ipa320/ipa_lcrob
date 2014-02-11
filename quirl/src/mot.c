/*******************************************************************************
***  Fraunhofer IPA
***  Robotersysteme
***  Projekt: Quirl
********************************************************************************
********************************************************************************
***  Author: Maik Siee
********************************************************************************
 ** \file mot.c
 *  
 *  Motor Driver
 *  
 *  Provides the motor-control for the Quirl-drive
 *  version 1.0
 */

#include "interfaces.h"
#include "mot.h"
//#include "odo.h"
#include <math.h>
#include "utils.h"

#include <stdio.h>
#include "usart.h"
char s[50];					///< String variable for output.


void mot_init(void)
{					// Clear OC1A & OC1B on Compare Match - Set OC1A & OC1B at BOTTOM (non inverting),
	TCCR1A = B(10100010);		// Fast PWM-Mode 9-bit (PWM @ 28.8 [kHz]), (OC1C disconnected)
	TCCR1B = B(00001001);		// No Prescaling
	OCR1A = 0;			// Clear Output Compare Register A
	OCR1B = 0;			// Clear Output Compare Register B
}

void set_left(int16_t vel)
{
// PWM is OC1A (PB5)
// Direction with PH4, PH5
// Set one output, the other low
	uint16_t v;
	
  	if(vel > 0)			//without the need to readjust the PI Controller
	{                  		// set direction
		PORTH |=  (1 << PH4);	// Set PH5   ...backw left
		PORTH &= ~(1 << PH5);	// Reset PH4 ...forw left
		v = (uint16_t)vel;
	}
	else
	{				// other direction
		PORTH &= ~(1 << PH4);	// Reset PH5
		PORTH |=  (1 << PH5);	// Set PH4
		v = (uint16_t)(-1*vel);
	}


  	OCR1A = v;		// and the speed in Output Compare Register A of Timer 1
}

void set_right(int16_t vel)
{
// PWM is OC1B (PB6)
// Direction with PH6, PH7
// Set one output, the other low
	uint16_t v;
	
	if(vel > 0)			//without the need to readjust the PI Controller
	{				// set direction
		PORTH &= ~(1 << PH6);	// Reset PH6
		PORTH |=  (1 << PH7);	// Set PH7
		v = (uint16_t)vel;
	}
	else 
	{				// other direction
		PORTH |=  (1 << PH6);	// Set PH6
		PORTH &= ~(1 << PH7);	// Reset PH7
		v = (uint16_t)(-1*vel);
	}

	OCR1B = v;	// and the speed in Output Compare Register B of Timer 1
}



/*
void mot_ctrl(int8_t soll_l, int8_t soll_r) 
{
	//Encoder: 16 holes -> 64 Impulses/revolution -> 256 flank change/revolution
	//Gear reduction: 28:1 -> 7168 flank change per wheel rotation
	
	//Each motor revolution 5,32 -> each encoder signal 0,019mm
	

	static uint16_t left_old, right_old;
	static double x_l, x1_l, x_r, x1_r;
	int16_t delta;
	double y;
	double Kp=3.31, Ki=25;		//PI Setting values (Kp=1.8 Ki=30)
	double T=0.01;			//Time interval
	
	delta = enc_left - left_old;	//Calculates the change of position of the left wheel
	left_old = enc_left;		//Current position becomes the old position
	
	x_l = (double)(soll_l - delta);	//Difference between the reference and the output, e(t)
	
	yl_I += T*x_l;			//Integration of e(t)

	y = Kp*x_l + Ki*yl_I;		//PI controller output calculation for the left motor

	if(y > 1023) y=1023;						 
	if(y < -1023) y=-1023;		//Limit output value
	set_left((int16_t)y);		//Send output to left motor

	x1_l = x_l;			//Current e(t) becomes old one

	
	Kp=3.37; 
	delta = enc_right - right_old;	//Calculates the change of position of the right wheel
	right_old = enc_right;		//Current position becomes the old position

	x_r = (double)(soll_r - delta);	//Difference between the reference and the output gives e(t)
	
	yr_I += T*x_r;			//Integration of e(t)
	
	y = Kp*x_r + Ki*yr_I;		//PI controller output calculation for the right motor

	if(y > 1023) y=1023;						
	if(y < -1023) y=-1023;		//Limit output value
	set_right((int16_t)y);		//Send output to right motor

	x1_r = x_r;			//Current e(t) becomes old one

}
*/

void mot_ctrl(int16_t soll_l, int16_t soll_r) 
{
	//static int16_t left_old, right_old;
	float setVel;

	float leftdamp, rightdamp;	//drift compensation in case of unequal motor strength
	leftdamp  = 1.0;		//  left compensation factor
	rightdamp = 0.94;		// right compensation factor

//	sprintf(s,"\n int16_t soll_l, %d\t soll_r %d\n",soll_l, soll_r );
//	usart0_puts(s);
//	usart0_sendbyte(0x0D);


	setVel = leftdamp * soll_l * 511;
	if(setVel >  511) setVel= 511;						
	if(setVel < -511) setVel=-511;			//Limit output value
	set_left((int16_t)setVel);			//Send output to left motor

//		sprintf(s,"\n Send output to left motor %d\n",(int16_t)setVel);
//		usart0_puts(s);
//		usart0_sendbyte(0x0D);


	setVel =  rightdamp * soll_r * 511;
	if(setVel >  511) setVel= 511;						
	if(setVel < -511) setVel=-511;			//Limit output value
	set_right((int16_t)setVel);			//Send output to right motor

//		sprintf(s,"\n Send output to right motor %d\n",(int16_t)setVel);
//		usart0_puts(s);
//		usart0_sendbyte(0x0D);

		delay(1);
	
	//left_old = fabs(soll_l);

	//right_old = fabs(soll_r);

}





/*
void mot_ctrl(int8_t direction, int16_t vel) 
{

	static int16_t oldVel;
	float setVel, absVel;

	int8_t  sign;
 	int16_t ramp;

	float leftdamp, rightdamp;	//fix drift compensation in case of unequal motor strength

	leftdamp  = 1.0;		//  left compensation factor
	rightdamp = 0.94;		// right compensation factor
	

	absVel = (int16_t)fabs(vel);
	sign   = (int8_t)vel/absVel;  
	if(sign == 0) sign=1;

	switch(direction)	
	{
		case STOP:		// both motors stops immediately

			sprintf(s,"Mot STOP %d\n",0);
			usart0_puts(s);
			usart0_sendbyte(0x0D);

			OCR1A = 0;		// Clear Output Compare Register A
			OCR1B = 0;		// Clear Output Compare Register B

			oldVel = 0;
			break;

		case FORW:		// straight forwards/backwards 

			sprintf(s,"\n Mot forwards/backwards %d\n",(int16_t)oldVel);
			usart0_puts(s);
			usart0_sendbyte(0x0D);

			ramp = oldVel;

			while(ramp != absVel)
			{
				if (ramp < absVel) ramp++;
				else ramp--;
	
				setVel = leftdamp * (float)ramp * (float)sign;
				set_left((int16_t)setVel);		//Send output to left motor

				setVel =  rightdamp * (float)ramp * (float)sign;
				set_right((int16_t)setVel);		//Send output to right motor


			sprintf(s,"\n setVel %d\n",(int16_t)setVel);
			usart0_puts(s);
			usart0_sendbyte(0x0D);

				delay(1);
			}
			oldVel = fabs(vel);


			sprintf(s,"\n Mot forwards/backwards %d\n",(int16_t)setVel);
			usart0_puts(s);
			usart0_sendbyte(0x0D);

			break;


		case LEFT:		// turn left

			sprintf(s,"\n turn left %d\n",vel);
			usart0_puts(s);
			usart0_sendbyte(0x0D);

			for(ramp=oldVel ; ramp <= absVel; ramp++)
			{

				set_left(0);				//Send output to left motor

				setVel = ramp * sign;
				set_right((int16_t)setVel);		//Send output to right motor

				delay(1);
				
			}
			oldVel = 0;
			break;


		case RIGHT:		// turn right

			sprintf(s,"\n turn right %d\n",vel);
			usart0_puts(s);
			usart0_sendbyte(0x0D);

			for(ramp=oldVel ; ramp <= absVel; ramp++)
			{

				setVel = ramp * sign;
				set_left((int16_t)setVel);		//Send output to left motor

				set_right(0);				//Send output to right motor

				delay(1);
				
			}
			oldVel = 0;
			break;

		case CIRCLE:		// rotate around own axis

			sprintf(s,"\n Mot CIRCLE %d\n",vel);
			usart0_puts(s);
			usart0_sendbyte(0x0D);

			for(ramp=oldVel ; ramp <= absVel; ramp++)
			{

				setVel = ramp * sign;
				set_left((int16_t)setVel);				//Send output to left motor

				setVel = ramp * sign;
				set_left((int16_t)-setVel);				//Send output to right motor

				delay(1);				
			}
			oldVel = fabs(vel);
			break;


	}

}


*/

void vrad_ctrl(int8_t soll_v, double v_phi)
{
	int8_t soll_r, soll_l;
	
	soll_r = soll_v * (cos(v_phi) + sin(v_phi)); 		// Revolution of virtual wheel -> Left motor setup
	soll_l = soll_v * (cos(v_phi) - sin(v_phi)); 		// Revolution of virtual wheel -> Right motor setup

	mot_ctrl(soll_l, soll_r);


}



