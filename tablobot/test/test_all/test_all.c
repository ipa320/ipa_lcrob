/************************************************************************************************
 *	Main program of the Tabl-O-bot		   						
 *	Alejandro Merello									
 ************************************************************************************************/
/** \mainpage Tabl-O-Bot documentation
 *  \image html tablobot_logo.png
 *  \section intro_sec Introduction
 *  This is the robot's code documentation of project Tabl-O-bot. It is intended for internal use only.
 */
/** \file test_all.c
 *  \brief Main program of the Tabl-O-bot
 *  
 *  This file contains the program that runs the operation of the robot.
 *  \version 1.0
 *  \author Alejandro Merello
 */

#include <avr/interrupt.h>
#include "scheduler.h"

/**Runs the main code
 */
int main(void)
{
	sei();			//Interrupts on.
	sched_init();	//Initializes the scheduler.
	bluetooth();	//Bluetooth Communication Control.
	return(0);
}

