//*****************************************************************************
// File Name	: basiciotest.c
// 
// Title		: example usage of basic input and output functions on the AVR
// Revision		: 1.0
// Notes		:
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
// 
// Revision History:
// When			Who			Description of change
// -----------	-----------	-----------------------
// 02-Feb-2003	pstang		Created the program
//*****************************************************************************

 
//----- Include Files ---------------------------------------------------------
#include <avr/io.h>		// include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support

#include "global.h"		// include our global settings
#include "protocol.h"		// include our global settings
//#include "uart.h"		// include uart function library
//#include "rprintf.h"	// include printf function library
//#include "timer.h"		// include timer function library (timing, PWM, etc)
//#include "vt100.h"		// include VT100 terminal support
//#include "encoder.h"	// include encoder driver


//----- Begin Code ------------------------------------------------------------
int main(void)
{
	init();

	while(1) {
		parse();	
	}
	
	return 0;
}

