
//*****************************************************************************
//
// File Name	: 'global.h'
// Title		: AVR project global include 
// Author		: Pascal Stang
// Created		: 7/12/2001
// Revised		: 9/30/2002
// Version		: 1.1
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
//	Description : This include file is designed to contain items useful to all
//					code files and projects.
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#ifndef GLOBAL_H
#define GLOBAL_H

// global AVRLIB defines
#include "avrlibdefs.h"
// global AVRLIB types definitions
#include "avrlibtypes.h"

// project/system dependent defines

// CPU clock speed
//#define F_CPU        16000000               		// 16MHz processor
//#define F_CPU        14745000               		// 14.745MHz processor
//#define F_CPU        8000000               		// 8MHz processor
#define F_CPU        2372800               		// 7.37MHz processor
//#define F_CPU        4000000               		// 4MHz processor
//#define F_CPU        3686400               		// 3.69MHz processor

#define CYCLES_PER_US ((F_CPU+500000)/1000000) 	// cpu cycles per microsecond

/*
HERE COME MY HACKS FOR ATMega186P(A)
*/

#define SIG_OVERFLOW0	TIMER0_OVF_vect
#define SIG_OVERFLOW1	TIMER1_OVF_vect
#define SIG_OVERFLOW2	TIMER2_OVF_vect

#define SIG_OUTPUT_COMPARE0A	TIMER0_COMPA_vect
#define SIG_OUTPUT_COMPARE0B	TIMER0_COMPB_vect
#define SIG_OUTPUT_COMPARE1A	TIMER1_COMPA_vect
#define SIG_OUTPUT_COMPARE1B	TIMER1_COMPB_vect
#define SIG_OUTPUT_COMPARE2A	TIMER2_COMPA_vect
#define SIG_OUTPUT_COMPARE2B	TIMER2_COMPB_vect

#define SIG_INPUT_CAPTURE1	TIMER1_CAPT_vect

#define SIG_ADC			ADC_vect

#define SIG_INTERRUPT0		INT0_vect
#define SIG_INTERRUPT1		INT1_vect


#endif
