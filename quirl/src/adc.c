/*******************************************************************************
***  Fraunhofer IPA
***  Robotersysteme
***  Projekt: Quirl  (Quelle: Raser)
********************************************************************************
********************************************************************************
***  Author: Winfried Baum
*******************************************************************************/
 
/** \file adc.c
 * 
 * Functions to read analog values through ADC.
 * 
 * Version: 1.0  
 */

#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "adc.h"
#include "interfaces.h"

static uint16_t values[NCHAN];

void adc_init(void) 
{
	ADMUX &= ~(1<<MUX4);					// Reset MUX4 and MUX5 
	ADCSRB &= ~(1<<MUX5); 
	ADMUX |= (1<<REFS0); 					// internal 5-V-Reference
	ADCSRA = 1<<ADEN | 1<<ADPS2 | 1<<ADPS1 | 1<< ADPS0;	// Enable ADC, sets all Prescaler Bits to a divide factor of 128.
	//DIDR0 = DIDR2 = 0xff;					// Turn-off Digital Inputs
	ADCSRA |= 1<<ADSC;					// Start Conversion for initialization
}

void adc_cycle(void) {
	static uint8_t chan;

	if(!(ADCSRA & 1<<ADSC)) 				// If Start Conversion Bit is not set (Not busy)
	{
		values[chan] = ADC;				// Write last conversion result in conversion array
		chan++;						// Increase counter
    
		if(chan == NCHAN) 				// If Counter = Number of used channels
		{
			chan = 0;				// Reset counter
		}
		
		if(chan<6) ADCSRB &= ~(1<<MUX5);		//Condition added to store ACD14 reading on values[6]
		else ADCSRB |= (1<<MUX5);			//(Easy addition of ADC15 on values[7] possible)
		
		ADMUX = ADMUX & ~0x07 | chan & 0x07;		// Adjust lower 3 Bits of ADMUX to counter
				
		
		ADCSRA |= 1<<ADSC;				// Start conversion
		
	}
}

uint16_t adc_read(uint8_t chan) 
{
	uint16_t value;

	cli();
	value = values[chan];					// Delivers current channel value

	sei();
	return value;
}
