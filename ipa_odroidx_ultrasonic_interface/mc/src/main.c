#include <avr/io.h>
#include <avr/interrupt.h>

#include "iocompat.h"

#define TRUE 1
#define FALSE 0

/*
*PORTA and PORTD are for output whereas PORTB and PORTC are for input
*The board is programmed to use the external crystal of 18.432MHz and
*the CLKDIV8 fuse is also programmed, causing effective frequency
*equal to 2304000
*/

volatile unsigned char PING_STAGE_0=FALSE;
volatile unsigned char PING_STAGE_1=FALSE;
volatile unsigned char PING_STAGE_2=FALSE;
volatile unsigned char PING_STAGE_3=FALSE;

int main(void){
	DDRD |= (1 << PD1)| (1 << PD3);
	PORTD &= (0 << PD1); // Setting Initial value
	PORTD &= (0 << PD3);

	TCNT0 = 1;
	TCCR0B |= (1 << CS01);

	TIMSK0 |= (1 << TOIE0); // Enabling timer 1 overflow
	sei(); // Setting global interrupt


	for(;;){
	}
}
ISR(TIMER0_OVF_vect){ // Timer 0 is dedicated for Pinging and listening.
	if(PING_STAGE_0 == FALSE){
		PORTD |= (1 << PD1);
		PORTD |= (1 << PD3);
		TCNT0 = 160; // Setting for 330us
		PING_STAGE_0 = TRUE;
		TCCR0B &= (0 << CS00);
		TCCR0B &= (0 << CS02);
		TCCR0B |= (1 << CS01);
	}
	else if(PING_STAGE_1 == FALSE){
		PORTD &= (0 << PD1);
		TCNT0 = 154; // Setting for 350us -0.02% error
		PING_STAGE_1 = TRUE;
	}
	else if(PING_STAGE_2 == FALSE){
		PORTD |= (1 << PD1);
		PORTD &= (0 << PD3);
		TCNT0 = 177; // Setting for 270us -0.02% error
		PING_STAGE_2 = TRUE;
	}
	else if(PING_STAGE_3 == FALSE){
		PORTD &= (0 << PD1);
		TCNT0 = 30; // Setting for 100ms
		TCCR0B &= (0 << CS01);
		TCCR0B |= (1 << CS00);
		TCCR0B |= (1 << CS02);
		PING_STAGE_0 = FALSE;
		PING_STAGE_1 = FALSE;
		PING_STAGE_2 = FALSE;
	}
}
