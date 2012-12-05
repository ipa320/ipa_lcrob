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
	DDRA |= (1 << PA1);
	PORTA |= (1 << PA1); // Setting Initial value

	TIMSK |= (1 << TOIE1); // Enabling timer 1 overflow
	sei(); // Setting global interrupt

	TCNT1 = 1;
	TCCR1B |= (1 << CS10);

	for(;;){
	}
}
ISR(TIMER1_OVF_vect){ // Timer 1 is dedicated for Pinging.
	if(PING_STAGE_0 == FALSE){
		PORTA |= (1 << PA1);
		TCNT1 = 64775; // Setting for 330us
		PING_STAGE_0 = TRUE;
		TCCR1B &= (0 << CS11);
		TCCR1B |= (1 << CS10);
	}
	else if(PING_STAGE_1 == FALSE){
		PORTA &= (0 << PA1);
		TCNT1 = 64729; // Setting for 350us
		PING_STAGE_1 = TRUE;
	}
	else if(PING_STAGE_2 == FALSE){
		PORTA |= (1 << PA1);
		TCNT1 =  64913; // Setting for 270us
		PING_STAGE_2 = TRUE;
	}
	else if(PING_STAGE_3 == FALSE){
		PORTA &= (0 << PA1);
		TCNT1 = 14126; // Setting for 50m-330u-350u-270u
		TCCR1B &= (0 << CS10);
		TCCR1B |= (1 << CS11);
		PING_STAGE_0 = FALSE;
		PING_STAGE_1 = FALSE;
		PING_STAGE_2 = FALSE;
	}
}
