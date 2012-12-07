#include <avr/io.h>
#include <avr/interrupt.h>

#include "iocompat.h"

#define MAX_NUMBER_OF_VALUES 20

struct TIME_KEEPER{
	uint8_t port_val;
	uint16_t time_reg_val;
};

/*
 *PORTA and PORTD are for output whereas PORTB and PORTC are for input
 *The board is programmed to use the external crystal of 18.432MHz and
 *the CLKDIV8 fuse is also programmed, causing effective frequency
 *equal to 2304000
 *In PORTA_CONTROL and PORTD_CONTROL 1 denotes ping and 0 denotes listen,
 *whereas MSB is unused.
 */

volatile unsigned char PING_STAGE=0;

volatile unsigned char PORTA_CONTROL=0x7f;
volatile unsigned char PORTD_CONTROL=0x33;

volatile struct TIME_KEEPER PORT_B_VALS[MAX_NUMBER_OF_VALUES]; 
volatile struct TIME_KEEPER PORT_C_VALS[MAX_NUMBER_OF_VALUES];

volatile uint8_t COUNT_PORT_B = 0;
volatile uint8_t COUNT_PORT_C = 0;

int main(void){
	//Setting PORTA and PORTD as output
	DDRA = 0x7F;
	DDRD = 0x7F;

	//Setting PORTB and PORTC as input
	DDRB = 0x00;
	DDRC = 0x00;

	//Setting initial values for PORTA and PORTC
	PORTA = 0;
	PORTD = 0;

	PORTB = 0x00; //Disabling use of pull up resistors
	PORTC = 0x00;

	TCNT0 = 1;
	TCCR0B |= (1 << CS01);

	TIMSK0 |= (1 << TOIE0); // Enabling timer 1 overflow
	sei(); // Setting global interrupt

	for(;;){
	}
}
ISR(TIMER0_OVF_vect){ // Timer 0 is dedicated for Pinging and listening.
	if(PING_STAGE == 0){
		PORTA = 0x7F;
		PORTD = 0x7F;
		TCNT0 = 160; // Setting for 330us
		PING_STAGE = 1;
		TCCR0B &= ~(1 << CS00);
		TCCR0B &= ~(1 << CS02);
		TCCR0B |= (1 << CS01);
	}
	else if(PING_STAGE == 1){
		PORTA = ~(PORTA_CONTROL) & (0x7F);
		PORTD = ~(PORTD_CONTROL) & (0x7F);
		TCNT0 = 154; // Setting for 350us -0.02% error
		PING_STAGE = 2;
	}
	else if(PING_STAGE == 2){
		PORTA = PORTA_CONTROL;
		PORTD = PORTD_CONTROL;
		TCNT0 = 177; // Setting for 270us -0.02% error
		PING_STAGE = 3;
	}
	else if(PING_STAGE == 3){
		PORTA = 0x00;
		PORTD = 0x00;
		//Disabling timer0 to setup timer1 to enable data reading from sensors.
		TCCR0B &= ~(1 << CS01);
		TCCR0B &= ~(1 << CS00);
		TCCR0B &= ~(1 << CS02);
		TIMSK0 &= ~(1 << TOIE0);

		//Setting up timer1 for 100ms
		TCNT1 = 36735;
		TCCR1B |= (1 << CS11);
		TIMSK |= (1 << TOIE1);
		// Place to enable PCINT
		//
		//
	}
}
ISR(TIMER1_OVF_vect){
	//Disabling timer1
	TCCR1B &= ~(1 << CS11);
	TIMSK &= ~(1 << TOIE1);

	TCNT0 = 255; // Initializing timer 0 to trigger.
	TCCR0B &= ~(1 << CS01);
	TCCR0B |= (1 << CS00);
	TIFR0 |= (1 << TOV0); //Forced timer0 interrupt trigger.
	TIMSK0 |= (1 << TOIE0);
	PING_STAGE = 0;

}

