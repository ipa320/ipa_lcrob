#include <avr/io.h>
#include <avr/interrupt.h>

#include "iocompat.h"
#include "softuart.h"

#define MAX_VALUES 32
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

volatile  uint8_t PING_STAGE=0;

volatile unsigned char PORTA_CONTROL=0x00;
volatile unsigned char PORTD_CONTROL=0x33;

volatile struct TIME_KEEPER PC7_VALS[32];
volatile struct TIME_KEEPER PB6_VALS[32];
volatile struct TIME_KEEPER PB4_VALS[32];
volatile struct TIME_KEEPER PB3_VALS[32];
volatile struct TIME_KEEPER PB2_VALS[32];
volatile struct TIME_KEEPER PB1_VALS[32];
volatile struct TIME_KEEPER PB0_VALS[32];

volatile struct TIME_KEEPER PD7_VALS[32];
volatile struct TIME_KEEPER PC2_VALS[32];
volatile struct TIME_KEEPER PC3_VALS[32];
volatile struct TIME_KEEPER PC4_VALS[32];
volatile struct TIME_KEEPER PC5_VALS[32];
volatile struct TIME_KEEPER PC6_VALS[32];


volatile uint8_t PC7_VALS_count = 0;
volatile uint8_t PB6_VALS_count = 0;
volatile uint8_t PB4_VALS_count = 0;
volatile uint8_t PB3_VALS_count = 0;
volatile uint8_t PB2_VALS_count = 0;
volatile uint8_t PB1_VALS_count = 0;
volatile uint8_t PB0_VALS_count = 0;

volatile uint8_t PD7_VALS_count = 0;
volatile uint8_t PC2_VALS_count = 0;
volatile uint8_t PC3_VALS_count = 0;
volatile uint8_t PC4_VALS_count = 0;
volatile uint8_t PC5_VALS_count = 0;
volatile uint8_t PC6_VALS_count = 0;

volatile uint8_t CYCLE_COMPLETE = 0;

void print_val(struct TIME_KEEPER val){
	softuart_putchar(val.port_val);
	softuart_putchar((uint8_t)((val.time_reg_val & 0xFF00)>> 8));
	softuart_putchar((uint8_t)(val.time_reg_val & 0x00FF));
}

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

	softuart_init();

	sei(); // Setting global interrupt

	for(;;){
		if(CYCLE_COMPLETE == 1){
			for (uint8_t count = 0; count < PC7_VALS_count; count++){
				print_val(PC7_VALS[count]);
			}
			PC7_VALS_count = 0;
			
			for (uint8_t count = 0; count < PB6_VALS_count; count++){
				print_val(PB6_VALS[count]);
			}
			PB6_VALS_count = 0;

			for (uint8_t count = 0; count < PB4_VALS_count; count++){
				print_val(PB4_VALS[count]);
			}
			PB4_VALS_count = 0;

			for (uint8_t count = 0; count < PB3_VALS_count; count++){
				print_val(PB3_VALS[count]);
			}
			PB3_VALS_count = 0;

			for (uint8_t count = 0; count < PB2_VALS_count; count++){
				print_val(PB2_VALS[count]);
			}
			PB2_VALS_count = 0;

			for (uint8_t count = 0; count < PB1_VALS_count; count++){
				print_val(PB1_VALS[count]);
			}
			PB1_VALS_count = 0;

			for (uint8_t count = 0; count < PB0_VALS_count; count++){
				print_val(PB0_VALS[count]);
			}
			PB0_VALS_count = 0;

			//After All Values have been sent out to the master.
			CYCLE_COMPLETE = 0;

			TCNT0 = 254; // Initializing timer 0 to trigger.
			TCCR0B |= (1 << CS01);
		//	TCCR0B |= (1 << CS00);
			TIFR0 |= (1 << TOV0); //Forced timer0 interrupt trigger.
			TIMSK0 |= (1 << TOIE0);
		}
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
		// For PA6-PC7
		if((~(PORTA_CONTROL) & (1<<6))==0x40) {
			PCMSK2 |= (1 << PCINT23);
			PCICR |= (1 << PCIE2);
		}
		//For PA5-PB6
		if ((~(PORTA_CONTROL) & (1<<5))==0x20){
			PCMSK1 |= (1 << PCINT14);
			PCICR |= (1 << PCIE1);
		}

		//For PA[4:0]-PB[4:0]
		PCMSK1 |= (0x1F &  ~(PORTA_CONTROL));
		if ((~(PORTA_CONTROL) & 0x1F) >0 )
			PCICR |= (1 << PCIE1);


	}	
}
ISR(TIMER1_OVF_vect){
	//Disabling timer1
	TCCR1B &= ~(1 << CS11);
	TIMSK &= ~(1 << TOIE1);

	PING_STAGE = 0;

	CYCLE_COMPLETE = 1;

	//Disbaling all PCINTs
	PCMSK0 = 0x00;
	PCMSK1 = 0x00;
	PCMSK2 = 0x00;
	PCICR &= 0xF0;


}

ISR(PCINT2_vect){
	if((~(PORTA_CONTROL) & (1<<6))==0x40) {
		if(PC7_VALS_count < MAX_VALUES){
			PC7_VALS[PC7_VALS_count].port_val=PINC;
			PC7_VALS[PC7_VALS_count].time_reg_val = TCNT1 - 36735;
			PC7_VALS_count++;
		}
	}
}
ISR(PCINT1_vect){
	if((~(PORTA_CONTROL) & (1<<5))==0x20) {
		if(PB6_VALS_count < MAX_VALUES){
			PB6_VALS[PB6_VALS_count].port_val=PINB;
			PB6_VALS[PB6_VALS_count].time_reg_val = TCNT1 - 36735;
			PB6_VALS_count++;
		}
	}

	if((~(PORTA_CONTROL) & (1<<4))==0x10) {
		if(PB4_VALS_count < MAX_VALUES){
			PB4_VALS[PB4_VALS_count].port_val=PINB;
			PB4_VALS[PB4_VALS_count].time_reg_val = TCNT1 - 36735;
			PB4_VALS_count++;
		}
	}
	if((~(PORTA_CONTROL) & (1<<3))==0x08) {
		if(PB3_VALS_count < MAX_VALUES){
			PB3_VALS[PB3_VALS_count].port_val=PINB;
			PB3_VALS[PB3_VALS_count].time_reg_val = TCNT1 - 36735;
			PB3_VALS_count++;
		}
	}
	if((~(PORTA_CONTROL) & (1<<2))==0x04) {
		if(PB2_VALS_count < MAX_VALUES){
			PB2_VALS[PB2_VALS_count].port_val=PINB;
			PB2_VALS[PB2_VALS_count].time_reg_val = TCNT1 - 36735;
			PB2_VALS_count++;
		}
	}
	if((~(PORTA_CONTROL) & (1<<1))==0x02) {
		if(PB1_VALS_count < MAX_VALUES){
			PB1_VALS[PB1_VALS_count].port_val=PINB;
			PB1_VALS[PB1_VALS_count].time_reg_val = TCNT1 - 36735;
			PB1_VALS_count++;
		}
	}
	if((~(PORTA_CONTROL) & (1<<0))==0x01) {
		if(PB0_VALS_count < MAX_VALUES){
			PB0_VALS[PB0_VALS_count].port_val=PINB;
			PB0_VALS[PB0_VALS_count].time_reg_val = TCNT1 - 36735;
			PB0_VALS_count++;
		}
	}
}
