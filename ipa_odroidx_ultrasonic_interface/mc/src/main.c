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
volatile unsigned char PORTD_CONTROL=0x00;

volatile struct TIME_KEEPER PORTA_INPUT_VALS[32];
volatile uint8_t PORTA_INPUT_count=0;

volatile struct TIME_KEEPER PORTD_INPUT_VALS[32];
volatile uint8_t PORTD_INPUT_count=0;

volatile struct TIME_KEEPER PORTC_INPUT_VALS[32];
volatile uint8_t PORTC_INPUT_count=0;

volatile struct TIME_KEEPER PORTB_INPUT_VALS[32];
volatile uint8_t PORTB_INPUT_count=0;

volatile uint16_t TIMER[MAX_VALUES];
volatile uint8_t TIMER_count=0;

volatile uint8_t LAST_VALUE=0;

volatile uint8_t CYCLE_COMPLETE = 0;

void print_TIMER(){
	softuart_putchar(TIMER_count);
	for (uint8_t count=0; count <TIMER_count; count++){
		softuart_putchar((uint8_t)((TIMER[count] & 0xFF00)>> 8));
		softuart_putchar((uint8_t)(TIMER[count] & 0x00FF));
	}
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
			LAST_VALUE=0;
			//For PA6-PC7
			if ((~(PORTA_CONTROL) & 0x40)== 0x40){
				for (uint8_t count=0; count <PORTC_INPUT_count; count++){
					if((0x80 & PORTC_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTC_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x80 & PORTC_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}

			//For PA5-PB6
			if ((~(PORTA_CONTROL) & 0x20) == 0x20){
				for (uint8_t count=0; count <PORTB_INPUT_count; count++){
					if((0x40 & PORTB_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTB_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x40 & PORTB_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}

			//For PA4-PB4
			if ((~(PORTA_CONTROL) & 0x10) == 0x10){
				for (uint8_t count=0; count <PORTB_INPUT_count; count++){
					if((0x10 & PORTB_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTB_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x10 & PORTB_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}

			//For PA3-PB3
			if ((~(PORTA_CONTROL) & 0x08) == 0x08){
				for (uint8_t count=0; count <PORTB_INPUT_count; count++){
					if((0x08 & PORTB_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTB_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x08 & PORTB_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}

			//For PA2-PB2
			if ((~(PORTA_CONTROL) & 0x04) == 0x04){
				for (uint8_t count=0; count <PORTB_INPUT_count; count++){
					if((0x04 & PORTB_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTB_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x04 & PORTB_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}

			//For PA1-PB1
			if ((~(PORTA_CONTROL) & 0x02) == 0x02){
				for (uint8_t count=0; count <PORTB_INPUT_count; count++){
					if((0x02 & PORTB_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTB_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x02 & PORTB_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}

			//For PA0-PB0
			if ((~(PORTA_CONTROL) & 0x01) == 0x01){
				for (uint8_t count=0; count <PORTB_INPUT_count; count++){
					if((0x01 & PORTB_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTB_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x01 & PORTB_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}
			
			//For PD6-PD7
			if ((~(PORTD_CONTROL) & 0x40) == 0x40){
				for (uint8_t count=0; count <PORTD_INPUT_count; count++){
					if((0x80 & PORTD_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTD_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x80 & PORTD_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}
			
			//For PD5-PC2
			if ((~(PORTD_CONTROL) & 0x20) == 0x20){
				for (uint8_t count=0; count <PORTD_INPUT_count; count++){
					if((0x02 & PORTC_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTC_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x02 & PORTC_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}
			
			//For PD4-PC3
			if ((~(PORTD_CONTROL) & 0x10) == 0x10){
				for (uint8_t count=0; count <PORTD_INPUT_count; count++){
					if((0x04 & PORTC_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTC_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x04 & PORTC_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}
			
			//For PD3-PC4
			if ((~(PORTD_CONTROL) & 0x08) == 0x08){
				for (uint8_t count=0; count <PORTD_INPUT_count; count++){
					if((0x10 & PORTC_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTC_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x10 & PORTC_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}
			
			//For PD2-PC5
			if ((~(PORTD_CONTROL) & 0x04) == 0x04){
				for (uint8_t count=0; count <PORTD_INPUT_count; count++){
					if((0x20 & PORTC_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTC_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x20 & PORTC_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}
			
			//For PD1-PC6
			if ((~(PORTD_CONTROL) & 0x02) == 0x02){
				for (uint8_t count=0; count <PORTD_INPUT_count; count++){
					if((0x40 & PORTC_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTC_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x40 & PORTC_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}
			
			//For PD0-PA7
			if ((~(PORTD_CONTROL) & 0x01) == 0x01){
				for (uint8_t count=0; count <PORTD_INPUT_count; count++){
					if((0x80 & PORTA_INPUT_VALS[count].port_val) != LAST_VALUE){
						TIMER[TIMER_count++] = PORTA_INPUT_VALS[count].time_reg_val;
						LAST_VALUE = (0x80 & PORTA_INPUT_VALS[count].port_val);
					}
				}
				//Sending data out
				if (TIMER_count>0){
					print_TIMER();
					TIMER_count=0;
				}
				LAST_VALUE = 0;
			}
			

			//After All Values have been sent out to the master.
			PORTA_INPUT_count=0;
			PORTB_INPUT_count=0;
			PORTC_INPUT_count=0;
			PORTD_INPUT_count=0;

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

		//For PD0-PA7
		if ((~(PORTD_CONTROL) & 0x01) == 0x01){
			PCMSK0 |= (1 << PCINT7);
			PCICR |= (1 <<PCIE0);
		}

		//For PD1-PC6
		if ((~(PORTD_CONTROL) & 0x02) == 0x02){
			PCMSK2 |= (1 << PCINT22);
			PCICR |= (1<< PCIE2);
		}

		//For PD2-PC5
		if ((~(PORTD_CONTROL) & 0x04) == 0x04){
			PCMSK2 |= (1 << PCINT21);
			PCICR |= (1 << PCIE2);
		}

		//For PD3-PC4
		if ((~(PORTD_CONTROL) & 0x08) == 0x08){
			PCMSK2 |= (1 << PCINT20);
			PCICR |= (1 << PCIE2);
		}

		//For PD4-PC3
		if ((~(PORTD_CONTROL) & 0x10) == 0x10){
			PCMSK2 |= (1 << PCINT19);
			PCICR |= (1 << PCIE2);
		}

		//For PD5-PC2
		if ((~(PORTD_CONTROL) & 0x20) == 0x20){
			PCMSK2 |= (1 << PCINT18);
			PCICR |= (1 << PCIE2);
		}

		//For PD6-PD7
		if ((~(PORTD_CONTROL) & 0x40) == 0x40){
			PCMSK3 |= (1 << PCINT31);
			PCICR |= (1 << PCIE3);
		}

	}	
}
ISR(TIMER1_OVF_vect){
	//Disabling timer1
	TCCR1B &= ~(1 << CS11);
	TIMSK &= ~(1 << TOIE1);

	PING_STAGE = 0;

	//Trigger time send
	CYCLE_COMPLETE = 1;

	//Disbaling all PCINTs
	PCMSK0 = 0x00;
	PCMSK1 = 0x00;
	PCMSK2 = 0x00;
	PCICR &= 0xF0;
}

ISR(PCINT2_vect){
	if(((~(PORTA_CONTROL) & (1<<6))==0x40) ||((~(PORTD_CONTROL) & 0X3E)>0)){
		if(PORTC_INPUT_count < MAX_VALUES){
			PORTC_INPUT_VALS[PORTC_INPUT_count].port_val=PINC;
			PORTC_INPUT_VALS[PORTC_INPUT_count].time_reg_val = TCNT1 - 36735;
			PORTC_INPUT_count++;
		}
	}
}
ISR(PCINT1_vect){
	if((~(PORTA_CONTROL) & (0x3F))>0) {
		if(PORTB_INPUT_count < MAX_VALUES){
			PORTB_INPUT_VALS[PORTB_INPUT_count].port_val=PINB;
			PORTB_INPUT_VALS[PORTB_INPUT_count].time_reg_val = TCNT1 - 36735;
			PORTB_INPUT_count++;
		}
	}
}

ISR(PCINT0_vect){
	if((~(PORTD_CONTROL) & 0x01)==0x01) {
		if(PORTA_INPUT_count < MAX_VALUES){
			PORTA_INPUT_VALS[PORTA_INPUT_count].port_val=PINA;
			PORTA_INPUT_VALS[PORTA_INPUT_count].time_reg_val = TCNT1 - 36735;
			PORTA_INPUT_count++;
		}
	}
}

ISR(PCINT3_vect){
	if((~(PORTD_CONTROL) & 0x40)==0x40) {
		if(PORTD_INPUT_count < MAX_VALUES){
			PORTD_INPUT_VALS[PORTD_INPUT_count].port_val=PIND;
			PORTD_INPUT_VALS[PORTD_INPUT_count].time_reg_val = TCNT1 - 36735;
			PORTD_INPUT_count++;
		}
	}
}
