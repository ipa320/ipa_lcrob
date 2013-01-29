#include <avr/io.h>
#include <avr/interrupt.h>
#include "helper.h"
#include "softuart.h"
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

	softuart_init();

	//prescalers
	TCCR0B |= (1 << CS01);
	TCCR1B |= (1 << CS11);

	// Place to enable PCINT
	//

	PCMSK0 |= 0x80;
	PCMSK1 |= 0x5F;
	PCMSK2 |= 0xFE;
	PCMSK3 |= 0x80;

	sei(); // Setting global interrupt

	for(;;){
		if(CURRENT_SENSOR_CONFIG < TOTAL_SENSOR_CONFIGS){ //Traversing through each sensor config sequentially.
			if(CONTROL_PORTS_SET==1){ // If current config has been loaded.
				if(CYCLE_COMPLETE == 1){ //After PORT[A-D]_TIMER_VALS have been populated.
					softuart_putchar(CURRENT_SENSOR_CONFIG);//Sending out current sensor configuration number.
					printPORTD();
					printPORTA();

					//After All Values have been sent out to the master.
					PORTA_INPUT_count=0;
					PORTB_INPUT_count=0;
					PORTC_INPUT_count=0;
					PORTD_INPUT_count=0;

					CONTROL_PORTS_SET =0; //To load the new configuration.
					CURRENT_SENSOR_CONFIG++;
				}
			}
			else{
				PORTA_CONTROL = (uint8_t)(SENSOR_CONFIG[CURRENT_SENSOR_CONFIG] & 0x00FF);
				PORTD_CONTROL = (uint8_t)(SENSOR_CONFIG[CURRENT_SENSOR_CONFIG] >> 8);
				CONTROL_PORTS_SET=1;

				CYCLE_COMPLETE = 0;

				TCNT0 = 1; //Loading random overflow value.
				TIFR0 |= (1 << TOV0); //Forced timer0 interrupt trigger.
				TIMSK0 |= (1 << TOIE0); // Enabling timer 1 overflow

			}
		}
		else{
			if(softuart_kbhit()){ //To check if any new configuration is received from the master.
				//Reading config from serial port
				uint8_t number_of_config = 0;
				number_of_config = softuart_getchar(); //Should be less than 16, however no error detection.
				if(number_of_config >0){
					uint16_t temp16 = 0; // 16bit variable to store configuration input from the master.
					uint8_t temp8 = 0;
					for (uint8_t count = 0; count < number_of_config; count++){
						temp16 = (softuart_getchar() << 8);
						temp8 = softuart_getchar();
						SENSOR_CONFIG[count] = (temp16 | temp8) ;
					}
					TOTAL_SENSOR_CONFIGS = number_of_config;
					CURRENT_SENSOR_CONFIG = 0;
					softuart_putchar(0x12); //Sending acknowledgement back to the master.
				}
			}
			else if(TOTAL_SENSOR_CONFIGS > 0){
				CURRENT_SENSOR_CONFIG = 0; // Resetting the configuration
			}
		}
	}
}
ISR(TIMER0_OVF_vect){ // Timer 0 is dedicated for Pinging and listening.
	if(PING_STAGE == 0){
		PORTA = 0x7F;
		PORTD = 0x7F;
		TCNT0 = 160; // Setting for 330us
		PING_STAGE = 1;
	}
	else if(PING_STAGE == 1){
		PORTA = (~PORTA_CONTROL) & (0x7F);
		PORTD = (~PORTD_CONTROL) & (0x7F);
		TCNT0 = 154; // Setting for 350us -0.02% error
		PING_STAGE = 2;

		//Setting up timer1 for 100ms
		TCNT1 = 36735;
		TIMSK1 |= (1 << TOIE1);

		PCICR |= 0x0F; // Enabling input interrupts.
	}
	else if(PING_STAGE == 2){
		PORTA = 0x00;
		PORTD = 0x00;
		//Disabling timer0 to setup timer1 to enable data reading from sensors.
		TIMSK0 &= ~(1 << TOIE0);

//		softuart_disable(); // Not sure if works or not.

	}	
}
ISR(TIMER1_OVF_vect){
	//Disabling timer1
	TIMSK1 &= ~(1 << TOIE1);

	PING_STAGE = 0;

	//Trigger time send
	CYCLE_COMPLETE = 1;

	//Disbaling all PCINTs
	PCICR &= 0xF0;

//	softuart_enable(); //Not sure if works or not 
}

ISR(PCINT0_vect){
	if(PORTA_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTA)){
		PORTA_INPUT_VALS[PORTA_INPUT_count].port_val=PINA;
		PORTA_INPUT_VALS[PORTA_INPUT_count].time_reg_val = TCNT1 - 36735; // Subtracting the 100ms offset before recording the Timer1 value.
		PORTA_INPUT_count++;
	}
}

ISR(PCINT1_vect){
	if(PORTB_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTB)){
		PORTB_INPUT_VALS[PORTB_INPUT_count].port_val=PINB;
		PORTB_INPUT_VALS[PORTB_INPUT_count].time_reg_val = TCNT1 - 36735;
		PORTB_INPUT_count++;
	}
}

ISR(PCINT2_vect){
	if(PORTC_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTC)){
		PORTC_INPUT_VALS[PORTC_INPUT_count].port_val=PINC;
		PORTC_INPUT_VALS[PORTC_INPUT_count].time_reg_val = TCNT1 - 36735;
		PORTC_INPUT_count++;
	}
}

ISR(PCINT3_vect){
	if(PORTD_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTD)){
		PORTD_INPUT_VALS[PORTD_INPUT_count].port_val=PIND;
		PORTD_INPUT_VALS[PORTD_INPUT_count].time_reg_val = TCNT1 - 36735;
		PORTD_INPUT_count++;
	}
}

