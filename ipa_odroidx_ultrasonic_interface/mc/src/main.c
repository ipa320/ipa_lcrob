#include <avr/io.h>
#include <avr/interrupt.h>

#include "iocompat.h"
#include "softuart.h"

#define MAX_VALUES 32

#define MAX_INPUTS_PORTA 1
#define MAX_INPUTS_PORTB 6
#define MAX_INPUTS_PORTC 6
#define MAX_INPUTS_PORTD 1

#define MAX_SENSOR_CONFIGS 16

struct TIME_KEEPER{
	uint8_t port_val;
	uint16_t time_reg_val;
};

/*
 *PORTA and PORTD are for output whereas PORTB and PORTC are for input
 *The board is programmed to use the external crystal of 18.432MHz and
 *the CLKDIV8 fuse is also programmed, causing effective frequency
 *equal to 2304000.
 *In PORTA_CONTROL and PORTD_CONTROL 1 denotes ping and 0 denotes listen,
 *whereas MSB is unused.
 */

volatile  uint8_t PING_STAGE=0;

volatile unsigned char PORTA_CONTROL=0x00;
volatile unsigned char PORTD_CONTROL=0x00;

volatile struct TIME_KEEPER PORTA_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTA];
volatile uint8_t PORTA_INPUT_count=0;

volatile struct TIME_KEEPER PORTB_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTB];
volatile uint8_t PORTB_INPUT_count=0;

volatile struct TIME_KEEPER PORTC_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTC];
volatile uint8_t PORTC_INPUT_count=0;

volatile struct TIME_KEEPER PORTD_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTD];
volatile uint8_t PORTD_INPUT_count=0;

volatile uint16_t TIMER[MAX_VALUES];
volatile uint8_t TIMER_count=0;

volatile uint8_t CYCLE_COMPLETE = 0;

volatile uint16_t SENSOR_CONFIG[MAX_SENSOR_CONFIGS];
volatile uint8_t TOTAL_SENSOR_CONFIGS=0;
volatile uint8_t CURRENT_SENSOR_CONFIG=0;
volatile uint8_t CONTROL_PORTS_SET=0;

void print_TIMER(){
	softuart_putchar(TIMER_count);
	for (uint8_t count=0; count <TIMER_count; count++){
		softuart_putchar((uint8_t)((TIMER[count] & 0xFF00)>> 8));
		softuart_putchar((uint8_t)(TIMER[count] & 0x00FF));
	}
}
void populateTIMER_SEND(uint8_t PORT_CONTROL, uint8_t CHANNEL_POSITION, volatile struct TIME_KEEPER * time_keeper, uint8_t INPUT_count, uint8_t last_value, uint8_t sensor_address){
	uint8_t PING_CHECK=0;
	uint8_t last_value_temp=last_value;
	TIMER_count=0;
	for(uint8_t count=0; count < INPUT_count; count++){ 
		if((~(PORT_CONTROL) & CHANNEL_POSITION) == CHANNEL_POSITION){//If listening
			if(((last_value & time_keeper[count].port_val)==0x00) && (last_value_temp==last_value)){
				if(TIMER_count < MAX_VALUES)
					TIMER[TIMER_count++] = time_keeper[count].time_reg_val;
			}
			last_value_temp = (last_value & time_keeper[count].port_val);
		}
		else { // Pinging
			if(((last_value & time_keeper[count].port_val) == last_value) && (PING_CHECK==0)){
				TIMER[TIMER_count++] = time_keeper[count].time_reg_val;
				PING_CHECK = 1;
			}
			else if (((last_value & time_keeper[count].port_val)==0x00) && (last_value_temp==last_value) && (PING_CHECK==1)){
				if(TIMER_count < MAX_VALUES)
					TIMER[TIMER_count++] = time_keeper[count].time_reg_val;
			}
			last_value_temp = (last_value & time_keeper[count].port_val);
		}
	}
	//Sending the data out
	if (TIMER_count>0){
		softuart_putchar((sensor_address<<4) | TIMER_count); //Address + TIMER value
		print_TIMER();
	}
	TIMER_count=0;
}

void printPORTA(){
	//For PA6-PC7
	//	POSITION ON PORT=0x80;
	populateTIMER_SEND(PORTA_CONTROL, 0x40, PORTC_INPUT_VALS, PORTC_INPUT_count, 0x80, 0x06);

	//For PA5-PB6
	//POSITION ON PORT = 0x40;
	populateTIMER_SEND(PORTA_CONTROL, 0x20, PORTB_INPUT_VALS, PORTB_INPUT_count, 0x40, 0x05);

	//For PA[4:0]-PB[4:0]
	uint8_t sensor_address = 0x04;
	for(uint8_t COMMON_MASK = 0x10; COMMON_MASK >0; COMMON_MASK>>=1){
		//	POSITION ON PORT = COMMON_MASK;
		populateTIMER_SEND(PORTA_CONTROL, COMMON_MASK, PORTB_INPUT_VALS, PORTB_INPUT_count, COMMON_MASK, sensor_address);
		sensor_address--;
	}
}

void printPORTD(){
	//The last values are set assuming that the inputs were initially high

	//For PD6-PD7
	//POSITION ON PORT = 0x80;
	populateTIMER_SEND(PORTD_CONTROL, 0x40, PORTD_INPUT_VALS, PORTD_INPUT_count, 0x80, 0x0D);

	//For PD[5:1]-PC[2:6]
	uint8_t sensor_address = 0xC;
	for (uint8_t count=2; count<7 ;count++)
		populateTIMER_SEND(PORTD_CONTROL, (0x80>>count), PORTC_INPUT_VALS, PORTC_INPUT_count, (1 << count), sensor_address--);

	//For PD0-PA7
	//POSITION ON PORT = 0x80;
	populateTIMER_SEND(PORTD_CONTROL, 0x01, PORTA_INPUT_VALS, PORTA_INPUT_count, 0x80, 0x07);
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

	softuart_init();

	sei(); // Setting global interrupt

	for(;;){
		if(TOTAL_SENSOR_CONFIGS > 0){
			if(CONTROL_PORTS_SET==1){
				if(CYCLE_COMPLETE == 1){
					softuart_putchar(CURRENT_SENSOR_CONFIG);//Sending out current sensor configuration number.
					printPORTA();
					printPORTD();

					//After All Values have been sent out to the master.
					PORTA_INPUT_count=0;
					PORTB_INPUT_count=0;
					PORTC_INPUT_count=0;
					PORTD_INPUT_count=0;

					CONTROL_PORTS_SET =0;
					CURRENT_SENSOR_CONFIG++;
				}
			}
			else{
				if(CURRENT_SENSOR_CONFIG >= TOTAL_SENSOR_CONFIGS){
					TOTAL_SENSOR_CONFIGS = 0;
					CURRENT_SENSOR_CONFIG = 0;
				}
				else{
					PORTA_CONTROL = (uint8_t)(SENSOR_CONFIG[CURRENT_SENSOR_CONFIG] & 0x00FF);
					PORTD_CONTROL = (uint8_t)(SENSOR_CONFIG[CURRENT_SENSOR_CONFIG] >> 8);
					CONTROL_PORTS_SET=1;

					CYCLE_COMPLETE = 0;

					TCNT0 = 1;
					TCCR0B |= (1 << CS01);
					TIFR0 |= (1 << TOV0); //Forced timer0 interrupt trigger.
					TIMSK0 |= (1 << TOIE0); // Enabling timer 1 overflow

				}
			}
		}
		else{
			//Test Initialization Code (MUST BE REMOVED) --- START---
			SENSOR_CONFIG[0]=0x1234;
			SENSOR_CONFIG[1]=0x7351;
			SENSOR_CONFIG[2]=0x007F;
			TOTAL_SENSOR_CONFIGS = 3;
			//Test Initialization Code (MUST BE REMOVED) ---- STOP ---

			//Reading config from serial port
		/*	uint8_t number_of_config = 0;
			number_of_config = softuart_getchar();
			if(number_of_config >0){
				uint16_t temp16 = 0;
				uint8_t temp8 = 0;
				for (uint8_t count = 0; count < number_of_config; count++){
					temp16 = (softuart_getchar() << 8);
					temp8 = softuart_getchar();
					SENSOR_CONFIG[count] = (temp16 | temp8) ;
				}
				TOTAL_SENSOR_CONFIGS = number_of_config;
				softuart_putchar(0x12);
			} */
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
		//

		//For PORTA
		PCMSK0 |= 0x80;

		//For PORTB
		PCMSK1 |= 0x5F;

		//For PORTC
		PCMSK2 |= 0xFE;

		//For  PORTD
		PCMSK3 |= 0x80;

		PCICR |= 0x0F;
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
	if(PORTC_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTC)){
		PORTC_INPUT_VALS[PORTC_INPUT_count].port_val=PINC;
		PORTC_INPUT_VALS[PORTC_INPUT_count].time_reg_val = TCNT1 - 36735;
		PORTC_INPUT_count++;
	}
}
ISR(PCINT1_vect){
	if(PORTB_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTB)){
		PORTB_INPUT_VALS[PORTB_INPUT_count].port_val=PINB;
		PORTB_INPUT_VALS[PORTB_INPUT_count].time_reg_val = TCNT1 - 36735;
		PORTB_INPUT_count++;
	}
}

ISR(PCINT0_vect){
	if(PORTA_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTA)){
		PORTA_INPUT_VALS[PORTA_INPUT_count].port_val=PINA;
		PORTA_INPUT_VALS[PORTA_INPUT_count].time_reg_val = TCNT1 - 36735;
		PORTA_INPUT_count++;
	}
}

ISR(PCINT3_vect){
	if(PORTD_INPUT_count < (MAX_VALUES * MAX_INPUTS_PORTD)){
		PORTD_INPUT_VALS[PORTD_INPUT_count].port_val=PIND;
		PORTD_INPUT_VALS[PORTD_INPUT_count].time_reg_val = TCNT1 - 36735;
		PORTD_INPUT_count++;
	}
}

