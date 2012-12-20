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

struct TIME_KEEPER{ //struct for storing input port value and TIMER1 value in ISR associated with PCINTs
	uint8_t port_val;
	uint16_t time_reg_val;
};

/*
 *PORTA and PORTD are for output whereas PORTB and PORTC are for input
 *The board is programmed to use the external crystal of 18.432MHz and
 *the CLKDIV8 fuse is also programmed, causing effective frequency
 *equal to 2304000.
 *In PORTA_CONTROL and PORTD_CONTROL 1 denotes ping and 0 denotes listen,
 *whereas MSB is unused. Max value for a port should be 0x7F
 *NOTE:TIMER2 IN USE BY SOFTUART.
 */

volatile  uint8_t PING_STAGE=0; //For denoting different stages of a ping or listening pulse, from 0-3

volatile unsigned char PORTA_CONTROL=0x00;
volatile unsigned char PORTD_CONTROL=0x00;

volatile struct TIME_KEEPER PORTA_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTA];
volatile uint8_t PORTA_INPUT_count=0;// Counter for Input value for PORTA

volatile struct TIME_KEEPER PORTB_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTB];
volatile uint8_t PORTB_INPUT_count=0;// Counter for Input value for PORTB

volatile struct TIME_KEEPER PORTC_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTC];
volatile uint8_t PORTC_INPUT_count=0;// Counter for Input value for PORTC

volatile struct TIME_KEEPER PORTD_INPUT_VALS[MAX_VALUES * MAX_INPUTS_PORTD];
volatile uint8_t PORTD_INPUT_count=0;// Counter for Input value for PORTD

volatile uint16_t TIMER[MAX_VALUES]; //For recording TIMER 1 due edge change.
volatile uint8_t TIMER_count=0;

volatile uint8_t CYCLE_COMPLETE = 0;

volatile uint16_t SENSOR_CONFIG[MAX_SENSOR_CONFIGS]; //For storing sensor configurations sent by the master
volatile uint8_t TOTAL_SENSOR_CONFIGS=0;
volatile uint8_t CURRENT_SENSOR_CONFIG=0;
volatile uint8_t CONTROL_PORTS_SET=0;

void print_TIMER(){ //Send the set timer values to the master, higher byte first.
	for (uint8_t count=0; count <TIMER_count; count++){
		softuart_putchar((uint8_t)((TIMER[count] & 0xFF00)>> 8));
		softuart_putchar((uint8_t)(TIMER[count] & 0x00FF));
	}
}
void populateTIMER_SEND(uint8_t PORT_CONTROL, uint8_t CHANNEL_POSITION, volatile struct TIME_KEEPER * time_keeper, uint8_t INPUT_count, uint8_t INPUT_POSITION, uint8_t SENSOR_ADDRESS){
	/* This function is for populating the variable TIMER and sending it to the master (Check doc/pin_mapping.ods for correct PORT_CONTROL, CHANNEL_POSITION, INPUT_POSITION and SENSOR_ADDRESS).
	 * PORT_CONTROL: Is either set to PORTA_CONTROL or PORTD_CONTROL, based on the input pin to be checked. 
	 * time_keeper: Pointer to the TIME_KEEPER structure the channel input pin is located in.(PORT[A-D]_TIMER_VALS)
	 * INPUT_count: The PORT[A-D]_INPUT_count associated with time_keeper.
	 * INPUT_POSITION: Position of the channel's input pin on the associated input port.
	 * SENSOR_ADDRESS: Sensor address assigned to the sensor.
	 */

	uint8_t PING_CHECK=0; //For storing, if timer value for a rising edge has been written in the variable TIMER
	uint8_t last_value_temp=INPUT_POSITION; //Is used to check for falling edges.
	TIMER_count=0;
	for(uint8_t count=0; count < INPUT_count; count++){ //For all input values.
		if((~(PORT_CONTROL) & CHANNEL_POSITION) == CHANNEL_POSITION){//If listening
			if(((INPUT_POSITION & time_keeper[count].port_val)==0x00) && (last_value_temp==INPUT_POSITION)){ //Checks for falling edge.
				if(TIMER_count < MAX_VALUES)
					TIMER[TIMER_count++] = time_keeper[count].time_reg_val;
			}
			last_value_temp = (INPUT_POSITION & time_keeper[count].port_val); //Assign value to last_value_temp before looping.
		}
		else { // Pinging
			if(((INPUT_POSITION & time_keeper[count].port_val) == INPUT_POSITION) && (PING_CHECK==0)){ //checks for rising edge.
				TIMER[TIMER_count++] = time_keeper[count].time_reg_val; 
				PING_CHECK = 1;
			}
			else if (((INPUT_POSITION & time_keeper[count].port_val)==0x00) && (last_value_temp==INPUT_POSITION) && (PING_CHECK==1)){ //After the first rising edge has been recoded, check for consecutive falling edges.
				if(TIMER_count < MAX_VALUES)
					TIMER[TIMER_count++] = time_keeper[count].time_reg_val;
			}
			last_value_temp = (INPUT_POSITION & time_keeper[count].port_val);
		}
	}
	//Sending the data out
	softuart_putchar((SENSOR_ADDRESS<<4) | TIMER_count); //Address + TIMER value
	if (TIMER_count>0){ //If any timer value recorded.
		print_TIMER();
	}
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
	}
	else if(PING_STAGE == 2){
		PORTA = PORTA_CONTROL&0x7F;
		PORTD = PORTD_CONTROL&0x7F;
		TCNT0 = 177; // Setting for 270us -0.02% error
		PING_STAGE = 3;
	}
	else if(PING_STAGE == 3){
		PORTA = 0x00;
		PORTD = 0x00;
		//Disabling timer0 to setup timer1 to enable data reading from sensors.
		TIMSK0 &= ~(1 << TOIE0);

//		softuart_disable(); // Not sure if works or not.

		//Setting up timer1 for 100ms
		TCNT1 = 36735;
		TIMSK1 |= (1 << TOIE1);

		PCICR |= 0x0F;
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

