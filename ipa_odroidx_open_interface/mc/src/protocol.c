
//----- Include Files ---------------------------------------------------------
#include <avr/io.h>		// include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support
#define __DELAY_BACKWARD_COMPATIBLE__  // For delay 
#include <util/delay.h>
#include "protocol.h"
#include "uart.h"

#define 	DEFAULT_UART_BAUD_RATE 57600
#define 	MAX_STREAM_PACKETS 10 //Sets the maximum number of packet IDs to be expected with the command STREAM (Should confirmed and set according to the capacity of the controller).
uint32_t	MASTER_UART_BAUD_RATE = DEFAULT_UART_BAUD_RATE;

uint8_t		STREAM_PACKET_ID[MAX_STREAM_PACKETS];
uint8_t 	NUMBER_OF_PACKETS;	
uint8_t		STREAM_ENABLED;
/*
implement

(https://kforge.ros.org/turtlebot/turtlebot/file/d21c5431efda/turtlebot_driver/src/turtlebot_driver.py)
    57 ROOMBA_OPCODES = dict(
    58     start = 128,				impl
    59     baud = 129,				impl
    60     control = 130,			impl
    61     safe = 131,				impl
    62     full = 132,				impl
    63     power = 133,				
    64     spot = 134,				impl
    65     clean = 135,				impl
    66     max = 136,				?
    67     drive = 137,				TODO
    68     motors = 138,			impl
    69     leds = 139,				impl
    70     song = 140,				impl
    71     play = 141,				impl
    72     sensors = 142,			TODO
    73     force_seeking_dock = 143,		impl
    74     )
    75 
    76 CREATE_OPCODES = dict(
    77     soft_reset = 7,  # Where is this documented?
    78     low_side_drivers = 138,
    79     song = 140,
    80     play_song = 141,
    81     pwm_low_side_drivers = 144,
    82     direct_drive = 145,
    83     digital_outputs = 147,
    84     stream = 148,
    85     query_list = 149,
    86     pause_resume_stream = 150,
    87     send_ir = 151,
    88     script = 152,
    89     play_script = 153,
    90     show_script = 154,
    91     wait_time = 155,
    92     wait_distance = 156,
    93     wait_angle = 157,
    94     wait_event = 158,
    95     )
*/

#define OP_START	128
#define OP_BAUD		129
#define OP_CONTROL	130
#define OP_SAFE		131
#define OP_FULL		132
#define OP_SPOT		134
#define OP_COVER	135
#define OP_DEMO		136
#define OP_DRIVE	137
#define OP_LS_DRIVERS	138
#define OP_LEDS		139
#define OP_SONG		140
#define OP_PLAY_SONG	141
#define OP_SENSORS	142
#define OP_COVERDOCK	143
#define OP_DRIVE_DIRECT	145
#define OP_OUTPUT	147
#define OP_STREAM	148
#define OP_PAUSE_RESUME	150
#define OP_SEND_IR	151
#define OP_SCRIPT	152
#define OP_PLAY_SCRIPT	153
#define OP_SHOW_SCRIPT	154
#define OP_WAIT		155

#define PID_BW_DROPS	7
#define PID_WALL	8
#define PID_CLIFF_L	9
#define PID_CLIFF_FL	10
#define PID_CLIFF_FR	11
#define PID_CLIFF_R	12
#define PID_VWALL	13
#define PID_LS_DRIVER	14
#define PID_UNUSED1	15
#define PID_UNUSED2	16
#define PID_IR		17
#define PID_BUTTONS	18

void init(void){
	NUMBER_OF_PACKETS = 0;
	STREAM_ENABLED = 1;
	uart_init(UART_BAUD_SELECT(MASTER_UART_BAUD_RATE, F_CPU ));
	//All initiialization is to be done here.
	//Soft UARTs for motors 
	//ADC
	//Timer to generate 15ms for Stream command?
	//Watchdog timer(Before anything else)
	sei();
}
uint8_t uart_get_valid_char(){
	uint16_t buffer = 0;
	do{
		buffer = uart_getc();
	}while((buffer & UART_NO_DATA) || (buffer & UART_OVERRUN_ERROR) || (buffer & UART_BUFFER_OVERFLOW));
	return (uint8_t)(buffer & 0xff);
}
void sendSensorPacket(uint8_t packet_id) {
	switch(packet_id) {
		case PID_LS_DRIVER:

		case PID_CLIFF_L:
		case PID_CLIFF_R:
		case PID_CLIFF_FR:
		case PID_CLIFF_FL:
		case PID_WALL:
		case PID_VWALL:
		case PID_BUTTONS:
		case PID_UNUSED1:
		case PID_UNUSED2:
		case PID_BW_DROPS:
			uart_putc(0);
			break;

		case PID_IR:
			uart_putc(0xff);
			break;
	}
}

void parse(void)
{
	int16_t vel, radius;
	int16_t vel1, vel2;
	uint8_t command = 0;
	uint8_t packet_id = 0;

	command = uart_get_valid_char();
	switch(command ) {	//opcode
		case OP_START:
			break;
		case OP_BAUD:
			switch(uart_get_valid_char()) {//baud
				case 0: MASTER_UART_BAUD_RATE=300;break;
				case 1: MASTER_UART_BAUD_RATE=600;break;
				case 2: MASTER_UART_BAUD_RATE=1200;break;
				case 3: MASTER_UART_BAUD_RATE=2400;break;
				case 4: MASTER_UART_BAUD_RATE=4800;break;
				case 5: MASTER_UART_BAUD_RATE=9600;break;
				case 6: MASTER_UART_BAUD_RATE=14400;break;
				case 7: MASTER_UART_BAUD_RATE=19200;break;
				case 8: MASTER_UART_BAUD_RATE=28800;break;
				case 9: MASTER_UART_BAUD_RATE=38400;break;
				case 10: MASTER_UART_BAUD_RATE=57600;break;
				default: MASTER_UART_BAUD_RATE=115200;break;
			}
			cli();// Should be tested
			uart_init(UART_BAUD_SELECT(MASTER_UART_BAUD_RATE, F_CPU ));
			sei();
			break;
		case OP_CONTROL: 
		case OP_SAFE:
			break;
		case OP_PLAY_SCRIPT:
			break;
		case OP_SHOW_SCRIPT:
//			uartPutByte(0); //no script
			uart_get_valid_char();
			break;
		case OP_FULL:
			break;
		case OP_DEMO:	
//			uartGetByte();//dummy
			uart_get_valid_char();
			break;
		case OP_SEND_IR:	
//			uartGetByte();//dummy
			uart_get_valid_char();
			break;
		case OP_PLAY_SONG:	
//			uartGetByte();//dummy
			uart_get_valid_char();			
			break;
		case OP_LEDS:	
//			uartGetByte();uartGetByte();uartGetByte();//dummy
			uart_get_valid_char();uart_get_valid_char();uart_get_valid_char();
			break;
		case OP_OUTPUT:	
//			uartGetByte();//dummy
			uart_get_valid_char();			
			break;
		case OP_COVER:
			break;
		case OP_COVERDOCK:
			break;
		case OP_SPOT:
			break;
		case OP_LS_DRIVERS:	
//			uartGetByte();//dummy
			uart_get_valid_char();			
			break;
		case OP_DRIVE:
			//TODO: Maybe? since all calculations are to be done on the master. 
//			vel    = (((u16)uartGetByte())<<8) | uartGetByte();
//			radius = (((u16)uartGetByte())<<8) | uartGetByte();
			vel    = (((uint16_t)uart_get_valid_char())<<8) | uart_get_valid_char();
			radius = (((uint16_t)uart_get_valid_char())<<8) | uart_get_valid_char();
			break;
		case OP_DRIVE_DIRECT:
			//TODO:
//			vel1 = (((u16)uartGetByte())<<8) | uartGetByte();
//			vel2 = (((u16)uartGetByte())<<8) | uartGetByte();
			vel1 = (((uint16_t)uart_get_valid_char())<<8) | uart_get_valid_char();
			vel2 = (((uint16_t)uart_get_valid_char())<<8) | uart_get_valid_char();
			break;
		case OP_SONG:
//			uartGetByte();//dummy
			uart_get_valid_char();			
//			vel = uartGetByte();
			vel = uart_get_valid_char();			
			for(radius=0; radius!=vel; radius++) {
//				uartGetByte();uartGetByte();
				uart_get_valid_char();uart_get_valid_char();
			}
			break;
		case OP_SENSORS:
//			packet_id = uartGetByte();
			packet_id = uart_get_valid_char();			
			switch(packet_id) {
				case 0:
					for(packet_id=7; packet_id<=26; packet_id++) sendSensorPacket(packet_id);
					break;
				case 1:
					for(packet_id=7; packet_id<=16; packet_id++) sendSensorPacket(packet_id);
					break;
				case 2:
					for(packet_id=17; packet_id<=20; packet_id++) sendSensorPacket(packet_id);
					break;
				case 3:
					for(packet_id=21; packet_id<=26; packet_id++) sendSensorPacket(packet_id);
					break;
				case 4:
					for(packet_id=27; packet_id<=34; packet_id++) sendSensorPacket(packet_id);
					break;
				case 5:
					for(packet_id=35; packet_id<=42; packet_id++) sendSensorPacket(packet_id);
					break;
				case 6:
					for(packet_id=7; packet_id<=42; packet_id++) sendSensorPacket(packet_id);
					break;
				default: sendSensorPacket(packet_id); 
					break;
			}
			break;
		case OP_STREAM:
/*			for(number_of_packets=0; number_of_packets<?; number_of_packets++) enable[number_of_packets] = false;
			number_of_packets = uartGetByte();
			while(number_of_packets--)
				enable[uartGetByte()] = true;
			//TODO:
*/
			NUMBER_OF_PACKETS = uart_get_valid_char();
			for (int i=0; i<NUMBER_OF_PACKETS; i++){
				STREAM_PACKET_ID[i] = uart_get_valid_char();
			}
			//Stream is enabled by setting NUMBER_OF_PACKETS and STREAM_ENABLED
			//Depending on the number of packets to be sent a timer should be set to
			//appoximate around 15ms. Here in the parser, timer 1 interrupt should
			//be forced to enable calling of STREAM responder function. 
			break;
		case OP_PAUSE_RESUME:
			STREAM_ENABLED = uart_get_valid_char();
			//TODO:
			break;
		case OP_SCRIPT:
//			vel = uartGetByte();
			vel = uart_get_valid_char();			
			for(radius=0; radius!=vel; radius++) {
//				uartGetByte();
				uart_get_valid_char();
			}
			break;
		case OP_WAIT:
			_delay_ms( uart_get_valid_char()*15 );
			break;
	}

/*	if(recveivedM1) {
	}

	if(recveivedM2) {
	}
	*/
}
