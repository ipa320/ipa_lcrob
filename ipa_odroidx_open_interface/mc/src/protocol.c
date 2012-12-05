#include "protocol.h"

//----- Include Files ---------------------------------------------------------
#include <avr/io.h>		// include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support

#include "uart.h"
#include "uartsw.h"


void init(void)
{
	uartInit();
....
}

#define OP_START	128
#define OP_BAUD		129
#define OP_SAFE		131
#define OP_FULL		132
#define OP_DEMO		136
#define OP_COVER	135
#define OP_COVERDOCK	143
#define OP_SPOT		134
#define OP_DRIVE	137
#define OP_DRIVE_DIRECT	145
#define OP_LEDS		139
#define OP_OUTPUT	147
#define OP_SEND_IR	151
#define OP_SONG		140
#define OP_PLAY_SONG	141

void parse(void)
{
	u32 baud;
	s16 vel, radius;
	s16 vel1, vel2;

	if(receivedU) {
		switch( uartGetByte() ) {	//opcode
			case OP_START:
				break;
			case OP_BAUD:
				switch(uartGetByte()) {//baud
					case 0: baud=300;break;
					case 1: baud=600;break;
					case 2: baud=1200;break;
					case 3: baud=2400;break;
					case 4: baud=4800;break;
					case 5: baud=9600;break;
					case 6: baud=14400;break;
					case 7: baud=19200;break;
					case 8: baud=28800;break;
					case 9: baud=38400;break;
					case 10: baud=57600;break;
					default: baud=115200;break;
				}
				uartSetBaudRate(baud);
				break;
			case OP_SAFE:
				break;
			case OP_FULL:
				break;
			case OP_DEMO:	uartGetByte();//dummy
				break;
			case OP_SEND_IR:	uartGetByte();//dummy
				break;
			case OP_PLAY_SONG:	uartGetByte();//dummy
				break;
			case OP_LEDS:	uartGetByte();uartGetByte();uartGetByte();//dummy
				break;
			case OP_OUTPUT:	uartGetByte();//dummy
				break;
			case OP_COVER:
				break;
			case OP_COVERDOCK:
				break;
			case OP_SPOT:
				break;
			case OP_DRIVE:
				vel    = (((u16)uartGetByte())<<8) | uartGetByte();
				radius = (((u16)uartGetByte())<<8) | uartGetByte();
				break;
			case OP_DRIVE_DIRECT:
				vel1 = (((u16)uartGetByte())<<8) | uartGetByte();
				vel2 = (((u16)uartGetByte())<<8) | uartGetByte();
				break;
			case OP_SONG:
				uartGetByte();//dummy
				vel = uartGetByte();
				for(radius=0; radius!=vel; radius++) {
					uartGetByte();uartGetByte();
				}
				break;
		}
	}

	if(recveivedM1) {
	}

	if(recveivedM2) {
	}
}
