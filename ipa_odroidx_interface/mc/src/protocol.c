#include "protocol.h"

//----- Include Files ---------------------------------------------------------
#include <avr/io.h>		// include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support

#include "global.h"		// include our global settings

#include "a2d.h"		// include A/D converter function library
#include "timer.h"		// include timer function library (timing, PWM, etc)
#include "soft_spi.h"		

#define CHANNEL2PIN(x) (x==0?5:7)

//set speed, corred speed according to direction
void set_motor_speed(u08 channel, u08 speed) {
	if( inb(PIND)&(1<<CHANNEL2PIN(channel)) )
		speed = ~speed;

	if(channel&1)
		timer1PWMASet(speed);
	else
		timer1PWMBSet(speed);
}

#ifdef WATCH_MOTOR

//check soft-limits before moving
u08 set_motor_direction(u08 channel, u08 dir) {
	u16 ad = a2dConvert10bit(0);

	if(dir) {
		sbi(PORTD,CHANNEL2PIN(channel));
		if(ad<WMOTOR_MIN) {
			set_motor_speed(channel, 0);
			return 0;
		}
	}
	else {
		cbi(PORTD,CHANNEL2PIN(channel));
		if(ad>WMOTOR_MAX) {
			set_motor_speed(channel, 0);
			return 0;
		}
	}

	return 1;
}

void check_motor(void) {
	timer0ClearOverflowCount();
	set_motor_direction(WATCH_MOTOR, inb(PIND)&(1<<CHANNEL2PIN(WATCH_MOTOR)) ); //motor 0
}

#else

//no check if movement is possible
u08 set_motor_direction(u08 channel, u08 dir) {

	if(dir)
		sbi(PORTD,CHANNEL2PIN(channel));
	else
		cbi(PORTD,CHANNEL2PIN(channel));

	return 1;
}
#endif

void init(void) {
	// turn on and initialize A/D converter
	a2dInit();

	// initialize the timer system
	timerInit();

	outb(DDRB, 0x06);
	outb(DDRC, 0xF0);
	outb(DDRD, 0xA2);

	timer1PWMInit(8); //8 bit resolution

	timer1PWMAOn();
	timer1PWMASet(0);

	timer1PWMBOn();
	timer1PWMBSet(0);

	a2dSetPrescaler(ADC_PRESCALE_DIV64);
	a2dSetReference(ADC_REFERENCE_AVCC);

	softSpiInit();

#ifdef WATCH_MOTOR
	//check if motor went too far  for security
	//timer0SetPrescaler();
	timerAttach(TIMER0OVERFLOW_INT, check_motor);
#endif

}

void set_output(u08 out) {
	outb(PORTC, ((inb(PINC)&0x0F)|(out<<4)) );
	outb(PORTD, ((inb(PIND)&0xFC)|(out>>4)) );
}

/*
	motor: 0b???? ??DC
	D = direction
	C = channel
*/
void set_motor(u08 motor, u08 speed) {
	u08 channel = motor&1;

	if(set_motor_direction(channel, motor&2))
		set_motor_speed(channel, speed);
}

u16 get_analog(u08 ch) {
	if(ch>=4) return 0xffff;
	return a2dConvert10bit(ch);
}

u08 get_input(void) {
	return inb(PINC)&0x0f;
}

void parse(void) {
	u08 c = softSpiGetByte();
softSpiSendByte(c+1);
return;
	switch(c&0xF0) {
		case SET_OUTPUT:
			set_output(softSpiGetByte());
			break;

		case SET_MOTOR:
			set_motor(c&0x03, softSpiGetByte());
			break;

		case GET_ANALOG:
			softSpiSendWord(get_analog(c&0x03));
			break;

		case GET_INPUT:
			softSpiSendByte(get_input());
			break;

		case SETUP:
			softSpiSendByte('O');
			PORTC = softSpiGetByte()&0x0f;
			break;
	}
}
