#include "protocol.h"

//----- Include Files ---------------------------------------------------------
#include <avr/io.h>		// include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support

#include "global.h"		// include our global settings

#include "a2d.h"		// include A/D converter function library
#include "timerx8.h"		// include timer function library (timing, PWM, etc)
#include "soft_spi.h"		

#undef WATCH_MOTOR

#define CHANNEL2PIN(x) (x==0?7:5)

#define SOFT_PWM_CHANNELS 6
u08 pwm_vals[SOFT_PWM_CHANNELS] = {};


static void set_output(u08 out) {
	outb(PORTC, ((inb(PINC)&0xCF)|((out&3)<<4)) );
	outb(PORTD, ((inb(PIND)&0xFC)|(out>>4)) );
}

static void soft_pwm(void) {
	static u08 cnt = 0, vo=0;
	u08 i,v=0xFF;

	for(i=0; i<SOFT_PWM_CHANNELS; i++) {
		if(pwm_vals[i]<=cnt) {
			v&=~(1<<i);
		}
	}

	if(v!=vo) {
		set_output(v);
		vo=v;
	}

	cnt+=3;
	//timer2ClearOverflowCount();
}

//set speed, corred speed according to direction
void set_motor_speed(u08 channel, u08 speed) {
	if( (inb(PIND)&(1<<CHANNEL2PIN(channel))) )
		speed = ~speed;

	if(channel&1)
		timer1PWMASet(speed);
	else
		timer1PWMBSet(speed);
}

#ifdef WATCH_MOTOR
#error
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

void on_parse(void) {
	u08 c = softSpiGetByte();

	switch(c&0xF0) {
		case SET_OUTPUT:
			//set_output(softSpiGetByte());
			if((c&0x0F)>=6) break;
			while(!softSpiHasByte()) soft_pwm();
			pwm_vals[c&0x0F] = softSpiGetByte();
			break;

		case SET_MOTOR:
			while(!softSpiHasByte()) soft_pwm();
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

void parse(void) {
	if(softSpiHasByte())
		on_parse();
	soft_pwm();
}

void init(void) {
	// turn on and initialize A/D converter
	a2dInit();

	// initialize the timer system
	//timerInit();
	sei();

	outb(DDRB, 0x06);
	outb(DDRC, 0x30);
	outb(DDRD, 0xA2);

	timer1PWMInit(8); //8 bit resolution

	timer1PWMAOn();
	timer1PWMASet(0);

	timer1PWMBOn();
	timer1PWMBSet(0);

	a2dSetPrescaler(ADC_PRESCALE_DIV8);
	a2dSetReference(ADC_REFERENCE_AVCC);

	softSpiInit();

	//timer2SetPrescaler(TIMERRTC_CLK_DIV32);
	//timerAttach(TIMER2OVERFLOW_INT, soft_pwm);

#ifdef WATCH_MOTOR
#error
	//check if motor went too far  for security
	//timer0SetPrescaler();
	timerAttach(TIMER0OVERFLOW_INT, check_motor);
#endif
}
