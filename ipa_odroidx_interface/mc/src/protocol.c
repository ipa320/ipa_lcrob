#include "global.h"		// include our global settings
#include "protocol.h"

//----- Include Files ---------------------------------------------------------
#include <avr/io.h>		// include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support

#include "a2d.h"		// include A/D converter function library
#include "timerx8.h"		// include timer function library (timing, PWM, etc)
#include "soft_spi.h"		

#undef WATCH_MOTOR

#define CHANNEL2PIN(x) (x==0?7:5)

#define SOFT_PWM_CHANNELS 6
volatile u08 pwm_vals[SOFT_PWM_CHANNELS] = {0,0,0,0,0,0};
volatile u08 pwm_vals2[SOFT_PWM_CHANNELS] = {0,0,0,0,0,0};
volatile u08 pwm_mode[SOFT_PWM_CHANNELS] = {0,0,0,0,0,0};	//pulse: 0 -> no pulsing
volatile u16 motor_aim;	//0 just move
volatile u08  motor_enabled;


static void set_output(u08 out) {
	outb(PORTC, ((inb(PINC)&0xCF)|((out&3)<<4)) );
	outb(PORTD, ((inb(PIND)&0xFC)|(out>>4)) );
}

static void soft_pwm(void) {
	static u08 cnt = 0, vo=0, cnt2=0;
#ifdef CREATE_AUTO_ON
	static u08 power_state = 0;
#endif
	u08 i,v=0, t;
	u16 ad;

	//calc. output
	for(i=0; i<SOFT_PWM_CHANNELS; i++)
		if(pwm_vals[i]>cnt)
			v|=(1<<i);

	//pulsing mode
	if(cnt==0) {
		cnt2+=8;
		if(cnt2==0) {
			for(i=0; i<SOFT_PWM_CHANNELS; i++) {
				if(pwm_mode[i]&1) {
					t = (pwm_mode[i]&(~1))>>1;
					if(t!=0 && pwm_vals[i]<=t) {
						pwm_mode[i]&=~1;
						pwm_vals[i]=0;
					}
					else pwm_vals[i]-=t;
				} else {
					t = (pwm_mode[i]&(~1))>>1;
					if(t!=0 && pwm_vals[i]>=pwm_vals2[i]-t) {
						pwm_mode[i]|=1;
						pwm_vals[i]=pwm_vals2[i];
					}
					else pwm_vals[i]+=t;
				}
			}
		}
	}

	if(v!=vo) {
		set_output(v);
		vo=v;
	}

	cnt+=7;
	//timer2ClearOverflowCount();

	if(motor_enabled && motor_aim!=0) {
		if(!a2dIsComplete()) {
			ad = (inb(ADCL) | (inb(ADCH)<<8));
			if(ad>motor_aim) {
				if(ad>motor_aim+MOTOR_TOLERANCE) set_motor(MOTOR_OUT_CHANNEL, MOTOR_SPEED);
				else  set_motor(MOTOR_OUT_CHANNEL, 0);
			} else {
				if(ad+MOTOR_TOLERANCE<motor_aim) set_motor(MOTOR_OUT_CHANNEL|2, MOTOR_SPEED);
				else  set_motor(MOTOR_OUT_CHANNEL, 0);
			}

			a2dSetChannel(MOTOR_AD_CHANNEL);
			a2dStartConvert();
		}
	}

#ifdef CREATE_AUTO_ON
	if(!(inb(PINC)&CREATE_CHECK_ON) && !(inb(PINC)&CREATE_EXT_POWER) && power_state) {
		PORTC |= CREATE_POWER_CONTROL;
	}
	else {
		PORTC &= ~CREATE_POWER_CONTROL;
	}
	power_state = inb(PINC)&CREATE_EXT_POWER;
#endif
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

u08 get_input(void) {
	return inb(PINC)&0x0f;
}

u16 get_analog(u08 ch) {
	u16 t;
	if(ch>=4) return 0xffff;
	motor_enabled=0;
	a2dSetChannel(ch);
	a2dStartConvert();
	while(a2dIsComplete()) soft_pwm();
	t = (inb(ADCL) | (inb(ADCH)<<8));
	motor_enabled = 1;
	return t;
}

void on_parse(void) {
	u08 c = softSpiGetByte();
	u16 t;

	switch(c&0xF0) {
		case SET_OUTPUT:
			//set_output(softSpiGetByte());
			if((c&0x0F)>=6) break;
			while(!softSpiHasByte()) soft_pwm();
			pwm_vals2[c&0x0F] = pwm_vals[c&0x0F] = softSpiGetByte();
			pwm_mode[c&0x0F] = 0;
			break;

		case SET_MOTOR:
			while(!softSpiHasByte()) soft_pwm();
			set_motor(c&0x03, softSpiGetByte());
			motor_aim = 0;
			break;

		case GET_ANALOG:
			while(!softSpiCanSend()) soft_pwm();
			softSpiSendWord(get_analog(c&0x03));
			break;

		case GET_INPUT:
			while(!softSpiCanSend()) soft_pwm();
			softSpiSendByte(get_input());
			break;

		case SETUP:
			while(!softSpiCanSend()) soft_pwm();
			softSpiSendByte('O');
			PORTC = softSpiGetByte()&0x0f;
			break;

		case SET_PULSE:
			if((c&0x0F)>=6) break;
			while(!softSpiHasByte()) soft_pwm();
			pwm_mode[c&0x0F] = softSpiGetByte();
			break;

		case SET_MOTORAIM:
			while(!softSpiHasByte()) soft_pwm();
			t = softSpiGetByte()<<8;
			while(!softSpiHasByte()) soft_pwm();
			t |= softSpiGetByte();
			motor_aim = t;
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

	a2dSetPrescaler(ADC_PRESCALE_DIV128);
	a2dSetReference(ADC_REFERENCE_AVCC);

	// initialize the timer system
	timer1Init();

	outb(DDRB, 0x06);
	outb(DDRC, 0x30);
	outb(DDRD, 0xA3);

	timer1PWMInit(8); //8 bit resolution

	timer1PWMAOn();
	timer1PWMASet(0);

	timer1PWMBOn();
	timer1PWMBSet(0);

	softSpiInit();

	//timer2SetPrescaler(TIMERRTC_CLK_DIV32);
	//timerAttach(TIMER2OVERFLOW_INT, soft_pwm);

#ifdef WATCH_MOTOR
#error
	//check if motor went too far  for security
	//timer0SetPrescaler();
	timerAttach(TIMER0OVERFLOW_INT, check_motor);
#endif

	motor_aim=0;
	motor_enabled=1;

	sei();

	set_motor(0,0);
	set_motor(1,0);
}
