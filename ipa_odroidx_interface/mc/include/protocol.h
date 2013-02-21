
#ifndef PROTOCOL_H
#define PROTOCOL_H

#define SET_OUTPUT	0x10
#define SET_MOTOR	0x20
#define GET_ANALOG	0x30
#define GET_INPUT	0x40
#define SETUP		0x50
//extended functions
#define SET_PULSE	0x60
#define SET_MOTORAIM	0x70

#define WATCH_MOTOR	0
#define WMOTOR_MIN	40
#define WMOTOR_MAX	800

#define MOTOR_AD_CHANNEL  0
#define MOTOR_OUT_CHANNEL 0
#define MOTOR_TOLERANCE   6
#define MOTOR_SPEED	  200

void init(void);
void parse(void);

u16 get_analog(u08 ch);
void set_motor(u08 motor, u08 speed);

#endif
