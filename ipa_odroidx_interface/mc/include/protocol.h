
#ifndef PROTOCOL_H
#define PROTOCOL_H

#define SET_OUTPUT	0x10
#define SET_MOTOR	0x20
#define GET_ANALOG	0x30
#define GET_INPUT	0x40
#define SETUP		0x50

#define WATCH_MOTOR	0
#define WMOTOR_MIN	40
#define WMOTOR_MAX	800

void init(void);
void parse(void);

#endif
