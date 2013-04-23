
#ifndef PROTOCOL_H
#define PROTOCOL_H

void init(void);
void parse(void);
void init_motors(void);
void parseSendSensorPacket(uint8_t);
void generateStreamResponse(void);
uint8_t is_stream_enabled(void);
void updatePosition(void);

extern volatile uint8_t		TIMER_OVERFLOW;
#endif
