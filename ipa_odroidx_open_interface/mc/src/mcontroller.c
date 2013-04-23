#include <stdlib.h>
#include "mcontroller.h"
#include "softuart.h"

void int2str(char *str, int16_t val){
	int16_t div=10000;
	if (val <0){
		*str = '-';
		str++;
		val=abs(val);
	}
	while(!(val/div) && div > 1) div/=10;
	while(div){
		*str = '0'+(val/div);
		str++;
		val %= div;
		div/=10;
	}
	*str = '\r';
	str++;
	*str = '\n';
	str++;
	*str = 0;
}

void motor_init() {
	softuart_broadcast(ENABLE);
	softuart_broadcast("HO\r\n"); // set current position to 0
}

void motor_stop() {
	softuart_broadcast(DISABLE);
}

void motor_setVel(int16_t rpm1, int16_t rpm2) {
	char buffer[8];
	buffer[0]='V';

	int2str(buffer+1, rpm1);
	softuart_puts(PORT_1, buffer);

	int2str(buffer+1, rpm2);
	softuart_puts(PORT_2, buffer);
}

void motor_reqPos(uint8_t motor){
	softuart_flush_input_buffer(motor);
	softuart_puts(motor, POSITION);
}
int32_t motor_getPos(uint8_t motor, uint8_t * valid_val) {
	if(softuart_kbhit(motor)){	
		char c;
		int32_t r=0;
		uint8_t is_negative= 0;
		while( (c=softuart_getchar(motor))!='\r' ) { // MUST IMPLEMENT WATCHDOG TIMER HERE
			if(c=='-') is_negative=1; // Assuming "-" is the first character.
			else if(c>='0'&&c<='9') {
				r*=10;
				r+=c-'0';
			}
		}
		if(is_negative)r*=-1;
		softuart_getchar(motor); // for '\n', MUST IMPLEMENT WATCHDOG TIMER HERE
		*valid_val = 1;
		return r;
	}
	else{
		*valid_val = 0;
		return 0;
	}
}
