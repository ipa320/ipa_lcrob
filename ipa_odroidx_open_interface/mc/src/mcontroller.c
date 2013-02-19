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
	*str = '\n';
	str++;
	*str = 0;
}


void motor_init() {
	softuart_broadcast(ENABLE);
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

int32_t motor_getPos(uint8_t motor) {
	char c;
	int32_t r=0;

	softuart_puts(motor, POSITION);
	while( (c=softuart_getchar(motor))!='\r' ) {
		if(c=='-') r*=-1;
		else if(c>='0'&&c<='9') {
			r*=10;
			r+=c-'0';
		}
	}

	return r;
}
