#include <avr/io.h>		// include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support

#include "protocol.h"		// include our global settings

volatile uint8_t TIMER_OVERFLOW;

int main(void)
{
	//Add watchdog related code here.
	init();

	while(1) {
		parse();	
		if (TIMER_OVERFLOW)
		{
			if (is_stream_enabled())
				generateStreamResponse();
			updatePosition(); // Comment when motor controller no connected
			TIMER_OVERFLOW=0;
		}
	}
	
	return 0;
}

