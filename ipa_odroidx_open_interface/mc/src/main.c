#include <avr/io.h>		// include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support

#include "protocol.h"		// include our global settings


int main(void)
{
	init();

	while(1) {
		parse();	
	}
	
	return 0;
}

