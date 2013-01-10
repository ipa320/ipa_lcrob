/*
 * UARTDriver.h
 *
 *  Created on: Jan 9, 2013
 *  	Author: josh-as
 */

#ifndef __UARTDRIVER_H__
#define __UARTDRIVER_H__
#include <unistd.h>
#include <termios.h>
#include "ipa_odroidx_ultrasonic_interface/CommPortDriver.h"
class UARTDriver:CommPortDriver
{
	public:
		UARTDriver(char * device_filename, int baud_rate = 2400);
		ssize_t readBytes(void * buf, size_t count);
		ssize_t writeBytes(void * buf, size_t count);
		~UARTDriver();
	private:
		int fd_;
		unsigned char buffer_[64];
		struct termios old_config_, new_config_;
};

#endif
