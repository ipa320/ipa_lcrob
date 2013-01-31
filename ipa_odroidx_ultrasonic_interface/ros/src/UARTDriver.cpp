/*
 * UARTDriver.cpp
 *
 *  Created on: Jan 9, 2013
 *  	Author: josh-as
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "ipa_odroidx_ultrasonic_interface/UARTDriver.h"
UARTDriver::UARTDriver(const char * device_filename, int baud_rate)
{
	ROS_INFO("Opening device file: %s", device_filename);
	this->fd_ = open(device_filename, O_RDWR | O_NOCTTY | O_SYNC);
	if (this->fd_ == -1)
	{
		ROS_ERROR("%s", strerror(errno));
		exit(EXIT_FAILURE);
	}
	ROS_INFO("Checking if device file is a tty.");
	if(!isatty(this->fd_))
	{
		ROS_ERROR("%s", strerror(errno));
		exit(EXIT_FAILURE);
	}
	ROS_INFO("Saving device configuration.");
	if (tcgetattr(this->fd_, &(this->old_config_)) < 0)
	{
		ROS_ERROR("%s", strerror(errno));
		exit(EXIT_FAILURE);
	}
	//Making a config for raw binary data transfer.
	cfmakeraw(&(this->new_config_));
	cfsetspeed(&(this->new_config_), baud_rate);
	//Setting up the new configuration.
	tcsetattr(this->fd_, TCSANOW, &(this->new_config_));
	tcsetattr(this->fd_, TCSAFLUSH, &(this->new_config_));
	ROS_INFO("Sleeping for four seconds to ensure settings are applied.");
	sleep(4);
	tcflush(this->fd_, TCIFLUSH);
}

ssize_t UARTDriver::readBytes(void * buf, size_t count)
{
	return read(this->fd_, buf, count); //No extra work is required.
}
ssize_t UARTDriver::writeBytes(void * buf, size_t count)
{
	int return_val =  write(this->fd_, buf, count);
	if(return_val > 0)
	{
		tcdrain(this->fd_);
		usleep(250000);
	}
	return return_val;
}

UARTDriver::~UARTDriver()
{
	//Setting up original serial port configuration again.
	ROS_INFO("Loading original serial port settings.");
	tcsetattr(this->fd_, TCSANOW, &(this->old_config_));
	close(this->fd_);
}
