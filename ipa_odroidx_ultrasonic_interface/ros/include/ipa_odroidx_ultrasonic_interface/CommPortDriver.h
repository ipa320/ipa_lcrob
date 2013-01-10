/*
 * CommPortDriver.h
 *
 *  Created on: Jan 9, 2013
 *  	Author: josh-as
 */

#ifndef __COMMPORTDRIVER_H__
#define __COMMPORTDRIVER_H__

class CommPortDriver{
	public:
		virtual ssize_t readBytes(void * buf, size_t count) = 0;
		virtual ssize_t writeBytes(void * buf, size_t count) = 0;
		virtual ~CommPortDriver(){}
};

#endif
