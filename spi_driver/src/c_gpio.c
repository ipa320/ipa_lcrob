#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "c_gpio.h"
//#include <linux/gpio.h>

/*
 * End of GPIO user space helpers
 */

#define N_GPIO 256

/*
* an array which holds open FDs to /sys/class/gpio/gpioXX/value for all needed pins
*/
static int gpio_fds[N_GPIO] ;
static int gpio_direction[N_GPIO];

/*
 * GPIO user space helpers
 *
 * Copyright 2009 Analog Devices Inc.
 * Michael Hennerich (hennerich@blackfin.uclinux.org)
 *
 * Licensed under the GPL-2 or later
 */
 
#define GPIO_DIR_IN	0
#define GPIO_DIR_OUT	1
 
static int gpio_export(unsigned gpio)
{
	int fd, len;
	char buf[11];
 
	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

	usleep(100*1000);
 
	return 0;
}
 
static int gpio_unexport(unsigned gpio)
{
	int fd, len;
	char buf[11];
 
	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}
 
static int gpio_dir(unsigned gpio, unsigned dir)
{
	int fd;
	char buf[60];
 
	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/direction");
		return fd;
	}
 
	if (dir == GPIO_DIR_OUT)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);

	gpio_direction[gpio] = dir;
 
	close(fd);
	return 0;
}
 
static int gpio_dir_out(unsigned gpio)
{
	return gpio_dir(gpio, GPIO_DIR_OUT);
}
 
static int gpio_dir_in(unsigned gpio)
{
	return gpio_dir(gpio, GPIO_DIR_IN);
}


int gpio_setpin(int pin, int value)
{
  int r;

  if(gpio_direction[pin]!=GPIO_DIR_OUT)
	return -2;

#ifdef PIN_INVERSE
  if (pin & PIN_INVERSE)
  {
    value  = !value;
    pin   &= PIN_MASK;
  }
#endif

    if ( gpio_fds[pin] < 0 )
      return -1;

  if (value)
    r=write(gpio_fds[pin], "1", 1);
  else
    r=write(gpio_fds[pin], "0", 1);

  if (r!=1) return -1;

  return 0;
}

int gpio_getpin(int pin)
{
  unsigned char invert=0;
  char c;

#ifdef PIN_INVERSE
  if (pin & PIN_INVERSE)
  {
    invert = 1;
    pin   &= PIN_MASK;
  }
#endif

  if ( gpio_fds[pin] < 0 )
    return -1;

  if (lseek(gpio_fds[pin], 0, SEEK_SET)<0)
    return -1;  
    
  if (read(gpio_fds[pin], &c, 1)!=1)
    return -1;    
    
  if (c=='0')
    return 0+invert;
  else if (c=='1')
    return 1-invert;
  else 
    return -1;
  
}

#if 0
static int gpio_highpulsepin(PROGRAMMER * pgm, int pin)
{

  if ( gpio_fds[pin & PIN_MASK] < 0 )
    return -1;

  gpio_setpin(pgm, pin, 1);
  gpio_setpin(pgm, pin, 0);

  return 0;
}
#endif

int gpio_init() {
  int i;

  for (i=0; i<N_GPIO; i++) {
	gpio_fds[i]=-1; 
	gpio_direction[i]=GPIO_DIR_IN;
  }

  return 0;
}

int gpio_open(int pinno, bool out)
{
	int r;
	char filepath[60];

        if ((r=gpio_export(pinno)) < 0) 
	    return r;
	    
	if (!out)
	    r=gpio_dir_in(pinno);
	else
	    r=gpio_dir_out(pinno);
	    
	if (r < 0)
	    return r;
    
	if ((r=snprintf(filepath, sizeof(filepath), "/sys/class/gpio/gpio%d/value", pinno))<0)
	    return r;	       
    
        if ((gpio_fds[pinno]=open(filepath, O_RDWR))<0)
	    return gpio_fds[pinno];
  
 return(0);
}

void gpio_close()
{
  int i;
  for (i=0; i<N_GPIO; i++) 
    if (gpio_fds[i]>=0) {
       close(gpio_fds[i]);
       gpio_unexport(i);
    }
  return;
}
