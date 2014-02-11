/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Tabl-O-bot
****************************************************************************
****************************************************************************
***  Autor: Alejandro Merello
***************************************************************************/

/** \file srf10.c
 *  \brief SRF10 Ultrasonic Ranger driver
 *  
 *  Driver for the SRF10 Ultrasonic Ranger.
 *  \version 1.0
 *  \author Alejandro Merello
 */


#include <stdint.h>
#include "i2c.h"
#include "srf10.h"
#include "utils.h"

void srf10_range_req(unsigned char sensor)
{
	srf10_busy(sensor);	
	i2c_transmit(sensor,US_REG,SRF10_CM);
}

void srf10_busy(unsigned char sensor)
{
	unsigned char data;	
	do
	{
		data=0;
		i2c_receive(sensor,SW_VERSION,&data);
		delay(1);
	}
	while(!data);
}

uint16_t srf10_range_read(unsigned char sensor)
{
	uint16_t range;
	unsigned char data[2];
	srf10_busy(sensor);
	i2c_receive(sensor,VAL_MSB, data+1);
	srf10_busy(sensor);
	i2c_receive(sensor,VAL_LSB, data);
	range = ((uint8_t)data[1] << 8) + (uint8_t)data[0];
	return range;
}

void srf10_set_range(unsigned char sensor,unsigned char range)
{
	srf10_busy(sensor);	
	i2c_transmit(sensor,RANGE_REG,range);
}


void srf10_set_gain(unsigned char sensor, unsigned char gain)
{
	srf10_busy(sensor);	
	i2c_transmit(sensor,GAIN_REG,gain);
}

void srf10_change_address(unsigned char sensor,unsigned char new_address)
{
	srf10_busy(sensor);	
	i2c_transmit(sensor,US_REG,0xA0);
	srf10_busy(sensor);
	i2c_transmit(sensor,US_REG,0xAA);
	srf10_busy(sensor);
	i2c_transmit(sensor,US_REG,0xA5);
	srf10_busy(sensor);
	i2c_transmit(sensor,US_REG,new_address);
}
