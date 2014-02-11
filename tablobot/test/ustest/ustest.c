/************************************************************************************************/
/*	This is a test program for the ultrasound sensors					*/
/*	Alejandro Merello									*/
/************************************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "usart.h"
#include "stdio_init.h"
#include "utils.h"
#include "interfaces.h"
#include "digio.h"
#include "i2c.h"
#include "srf10.h"


/* Performes a ranging operation of all three units
 *for one time and prints the results  on USART0 */
void range(uint8_t sensors)
{
	char s[6];
	if(sensors & 0x01)
	{
		srf10_range_req(SRF10_UNIT_0);
		sprintf(s,"%u ",srf10_range_read(SRF10_UNIT_0));
		usart0_puts(s);
		usart0_sendbyte(' ');
	}
	if(sensors & 0x02)
	{
		srf10_range_req(SRF10_UNIT_1);		
		sprintf(s,"%u",srf10_range_read(SRF10_UNIT_1));
		usart0_puts(s);
		usart0_sendbyte(' ');
	}
	if(sensors & 0x04)
	{
		srf10_range_req(SRF10_UNIT_2);
		sprintf(s,"%u",srf10_range_read(SRF10_UNIT_2));
		usart0_puts(s);
	}
	usart0_sendbyte(0x0D);
}

void range_acc(void)
{
	int ma_0,mb_0,ma_1,mb_1,ma_2,mb_2;
	char s[6];

	delay(50);
	srf10_range_req(SRF10_UNIT_0);
	delay(45);
	srf10_range_req(SRF10_UNIT_2);
	delay(40);
	srf10_range_req(SRF10_UNIT_1);
	delay(35);
	
	ma_0=srf10_range_read(SRF10_UNIT_0);
	srf10_range_req(SRF10_UNIT_0);
	delay(35);
	ma_2=srf10_range_read(SRF10_UNIT_2);
	srf10_range_req(SRF10_UNIT_2);
	delay(40);
	
	ma_1=srf10_range_read(SRF10_UNIT_1);
	srf10_range_req(SRF10_UNIT_1);
	delay(45);
	mb_0=srf10_range_read(SRF10_UNIT_0);
	mb_2=srf10_range_read(SRF10_UNIT_2);
	mb_1=srf10_range_read(SRF10_UNIT_1);
	if((ma_0-mb_0<10)&&(ma_0-mb_0>-10)){sprintf(s,"%u",(ma_0+mb_0)/2); usart0_puts(s);}else usart0_sendbyte('-');
	usart0_sendbyte(',');
	if((ma_1-mb_1<10)&&(ma_1-mb_1>-10)){sprintf(s,"%u",(ma_1+mb_1)/2); usart0_puts(s);}else usart0_sendbyte('-');
	usart0_sendbyte(',');
	if((ma_2-mb_2<10)&&(ma_2-mb_2>-10)){sprintf(s,"%u",(ma_2+mb_2)/2); usart0_puts(s);}else usart0_sendbyte('-');
	usart0_sendbyte(0x0D);
}

/* Performs a gain test on one sensor trying all possible
 * gain configurations and prints the results on USART0 */
void gain_test(uint8_t sensor)
{
	uint8_t i,gain;
	char s[10];

	for(gain=0x10;gain>0x00;gain--)
	{
		sprintf(s,"Gain: 0x%x",gain); usart0_puts(s);usart0_sendbyte(0x0D);	
		srf10_set_gain(sensor,gain);
		for(i=0;i<10;i++)
		{
			srf10_range_req(sensor);
			sprintf(s,"%u",srf10_range_read(sensor)); 
			usart0_puts(s);
			usart0_sendbyte(0x0D);
			delay(100);
		}
		usart0_puts("*********************");usart0_sendbyte(0x0D);
	}
	srf10_set_gain(sensor,0x10);	//Restore gain to default (700)

}

/* Performs a range test on one sensor trying as many intervals as are
 * passed on the granularity variable 'gran' and prints the results on USART0 */
void range_test(uint8_t sensor,uint8_t gran) 
{
	uint8_t i,range;
	char s[20];

	for(range=0xFF;range>(0xFF%gran);range-=(0xFF/gran))
	{
		sprintf(s,"Range: %d [mm]",range*43+43); usart0_puts(s);usart0_sendbyte(0x0D);	
		srf10_set_range(sensor,range);
		for(i=0;i<10;i++)
		{
			srf10_range_req(sensor);
			sprintf(s,"%u",srf10_range_read(sensor)); 
			usart0_puts(s);
			usart0_sendbyte(0x0D);
			delay(100);
		}
		usart0_puts("*********************");usart0_sendbyte(0x0D);
	}
	srf10_set_range(sensor,0xFF); //Restore range to default (11008 [mm])
}

void init(void)
{
	digio_init();
	usart_init();
	stdio_init();
	i2c_init ();
	sei();
}

int main(void)
{
	init();
	
	srf10_set_gain(SRF10_UNIT_0,0x08);
	srf10_set_gain(SRF10_UNIT_1,0x08);
	srf10_set_gain(SRF10_UNIT_2,0x08);
	
	while(1)
	{	
		range(0x03);delay(100);		
		//range_test(SRF10_UNIT_2,30);
		//gain_test(SRF10_UNIT_2);
	}
	return 0;
	
}
