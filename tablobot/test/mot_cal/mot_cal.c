/************************************************************************************************/
/*	This is a test program to calibrate the PID Controller of the motors			*/
/*	through step response									*/
/************************************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "usart.h"
#include "stdio_init.h"
#include "utils.h"
#include "interfaces.h"
#include "digio.h"
#include "mot.h"
#include "odo.h"
#include "scheduler.h"


int8_t soll_v; double v_phi;
int8_t start,trigg,k_trigg;
char s[10];
int i=1,count,wcount;

void sched_layer0(void)
{
	enc_cycle_right();
	enc_cycle_left();
}

void sched_layer1(void)
{

}

void sched_layer2(void)
{	
	static uint16_t left_old,right_old;
	int16_t delta_left,delta_right;

	delta_left = enc_left - left_old;
	left_old = enc_left;
	delta_right = enc_right - right_old;
	right_old = enc_right;			
	
	if(trigg&&(count<500)){sprintf(s,"%d",delta_left); usart0_puts(s);usart0_sendbyte(' ');count++;}

	//vrad_ctrl(soll_v, v_phi);
	
	if(trigg)
	{
		if(wcount<150)
		{
			//soll_v = 100;v_phi = 0;
			if(start) {set_right(237);set_left(200);} else {set_right(0);set_left(0);} //set_right(252);set_left(220);
			wcount++;
		}
		else
		{ 
			if(wcount<500)
			{
				//soll_v = 0;v_phi = 0;
				if(start) {set_right(260);set_left(220);} else {set_right(0);set_left(0);}
				wcount++;
			}
			else 
			{
				trigg=0;wcount=0;count=0;k_trigg=0;set_right(0);set_left(0);
			}
		}
	}	
	
	if(!ROT_SCHLTR)
	{
	        //soll_v = 100;
		//v_phi = 0;
		//trigg=1;
		start=1;
	}
	if(!SCHWRZ_SCHLTR)
	{	
	        //soll_v = 0;
		//v_phi = 0;
		//trigg=0;count=0;usart0_sendbyte(0x0D);
		start=0;
	}
	
	
	odo_cycle();
}

void sched_layer3(void)
{	
	//if((Kp<=1.5)&&(start)&&(!k_trigg)){Kp+=0.1;trigg=1;k_trigg=1;usart0_sendbyte(0x0D);sprintf(s,"%f: ",Kp); usart0_puts(s);}
	//if((Kp<=1.8)&&(start)&&(!k_trigg)){Kp+=.1;trigg=1;k_trigg=1;sprintf(s,"];");usart0_puts(s);usart0_sendbyte(0x0D);sprintf(s,"y%d = [ ",i++); usart0_puts(s);}
	if((i<=7)&&(start)&&(!k_trigg)){trigg=1;k_trigg=1;sprintf(s,"];");usart0_puts(s);usart0_sendbyte(0x0D);sprintf(s,"y%d = [ ",i++); usart0_puts(s);}
	if(!start){soll_v = 0;v_phi = 0;}

}

void init(void)
{
	sei();
	digio_init();
	usart_init();
	stdio_init();
	init_mot();	
	sched_init();
	Kp=1.4; Ki=20; Kd=0.003;
}

int main(void)
{
	init();
	
	soll_v = 0;
	v_phi = 0;	

	
	
	while(1)
	{
	
	}

	return(0);
}
