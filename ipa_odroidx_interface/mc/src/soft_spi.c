/*! \file spi.c \brief SPI interface driver. */
//*****************************************************************************
//
// File Name	: 'spi.c'
// Title		: SPI interface driver
// Author		: Pascal Stang - Copyright (C) 2000-2002
// Created		: 11/22/2000
// Revised		: 06/06/2002
// Version		: 0.6
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
// NOTE: This code is currently below version 1.0, and therefore is considered
// to be lacking in some functionality or documentation, or may not be fully
// tested.  Nonetheless, you can expect most functions to work.
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#include <avr/io.h>
#include <avr/interrupt.h>

#include "soft_spi.h"
#include "extint.h"

volatile u16 data_out = 0;
volatile u08 data_out_len = 0;
volatile u16 data_in = 0;
volatile u08 data_in_len = 0;
//typedef void (*func)(void);
//volatile func _handler = 0;

ISR(PCINT0_vect)
{
	if( !(inb(PINB)&(1<<5)) )
		return;
	//set output
	if(data_out&1)
		PORTB |= (1<<4);
	else
		PORTB &= ~(1<<4);
	data_out>>=1;
	if(data_out_len>0) --data_out_len;

	data_in<<=1;
	data_in |= (inb(PINB)&(1<<3)?1:0);
	++data_in_len;

	//if(data_in_len>=8 && _handler)
	//	_handler();
}

void softSpiClear(void) {
	data_in_len = 0;
	data_out_len = 0;
}

// access routines
void softSpiInit(void)
{
	PCICR |= (1<<PCIE0);
	//EICRA |= (1<<ISC01)|(1<<ISC00);
	PCMSK0 |= (1<<PCINT5);

	DDRB |= (1<<4);
}

void softSpiSendByte(u08 data)
{
	while(data_out_len>0);
	data_out_len = 8;
	data_out = data;
}

void softSpiSendWord(u16 data)
{
	while(data_out_len>0);
	data_out_len = 16;
	data_out = data;
}

u08 softSpiCanSend(void) {
	return (data_out_len==0);
}

u08 softSpiHasByte(void) {
	return data_in_len>=8;
}

u08 softSpiGetByte(void) {
	while(data_in_len<8);
	u16 tmp = data_in;
	data_in>>=8;
	data_in_len-=8;
	return tmp;
}

u16 softSpiGetWord(void) {
	while(data_in_len<16);
	u16 tmp = data_in;
	data_in=0;
	data_in_len-=16;
	return tmp;
}
