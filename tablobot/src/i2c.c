/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Tabl-O-bot
****************************************************************************
****************************************************************************
***  Autor: Alejandro Merello
***************************************************************************/

/** \file i2c.c
 *  \brief I2C Module Driver.
 *  
 *  Driver for the I2C Module of the tabl-O-bot.
 *  \version 1.0
 *  \author Alejandro Merello
 */

#include "i2c.h"
#include <avr/io.h>


void i2c_init ()
{
	TWBR = I2C_TWBR;	//Sets SCL frequency	
	TWDR = 0xFF;		// Default content = SDA released.
	TWCR = (1<<TWEN) | (0<<TWIE) | (0<<TWINT) | (0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWWC);  // Enable TWI-interface, release TWI pins, disable Interrupt. No signal request        	
}

char i2c_transmit (unsigned char address, unsigned char subaddress, unsigned char data)
{	
        char error;
	
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);     // Send START condition
	while (!(TWCR&(1<<TWINT)));                     // Wait for Interrupt flag
	error = ((TWSR&0xF8)!=START);                   // Check if START condition has been transmitted
	
	if (!error)
	{	TWDR = address;				// Load slave address
	        TWCR = (1<<TWINT) | (1<<TWEN);          // Clear Interrupt flag to start transmission
		while (!(TWCR&(1<<TWINT)));             // Wait for Interrupt flag
		error = ((TWSR&0xF8)!=SLA_W_ACK);       // Check if SLA+W has been transmitted and ACK has been received
	}
	
	if (!error)
	  {	TWDR = subaddress;                       // Load sub-address
		TWCR = (1<<TWINT) | (1<<TWEN);          // Clear Interrupt flag to start transmission
		while (!(TWCR&(1<<TWINT)));             // Wait for Interrupt flag
		error = ((TWSR&0xF8)!=DATA_W_ACK);      // Check if Data byte has been transmitted and ACK has been received
	}
	
		if (!error)
	  {     TWDR = data;                            // Load data
		TWCR = (1<<TWINT) | (1<<TWEN);          // Clear Interrupt flag to start transmission
		while (!(TWCR&(1<<TWINT)));             // Wait for Interrupt flag
		error = ((TWSR&0xF8)!=DATA_W_ACK);      // Check if Data byte has been transmitted and ACK has been received
	}

	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);     // Send STOP condition
		
	return error;
}

char i2c_receive (unsigned char address, unsigned char subaddress, unsigned char* data)
{	char error;
	
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);     // Send START condition
	while (!(TWCR&(1<<TWINT)));                     // Wait for Interrupt flag
	error = ((TWSR&0xF8)!=START);                   // Check if START condition has been transmitted
	
	if (!error)
	{	TWDR = address;			        // Load slave adress
		TWCR = (1<<TWINT) | (1<<TWEN);          // Clear Interrupt flag to start transmission
		while (!(TWCR&(1<<TWINT)));             // Wait for Interrupt flag
		error = ((TWSR&0xF8)!=SLA_W_ACK);       // Check if SLA+W has been transmitted and ACK has been received
	}
	
	if (!error)
	  {	TWDR = subaddress;                      // Load sub-address
		TWCR = (1<<TWINT) | (1<<TWEN);          // Clear Interrupt flag to start transmission
		while (!(TWCR&(1<<TWINT)));             // Wait for Interrupt flag
		error = ((TWSR&0xF8)!=DATA_W_ACK);      // Check if Data byte has been transmitted and ACK has been received
	}

	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);     // Send START condition
	while (!(TWCR&(1<<TWINT)));                     // Wait for Interrupt flag
	error = ((TWSR&0xF8)!=REP_START);               // Check if repeated START condition has been transmitted

	if (!error)
	{	TWDR = address | 0x01;			// Load slave address + Read bit
		TWCR = (1<<TWINT) | (1<<TWEN);          // Clear Interrupt flag to start transmission
		while (!(TWCR&(1<<TWINT)));             // Wait for Interrupt flag
		error = ((TWSR&0xF8)!=SLA_R_ACK);       // Check if Data byte has been received and ACK has been returned
	}
	
	if (!error)
	{	TWCR = (1<<TWINT) | (1<<TWEN);          // Clear Interrupt flag to start transmission
		while (!(TWCR&(1<<TWINT)));             // Wait for Interrupt flag
	  *data = TWDR;                                 // Store received Data
	}

	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);     // Send STOP condition

	return error;
}
