/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Tabl-O-bot
***************************************************************************/

/** \file i2c.h
 *  \brief I2C Module Driver.
 *  
 *  Driver for the I2C Module of the tabl-O-bot.
 *  \version 1.0
 *  \author Alejandro Merello
 */

#ifndef __I2C_H__
#define __I2C_H__

#define I2C_TWBR     0x40  ///< SCL = CPU_CLK / (16 + 2 * TWBR * 4^TWPS)  => SCL ~ 100[kHz]

///Initializes the I2C Module
void i2c_init ();

/**Transmits a byte to a given register located in an addressed I2C module 
 * \param address Destination address of for the data to be sent.
 * \param subaddress Register in the receiver to which the data is addressed.
 * \param data Byte to be sent.
 * \return An error code or 0
 */
char i2c_transmit (unsigned char address, unsigned char subaddress, unsigned char data);

/**Stores a byte read from a register of an addressed I2C module.
 * \param address Address of the I2C module to listen to.
 * \param subaddress Register of the module which value is being requested.
 * \param data Byte read.
 * \return An error code or 0
 */
char i2c_receive (unsigned char address, unsigned char subaddress, unsigned char* data);


//TWI Master Codes
#define START        0x08  ///< START has been transmitted
#define REP_START    0x10  ///< Repeated START has been transmitted
#define SLA_W_ACK    0x18  ///< SLA+W has been transmitted and ACK received
#define SLA_W_NACK   0x20  ///< SLA+W has been transmitted and NACK received 
#define DATA_W_ACK   0x28  ///< Data byte has been transmitted and ACK received
#define DATA_W_NACK  0x30  ///< Data byte has been transmitted and NACK received
#define SLA_R_ACK    0x40  ///< SLA+R has been transmitted and ACK received
#define SLA_R_NACK   0x48  ///< SLA+R has been transmitted and NACK received
#define DATA_R_ACK   0x50  ///< Data byte has been received and ACK transmitted
#define DATA_R_NACK  0x58  ///< Data byte has been received and NACK transmitted
#define ARB_LOST     0x38  ///< Arbitration lost

#endif //__I2C_H_
