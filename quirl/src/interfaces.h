/*******************************************************************************
***  Fraunhofer IPA
***  Robotersysteme
***  Projekt: Quirl  (Quelle: Tabl-O-bot)
********************************************************************************
********************************************************************************
***  Author: Sebastian Ott
*******************************************************************************/

#ifndef INTERFACES_H
#define INTERFACES_H
/** \file interfaces.h
 *
 * Hardware interfaces of the microcontroller
 *  \version 1.1
 *  
 */

/*
 * |  DDRx   |  PORTx  |
 * |---------|---------|
 * |    0    |    0    | Input 	High-Z
 * |    0    |    1    | Input	Pull-Up	
 * |    1    |    0    | Output	Low
 * |    1    |    1    | Output	High
 */

#include <stdint.h>
#include <avr/io.h>
#include "regtools.h"
#include "config.h"

//Port A
#define DEFAULTS_A		B(11111111)		///< Port A configuration
#define OUTPUTS_A		B(11111111)		///< Port A directions

//Port B
#define FREE_B7			SBIT(PORTB, 7)		///< Free Pin B7
#define MOT2_PWM		SBIT(PORTB, 6)		///< Right Motor PWM
#define MOT1_PWM		SBIT(PORTB, 5)		///< Left Motor PWM
#define CTR_LEFT		SBIT(PINB, 4)		///< Control button left 
#define CTR_FORW		SBIT(PINB, 3)		///< Control button forwards
#define CTR_RIGHT		SBIT(PINB, 2)		///< Control button right
#define CTR_STOP		SBIT(PINB, 1)		///< Control button stop
#define CTR_BACK		SBIT(PINB, 0)		///< Control button backwards
#define OUTPUTS_B		B(11100000)		///< Port B directions
#define DEFAULTS_B		B(11111111)		///< Port B configuration

//Port C		
#define LED3			SBIT(PORTC, 7)		///< LED 3
#define LED4			SBIT(PORTC, 6)		///< LED 4
#define LED2 			SBIT(PORTC, 5)		///< LED 2
#define LED1_RED		SBIT(PORTC, 4)		///< LED 1 Red
#define LED1_GREEN		SBIT(PORTC, 3)		///< LED 1 Green
#define FREE_C2			SBIT(PORTC, 2)		///< Free Pin C2
#define FREE_C1			SBIT(PORTC, 1)		///< Free Pin C1
#define FREE_C0			SBIT(PORTC, 0)		///< Free Pin C0
#define OUTPUTS_C   		B(11111111)		///< Port C directions
#define DEFAULTS_C  		B(00000111)		///< Port C configuration

//Port D
#define FREE_D7			SBIT(PORTD, 7)		///< Free Pin D7
#define FREE_D6			SBIT(PORTD, 6)		///< Free Pin D6
#define FREE_D5			SBIT(PORTD, 5)		///< Free Pin D5
#define FREE_D4			SBIT(PORTD, 4)		///< Free Pin D4
#define UART1_TXD		SBIT(PIND,  3)		///< USART 1 Transmit Pin
#define UART1_RXD		SBIT(PIND,  2)		///< USART 1 Receive Pin
#define US_SDA			SBIT(PORTD, 1)		///< TWI Serial DAta
#define US_SCL			SBIT(PORTD, 0)		///< TWI Serial CLock
#define OUTPUTS_D   		B(11110000)		///< Port D directions
#define DEFAULTS_D  		B(11110000)		///< Port D configuration

//Port E
#define DEFAULTS_E		B(11111111)		///< Port E configuration
#define OUTPUTS_E   		B(11111111)		///< Port E directions

//Port F
#define POWER			SBIT(PORTF, 7)	
#define START_BUTTON		SBIT(PINF, 6)		///< Start Button (Black)
#define VOLT_BATT		SBIT(PORTF, 5)		///< Battery Voltage
#define HALL_B			SBIT(PORTF, 4)		///< Right Hall Effect Sensor
#define HALL_A			SBIT(PORTF, 3)		///< Left Hall Effect Sensor
#define LOAD			SBIT(PORTF, 2)		///< Weight Sensor
#define IR_B			SBIT(PORTF, 1)		///< Right IR Sensor
#define IR_A			SBIT(PORTF, 0)		///< Left IR Sensor
#define OUTPUTS_F   		B(10000000)		///< Port F configuration
#define DEFAULTS_F  		B(11000000)		///< Port F configuration

//Port G
#define OUTPUTS_G      		B(11111111)		///< Port G directions
#define DEFAULTS_G		B(11111111)		///< Port G configuration

//Port H
#define MOT2_BACK		SBIT(PORTH, 7)		///< Right Motor Backwards 
#define MOT2_FORW		SBIT(PORTH, 6)		///< Right Motor Forwards 
#define MOT1_BACK		SBIT(PORTH, 5)		///< Left Motor Backwards 
#define MOT1_FORW		SBIT(PORTH, 4)		///< Left Motor Forwards 
#define FREE_H3			SBIT(PORTH, 3)		///< Free Pin H3
#define FREE_H2			SBIT(PORTH, 2)		///< Free Pin H2
#define UART2_TX		SBIT(PORTH, 1)		///< Free Pin H1
#define UART2_RX		SBIT(PORTH, 0)		///< Free Pin H0
#define OUTPUTS_H      		B(11111111)		///< Port H directions
#define DEFAULTS_H     		B(11111111)		///< Port H configuration

//Port I
#define OUTPUTS_I      		B(11111111)		///< Port I directions
#define DEFAULTS_I		B(11111111)		///< Port I configuration

//Port J
#define FREE_J7			SBIT(PORTJ, 7)		///< Free Pin J7
#define FREE_J6			SBIT(PORTJ, 6)		///< Free Pin J6
#define BATT_LED5		SBIT(PORTJ, 5)		///< Batt state LED Full
#define BATT_LED4		SBIT(PORTJ, 4)		///< Batt state LED 
#define BATT_LED3		SBIT(PORTJ, 3)		///< Batt state LED
#define BATT_LED2		SBIT(PORTJ, 2)		///< Batt state LED 
#define BATT_LED1		SBIT(PORTJ, 1)		///< Batt state LED LOW
#define STOP_BUTTON		SBIT(PINJ, 0)		///< Stop Button (Red)
#define OUTPUTS_J		B(11111110)		///< Port J directions
#define DEFAULTS_J		B(11111111)		///< Port J configuration

//Port K
#define LEFT_PORT		PINK
#define RIGHT_PORT		PINK
#define HALL_C			SBIT(PORTK, 7)		///< Back Hall Effect Sensor
#define IR_C			SBIT(PORTK, 6)		///< Front IR Sensor
#define FREE_K5			SBIT(PORTK, 5)		///< Free Pin K5
#define FREE_K4			SBIT(PORTK, 4)		///< Free Pin K4
#define ODO_B2			SBIT(PINK, 3)		///< Odometry Right Register 1
#define ODO_A2			SBIT(PINK, 2)		///< Odometry Right Register 0
#define ODO_B1			SBIT(PINK, 1)		///< Odometry Left Register 1
#define ODO_A1 			SBIT(PINK, 0)		///< Odometry Left Register 0
#define OUTPUTS_K		B(00110000)		///< Port K directions
#define DEFAULTS_K		B(00110000)		///< Port K configuration

#define LEFT_MASK_A		0x01				///< Left Mask A (Odometry)
#define LEFT_MASK_B		0x02				///< Left Mask B (Odometry)
#define RIGHT_MASK_A		0x04				///< Right Mask A (Odometry)
#define RIGHT_MASK_B		0x08				///< Right Mask B (Odometry)
#define LEFT_MASK		(LEFT_MASK_A | LEFT_MASK_B)	///< Left Mask (Odometry)
#define RIGHT_MASK		(RIGHT_MASK_A | RIGHT_MASK_B)	///< Right Mask (Odometry)

//Port L
#define OUTPUTS_L		B(11111111)			///< Port L directions
#define DEFAULTS_L		B(11111111)			///< Port L configuration



// ADC Channels
#define NCHAN			8	///< Number of ADC Channels used

#define ADC_IR1			0	///< Left IR Sensor
#define ADC_IR2			1	///< Right IR Sensor
#define ADC_LOAD		2	///< Weight Sensor
#define ADC_HALL1		3	///< Left Hall Effect Sensor
#define ADC_HALL2		4	///< Right Hall Effect Sensor
#define ADC_VBAT		5	///< Battery Voltage
#define ADC_IR3			6	///< Front IR Sensor
#define ADC_HALL3		7	///< Back Hall Effect Sensor

#define ON	1				///< ON General use Macro
#define OFF	0				///< OFF General use Macro

#endif


