#ifndef __HELPER_H__
#define __HELPER_H__
#include "definitions.h"
void print_TIMER(); //Send the set timer values to the master, higher byte first.

/* This function is for populating the variable TIMER and sending it to the master
 * (Check doc/pin_mapping.ods for correct PORT_CONTROL, CHANNEL_POSITION, INPUT_POSITION and SENSOR_ADDRESS).
 * PORT_CONTROL: Is either set to PORTA_CONTROL or PORTD_CONTROL, based on the input pin to be checked. 
 * time_keeper: Pointer to the TIME_KEEPER structure the channel input pin is located in.(PORT[A-D]_TIMER_VALS)
 * INPUT_count: The PORT[A-D]_INPUT_count associated with time_keeper.
 * INPUT_POSITION: Position of the channel's input pin on the associated input port.
 * SENSOR_ADDRESS: Sensor address assigned to the sensor.
 */
void populateTIMER_SEND(uint8_t PORT_CONTROL, uint8_t CHANNEL_POSITION, volatile struct TIME_KEEPER * time_keeper, uint8_t INPUT_count, uint8_t INPUT_POSITION, uint8_t SENSOR_ADDRESS);

void printPORTA(); //Send timer values calculated on PORTA to the master.
void printPORTD(); ///Send timer values calculated on PORTD to the master.

#endif
