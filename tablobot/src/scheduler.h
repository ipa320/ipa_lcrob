/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Raser 
****************************************************************************
****************************************************************************
***  Autor: Winfried Baum
***************************************************************************/
/** \file scheduler.h
 *
 * Scheduler's setup and implementation
 *
 *  \author Winfried Baum
 */
#ifndef SCHEDULER_H
#define SCHEDULER_H

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
#include "i2c.h"
#include "srf10.h"
#include "mot.h"
#include "adc.h"
#include "odo.h"

//States 
#define STATE_OFF							0	///< Off State, responds to direct manual movements.
#define STATE_ON							1	///< On State, executes automatic control.
#define STATE_BT							2	///< Manual state, responds to bluetooth manual commands.

#define STATE_OBSTACLE_DETECTED				1	///< Obstacle detected.
#define STATE_SEEN_BY_UNIT2					1	///< Obstacle is seen by the lateral sensor.
#define STATE_FURTHER_POINT_OF_LINE			1	///< Tablobot is located on a further point of the line.
#define STATE_ALIGNED						1	///< Tablobot is aligned with the virtual line.

//Battery state
#define BATT_100	0x2F	///< Battery full.
#define BATT_75		0x2E	///< Battery 75% load.
#define BATT_50		0x2D	///< Battery 50% load.
#define BATT_25		0x2C	///< Battery 25% load.
#define BATT_0		0x2B	///< Battery empty.

//Bluetooth messages
#define BATT_REQ	0x20	///< Battery status request.
#define TB_AUTO		0x03	///< Tablobot's change to Automatic Mode.
#define BT_MANUAL	0x02	///< Manual Mode.
#define BT_AUTO		0x01	///< Automatic Mode.
#define BT_UP		0xFF	///< Bluetooth command Arrow UP
#define BT_DOWN		0xFE	///< Bluetooth command Arrow DOWN
#define BT_LEFT		0xFD	///< Bluetooth command Arrow LEFT
#define BT_RIGHT	0xFC	///< Bluetooth command Arrow RIGHT
#define BT_FIRE		0xFB	///< Bluetooth command FIRE
#define BT_START	0xFA	///< Bluetooth command Start (Automatic mode)
#define BT_STOP		0xF9	///< Bluetooth command Stop (Automatic mode)

#define HALL_CALIBRATION 0	///< For manual calibration of the Hall Effect Sensors set to 1. Gives serial output of the ADC readings.

int8_t soll_v;				///< Used to set the speed of the virtual wheel.
double v_phi;				///< Used to set the angle of the virtual wheel.
double x0;					///< X coordinate of the line to follow
double y0;					///< Y coordinate of the line to follow
double phi0;				///< Angle coordinate of the line to follow
double phi_obstacle;		///< Perpendicular angle of the obstacle, serves to limit rotation
double x1;					///< X coordinate of the robot when it detects an obstacle.
double y1;					///< Y coordinate of the robot when it detects an obstacle.
double delta[2];			///< Distance from the line in milimeters.
double Px;					///< Proportional controller for lateral distance
double e_x;					///< Difference between reference and measurement in P distance controller for object avoidance.
double soll_x;				///< Reference distance in P distance controller for object avoidance.

int16_t r0[2];				///< Stores last 2 values read from the front left US Ranger.
int16_t r1[2];				///< Stores last 2 values read from the front right US Ranger.
int16_t r2[2];				///< Stores last 2 values read from the lateral US Ranger.
uint8_t poweroff_count;		///< Timer for red button Power off.
uint8_t emptyBatTimer;		///< Timer for Empty Battery Auto Power Off.
uint8_t pushStartTimer;		///< Timer for push start.
uint8_t batt_state;			///< Battery load state.
uint8_t btControl;			///< Receives the manual movement commands from the Bluetooth device.
int8_t power;				///< State variable for main states (OFF, AUTO, MANUAL)
int8_t obstacle;			///< State variable for obstacle detection (NO OBSTACLE, OBSTACLE DETECTED)
int8_t line;				///< State variable for line following (INLINE, TILTED RIGHT, TILTED LEFT)
int8_t us_unit2;			///< State variable for lateral US Ranger (NOT SEEN BY UNIT_2, SEEN BY UNIT_2)
int8_t line2;				///< State variable for line identification (FURTHER POINT OF LINE, NOT A FURTHER POINT OF LINE)
int8_t angle;				///< State variable for angle identification (WIDE, NARROW)
int8_t perpendicular;		///< State variable for perpendicular angle identification to avoid backtracking.
int mass;					///< Load in gramms.

double iphi0;				///< Stores 1 cycle old robot's orientation.
double delta0;				///< Stores 1 cycle old robot's distance to the line.
int8_t off[3];				///< High if the corresponding US Sensor hasn't been pinged in a long time, meaning an outdated buffer value .

char s[50];					///< String variable for output.

/**Executes the movement from the Hall Effect sensors input
 */
void hullMovement(void);

/**Detects movement of the hull
 * \return 1 if the Hall Effect Sensors are triggered, otherwise 0.
 */
int8_t hullStop(void);

/**Stops the robot if it detects a pit through input from the IR sensors.
 */
void fallStop(void);

/**
 *  Ultrasonic ranging in two modes, mode 0 measures sequentially the 3 US Sensors, mode 1 measures
 *  sequentially the 2 Frontal US Sensors. This function must be called cyclically.
 *  \param mode Operating mode. 1: Two frontal sensors. 2: The three sensors.
 */
void range(int8_t mode);

/**Updates the charge state of the battery
 */
void battState(void);

/**Checks if the robot is oriented to a given angle.
 * \param ref Reference angle in radians.
 * \return 1 If the robot is oriented in <code>ref</code> direction, otherwise 0.
 */
int8_t isAngle(double ref);

/** Resets all state variables
 */
void stateReset(void);

/**Initializes the interrupt registers for the scheduler
 */
void sched_init(void);

/**Scheduler layer 0
 */
void sched_layer0(void);

/**Scheduler layer 1
 */
void sched_layer1(void);

/**Scheduler layer 2
 */
void sched_layer2(void);

/**Scheduler layer 3
 */
void sched_layer3(void);

/**Bluetooth Communication Control
 */
void bluetooth(void);

#endif
