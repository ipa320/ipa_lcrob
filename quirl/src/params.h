/*******************************************************************************
***  Fraunhofer IPA
***  Robotersysteme
***  Projekt: Quirl  (Quelle: Tabl-O-bot)
********************************************************************************
********************************************************************************
***  Author: Alejandro Merello, Maik Siee
*******************************************************************************/
#ifndef PARAMS_H
#define PARAMS_H
/** \file params.h
 *
 * Constant parameters.
 *
 * Here are the fixed parameters of the state machine.
 *  
 *  version 1.1
 */

#define STOP_BUTTON_OFF_TIME			120			///< Time in centiseconds required to turn off the robot by pressing the stop button.
#define US_SENSOR_TRIGGER			13			///< Threshold reading of the US Sensor for obstacle detection in [cm]
#define US_SENSOR_DISTANT			16			///< Threshold reading of the US Sensor for cautious approach.

#define IR_EDGE_TRIGGER				350			///< Infrared ADC Measure that triggers the edge detection for fall avoidance.

#define AVOIDANCE_P				0.13			///< Proportional constant for the obstacle avoidance controller.
#define AVOIDANCE_REF_DIST			12			///< Reference distance for the obstacle avoidance controller.

#define PUSH_START_TIME				6			///< Time in deciseconds required to start the robot by pushing the hull.

#define D_315_GRAD					5.4977871	///< 315 degrees angle in rad.
#define D_270_GRAD					4.7123890	///< 270 degrees angle in rad.
#define D_265_GRAD					4.6251225	///< 265 degrees angle in rad.
#define D_95_GRAD					1.6580628	///< 95 degrees angle in rad.
#define D_90_GRAD					1.5707963	///< 90 degrees angle in rad.
#define D_85_GRAD					1.4835299	///< 85 degrees angle in rad.
#define D_45_GRAD					0.7853982	///< 45 degrees angle in rad.
#define NEAR_PI						3.054		///< Near PI for angle detection solves resolution problem.
#define NEAR_0						0.0872		///< Near  0 for angle detection solves resolution problem.

#define WEIGHT_PROPORTIONAL			2.9675		///< Proportion of change between the ADC reading and the weight in gramms.
#define WEIGHT_OFFSET				337.19		///< Offset of the ADC reading for approximate weight measurement.
#define WEIGHT_LOAD				150			///< Minimum weight to recognize as load.

#define STOP_HALL1				950			///<(670) Hall Sensor 1 ADC reading interpreted as a Hull Stop command.
#define STOP_HALL2_TOP				480			///<(480) Hall Sensor 2 ADC top limit interpreted as a Hull Stop command.
#define STOP_HALL2_BOTTOM			200			///<(200) Hall Sensor 2 ADC bottom limit interpreted as a Hull Stop command.


//Battery state thresholds
// BAT = N*0.015625 [V]
#define LOW_100						693		///< Lower voltage threshold for 100% charge.
#define TOP_75						683		///< Top voltage threshold for 75% charge.
#define LOW_75						661		///< Lower voltage threshold for 75% charge.
#define TOP_50						651		///< Top voltage threshold for 50% charge.
#define LOW_50						629		///< Lower voltage threshold for 50% charge.
#define TOP_25						619		///< Top voltage threshold for 25% charge.
#define LOW_25						597		///< Lower voltage threshold for 25% charge.
#define TOP_0						587		///< Top voltage threshold for 0% charge.
#define LOW_CHARGE					570		///< Voltage too low
#define BATT_OFF_TIME					20		///< Time in deciseconds for auto turn off if battery voltage is too low.

//Constants used by range to determine the US Sensors to ping.
#define ALL_SENSORS					0x00		///< Select all sensors
#define FRONT_SENSORS					0x01		///< Select frontal sensors

#define	SPEED_FW	100		///< Operation Speed Forwards (default: 70)
#define	SPEED_OB	70		///< Operation Speed Obstacle avoidance (default: 40)
#define SPEED_ROT	90		///< Operation Speed manual rotation (default: 70)

#endif
