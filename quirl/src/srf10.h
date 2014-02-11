/*******************************************************************************
***  Fraunhofer IPA
***  Robotersysteme
***  Projekt: Quirl  (Quelle: Tabl-O-bot)
********************************************************************************
********************************************************************************
***  Author: Alejandro Merello 
*******************************************************************************/

/** \file srf10.h
 *  \brief SRF10 Ultrasonic Ranger driver.
 *  
 *  Driver for the SRF10 Ultrasonic Ranger.
 *  
 *  version 1.0
 */

#ifndef __SRF10_H__
#define __SRF10_H__

//US Read registers:
#define SW_VERSION  		0x00	///< Software Revision
#define VAL_MSB			0x02	///< Range High Byte
#define VAL_LSB			0x03	///< Range Low Byte

//US Write registers:
#define US_REG 			0x00	///< Instructions register
#define GAIN_REG		0x01	///< Gain set register
#define RANGE_REG		0x02	///< Range set register

//US instructions:
#define	SRF10_INCH		0x50	///< Ranging Mode - Result in inches
#define SRF10_CM		0x51	///< Ranging Mode - Result in centimeters
#define SRF10_MS		0x52	///< Ranging Mode - Result in micro-seconds

//US addresses:
#define SRF10_UNIT_0	0xE0	///< SRF10 Unit Address 0 (Front Left US Sensor)
#define SRF10_UNIT_1	0xE2	///< SRF10 Unit Address 1 (Front Right US Sensor)
#define SRF10_UNIT_2	0xE4	///< SRF10 Unit Address 2 (Lateral US Sensor)
#define SRF10_UNIT_3	0xE6	///< SRF10 Unit Address 3
#define SRF10_UNIT_4	0xE8	///< SRF10 Unit Address 4
#define SRF10_UNIT_5	0xEA	///< SRF10 Unit Address 5
#define SRF10_UNIT_6	0xEC	///< SRF10 Unit Address 6
#define SRF10_UNIT_7	0xEE	///< SRF10 Unit Address 7
#define SRF10_UNIT_8	0xF0	///< SRF10 Unit Address 8
#define SRF10_UNIT_9	0xF2	///< SRF10 Unit Address 9
#define SRF10_UNIT_10	0xF4	///< SRF10 Unit Address 10
#define SRF10_UNIT_11	0xF6	///< SRF10 Unit Address 11
#define SRF10_UNIT_12	0xF8	///< SRF10 Unit Address 12
#define SRF10_UNIT_13	0xFA	///< SRF10 Unit Address 13
#define SRF10_UNIT_14	0xFC	///< SRF10 Unit Address 14
#define SRF10_UNIT_15	0xFE	///< SRF10 Unit Address 15

//Range distances: 
//(For manual setting the range is calculated as: Range register*43[mm] + 43[mm])
#define RANGE_43MM	0x00	///< Set Maximum Range Distance to 43[mm]
#define RANGE_86MM	0x01	///< Set Maximum Range Distance to 86[mm]
#define	RANGE_1M	0x18	///< Set Maximum Range Distance to 1 [m]
#define RANGE_4M	0x5D	///< Set Maximum Range Distance to 4 [m]
#define RANGE_11M	0xFF	///< Set Maximum Range Distance to 11[m]	(default)

//Analogue gain:
#define GAIN_40		0x01	///< Set Maximum Analogue Gain to 40
#define GAIN_50		0x02	///< Set Maximum Analogue Gain to 50
#define GAIN_60		0x03	///< Set Maximum Analogue Gain to 60
#define GAIN_70		0x04	///< Set Maximum Analogue Gain to 70
#define GAIN_80		0x05	///< Set Maximum Analogue Gain to 80
#define GAIN_100	0x06	///< Set Maximum Analogue Gain to 100
#define GAIN_120	0x07	///< Set Maximum Analogue Gain to 120
#define GAIN_140	0x08	///< Set Maximum Analogue Gain to 140
#define GAIN_200	0x09	///< Set Maximum Analogue Gain to 200
#define GAIN_250	0x0A	///< Set Maximum Analogue Gain to 250
#define GAIN_300	0x0B	///< Set Maximum Analogue Gain to 300
#define GAIN_350	0x0C	///< Set Maximum Analogue Gain to 350
#define GAIN_400	0x0D	///< Set Maximum Analogue Gain to 400
#define GAIN_500	0x0E	///< Set Maximum Analogue Gain to 500
#define GAIN_600	0x0F	///< Set Maximum Analogue Gain to 600
#define GAIN_700	0x10	///< Set Maximum Analogue Gain to 700	(default)

/**Ranging request to sensor in centimiters.
 * \param sensor Address of the sensor.
 */
void srf10_range_req(unsigned char sensor);


/**Waits while sensor is busy
 * \param sensor Address of the sensor.
 */
void srf10_busy(unsigned char sensor);


/**Reads the ranging value of a sensor.
 * \param sensor Address of the sensor.
 * \return 16 bit result of the ultrasonic ranging.
 */
uint16_t srf10_range_read(unsigned char sensor);


/**Function to change the range of the sensor, 
 * the range is calculated as: Range register*43[mm] + 43[mm]
 * (See header file for some useful values)
 * \param sensor Address of the sensor.
 * \param range New range (default: 11[m])
 */
void srf10_set_range(unsigned char sensor,unsigned char range);


/** Function to change the analogue gain of the sensor.
 * \param sensor Address of the sensor.
 * \param gain New gain (default: 700)
 */
void srf10_set_gain(unsigned char sensor, unsigned char gain);

/** Changes the I2C Address of a SRF10 Ultrasonic Ranger.
 * \param sensor Current address of the sensor.
 * \param new_address New address of the sensor.
 * \warning To change the I2C Address of the SRF10 US Sensor there must be only ONE module connected on the bus
 */
void srf10_change_address(unsigned char sensor,unsigned char new_address);

#endif //__SRF10_H__
