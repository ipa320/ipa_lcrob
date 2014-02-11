/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Raser 
****************************************************************************
****************************************************************************
***  Autor: Winfried Baum
***************************************************************************/
#ifndef ADC_H
#define ADC_H

/** \file adc.h
 *
 * Functions to read analog values through ADC.
 *  \author Winfried Baum
 */

/** Initialization of the ADC.
 */
void adc_init(void);

/** Cyclic reading of analog values.
 * This function must be called cyclically. In every cycle a channel of the ADC
 * will be read in the order(0,1,2,3,4,5,14,15) and written into a buffer.
 */
void adc_cycle(void);

/** Reads a buffered analog value.
 * The last value written on the buffer by the cyclic call of adc_cycle()
 * will be read.
 *
 * \param chan Channel number. The channel names are found in
 * interfaces.h under ADC_*.
 *
 * \return Buffered analog value. The minimum value is 0 (0 Volt), and the
 * maximum 1023 (5 V).
 */
uint16_t adc_read(uint8_t chan);

#endif
