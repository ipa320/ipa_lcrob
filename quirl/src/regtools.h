/*******************************************************************************
***  Fraunhofer IPA
***  Robotersysteme
***  Projekt: Quirl  (Quelle: Raser)
********************************************************************************
********************************************************************************
***  Author: Winfried Baum
*******************************************************************************/
#ifndef REGTOOLS_H
#define REGTOOLS_H


/** \file regtools.h
 *
 * Functions to simplify the control of the I/O Registers.
 *  \author Winfried Baum
 */

/** Converts a number in binary representation into an integer.
 * The binary representation can be from 1 to 8 digits long. Leading zeros are optional.
 */
#define B(n) ( \
    0##n >>  0 &   1 | \
    0##n >>  2 &   2 | \
    0##n >>  4 &   4 | \
    0##n >>  6 &   8 | \
    0##n >>  8 &  16 | \
    0##n >> 10 &  32 | \
    0##n >> 12 &  64 | \
    0##n >> 14 & 128 )

/** Data structure to access the bits of a byte.
 *
 * Used in SBIT() macro.
 */
struct bitsinbyte {
  unsigned int bit0:1, bit1:1, bit2:1, bit3:1, bit4:1, bit5:1, bit6:1, bit7:1;
};

/** Access of a single bit from an I/O Port.
 *
 * @param port  I/O-Port
 * @param pin   Pin or Bit number
 */
#define SBIT(port,pin) (((volatile struct bitsinbyte *)&port)->bit##pin)

#endif
