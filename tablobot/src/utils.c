/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Raser 
****************************************************************************
****************************************************************************
***  Autor: Winfried Baum
***************************************************************************/
/** \file utils.c
 * 
 * Delay function declaration
 *
 *  \author Winfried Baum
 */

#include "utils.h"

void delay(uint16_t ms) {
  while(ms--)
    _delay_ms(1);
}
