/*******************************************************************************
***  Fraunhofer IPA
***  Robotersysteme
***  Projekt: Quirl  (Quelle: Raser)
********************************************************************************
********************************************************************************
***  Author: Winfried Baum
*******************************************************************************/
/*  Version: 1.0 
 ** \file stdio_init.c
 * 
 * I/O Redirection
 *
 * Functions to overload the I/O statements to work with the USART0.
 */

#include <stdio.h>
#include "stdio_init.h"
#include "usart.h"

/** Receives character from USART0
 * \param f
 * \return Character read from USART0
 */
int mygetchar(FILE *f) {
  uint8_t b = usart0_recvbyte();

  usart0_sendbyte(b);
  if(b == '\r')
    usart0_sendbyte('\n');
  return (int)b;
}

/** Sends character to USART0
 * \param c Character to send
 * \param f
 * \return Character sent
 */
int myputchar(char c, FILE *f) {
  if(c == '\n')
    usart0_sendbyte('\r');
  usart0_sendbyte((char)c);
  return (int)c;
}

static FILE mystdin = FDEV_SETUP_STREAM(NULL, mygetchar, _FDEV_SETUP_READ);

static FILE mystdout = FDEV_SETUP_STREAM(myputchar, NULL, _FDEV_SETUP_WRITE);

void stdio_init(void) {
  stdin = &mystdin;
  stdout = &mystdout;
}
