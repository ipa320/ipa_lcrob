/*******************************************************************************
***  Fraunhofer IPA
***  Robotersysteme
***  Projekt: Quirl  (Quelle: Raser)
********************************************************************************
********************************************************************************
***  Author: Winfried Baum
*******************************************************************************/
/** \file stdio_init.h
 * 
 * I/O Redirection
 *
 * Functions to overload the I/O statements to work with the USART0.
 *  \author Winfried Baum
 */

#ifndef STDIO_INIT_H
#define STDIO_INIT_H

/** Redirects the standard I/O to USART0
 */
void stdio_init(void);

#endif
