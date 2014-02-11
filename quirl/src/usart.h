/*******************************************************************************
***  Fraunhofer IPA
***  Robotersysteme
***  Projekt: Quirl  (Quelle: Raser)
********************************************************************************
********************************************************************************
***  Author: Winfried Baum
*******************************************************************************/
/** \file usart.h
 * 
 * USART Driver.
 *
 * Template to work with the USART interfaces.
 *  \author Winfried Baum
 */

#ifndef USART_H
#define USART_H
#include <stdint.h>
#include "config.h"

/// Initialization of the used USARTs
void usart_init(void);

///I/O Functions for the USARTs
#define DECLS(n)                                       \
extern struct sendbuf##n##_t {                         \
  uint8_t rdind, wrind;                                \
  volatile uint8_t count;                              \
  char data[USART##n##_SENDBUFSIZE];                \
} sendbuf##n;                                          \
                                                       \
extern struct recvbuf##n##_t {                         \
  uint8_t rdind, wrind;                                \
  volatile uint8_t count;                              \
  uint8_t data[USART##n##_RECVBUFSIZE];                \
} recvbuf##n;                                          \
                                                       \
void usart##n##_sendbyte(uint8_t b);                   \
char usart##n##_recvbyte();                         \
				                       \
                                                       \
static inline uint8_t usart##n##_free(void) {          \
  return USART##n##_SENDBUFSIZE - sendbuf##n.count;    \
}                                                      \
                                                       \
static inline uint8_t usart##n##_avail(void) {         \
  return recvbuf##n.count;                             \
}							\
							\
void usart##n##_puts (char *s);

#if USART0_USED
DECLS(0)
#endif
#if USART1_USED
DECLS(1)
#endif
#if USART2_USED
DECLS(2)
#endif
#if USART3_USED
DECLS(3)
#endif

#endif
