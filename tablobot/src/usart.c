/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Raser 
****************************************************************************
****************************************************************************
***  Autor: Winfried Baum
***************************************************************************/
/** \file usart.c
 * 
 * USART Driver.
 *
 * Template to work with the USART interfaces.
 *  \author Winfried Baum
 */
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"

/// Template for the Interrupt Handlers of the USARTs
#define INT_HANDLER(n)                                 \
struct sendbuf##n##_t sendbuf##n;                      \
                                                       \
struct recvbuf##n##_t recvbuf##n;                      \
                                                       \
ISR(USART##n##_UDRE_vect) {                            \
  if(sendbuf##n.count) {                               \
    UDR##n = sendbuf##n.data[sendbuf##n.rdind];        \
    if(++sendbuf##n.rdind == USART##n##_SENDBUFSIZE)   \
      sendbuf##n.rdind = 0;                            \
    sendbuf##n.count--;                                \
  }                                                    \
  else                                                 \
    UCSR##n##B &= ~(1<<UDRIE##n);                      \
}                                                      \
                                                       \
void usart##n##_sendbyte(uint8_t b) {                  \
  if(sendbuf##n.count == 0 &&                          \
      UCSR##n##A & 1<<UDRE##n) {                       \
    UDR##n = b;                                        \
    UCSR##n##B |= 1<<UDRIE##n;                         \
  }                                                    \
  else if(sendbuf##n.count < USART##n##_SENDBUFSIZE) { \
    sendbuf##n.data[sendbuf##n.wrind] = b;             \
    if(++sendbuf##n.wrind == USART##n##_SENDBUFSIZE)   \
      sendbuf##n.wrind = 0;                            \
    cli();                                             \
    sendbuf##n.count++;                                \
    sei();                                             \
  }                                                    \
}                                                      \
                                                       \
char usart##n##_recvbyte() {                           \
  	char ch;                                       \
  	while(!(UCSR##n##A & (1<<RXC##n)));            \
	ch = UDR##n;					\
	if(ch == '\r')					\
    		ch = '\n';				\
  	usart##n##_sendbyte(ch);			\
	return ch;                                      \
}                                                      \
							\
void usart##n##_puts (char *s)				\
{							\
    while (*s)						\
    {   						\
        usart##n##_sendbyte(*s);			\
        s++;						\
    }							\
}	


// Setup of the used Interrupt Handlers
#if USART0_USED
INT_HANDLER(0)
#endif
#if USART1_USED
INT_HANDLER(1)
#endif
#if USART2_USED
INT_HANDLER(2)
#endif
#if USART3_USED
INT_HANDLER(3)
#endif

/// Template for the initialization sequence of the USARTs
#define INIT(n) do { 							\
	UCSR##n##A = 0;							\
	UCSR##n##B = 0<<RXCIE##n | 0<<UDRIE##n | 1<<RXEN##n | 1<<TXEN##n; \
	UCSR##n##C = 1<<UCSZ##n##1 | 1<<UCSZ##n##0;                       \
	UBRR##n = F_CPU / 16 / USART##n##_BAUD - 1;                       \
  } while(0)

void usart_init(void) {
  // Initialization of the used USARTs
#if USART0_USED
  INIT(0);
#endif
#if USART1_USED
  INIT(1);
#endif
#if USART2_USED
  INIT(2);
#endif
#if USART3_USED
  INIT(3);
#endif
}
