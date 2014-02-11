/*******************************************************************************
***  Fraunhofer IPA
***  Robotersysteme
***  Projekt: Quirl  (Quelle: Raser)
********************************************************************************
********************************************************************************
***  Author: Sebastian Ott
*******************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H
/** \file config.h
 *
 * Constant parameters.
 *
 * Here are the parameters that aren't fixed by the Hardware, however aren't modified
 * at runtime. Examples: cycle time, Baud rate, Buffer sizes, etc.
 *  \version 1.0
 *  
 */



/// Period of cycle 0 (in 1/CLK) 1 ~ 68[ns]; 18 ~ 10 [us].
#define SCHED_PER0 18
/// Period of cycle 1 in times of SCHED_PER0 100 ~ 1 [ms].
#define SCHED_PER1 100
/// Period of cycle 2 in times of SCHED_PER1 10 ~ 10 [ms].
#define SCHED_PER2 10
/// Period of cycle 3 in times of SCHED_PER2 10 ~ 100 [ms].
#define SCHED_PER3  10

// UARTs

/// 1, if USART0 is used, otherwise 0
#define USART0_USED 1
/// Baud rate for USART0
#define USART0_BAUD 230400
/// Send buffer size for USART0
#define USART0_SENDBUFSIZE 100
/// Receive buffer size for USART0
#define USART0_RECVBUFSIZE 4

/// 1, if USART1 is used, otherwise 0
#define USART1_USED 1
/// Baud rate for USART1
#define USART1_BAUD 19200
/// Send buffer size for USART1
#define USART1_SENDBUFSIZE 100
/// Receive buffer size for USART1
#define USART1_RECVBUFSIZE 100

/// 1, if USART2 is used, otherwise 0
#define USART2_USED 1
/// Baud rate for USART2
#define USART2_BAUD 9600
/// Send buffer size for USART2
#define USART2_SENDBUFSIZE 4
/// Receive buffer size for USART2
#define USART2_RECVBUFSIZE 100

/// 1, if USART3 is used, otherwise 0
#define USART3_USED 0
/// Baud rate for USART3
#define USART3_BAUD 9600
/// Send buffer size for USART3
#define USART3_SENDBUFSIZE 4
/// Receive buffer size for USART3
#define USART3_RECVBUFSIZE 4

#endif
