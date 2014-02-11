/*****************************************************************************
 *
 * AVRPROG compatible boot-loader
 * Version  : 0.80beta3 (May 2006)
 * Compiler : avr-gcc 3.4.6 / avr-libc 1.4.4
 * size     : depends on features and startup ( minmal features < 512 words)
 * by       : Martin Thomas, Kaiserslautern, Germany
 *            eversmith@heizung-thomas.de
 *            Additional code and improvements contributed by:
 *           - Uwe Bonnes
 *           - Bjoern Riemer
 *           - Olaf Rempel
 *
 * License  : Copyright (c) 2006 Martin Thomas
 *            Free to use. You have to mention the copyright
 *            owners in source-code and documentation of derived
 *            work. No warranty!
 *
 * Tested with ATmega8, ATmega16, ATmega32, ATmega128, AT90CAN128
 *
 * - Initial versions have been based on the Butterfly bootloader-code
 *   by Atmel Corporation (Authors: BBrandal, PKastnes, ARodland, LHM)
 *
 ****************************************************************************
 *
 *  See the makefile for information how to adapt the linker-settings to
 *  the selected Boot Size (BOOTSIZE=xxxx), the AVR clock-frequency and the
 *  MCU-type in
 *
 *  With BOOT_SIMPLE, minimal features and discarded int-vectors
 *  this bootloader has
 *  0x2CA - atmega8
 *  0x308 - atmega16
 *  0x322 - atmega169
 *  0x308 - atmega32
 *  0x34C - atmega128
 *  0x352 - at90can128
 *  bytes size and should fit into a 512 word (0x400 byte) bootloader-section.
 *  The above values depend on the compiler-version and may differ from
 *  your results.
 *
 ****************************************************************************/
/*
TODOs:
- check lock-bits set
- __bad_interrupt still linked even with modified 
linker-scripts which needs a default-handler,
"wasted": 3 words for AVR5 (>8kB), 2 words for AVR4
*/
// tabsize: 4

/* MCU frequency */
#ifndef F_CPU
// #define F_CPU 7372800
#define F_CPU   14745600
#endif

/* UART Baudrate */
// #define BAUDRATE 9600
// #define BAUDRATE 19200
#define BAUDRATE 230400

/* use "Double Speed Operation" */
//#define UART_DOUBLESPEED

/* use second UART on mega128 / can128 */
//#define UART_USE_SECOND

/* Device-Type:
   For AVRProg the BOOT-option is prefered 
   which is the "correct" value for a bootloader.
   avrdude may only detect the part-code for ISP */
#define DEVTYPE     DEVTYPE_BOOT
//#define DEVTYPE     DEVTYPE_ISP

/*
 * Pin "STARTPIN" on port "STARTPORT" in this port has to grounded
 * (active low) to start the bootloader
 */
#define BLPORT		PORTB
#define BLDDR		DDRB
#define BLPIN		PINB
#define BLPNUM		PINB0

/*
 * Select startup-mode
 * SIMPLE-Mode - Jump to bootloader main BL-loop if key is
 *   pressed (Pin grounded) "during" reset or jump to the
 *   application if the pin is not grounded. The internal
 *   pull-up resistor is enabled during the startup and
 *   gets disabled before the application is started.
 * POWERSAVE-Mode - Startup is separated in two loops
 *   which makes power-saving a little easier if no firmware
 *   is on the chip. Needs more memory
 * BOOTICE-Mode - to flash the JTAGICE upgrade.ebn file.
 *   No startup-sequence in this mode. Jump directly to the
 *   parser-loop on reset
 *   F_CPU in BOOTICEMODE must be 7372800 Hz to be compatible
 *   with the org. JTAGICE-Firmware
 * WAIT-mode waits 1 sec for the defined character if nothing 
 *    is recived then the user prog is started.
 */
//#define START_SIMPLE
#define START_WAIT
//#define START_POWERSAVE
//#define START_BOOTICE

/* character to start the bootloader in mode START_WAIT */
#define START_WAIT_UARTCHAR 'S'

/* wait 1s in START_WAIT mode (10ms steps) */
#define WAIT_VALUE 20

/*
 * enable/disable readout of fuse and lock-bits
 * (AVRPROG has to detect the AVR correctly by device-code
 * to show the correct information).
 */
//#define ENABLEREADFUSELOCK

/* enable/disable write of lock-bits
 * WARNING: lock-bits can not be reseted by bootloader (as far as I know)
 * Only protection no unprotection, "chip erase" from bootloader only
 * clears the flash but does no real "chip erase" (this is not possible
 * with a bootloader as far as I know)
 * Keep this undefined!
 */
//#define WRITELOCKBITS

#define VERSION_HIGH '0'
#define VERSION_LOW  '8'

#define GET_LOCK_BITS           0x0001
#define GET_LOW_FUSE_BITS       0x0000
#define GET_HIGH_FUSE_BITS      0x0003
#define GET_EXTENDED_FUSE_BITS  0x0002


#include <stdint.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
// #include <util/delay.h>
#include <util/delay.h>

#include "chipdef.h"

uint8_t gBuffer[SPM_PAGESIZE];

#if defined(BOOTLOADERHASNOVECTORS)
#warning "This Bootloader does not link interrupt vectors - see makefile"
/* make the linker happy - it wants to see __vector_default */
// void __vector_default(void) { ; }
void __vector_default(void) { ; }
#endif

static void sendchar(uint8_t data)
{
  while (!(UART_STATUS & (1<<UART_TXREADY)));
  UART_DATA = data;
}

static uint8_t recvchar(void)
{
  while (!(UART_STATUS & (1<<UART_RXREADY)));
  return UART_DATA;
}

static inline void eraseFlash(void)
{
  // erase only main section (bootloader protection)
  uint32_t addr = 0;
  while (APP_END > addr) {
    boot_page_erase(addr);		// Perform page erase
    boot_spm_busy_wait();		// Wait until the memory is erased.
    addr += SPM_PAGESIZE;
  }
  boot_rww_enable();
}

static inline void recvBuffer(pagebuf_t size)
{
  pagebuf_t cnt;
  uint8_t *tmp = gBuffer;

  for (cnt = 0; cnt < sizeof(gBuffer); cnt++)
    *tmp++ = (cnt < size) ? recvchar() : 0xFF;
}

static inline uint16_t writeFlashPage(uint16_t waddr, pagebuf_t size)
{
  uint32_t pagestart = (uint32_t)waddr<<1;
  uint32_t baddr = pagestart;
  uint16_t data;
  uint8_t *tmp = gBuffer;

  do {
    data = *tmp++;
    data |= *tmp++ << 8;
    boot_page_fill(baddr, data);	// call asm routine.

    baddr += 2;			// Select next word in memory
    size -= 2;			// Reduce number of bytes to write by two
  } while (size);		// Loop until all bytes written

  boot_page_write(pagestart);
  boot_spm_busy_wait();
  boot_rww_enable();		// Re-enable the RWW section

  return baddr>>1;
}

static inline uint16_t writeEEpromPage(uint16_t address, pagebuf_t size)
{
  uint8_t *tmp = gBuffer;

  do {
    EEARL = address;		// Setup EEPROM address
    EEARH = (address >> 8);
    EEDR = *tmp++;
    address++;			// Select next byte

#ifdef __AVR_ATmega8__
    EECR |= (1<<EEMWE);		// Write data into EEPROM
    EECR |= (1<<EEWE);
#else
    EECR |= (1<<EEMPE);		// Write data into EEPROM
    EECR |= (1<<EEPE);
#endif
    eeprom_busy_wait();

    size--;			// Decreas number of bytes to write
  } while (size);		// Loop until all bytes written

  return address;
}

static inline uint16_t readFlashPage(uint16_t waddr, pagebuf_t size)
{
  uint32_t baddr = (uint32_t)waddr<<1;
  uint16_t data;

  do {
#if defined(RAMPZ)
    data = pgm_read_word_far(baddr);
#else
    data = pgm_read_word_near(baddr);
#endif
    sendchar(data);		// send LSB
    sendchar((data >> 8));	// send MSB
    baddr += 2;			// Select next word in memory
    size -= 2;			// Subtract two bytes from number of bytes to read
  } while (size);				// Repeat until all block has been read

  return baddr>>1;
}

static inline uint16_t readEEpromPage(uint16_t address, pagebuf_t size)
{
  do {
    EEARL = address;		// Setup EEPROM address
    EEARH = (address >> 8);
    EECR |= (1<<EERE);		// Read EEPROM
    address++;			// Select next EEPROM byte

    sendchar(EEDR);			// Transmit EEPROM data to PC

    size--;				// Decrease number of bytes to read
  } while (size);				// Repeat until all block has been read

  return address;
}

#if defined(ENABLEREADFUSELOCK)
static uint8_t read_fuse_lock(uint16_t addr)
{
  uint8_t mode = (1<<BLBSET) | (1<<SPMEN);
  uint8_t retval;

  asm volatile
    (
     "movw r30, %3\n\t"		/* Z to addr */ \
     "sts %0, %2\n\t"		/* set mode in SPM_REG */ \
     "lpm\n\t"			/* load fuse/lock value into r0 */ \
     "mov %1,r0\n\t"			/* save return value */ \
     : "=m" (SPM_REG),
     "=r" (retval)
     : "r" (mode),
     "r" (addr)
     : "r30", "r31", "r0"
    );
  return retval;
}
#endif

static void send_boot(void)
{
  sendchar('A');
  sendchar('V');
  sendchar('R');
  sendchar('B');
  sendchar('O');
  sendchar('O');
  sendchar('T');
}

static void (*jump_to_app)(void) = 0x0000;

int main(void)
{
  uint16_t address = 0;
  uint8_t device = 0, val;

#ifdef START_POWERSAVE
  uint8_t OK = 1;
#endif

#ifndef START_WAIT
  BLDDR  &= ~(1<<BLPNUM);		// set as Input
  BLPORT |= (1<<BLPNUM);		// Enable pullup
#endif

  // Set baud rate
  UART_BAUD_HIGH = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
  UART_BAUD_LOW = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

#ifdef UART_DOUBLESPEED
  UART_STATUS = (1<<UART_DOUBLE);
#endif

  UART_CTRL = UART_CTRL_DATA;
  UART_CTRL2 = UART_CTRL2_DATA;

start:
  ;

#if defined(START_POWERSAVE)
  /*
     This is an adoption of the Butterfly Bootloader startup-sequence.
     It may look a little strange but separating the login-loop from
     the main parser-loop gives a lot a possibilities (timeout, sleep-modes
     etc.).
     */
  for(;OK;) {
    if ((BLPIN & (1<<BLPNUM))) {
      // jump to main app if pin is not grounded
      BLPORT &= ~(1<<BLPNUM);	// set to default
      jump_to_app();		// Jump to application sector

    } else {
      val = recvchar();
      /* ESC */
      if (val == 0x1B) {
        // AVRPROG connection
        // Wait for signon
        while (val != 'S')
          val = recvchar();

        send_boot();			// Report signon
        OK = 0;

      } else {
        sendchar('?');
      }
    }
    // Power-Save code here
  }

#elif defined(START_SIMPLE)

  if ((BLPIN & (1<<BLPNUM))) {
    // jump to main app if pin is not grounded
    BLPORT &= ~(1<<BLPNUM);		// set to default
    jump_to_app();			// Jump to application sector
  }

#elif defined(START_WAIT)

  uint16_t cnt = 0;
  uint8_t i;
  DDRB = 1<<PB3;
  DDRF |= (1<<PF7); PORTF |= (1<<PF7); // Tablobot's Power Pin

  while (1) {
    if (UART_STATUS & (1<<UART_RXREADY))
      if (UART_DATA == START_WAIT_UARTCHAR)
        break;

    if (cnt++ >= WAIT_VALUE) {
      DDRB = 0x00;
      PORTB = 0x00;
      //BLPORT &= ~(1<<BLPNUM);		// set to default
      jump_to_app();			// Jump to application sector
    }

    for(i=0; i<10; i++)
      _delay_ms(10);
    PINB |= 1 << PB3;
  }
  send_boot();

#elif defined(START_BOOTICE)
#warning "BOOTICE mode - no startup-condition"

#else
#error "Select START_ condition for bootloader in main.c"
#endif

  for(;;) {
    val = recvchar();
    // Autoincrement?
    if (val == 'a') {
      sendchar('Y');			// Autoincrement is quicker

      //write address
    } else if (val == 'A') {
      address = recvchar();		//read address 8 MSB
      address = (address<<8) | recvchar();
      sendchar('\r');

      // Buffer load support
    } else if (val == 'b') {
      sendchar('Y');					// Report buffer load supported
      sendchar((sizeof(gBuffer) >> 8) & 0xFF);	// Report buffer size in bytes
      sendchar(sizeof(gBuffer) & 0xFF);

      // Start buffer load
    } else if (val == 'B') {
      pagebuf_t size;
      size = recvchar() << 8;				// Load high byte of buffersize
      size |= recvchar();				// Load low byte of buffersize
      val = recvchar();				// Load memory type ('E' or 'F')
      recvBuffer(size);

      if (device == DEVTYPE) {
        if (val == 'F') {
          address = writeFlashPage(address, size);

        } else if (val == 'E') {
          address = writeEEpromPage(address, size);
        }
        sendchar('\r');

      } else {
        sendchar(0);
      }

      // Block read
    } else if (val == 'g') {
      pagebuf_t size;
      size = recvchar() << 8;				// Load high byte of buffersize
      size |= recvchar();				// Load low byte of buffersize
      val = recvchar();				// Get memtype

      if (val == 'F') {
        address = readFlashPage(address, size);

      } else if (val == 'E') {
        address = readEEpromPage(address, size);
      }

      // Chip erase
    } else if (val == 'e') {
      if (device == DEVTYPE)
        eraseFlash();

      sendchar('\r');

      // Exit upgrade
    } else if (val == 'E') {
      //WMB wdt_enable(WDTO_15MS); // Enable Watchdog Timer to give reset
      sendchar('\r');
      //_delay_ms(10);
      goto start;

#ifdef WRITELOCKBITS
#warning "Extension 'WriteLockBits' enabled"
      // TODO: does not work reliably
      // write lockbits
    } else if (val == 'l') {
      if (device == DEVTYPE) {
        // write_lock_bits(recvchar());
        boot_lock_bits_set(recvchar());	// boot.h takes care of mask
        boot_spm_busy_wait();
      }
      sendchar('\r');
#endif
      // Enter programming mode
    } else if (val == 'P') {
      sendchar('\r');

      // Leave programming mode
    } else if (val == 'L') {
      sendchar('\r');

      // return programmer type
    } else if (val == 'p') {
      sendchar('S');		// always serial programmer

#ifdef ENABLEREADFUSELOCK
#warning "Extension 'ReadFuseLock' enabled"
      // read "low" fuse bits
    } else if (val == 'F') {
      sendchar(read_fuse_lock(GET_LOW_FUSE_BITS));

      // read lock bits
    } else if (val == 'r') {
      sendchar(read_fuse_lock(GET_LOCK_BITS));

      // read high fuse bits
    } else if (val == 'N') {
      sendchar(read_fuse_lock(GET_HIGH_FUSE_BITS));

      // read extended fuse bits
    } else if (val == 'Q') {
      sendchar(read_fuse_lock(GET_EXTENDED_FUSE_BITS));
#endif

      // Return device type
    } else if (val == 't') {
      sendchar(DEVTYPE);
      sendchar(0);

      // clear and set LED ignored
    } else if ((val == 'x') || (val == 'y')) {
      recvchar();
      sendchar('\r');

      // set device
    } else if (val == 'T') {
      device = recvchar();
      sendchar('\r');

      // Return software identifier
    } else if (val == 'S') {
      send_boot();

      // Return Software Version
    } else if (val == 'V') {
      sendchar(VERSION_HIGH);
      sendchar(VERSION_LOW);

      // Return Signature Bytes (it seems that 
      // AVRProg expects the "Atmel-byte" 0x1E last
      // but shows it first in the dialog-window)
    } else if (val == 's') {
      sendchar(SIG_BYTE3);
      sendchar(SIG_BYTE2);
      sendchar(SIG_BYTE1);

      /* ESC */
    } else if(val != 0x1b) {
      sendchar('?');
    }
  }
  return 0;
}
