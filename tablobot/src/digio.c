/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Raser 
****************************************************************************
****************************************************************************
***  Autor: Winfried Baum
***************************************************************************/
/** \file digio.c
 *
 * Ports configuration.
 *
 * Sets data direction registers and ports.
 *  \author Winfried Baum
 */
#include "interfaces.h"
#include "digio.h"

void digio_init() 
{
  PORTA = DEFAULTS_A;
  DDRA = OUTPUTS_A;
  PORTB = DEFAULTS_B;
  DDRB = OUTPUTS_B;
  PORTC = DEFAULTS_C;
  DDRC = OUTPUTS_C;
  PORTD = DEFAULTS_D;
  DDRD = OUTPUTS_D;
  PORTE = DEFAULTS_E;
  DDRE = OUTPUTS_E;
  PORTF = DEFAULTS_F;
  DDRF = OUTPUTS_F;
  PORTG = DEFAULTS_G;
  DDRG = OUTPUTS_G;
  PORTH = DEFAULTS_H;
  DDRH = OUTPUTS_H;
  PORTJ = DEFAULTS_J;
  DDRJ = OUTPUTS_J;
  PORTK = DEFAULTS_K;
  DDRK = OUTPUTS_K;
  PORTL = DEFAULTS_L;
  DDRL = OUTPUTS_L;			// Data Direction Register und Ports setzen
}
