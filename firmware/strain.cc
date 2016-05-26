#include "MKL05Z4.h"
#include <stdio.h>
#include <stdint.h>
#include "debugf.h"
#include "strain.h"

// PTB6 -> SCK
// PTB13 -> DATA
// PTA12 -> RATE

#define READ_DATA() ((FPTB->PDIR & _BV(13)) != 0)
#define SET_SCK() FPTB->PSOR = _BV(6)
#define CLEAR_SCK() FPTB->PCOR = _BV(6)

#define DELAY1() do {__NOP(); __NOP(); __NOP();} while(0)

long
poll_strain()
{
  long val;
  int i;

  if (READ_DATA())
    return STRAIN_BUSY;
  val = 0;
  for (i = 0; i < 24; i++) {
    SET_SCK();
    DELAY1();
    val <<= 1;
    CLEAR_SCK();
    DELAY1();
    if (READ_DATA())
      val |= 1;
  }
  // Extra pulse
  SET_SCK();
  DELAY1();
  CLEAR_SCK();
  return val;
}

static void
delay_us(int n)
{
  while (n--)
    DELAY1();
}

void
init_strain()
{
  PORTB->PCR[6] = PORT_PCR_MUX(1);
  PORTB->PCR[13] = PORT_PCR_MUX(1);
  PORTA->PCR[12] = PORT_PCR_MUX(1);

  FPTB->PDDR |= _BV(6);
  FPTB->PDDR &= ~_BV(13);
  FPTA->PDDR |= _BV(12);
  // 80Hz mode
  FPTA->PSOR = _BV(12);

  // Set SCK high to enter low power mode
  SET_SCK();
  delay_us(100);
  CLEAR_SCK();
}