#ifdef __MBED__
#include "mbed.h"

#include "USBSerial.h"
extern USBSerial serial;
#define debugf serial.printf

#ifndef _BV
#define _BV(n) (1u << (n))
#endif

#else
#include <stdlib.h>
#include <debugf.h>
#include "common.h"
#endif

#include "rf24.h"
#include "nRF24L01.h"

#ifdef __MBED__

static void
udelay(int n)
{
  wait_us(n);
}

SPI ser = SPI(PTD6, PTD7, PTD5);

DigitalOut cs(PTD4);
// Schematic says PTC12. It lies.
DigitalOut ce(PTB20);

#define SET_CS() cs.write(1)
#define CLEAR_CS() cs.write(0)
#define SET_CE() ce.write(1)
#define CLEAR_CE() ce.write(0)

static void
spi_init()
{
  ser.format(8, 0);
  ser.frequency(10000000);
}

static uint8_t
spi_transfer(uint8_t val)
{
  return ser.write(val);
}

#else

static void
udelay(int n)
{
  delay_us(n);
}

#define SET_CS() FPTA->PSOR = _BV(5)
#define CLEAR_CS() FPTA->PCOR = _BV(5)
#define SET_CE() FPTB->PSOR = _BV(7)
#define CLEAR_CE() FPTB->PCOR = _BV(7)

static void
spi_init()
{
  SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;
  SPI0->C1 = SPI_C1_SPE_MASK | SPI_C1_MSTR_MASK;
  SPI0->C2 = 0;
  SPI0->BR = SPI_BR_SPPR(0) | SPI_BR_SPR(7);
  PORTA->PCR[6] = PORT_PCR_MUX(3); /* MISO */
  PORTA->PCR[7] = PORT_PCR_MUX(3); /* MOSI */
  PORTB->PCR[0] = PORT_PCR_MUX(3); /* SCK */
}

static uint8_t
spi_transfer(uint8_t val)
{
  while ((SPI0->S & SPI_S_SPTEF_MASK) == 0)
    /* no-op.  */;
  SPI0->D  = val;
  while ((SPI0->S & SPI_S_SPRF_MASK) == 0)
    /* no-op.  */;
  return SPI0->D;
}

#endif

uint8_t
RF24::cmd(uint8_t cmd, const uint8_t *data_in, uint8_t *data_out, uint8_t data_len)
{
  uint8_t status;
  uint8_t val = 0;
  CLEAR_CS();
  status = spi_transfer(cmd);
  while (data_len > 0) {
      if (data_in)
	val = *(data_in++);
      val = spi_transfer(val);
      if (data_out)
	*(data_out++) = val;
      data_len--;
  }
  udelay(1);
  SET_CS();
  udelay(1);
  return status;
}

inline void
RF24::write_reg(uint8_t reg, uint8_t val)
{
  cmd(W_REGISTER | reg, &val, NULL, 1);
}

inline uint8_t
RF24::read_reg(uint8_t reg)
{
  uint8_t val;

  cmd(R_REGISTER | reg, NULL, &val, 1);
  return val;
}

void
RF24::init()
{
  uint8_t tmp;

#ifndef __MBED__
  /* CE */
  FPTB->PDDR |= _BV(7);
  PORTB->PCR[7] = PORT_PCR_MUX(1);
  /* CS */
  FPTA->PDDR |= _BV(5);
  PORTA->PCR[5] = PORT_PCR_MUX(1);
  /* IRQ */
  PORTB->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
#endif
  CLEAR_CE();
  SET_CS();
  spi_init();

  // 16-bit CRC
  write_reg(CONFIG, _BV(EN_CRC) | _BV(CRCO));
  // 5-byte address
  write_reg(SETUP_AW, 3);
  // 1ms timeout, 3 retries
  write_reg(SETUP_RETR, 0x7f);
  // Channel 76 (arbitrary)
  write_reg(RF_CH, 76);
  // 1Mbps, 0db TX power
  write_reg(RF_SETUP, _BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
  // Enable pipe 0 with auto-ack
  write_reg(EN_AA, 1);
  write_reg(EN_RXADDR, 1);

  // Dynamic payload length.
  write_reg(DYNPD, _BV(DPL_P0));
  // Enable dynamic payload length
  write_reg(FEATURE, _BV(EN_DPL));
  if ((read_reg(FEATURE) & _BV(EN_DPL)) == 0) {
      // If they did not stick then sent the ACTIVATE command and try again.
      tmp = 0x73;
      cmd(ACTIVATE, &tmp, NULL, 1);
      write_reg(FEATURE, _BV(EN_DPL));
  }

  write_reg(RX_PW_P0, 32);

  /* Clear status flags.  */
  tmp = read_status();
  tmp &= _BV(MAX_RT) | _BV(TX_DS) | _BV(RX_DR);
  write_reg(STATUS, tmp);
}

void RF24::wake()
{
  uint8_t tmp;
  // power on device
  write_reg(CONFIG, read_reg(CONFIG) | _BV(PWR_UP));
  // Wait 1.5ms
  for (tmp = 0; tmp < 10; tmp++)
    udelay(150);
}

void RF24::sleep()
{
  write_reg(CONFIG, read_reg(CONFIG) & ~_BV(PWR_UP));
}

void
RF24::set_address(uint32_t addr)
{
  address[1] = addr & 0xff;
  address[2] = (addr >> 8) & 0xff;
  address[3] = (addr >> 16) & 0xff;
  address[4] = (addr >> 24) & 0xff;
}

void
RF24::set_node(uint8_t reg, uint8_t node)
{
  address[0] = node;
  cmd(W_REGISTER | reg, address, NULL, 5);
}

void
RF24::rx_enable(uint8_t node)
{
  uint8_t config;
  int tmp;

  set_node(RX_ADDR_P0, node);
  config = read_reg(CONFIG);
  config &= ~_BV(PWR_UP);
  write_reg(CONFIG, config);
  udelay(10);
  config |= _BV(PWR_UP) | _BV(PRIM_RX);
  write_reg(CONFIG, config);
  // Wait 1.5ms
  for (tmp = 0; tmp < 10; tmp++)
    udelay(150);
  SET_CE();
  udelay(150);
}

uint8_t RF24::read_status()
{
  return cmd(NOP, NULL, NULL, 0);
}

// Returns payload length (or 0 if no packet present).
uint8_t
RF24::rx(uint8_t *buf)
{
  uint8_t status;
  uint8_t pipe;

  status = read_status();
  pipe = (status >> 1) & 7;
  if (pipe == 7)
    return 0;

  cmd(R_RX_PAYLOAD, NULL, buf, RF24_PACKET_LEN);
  return RF24_PACKET_LEN;
}

bool
RF24::tx_avail()
{
  return (read_status() & _BV(TX_FULL)) == 0;
}

void
RF24::tx(uint8_t node, const uint8_t *buf)
{
  cmd(FLUSH_TX, NULL, NULL, 0);
  set_node(RX_ADDR_P0, node);
  set_node(TX_ADDR, node);
  cmd(W_TX_PAYLOAD, buf, NULL, RF24_PACKET_LEN);
  SET_CE();
  udelay(15);
  CLEAR_CE();
}

int8_t
RF24::tx_poll()
{
  uint8_t status;

#ifndef __MBED__
  if (FPTB->PDIR & _BV(11))
    return -1;
#endif

  // Read status and clear interrupt flags
  status = read_status();
  //debugf("Status 0x%02x\n", status);
  status &= _BV(MAX_RT) | _BV(TX_DS) | _BV(RX_DR);
  write_reg(STATUS, status);
  if (status & _BV(MAX_RT)) {
    // Transmission failed (timeout)
    // Reset TX fifo
    cmd(FLUSH_TX, NULL, NULL, 0);
    return -2; 
  }
  if ((status & _BV(TX_DS)) == 0)
    return -1; // Transmission not yet complete
  return 0;
}
