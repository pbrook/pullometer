#include "imu.h"
#include "debugf.h"

//I2C i2c(PTB4, PTB3);

static void
i2c_wait()
{
  while ((I2C0->S & I2C_S_IICIF_MASK) == 0)
    /* no-op */;
  I2C0->S |= I2C_S_IICIF_MASK;
  while ((I2C0->S & I2C_S_TCF_MASK) == 0)
    /* no-op */;
}

static void
i2c_do_addr(uint8_t addr)
{
  if (I2C0->S & I2C_S_BUSY_MASK) {
      I2C0->C1 |= I2C_C1_RSTA_MASK;
  } else {
      I2C0->C1 |= I2C_C1_MST_MASK;
      I2C0->C1 |= I2C_C1_TX_MASK;
  }
  I2C0->D = addr;
  i2c_wait();
}

static void
i2c_write(uint8_t addr, const uint8_t *data, int len, bool stop)
{
  i2c_do_addr(addr << 1);
  while (len != 0) {
      I2C0->D = *data;
      data++;
      len--;
      i2c_wait();
  }
  if (stop) {
      I2C0->C1 &= ~I2C_C1_MST_MASK;
  }
}

static void
i2c_read(uint8_t addr, uint8_t *data, int len)
{
  bool dummy;
  i2c_do_addr((addr << 1) | 1);
  I2C0->C1 &= ~I2C_C1_TX_MASK;
  // Discard the first byte read to flush the data register.
  dummy = true;
  len++;
  while (len != 0) {
      len--;
      if (len == 1) {
	  // Indicate last byte by suppressing ACK.
	  I2C0->C1 |= I2C_C1_TXAK_MASK;
      } else if (len == 0) {
	  // All bytes transferred, send STOP.
	  I2C0->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TXAK_MASK);
      }
      *data = I2C0->D;
      if (len == 0)
	return;
      i2c_wait();
      if (dummy)
	dummy = false;
      else
	data++;
  }
}

uint8_t
i2c_slave::read_reg(uint8_t reg)
{
  uint8_t val;

  i2c_write(addr, &reg, 1, false);
  val = 0xff;
  i2c_read(addr, &val, 1);
  return val;
}

void
i2c_slave::read_block(uint8_t reg, uint8_t len, uint8_t *data)
{
  i2c_write(addr, &reg, 1, false);
  i2c_read(addr, data, len);
}

void
i2c_slave::write_reg(uint8_t reg, uint8_t val)
{
  uint8_t buf[2];

  buf[0] = reg;
  buf[1] = val;
  i2c_write(addr, buf, 2, true);
}

static inline void
i2c_delay()
{
  int i;
  for (i = 0 ; i < 100; i++) __NOP();
}

static void
i2c_debrick(void)
{
  int n;
  PORTB->PCR[3] = PORT_PCR_MUX(1);
  PORTB->PCR[4] = PORT_PCR_MUX(1);

  FPTB->PSOR = _BV(3) | _BV(4);
  // Output clocks until data line released.
  while ((FPTB->PDIR & _BV(4)) == 0) {
      FPTB->PDDR |= _BV(3);
      i2c_delay();
      FPTB->PDDR &= ~_BV(3);
      i2c_delay();
  }
  // Generate STOP condition
  FPTB->PDDR |= _BV(4);
  i2c_delay();
  FPTB->PDDR &= ~_BV(4);
  i2c_delay();

  PORTB->PCR[3] = PORT_PCR_MUX(2);
  PORTB->PCR[4] = PORT_PCR_MUX(2);
}

void
init_imu(void)
{
  SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
  i2c_debrick();
  I2C0->F = 0;
  I2C0->C1 = I2C_C1_IICEN_MASK;
}
