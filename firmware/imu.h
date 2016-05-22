#ifndef IMU_H
#define IMU_H

#include "MKL05Z4.h"

#define ADXL_I2C_ADDR 0x53u
#define ITG_I2C_ADDR 0x68u

#ifndef _BV
#define _BV(n) (1u << (n))
#endif

class i2c_slave {
  public:
    uint8_t addr;
    i2c_slave(uint8_t _addr): addr(_addr) {}
    uint8_t read_reg(uint8_t addr);
    void write_reg(uint8_t addr, uint8_t val);
    void read_block(uint8_t reg, uint8_t len, uint8_t *data);
};

class adxl : public i2c_slave
{
public:
  adxl() : i2c_slave(ADXL_I2C_ADDR) {}
  void init();
  void sleep();
  void poll();
};

class itg_gyro : public i2c_slave
{
public:
  itg_gyro() : i2c_slave(ITG_I2C_ADDR) {}
  void init();
  void sleep();
};

void init_imu(void);

#endif
