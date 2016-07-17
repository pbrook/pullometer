#ifndef IMU_H
#define IMU_H

#include "common.h"

#define ADXL_I2C_ADDR 0x53u
#define ITG_I2C_ADDR 0x68u

#define IMU_TICK_MS 10

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
    void wake();
    bool poll(bool sleep=false);
};

class itg_gyro : public i2c_slave
{
public:
    itg_gyro() : i2c_slave(ITG_I2C_ADDR) {}
    void init();
    void sleep();
    void wake();
    void poll();
};

void init_imu(void);
void imu_tick(void);
void imu_set_accel(float x, float y, float z);
void imu_set_gyro(float x, float y, float z);
int16_t imu_current_pos();

#endif
