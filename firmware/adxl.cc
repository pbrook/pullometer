#include "debugf.h"
#include "imu.h"

#define ADXL_I2C_ADDR 0x53u

#define ADXL_ID 0x00
#define ADXL_THRESH_ACT 0x24
#define ADXL_ACT_INACT_CTL 0x27
#define ADXL_BW_RATE 0x2c
#define ADXL_POWER_CTL 0x2d
#define ADXL_INT_ENABLE 0x2e
#define ADXL_INT_MAP 0x2f
#define ADXL_INT_SOURCE 0x30
#define ADXL_DATA_FORMAT 0x31
#define ADXL_DATA 0x32
#define ADXL_FIFO_CTL 0x38
#define ADXL_FIFO_STATUS 0x39

#define ADXL_ACT_AC _BV(7)
#define ADXL_ACT_X _BV(6)
#define ADXL_ADC_Y _BV(5)
#define ADXL_ACT_Z _BV(4)

#define ADXL_POWER_CTL_8HZ 0
#define ADXL_POWER_CTL_4HZ 1
#define ADXL_POWER_CTL_2HZ 2
#define ADXL_POWER_CTL_1HZ 3
#define ADXL_POWER_CTL_SLEEP _BV(2)
#define ADXL_POWER_CTL_MEASURE _BV(3)
#define ADXL_POWER_CTL_AUTO_SLEEP _BV(4)
#define ADXL_POWER_CTL_LINK _BV(5)

#define ADXL_BW_0HZ1 0x0
#define ADXL_BW_0HZ2 0x1
#define ADXL_BW_0HZ4 0x2
#define ADXL_BW_0HZ8 0x3
#define ADXL_BW_1HZ 0x4
#define ADXL_BW_3HZ 0x5
#define ADXL_BW_6HZ 0x6
#define ADXL_BW_12HZ 0x7
#define ADXL_BW_25HZ 0x8
#define ADXL_BW_50HZ 0x9
#define ADXL_BW_100HZ 0xa
#define ADXL_BW_200HZ 0xb
#define ADXL_BW_400HZ 0xc
#define ADXL_BW_800HZ 0xd
#define ADXL_BW_1600HZ 0xe
#define ADXL_BW_3200HZ 0xf

#define ADXL_INT_DATA_READY _BV(7)
#define ADXL_INT_SINGLE_TAP _BV(6)
#define ADXL_INT_DOUBLE_TAP _BV(5)
#define ADXL_INT_ACTIVITY _BV(4)
#define ADXL_INT_INACTIVITY _BV(3)
#define ADXL_INT_FREE_FALL _BV(2)
#define ADXL_INT_WATERMARK _BV(1)
#define ADXL_INT_OVERRUN _BV(0)

#define ADXL_FORMAT_SELF_TEST _BV(7)
#define ADXL_FORMAT_SPI _BV(6)
#define ADXL_FORMAT_INT_INVERT _BV(5)
#define ADXL_FORMAT_ _BV(4)
#define ADXL_FORMAT_FULL_RES _BV(3)
#define ADXL_FORMAT_JUSTIFY _BV(2)
#define ADXL_FORMAT_2G 0
#define ADXL_FORMAT_4G 1
#define ADXL_FORMAT_8G 2
#define ADXL_FORMAT_16G 3

#define ADXL_FIFO_MODE_BYPASS 0
#define ADXL_FIFO_MODE_FIFO (1u << 6)
#define ADXL_FIFO_MODE_STREAM (2u << 6)
#define ADXL_FIFO_MODE_TRIGGER (3u << 6)
#define ADXL_FIFO_TRIGGER (3u << 6)
#define ADXL_FIFO_SAMPLES 0x1f

void
adxl::sleep()
{
  write_reg(ADXL_POWER_CTL, 0);
  write_reg(ADXL_POWER_CTL, ADXL_POWER_CTL_MEASURE | ADXL_POWER_CTL_SLEEP | ADXL_POWER_CTL_4HZ);
}

void
adxl::init()
{
  write_reg(ADXL_POWER_CTL, 0);
  write_reg(ADXL_INT_ENABLE, 0);
  write_reg(ADXL_INT_MAP, 0);
  write_reg(ADXL_BW_RATE, ADXL_BW_50HZ);
  write_reg(ADXL_FIFO_CTL, 1);
  write_reg(ADXL_DATA_FORMAT, ADXL_FORMAT_4G);
  write_reg(ADXL_THRESH_ACT, 8); // 0.5G
  write_reg(ADXL_ACT_INACT_CTL, ADXL_ACT_AC | ADXL_ACT_X | ADXL_ADC_Y | ADXL_ACT_Z);
  read_reg(ADXL_INT_SOURCE);
  write_reg(ADXL_INT_ENABLE, ADXL_INT_ACTIVITY | ADXL_INT_DATA_READY);
  write_reg(ADXL_POWER_CTL, ADXL_POWER_CTL_MEASURE | ADXL_POWER_CTL_SLEEP | ADXL_POWER_CTL_4HZ);
}

void
adxl::poll()
{
  uint8_t status;
  uint8_t data[6];
  int x, y, z;
  static int readings;

  status = read_reg(ADXL_INT_SOURCE);
  if (status & ADXL_INT_ACTIVITY) {
      if (readings == 0) {
	  write_reg(ADXL_POWER_CTL, 0);
	  write_reg(ADXL_POWER_CTL, ADXL_POWER_CTL_MEASURE | ADXL_POWER_CTL_4HZ);
      }
      readings = 10;
  }
  if (status & ADXL_INT_DATA_READY) {
      read_block(ADXL_DATA, 6, data);
      x = data[0] | (data[1] << 8);
      y = data[2] | (data[3] << 8);
      z = data[4] | (data[5] << 8);
      debugf("%04x %04x %04x\r\n", x, y, z);
      if (readings == 0) {
	  sleep();
      } else {
	  readings--;
      }

  } else {
      for (int i = 0; i < 100000; i++) __NOP();
  }
}
