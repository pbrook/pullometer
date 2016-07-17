#include <debugf.h>
#include "imu.h"
#include <math.h>

#define ITG_WHOAMI 0x00
#define ITG_SRD 0x15
#define ITG_DLPF 0x16
#define ITG_INT 0x17
#define ITG_STATUS 0x1a
#define ITG_TEMP 0x1b
#define ITG_GYRO 0x1d
#define ITG_POWER 0x3e

#define ITG_DLPF_CFG_256Hz 0
#define ITG_DLPF_CFG_188Hz 1
#define ITG_DLPF_CFG_98Hz 2
#define ITG_DLPF_CFG_42Hz 3
#define ITG_DLPF_CFG_20Hz 4
#define ITG_DLPF_CFG_10Hz 5
#define ITG_DLPF_CFG_5Hz 6
#define ITG_DLPF_FS_SEL_2k (3 << 3)

#define ITG_INT_ACTL _BV(7)
#define ITG_INT_OPEN _BV(6)
#define ITG_INT_LATCH _BV(5)
#define ITG_INT_ANYRD _BV(4)
#define ITG_INT_PLL_EN _BV(2)
#define ITG_INT_DRDY_EN _BV(0)

#define ITG_STATUS_PLL _BV(2)
#define ITG_STATUS_DRDY _BV(0)

#define ITG_POWER_CLK_SEL_INTERNAL 0
#define ITG_POWER_CLK_SEL_X 1
#define ITG_POWER_CLK_SEL_Y 2
#define ITG_POWER_CLK_SEL_Z 3
#define ITG_POWER_STBY_ZG _BV(3)
#define ITG_POWER_STBY_YG _BV(4)
#define ITG_POWER_STBY_XG _BV(5)
#define ITG_POWER_SLEEP _BV(6)
#define ITG_POWER_H_RESET _BV(7)

void
itg_gyro::init()
{
    PORTA->PCR[8] = PORT_PCR_MUX(1);
    sleep();
}

void
itg_gyro::wake()
{
    uint8_t val;

    write_reg(ITG_POWER, 0x80);
    /* Wait for device to reset.  */
    while (read_reg(ITG_POWER) & 0x80);
    // 8kHz sample rate / 32 -> 250Hz update rate
    write_reg(ITG_SRD, 31);
    write_reg(ITG_DLPF, ITG_DLPF_FS_SEL_2k | ITG_DLPF_CFG_256Hz);
    write_reg(ITG_INT, ITG_INT_ACTL | ITG_INT_LATCH | ITG_INT_DRDY_EN);
    write_reg(ITG_POWER, ITG_POWER_CLK_SEL_X);
}

// 2000 degrees/sec full range signed 16bit int
#define SCALE (2000.0f * M_PI / (180 * 0x8000))
int gyro_z;

void
itg_gyro::poll(void)
{
    uint8_t data[6];
    int16_t x, y, z;
    uint8_t status;


    if (FPTA->PDIR & _BV(8)) {
        return;
    }
    status = read_reg(ITG_STATUS);
    if ((status & 1) == 0) {
        // Should never happen
        return;
    }

    read_block(ITG_GYRO, 6, data);
    x = (data[0] << 8) | data[1];
    y = (data[2] << 8) | data[3];
    z = (data[4] << 8) | data[5];
    imu_set_gyro(x * SCALE, y * SCALE, z * SCALE);
    gyro_z = z;
}

void
itg_gyro::sleep()
{
    write_reg(ITG_INT, ITG_INT_ACTL | ITG_INT_LATCH | ITG_INT_ANYRD);
    write_reg(ITG_POWER, ITG_POWER_SLEEP);
    read_reg(ITG_STATUS);
}
