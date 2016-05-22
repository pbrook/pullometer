#include "imu.h"

#define ITG_WHOAMI 0x00
#define ITG_POWER 0x3e

#define ITG_POWER_CLK_SEL_INTERNAL 0
#define ITG_POWER_CLK_SEL_X 1
#define ITG_POWER_STBY_ZG _BV(3)
#define ITG_POWER_STBY_YG _BV(4)
#define ITG_POWER_STBY_XG _BV(5)
#define ITG_POWER_SLEEP _BV(6)
#define ITG_POWER_H_RESET _BV(7)

void
itg_gyro::init()
{
}

void
itg_gyro::sleep()
{
    write_reg(ITG_POWER, ITG_POWER_SLEEP);
}
