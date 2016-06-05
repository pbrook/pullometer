#ifndef COMMON_H
#define COMMON_H

#include "MKL05Z4.h"

#ifndef _BV
#define _BV(n) (1u << (n))
#endif

#define DELAY1() do {__NOP(); __NOP(); __NOP();} while(0)
void delay_us(int n);

#endif
