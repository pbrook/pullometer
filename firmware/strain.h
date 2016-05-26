#ifndef STRAIN_H
#define STRAIN_H

#include "common.h"
#include <limits.h>

#define STRAIN_BUSY LONG_MAX

void init_strain();
long poll_strain();

#endif
