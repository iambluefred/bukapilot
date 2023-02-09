#pragma once

#include <stdbool.h>

// platform includes
#ifdef STM32F4
  #include "stm32fx/inc/stm32f4xx.h"
#else
  // TODO: uncomment this, cppcheck complains
  // building for tests
  //#include "fake_stm.h"
#endif

#include "libc.h"
#include "critical.h"
#include "faults.h"

#include "drivers/registers.h"
#include "drivers/gpio.h"
#include "stm32fx/clock.h"
#include "stm32fx/peripherals.h"
