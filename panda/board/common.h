#pragma once

#include <stdbool.h>

// USB definitions
#define USB_VID 0xBBAAU
#define USB_PID 0xDDCCU

#define USBPACKET_MAX_SIZE 0x40U

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
#include "drivers/interrupts.h"
#include "drivers/gpio.h"
#include "stm32fx/clock.h"
#include "stm32fx/peripherals.h"
#include "stm32fx/llusb.h"

