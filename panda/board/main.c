#include "common.h"

// Todo: Initialise the most basic thing
// 1. First milestone is a blinky code

int main(void) {

  // configure the clock, enable memory protection unit, initilize stack
  // set up interrupts and configure peripherals
  clock_init();
  peripherals_init();
  common_init_gpio();

  GPIOC->BSRR = GPIO_BSRR_BS_2;
  GPIOC->BSRR = GPIO_BSRR_BS_14;
  GPIOC->BSRR = GPIO_BSRR_BS_3;

  return 0;
}
