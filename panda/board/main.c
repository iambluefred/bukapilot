#include "common.h"
#include "drivers/usb.h"

// Todo: Initialise the most basic thing
// 1. First milestone is a blinky code

int main(void) {
  init_interrupts(true);
  disable_interrupts();

  // configure the clock, enable memory protection unit, initilize stack
  // set up interrupts and configure peripherals
  clock_init();
  peripherals_init();
  common_init_gpio();

  // enable USB (right before interrupts or enum can fail!)
  usb_init();
  enable_interrupts();

  set_gpio_output(GPIOC, 2, true);
  set_gpio_output(GPIOC, 3, false);
  set_gpio_output(GPIOC, 14, true);

  return 0;
}
