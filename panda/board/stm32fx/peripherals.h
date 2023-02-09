#define GREEN_LED_Pin          2
#define BLUE_LED_Pin           3
#define RED_LED_Pin            14

#define LED_GPIO_Port          GPIOC

// Common GPIO initialization
void common_init_gpio(void) {

  // LEDs
  // Todo: Bitwise or them
  set_gpio_mode(LED_GPIO_Port, RED_LED_Pin, MODE_OUTPUT);
  set_gpio_mode(LED_GPIO_Port, GREEN_LED_Pin, MODE_OUTPUT);
  set_gpio_mode(LED_GPIO_Port, BLUE_LED_Pin, MODE_OUTPUT);
  set_gpio_output_type(LED_GPIO_Port, RED_LED_Pin, OUTPUT_TYPE_OPEN_DRAIN);
  set_gpio_output_type(LED_GPIO_Port, GREEN_LED_Pin, OUTPUT_TYPE_OPEN_DRAIN);
  set_gpio_output_type(LED_GPIO_Port, BLUE_LED_Pin, OUTPUT_TYPE_OPEN_DRAIN);

}

// Peripheral initialization
void peripherals_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
}
