void clock_init(void) {

  // set up flash latency, it is the delay time for flash accesses,
  // flash access time depends on the supply voltage and the system clock frequency.
  // below we are setting it to 5 wait states. Too high it will be inefficient, to low
  // it may cause corruption.
  register_set(&(FLASH->ACR), FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS, 0x1F0FU);

  // enable HSE clock/ external oscillator
  register_set_bits(&(RCC->CR), RCC_CR_HSEON);
  while ((RCC->CR & RCC_CR_HSERDY) == 0);

  // select, configure and enable PLL

  // 16mhz external oscillator is used to drive the SYSCLK using PLL
  // that means HSE as PLL Source, /8 (M), x168 (N), /2 (P), /7 (Q)
  // AHB Prescaler /1. APB1 Prescaler /4 and APB2 Prescaler /2. The values
  // are obtained from STM32CubeIDE clock config UI
  // docs: RM0090 PG163
  register_set(&(RCC->CFGR), RCC_CFGR_SW_PLL | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4, 0xFF7FFCF3U);
  register_set(&(RCC->PLLCFGR), RCC_PLLCFGR_PLLSRC_HSE | (168 << RCC_PLLCFGR_PLLN_Pos) | (8 << RCC_PLLCFGR_PLLM_Pos) | (2 << RCC_PLLCFGR_PLLP_Pos) | (7 << RCC_PLLCFGR_PLLQ_Pos), 0x7F437FFFU);

  // start PLL
  register_set_bits(&(RCC->CR), RCC_CR_PLLON);
  while ((RCC->CR & RCC_CR_PLLRDY) == 0);

  // switch to PLL
  register_set_bits(&(RCC->CFGR), RCC_CFGR_SW_PLL);
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

}

// setup the independent watchdog (IWDG)
void watchdog_init(void) {
  // enable IWDG
  IWDG->KR = 0x5555U;

  // set the prescaler to device the 32kHz LSI clock by 4
  // docs: RM0090 PG711
  register_set(&(IWDG->PR), 0x0U, 0x7U);

  // clock divide by 4 = 0.125 ms, let's have a 50ms watchdog
  // setting the reload counter 50/0.125 = 400
  register_set(&(IWDG->RLR), (400U-1U), 0xFFFU);

  // start the watchdog timer
  IWDG->KR = 0xCCCCU;
}

void watchdog_feed(void) {
  IWDG->KR = 0xAAAAU;
}
