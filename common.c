#include "common.h"

static void enable_caches(void);
static void configure_clock(void);
static void configure_gpios(bool use_button);
static void configure_gpio_output(GPIO_TypeDef *port, uint16_t pin);
static void configure_gpio_input(GPIO_TypeDef *port, uint16_t pin);
static void configure_gpio_alternate(GPIO_TypeDef *port, uint16_t pin, uint8_t af);
static void configure_gpio_analog(GPIO_TypeDef *port, uint16_t pin);
static void configure_stlink_usart(void);


//------------------------------------------------------------------------------
//
//  Common initialisation
//

void common_init(bool use_button) {
  enable_caches();
  configure_clock();
  SysTick_Config(SystemCoreClock / 1000); // Set SysTick frequency to 1 kHz.
  configure_gpios(use_button);
  configure_stlink_usart();
}


//------------------------------------------------------------------------------
//
//  MCU configuration: caches & clocks
//

// Enable L1 instruction and data caches.

static void enable_caches(void) {
  SCB_EnableICache();
  SCB_EnableDCache();
}


// Initialise clock to 216 MHz using Nucleo 8 MHz HSE bypass input.

static void configure_clock(void) {
  // Enable HSE with bypass input (8 MHz input clock from Nucleo).
  SET_BIT(RCC->CR, RCC_CR_HSEBYP);
  SET_BIT(RCC->CR, RCC_CR_HSEON);
  while (!READ_BIT(RCC->CR, RCC_CR_HSERDY)) { __asm("nop"); }

  // Disable PLL (can't do this if the system clock is already being
  // driven by the PLL, so don't come back in here afterwards!).
  CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
  while (READ_BIT(RCC->CR, RCC_CR_PLLRDY)) { __asm("nop"); }

  // Set up main PLL: 8 MHz (HSE) / 8 (M) * 432 (N) / 2 (P) = 216 MHz.
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE, RCC_PLLCFGR_PLLSRC_HSE);
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, 8 << RCC_PLLCFGR_PLLM_Pos);
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, 432 << RCC_PLLCFGR_PLLN_Pos);
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLP, (2/2-1) << RCC_PLLCFGR_PLLP_Pos);

  // Set default values for other PLLs (Q = 9 gives 48 MHz for the USB
  // clock, which is required for that to work).
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ, 9 << RCC_PLLCFGR_PLLQ_Pos);
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLR, 7 << RCC_PLLCFGR_PLLR_Pos);

  // Enable PLL.
  SET_BIT(RCC->CR, RCC_CR_PLLON);
  while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY)) { __asm("nop"); }

  // Activate power over-drive to reach 216 MHz.
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
  // Delay for clock enable...
  (void)READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
  SET_BIT(PWR->CR1, PWR_CR1_ODEN);
  while (!READ_BIT(PWR->CSR1, PWR_CSR1_ODRDY)) { __asm("nop"); }
  SET_BIT(PWR->CR1, PWR_CR1_ODSWEN);
  while (!READ_BIT(PWR->CSR1, PWR_CSR1_ODSWRDY)) { __asm("nop"); }

  // Running the system clock at 216 MHz means we need 7 wait states
  // for the flash (see Table 7 in the STM32F76xxx reference manual).
  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, 7 << FLASH_ACR_LATENCY_Pos);

  // Select PLL as system clock source and configure HCLK, PCLK1 and
  // PCLK2 clock dividers. Set APB dividers to safe values, then
  // update system clock divider and source before setting APB
  // dividers to final values.
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV16);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV16);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1); // 216 MHz
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { __asm("nop"); }
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV8); // 27 MHz
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV4); // 54 MHz

  // Update CMSIS system core clock.
  SystemCoreClockUpdate();
}


// SysTick is set to run at 1 kHz.

volatile uint32_t systick_count = 0;

void SysTick_Handler(void) {
  ++systick_count;
}


//------------------------------------------------------------------------------
//
//  GPIO configuration: indicator LEDs
//

// Set up LED and button GPIOs.

void configure_gpios(bool use_button) {
  // Initialise GPIOs for LEDs as outputs.
  configure_gpio_output(LED1_PORT, LED1_PIN);
  configure_gpio_output(LED2_PORT, LED2_PIN);
  configure_gpio_output(LED3_PORT, LED3_PIN);

  // Initialise button GPIO (PC13) as input and configure interrupt.
  if (use_button) {
    configure_gpio_input(USER_BUTTON_PORT, USER_BUTTON_PIN);
    __IO uint32_t tmpreg;
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
    tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
    (void)tmpreg;
    uint32_t syscfg_exti_line13 = 0x00F0U << 16U | 3U;
    MODIFY_REG(SYSCFG->EXTICR[syscfg_exti_line13 & 0xFFU],
               (syscfg_exti_line13 >> 16U),
               2U << POSITION_VAL((syscfg_exti_line13 >> 16U)));
    SET_BIT(EXTI->IMR, 1 << USER_BUTTON_EXTI_LINE);
    SET_BIT(EXTI->FTSR, 1 << USER_BUTTON_EXTI_LINE);
    NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);
    NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, 0x03);
  }
}

// GPIO configuration constants.
#define GPIO_SPEED_VERY_HIGH 3
#define GPIO_TYPE_PUSH_PULL 0
#define GPIO_PUPD_NONE 0

static void enable_gpio_clock(GPIO_TypeDef *port) {
  // Enable GPIO clock: AHB1ENR bits are one per port, starting from
  // zero, and the port addresses are every 0x0400 starting at the
  // base address.
  uint32_t mask =
    1 << (((uintptr_t)port - (uintptr_t)AHB1PERIPH_BASE) / 0x0400UL);
  RCC->AHB1ENR |= mask;
}

static void configure_gpio_output(GPIO_TypeDef *port, uint16_t pin) {
  enable_gpio_clock(port);
  MODIFY_REG(port->OSPEEDR, 0x03 << (2 * pin), GPIO_SPEED_VERY_HIGH << (2 * pin));
  MODIFY_REG(port->OTYPER, 0x01 << pin, GPIO_TYPE_PUSH_PULL << pin);
  MODIFY_REG(port->PUPDR, 0x03 << (2 * pin), GPIO_PUPD_NONE << (2 * pin));
  MODIFY_REG(port->MODER, 0x03 << (2 * pin), 0x01 << (2 * pin));
}

static void configure_gpio_input(GPIO_TypeDef *port, uint16_t pin) {
  enable_gpio_clock(port);
  MODIFY_REG(port->OSPEEDR, 0x03 << (2 * pin), GPIO_SPEED_VERY_HIGH << (2 * pin));
  MODIFY_REG(port->MODER, 0x03 << (2 * pin), 0x00 << (2 * pin));
}

static void configure_gpio_alternate(GPIO_TypeDef *port, uint16_t pin, uint8_t af) {
  enable_gpio_clock(port);
  MODIFY_REG(port->MODER, 0x03 << (2 * pin), 0x02 << (2 * pin));
  MODIFY_REG(port->AFR[pin / 8], 0x0F << 4 * (pin % 8), af << 4 * (pin % 8));
}

static void configure_gpio_analog(GPIO_TypeDef *port, uint16_t pin) {
  enable_gpio_clock(port);
  MODIFY_REG(port->MODER, 0x03 << (pin * 2), 0x03 << (pin * 2));
}


volatile bool button_pressed = false;

void USER_BUTTON_IRQHANDLER(void)
{
  if (READ_BIT(EXTI->PR, 1 << USER_BUTTON_EXTI_LINE)) {
    button_pressed = true;
    WRITE_REG(EXTI->PR, 1 << USER_BUTTON_EXTI_LINE);
  }
}


//------------------------------------------------------------------------------
//
//  ST-Link debug output USART
//

static void configure_stlink_usart(void) {
  // Disable USART3.
  CLEAR_BIT(USART3->CR1, USART_CR1_UE);

  // Enable USART3 APB clock.
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);
  (void)READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);

  // Select system clock (216 MHz) as clock source for USART3.
  const uint32_t dckcfgr2_pos = 2 * (3 - 1);
  const uint32_t dckcfgr2_mask = 0x03 << dckcfgr2_pos;
  MODIFY_REG(RCC->DCKCFGR2, dckcfgr2_mask, 0x01 << dckcfgr2_pos);

  // Set word length (M1 = 0, M0 = 0 => 1 start bit, 8 data bits, n
  // stop bits).
  CLEAR_BIT(USART3->CR1, USART_CR1_M0);
  CLEAR_BIT(USART3->CR1, USART_CR1_M1);

  // Disable parity.
  CLEAR_BIT(USART3->CR1, USART_CR1_PCE);

  // Disable auto baudrate detection.
  CLEAR_BIT(USART3->CR2, USART_CR2_ABREN);

  // Send/receive LSB first.
  CLEAR_BIT(USART3->CR2, USART_CR2_MSBFIRST);

  // Set oversampling rate to 16 and select baud rate 115200 based on
  // USART clock running at 216 MHz (value taken from Table 220 in
  // STM32F767ZI reference manual). (216000000 / 115200 = 0x0753)
  CLEAR_BIT(USART3->CR1, USART_CR1_OVER8);
  USART3->BRR = 0x0753;

  // One stop bit (USART.CR2.STOP[1:0] = 0 => 1 stop bit).
  MODIFY_REG(USART3->CR2, USART_CR2_STOP_Msk, 0);

  // Enable USART3 transmitter.
  SET_BIT(USART3->CR1, USART_CR1_UE);
  SET_BIT(USART3->CR1, USART_CR1_RE);
  SET_BIT(USART3->CR1, USART_CR1_TE);

  // Pin configuration: set TX to output (PP, pull-up), RX to input,
  // set both to the appropriate alternate function.
  configure_gpio_output(GPIOD, 8);
  configure_gpio_alternate(GPIOD, 8, 7);
  configure_gpio_input(GPIOD, 9);
  configure_gpio_alternate(GPIOD, 9, 7);
}


// Print a string via USART3.

void usart_print(const char *s) {
  for (const char *pt = s; *pt; ++pt) {
    usart_tx(*pt);
  }
  usart_wait();
}


// Send a single character to USART3.

void usart_tx(char c) {
  while (!(USART3->ISR & USART_ISR_TXE)) { __asm("nop"); }
  USART3->TDR = c;
}


// Wait for USART3 idle.

void usart_wait(void) {
  while (!(USART3->ISR & USART_ISR_TC)) { __asm("nop"); }
}


//------------------------------------------------------------------------------
//
//  Common ADC configuration
//

void configure_common_adc(int nchannels, bool interrupts)
{
  // Configure GPIOs: ADC1 channels 4-7 are assigned to PA4, PA5, PA6,
  // PA7 on the STM32F767ZI
  configure_gpio_analog(GPIOA, 4);
  if (nchannels > 1) configure_gpio_analog(GPIOA, 5);
  if (nchannels > 2) configure_gpio_analog(GPIOA, 6);
  if (nchannels > 3) configure_gpio_analog(GPIOA, 7);

  // Enable ADC interrupts.
  if (interrupts) {
    NVIC_SetPriority(ADC_IRQn, 0);
    NVIC_EnableIRQ(ADC_IRQn);
  }

  // Start ADC peripheral clock.
  __IO uint32_t tmpreg;
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
  tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
  (void)tmpreg;

  // Set ADC clock source: APB2 clock / 2.
  MODIFY_REG(ADC123_COMMON->CCR, ADC_CCR_ADCPRE, 0);

  // Set ADC group regular continuous mode: off.
  MODIFY_REG(ADC1->CR2, ADC_CR2_CONT, 0);
}
