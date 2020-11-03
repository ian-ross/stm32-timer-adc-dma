//------------------------------------------------------------------------------
//
//  ST-Link debug output USART
//

#include <cstdint>
#include "bsp-nucleo.h"
#include "gpio.hpp"
#include "mcu.hpp"
#include "usart.hpp"

static void usart_tx(char c);


void configure_stlink_usart(void) {
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
  for (const char *pt = s; *pt; ++pt) usart_tx(*pt);
  while (!(USART3->ISR & USART_ISR_TC)) { __asm("nop"); }
}


// Send a single character.

static void usart_tx(char c) {
  while (!(USART3->ISR & USART_ISR_TXE)) { __asm("nop"); }
  USART3->TDR = c;
}
