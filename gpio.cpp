//------------------------------------------------------------------------------
//
//  GPIO configuration: indicator LEDs
//

#include <cstdint>
#include "bsp-nucleo.h"
#include "gpio.hpp"

volatile bool button_pressed = false;

extern "C" void USER_BUTTON_IRQHANDLER(void)
{
  if (READ_BIT(EXTI->PR, 1 << USER_BUTTON_EXTI_LINE)) {
    button_pressed = true;
    WRITE_REG(EXTI->PR, 1 << USER_BUTTON_EXTI_LINE);
  }
}

void configure_gpios(void) {
  // Initialise GPIOs for LEDs as outputs.
  configure_gpio_output(LED1_PORT, LED1_PIN);
  configure_gpio_output(LED2_PORT, LED2_PIN);
  configure_gpio_output(LED3_PORT, LED3_PIN);

  // Initialise button GPIO (PC13) as input and configure interrupt.
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

// GPIO configuration constants.
#define GPIO_SPEED_VERY_HIGH 3
#define GPIO_TYPE_PUSH_PULL 0
#define GPIO_PUPD_NONE 0

static void enable_clock(GPIO_TypeDef *port) {
  // Enable GPIO clock: AHB1ENR bits are one per port, starting from
  // zero, and the port addresses are every 0x0400 starting at the
  // base address.
  uint32_t mask =
    1 << (((uintptr_t)port - (uintptr_t)AHB1PERIPH_BASE) / 0x0400UL);
  RCC->AHB1ENR |= mask;
}

void configure_gpio_output(GPIO_TypeDef *port, uint16_t pin) {
  enable_clock(port);
  MODIFY_REG(port->OSPEEDR, 0x03 << (2 * pin), GPIO_SPEED_VERY_HIGH << (2 * pin));
  MODIFY_REG(port->OTYPER, 0x01 << pin, GPIO_TYPE_PUSH_PULL << pin);
  MODIFY_REG(port->PUPDR, 0x03 << (2 * pin), GPIO_PUPD_NONE << (2 * pin));
  MODIFY_REG(port->MODER, 0x03 << (2 * pin), 0x01 << (2 * pin));
}

void configure_gpio_input(GPIO_TypeDef *port, uint16_t pin) {
  enable_clock(port);
  MODIFY_REG(port->OSPEEDR, 0x03 << (2 * pin), GPIO_SPEED_VERY_HIGH << (2 * pin));
  MODIFY_REG(port->MODER, 0x03 << (2 * pin), 0x00 << (2 * pin));
}

void configure_gpio_alternate(GPIO_TypeDef *port, uint16_t pin, uint8_t af) {
  enable_clock(port);
  MODIFY_REG(port->MODER, 0x03 << (2 * pin), 0x02 << (2 * pin));
  MODIFY_REG(port->AFR[pin / 8], 0x0F << 4 * (pin % 8), af << 4 * (pin % 8));
}

void configure_gpio_analog(GPIO_TypeDef *port, uint16_t pin) {
  enable_clock(port);
  MODIFY_REG(port->MODER, 0x03 << (pin * 2), 0x03 << (pin * 2));
}
