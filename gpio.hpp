#pragma once
#include <cstdbool>
#include <cstdint>
#include "bsp-nucleo.h"
extern volatile bool button_pressed;
void configure_gpios(void);
void configure_gpio_output(GPIO_TypeDef *port, uint16_t pin);
void configure_gpio_input(GPIO_TypeDef *port, uint16_t pin);
void configure_gpio_alternate(GPIO_TypeDef *port, uint16_t pin, uint8_t af);
void configure_gpio_analog(GPIO_TypeDef *port, uint16_t pin);
