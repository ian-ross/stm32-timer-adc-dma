#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "bsp-nucleo.h"

// Configuration shared among all examples.
void common_init(bool use_button);

// 1 kHz SysTick count.
extern volatile uint32_t systick_count;

// User button pressed?
extern volatile bool button_pressed;

// Print to ST-Link USART.
void usart_print(const char *s);
void usart_tx(char c);
void usart_wait(void);

// Common ADC configuration.
void configure_common_adc(int nchannels, bool interrupts);
