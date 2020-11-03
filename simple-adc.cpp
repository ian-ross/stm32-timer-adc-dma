#include <cstdio>
#include <cstdint>
#include "bsp-nucleo.h"

#include "mcu.hpp"
#include "gpio.hpp"
#include "usart.hpp"
#include "adc.hpp"

int main(void)
{
  common_init();
  usart_print("START\r\n");

  configure_polled_adc();

  char buff[64];
  uint32_t blink_start = systick_count;
  uint32_t adc_blink_start;
  bool adc_blink_on = false;
  while (1) {
    if (systick_count - blink_start >= 100) {
      LED1_PORT->ODR ^= 1 << LED1_PIN;
      blink_start = systick_count;
    }

    if (adc_blink_on && systick_count - adc_blink_start > 250) {
      CLEAR_BIT(LED2_PORT->ODR, 1 << LED2_PIN);
      adc_blink_on = false;
    }

    if (button_pressed) {
      button_pressed = false;
      usart_print("BUTTON\r\n");
      SET_BIT(LED2_PORT->ODR, 1 << LED2_PIN);
      adc_blink_on = true;
      adc_blink_start = systick_count;

      SET_BIT(ADC1->CR2, ADC_CR2_ADON);
      uint16_t sample = polled_adc();
      CLEAR_BIT(ADC1->CR2, ADC_CR2_ADON);

      sprintf(buff, "ADC: %d\r\n", sample);
      usart_print(buff);
    }
  }
}
