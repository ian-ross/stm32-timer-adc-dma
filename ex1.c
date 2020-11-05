#include "common.h"

static void configure_polled_adc(void);
static uint16_t polled_adc(void);


int main(void)
{
  // Setup: caches, clocks, LED and button GPIOs, ST-Link USART.
  common_init(true);

  usart_print("START\r\n");

  // Set up for polled ADC of single input channel.
  configure_polled_adc();

  char buff[64];
  uint32_t blink_start = systick_count;
  uint32_t adc_blink_start;
  bool adc_blink_on = false;
  while (1) {
    // "I'm alive" blinky.
    if (systick_count - blink_start >= 100) {
      LED1_PORT->ODR ^= 1 << LED1_PIN;
      blink_start = systick_count;
    }

    // "ADC in progress" blinky.
    if (adc_blink_on && systick_count - adc_blink_start > 250) {
      CLEAR_BIT(LED2_PORT->ODR, 1 << LED2_PIN);
      adc_blink_on = false;
    }

    // Handle button press: start "ADC in progress" blinky, do polled
    // ADC and print result.
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


static void configure_polled_adc(void) {
  configure_common_adc(1, false);

  // Set ADC group regular trigger source: software trigger.
  MODIFY_REG(ADC1->CR2, ADC_CR2_EXTSEL, 0);

  // Set ADC group regular sequencer length: one channel.
  MODIFY_REG(ADC1->SQR1, ADC_SQR1_L, 0);

  // Set ADC group regular sequence: channel 4 @ rank 0.
  MODIFY_REG(ADC1->SQR3, 0x0000001FU, 0x04);

  // Set ADC channel 4 sample time: 56 cycles.
  MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP4, 0x03 << ADC_SMPR2_SMP4_Pos);
}


static uint16_t polled_adc(void) {
  // Manually start ADC conversion.
  SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);

  // Wait for conversion complete.
  while (!READ_BIT(ADC1->SR, ADC_SR_EOC)) { __asm("nop"); }

  // Read converted data (clears EOC flag).
  return (uint16_t)(READ_BIT(ADC1->DR, ADC_DR_DATA));
}
