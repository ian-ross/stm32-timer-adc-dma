#include "bsp-nucleo.h"

#include "gpio.hpp"
#include "adc.hpp"

static volatile bool adc_error = false;
static volatile bool adc_eoc = false;

// extern "C" void ADC_IRQHandler(void)
// {
//   // if (READ_BIT(ADC1->SR, ADC_SR_OVR)) {
//   //   WRITE_REG(ADC1->SR, ~ADC_SR_OVR);
//   //   CLEAR_BIT(ADC1->CR1, ADC_CR1_OVRIE);
//   //   adc_error = true;
//   // }
//   if (READ_BIT(ADC1->SR, ADC_SR_EOC)) {
//     adc_eoc = true;
//   }
// }


uint16_t polled_adc(void) {
  SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);
  while (!READ_BIT(ADC1->SR, ADC_SR_EOC)) { __asm("nop"); }
  return (uint16_t)(READ_BIT(ADC1->DR, ADC_DR_DATA));
}

void configure_polled_adc(void)
{
  configure_gpio_analog(GPIOA, 4);
  NVIC_SetPriority(ADC_IRQn, 0);
  NVIC_EnableIRQ(ADC_IRQn);

  __IO uint32_t tmpreg;
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
  tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
  (void)tmpreg;

  // Set ADC clock (conversion clock).
  MODIFY_REG(ADC123_COMMON->CCR, ADC_CCR_ADCPRE, 0); // ADCLK = PCLK2/2

  // Set ADC group regular trigger source.
  MODIFY_REG(ADC1->CR2, ADC_CR2_EXTSEL, 0);

  // Set ADC group regular continuous mode.
  MODIFY_REG(ADC1->CR2, ADC_CR2_CONT, 0);

  // Set ADC group regular sequencer length and scan direction.
  MODIFY_REG(ADC1->SQR1, ADC_SQR1_L, 0);

  // Set ADC group regular sequence: channel on the selected sequence rank.
  MODIFY_REG(ADC1->SQR3, 0x0000001FU, 0x04);

  // Set ADC channels sampling time.
  MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP4, 0x03 << ADC_SMPR2_SMP4_Pos);

  SET_BIT(ADC1->CR1, ADC_CR1_OVRIE);
  // SET_BIT(ADC1->CR1, ADC_CR1_EOCIE);
}
