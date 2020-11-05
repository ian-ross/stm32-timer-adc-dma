#include "common.h"


static void configure_dma(void);
static void configure_dma_adc(void);
static void start_dma_adc(void);

static volatile bool dma_complete = false;
static volatile bool dma_error = false;
static volatile bool adc_error = false;

#define NCHANNELS 4
static volatile uint16_t dma_adc_sample[NCHANNELS];

int main(void)
{
  common_init(true);
  usart_print("START\r\n");

  configure_dma_adc();

  char buff[64];
  uint32_t blink_start = systick_count;
  uint32_t adc_blink_start;
  uint32_t adc_start;
  bool adc_blink_on = false;
  bool adc_running = false;
  while (1) {
    if (systick_count - blink_start >= 100) {
      LED1_PORT->ODR ^= 1 << LED1_PIN;
      blink_start = systick_count;
    }

    if (adc_blink_on && systick_count - adc_blink_start > 250) {
      CLEAR_BIT(LED2_PORT->ODR, 1 << LED2_PIN);
      adc_blink_on = false;
    }

    if (adc_running && systick_count - adc_start > 500) {
      adc_running = false;
      usart_print("ADC TIMEOUT\r\n");
    }

    if (button_pressed) {
      button_pressed = false;
      SET_BIT(LED2_PORT->ODR, 1 << LED2_PIN);
      adc_blink_on = true;
      adc_blink_start = systick_count;
      adc_running = true;
      adc_start = systick_count;

      SET_BIT(ADC1->CR2, ADC_CR2_ADON);
      start_dma_adc();
    }

    if (adc_error) {
      adc_error = false;
      adc_running = false;
      usart_print("ADC ERROR!\r\n");
    }

    if (dma_complete) {
      dma_complete = false;
      adc_running = false;
      sprintf(buff, "%d %d %d %d\r\n",
              dma_adc_sample[0], dma_adc_sample[1], dma_adc_sample[2], dma_adc_sample[3]);
      usart_print(buff);
      CLEAR_BIT(ADC1->CR2, ADC_CR2_ADON);
    }

    if (dma_error) {
      dma_error = false;
      adc_running = false;
      usart_print("DMA ERROR!\r\n");
    }
  }
}

static void configure_dma_adc(void) {
  // XXX: TRY true HERE...
  configure_common_adc(4, true);

  configure_dma();

  // Set ADC group regular trigger source: software trigger.
  MODIFY_REG(ADC1->CR2, ADC_CR2_EXTSEL, 0);

  // Set ADC group regular sequencer length: four channels.
  MODIFY_REG(ADC1->SQR1, ADC_SQR1_L, (4 - 1) << ADC_SQR1_L_Pos);

  // Set ADC group regular sequence:
  // channel 4 @ rank 0, 5 @ rank 1, 6 @ rank 2, 7 @ rank 3.
  MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ1, ADC_SQR3_SQ1_Pos);
  MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ2, ADC_SQR3_SQ2_Pos);
  MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ3, ADC_SQR3_SQ3_Pos);
  MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ4, ADC_SQR3_SQ4_Pos);

  // Set ADC channels 4-7 sample time: 56 cycles.
  MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP4, 0x03 << ADC_SMPR2_SMP4_Pos);
  MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP5, 0x03 << ADC_SMPR2_SMP5_Pos);
  MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP6, 0x03 << ADC_SMPR2_SMP6_Pos);
  MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP7, 0x03 << ADC_SMPR2_SMP7_Pos);

  // Enable multiple conversions.
  SET_BIT(ADC1->CR1, ADC_CR1_SCAN);

  // ADC interrupts.
  SET_BIT(ADC1->CR1, ADC_CR1_OVRIE);
  SET_BIT(ADC1->CR1, ADC_CR1_EOCIE);

  // Enable DMA transfer for ADC.
  MODIFY_REG(ADC1->CR2, ADC_CR2_DMA | ADC_CR2_DDS, ADC_CR2_DDS | ADC_CR2_DMA);
}

static void configure_dma(void) {
  NVIC_SetPriority(DMA2_Stream0_IRQn, 1); // DMA IRQ lower priority than ADC IRQ.
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  // Enable DMA peripheral clock.
  __IO uint32_t tmpreg;
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
  tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
  (void)tmpreg;

  // Configure DMA transfer:
  //  - DMA transfer in circular mode to match with ADC configuration:
  //    DMA unlimited requests.
  //  - DMA transfer from ADC without address increment.
  //  - DMA transfer to memory with address increment.
  //  - DMA transfer from ADC by half-word to match with ADC configuration:
  //    ADC resolution 12 bits.
  //  - DMA transfer to memory by half-word to match with ADC conversion data
  //    buffer variable type: half-word.
  MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_CHSEL, 0x00);
  MODIFY_REG(DMA2_Stream0->CR,
             DMA_SxCR_DIR | DMA_SxCR_CIRC | DMA_SxCR_PINC | DMA_SxCR_MINC |
             DMA_SxCR_PSIZE | DMA_SxCR_MSIZE | DMA_SxCR_PL | DMA_SxCR_PFCTRL,
             0x00000000U |      // Direction: peripheral to memory
             DMA_SxCR_CIRC |    // Mode: circular
             0x00000000U |      // Peripheral: no increment
             DMA_SxCR_MINC |    // Memory: increment
             DMA_SxCR_PSIZE_0 | // Peripheral data align: halfword
             DMA_SxCR_MSIZE_0 | // Memory data align: halfword
             DMA_SxCR_PL_1);    // Priority: high

  // Set DMA transfer addresses.
  WRITE_REG(DMA2_Stream0->PAR, (uint32_t)&(ADC1->DR));
  WRITE_REG(DMA2_Stream0->M0AR, (uint32_t)&dma_adc_sample);

  // Set DMA transfer size.
  MODIFY_REG(DMA2_Stream0->NDTR, DMA_SxNDT, NCHANNELS);

  // Enable DMA interrupts.
  SET_BIT(DMA2_Stream0->CR, DMA_SxCR_TCIE);
  SET_BIT(DMA2_Stream0->CR, DMA_SxCR_TEIE);

  // Enable DMA transfer.
  SET_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);
}

static void start_dma_adc(void) {
  // Manually start ADC conversion.
  SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);
}


void ADC_IRQHandler(void) {
  if (READ_BIT(ADC1->SR, ADC_SR_OVR)) {
    WRITE_REG(ADC1->SR, ~ADC_SR_OVR);
    adc_error = true;
  }
}

void DMA2_Stream0_IRQHandler(void) {
  // DMA transfer complete.
  if (READ_BIT(DMA2->LISR, DMA_LISR_TCIF0)) {
    WRITE_REG(DMA2->LIFCR , DMA_LIFCR_CTCIF0);
    dma_complete = true;
  }

  // DMA transfer error.
  if (READ_BIT(DMA2->LISR ,DMA_LISR_TEIF0)) {
    WRITE_REG(DMA2->LIFCR , DMA_LIFCR_CTEIF0);
    dma_error = true;
  }
}
