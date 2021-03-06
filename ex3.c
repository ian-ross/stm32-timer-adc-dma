#include "common.h"


static void configure_dma(void);
static void configure_dma_adc(void);
static void start_dma_adc(void);

static volatile bool dma_complete = false;
static volatile bool dma_error = false;

#define NCHANNELS 4
static volatile uint16_t dma_adc_sample[NCHANNELS];

int main(void)
{
  // Setup: caches, clocks, LED and button GPIOs, ST-Link USART.
  common_init(true);

  usart_print("START\r\n");

  // Set up for DMA ADC of four input channels.
  configure_dma_adc();

  char buff[64];
  uint32_t blink_start = systick_count;
  uint32_t adc_blink_start;
  uint32_t adc_start;
  bool adc_blink_on = false;
  bool adc_running = false;
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

    // ADC timeout error.
    if (adc_running && systick_count - adc_start > 500) {
      adc_running = false;
      usart_print("ADC TIMEOUT\r\n");
    }

    // Handle button press: start "ADC in progress" blinky, start DMA
    // ADC.
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

    // DMA complete interrupt received.
    if (dma_complete) {
      dma_complete = false;
      adc_running = false;
      sprintf(buff, "%d %d %d %d\r\n",
              dma_adc_sample[0], dma_adc_sample[1],
              dma_adc_sample[2], dma_adc_sample[3]);
      usart_print(buff);
      CLEAR_BIT(ADC1->CR2, ADC_CR2_ADON);
    }

    // DMA error interrupt received.
    if (dma_error) {
      dma_error = false;
      adc_running = false;
      usart_print("DMA ERROR!\r\n");
    }
  }
}

static void configure_dma_adc(void) {
  configure_common_adc(4, false);

  configure_dma();

  // Set ADC group regular trigger source: software trigger.
  MODIFY_REG(ADC1->CR2, ADC_CR2_EXTEN, 0);

  // Set ADC group regular sequencer length: four channels.
  MODIFY_REG(ADC1->SQR1, ADC_SQR1_L, (4 - 1) << ADC_SQR1_L_Pos);

  // Set ADC group regular sequence:
  // channel 4 @ rank 1, 5 @ rank 2, 6 @ rank 3, 7 @ rank 4.
  MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ1, 0x04 << ADC_SQR3_SQ1_Pos);
  MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ2, 0x05 << ADC_SQR3_SQ2_Pos);
  MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ3, 0x06 << ADC_SQR3_SQ3_Pos);
  MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ4, 0x07 << ADC_SQR3_SQ4_Pos);

  // Set ADC channels 4-7 sample time: 56 cycles.
  MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP4, 0x03 << ADC_SMPR2_SMP4_Pos);
  MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP5, 0x03 << ADC_SMPR2_SMP5_Pos);
  MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP6, 0x03 << ADC_SMPR2_SMP6_Pos);
  MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP7, 0x03 << ADC_SMPR2_SMP7_Pos);

  // Enable multiple conversions.
  SET_BIT(ADC1->CR1, ADC_CR1_SCAN);
}

static void configure_dma(void) {
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
  WRITE_REG(DMA2_Stream0->M0AR, (uint32_t)&dma_adc_sample[0]);

  // Set DMA transfer size.
  MODIFY_REG(DMA2_Stream0->NDTR, DMA_SxNDT, NCHANNELS);

  // Enable DMA interrupts.
  NVIC_SetPriority(DMA2_Stream0_IRQn, 1); // DMA IRQ lower priority than ADC IRQ.
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  SET_BIT(DMA2_Stream0->CR, DMA_SxCR_TCIE);
  SET_BIT(DMA2_Stream0->CR, DMA_SxCR_TEIE);

  // Enable DMA transfer.
  SET_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);
}

static void start_dma_adc(void) {
  // Reset ADC DMA status (cf. RM Section 15.8.1).
  CLEAR_BIT(ADC1->CR2, ADC_CR2_DMA);
  SET_BIT(ADC1->CR2, ADC_CR2_DMA);

  // Manually start ADC conversion.
  SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);
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
