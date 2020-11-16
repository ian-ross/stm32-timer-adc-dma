#include "common.h"


static void configure_timer(void);
static void configure_dma(void);
static void configure_timer_dma_adc(void);

static volatile bool dma_complete = false;
static volatile bool dma_error = false;
static volatile bool timer_fired = true;

// ADC trigger timer parameters.

// Timer frequency (Hz). With a 16-bit timer and time base freq min
// 1Hz, range is min=1Hz, max=32kHz.
#define TIMER_FREQUENCY ((uint32_t)500)

// Timer minimum frequency (Hz), used to calculate frequency range.
// With a timer 16 bits, maximum frequency will be 32000 times this
// value.
#define TIMER_FREQUENCY_RANGE_MIN ((uint32_t)1)

// Timer prescaler maximum value (0xFFFF for a 16-bit timer).
#define TIMER_PRESCALER_MAX_VALUE ((uint32_t)0xFFFF-1)


#define NCHANNELS 4
static volatile uint16_t dma_adc_sample[NCHANNELS];

int main(void)
{
  // Setup: caches, clocks, LED and button GPIOs, ST-Link USART.
  common_init(false);

  // Set up for timer-driven DMA ADC of four input channels.
  configure_timer_dma_adc();
  SET_BIT(ADC1->CR2, ADC_CR2_ADON);

  uint32_t blink_start = systick_count;
  uint32_t adc_blink_start;
  uint32_t adc_start;
  bool adc_blink_on = false;
  bool adc_running = false;
  for (int i = 0; i < 8; ++i)
    usart_tx(0xAA);
  usart_wait();
  while (1) {
    // "I'm alive" blinky.
    if (systick_count - blink_start >= 100) {
      LED1_PORT->ODR ^= 1 << LED1_PIN;
      blink_start = systick_count;
    }

    // "ADC in progress" blinky.
    if (adc_blink_on && systick_count - adc_blink_start >= 10) {
      CLEAR_BIT(LED2_PORT->ODR, 1 << LED2_PIN);
      adc_blink_on = false;
    }

    // ADC timeout error.
    if (adc_running && systick_count - adc_start >= 1500) {
      adc_running = false;
      SET_BIT(LED3_PORT->ODR, 1 << LED3_PIN);
    }

    // Timer event: just used here for blinky and timeout.
    if (timer_fired) {
      timer_fired = false;
      SET_BIT(LED2_PORT->ODR, 1 << LED2_PIN);
      adc_blink_on = true;
      adc_blink_start = systick_count;
      adc_running = true;
      adc_start = systick_count;
    }

    // DMA complete interrupt received.
    if (dma_complete) {
      dma_complete = false;
      adc_running = false;
      usart_tx(dma_adc_sample[0] & 0x00FF);
      usart_tx((dma_adc_sample[0] & 0xFF00) >> 8);
      usart_tx(dma_adc_sample[1] & 0x00FF);
      usart_tx((dma_adc_sample[1] & 0xFF00) >> 8);
      usart_tx(dma_adc_sample[2] & 0x00FF);
      usart_tx((dma_adc_sample[2] & 0xFF00) >> 8);
      usart_tx(dma_adc_sample[3] & 0x00FF);
      usart_tx((dma_adc_sample[3] & 0xFF00) >> 8);
      usart_wait();
    }

    // DMA error interrupt received.
    if (dma_error) {
      dma_error = false;
      adc_running = false;
      SET_BIT(LED3_PORT->ODR, 1 << LED3_PIN);
    }
  }
}

static void configure_timer_dma_adc(void) {
  configure_dma();
  configure_timer();
  configure_common_adc(4, false);

  // Set ADC group regular trigger source: rising edge of TIM2 TRGO.
  MODIFY_REG(ADC1->CR2, ADC_CR2_EXTSEL, 0x0B << ADC_CR2_EXTSEL_Pos);
  MODIFY_REG(ADC1->CR2, ADC_CR2_EXTEN, 0x01 << ADC_CR2_EXTEN_Pos);

  // Set ADC group regular sequencer length: four channels.
  MODIFY_REG(ADC1->SQR1, ADC_SQR1_L, (4 - 1) << ADC_SQR1_L_Pos);

  // Set ADC group regular sequence:
  // channel 4 @ rank 0, 5 @ rank 1, 6 @ rank 2, 7 @ rank 3.
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

  // Enable DMA transfer for ADC.
  MODIFY_REG(ADC1->CR2, ADC_CR2_DMA | ADC_CR2_DDS, ADC_CR2_DDS | ADC_CR2_DMA);
}

// Timer interrupt handler: not actually needed for the ADC
// triggering, but we use it to blink an LED to show when ADC
// conversions are kicked off.
void TIM2_IRQHandler(void) {
  if (READ_BIT(TIM2->SR, TIM_SR_UIF))
    CLEAR_BIT(TIM2->SR, TIM_SR_UIF);
  timer_fired = true;
}

static void configure_timer(void) {
  // Set up timer TIM2 to fire trigger every SAMPLE_PERIOD ms.

  // Configuration of timer as time base:
  //
  // Computation of frequency is done for a timer instance on APB1
  // (clocked by PCLK1).
  //
  // Timer frequency is calculated from the following constants:
  // - TIMER_FREQUENCY: timer frequency (Hz).
  // - TIMER_FREQUENCY_RANGE_MIN: timer minimum frequency possible (Hz).

  // Retrieve timer clock source frequency.
  // If APB1 prescaler != 1, timers have a x2 factor on their clock source.
  uint32_t apb1_prescaler = READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1);
  uint32_t timer_clock_frequency =
    SystemCoreClock >> APBPrescTable[apb1_prescaler >>  RCC_CFGR_PPRE1_Pos];
  if (apb1_prescaler != RCC_CFGR_PPRE1_DIV1) {
    timer_clock_frequency *= 2;
  }

  // Timer prescaler calculation (computation for 16-bit timer,
  // additional + 1 to round the prescaler up).

  // Time base prescaler to have timebase aligned on minimum frequency
  // possible.
  uint32_t timer_prescaler =
    (timer_clock_frequency /
     (TIMER_PRESCALER_MAX_VALUE * TIMER_FREQUENCY_RANGE_MIN)) + 1;

  // Timer reload value in function of timer prescaler to achieve time
  // base period.
  uint32_t timer_reload =
    timer_clock_frequency / (timer_prescaler * TIMER_FREQUENCY);

  // Enable timer peripheral clock.
  __IO uint32_t tmpreg;
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
  tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
  (void)tmpreg;

  // Configure timer.
  WRITE_REG(TIM2->PSC, timer_prescaler - 1);
  WRITE_REG(TIM2->ARR, timer_reload - 1);
  MODIFY_REG(TIM2->CR1, (TIM_CR1_DIR | TIM_CR1_CMS), 0);
  WRITE_REG(TIM2->RCR, 0);
  MODIFY_REG(TIM2->CR2, TIM_CR2_MMS, TIM_CR2_MMS_1); // TRGO update

  // Enable timer interrupts for debugging.
  SET_BIT(TIM2->DIER, TIM_DIER_UIE);
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);

  // Enable counter.
  SET_BIT(TIM2->CR1, TIM_CR1_CEN);

  // Force update generation.
  SET_BIT(TIM2->EGR, TIM_EGR_UG);
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
