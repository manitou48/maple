// timer clock DMA sine table to DAC   
// DAC1/2   D10/D13

#include <stdint.h>
#include <stdio.h>
#include "timer.h"
#include "dac.h"
#include "dma.h"

// TIM2_UP DMA1 ch2  
#define DMAn DMA1
#define DMA_CHn DMA_CH2

#define PRREG(x) SerialUSB.print(#x" 0x"); SerialUSB.println(x,HEX)

volatile uint32_t ticks;
void timer_isr(void ) {
  ticks++;
}

static volatile uint16_t sinetable[] = {
  2047,    2147,    2248,    2348,    2447,    2545,    2642,    2737,
  2831,    2923,    3012,    3100,    3185,    3267,    3346,    3422,
  3495,    3564,    3630,    3692,    3750,    3804,    3853,    3898,
  3939,    3975,    4007,    4034,    4056,    4073,    4085,    4093,
  4095,    4093,    4085,    4073,    4056,    4034,    4007,    3975,
  3939,    3898,    3853,    3804,    3750,    3692,    3630,    3564,
  3495,    3422,    3346,    3267,    3185,    3100,    3012,    2923,
  2831,    2737,    2642,    2545,    2447,    2348,    2248,    2147,
  2047,    1948,    1847,    1747,    1648,    1550,    1453,    1358,
  1264,    1172,    1083,     995,     910,     828,     749,     673,
  600,     531,     465,     403,     345,     291,     242,     197,
  156,     120,      88,      61,      39,      22,      10,       2,
  0,       2,      10,      22,      39,      61,      88,     120,
  156,     197,     242,     291,     345,     403,     465,     531,
  600,     673,     749,     828,     910,     995,    1083,    1172,
  1264,    1358,    1453,    1550,    1648,    1747,    1847,    1948,
};

#define LED_RATE 1000    // in microseconds; 

HardwareTimer timer(2);

void setup() {
  SerialUSB.begin();

  // DAC init
  dac_init(DAC,DAC_CH1);
  DAC_BASE->CR |= 0x1020;   // DMAEN1 TIM2 TRGO

  // DMA init
  dma_init(DMAn);
  dma_setup_transfer(DMAn, DMA_CHn,
  &(DAC_BASE->DHR12R1), DMA_SIZE_16BITS,
  sinetable, DMA_SIZE_16BITS,
  (DMA_MINC_MODE | DMA_FROM_MEM | DMA_CIRC_MODE)
    );
  dma_set_num_transfers(DMAn, DMA_CHn, sizeof(sinetable)/2);  // 16-bit words
  dma_enable(DMAn, DMA_CHn);

  // timer init
  timer.pause();
  timer.setPeriod(LED_RATE); // in microseconds

  // Set up an interrupt on channel 1
  //  timer.setChannelMode(TIMER_OUTPUT_COMPARE);
  timer.setMode(1,TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  // timer.attachCompare1Interrupt(timer_isr);  
  TIMER2_BASE->CR2 = 0x20;    // TRGO
  TIMER2_BASE->DIER = TIMER_DIER_UDE | TIMER_DIER_UIE;  // causes hang

  // Refresh the timer's count, prescale, and overflow
  timer.refresh();

  // Start the timer counting
  timer.resume();
}

void loop() {
  PRREG(DAC_BASE->CR);
  PRREG(TIMER2_BASE->CR1);
  PRREG(DMA1_BASE->CCR2);
  SerialUSB.println(ticks);
  delay(2000);
}



