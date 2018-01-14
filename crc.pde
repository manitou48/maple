// maple CRC  fixed poly
//  (poly=0x04c11db7 init=0xffffffff refin=true refout=true xorout=0xffffffff check=0xcbf43926)
#include <stdint.h>

#define PRREG(x) SerialUSB.print(#x" 0x"); SerialUSB.println(x,HEX)

typedef struct crc_reg_map {
  __io uint32 DR;        //data
  __io uint32 IDR;       // independent data
  __io uint32 CR;       // control

} 
crc_reg_map;

#define CRC        ((struct crc_reg_map*)0x40023000)

__attribute__((always_inline))  uint32_t rbit(uint32_t x) {
  uint32_t y;
  asm("rbit %0,%1" : 
  "=r" (y) : 
  "r" (x));
  return y;
}


void setup() {
  SerialUSB.begin();
  delay(3000);
  RCC_BASE->AHBENR |= 0x40; //RCC_AHBENR_CRCEN;
  PRREG(RCC_BASE->AHBENR);
  CRC->CR = 1;
  PRREG(CRC->DR);
  CRC->DR = 0;
  PRREG(CRC->DR);
  uint32_t v = ~0 ^ CRC->DR;
  SerialUSB.println(v,HEX);
  v = rbit(v);
  SerialUSB.println(v, HEX);  
}

#define REPS 1000000

void loop() {
  uint32_t t, v;

  t=micros();
  CRC->CR = 1;
  for (int i=0;i<REPS;i++) {
    CRC->DR = i;
  }
  v = CRC->DR;
  t=micros()-t;
  SerialUSB.println(t);
  float mbs = 32.*REPS/t;
  SerialUSB.println(mbs);
  delay(3000);
}



