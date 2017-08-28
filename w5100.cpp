/*   thd  all DMA version 2
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include <stdio.h>
#include <string.h>
//#include <avr/interrupt.h>

#include "wirish.h"
#include "w5100.h"

// DMA config stuff
// SPI1 DMA1  RX CH2   TX CH3
// SPI2 DMA1  RX CH4   TX CH5
#include "dma.h"
#define DMA_RX DMA_CH4
#define DMA_TX DMA_CH5
static uint8_t spi_tx_dma_buf [2048];
static uint8_t spi_rx_dma_buf [2048];
volatile static bool dmaActive;

// W5100 controller instance
W5100Class W5100;

#define TX_RX_MAX_BUF_SIZE 2048
#define TX_BUF 0x1100
#define RX_BUF (TX_BUF + TX_RX_MAX_BUF_SIZE)

#ifdef W5200
#define TXBUF_BASE 0x8000
#define RXBUF_BASE 0xC000
#else
#define TXBUF_BASE 0x4000
#define RXBUF_BASE 0x6000
#endif

static DEF_SPI_PORT;

// DMA support routines
static void spi_dma_irq(){
    dma_irq_cause event = dma_get_irq_cause(DMA1, DMA_TX);
    switch(event) {
        case DMA_TRANSFER_COMPLETE:
            dmaActive = false;
            break;
        case DMA_TRANSFER_ERROR:
            SerialUSB.println("DMA Error - read/write data might be corrupted");
            break;
    }
}

static void do_dma( uint16_t len) 
{

    dma_set_num_transfers(DMA1, DMA_RX, len);
    dma_set_num_transfers(DMA1, DMA_TX, len);

    dmaActive = true;
    dma_enable(DMA1, DMA_TX);
    dma_enable(DMA1, DMA_RX);

    while(dmaActive) delayMicroseconds(1);

    dma_disable(DMA1, DMA_TX);
    dma_disable(DMA1, DMA_RX);
}

void dma_read (uint16_t addr, uint8_t *dat, uint16_t len)
{
    uint8_t *buf = spi_tx_dma_buf;
    uint16_t i;

    *buf++ = addr >> 8;
    *buf++ = addr & 0xFF;
    *buf++ = (0x00 | ((len & 0x7F00) >> 8));
    *buf++ = len & 0x00FF;

    memset (buf, 0, len);
    buf += len;

    W5100.setSS();
    do_dma(buf-spi_tx_dma_buf);
    W5100.resetSS();

    memcpy (dat, &spi_rx_dma_buf[4], len);
}

void dma_write (uint16_t addr, uint8_t *dat, uint16_t len)
{
    uint8_t *buf = spi_tx_dma_buf;
    uint16_t i;

    *buf++ = addr >> 8;
    *buf++ = addr & 0xFF;
    *buf++ = (0x80 | ((len & 0x7F00) >> 8));
    *buf++ = len & 0x00FF;

    memcpy (buf, dat, len);
    buf += len;

    W5100.setSS();
    do_dma(buf-spi_tx_dma_buf);
    W5100.resetSS();
}



void W5100Class::init(void)
{
  delay(300);

  SPI.begin(ETHERNET_SPI_FREQ, MSBFIRST,0); 
  dma_init(DMA1);
  spi_rx_dma_enable(SPI2); //enable SPI over DMA
  spi_tx_dma_enable(SPI2);
  dmaActive = false;       //DMA activity control
    dma_setup_transfer (
        DMA1,
        DMA_RX,
        &SPI2->regs->DR,
        DMA_SIZE_8BITS,
        spi_rx_dma_buf,
        DMA_SIZE_8BITS,
        DMA_MINC_MODE | DMA_TRNS_CMPLT
    );
    dma_set_priority (DMA1, DMA_RX, DMA_PRIORITY_HIGH);
    dma_attach_interrupt (DMA1, DMA_RX, spi_dma_irq);
    dma_setup_transfer (
        DMA1,
        DMA_TX,
        &SPI2->regs->DR,
        DMA_SIZE_8BITS,
        spi_tx_dma_buf,
        DMA_SIZE_8BITS,
        DMA_MINC_MODE | DMA_FROM_MEM
    );
    dma_set_priority (DMA1, DMA_TX, DMA_PRIORITY_HIGH);


  initSS();
  
  writeMR(1<<RST);
  
#ifdef W5200
  for (int i=0; i<MAX_SOCK_NUM; i++) {
    write((0x4000 + i * 0x100 + 0x001F), 2);
    write((0x4000 + i * 0x100 + 0x001E), 2);
  }
#else  
  writeTMSR(0x55);
  writeRMSR(0x55);
#endif

  for (int i=0; i<MAX_SOCK_NUM; i++) {
    SBASE[i] = TXBUF_BASE + SSIZE * i;
    RBASE[i] = RXBUF_BASE + RSIZE * i;
  }
}

uint16_t W5100Class::getTXFreeSize(SOCKET s)
{
  uint16_t val=0, val1=0;
  do {
    val1 = readSnTX_FSR(s);
    if (val1 != 0)
      val = readSnTX_FSR(s);
  } 
  while (val != val1);
  return val;
}

uint16_t W5100Class::getRXReceivedSize(SOCKET s)
{
  uint16_t val=0,val1=0;
  do {
    val1 = readSnRX_RSR(s);
    if (val1 != 0)
      val = readSnRX_RSR(s);
  } 
  while (val != val1);
  return val;
}


void W5100Class::send_data_processing(SOCKET s, const uint8_t *data, uint16_t len)
{
  // This is same as having no offset in a call to send_data_processing_offset
  send_data_processing_offset(s, 0, data, len);
}

void W5100Class::send_data_processing_offset(SOCKET s, uint16_t data_offset, const uint8_t *data, uint16_t len)
{
  uint16_t ptr = readSnTX_WR(s);
  ptr += data_offset;
  uint16_t offset = ptr & SMASK;
  uint16_t dstAddr = offset + SBASE[s];

  if (offset + len > SSIZE) 
  {
    // Wrap around circular buffer
    uint16_t size = SSIZE - offset;
    write(dstAddr, data, size);
    write(SBASE[s], data + size, len - size);
  } 
  else {
    write(dstAddr, data, len);
  }

  ptr += len;
  writeSnTX_WR(s, ptr);
}


void W5100Class::recv_data_processing(SOCKET s, uint8_t *data, uint16_t len, uint8_t peek)
{
  uint16_t ptr;
  ptr = readSnRX_RD(s);
  read_data(s, (uint8_t *)ptr, data, len);
  if (!peek)
  {
    ptr += len;
    writeSnRX_RD(s, ptr);
  }
}

void W5100Class::read_data(SOCKET s, volatile uint8_t *src, volatile uint8_t *dst, uint16_t len)
{
  uint16_t size;
  uint16_t src_mask;
  uint16_t src_ptr;

  src_mask = (uint16_t)((uint32_t)src & RMASK);
  src_ptr = RBASE[s] + src_mask;

  if( (src_mask + len) > RSIZE ) 
  {
    size = RSIZE - src_mask;
    read(src_ptr, (uint8_t *)dst, size);
    dst += size;
    read(RBASE[s], (uint8_t *) dst, len - size);
  } 
  else
    read(src_ptr, (uint8_t *) dst, len);
}


uint8_t W5100Class::write(uint16_t _addr, uint8_t _data)
{
  write(_addr,&_data,1);
  return 1;
}

uint16_t W5100Class::write(uint16_t _addr,const uint8_t *_buf, uint16_t _len)
{
	
#ifdef W5200
    dma_write(_addr,(uint8_t *)_buf,_len);
#else	
	
  for (uint16_t i=0; i<_len; i++)
  {
    setSS();    
    SPI.transfer(0xF0);
    SPI.transfer(_addr >> 8);
    SPI.transfer(_addr & 0xFF);
    _addr++;
    SPI.transfer(_buf[i]);
    resetSS();
  }
#endif
  
  return _len;
}

uint8_t W5100Class::read(uint16_t _addr)
{
  
  uint8_t _data;
  read(_addr,&_data,1);
  return _data;
}

uint16_t W5100Class::read(uint16_t _addr, uint8_t *_buf, uint16_t _len)
{
#ifdef W5200
    dma_read(_addr,_buf,_len);
#else	
	
  for (uint16_t i=0; i<_len; i++)
  {
    setSS();
    SPI.transfer(0x0F);
    SPI.transfer(_addr >> 8);
    SPI.transfer(_addr & 0xFF);
    _addr++;
    _buf[i] = SPI.transfer(0);
    resetSS();
  }
#endif  
  return _len;
}

void W5100Class::execCmdSn(SOCKET s, SockCMD _cmd) {
  // Send command to socket
  writeSnCR(s, _cmd);
  // Wait for command to complete
  while (readSnCR(s))
    ;
}
