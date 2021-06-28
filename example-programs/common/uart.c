#include "uart.h"

#include <stdint.h>
// #include <stdio.h>
// #include <string.h>
// #include "common/printf.h"
#include "common/string.h"
#include "drivers/5316/dma.h"
#include "drivers/5316/irq.h"
#include "drivers/5316/uart.h"

// Dummy puchar() implementation. It's required by printf.c, but we don't need
// it.
int putchar(int c) { return 0; }

void uart_init(uint8_t *recv_buff, uint16_t recv_buff_len) {
  uart_set_recbuff((unsigned short *)&recv_buff, recv_buff_len);

  // TX/RX pads on the board.
  uart_set_pin(GPIO_PB4, GPIO_PB5);

  uart_reset();

  // With a system clock of 16 MHz, setting the clock division
  // to 9 and bwc to 13 => baud rate of 115200.
  uart_init_baudrate(9, 13, PARITY_NONE, STOP_BIT_ONE);

  // Enable DMA for TX & RX.
  uart_dma_en(1, 1);
  irq_set_mask(FLD_IRQ_DMA_EN);
  dma_chn_irq_enable(FLD_DMA_CHN_UART_RX | FLD_DMA_CHN_UART_TX, 1);

  // Since we're using DMA, we can disable the irq for the TX and RX pins.
  uart_irq_en(0, 0);
}

void uart_send(const m6_uart_data_t *data) { uart_dma_send((uint16_t *)data); }