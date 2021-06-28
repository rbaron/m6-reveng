#ifndef __EXAMPLE_PROGRAMS_COMMON_UART_H__
#define __EXAMPLE_PROGRAMS_COMMON_UART_H__

#include <stdarg.h>
#include <stdint.h>

#include "drivers/5316/gpio.h"
#include "drivers/5316/uart.h"

#define UART_TX_LEN (128 - 4)

// Hacky macro to forward variadic arguments to sprintf.
#define uart_printf(send_buff, format, ...)               \
  sprintf((char *)send_buff.data, format, ##__VA_ARGS__); \
  send_buff.len = strlen((char *)send_buff.data);         \
  uart_send(&send_buff);

typedef struct {
  unsigned int len;
  uint8_t data[UART_TX_LEN];
} m6_uart_data_t;

// Initializes DMA UART with baud rate of 115200.
// TX pin => GPIO_PB4
// RX pin => GPIO_PB5
void uart_init(uint8_t *recv_buff, uint16_t recv_buff_len);

void uart_send(const m6_uart_data_t *data);

#endif  // __EXAMPLE_PROGRAMS_COMMON_UART_H__