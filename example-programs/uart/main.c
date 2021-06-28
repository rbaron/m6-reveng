#include "../common/uart.h"
#include "common/printf.h"
#include "common/string.h"
#include "drivers/5316/driver_5316.h"

_attribute_ram_code_ void irq_handler(void) {}

// Buffer that holds the data to be sent over UART.
m6_uart_data_t send_buff;

// Buffer that holds the received data. Not used in this example.
uint8_t recv_buff[32];

int main() {
  cpu_wakeup_init();
  clock_init(SYS_CLK_16M_Crystal);
  gpio_init();

  uart_init(recv_buff, sizeof(recv_buff));

  int tick = 0;
  while (1) {
    uart_printf(send_buff, "Tick counter: #%d\n", tick++);
    sleep_ms(500);
  }
  return 0;
}