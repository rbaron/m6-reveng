#include "drivers/5316/bsp.h"
#include "drivers/5316/clock.h"
#include "drivers/5316/compiler.h"
#include "drivers/5316/driver_5316.h"
#include "drivers/5316/gpio.h"
#include "drivers/5316/timer.h"
#include "drivers/5316/uart.h"

_attribute_ram_code_ void irq_handler(void) {}

int main() {
  cpu_wakeup_init();
  clock_init(SYS_CLK_16M_Crystal);
  gpio_init();

  // TX pad.
  gpio_set_func(GPIO_PB4, AS_GPIO);
  gpio_set_output_en(GPIO_PB4, 1);
  gpio_set_input_en(GPIO_PB4, 0);
  gpio_write(GPIO_PB4, 1);

  while (1) {
    gpio_toggle(GPIO_PB4);
    sleep_ms(500);
  }
  return 0;
}