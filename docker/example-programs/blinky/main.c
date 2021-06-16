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

  // clock_init(0x42);
  clock_init(SYS_CLK_16M_Crystal);
  gpio_init();

  // TEST pad.
  // gpio_set_func(GPIO_PC1, AS_GPIO);
  // gpio_set_output_en(GPIO_PC1, 1);
  // gpio_set_input_en(GPIO_PC1, 0);
  // gpio_write(GPIO_PC1, 0);

  // // DAT pad.
  // gpio_set_func(GPIO_PA4, AS_GPIO);
  // gpio_set_output_en(GPIO_PA4, 1);
  // gpio_set_input_en(GPIO_PA4, 0);
  // gpio_write(GPIO_PA4, 0);

  // // RX pad.
  // gpio_set_func(GPIO_PB5, AS_GPIO);
  // gpio_set_output_en(GPIO_PB5, 1);
  // gpio_set_input_en(GPIO_PB5, 0);
  // gpio_write(GPIO_PB5, 1);

  // TX pad.
  gpio_set_func(GPIO_PB4, AS_GPIO);
  gpio_set_output_en(GPIO_PB4, 1);
  gpio_set_input_en(GPIO_PB4, 0);
  gpio_write(GPIO_PB4, 1);

  while (1) {
    // gpio_toggle(GPIO_PC1);
    // gpio_toggle(GPIO_PA4);
    // gpio_toggle(GPIO_PB5);
    gpio_toggle(GPIO_PB4);
    sleep_ms(100);
  }
  return 0;
}