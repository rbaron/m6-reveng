#include "drivers/5316/bsp.h"
#include "drivers/5316/clock.h"
#include "drivers/5316/compiler.h"
#include "drivers/5316/driver_5316.h"
#include "drivers/5316/gpio.h"
#include "drivers/5316/timer.h"
#include "drivers/5316/uart.h"

_attribute_ram_code_ void irq_handler(void) {}

// #define N_PINS 13
// Attach a LED to the TX pad (GPIO_PB4) - don't forget a resistor!
#define LED_PIN GPIO_PB4

// Built-in capacitive button is connected to GPIO_PC2.
#define BUTTON_PIN GPIO_PC2

int main() {
  cpu_wakeup_init();
  clock_init(SYS_CLK_16M_Crystal);
  gpio_init();

  // LED hooked up to TX pad.
  gpio_set_func(LED_PIN, AS_GPIO);
  gpio_set_output_en(LED_PIN, 1);
  gpio_set_input_en(LED_PIN, 0);
  gpio_write(LED_PIN, 1);

  // Built-in capacitive button.
  gpio_set_func(BUTTON_PIN, AS_GPIO);
  gpio_set_output_en(BUTTON_PIN, 0);
  gpio_set_input_en(BUTTON_PIN, 1);

  unsigned int button_state = 0;
  while (1) {
    unsigned int curr_button_state = gpio_read(BUTTON_PIN);

    // When you press/release the built-in button, the LED should toggle.
    if (curr_button_state != button_state) {
      button_state = curr_button_state;
      gpio_toggle(GPIO_PB4);
    }
    sleep_ms(200);
  }
  return 0;
}