#include "drivers/5316/bsp.h"
#include "drivers/5316/clock.h"
#include "drivers/5316/compiler.h"
#include "drivers/5316/driver_5316.h"
#include "drivers/5316/gpio.h"
#include "drivers/5316/timer.h"
#include "drivers/5316/uart.h"

#define CLOCK_SYS_CLOCK_HZ 16000000
#define CLOCK_SYS_CLOCK_1MS (CLOCK_SYS_CLOCK_HZ / 1000)
#define TIMER_TRIGGER_PERIOD (200 * CLOCK_SYS_CLOCK_1MS)

// Attach a LED to the TX pad (GPIO_PB4) - don't forget a resistor!
#define LED_PIN GPIO_PB4

_attribute_ram_code_ void irq_handler(void) {
  // If this is a timer2 interrupt.
  if (reg_tmr_sta & FLD_TMR_STA_TMR2) {
    // Toogle our LED.
    gpio_toggle(LED_PIN);
    // Clear the interrupt bit.
    reg_tmr_sta |= FLD_TMR_STA_TMR2;
    // Reset the tick counter.
    reg_tmr2_tick = 0;
  }
}

int main() {
  blc_pm_select_internal_32k_crystal();
  cpu_wakeup_init();
  clock_init(SYS_CLK_16M_Crystal);
  gpio_init();

  // LED hooked up to TX pad.
  gpio_set_func(LED_PIN, AS_GPIO);
  gpio_set_output_en(LED_PIN, 1);
  gpio_set_input_en(LED_PIN, 0);
  gpio_write(LED_PIN, 1);

  timer2_set_mode(TIMER_MODE_SYSCLK, 0, TIMER_TRIGGER_PERIOD);
  timer_start(TIMER2);

  irq_enable();

  while (1) {
    sleep_ms(5000);
  }
  return 0;
}