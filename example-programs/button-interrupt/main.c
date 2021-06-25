/*
 *  WARNING: THIS IS NOT WORKING YET.
 *  The interrupt is not being triggered and I need to figure out why.
 */

#include "drivers/5316/bsp.h"
#include "drivers/5316/clock.h"
#include "drivers/5316/compiler.h"
#include "drivers/5316/driver_5316.h"
#include "drivers/5316/gpio.h"
#include "drivers/5316/timer.h"
#include "drivers/5316/uart.h"
#include "drivers/5316/watchdog.h"
#include "stack/ble/blt_config.h"

// Attach a LED to the TX pad (GPIO_PB4) - don't forget a resistor!
#define LED_PIN GPIO_PB4

// Built-in capacitive button is connected to GPIO_PC2.
// #define BUTTON_PIN GPIO_PC2

// RX pad.
#define BUTTON_PIN GPIO_PB5

volatile unsigned int gpio_irq_cnt;

_attribute_ram_code_ void irq_handler(void) {
  // if ((reg_irq_src & FLD_IRQ_GPIO_EN) == FLD_IRQ_GPIO_EN) {
  //   reg_irq_src |= FLD_IRQ_GPIO_EN;
  //   gpio_toggle(LED_PIN);
  // }
  gpio_toggle(LED_PIN);
  gpio_irq_cnt++;
}

int main() {
  blc_pm_select_internal_32k_crystal();
  cpu_wakeup_init();
  clock_init(SYS_CLK_16M_Crystal);
  gpio_init();

  blc_app_loadCustomizedParameters();
  rf_drv_init(RF_MODE_BLE_1M);

  sleep_ms(2000);

  // LED hooked up to TX pad.
  gpio_set_func(LED_PIN, AS_GPIO);
  gpio_set_output_en(LED_PIN, 1);
  gpio_set_input_en(LED_PIN, 0);
  gpio_write(LED_PIN, 1);

  // Built-in capacitive button.
  gpio_set_func(BUTTON_PIN, AS_GPIO);
  gpio_set_output_en(BUTTON_PIN, 0);
  gpio_set_input_en(BUTTON_PIN, 1);
  gpio_setup_up_down_resistor(BUTTON_PIN, PM_PIN_PULLUP_10K);
  gpio_set_interrupt(BUTTON_PIN, GPIO_Pol_falling);

  // RX pad.
  gpio_set_func(GPIO_PC2, AS_GPIO);
  gpio_set_output_en(GPIO_PC2, 0);
  gpio_set_input_en(GPIO_PC2, 1);
  gpio_setup_up_down_resistor(GPIO_PC2, PM_PIN_PULLUP_10K);
  gpio_set_interrupt(GPIO_PC2, GPIO_Pol_falling);

  irq_enable();

  while (1) {
    sleep_ms(200);
  }
  return 0;
}