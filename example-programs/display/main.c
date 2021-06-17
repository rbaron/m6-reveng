#include "drivers/5316/bsp.h"
#include "drivers/5316/clock.h"
#include "drivers/5316/compiler.h"
#include "drivers/5316/driver_5316.h"
#include "drivers/5316/gpio.h"
#include "drivers/5316/timer.h"
#include "drivers/5316/uart.h"

// 16 Mhz.
#define SYSTEM_CLOCK 16000000

// SPI clock = SYSTEM_CLOCK / (2 * (SPI_DIV + 1))
// So by choosing 15, we get 2 uS SPI clock period.
#define SPI_DIV 15

// SDA/MOSI.
#define PIN_SDA GPIO_PC3
// SCL/Clock.
#define PIN_SCL GPIO_PC5
// DC/RS Data selection (labeled RS in the display datasheet).
#define PIN_DC GPIO_PC6
// Chip select, active low.
#define PIN_CS GPIO_PA1

#define PIN_LED GPIO_PB4

_attribute_ram_code_ void irq_handler(void) {}

void init_spi_master() {
  spi_master_init(SPI_DIV, SPI_MODE0);

  // SDA, SCL setup.
  gpio_set_func(PIN_SDA, AS_SPI_MDO);
  gpio_set_func(PIN_SCL, AS_SPI_MCK);
  gpio_set_input_en(PIN_SDA, 0);
  gpio_set_input_en(PIN_SCL, 0);
  gpio_set_output_en(PIN_SDA, 1);
  gpio_set_output_en(PIN_SCL, 1);

  // CS setup.
  gpio_set_func(PIN_CS, AS_GPIO);
  gpio_set_input_en(PIN_CS, 0);
  gpio_set_output_en(PIN_CS, 1);
  gpio_write(PIN_CS, 1);

  // DC setup.
  gpio_set_func(PIN_DC, AS_GPIO);
  gpio_set_input_en(PIN_DC, 0);
  gpio_set_output_en(PIN_DC, 1);
  gpio_write(PIN_DC, 0);
}

int main() {
  cpu_wakeup_init();
  clock_init(SYS_CLK_16M_Crystal);
  gpio_init();

  init_spi_master();

  gpio_set_func(PIN_LED, AS_GPIO);
  gpio_set_output_en(PIN_LED, 1);
  gpio_set_input_en(PIN_LED, 0);
  gpio_write(PIN_LED, 1);

  reg_spi_data = 0x5a;
  while (reg_spi_ctrl & FLD_SPI_BUSY)
    ;

  while (1) {
    gpio_toggle(PIN_LED);
    reg_spi_data = 0x5a;
    while (reg_spi_ctrl & FLD_SPI_BUSY)
      ;
    sleep_ms(100);
  }
  return 0;
}