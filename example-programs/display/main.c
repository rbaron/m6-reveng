#include <stdint.h>

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

// Normal mode (not partial).
#define TFT_CMD_NRON 0x13

#define TFT_CMD_ON 0x29
// Column set.
#define TFT_CMD_CASET 0x2a
// Row set.
#define TFT_CMD_RASET 0x2b
// Pixel format.
#define TFT_CMD_COLMOD 0x3a
// Signals the start of image drawing.
#define TFT_CMD_RAMWR 0x2c

typedef struct {
  uint8_t cmd;
  uint8_t args_len;
  uint8_t args[];
} tft_cmd_t;

static const tft_cmd_t tft_cmd_normal = {
    .cmd = TFT_CMD_NRON, .args_len = 0, .args = {}};

static const tft_cmd_t tft_cmd_turn_on = {
    .cmd = TFT_CMD_ON, .args_len = 0, .args = {}};

static const tft_cmd_t tft_cmd_set_pixelformat = {
    .cmd = TFT_CMD_COLMOD, .args_len = 1, .args = {0x05}};

// The TFT is 0.96" - 80 cols x 160 rows.
static const tft_cmd_t tft_cmd_set_col_dimension = {
    .cmd = TFT_CMD_CASET,
    .args_len = 4,
    .args = {
        // 2 bytes representing the starting column: 0.
        0x00,
        0x00,
        // 2 bytes representing the end column (inclusive): 79.
        0x00,
        79,
    }};
static const tft_cmd_t tft_cmd_set_row_dimension = {
    .cmd = TFT_CMD_RASET,
    .args_len = 4,
    .args = {
        // 2 bytes representing the starting row: 0.
        0x00,
        0x00,
        // 2 bytes representing the end row (inclusive): 159.
        0x00,
        159,
    }};

static const tft_cmd_t tft_cmd_set_data_write = {
    .cmd = TFT_CMD_RAMWR, .args_len = 0, .args = {}};

static void spi_write_data(const uint8_t *data, size_t len) {
  while (len--) {
    reg_spi_data = *data++;
    while (reg_spi_ctrl & FLD_SPI_BUSY)
      ;
  }
}

static void tft_send_cmd(const tft_cmd_t *cmd) {
  // Chip select low.
  gpio_write(PIN_CS, 0);

  // Command mode, DC low.
  gpio_write(PIN_DC, 0);
  spi_write_data(&cmd->cmd, 1);

  // Data mode, DC high.
  gpio_write(PIN_DC, 1);
  spi_write_data(cmd->args, cmd->args_len);

  // Chip select high.
  gpio_write(PIN_CS, 1);
  sleep_ms(10);
}

static void init_spi_master(void) {
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

static void init_led(void) {
  gpio_set_func(PIN_LED, AS_GPIO);
  gpio_set_output_en(PIN_LED, 1);
  gpio_set_input_en(PIN_LED, 0);
  gpio_write(PIN_LED, 1);
}

int main() {
  cpu_wakeup_init();
  clock_init(SYS_CLK_16M_Crystal);
  gpio_init();
  init_led();

  init_spi_master();

  tft_send_cmd(&tft_cmd_set_pixelformat);
  tft_send_cmd(&tft_cmd_set_col_dimension);
  tft_send_cmd(&tft_cmd_set_row_dimension);
  tft_send_cmd(&tft_cmd_normal);
  tft_send_cmd(&tft_cmd_turn_on);
  tft_send_cmd(&tft_cmd_set_data_write);

  gpio_write(PIN_CS, 0);
  // Data mode, DC high.
  gpio_write(PIN_DC, 1);
  uint8_t b = 0x66;
  for (int i = 0; i < 1024; i++) {
    spi_write_data(&b, 1);
    // sleep_ms(10);
  }
  gpio_write(PIN_CS, 1);

  while (1) {
    gpio_toggle(PIN_LED);
    sleep_ms(500);
  }
  return 0;
}