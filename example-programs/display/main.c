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
// For exaxmple, by setting it to 15, we get 2 uS SPI clock period.
#define SPI_DIV 3

// SDA/MOSI.
#define PIN_SDA GPIO_PC3
// SCL/Clock.
#define PIN_SCL GPIO_PC5
// DC/RS Data selection (labeled RS in the display datasheet).
#define PIN_DC GPIO_PC6
// Chip select, active low.
#define PIN_CS GPIO_PA1
// RST pin, active low.
#define PIN_RST GPIO_PA0
// LED panel catode transistor base pin, active low.
#define PIN_LEDKBASE GPIO_PA2
// LED connected to the TX labeled pin.
#define PIN_LED GPIO_PB4

_attribute_ram_code_ void irq_handler(void) {}

// Normal mode (not partial).
#define TFT_CMD_NRON 0x13

#define TFT_CMD_ON 0x29
// Memory access pattern.
#define TFT_CMD_MADCTL 0x36
// Column set.
#define TFT_CMD_CASET 0x2a
// Row set.
#define TFT_CMD_RASET 0x2b
// Pixel format.
#define TFT_CMD_COLMOD 0x3a
// Signals the start of image drawing.
#define TFT_CMD_RAMWR 0x2c
// Sleep out.
#define TFT_CMD_SLEEPOUT 0x11
// Software reset.
#define TFT_CMD_SWRESET 0x01
// Set inversion on.
#define TFT_CMD_INVON 0x21

// When the TFT is screen side up, ribbon connector on top:
// x <----.
//        |
//        |
//        \/ y
// x-direction
#define TFT_WIDTH 80
// y-direction
#define TFT_HEIGHT 160

#define TFT_X_OFFSET 26
#define TFT_Y_OFFSET 1

// 16-bit RGB-565 format.
#define TFT_COLOR_BLACK 0x0000
#define TFT_COLOR_WHITE 0xffff
#define TFT_COLOR_RED 0xf800
#define TFT_COLOR_GREEN 0x07e0
#define TFT_COLOR_BLUE 0x001f

typedef struct {
  uint8_t cmd;
  uint8_t sleep_ms;
  uint8_t args_len;
  uint8_t args[];
} tft_cmd_t;

// Get out of sleep mode.
static const tft_cmd_t tft_cmd_sleepout = {
    .cmd = TFT_CMD_SLEEPOUT, .sleep_ms = 120, .args_len = 0, .args = {}};

// Memory access direction - it controls the direction the pixels are written
// to the display.
// 0x00 - no mirrorring
// 0x80 - x mirrorring
static const tft_cmd_t tft_cmd_madctl = {
    .cmd = TFT_CMD_MADCTL, .sleep_ms = 10, .args_len = 1, .args = {0x00}};

// Turn on the display.
static const tft_cmd_t tft_cmd_disp_on = {
    .cmd = TFT_CMD_ON, .sleep_ms = 120, .args_len = 0, .args = {}};

// Soft reset command.
static const tft_cmd_t tft_cmd_swreset = {
    .cmd = TFT_CMD_SWRESET, .sleep_ms = 120, .args_len = 0, .args = {}};

// Normal (not partial) mode.
static const tft_cmd_t tft_cmd_normal = {
    .cmd = TFT_CMD_NRON, .sleep_ms = 10, .args_len = 0, .args = {}};

// Set pixel color format (565 RGB - 16 bits per pixel).
static const tft_cmd_t tft_cmd_set_pixelformat = {
    .cmd = TFT_CMD_COLMOD, .sleep_ms = 10, .args_len = 1, .args = {0x05}};

// Set column max dimmensions.
static const tft_cmd_t tft_cmd_set_col_dimension = {
    .cmd = TFT_CMD_CASET,
    .sleep_ms = 10,
    .args_len = 4,
    .args = {
        // 2 bytes representing the starting column.
        0x00,
        0x00,
        // 2 bytes representing the end column (inclusive).
        0x00,
        TFT_WIDTH - 1,
    }};

// Set row max dimmensions.
static const tft_cmd_t tft_cmd_set_row_dimension = {
    .cmd = TFT_CMD_RASET,
    .sleep_ms = 10,
    .args_len = 4,
    .args = {
        // 2 bytes representing the starting row.
        0x00,
        0x00,
        // 2 bytes representing the end row (inclusive).
        0x00,
        159,
        TFT_HEIGHT - 1,
    }};

// Signal that we are about to push some image data to the driver.
static const tft_cmd_t tft_cmd_set_data_write = {
    .cmd = TFT_CMD_RAMWR, .sleep_ms = 0, .args_len = 0, .args = {}};

// In this module, it seems like _enabling_ inversion produces the expected
// colors. If we don't enable this, colors look inverted by default.
static const tft_cmd_t tft_cmd_invon = {
    .cmd = TFT_CMD_INVON, .sleep_ms = 10, .args_len = 0, .args = {}};

static void inline spi_write_data(const uint8_t *data, size_t len) {
  // Set the SPI control register to write.
  reg_spi_ctrl &= ~(FLD_SPI_DATA_OUT_DIS | FLD_SPI_RD);
  while (len--) {
    reg_spi_data = *data++;
    while (reg_spi_ctrl & FLD_SPI_BUSY)
      ;
  }
}

static inline void spi_write8(uint8_t data) { return spi_write_data(&data, 1); }

static inline void spi_write16(uint16_t data) {
  spi_write8(data >> 8);
  spi_write8(data & 0xff);
}

static void spi_send_cmd(uint8_t cmd) {
  // Command mode - DC low.
  gpio_write(PIN_DC, 0);
  spi_write_data(&cmd, 1);
  // Data mode - DC high.
  gpio_write(PIN_DC, 1);
}

static void tft_set_window(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  // Set column.
  spi_send_cmd(TFT_CMD_CASET);
  x += TFT_X_OFFSET;
  spi_write16(x);
  spi_write16(x + w - 1);
  // Set row.
  spi_send_cmd(TFT_CMD_RASET);
  y += TFT_Y_OFFSET;
  spi_write16(y);
  spi_write16(y + h - 1);
  // Signal that we will send image data next.
  spi_send_cmd(TFT_CMD_RAMWR);
}

static void tft_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                          uint16_t color) {
  gpio_write(PIN_CS, 0);
  tft_set_window(x, y, w, h);
  for (int i = 0; i < w * h; i++) {
    spi_write16(color);
  }
  gpio_write(PIN_CS, 1);
}

static void tft_clear_screen() {
  return tft_draw_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_COLOR_BLACK);
}

static void tft_send_cmd(const tft_cmd_t *cmd) {
  // Chip select low.
  gpio_write(PIN_CS, 0);
  // Command mode, DC low.
  gpio_write(PIN_DC, 0);
  spi_write_data(&cmd->cmd, 1);
  // Data mode, DC high - push command arguments, if any.
  gpio_write(PIN_DC, 1);
  spi_write_data(cmd->args, cmd->args_len);
  // Chip select high - we're done.
  gpio_write(PIN_CS, 1);
  sleep_ms(cmd->sleep_ms);
}

static void spi_init_master(void) {
  // MODE0 means SDA (MOSI) is sampled at the leading edge of SCK.
  spi_master_init(SPI_DIV, SPI_MODE0);

  // SDA, SCL setup.
  gpio_set_func(PIN_SDA, AS_SPI_MDO);
  gpio_set_func(PIN_SCL, AS_SPI_MCK);
  gpio_set_output_en(PIN_SDA, 1);
  gpio_set_output_en(PIN_SCL, 1);

  gpio_set_data_strength(PIN_SDA, 0);
  gpio_set_data_strength(PIN_SCL, 0);

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

  // LEDKBASE setup.
  gpio_set_func(PIN_LEDKBASE, AS_GPIO);
  gpio_set_input_en(PIN_LEDKBASE, 0);
  gpio_set_output_en(PIN_LEDKBASE, 1);
  gpio_write(PIN_LEDKBASE, 1);

  // RST setup - active low.
  gpio_set_func(PIN_RST, AS_GPIO);
  gpio_set_input_en(PIN_RST, 0);
  gpio_set_output_en(PIN_RST, 1);
  gpio_write(PIN_RST, 1);
}

static void led_init(void) {
  gpio_set_func(PIN_LED, AS_GPIO);
  gpio_set_output_en(PIN_LED, 1);
  gpio_set_input_en(PIN_LED, 0);
  gpio_write(PIN_LED, 1);
}

static void tft_init_display(void) {
  // Reset pin.
  gpio_write(PIN_RST, 0);
  sleep_ms(100);
  gpio_write(PIN_RST, 1);
  sleep_ms(100);

  tft_send_cmd(&tft_cmd_swreset);
  tft_send_cmd(&tft_cmd_sleepout);
  tft_send_cmd(&tft_cmd_madctl);
  tft_send_cmd(&tft_cmd_set_pixelformat);
  tft_send_cmd(&tft_cmd_invon);
  tft_send_cmd(&tft_cmd_disp_on);
  tft_send_cmd(&tft_cmd_set_col_dimension);
  tft_send_cmd(&tft_cmd_set_row_dimension);

  tft_clear_screen();
}

int main() {
  cpu_wakeup_init();
  clock_init(SYS_CLK_16M_Crystal);
  gpio_init();
  led_init();
  spi_init_master();
  tft_init_display();

  // Draw a few rectangles in the middle of the display.
  uint8_t side = TFT_WIDTH;
  tft_draw_rect((TFT_WIDTH - side) / 2, (TFT_HEIGHT - side) / 2, side, side,
                TFT_COLOR_RED);

  side -= 10;
  tft_draw_rect((TFT_WIDTH - side) / 2, (TFT_HEIGHT - side) / 2, side, side,
                TFT_COLOR_GREEN);

  side -= 10;
  tft_draw_rect((TFT_WIDTH - side) / 2, (TFT_HEIGHT - side) / 2, side, side,
                TFT_COLOR_BLUE);

  while (1) {
    gpio_toggle(PIN_LED);
    sleep_ms(500);
  }
  return 0;
}
