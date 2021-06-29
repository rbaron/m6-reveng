#include "tft.h"

#include "Picopixel.h"
#include "drivers/5316/clock.h"
#include "drivers/5316/spi.h"
#include "gfxfont.h"

// Get out of sleep mode.
static const tft_cmd_t tft_cmd_sleepout = {
    .cmd = TFT_CMD_SLEEPOUT, .sleep_ms = 120, .args_len = 0, .args = {}};

// Memory access direction - it controls the direction the pixels are written
// to the display.
// 0x00 - no mirrorring
// 0x40 - x mirrorring
// 0x80 - y mirrorring
// 0xc0 - x and y mirrorring
static const tft_cmd_t tft_cmd_madctl = {
    .cmd = TFT_CMD_MADCTL, .sleep_ms = 10, .args_len = 1, .args = {0x80}};

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

static const GFXfont *font = &Picopixel;

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

void tft_set_window(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
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

void tft_draw_pixel(uint8_t x, uint8_t y, uint16_t color) {
  gpio_write(PIN_CS, 0);
  tft_set_window(x, y, 1, 1);
  spi_write16(color);
  gpio_write(PIN_CS, 1);
}

void tft_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
  gpio_write(PIN_CS, 0);
  tft_set_window(x, y, w, h);
  for (int i = 0; i < w * h; i++) {
    spi_write16(color);
  }
  gpio_write(PIN_CS, 1);
}

void tft_draw_char(char c, uint8_t x, uint8_t y, uint16_t color, uint8_t size) {
  GFXglyph *glyph = font->glyph + c - font->first;
  uint16_t bitmap_offset = glyph->bitmapOffset;

  uint8_t height = size * (font->yAdvance - 1);
  uint8_t baseline = y + height - 2 * size;
  uint8_t y0 = baseline + size * glyph->yOffset;

  // bitmask holds the current bit we're inspecting. If that bit is set, we draw
  // a pixel there.
  uint8_t bitmask = 0;
  uint8_t bitmap_byte;
  for (uint8_t dy = 0; dy < glyph->height; dy++) {
    for (uint8_t dx = 0; dx < glyph->width; dx++) {
      // Do we need to load a new byte from the bitmap?
      if (!bitmask) {
        bitmap_byte = font->bitmap[bitmap_offset++];
        bitmask = 0x80;
      }
      // Is the current bit set?
      if (bitmap_byte & bitmask) {
        if (size == 1) {
          tft_draw_pixel(x + dx, y0 + dy, color);
        } else {
          tft_draw_rect(x + dx * size, y0 + dy * size, size, size, color);
        }
      }
      bitmask >>= 1;
    }
  }
}

// Draws a NULL-terminated string pointed by text.
void tft_draw_text(char *text, uint8_t x, uint8_t y, uint16_t color,
                   uint8_t size) {
  uint8_t offset = 0;
  for (; *text; text++) {
    GFXglyph *glyph = font->glyph + *text - font->first;
    tft_draw_char(*text, x + offset, y, color, size);
    offset += size * (glyph->xAdvance + 1);
  }
}

void tft_clear_screen() {
  return tft_draw_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_COLOR_BLACK);
}

void tft_send_cmd(const tft_cmd_t *cmd) {
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

void spi_init_master(void) {
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

void tft_init_display(void) {
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