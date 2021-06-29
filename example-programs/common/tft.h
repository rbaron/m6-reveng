#ifndef __EXAMPLE_PROGRAMS_COMMON_TFT_H__
#define __EXAMPLE_PROGRAMS_COMMON_TFT_H__

#include <stdint.h>

// 16 Mhz.
#define SYSTEM_CLOCK 16000000

// SPI clock = SYSTEM_CLOCK / (2 * (SPI_DIV + 1))
// For exaxmple, by setting it to 15, we get 2 uS SPI clock period.
#define SPI_DIV 1

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
// Normal mode (not partial).
#define TFT_CMD_NRON 0x13
// Turn the display on.
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
// 1. Without any mirrorring:
// x <----.
//        |
//        |
//        \/ y
// x-direction
//
// 1. With X-mirrorring:
// .-----> x
// |
// |
// \/ y
// 1. With Y-mirrorring:
//        /\ y
//        |
//        |
// x <----.
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

void spi_init_master(void);

void tft_init_display(void);

void tft_set_window(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

void tft_draw_pixel(uint8_t x, uint8_t y, uint16_t color);

void tft_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);

void tft_draw_char(char c, uint8_t x, uint8_t y, uint16_t color, uint8_t size);

void tft_draw_text(char *text, uint8_t x, uint8_t y, uint16_t color,
                   uint8_t size);

void tft_clear_screen();

void tft_send_cmd(const tft_cmd_t *cmd);

#endif  // __EXAMPLE_PROGRAMS_COMMON_TFT_H__