#include <stdint.h>

#include "drivers/5316/bsp.h"
#include "drivers/5316/clock.h"
#include "drivers/5316/compiler.h"
#include "drivers/5316/driver_5316.h"
#include "drivers/5316/gpio.h"
#include "drivers/5316/timer.h"
#include "drivers/5316/uart.h"

#define ST7735_TFTWIDTH_128 128   // for 1.44 and mini
#define ST7735_TFTWIDTH_80 80     // for mini
#define ST7735_TFTHEIGHT_128 128  // for 1.44" display
#define ST7735_TFTHEIGHT_160 160  // for 1.8" and mini display

#define ST_CMD_DELAY 0x80  // special signifier for command lists

#define ST77XX_NOP 0x00
#define ST77XX_SWRESET 0x01
#define ST77XX_RDDID 0x04
#define ST77XX_RDDST 0x09

#define ST77XX_SLPIN 0x10
#define ST77XX_SLPOUT 0x11
#define ST77XX_PTLON 0x12
#define ST77XX_NORON 0x13

#define ST77XX_INVOFF 0x20
#define ST77XX_INVON 0x21
#define ST77XX_DISPOFF 0x28
#define ST77XX_DISPON 0x29
#define ST77XX_CASET 0x2A
#define ST77XX_RASET 0x2B
#define ST77XX_RAMWR 0x2C
#define ST77XX_RAMRD 0x2E

#define ST77XX_PTLAR 0x30
#define ST77XX_TEOFF 0x34
#define ST77XX_TEON 0x35
#define ST77XX_MADCTL 0x36
#define ST77XX_COLMOD 0x3A

#define ST77XX_MADCTL_MY 0x80
#define ST77XX_MADCTL_MX 0x40
#define ST77XX_MADCTL_MV 0x20
#define ST77XX_MADCTL_ML 0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_RDID1 0xDA
#define ST77XX_RDID2 0xDB
#define ST77XX_RDID3 0xDC
#define ST77XX_RDID4 0xDD

// Some ready-made 16-bit ('565') color settings:
#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF
#define ST77XX_RED 0xF800
#define ST77XX_GREEN 0x07E0
#define ST77XX_BLUE 0x001F
#define ST77XX_CYAN 0x07FF
#define ST77XX_MAGENTA 0xF81F
#define ST77XX_YELLOW 0xFFE0
#define ST77XX_ORANGE 0xFC00

#define INITR_GREENTAB 0x00
#define INITR_REDTAB 0x01
#define INITR_BLACKTAB 0x02
#define INITR_18GREENTAB INITR_GREENTAB
#define INITR_18REDTAB INITR_REDTAB
#define INITR_18BLACKTAB INITR_BLACKTAB
#define INITR_144GREENTAB 0x01
#define INITR_MINI160x80 0x04
#define INITR_HALLOWING 0x05

// Some register settings
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH 0x04

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5

#define ST7735_PWCTR6 0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Some ready-made 16-bit ('565') color settings:
#define ST7735_BLACK ST77XX_BLACK
#define ST7735_WHITE ST77XX_WHITE
#define ST7735_RED ST77XX_RED
#define ST7735_GREEN ST77XX_GREEN
#define ST7735_BLUE ST77XX_BLUE
#define ST7735_CYAN ST77XX_CYAN
#define ST7735_MAGENTA ST77XX_MAGENTA
#define ST7735_YELLOW ST77XX_YELLOW
#define ST7735_ORANGE ST77XX_ORANGE

static const uint8_t Bcmd[] =
    {                                  // Init commands for 7735B screens
        18,                            // 18 commands in list:
        ST77XX_SWRESET, ST_CMD_DELAY,  //  1: Software reset, no args, w/delay
        50,                            //     50 ms delay
        ST77XX_SLPOUT, ST_CMD_DELAY,  //  2: Out of sleep mode, no args, w/delay
        255,                          //     255 = max (500 ms) delay
        ST77XX_COLMOD, 1 + ST_CMD_DELAY,  //  3: Set color mode, 1 arg + delay:
        0x05,                             //     16-bit color
        10,                               //     10 ms delay
        ST7735_FRMCTR1,
        3 + ST_CMD_DELAY,   //  4: Frame rate control, 3 args + delay:
        0x00,               //     fastest refresh
        0x06,               //     6 lines front porch
        0x03,               //     3 lines back porch
        10,                 //     10 ms delay
        ST77XX_MADCTL, 1,   //  5: Mem access ctl (directions), 1 arg:
        0x08,               //     Row/col addr, bottom-top refresh
        ST7735_DISSET5, 2,  //  6: Display settings #5, 2 args:
        0x15,               //     1 clk cycle nonoverlap, 2 cycle gate
                            //     rise, 3 cycle osc equalize
        0x02,               //     Fix on VTL
        ST7735_INVCTR, 1,   //  7: Display inversion control, 1 arg:
        0x0,                //     Line inversion
        ST7735_PWCTR1, 2 + ST_CMD_DELAY,  //  8: Power control, 2 args + delay:
        0x02,                             //     GVDD = 4.7V
        0x70,                             //     1.0uA
        10,                               //     10 ms delay
        ST7735_PWCTR2, 1,                 //  9: Power control, 1 arg, no delay:
        0x05,                             //     VGH = 14.7V, VGL = -7.35V
        ST7735_PWCTR3, 2,  // 10: Power control, 2 args, no delay:
        0x01,              //     Opamp current small
        0x02,              //     Boost frequency
        ST7735_VMCTR1, 2 + ST_CMD_DELAY,  // 11: Power control, 2 args + delay:
        0x3C,                             //     VCOMH = 4V
        0x38,                             //     VCOML = -1.1V
        10,                               //     10 ms delay
        ST7735_PWCTR6, 2,  // 12: Power control, 2 args, no delay:
        0x11, 0x15, ST7735_GMCTRP1,
        16,  // 13: Gamma Adjustments (pos. polarity), 16 args + delay:
        0x09, 0x16, 0x09, 0x20,  //     (Not entirely necessary, but provides
        0x21, 0x1B, 0x13, 0x19,  //      accurate colors)
        0x17, 0x15, 0x1E, 0x2B, 0x04, 0x05, 0x02, 0x0E, ST7735_GMCTRN1,
        16 + ST_CMD_DELAY,  // 14: Gamma Adjustments (neg. polarity), 16 args +
                            // delay:
        0x0B, 0x14, 0x08, 0x1E,  //     (Not entirely necessary, but provides
        0x22, 0x1D, 0x18, 0x1E,  //      accurate colors)
        0x1B, 0x1A, 0x24, 0x2B, 0x06, 0x06, 0x02, 0x0F,
        10,               //     10 ms delay
        ST77XX_CASET, 4,  // 15: Column addr set, 4 args, no delay:
                          // 0x00, 0x02,                   //     XSTART = 2
                          // 0x00, 0x81,                   //     XEND = 129
        0x00, 0x1f,       //     XSTART = 2
        0x00, 0x81,       //     XEND = 129
        ST77XX_RASET, 4,  // 16: Row addr set, 4 args, no delay:
                          // 0x00, 0x02,                   //     XSTART = 1
                          // 0x00, 0x81,                   //     XEND = 160
        0x00, 0x1f,       //     XSTART = 1
        0x00, 0x81,       //     XEND = 160
        ST77XX_NORON, ST_CMD_DELAY,   // 17: Normal display on, no args, w/delay
        10,                           //     10 ms delay
        ST77XX_DISPON, ST_CMD_DELAY,  // 18: Main screen turn on, no args, delay
        255},                         //     255 = max (500 ms) delay

    Rcmd1[] =
        {        // 7735R init, part 1 (red or green tab)
            15,  // 15 commands in list:
            ST77XX_SWRESET,
            ST_CMD_DELAY,  //  1: Software reset, 0 args, w/delay
            150,           //     150 ms delay
            ST77XX_SLPOUT,
            ST_CMD_DELAY,  //  2: Out of sleep mode, 0 args, w/delay
            255,           //     500 ms delay
            ST7735_FRMCTR1,
            3,  //  3: Framerate ctrl - normal mode, 3 arg:
            0x01, 0x2C,
            0x2D,  //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
            ST7735_FRMCTR2,
            3,  //  4: Framerate ctrl - idle mode, 3 args:
            0x01, 0x2C,
            0x2D,  //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
            ST7735_FRMCTR3,
            6,  //  5: Framerate - partial mode, 6 args:
            0x01, 0x2C,
            0x2D,  //     Dot inversion mode
            0x01, 0x2C,
            0x2D,  //     Line inversion mode
            ST7735_INVCTR,
            1,     //  6: Display inversion ctrl, 1 arg:
            0x07,  //     No inversion
            ST7735_PWCTR1,
            3,  //  7: Power control, 3 args, no delay:
            0xA2,
            0x02,  //     -4.6V
            0x84,  //     AUTO mode
            ST7735_PWCTR2,
            1,     //  8: Power control, 1 arg, no delay:
            0xC5,  //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
            ST7735_PWCTR3,
            2,     //  9: Power control, 2 args, no delay:
            0x0A,  //     Opamp current small
            0x00,  //     Boost frequency
            ST7735_PWCTR4,
            2,     // 10: Power control, 2 args, no delay:
            0x8A,  //     BCLK/2,
            0x2A,  //     opamp current small & medium low
            ST7735_PWCTR5,
            2,  // 11: Power control, 2 args, no delay:
            0x8A, 0xEE, ST7735_VMCTR1,
            1,  // 12: Power control, 1 arg, no delay:
            0x0E, ST77XX_INVOFF,
            0,  // 13: Don't invert display, no args
            ST77XX_MADCTL,
            1,     // 14: Mem access ctl (directions), 1 arg:
                   // 0xC8,  //     row/col addr, bottom-top refresh
            0xC0,  //     row/col addr, bottom-top refresh
            ST77XX_COLMOD,
            1,      // 15: set color mode, 1 arg, no delay:
            0x05},  //     16-bit color

    Rcmd2green[] =
        {       // 7735R init, part 2 (green tab only)
            2,  //  2 commands in list:
            ST77XX_CASET,
            4,  //  1: Column addr set, 4 args, no delay:
            0x00,
            0x02,  //     XSTART = 0
            0x00,
            0x7F + 0x02,  //     XEND = 127
            ST77XX_RASET,
            4,  //  2: Row addr set, 4 args, no delay:
            0x00,
            0x01,  //     XSTART = 0
            0x00,
            0x9F + 0x01},  //     XEND = 159

    Rcmd2red[] =
        {       // 7735R init, part 2 (red tab only)
            2,  //  2 commands in list:
            ST77XX_CASET,
            4,  //  1: Column addr set, 4 args, no delay:
            0x00,
            0x00,  //     XSTART = 0
            0x00,
            0x7F,  //     XEND = 127
            ST77XX_RASET,
            4,  //  2: Row addr set, 4 args, no delay:
            0x00,
            0x00,  //     XSTART = 0
            0x00,
            0x9F},  //     XEND = 159

    Rcmd2green144[] =
        {       // 7735R init, part 2 (green 1.44 tab)
            2,  //  2 commands in list:
            ST77XX_CASET,
            4,  //  1: Column addr set, 4 args, no delay:
            0x00,
            0x00,  //     XSTART = 0
            0x00,
            0x7F,  //     XEND = 127
            ST77XX_RASET,
            4,  //  2: Row addr set, 4 args, no delay:
            0x00,
            0x00,  //     XSTART = 0
            0x00,
            0x7F},  //     XEND = 127

    Rcmd2green160x80[] =
        {       // 7735R init, part 2 (mini 160x80)
            2,  //  2 commands in list:
            ST77XX_CASET,
            4,  //  1: Column addr set, 4 args, no delay:
            0x00,
            0x00,  //     XSTART = 0
            0x00,
            0x4F,  //     XEND = 79
            ST77XX_RASET,
            4,  //  2: Row addr set, 4 args, no delay:
            0x00,
            0x00,  //     XSTART = 0
            0x00,
            0x9F},  //     XEND = 159

    Rcmd3[] =
        {       // 7735R init, part 3 (red or green tab)
            4,  //  4 commands in list:
            ST7735_GMCTRP1,
            16,  //  1: Gamma Adjustments (pos. polarity), 16 args + delay:
            0x02,
            0x1c,
            0x07,
            0x12,  //     (Not entirely necessary, but provides
            0x37,
            0x32,
            0x29,
            0x2d,  //      accurate colors)
            0x29,
            0x25,
            0x2B,
            0x39,
            0x00,
            0x01,
            0x03,
            0x10,
            ST7735_GMCTRN1,
            16,  //  2: Gamma Adjustments (neg. polarity), 16 args + delay:
            0x03,
            0x1d,
            0x07,
            0x06,  //     (Not entirely necessary, but provides
            0x2E,
            0x2C,
            0x29,
            0x2D,  //      accurate colors)
            0x2E,
            0x2E,
            0x37,
            0x3F,
            0x00,
            0x00,
            0x02,
            0x10,
            ST77XX_NORON,
            ST_CMD_DELAY,  //  3: Normal display on, no args, w/delay
            10,            //     10 ms delay
            ST77XX_DISPON,
            ST_CMD_DELAY,  //  4: Main screen turn on, no args w/delay
            100},          //     100 ms delay
    StockCMDs[] = {
        6,
        // SWRESET
        0x01, ST_CMD_DELAY, 100,
        // Sleep out
        0x11, ST_CMD_DELAY, 150,

        // MADCTL
        // 0x36, 1 + ST_CMD_DELAY, 0x00, 10,
        // 0x36, 1 + ST_CMD_DELAY, 0xc0, 10,

        // Color format,
        0x3a, ST_CMD_DELAY + 1, 0x05, 10,

        // Display on
        0x29, ST_CMD_DELAY, 150,

        // 0x2a, 4 + ST_CMD_DELAY, 0x00, 0x1a, 0x00, 0x69, 10,

        // // 0x2b, 4 + ST_CMD_DELAY, 0x00, 0x01, 0x00, 0xa0, 10,
        // 0x2b, 4 + ST_CMD_DELAY, 0x00, 0x47, 0x00, 0x5b, 10,

        // 6,
        // // Sleep out
        // 0x11, ST_CMD_DELAY, 255,

        // // Disp ON
        // 0x21, ST_CMD_DELAY, 10,

        // 0x36, 1 + ST_CMD_DELAY, 0x00, 10,
        // // 0x36, 1 + ST_CMD_DELAY, 0xc0, 10,

        // 0x29, ST_CMD_DELAY, 150,

        // 0x2a, 4 + ST_CMD_DELAY, 0x00, 0x1a, 0x00, 0x69, 10,

        // // 0x2b, 4 + ST_CMD_DELAY, 0x00, 0x01, 0x00, 0xa0, 10,
        // 0x2b, 4 + ST_CMD_DELAY, 0x00, 0x47, 0x00, 0x5b, 10,

        // 0x2c, ST_CMD_DELAY,     20,

        // ST77XX_SLPOUT, ST_CMD_DELAY,  //  2: Out of sleep mode, no args,
        // w/delay 255,                          //     255 = max (500 ms) delay
        // ST77XX_COLMOD, 1 + ST_CMD_DELAY,  //  3: Set color mode, 1 arg +
        // delay: 0x05,                             //     16-bit color 10, //
        // 10 ms delay ST7735_FRMCTR1, 3 + ST_CMD_DELAY,   //  4: Frame rate
        // control, 3 args + delay: 0x00,               //     fastest refresh
        // 0x06,               //     6 lines front porch
        // 0x03,               //     3 lines back porch
        // 10                 //     10 ms delay
};  //     100 ms delay

// 16 Mhz.
#define SYSTEM_CLOCK 16000000

// SPI clock = SYSTEM_CLOCK / (2 * (SPI_DIV + 1))
// So by choosing 15, we get 2 uS SPI clock period.
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

#define PIN_LED GPIO_PB4

_attribute_ram_code_ void irq_handler(void) {}

// Normal mode (not partial).
#define TFT_CMD_NRON 0x13

// #define TFT_CMD_ON 0x29
// // Column set.
// #define TFT_CMD_CASET 0x2a
// // Row set.
// #define TFT_CMD_RASET 0x2b
// // Pixel format.
// #define TFT_CMD_COLMOD 0x3a
// // Signals the start of image drawing.
#define TFT_CMD_RAMWR 0x2c
// // Sleep out.
// #define TFT_CMD_SLEEPOUT 0x11
// // Software reset.
// #define TFT_CMD_SWRESET 0x01

typedef struct {
  uint8_t cmd;
  uint8_t args_len;
  uint8_t args[];
} tft_cmd_t;

static const tft_cmd_t tft_cmd_normal = {
    .cmd = TFT_CMD_NRON, .args_len = 0, .args = {}};

// static const tft_cmd_t tft_cmd_turn_on = {
//     .cmd = TFT_CMD_ON, .args_len = 0, .args = {}};

// static const tft_cmd_t tft_cmd_set_pixelformat = {
//     .cmd = TFT_CMD_COLMOD, .args_len = 1, .args = {0x05}};

// // The TFT is 0.96" - 80 cols x 160 rows.
// static const tft_cmd_t tft_cmd_set_col_dimension = {
//     .cmd = TFT_CMD_CASET,
//     .args_len = 4,
//     .args = {
//         // 2 bytes representing the starting column: 0.
//         0x00,
//         0x00,
//         // 2 bytes representing the end column (inclusive): 79.
//         0x00,
//         79,
//     }};
// static const tft_cmd_t tft_cmd_set_row_dimension = {
//     .cmd = TFT_CMD_RASET,
//     .args_len = 4,
//     .args = {
//         // 2 bytes representing the starting row: 0.
//         0x00,
//         0x00,
//         // 2 bytes representing the end row (inclusive): 159.
//         0x00,
//         159,
//     }};

// static const tft_cmd_t tft_cmd_set_data_write = {
//     .cmd = TFT_CMD_RAMWR, .args_len = 0, .args = {}};

// static const tft_cmd_t tft_cmd_sleepout = {
//     .cmd = TFT_CMD_SLEEPOUT, .args_len = 0, .args = {}};

// static const tft_cmd_t tft_cmd_swreset = {
//     .cmd = TFT_CMD_SWRESET, .args_len = 0, .args = {}};

// static const tft_cmd_t tft_cmd_powerctrl1 = {
//     .cmd = 0xc0, .args_len = 2, .args = {0x1e, 0x70}};

// static const tft_cmd_t tft_cmd_invon = {.cmd = 0x21, .args_len = 0, .args =
// {}};

static void spi_write_data(const uint8_t *data, size_t len) {
  // Set the SPI control register to write.
  reg_spi_ctrl &= ~(FLD_SPI_DATA_OUT_DIS | FLD_SPI_RD);
  while (len--) {
    reg_spi_data = *data++;
    while (reg_spi_ctrl & FLD_SPI_BUSY)
      ;
  }
}

static void spi_send_cmd(uint8_t cmd, const uint8_t *args, uint8_t len) {
  gpio_write(PIN_CS, 0);

  // Command mode, DC low.
  gpio_write(PIN_DC, 0);
  spi_write_data(&cmd, 1);

  gpio_write(PIN_DC, 1);
  // while (len--) {
  //   spi_write_data(args++, 1);
  // }
  spi_write_data(args, len);
  gpio_write(PIN_CS, 1);
}

static void display_init(const uint8_t *addr) {
  uint8_t n_cmds = *addr++;
  uint8_t cmd, n_args;
  uint16_t ms;
  while (n_cmds--) {
    cmd = *addr++;
    n_args = *addr++;
    ms = n_args & ST_CMD_DELAY;
    n_args &= ~ST_CMD_DELAY;
    spi_send_cmd(cmd, addr, n_args);
    addr += n_args;

    if (ms) {
      ms = *addr++;
      if (ms == 255) ms = 500;
      sleep_ms(ms);
    }
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

  sleep_ms(100);
  // RST setup.
  gpio_set_func(PIN_RST, AS_GPIO);
  gpio_set_input_en(PIN_RST, 0);
  gpio_set_output_en(PIN_RST, 1);
  gpio_write(PIN_RST, 0);
  sleep_ms(100);
  gpio_write(PIN_RST, 1);
  sleep_ms(100);
}

static void init_led(void) {
  gpio_set_func(PIN_LED, AS_GPIO);
  gpio_set_output_en(PIN_LED, 1);
  gpio_set_input_en(PIN_LED, 0);
  gpio_write(PIN_LED, 1);
}

static void write_cmd(uint8_t cmd) {
  gpio_write(PIN_DC, 0);
  spi_write_data(&cmd, 1);
  gpio_write(PIN_DC, 1);
}

static void set_window(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  uint8_t zero = 0;
  write_cmd(ST77XX_CASET);
  spi_write_data(&zero, 1);
  spi_write_data(&x, 1);
  // x = x + w - 1;
  x++;
  spi_write_data(&zero, 1);
  spi_write_data(&x, 1);

  write_cmd(ST77XX_RASET);
  spi_write_data(&zero, 1);
  spi_write_data(&y, 1);
  // y = y + h - 1;
  y++;
  spi_write_data(&zero, 1);
  spi_write_data(&x, 1);

  write_cmd(ST77XX_RAMWR);
}

int main() {
  cpu_wakeup_init();
  clock_init(SYS_CLK_16M_Crystal);
  gpio_init();
  init_led();

  init_spi_master();

  sleep_ms(500);

  // display_init(Rcmd1);
  // display_init(Rcmd2green160x80);
  // display_init(Rcmd3);
  // display_init(Bcmd);

  display_init(StockCMDs);

  uint8_t b = 0xff;
  for (uint8_t i = 0; i < 150; i++) {
    gpio_write(PIN_CS, 0);
    set_window(i, i, 1, 1);
    spi_write_data(&b, 1);
    spi_write_data(&b, 1);
    gpio_write(PIN_CS, 1);
    sleep_ms(10);
  }

  // sleep_ms(10);
  // tft_send_cmd(&tft_cmd_normal);

  while (1) {
    gpio_toggle(PIN_LED);
    sleep_ms(500);
  }
  return 0;
}

// tft_send_cmd(&tft_cmd_swreset);
// sleep_ms(100);
// tft_send_cmd(&tft_cmd_sleepout);
// sleep_ms(500);
// tft_send_cmd(&tft_cmd_set_pixelformat);
// sleep_ms(10);
// tft_send_cmd(&tft_cmd_set_col_dimension);
// sleep_ms(10);
// tft_send_cmd(&tft_cmd_set_row_dimension);
// sleep_ms(10);
// tft_send_cmd(&tft_cmd_powerctrl1);
// sleep_ms(10);
// tft_send_cmd(&tft_cmd_normal);
// sleep_ms(10);
// tft_send_cmd(&tft_cmd_invon);
// sleep_ms(10);
// tft_send_cmd(&tft_cmd_turn_on);
// sleep_ms(10);
// tft_send_cmd(&tft_cmd_set_data_write);
// sleep_ms(10);

// sleep_ms(100);

// Command mode, DC low.
// gpio_write(PIN_CS, 0);

// gpio_write(PIN_DC, 0);
// // Write RAM.
// uint8_t b = TFT_CMD_RAMWR;
// spi_write_data(&b, 1);

// // Data mode, DC high.
// gpio_write(PIN_DC, 1);

// set_window(15, 15, 50, 50);
// b = 0x0f;
// spi_write_data(&b, 1);
// b = 0xf0;
// spi_write_data(&b, 1);
// sleep_ms(10);
// uint8_t b;
// for (uint8_t i = 0; i < 256; i++) {
//   spi_write_data(&i, 1);
// }