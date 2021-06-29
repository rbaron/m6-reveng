#include "app.h"

#include "../common/tft.h"
#include "../common/uart.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "stack/ble/ll/ll_scan.h"
#include "tl_common.h"
#include "vendor/common/blt_common.h"
#include "vendor/common/keyboard.h"

// If set, will output extra BLE advertisement info via UART.
#define BPARASITE_DEBUG 0

#define MY_APP_ADV_CHANNEL BLT_ENABLE_ADV_ALL

#define MY_ADV_INTERVAL_MIN ADV_INTERVAL_30MS
#define MY_ADV_INTERVAL_MAX ADV_INTERVAL_35MS

#define BLE_DEVICE_ADDRESS_TYPE BLE_DEVICE_ADDRESS_PUBLIC

MYFIFO_INIT(blt_rxfifo, 64, 8);
MYFIFO_INIT(blt_txfifo, 40, 16);

m6_uart_data_t uart_send_buff;
uint8_t recv_buff[32];

const u8 tbl_advData[] = {
    0x07,
    0x09,
    'r',
    'b',
    'a',
    'r',
    'o',
    'n',
    // BLE limited discoverable mode and BR/EDR not supported.
    0x02,
    0x01,
    0x05,
    // List of service UUIDs - 0x180f => battery service.
    0x03,
    0x02,
    0x0F,
    0x18,
};

typedef struct {
  uint8_t counter;
  uint16_t battery_millivoltage;
  uint16_t temp_millicelcius;
  uint16_t air_humidity;
  uint16_t soil_moisture;
  uint8_t mac_addr[6];
} b_parasite_adv_t;

#define PRST_INFO_PADDING 8
#define PRST_INFO_FONT_HEIGHT 6
#define PRST_INFO_HEADER_COLOR 0x3694
#define PRST_INFO_VALUE_COLOR 0xffff
#define PRST_INFO_MOISTURE_BAR_HEIGHT 12
#define PRST_INFO_DRY_COLOR 0xe349
#define PRST_INFO_MID_COLOR 0xe60c
#define PRST_INFO_WET_COLOR 0x969c
static void draw_parasite_data(const b_parasite_adv_t *data) {
  char buff[10];
  tft_clear_screen();

  uint16_t y = 0;
  uint16_t size = 1;

  uint8_t moisture = (100 * data->soil_moisture) / (0xffff - 1);
  uint16_t moisture_color = PRST_INFO_WET_COLOR;
  if (moisture < 30) {
    moisture_color = PRST_INFO_DRY_COLOR;
  } else if (moisture < 70) {
    moisture_color = PRST_INFO_MID_COLOR;
  }

  sprintf(buff, "%02d%%", moisture);
  size = 2;
  tft_draw_text("Moisture", 0, y, PRST_INFO_HEADER_COLOR, size);
  y += size * PRST_INFO_FONT_HEIGHT + PRST_INFO_PADDING;
  size = 4;
  tft_draw_text(buff, 0, y, moisture_color, size);
  y += size * PRST_INFO_FONT_HEIGHT + PRST_INFO_PADDING;

  // Moisture bar.
  tft_draw_rect(0, y, max2(10, (moisture * TFT_WIDTH) / 100),
                PRST_INFO_MOISTURE_BAR_HEIGHT, moisture_color);
  y += PRST_INFO_MOISTURE_BAR_HEIGHT + PRST_INFO_PADDING;

  size = 2;
  tft_draw_text("Air", 0, y, PRST_INFO_HEADER_COLOR, size);
  sprintf(buff, "%dC", data->temp_millicelcius / 1000);
  size = 2;
  tft_draw_text(buff, TFT_WIDTH / 2, y, PRST_INFO_VALUE_COLOR, size);
  y += size * PRST_INFO_FONT_HEIGHT + PRST_INFO_PADDING;
  sprintf(buff, "%02d%%", (100 * data->air_humidity) / (0xffff - 1));
  tft_draw_text(buff, TFT_WIDTH / 2, y, PRST_INFO_VALUE_COLOR, size);
  y += size * PRST_INFO_FONT_HEIGHT + PRST_INFO_PADDING;

  size = 2;
  tft_draw_text("Batt", 0, y, PRST_INFO_HEADER_COLOR, size);
  sprintf(buff, "%01d.%01dV", data->battery_millivoltage / 1000,
          (data->battery_millivoltage % 1000) / 100);
  size = 2;
  tft_draw_text(buff, TFT_WIDTH / 2, y, PRST_INFO_VALUE_COLOR, size);
  y += size * PRST_INFO_FONT_HEIGHT + PRST_INFO_PADDING;

  size = 1;
  tft_draw_text("Counter", 0, y, PRST_INFO_HEADER_COLOR, size);
  sprintf(buff, "0x%02x", data->counter);
  tft_draw_text(buff, TFT_WIDTH / 2, y, PRST_INFO_VALUE_COLOR, size);
  y += size * PRST_INFO_FONT_HEIGHT + PRST_INFO_PADDING;
}

void blt_pm_proc(void) {
#if (BLE_REMOTE_PM_ENABLE)
  bls_pm_setSuspendMask(SUSPEND_ADV | SUSPEND_CONN);
#endif  // BLE_REMOTE_PM_ENABLE
}

void ble_remote_set_sleep_wakeup(u8 e, u8 *p,
                                 int n) {  // 3995*16     sys_tick_per_us
  if (blc_ll_getCurrentState() == BLS_LINK_STATE_CONN &&
      ((u32)(bls_pm_getSystemWakeupTick() - clock_time())) >
          80 * CLOCK_16M_SYS_TIMER_CLK_1MS) {  // suspend time > 30ms.add gpio
                                               // wakeup
    bls_pm_setWakeupSource(PM_WAKEUP_CORE);    // gpio CORE wakeup suspend
  }
}

int hci_event_handle(u32 h, u8 *para, int n) {
  static uint8_t last_counter = 0xff;

  u8 evCode = h & 0xff;
  if (evCode == HCI_EVT_LE_META) {
    event_adv_report_t *p = (event_adv_report_t *)para;
    u8 subEventCode = p->subcode;
    if (subEventCode == HCI_SUB_EVT_LE_ADVERTISING_REPORT) {
      if (p->mac[5] == 0xf0 && p->mac[4] == 0xca && p->mac[3] == 0xf0 &&
          p->mac[2] == 0xca && p->mac[1] == 0x00 && p->mac[0] == 0x08) {
        if (p->len != 31 || !p->data) return 0;

#if (BPARASITE_DEBUG)
        uart_printf(uart_send_buff, "%02x:%02x:%02x:%02x:%02x:%02x\n",
                    p->mac[5], p->mac[4], p->mac[3], p->mac[2], p->mac[1],
                    p->mac[0]);
        uart_printf(uart_send_buff, "%d\n", p->len);
        for (int i = 0; i < p->len; i++) {
          uart_printf(uart_send_buff, "%02x ", p->data[i]);
        }
        uart_printf(uart_send_buff, "\n");
#endif

        b_parasite_adv_t bp_data;
        bp_data.counter = p->data[8];

        if (bp_data.counter == last_counter) return 0;

        last_counter = bp_data.counter;
        bp_data.battery_millivoltage = p->data[9] << 8 | p->data[10];
        bp_data.temp_millicelcius = p->data[11] << 8 | p->data[12];
        bp_data.air_humidity = p->data[13] << 8 | p->data[14];
        bp_data.soil_moisture = p->data[15] << 8 | p->data[16];
        // memcpy(&bp_data.mac_addr, data + 17, sizeof(bp_data.mac_addr));
        // uart_printf(uart_send_buff, "Counter: %02x\n", bp_data.counter);
        // uart_printf(uart_send_buff, "Counter: %02x ; Soil moisture: %04x\n",
        //             bp_data.counter, bp_data.soil_moisture);
        draw_parasite_data(&bp_data);
        // uart_printf(uart_send_buff,
        //             "Counter: %02x ; Soil moisture: %04x (%u %%) ; Batt: %u
        //             mV "
        //             "; Air Humidity: %u %% ; Temp: %u mC\n",
        //             bp_data.counter, bp_data.soil_moisture,
        //             (100 * bp_data.soil_moisture) / (0xffff - 1),
        //             bp_data.battery_millivoltage,
        //             (100 * bp_data.air_humidity) / (0xffff - 1),
        //             bp_data.temp_millicelcius);
      }
    }
  }
  return 0;
}

void user_init() {
  uart_init(recv_buff, sizeof(recv_buff));
  spi_init_master();

  u8 mac_public[6];
  u8 mac_random_static[6];
  blc_initMacAddress(CFG_ADR_MAC, mac_public, mac_random_static);

  blc_ll_initBasicMCU(mac_public);
  blc_ll_initAdvertising_module(mac_public);
  blc_ll_initSlaveRole_module();

  extern void my_att_init(void);
  my_att_init();
  blc_l2cap_register_handler(blc_l2cap_packet_receive);

#if (BLE_REMOTE_SECURITY_ENABLE)
  blc_smp_param_setBondingDeviceMaxNumber(4);
  bls_smp_enableParing(SMP_PARING_CONN_TRRIGER);
#else
  bls_smp_enableParing(SMP_PARING_DISABLE_TRRIGER);
#endif

  // Set advertisement data.
  bls_ll_setAdvData((u8 *)tbl_advData, sizeof(tbl_advData));

  u8 status =
      bls_ll_setAdvParam(MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
                         ADV_TYPE_NONCONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC,
                         0, NULL, MY_APP_ADV_CHANNEL, ADV_FP_NONE);
  if (status != BLE_SUCCESS) {
    write_reg8(0x8000, 0x11);
    while (1)
      ;
  }

  // Start advertising.
  bls_ll_setAdvEnable(1);
  rf_set_power_level_index(RF_POWER_7P9dBm);

  // Scan
  blc_ll_initScanning_module(mac_public);
  blc_hci_le_setEventMask_cmd(HCI_LE_EVT_MASK_ADVERTISING_REPORT);
  blc_hci_registerControllerEventHandler(&hci_event_handle);

  blc_ll_setScanParameter(SCAN_TYPE_PASSIVE, SCAN_INTERVAL_100MS,
                          SCAN_INTERVAL_100MS, OWN_ADDRESS_PUBLIC,
                          SCAN_FP_ALLOW_ADV_ANY);
  blc_ll_addScanningInAdvState();

  bls_pm_setSuspendMask(SUSPEND_DISABLE);

  tft_init_display();

  tft_clear_screen();
}

void main_loop(void) {
  blt_sdk_main_loop();
  // blt_pm_proc();
}