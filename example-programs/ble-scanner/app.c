#include "app.h"

#include "../common/uart.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "stack/ble/ll/ll_scan.h"
#include "tl_common.h"
#include "vendor/common/blt_common.h"
#include "vendor/common/keyboard.h"

#define RC_DEEP_SLEEP_EN 0

#define ADV_IDLE_ENTER_DEEP_TIME 60   // 60 s
#define CONN_IDLE_ENTER_DEEP_TIME 60  // 60 s

#define MY_DIRECT_ADV_TIME_US 2000000

#define MY_APP_ADV_CHANNEL BLT_ENABLE_ADV_ALL

#define MY_ADV_INTERVAL_MIN ADV_INTERVAL_30MS
#define MY_ADV_INTERVAL_MAX ADV_INTERVAL_35MS

#define BLE_DEVICE_ADDRESS_TYPE BLE_DEVICE_ADDRESS_PUBLIC
own_addr_type_t app_own_address_type = OWN_ADDRESS_PUBLIC;

MYFIFO_INIT(blt_rxfifo, 64, 8);
MYFIFO_INIT(blt_txfifo, 40, 16);

m6_uart_data_t uart_send_buff;
uint8_t recv_buff[32];

/* ADV Packet, SCAN Response Packet define */
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

// const u8 tbl_scanRsp[] = {
//     0x07, 0x09, 'r', 'b', 'a', 'r', 'o', '1',
// };

void app_switch_to_indirect_adv(u8 e, u8 *p, int n) {
  bls_ll_setAdvParam(MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
                     ADV_TYPE_CONNECTABLE_UNDIRECTED, app_own_address_type, 0,
                     NULL, MY_APP_ADV_CHANNEL, ADV_FP_NONE);

  // Start advertising.
  bls_ll_setAdvEnable(1);
}

void ble_remote_terminate(u8 e, u8 *p, int n) {}

void task_connect(u8 e, u8 *p, int n) {
  // interval=10ms latency=99 timeout=4s
  bls_l2cap_requestConnParamUpdate(8, 8, 99, 400);
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
  u8 evCode = h & 0xff;
  if (evCode == HCI_EVT_LE_META) {
    event_adv_report_t *p = (event_adv_report_t *)para;
    u8 subEventCode = p->subcode;
    if (subEventCode == HCI_SUB_EVT_LE_ADVERTISING_REPORT) {
      event_adv_report_t *adv = (event_adv_report_t *)p;
      uart_printf(uart_send_buff, "%02x:%02x:%02x:%02x:%02x:%02x\n",
                  adv->mac[5], adv->mac[4], adv->mac[3], adv->mac[2],
                  adv->mac[1], adv->mac[0]);
      for (int i = 0; i < adv->len; i++) {
        uart_printf(uart_send_buff, "%02x ", adv->data[i]);
      }
      uart_printf(uart_send_buff, "\n");
    }
  }
  return 0;
}

void user_init() {
  uart_init(recv_buff, sizeof(recv_buff));

  u8 mac_public[6];
  u8 mac_random_static[6];
  blc_initMacAddress(CFG_ADR_MAC, mac_public, mac_random_static);

  app_own_address_type = OWN_ADDRESS_PUBLIC;

  blc_ll_initBasicMCU(mac_public);
  blc_ll_initAdvertising_module(mac_public);
  blc_ll_initSlaveRole_module();

  extern void my_att_init(void);
  // Initialize attribute table.
  my_att_init();
  // L2CAP initialization.
  blc_l2cap_register_handler(blc_l2cap_packet_receive);

  /*-- BLE SMP initialization ----------------------------------------------*/
#if (BLE_REMOTE_SECURITY_ENABLE)
  // This func must call before bls_smp_enableParing
  blc_smp_param_setBondingDeviceMaxNumber(4);
  bls_smp_enableParing(SMP_PARING_CONN_TRRIGER);
#else
  bls_smp_enableParing(SMP_PARING_DISABLE_TRRIGER);
#endif

  // Set advertisement data.
  bls_ll_setAdvData((u8 *)tbl_advData, sizeof(tbl_advData));
  // bls_ll_setScanRspData((u8 *)tbl_scanRsp, sizeof(tbl_scanRsp));

  // u8 status = bls_ll_setAdvParam(
  //     MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
  //     ADV_TYPE_CONNECTABLE_UNDIRECTED, app_own_address_type, 0, NULL,
  //     MY_APP_ADV_CHANNEL, ADV_FP_NONE);
  // if (status != BLE_SUCCESS) {
  //   write_reg8(0x8000, 0x11);
  //   while (1)
  //     ;
  // }
  u8 status = bls_ll_setAdvParam(MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
                                 ADV_TYPE_NONCONNECTABLE_UNDIRECTED,
                                 app_own_address_type, 0, NULL,
                                 MY_APP_ADV_CHANNEL, ADV_FP_NONE);
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
  // blc_ll_addScanningInConnSlaveRole();

  // Set BLE callbacks.
  bls_app_registerEventCallback(BLT_EV_FLAG_CONNECT, &task_connect);
  bls_app_registerEventCallback(BLT_EV_FLAG_TERMINATE, &ble_remote_terminate);

#if (BLE_REMOTE_PM_ENABLE)
  blc_ll_initPowerManagement_module();
  bls_pm_setSuspendMask(SUSPEND_ADV | SUSPEND_CONN);
  bls_app_registerEventCallback(BLT_EV_FLAG_SUSPEND_ENTER,
                                &ble_remote_set_sleep_wakeup);
#else
  bls_pm_setSuspendMask(SUSPEND_DISABLE);
#endif

  // Turn LED on if everything is alright by now.
  gpio_write(GPIO_PB4, 1);
}

void main_loop(void) {
  blt_sdk_main_loop();
  blt_pm_proc();
}