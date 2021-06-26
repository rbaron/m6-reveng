/********************************************************************************************************
 * @file     app.c
 *
 * @brief    for TLSR chips
 *
 * @author	 BLE Group
 * @date     May. 12, 2018
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *			 The information contained herein is confidential and
 *proprietary property of Telink Semiconductor (Shanghai) Co., Ltd. and is
 *available under the terms of Commercial License Agreement between Telink
 *Semiconductor (Shanghai) Co., Ltd. and the licensee in separate contract or
 *the terms described here-in. This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the
 *information in this file under Mutual Non-Disclosure Agreement. NO WARRENTY of
 *ANY KIND is provided.
 *
 *******************************************************************************************************/
#include "app.h"

#include <stack/ble/ble.h>

#include "drivers.h"
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

/* ADV Packet, SCAN Response Packet define */
const u8 tbl_advData[] = {
    0x07,
    0x09,
    'r',
    'b',
    'a',
    'r',
    'o',
    '7',
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

const u8 tbl_scanRsp[] = {
    0x07, 0x09, 'r', 'b', 'a', 'r', 'o', '5',
};

u8 user_task_flg;
u8 sendTerminate_before_enterDeep = 0;

u32 interval_update_tick = 0;
int device_in_connection_state;

u32 advertise_begin_tick;

unsigned int lowBattDet_tick = 0;

int ui_mtu_size_exchange_req = 0;

u32 latest_user_event_tick;

void app_switch_to_indirect_adv(u8 e, u8 *p, int n) {
  bls_ll_setAdvParam(MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
                     ADV_TYPE_CONNECTABLE_UNDIRECTED, app_own_address_type, 0,
                     NULL, MY_APP_ADV_CHANNEL, ADV_FP_NONE);

  bls_ll_setAdvEnable(1);  // must: set adv enable
}

void ble_remote_terminate(u8 e, u8 *p, int n)  //*p is terminate reason
{
  device_in_connection_state = 0;

  if (*p == HCI_ERR_CONN_TIMEOUT) {
  } else if (*p == HCI_ERR_REMOTE_USER_TERM_CONN) {  // 0x13

  } else if (*p == HCI_ERR_CONN_TERM_MIC_FAILURE) {
  } else {
  }

#if (BLE_REMOTE_PM_ENABLE)
  // user has push terminate pkt to ble TX buffer before deepsleep
  if (sendTerminate_before_enterDeep == 1) {
    sendTerminate_before_enterDeep = 2;
  }
#endif

  advertise_begin_tick = clock_time();
}

void task_connect(u8 e, u8 *p, int n) {
  /**
   * internal  interval=10ms latency=99  timeout=4s         36uA
   *           interval=10ms latency=199 timeout=6s         27uA
   *           interval=10ms latency=299 timeout=8s         21uA
   * external  interval=10ms latency=99  timeout=4s         33uA
   *           interval=10ms latency=199 timeout=6s         22uA
   *           interval=10ms latency=299 timeout=8s         17uA
   */
  bls_l2cap_requestConnParamUpdate(8, 8, 99,
                                   400);  // interval=10ms latency=99 timeout=4s

  latest_user_event_tick = clock_time();

  ui_mtu_size_exchange_req = 1;

  device_in_connection_state = 1;  //

  interval_update_tick = clock_time() | 1;  // none zero
}

//_attribute_ram_code_
void blt_pm_proc(void) {
#if (BLE_REMOTE_PM_ENABLE)
  {
    bls_pm_setSuspendMask(SUSPEND_ADV | SUSPEND_CONN);

    user_task_flg = key_not_release;

    if (user_task_flg) {
      bls_pm_setManualLatency(0);
    }

#if (RC_DEEP_SLEEP_EN)  // deepsleep
    if (sendTerminate_before_enterDeep ==
        1) {  // sending Terminate and wait for ack before enter deepsleep
      if (user_task_flg) {  // detect key Press again,  can not enter deep now
        sendTerminate_before_enterDeep = 0;
        bls_ll_setAdvEnable(1);  // enable adv again
      }
    } else if (sendTerminate_before_enterDeep == 2) {  // Terminate OK
      analog_write(DEEP_ANA_REG0, CONN_DEEP_FLG);

      cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);  // deepsleep
    }

    // adv 60s, deepsleep
    if (blc_ll_getCurrentState() == BLS_LINK_STATE_ADV &&
        !sendTerminate_before_enterDeep &&
        clock_time_exceed(advertise_begin_tick,
                          ADV_IDLE_ENTER_DEEP_TIME * 1000000)) {
      cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);  // deepsleep
    }
    // conn 60s no event(key/voice/led), enter deepsleep
    else if (device_in_connection_state && !user_task_flg &&
             clock_time_exceed(latest_user_event_tick,
                               CONN_IDLE_ENTER_DEEP_TIME * 1000000)) {
      bls_ll_terminateConnection(
          HCI_ERR_REMOTE_USER_TERM_CONN);  // push terminate cmd into ble TX
                                           // buffer
      bls_ll_setAdvEnable(0);              // disable adv
      sendTerminate_before_enterDeep = 1;
    }
#endif
  }
#endif  // END of  BLE_REMOTE_PM_ENABLE
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

void user_init() {
  gpio_set_func(GPIO_PB4, AS_GPIO);
  gpio_set_output_en(GPIO_PB4, 1);
  gpio_set_input_en(GPIO_PB4, 0);
  gpio_write(GPIO_PB4, 1);

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
  blc_smp_param_setBondingDeviceMaxNumber(
      4);  // default is SMP_BONDING_DEVICE_MAX_NUM, can not bigger that this
           // value and this func must call before bls_smp_enableParing
  bls_smp_enableParing(SMP_PARING_CONN_TRRIGER);
#else
  bls_smp_enableParing(SMP_PARING_DISABLE_TRRIGER);
#endif

  // Set advertisement data.
  bls_ll_setAdvData((u8 *)tbl_advData, sizeof(tbl_advData));
  bls_ll_setScanRspData((u8 *)tbl_scanRsp, sizeof(tbl_scanRsp));

  u8 status = bls_ll_setAdvParam(
      MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX, ADV_TYPE_CONNECTABLE_UNDIRECTED,
      app_own_address_type, 0, NULL, MY_APP_ADV_CHANNEL, ADV_FP_NONE);
  // debug: ADV setting err
  if (status != BLE_SUCCESS) {
    write_reg8(0x8000, 0x11);
    while (1)
      ;
  }

  // Start advertising.
  bls_ll_setAdvEnable(1);
  rf_set_power_level_index(RF_POWER_7P9dBm);

  // Set BLE callbacks.
  bls_app_registerEventCallback(BLT_EV_FLAG_CONNECT, &task_connect);
  bls_app_registerEventCallback(BLT_EV_FLAG_TERMINATE, &ble_remote_terminate);

  /* Power Management initialization */
#if (BLE_REMOTE_PM_ENABLE)
  blc_ll_initPowerManagement_module();  // pm module:      	 optional
  bls_pm_setSuspendMask(SUSPEND_ADV | SUSPEND_CONN);
  bls_app_registerEventCallback(BLT_EV_FLAG_SUSPEND_ENTER,
                                &ble_remote_set_sleep_wakeup);
#else
  bls_pm_setSuspendMask(SUSPEND_DISABLE);
#endif

  advertise_begin_tick = clock_time();
}

/*----------------------------------------------------------------------------*/
/*--------- Main Loop                                             ------------*/
/*----------------------------------------------------------------------------*/
u32 tick_loop;
void main_loop(void) {
  tick_loop++;

  /* BLE entry -------------------------------------------------------------*/
  blt_sdk_main_loop();

/* UI entry --------------------------------------------------------------*/
#if (RC_BTN_ENABLE)
  proc_button();
#endif

  /* Power Management  -----------------------------------------------------*/
  blt_pm_proc();
}