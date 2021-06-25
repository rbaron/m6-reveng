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

#if (__PROJECT_5316_BLE_SAMPLE__)

#define RC_DEEP_SLEEP_EN 1

#define ADV_IDLE_ENTER_DEEP_TIME 60   // 60 s
#define CONN_IDLE_ENTER_DEEP_TIME 60  // 60 s

#define MY_DIRECT_ADV_TMIE 2000000

#define MY_APP_ADV_CHANNEL BLT_ENABLE_ADV_ALL

#define MY_ADV_INTERVAL_MIN ADV_INTERVAL_30MS
#define MY_ADV_INTERVAL_MAX ADV_INTERVAL_35MS

#define BLE_DEVICE_ADDRESS_TYPE BLE_DEVICE_ADDRESS_PUBLIC
own_addr_type_t app_own_address_type = OWN_ADDRESS_PUBLIC;

MYFIFO_INIT(blt_rxfifo, 64, 8);
MYFIFO_INIT(blt_txfifo, 40, 16);

/* ADV Packet, SCAN Response Packet define */
const u8 tbl_advData[] = {
    0x05, 0x09, 'G',  'h',  'i',  'd',
    0x02, 0x01, 0x05,  // BLE limited discoverable mode and BR/EDR not supported
    0x03, 0x19, 0x80, 0x01,  // 384, Generic Remote Control, Generic category
    0x05, 0x02, 0x12, 0x18, 0x0F, 0x18,  // incomplete list of service class
                                         // UUIDs (0x1812, 0x180F)
};

const u8 tbl_scanRsp[] = {
    0x08, 0x09, 'G', 'R', 'e', 'm', 'o', 't', 'e',
};

u8 user_task_flg;
u8 sendTerminate_before_enterDeep = 0;

u32 interval_update_tick = 0;
int device_in_connection_state;

u32 advertise_begin_tick;

unsigned int lowBattDet_tick = 0;

int ui_mtu_size_exchange_req = 0;

u32 latest_user_event_tick;

#if (STUCK_KEY_PROCESS_ENABLE)
u32 stuckKey_keyPressTime;
#endif

/*----------------------------------------------------------------------------*/
/*------------- CallBack function of BLE                      ----------------*/
/*----------------------------------------------------------------------------*/
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

/*----------------------------------------------------------------------------*/
/*------------- Key Function                                  ----------------*/
/*----------------------------------------------------------------------------*/
#if (RC_BTN_ENABLE)
#define MAX_BTN_SIZE 2
#define BTN_VALID_LEVEL 0

#define USER_BTN_1 0x01
#define USER_BTN_2 0x02

u32 ctrl_btn[] = {SW1_GPIO, SW2_GPIO};
u8 btn_map[MAX_BTN_SIZE] = {USER_BTN_1, USER_BTN_2};

typedef struct {
  u8 cnt;  // count button num
  u8 btn_press;
  u8 keycode[MAX_BTN_SIZE];  // 6 btn
} vc_data_t;
vc_data_t vc_event;

typedef struct {
  u8 btn_history[4];  // vc history btn save
  u8 btn_filter_last;
  u8 btn_not_release;
  u8 btn_new;  // new btn  flag
} btn_status_t;
btn_status_t btn_status;

int key_not_release;
u8 btn_debounce_filter(u8 *btn_v) {
  u8 change = 0;

  for (int i = 3; i > 0; i--) {
    btn_status.btn_history[i] = btn_status.btn_history[i - 1];
  }
  btn_status.btn_history[0] = *btn_v;

  if (btn_status.btn_history[0] == btn_status.btn_history[1] &&
      btn_status.btn_history[1] == btn_status.btn_history[2] &&
      btn_status.btn_history[0] != btn_status.btn_filter_last) {
    change = 1;

    btn_status.btn_filter_last = btn_status.btn_history[0];
  }

  return change;
}

u8 vc_detect_button(int read_key) {
  u8 btn_changed, i;
  memset(&vc_event, 0, sizeof(vc_data_t));  // clear vc_event
  // vc_event.btn_press = 0;

  for (i = 0; i < MAX_BTN_SIZE; i++) {
    if (BTN_VALID_LEVEL != !gpio_read(ctrl_btn[i])) {
      vc_event.btn_press |= BIT(i);
    }
  }

  btn_changed = btn_debounce_filter(&vc_event.btn_press);

  if (btn_changed && read_key) {
    for (i = 0; i < MAX_BTN_SIZE; i++) {
      if (vc_event.btn_press & BIT(i)) {
        vc_event.keycode[vc_event.cnt++] = btn_map[i];
      }
    }
    return 1;
  }

  return 0;
}

void proc_button(void) {
  static u32 button_det_tick;
  if (clock_time_exceed(button_det_tick, 5000)) {
    button_det_tick = clock_time();
  } else {
    return;
  }

  //	static u32 button_history = 0;
  //	static u32 last_singleKey_press_tick;

  static int button1_press_flag;
  static u32 button1_press_tick;
  static int button2_press_flag;
  static u32 button2_press_tick;

  static int consumer_report = 0;

  int det_key = vc_detect_button(1);

  if (det_key)  // key change: press or release
  {
    key_not_release = 1;
    u8 key0 = vc_event.keycode[0];
    // u8 key1 = vc_event.keycode[1];

    if (vc_event.cnt == 2)  // two key press
    {
    } else if (vc_event.cnt == 1)  // one key press
    {
      if (key0 == USER_BTN_1) {
        button1_press_flag = 1;
        button1_press_tick = clock_time();
        u16 consumer_key = MKEY_VOL_UP;
        bls_att_pushNotifyData(HID_CONSUME_REPORT_INPUT_DP_H,
                               (u8 *)&consumer_key, 2);
        consumer_report = 1;
      } else if (key0 == USER_BTN_2) {
        button2_press_flag = 1;
        button2_press_tick = clock_time();
        u16 consumer_key = MKEY_VOL_DN;
        bls_att_pushNotifyData(HID_CONSUME_REPORT_INPUT_DP_H,
                               (u8 *)&consumer_key, 2);
        consumer_report = 1;
      }
    } else {  // release
      key_not_release = 0;
      button1_press_flag = 0;
      button2_press_flag = 0;
      if (consumer_report) {
        consumer_report = 0;
        u16 consumer_key = 0;
        bls_att_pushNotifyData(HID_CONSUME_REPORT_INPUT_DP_H,
                               (u8 *)&consumer_key, 2);
      }
    }
  }
}
#endif

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
  /*
   ***************************************************************************
   * Keyboard matrix initialization. These section must be before
   *battery_power_check. Because when low battery,chip will entry deep.if placed
   *after battery_power_check, it is possible that can not wake up chip.
   ***************************************************************************
   */
#if (RC_BTN_ENABLE)
  for (int i = 0; i < (sizeof(ctrl_btn) / sizeof(*ctrl_btn)); i++) {
    gpio_set_wakeup(ctrl_btn[i], 0,
                    1);  // drive pin core(gpio) high wakeup suspend
    cpu_set_gpio_wakeup(ctrl_btn[i], 0,
                        1);  // drive pin pad high wakeup deepsleep
  }
#endif

  /*-- BLE stack initialization --------------------------------------------*/
  u8 mac_public[6];
  u8 mac_random_static[6];
  blc_initMacAddress(CFG_ADR_MAC, mac_public, mac_random_static);

#if (BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_PUBLIC)
  app_own_address_type = OWN_ADDRESS_PUBLIC;
#elif (BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_RANDOM_STATIC)
  app_own_address_type = OWN_ADDRESS_RANDOM;
  blc_ll_setRandomAddr(mac_random_static);
#endif

  /*-- BLE Controller initialization ---------------------------------------*/
  blc_ll_initBasicMCU(mac_public);  // mandatory
  blc_ll_initAdvertising_module(
      mac_public);                // adv module: mandatory for BLE slave,
  blc_ll_initSlaveRole_module();  // slave module: mandatory for BLE slave,

  /*-- BLE Host initialization ---------------------------------------------*/
  extern void my_att_init(void);
  // GATT initialization
  my_att_init();
  // L2CAP initialization
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

  // HID_service_on_android7p0_init();  //hid device on android 7.0/7.1

  /*-- USER application initialization -------------------------------------*/
  bls_ll_setAdvData((u8 *)tbl_advData, sizeof(tbl_advData));
  bls_ll_setScanRspData((u8 *)tbl_scanRsp, sizeof(tbl_scanRsp));

  /* Configure ADV packet */
#if (BLE_REMOTE_SECURITY_ENABLE)
  // get bonded device number
  u8 bond_number = blc_smp_param_getCurrentBondingDeviceNumber();
  smp_param_save_t bondInfo;
  if (bond_number)  // at least 1 bonding device exist
  {
    // get the latest bonding device (index: bond_number-1 )
    blc_smp_param_loadByIndex(bond_number - 1, &bondInfo);
  }

  if (bond_number)  // set direct adv
  {
    // set direct adv
    u8 status =
        bls_ll_setAdvParam(MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
                           ADV_TYPE_CONNECTABLE_DIRECTED_LOW_DUTY,
                           app_own_address_type, bondInfo.peer_addr_type,
                           bondInfo.peer_addr, MY_APP_ADV_CHANNEL, ADV_FP_NONE);
    // debug: ADV setting err
    if (status != BLE_SUCCESS) {
      write_reg8(0x8000, 0x11);
      while (1)
        ;
    }

    // it is recommended that direct adv only last for several seconds, then
    // switch to indirect adv
    bls_ll_setAdvDuration(MY_DIRECT_ADV_TMIE, 1);
    bls_app_registerEventCallback(BLT_EV_FLAG_ADV_DURATION_TIMEOUT,
                                  &app_switch_to_indirect_adv);
  } else  // set indirect ADV
#endif
  {
    u8 status = bls_ll_setAdvParam(MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
                                   ADV_TYPE_CONNECTABLE_UNDIRECTED,
                                   app_own_address_type, 0, NULL,
                                   MY_APP_ADV_CHANNEL, ADV_FP_NONE);
    // debug: ADV setting err
    if (status != BLE_SUCCESS) {
      write_reg8(0x8000, 0x11);
      while (1)
        ;
    }
  }

  bls_ll_setAdvEnable(1);                     // adv enable
  rf_set_power_level_index(RF_POWER_7P9dBm);  // OK

  // ble event call back
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
#endif  // end of __PROJECT_5316_BLE_SAMPLE__
