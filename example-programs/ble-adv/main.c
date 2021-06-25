#include "drivers/5316/bsp.h"
#include "drivers/5316/clock.h"
#include "drivers/5316/compiler.h"
#include "drivers/5316/driver_5316.h"
#include "drivers/5316/gpio.h"
#include "drivers/5316/register.h"
#include "drivers/5316/timer.h"
#include "drivers/5316/uart.h"
// KEEP
#include "drivers/5316/driver_5316.h"
#include "stack/ble/ble_common.h"
#include "stack/ble/ll/ll.h"
#include "vendor/common/blt_common.h"

// TX pad.
#define PIN_LED GPIO_PB4

#define MY_APP_ADV_CHANNEL BLT_ENABLE_ADV_ALL

#define MY_ADV_INTERVAL_MIN ADV_INTERVAL_30MS
#define MY_ADV_INTERVAL_MAX ADV_INTERVAL_35MS

#define BLE_DEVICE_ADDRESS_TYPE BLE_DEVICE_ADDRESS_PUBLIC

own_addr_type_t app_own_address_type = OWN_ADDRESS_PUBLIC;

const u8 tbl_advData[] = {
    0x05, 0x09, 'r',  'b',  'a',  'r',
    0x02, 0x01, 0x05,  // BLE limited discoverable mode and BR/EDR not supported
    0x03, 0x19, 0x80, 0x01,  // 384, Generic Remote Control, Generic category
    0x05, 0x02, 0x12, 0x18, 0x0F, 0x18,  // incomplete list of service class
                                         // UUIDs (0x1812, 0x180F)
};

const u8 tbl_scanRsp[] = {
    0x08, 0x09, 'G', 'R', 'e', 'm', 'o', 't', 'e',
};

_attribute_ram_code_ void irq_handler(void) {}

void init_led(void) {
  // TX pad.
  gpio_set_func(GPIO_PB4, AS_GPIO);
  gpio_set_output_en(GPIO_PB4, 1);
  gpio_set_input_en(GPIO_PB4, 0);
  gpio_write(GPIO_PB4, 1);
}

int main() {
  cpu_wakeup_init();
  clock_init(SYS_CLK_16M_Crystal);
  gpio_init();
  init_led();
  blc_app_loadCustomizedParameters();

  rf_drv_init(RF_MODE_BLE_1M);

  /*-- BLE stack initialization
  --------------------------------------------*/
  u8 mac_public[6];
  u8 mac_random_static[6];
  blc_initMacAddress(CFG_ADR_MAC, mac_public, mac_random_static);

#if (BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_PUBLIC)
  app_own_address_type = OWN_ADDRESS_PUBLIC;
#elif (BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_RANDOM_STATIC)
  app_own_address_type = OWN_ADDRESS_RANDOM;
  blc_ll_setRandomAddr(mac_random_static);
#endif

  //   /*-- BLE Controller initialization
  //   ---------------------------------------*/
  blc_ll_initBasicMCU(mac_public);  // mandatory
  blc_ll_initAdvertising_module(
      mac_public);                // adv module: mandatory for BLE slave,
  blc_ll_initSlaveRole_module();  // slave module: mandatory for BLE slave,

  /*-- BLE Host initialization
  ---------------------------------------------*/
  extern void my_att_init(void);
  // GATT initialization
  my_att_init();
  // L2CAP initialization
  blc_l2cap_register_handler(blc_l2cap_packet_receive);

  // #if (BLE_REMOTE_SECURITY_ENABLE)
  // blc_smp_param_setBondingDeviceMaxNumber(4);  	//default is
  // SMP_BONDING_DEVICE_MAX_NUM, can not bigger that this value
  // 												//and
  // this func must call before bls_smp_enableParing bls_smp_enableParing
  // (SMP_PARING_CONN_TRRIGER
  // ); #else bls_smp_enableParing (SMP_PARING_DISABLE_TRRIGER ); #endif

  // HID_service_on_android7p0_init();  //hid device on android 7.0/7.1

  /*-- USER application initialization -------------------------------------*/
  bls_ll_setAdvData((u8 *)tbl_advData, sizeof(tbl_advData));
  bls_ll_setScanRspData((u8 *)tbl_scanRsp, sizeof(tbl_scanRsp));

  /* Configure ADV packet */
  // #if (BLE_REMOTE_SECURITY_ENABLE)
  //   // get bonded device number
  //   u8 bond_number = blc_smp_param_getCurrentBondingDeviceNumber();
  //   smp_param_save_t bondInfo;
  //   if (bond_number)  // at least 1 bonding device exist
  //   {
  //     // get the latest bonding device (index: bond_number-1 )
  //     blc_smp_param_loadByIndex(bond_number - 1, &bondInfo);
  //   }

  //   if (bond_number)  // set direct adv
  //   {
  //     // set direct adv
  //     u8 status =
  //         bls_ll_setAdvParam(MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
  //                            ADV_TYPE_CONNECTABLE_DIRECTED_LOW_DUTY,
  //                            app_own_address_type, bondInfo.peer_addr_type,
  //                            bondInfo.peer_addr, MY_APP_ADV_CHANNEL,
  //                            ADV_FP_NONE);
  //     // debug: ADV setting err
  //     if (status != BLE_SUCCESS) {
  //       write_reg8(0x8000, 0x11);
  //       while (1)
  //         ;
  //     }

  //     // it is recommended that direct adv only last for several seconds,
  //     then
  //     // switch to indirect adv
  //     bls_ll_setAdvDuration(MY_DIRECT_ADV_TMIE, 1);
  //     bls_app_registerEventCallback(BLT_EV_FLAG_ADV_DURATION_TIMEOUT,
  //                                   &app_switch_to_indirect_adv);
  //   } else  // set indirect ADV
  // #endif
  {
    u8 status = bls_ll_setAdvParam(MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
                                   //  ADV_TYPE_CONNECTABLE_UNDIRECTED,
                                   ADV_TYPE_NONCONNECTABLE_UNDIRECTED,
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

  irq_enable();
  while (1) {
    // main_loop();
    blt_sdk_main_loop();
    // blt_pm_proc();

    // gpio_toggle(PIN_LED);
    // sleep_ms(500);
  }

  return 0;
}