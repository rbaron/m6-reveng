/********************************************************************************************************
 * @file     main.c
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
#include <tl_common.h>

#include "./app_config.h"
#include "app.h"
#include "drivers/5316/driver_5316.h"
#include "stack/ble/ble.h"

_attribute_ram_code_ void irq_handler(void) { irq_blt_sdk_handler(); }

int main(void) {
  blc_pm_select_internal_32k_crystal();
  cpu_wakeup_init();
  clock_init(SYS_CLK_16M_Crystal);

  gpio_init();

  blc_app_loadCustomizedParameters();

  rf_drv_init(RF_MODE_BLE_1M);

  user_init();

  irq_enable();
  while (1) {
    main_loop();
  }
}
