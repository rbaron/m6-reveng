/********************************************************************************************************
 * @file     app_att.c
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

#include "./app_config.h"
// Must be included after app_config.h.
#include <stack/ble/ble.h>

typedef struct {
  /** Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25
   * ms) */
  u16 intervalMin;
  /** Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25
   * ms) */
  u16 intervalMax;
  /** Number of LL latency connection events (0x0000 - 0x03e8) */
  u16 latency;
  /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
  u16 timeout;
} gap_periConnectParams_t;

const u16 clientCharacterCfgUUID = GATT_UUID_CLIENT_CHAR_CFG;

const u16 extReportRefUUID = GATT_UUID_EXT_REPORT_REF;

const u16 reportRefUUID = GATT_UUID_REPORT_REF;

const u16 characterPresentFormatUUID = GATT_UUID_CHAR_PRESENT_FORMAT;

const u16 my_primaryServiceUUID = GATT_UUID_PRIMARY_SERVICE;

static const u16 my_characterUUID = GATT_UUID_CHARACTER;

const u16 my_devServiceUUID = SERVICE_UUID_DEVICE_INFORMATION;

const u16 my_PnPUUID = CHARACTERISTIC_UUID_PNP_ID;

const u16 my_devNameUUID = GATT_UUID_DEVICE_NAME;

// Device information service UUID.
const u16 my_gapServiceUUID = SERVICE_UUID_GENERIC_ACCESS;

// Appearance characteristic.
const u16 my_appearanceUIID = 0x2a01;
const u16 my_periConnParamUUID = 0x2a04;
u16 my_appearance = GAP_APPEARE_UNKNOWN;
gap_periConnectParams_t my_periConnParameters = {20, 40, 0, 1000};

const u16 my_gattServiceUUID = SERVICE_UUID_GENERIC_ATTRIBUTE;
const u16 serviceChangeUIID = GATT_UUID_SERVICE_CHANGE;
u16 serviceChangeVal[2] = {0};
static u8 serviceChangeCCC[2] = {0, 0};

const u8 my_devName[] = {'r', 'b', 'a', 'r', 'o', 'n'};

const u8 my_PnPtrs[] = {0x02, 0x8a, 0x24, 0x66, 0x82, 0x01, 0x00};

// Battery service.
const u16 my_batServiceUUID = SERVICE_UUID_BATTERY;
const u16 my_batCharUUID = CHARACTERISTIC_UUID_BATTERY_LEVEL;
static u8 batteryValueInCCC[2];
u8 my_batVal[1] = {99};

// Include attribute (Battery service)
static u16 include[3] = {BATT_PS_H, BATT_LEVEL_INPUT_CCB_H,
                         SERVICE_UUID_BATTERY};

static const u8 my_devNameCharVal[5] = {
    CHAR_PROP_READ | CHAR_PROP_NOTIFY, U16_LO(GenericAccess_DeviceName_DP_H),
    U16_HI(GenericAccess_DeviceName_DP_H), U16_LO(GATT_UUID_DEVICE_NAME),
    U16_HI(GATT_UUID_DEVICE_NAME)};
static const u8 my_appearanceCharVal[5] = {
    CHAR_PROP_READ, U16_LO(GenericAccess_Appearance_DP_H),
    U16_HI(GenericAccess_Appearance_DP_H), U16_LO(GATT_UUID_APPEARANCE),
    U16_HI(GATT_UUID_APPEARANCE)};
static const u8 my_periConnParamCharVal[5] = {
    CHAR_PROP_READ, U16_LO(CONN_PARAM_DP_H), U16_HI(CONN_PARAM_DP_H),
    U16_LO(GATT_UUID_PERI_CONN_PARAM), U16_HI(GATT_UUID_PERI_CONN_PARAM)};

//// GATT attribute values
static const u8 my_serviceChangeCharVal[5] = {
    CHAR_PROP_INDICATE, U16_LO(GenericAttribute_ServiceChanged_DP_H),
    U16_HI(GenericAttribute_ServiceChanged_DP_H),
    U16_LO(GATT_UUID_SERVICE_CHANGE), U16_HI(GATT_UUID_SERVICE_CHANGE)};

//// device Information  attribute values
static const u8 my_PnCharVal[5] = {
    CHAR_PROP_READ, U16_LO(DeviceInformation_pnpID_DP_H),
    U16_HI(DeviceInformation_pnpID_DP_H), U16_LO(CHARACTERISTIC_UUID_PNP_ID),
    U16_HI(CHARACTERISTIC_UUID_PNP_ID)};

//// Battery attribute values
static const u8 my_batCharVal[5] = {
    CHAR_PROP_READ | CHAR_PROP_NOTIFY, U16_LO(BATT_LEVEL_INPUT_DP_H),
    U16_HI(BATT_LEVEL_INPUT_DP_H), U16_LO(CHARACTERISTIC_UUID_BATTERY_LEVEL),
    U16_HI(CHARACTERISTIC_UUID_BATTERY_LEVEL)};

int cb(void *p) {
  gpio_toggle(GPIO_PB4);
  return 0;
}

const attribute_t my_Attributes[] = {

    {ATT_END_H - 1, 0, 0, 0, 0, 0},  // total num of attribute

    // 0001 - 0007  gap
    {7, ATT_PERMISSIONS_READ, 2, 2, (u8 *)(&my_primaryServiceUUID),
     (u8 *)(&my_gapServiceUUID), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_devNameCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_devNameCharVal), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_devName), (u8 *)(&my_devNameUUID),
     (u8 *)(my_devName), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_appearanceCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_appearanceCharVal), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_appearance),
     (u8 *)(&my_appearanceUIID), (u8 *)(&my_appearance), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_periConnParamCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_periConnParamCharVal), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_periConnParameters),
     (u8 *)(&my_periConnParamUUID), (u8 *)(&my_periConnParameters), 0},

    // 0008 - 000b gatt
    {4, ATT_PERMISSIONS_READ, 2, 2, (u8 *)(&my_primaryServiceUUID),
     (u8 *)(&my_gattServiceUUID), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_serviceChangeCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_serviceChangeCharVal), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(serviceChangeVal),
     (u8 *)(&serviceChangeUIID), (u8 *)(&serviceChangeVal), 0},
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(serviceChangeCCC),
     (u8 *)(&clientCharacterCfgUUID), (u8 *)(serviceChangeCCC), 0},

    // Battery service.
    // Service UUID: 0x180f

    // my_primaryServiceUUID: 0x2800 - Service declaration.
    {4, ATT_PERMISSIONS_READ, 2, 2, (u8 *)(&my_primaryServiceUUID),
     (u8 *)(&my_batServiceUUID), 0},
    // Characteristic UUID: 0x2803
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_batCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_batCharVal), 0},
    // batChar UUID: 0x2a19
    // {0, ATT_PERMISSIONS_READ, 2, sizeof(my_batVal), (u8 *)(&my_batCharUUID),
    //  (u8 *)(my_batVal), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_batVal), (u8 *)(&my_batCharUUID),
     (u8 *)(my_batVal), &cb, &cb},
    // clientCharacterCfgUUID: 0x2902
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(batteryValueInCCC),
     (u8 *)(&clientCharacterCfgUUID), (u8 *)(batteryValueInCCC), 0},  // value
};

void my_att_init(void) { bls_att_setAttributeTable((u8 *)my_Attributes); }
