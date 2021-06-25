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

// #include "tl_common.h"
#include "common/string.h"
#include "stack/ble/ble.h"

#if (__PROJECT_5316_BLE_SAMPLE__)

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

// device information
const u16 my_gapServiceUUID = SERVICE_UUID_GENERIC_ACCESS;
// Appearance Characteristic Properties
const u16 my_appearanceUIID = 0x2a01;
const u16 my_periConnParamUUID = 0x2a04;
u16 my_appearance = GAP_APPEARE_UNKNOWN;
gap_periConnectParams_t my_periConnParameters = {20, 40, 0, 1000};

const u16 my_gattServiceUUID = SERVICE_UUID_GENERIC_ATTRIBUTE;
const u16 serviceChangeUIID = GATT_UUID_SERVICE_CHANGE;
u16 serviceChangeVal[2] = {0};
static u8 serviceChangeCCC[2] = {0, 0};

const u8 my_devName[] = {'G', 'R', 'e', 'm', 'o', 't', 'e'};

const u8 my_PnPtrs[] = {0x02, 0x8a, 0x24, 0x66, 0x82, 0x01, 0x00};

//////////////////////// Battery
////////////////////////////////////////////////////
const u16 my_batServiceUUID = SERVICE_UUID_BATTERY;
const u16 my_batCharUUID = CHARACTERISTIC_UUID_BATTERY_LEVEL;
static u8 batteryValueInCCC[2];
u8 my_batVal[1] = {99};

//////////////////////// HID
////////////////////////////////////////////////////////

const u16 my_hidServiceUUID = SERVICE_UUID_HUMAN_INTERFACE_DEVICE;

const u16 hidServiceUUID = SERVICE_UUID_HUMAN_INTERFACE_DEVICE;
const u16 hidProtocolModeUUID = CHARACTERISTIC_UUID_HID_PROTOCOL_MODE;
const u16 hidReportUUID = CHARACTERISTIC_UUID_HID_REPORT;
const u16 hidReportMapUUID = CHARACTERISTIC_UUID_HID_REPORT_MAP;
const u16 hidbootKeyInReportUUID = CHARACTERISTIC_UUID_HID_BOOT_KEY_INPUT;
const u16 hidbootKeyOutReportUUID = CHARACTERISTIC_UUID_HID_BOOT_KEY_OUTPUT;
const u16 hidbootMouseInReportUUID = CHARACTERISTIC_UUID_HID_BOOT_MOUSE_INPUT;
const u16 hidinformationUUID = CHARACTERISTIC_UUID_HID_INFORMATION;
const u16 hidCtrlPointUUID = CHARACTERISTIC_UUID_HID_CONTROL_POINT;
const u16 hidIncludeUUID = GATT_UUID_INCLUDE;

u8 protocolMode = DFLT_HID_PROTOCOL_MODE;

// Key in Report characteristic variables
u8 reportKeyIn[8];
u8 reportKeyInCCC[2];
// HID Report Reference characteristic descriptor, key input
static u8 reportRefKeyIn[2] = {HID_REPORT_ID_KEYBOARD_INPUT,
                               HID_REPORT_TYPE_INPUT};

// Key out Report characteristic variables
u8 reportKeyOut[1];
u8 reportKeyOutCCC[2];
static u8 reportRefKeyOut[2] = {HID_REPORT_ID_KEYBOARD_INPUT,
                                HID_REPORT_TYPE_OUTPUT};

// Consumer Control input Report
static u8 reportConsumerControlIn[2];
static u8 reportConsumerControlInCCC[2];
static u8 reportRefConsumerControlIn[2] = {HID_REPORT_ID_CONSUME_CONTROL_INPUT,
                                           HID_REPORT_TYPE_INPUT};

// Boot Keyboard Input Report
static u8 bootKeyInReport;
static u8 bootKeyInReportCCC[2];

// Boot Keyboard Output Report
static u8 bootKeyOutReport;

// HID Information characteristic
const u8 hidInformation[] = {
    U16_LO(0x0111), U16_HI(0x0111),  // bcdHID (USB HID version)
    0x00,                            // bCountryCode
    0x01                             // Flags
};

// HID Control Point characteristic
static u8 controlPoint;

// HID Report Map characteristic
// Keyboard report descriptor (using format for Boot interface descriptor)

static const u8 reportMap[] = {
    // keyboard report in
    0x05, 0x01,                          // Usage Pg (Generic Desktop)
    0x09, 0x06,                          // Usage (Keyboard)
    0xA1, 0x01,                          // Collection: (Application)
    0x85, HID_REPORT_ID_KEYBOARD_INPUT,  // Report Id (keyboard)
                                         //
    0x05, 0x07,                          // Usage Pg (Key Codes)
    0x19, 0xE0,                          // Usage Min (224)  VK_CTRL:0xe0
    0x29, 0xE7,                          // Usage Max (231)  VK_RWIN:0xe7
    0x15, 0x00,                          // Log Min (0)
    0x25, 0x01,                          // Log Max (1)
                                         //
                                         // Modifier byte
    0x75, 0x01,                          // Report Size (1)   1 bit * 8
    0x95, 0x08,                          // Report Count (8)
    0x81, 0x02,                          // Input: (Data, Variable, Absolute)
                                         //
                                         // Reserved byte
    0x95, 0x01,                          // Report Count (1)
    0x75, 0x08,                          // Report Size (8)
    0x81, 0x01,                          // Input: (Constant)

    // keyboard output
    // 5 bit led ctrl: NumLock CapsLock ScrollLock Compose kana
    0x95, 0x05,  // Report Count (5)
    0x75, 0x01,  // Report Size (1)
    0x05, 0x08,  // Usage Pg (LEDs )
    0x19, 0x01,  // Usage Min
    0x29, 0x05,  // Usage Max
    0x91, 0x02,  // Output (Data, Variable, Absolute)
                 // 3 bit reserved
    0x95, 0x01,  // Report Count (1)
    0x75, 0x03,  // Report Size (3)
    0x91, 0x01,  // Output (Constant)

    // Key arrays (6 bytes)
    0x95, 0x06,  // Report Count (6)
    0x75, 0x08,  // Report Size (8)
    0x15, 0x00,  // Log Min (0)
    0x25, 0xF1,  // Log Max (241)
    0x05, 0x07,  // Usage Pg (Key Codes)
    0x19, 0x00,  // Usage Min (0)
    0x29, 0xf1,  // Usage Max (241)
    0x81, 0x00,  // Input: (Data, Array)

    0xC0,  // End Collection

    // consumer report in
    0x05, 0x0C,                                 // Usage Page (Consumer)
    0x09, 0x01,                                 // Usage (Consumer Control)
    0xA1, 0x01,                                 // Collection (Application)
    0x85, HID_REPORT_ID_CONSUME_CONTROL_INPUT,  //     Report Id
    0x75, 0x10,                                 // global, report size 16 bits
    0x95, 0x01,                                 // global, report count 1
    0x15, 0x01,                                 // global, min  0x01
    0x26, 0x8c, 0x02,                           // global, max  0x28c
    0x19, 0x01,                                 // local, min   0x01
    0x2a, 0x8c, 0x02,                           // local, max    0x28c
    0x81, 0x00,  // main,  input data varible, absolute
    0xc0,        // main, end collection

};

// HID External Report Reference Descriptor for report map
static u16 extServiceUUID;

/////////////////////////////////////////////////////////
const u8 my_OtaUUID[16] = TELINK_SPP_DATA_OTA;
const u8 my_OtaServiceUUID[16] = TELINK_OTA_UUID_SERVICE;
const u16 userdesc_UUID = GATT_UUID_CHAR_USER_DESC;

u8 my_OtaData = 0x00;

const u8 my_OtaName[] = {'O', 'T', 'A'};

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

//// HID attribute values
static const u8 my_hidProtocolModeCharVal[5] = {
    CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP,
    U16_LO(HID_PROTOCOL_MODE_DP_H), U16_HI(HID_PROTOCOL_MODE_DP_H),
    U16_LO(CHARACTERISTIC_UUID_HID_PROTOCOL_MODE),
    U16_HI(CHARACTERISTIC_UUID_HID_PROTOCOL_MODE)};
static const u8 my_hidbootKeyInReporCharVal[5] = {
    CHAR_PROP_READ | CHAR_PROP_NOTIFY, U16_LO(HID_BOOT_KB_REPORT_INPUT_DP_H),
    U16_HI(HID_BOOT_KB_REPORT_INPUT_DP_H),
    U16_LO(CHARACTERISTIC_UUID_HID_BOOT_KEY_INPUT),
    U16_HI(CHARACTERISTIC_UUID_HID_BOOT_KEY_INPUT)};
static const u8 my_hidbootKeyOutReporCharVal[5] = {
    CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RSP,
    U16_LO(HID_BOOT_KB_REPORT_OUTPUT_DP_H),
    U16_HI(HID_BOOT_KB_REPORT_OUTPUT_DP_H),
    U16_LO(CHARACTERISTIC_UUID_HID_BOOT_KEY_OUTPUT),
    U16_HI(CHARACTERISTIC_UUID_HID_BOOT_KEY_OUTPUT)};
static const u8 my_hidReportCCinCharVal[5] = {
    CHAR_PROP_READ | CHAR_PROP_NOTIFY, U16_LO(HID_CONSUME_REPORT_INPUT_DP_H),
    U16_HI(HID_CONSUME_REPORT_INPUT_DP_H),
    U16_LO(CHARACTERISTIC_UUID_HID_REPORT),
    U16_HI(CHARACTERISTIC_UUID_HID_REPORT)};
static const u8 my_hidReportKEYinCharVal[5] = {
    CHAR_PROP_READ | CHAR_PROP_NOTIFY, U16_LO(HID_NORMAL_KB_REPORT_INPUT_DP_H),
    U16_HI(HID_NORMAL_KB_REPORT_INPUT_DP_H),
    U16_LO(CHARACTERISTIC_UUID_HID_REPORT),
    U16_HI(CHARACTERISTIC_UUID_HID_REPORT)};
static const u8 my_hidReportKEYoutCharVal[5] = {
    CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RSP,
    U16_LO(HID_NORMAL_KB_REPORT_OUTPUT_DP_H),
    U16_HI(HID_NORMAL_KB_REPORT_OUTPUT_DP_H),
    U16_LO(CHARACTERISTIC_UUID_HID_REPORT),
    U16_HI(CHARACTERISTIC_UUID_HID_REPORT)};
static const u8 my_hidReportMapCharVal[5] = {
    CHAR_PROP_READ, U16_LO(HID_REPORT_MAP_DP_H), U16_HI(HID_REPORT_MAP_DP_H),
    U16_LO(CHARACTERISTIC_UUID_HID_REPORT_MAP),
    U16_HI(CHARACTERISTIC_UUID_HID_REPORT_MAP)};
static const u8 my_hidinformationCharVal[5] = {
    CHAR_PROP_READ, U16_LO(HID_INFORMATION_DP_H), U16_HI(HID_INFORMATION_DP_H),
    U16_LO(CHARACTERISTIC_UUID_HID_INFORMATION),
    U16_HI(CHARACTERISTIC_UUID_HID_INFORMATION)};
static const u8 my_hidCtrlPointCharVal[5] = {
    CHAR_PROP_WRITE_WITHOUT_RSP, U16_LO(HID_CONTROL_POINT_DP_H),
    U16_HI(HID_CONTROL_POINT_DP_H),
    U16_LO(CHARACTERISTIC_UUID_HID_CONTROL_POINT),
    U16_HI(CHARACTERISTIC_UUID_HID_CONTROL_POINT)};

//// Battery attribute values
static const u8 my_batCharVal[5] = {
    CHAR_PROP_READ | CHAR_PROP_NOTIFY, U16_LO(BATT_LEVEL_INPUT_DP_H),
    U16_HI(BATT_LEVEL_INPUT_DP_H), U16_LO(CHARACTERISTIC_UUID_BATTERY_LEVEL),
    U16_HI(CHARACTERISTIC_UUID_BATTERY_LEVEL)};

//// OTA attribute values
static const u8 my_OtaCharVal[19] = {
    CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP,
    U16_LO(OTA_CMD_OUT_DP_H),
    U16_HI(OTA_CMD_OUT_DP_H),
    TELINK_SPP_DATA_OTA,
};

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

    // 000c - 000e  device Information Service
    {3, ATT_PERMISSIONS_READ, 2, 2, (u8 *)(&my_primaryServiceUUID),
     (u8 *)(&my_devServiceUUID), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_PnCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_PnCharVal), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_PnPtrs), (u8 *)(&my_PnPUUID),
     (u8 *)(my_PnPtrs), 0},

    /////////////////////////////////// 4. HID Service
    ////////////////////////////////////////////////////////////
    // 000f
    //{27, ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID),
    //(u8*)(&my_hidServiceUUID), 0},
    {HID_CONTROL_POINT_DP_H - HID_PS_H + 1, ATT_PERMISSIONS_READ, 2, 2,
     (u8 *)(&my_primaryServiceUUID), (u8 *)(&my_hidServiceUUID), 0},

    // 0010  include battery service
    {0, ATT_PERMISSIONS_READ, 2, sizeof(include), (u8 *)(&hidIncludeUUID),
     (u8 *)(include), 0},

    // 0011 - 0012  protocol mode
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_hidProtocolModeCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_hidProtocolModeCharVal), 0},  // prop
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(protocolMode),
     (u8 *)(&hidProtocolModeUUID), (u8 *)(&protocolMode), 0},  // value

    // 0013 - 0015  boot keyboard input report (char-val-client)
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_hidbootKeyInReporCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_hidbootKeyInReporCharVal),
     0},  // prop
    {0, ATT_PERMISSIONS_READ, 2, sizeof(bootKeyInReport),
     (u8 *)(&hidbootKeyInReportUUID), (u8 *)(&bootKeyInReport), 0},  // value
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(bootKeyInReportCCC),
     (u8 *)(&clientCharacterCfgUUID), (u8 *)(bootKeyInReportCCC), 0},  // value

    // 0016 - 0017   boot keyboard output report (char-val)
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_hidbootKeyOutReporCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_hidbootKeyOutReporCharVal),
     0},  // prop
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(bootKeyOutReport),
     (u8 *)(&hidbootKeyOutReportUUID), (u8 *)(&bootKeyOutReport), 0},  // value

    // 0018 - 001b. consume report in: 4 (char-val-client-ref)
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_hidReportCCinCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_hidReportCCinCharVal), 0},  // prop
    {0, ATT_PERMISSIONS_READ, 2, sizeof(reportConsumerControlIn),
     (u8 *)(&hidReportUUID), (u8 *)(reportConsumerControlIn), 0},  // value
    {0, ATT_PERMISSIONS_RDWR | ATT_PERMISSIONS_AUTHEN_WRITE, 2,
     sizeof(reportConsumerControlInCCC), (u8 *)(&clientCharacterCfgUUID),
     (u8 *)(reportConsumerControlInCCC), 0},  // value
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(reportRefConsumerControlIn),
     (u8 *)(&reportRefUUID), (u8 *)(reportRefConsumerControlIn), 0},  // value

    // 001c - 001f . keyboard report in : 4 (char-val-client-ref)
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_hidReportKEYinCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_hidReportKEYinCharVal), 0},  // prop
    {0, ATT_PERMISSIONS_READ, 2, sizeof(reportKeyIn), (u8 *)(&hidReportUUID),
     (u8 *)(reportKeyIn), 0},  // value
    {0, ATT_PERMISSIONS_RDWR | ATT_PERMISSIONS_AUTHEN_WRITE, 2,
     sizeof(reportKeyInCCC), (u8 *)(&clientCharacterCfgUUID),
     (u8 *)(reportKeyInCCC), 0},  // value
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(reportRefKeyIn), (u8 *)(&reportRefUUID),
     (u8 *)(reportRefKeyIn), 0},  // value

    // 0020 - 0022 . keyboard report out: 3 (char-val-ref)
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_hidReportKEYoutCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_hidReportKEYoutCharVal), 0},  // prop
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(reportKeyOut), (u8 *)(&hidReportUUID),
     (u8 *)(reportKeyOut), 0},  // value
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(reportRefKeyOut),
     (u8 *)(&reportRefUUID), (u8 *)(reportRefKeyOut), 0},  // value

    // 0023 - 0025 . report map: 3
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_hidReportMapCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_hidReportMapCharVal), 0},  // prop
    {0, ATT_PERMISSIONS_READ, 2, sizeof(reportMap), (u8 *)(&hidReportMapUUID),
     (u8 *)(reportMap), 0},  // value
    {0, ATT_PERMISSIONS_READ | ATT_PERMISSIONS_WRITE, 2, sizeof(extServiceUUID),
     (u8 *)(&extReportRefUUID), (u8 *)(&extServiceUUID), 0},  // value

    // 0026 - 0027 . hid information: 2
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_hidinformationCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_hidinformationCharVal), 0},  // prop
    {0, ATT_PERMISSIONS_READ, 2, sizeof(hidInformation),
     (u8 *)(&hidinformationUUID), (u8 *)(hidInformation), 0},  // value

    // 0028 - 0029 . control point: 2
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_hidCtrlPointCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_hidCtrlPointCharVal), 0},  // prop
    {0, ATT_PERMISSIONS_WRITE, 2, sizeof(controlPoint),
     (u8 *)(&hidCtrlPointUUID), (u8 *)(&controlPoint), 0},  // value

    ////////////////////////////////////// Battery Service
    ////////////////////////////////////////////////////////
    // 002a - 002d
    {4, ATT_PERMISSIONS_READ, 2, 2, (u8 *)(&my_primaryServiceUUID),
     (u8 *)(&my_batServiceUUID), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_batCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_batCharVal), 0},  // prop
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_batVal), (u8 *)(&my_batCharUUID),
     (u8 *)(my_batVal), 0},  // value
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(batteryValueInCCC),
     (u8 *)(&clientCharacterCfgUUID), (u8 *)(batteryValueInCCC), 0},  // value

    ////////////////////////////////////// OTA
    ////////////////////////////////////////////////////////
    // 002e - 0031
    {4, ATT_PERMISSIONS_READ, 2, 16, (u8 *)(&my_primaryServiceUUID),
     (u8 *)(&my_OtaServiceUUID), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_OtaCharVal),
     (u8 *)(&my_characterUUID), (u8 *)(my_OtaCharVal), 0},  // prop
    {0, ATT_PERMISSIONS_RDWR, 16, sizeof(my_OtaData), (u8 *)(&my_OtaUUID),
     (&my_OtaData), &otaWrite, &otaRead},  // value
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_OtaName), (u8 *)(&userdesc_UUID),
     (u8 *)(my_OtaName), 0},

};

void my_att_init(void) { bls_att_setAttributeTable((u8 *)my_Attributes); }

#endif  // end of __PROJECT_5316_BLE_SAMPLE__
