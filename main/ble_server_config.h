#ifndef _BLE_SERVER_CONFIG_H
#define _BLE_SERVER_CONFIG_H

/***** Includes *****/

#include <stdint.h>

/***** Defines *****/

#define BLE_SERVER_CONFIG_DEVICE_NAME "ESP32"

// A uint8_t array representing the 128 bit UUID to identify the service. It
// is recommended to use the "generate-uuid" uitility to create your own UUIDs
// and place them into the ble_server.c source file.
#define BLE_SERVER_CONFIG_SERVICE_UUID (ble_service_uuid128)

// A uint8_t array representing the 128 bit UUID to identify the characteristic.
// It is recommended to use the "generate-uuid" uitility to create your own
// UUIDs and place them into the ble_server.c source file.
#define BLE_SERVER_CONFIG_CHARACTERISTIC_UUID (ble_characteristic_uuid128)

// Mutually exclusive option to enable either Indicate or Notify. Alternatively,
// comment out both to only support client initiated reads and writes.
#define BLE_SERVER_CONFIG_INDICATE_ENABLE 1
//#define BLE_SERVER_CONFIG_NOTIFY_ENABLE 1

/***** Global Data *****/

// Defined in ble_server_config.c source file.
extern uint8_t ble_service_uuid128[16];

// Defined in ble_server_config.c source file.
extern uint8_t ble_characteristic_uuid128[16];

#endif
