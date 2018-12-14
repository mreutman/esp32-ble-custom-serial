#ifndef _BLE_SERVER_H
#define _BLE_SERVER_H

/***** Includes *****/

#include <stdint.h>
#include <stdbool.h>

/***** Typedefs *****/

typedef void
(*ble_write_cb)(uint8_t * buf, uint16_t len);

typedef void
(*ble_read_cb)(uint8_t * buf, uint16_t * len, uint16_t len_max);

typedef void
(*ble_notify_indicate_cb)(uint8_t * buf, uint16_t * len, uint16_t len_max);

/***** Global Functions *****/

extern bool
ble_init(void);

extern void
ble_register_write_callback(ble_write_cb cb);

extern void
ble_register_read_callback(ble_read_cb cb);

extern void
ble_register_notify_indicate_callback(ble_notify_indicate_cb cb);

#endif
