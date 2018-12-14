/***** Includes *****/

#include "esp_log.h"
#include "nvs_flash.h"
#include "ble_server.h"

/***** Local Functions *****/

void
_write_callback(uint8_t * buf, uint16_t len)
{
  ESP_LOGI(__func__, "buf=%p, len=%d\n", buf, len);
  ESP_LOG_BUFFER_HEX(__func__, buf, len);
}

void
_read_callback(uint8_t * buf, uint16_t * len, uint16_t max_len)
{
  ESP_LOGI(__func__, "buf=%p, len=%p, max_len=%d\n", buf, len, max_len);
  buf[0] = 'w';
  buf[1] = 'o';
  buf[2] = 'r';
  buf[3] = 'l';
  buf[4] = 'd';
  *len = 5;
}

void
_notify_indicate_callback(uint8_t * buf, uint16_t * len, uint16_t max_len)
{
  ESP_LOGI(__func__, "buf=%p, len=%p, max_len=%d\n", buf, len, max_len);
  buf[0] = 'h';
  buf[1] = 'e';
  buf[2] = 'l';
  buf[3] = 'l';
  buf[4] = 'o';
  *len = 5;
}

/***** Global Functions *****/

void
app_main()
{
  bool r = true;

  nvs_flash_init();

  r = ble_init();
  if (!r) {
    ESP_LOGE(__func__, "ble_init() failed!\n");
  }

  ble_register_write_callback(_write_callback);
  ble_register_read_callback(_read_callback);
  ble_register_notify_indicate_callback(_notify_indicate_callback);
}
