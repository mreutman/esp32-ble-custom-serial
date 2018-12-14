/***** Includes *****/

#include <string.h>
#include "ble_server.h"
#include "ble_server_config.h"
#include "freertos/FreeRTOS.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"

/***** Defines *****/

#define FLAG_ADV_CONFIG (1 << 0)
#define FLAG_SCAN_RSP_CONFIG (1 << 1)

#define GATTS_CCCD_NOTIFICATION_ENABLED (0x0001)
#define GATTS_CCCD_INDICATION_ENABLED (0x0002)

#if defined(BLE_SERVER_CONFIG_INDICATE_ENABLE) && \
    defined(BLE_SERVER_CONFIG_NOTIFY_ENABLE)
#error \
"BLE_SERVER_CONFIG_INDICATE_ENABLE and BLE_SERVER_CONFIG_NOTIFY_ENABLE are \
muturally exclusive!"
#endif

/***** Structs *****/

struct gatts_profile_inst {
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;

  // Characteristic
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t char_permissions;
  esp_gatt_char_prop_t char_property;

  // Client Characteristic Configuration Descriptor
  uint16_t descr_handle;
  esp_bt_uuid_t descr_uuid;
  uint16_t descr_value;
  esp_gatt_perm_t desc_permissions;
};

/***** Local Data *****/

// The device name that will be advertised over BLE.
static const char * _device_name =
  BLE_SERVER_CONFIG_DEVICE_NAME;

// The top level service UUID.
static const uint8_t * _service_uuid128 =
  BLE_SERVER_CONFIG_SERVICE_UUID;

// The characteristic UUID of our service.
static const uint8_t * _characteristic_uuid128 =
  BLE_SERVER_CONFIG_CHARACTERISTIC_UUID;

static const uint16_t _descriptor_uuid16 =
  ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

// Number of handles to allocate for the service. In our case we need three,
// one for the characteristic, another for the characteristic descriptor, and
// one more for the actual service itself.
static const uint16_t _service_num_handles = 4;

// The length of adv data must be less than 31 bytes
static esp_ble_adv_data_t adv_data = {
  .set_scan_rsp = false,
  .include_name = true,
  .include_txpower = true,
  .min_interval = 0x20,
  .max_interval = 0x40,
  .appearance = 0x00,
  .manufacturer_len = 0,
  .p_manufacturer_data = NULL,
  .service_data_len = 0,
  .p_service_data = NULL,
//  .service_uuid_len = ESP_UUID_LEN_128,
//  .p_service_uuid = adv__service_uuid128,
  .service_uuid_len = 0,
  .p_service_uuid = 0,
  .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
  .set_scan_rsp = true,
  .include_name = true,
  .include_txpower = true,
  .min_interval = 0x20,
  .max_interval = 0x40,
  .appearance = 0x00,
  .manufacturer_len = 0,
  .p_manufacturer_data = NULL,
  .service_data_len = 0,
  .p_service_data = NULL,
//  .service_uuid_len = ESP_UUID_LEN_128,
//  .p_service_uuid = adv__service_uuid128,
  .service_uuid_len = 0,
  .p_service_uuid = 0,
  .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
  .adv_int_min        = 0x20,
  .adv_int_max        = 0x40,
  .adv_type           = ADV_TYPE_IND,
  .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
  //.peer_addr            =
  //.peer_addr_type       =
  .channel_map        = ADV_CHNL_ALL,
  .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static struct gatts_profile_inst _profile;
static esp_gatt_rsp_t _response;

static uint8_t adv_config_done = 0;
static uint8_t _write_buf[1024];
static uint32_t _write_buf_len = 0;
static uint32_t _write_buf_len_max = 1024;

static ble_write_cb _write_cb = NULL;
static ble_read_cb _read_cb = NULL;
static ble_notify_indicate_cb _notify_indicate_cb = NULL;

/***** Local Functions *****/

static void
gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t * param)
{
  switch (event) {
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~FLAG_ADV_CONFIG);
    if (adv_config_done == 0) {
      esp_ble_gap_start_advertising(&adv_params);
    }
    break;

  case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~FLAG_SCAN_RSP_CONFIG);
    if (adv_config_done == 0) {
      esp_ble_gap_start_advertising(&adv_params);
    }
    break;

  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(__func__, "Advertising start failed\n");
    }
    break;

  case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
    if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(__func__, "Advertising stop failed\n");
    }
    else {
      ESP_LOGI(__func__, "Stop adv successfully\n");
    }
    break;

  case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
    ESP_LOGI(
      __func__,
      "update connetion params status = %d, min_int = %d, "
      "max_int = %d,conn_int = %d,latency = %d, timeout = %d",
      param->update_conn_params.status,
      param->update_conn_params.min_int,
      param->update_conn_params.max_int,
      param->update_conn_params.conn_int,
      param->update_conn_params.latency,
      param->update_conn_params.timeout);
    break;

  default:
    break;
  }
}

static void
gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
  esp_ble_gatts_cb_param_t * param)
{
  esp_gatt_status_t gatt_err;
  esp_err_t err;

  switch (event) {
  case ESP_GATTS_REG_EVT:
    ESP_LOGI(
      __func__,
      "ESP_GATTS_REG_EVT, status %d, app_id %d\n",
      param->reg.status,
      param->reg.app_id);

    _profile.service_id.is_primary = true;
    _profile.service_id.id.inst_id = 0x00;
    _profile.service_id.id.uuid.len = 16;
    memcpy(_profile.service_id.id.uuid.uuid.uuid128, _service_uuid128, 16);

    err = esp_ble_gap_set_device_name(_device_name);
    if (ESP_OK != err) {
      ESP_LOGE(__func__, "%s:%d (%d)", __FILE__, __LINE__, err);
    }

    // config adv data
    err = esp_ble_gap_config_adv_data(&adv_data);
    if (ESP_OK != err) {
      ESP_LOGE(__func__, "%s:%d (%d)", __FILE__, __LINE__, err);
    }

    adv_config_done |= FLAG_ADV_CONFIG;
    // config scan response data
    err = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (err) {
      ESP_LOGE(__func__, "%s:%d (%d)", __FILE__, __LINE__, err);
    }
    adv_config_done |= FLAG_SCAN_RSP_CONFIG;

    err = esp_ble_gatts_create_service(
      gatts_if,
      &_profile.service_id,
      _service_num_handles);
    if (err) {
      ESP_LOGE(__func__, "%s:%d (%d)", __FILE__, __LINE__, err);
    }
    break;

  case ESP_GATTS_CREATE_EVT: {
    esp_attr_control_t control = { .auto_rsp = ESP_GATT_RSP_BY_APP };

    ESP_LOGI(
      __func__,
      "ESP_GATTS_CREATE_EVT, status %d,  service_handle %d\n",
      param->create.status,
      param->create.service_handle);

    _profile.service_handle = param->create.service_handle;
    _profile.char_uuid.len = 16;
    memcpy(_profile.char_uuid.uuid.uuid128, _characteristic_uuid128, 16);

    esp_ble_gatts_start_service(_profile.service_handle);

    err = esp_ble_gatts_add_char(
      _profile.service_handle,
      &_profile.char_uuid,
      _profile.char_permissions,
      _profile.char_property,
      NULL,
      &control);
    if (err) {
      ESP_LOGE(__func__, "add char failed, error code =%x", err);
    }
    break;
  }

  case ESP_GATTS_READ_EVT:
    ESP_LOGI(
      __func__,
      "ESP_GATTS_READ_EVT, conn_id %d, trans_id %d, handle %d\n",
      param->read.conn_id,
      param->read.trans_id,
      param->read.handle);

    memset(&_response, 0, sizeof(esp_gatt_rsp_t));
    _response.attr_value.handle = param->read.handle;
    _response.attr_value.len = 0;

    if (_profile.descr_handle == param->read.handle) {
      gatt_err = ESP_GATT_OK;
      // Little Endian
      _response.attr_value.value[0] = (_profile.descr_value << 0) & 0xFF;
      _response.attr_value.value[1] = (_profile.descr_value << 8) & 0xFF;
      _response.attr_value.len = 2;
    }
    else if (_profile.char_handle == param->read.handle) {
      gatt_err = ESP_GATT_OK;
      if (_read_cb) {
        _read_cb(
          _response.attr_value.value,
          &_response.attr_value.len,
          sizeof(_response.attr_value.value));
      }
    }
    else {
      gatt_err = ESP_GATT_INVALID_HANDLE;
    }

    esp_ble_gatts_send_response(
      gatts_if,
      param->read.conn_id,
      param->read.trans_id,
      gatt_err,
      &_response);
    break;

  case ESP_GATTS_WRITE_EVT:
    ESP_LOGI(
      __func__,
      "ESP_GATTS_WRITE_EVT, conn_id %d, handle %d\n",
      param->write.conn_id,
      param->write.handle);

    gatt_err = ESP_GATT_INVALID_HANDLE;

    if (_profile.char_handle == param->write.handle) {
      gatt_err = ESP_GATT_OK;

      if (param->write.is_prep) {
        if ((param->write.len + _write_buf_len) <= _write_buf_len_max) {
          memcpy(_write_buf, param->write.value, param->write.len);
          _write_buf_len += param->write.len;
        }
      }
      else if (_write_cb) {
        _write_cb(param->write.value, param->write.len);
      }
    }
    else if (_profile.descr_handle == param->write.handle) {
      if (sizeof(_profile.descr_value) == param->write.len) {
        gatt_err = ESP_GATT_OK;
        // Little Endian
        _profile.descr_value  = ((uint16_t) param->write.value[0]) << 0;
        _profile.descr_value |= ((uint16_t) param->write.value[1]) << 8;
      }
      else {
        gatt_err = ESP_GATT_INVALID_CFG;
      }
    }

    if (param->write.need_rsp) {
      esp_ble_gatts_send_response(
        gatts_if,
        param->write.conn_id,
        param->write.trans_id,
        gatt_err,
        NULL);
    }

    if ((_profile.char_handle == param->write.handle) &&
        (ESP_GATT_OK == gatt_err) &&
        (_profile.descr_value & GATTS_CCCD_INDICATION_ENABLED) &&
        (_notify_indicate_cb)) {
      memset(&_response, 0, sizeof(esp_gatt_rsp_t));

      _notify_indicate_cb(
        _response.attr_value.value,
        &_response.attr_value.len,
        sizeof(_response.attr_value.value));

      esp_ble_gatts_send_indicate(
        gatts_if,
        param->write.conn_id,
        param->write.handle,
        _response.attr_value.len,
        _response.attr_value.value,
        true);
    }

    break;

  case ESP_GATTS_EXEC_WRITE_EVT:
    ESP_LOGI(__func__,"ESP_GATTS_EXEC_WRITE_EVT");
    esp_ble_gatts_send_response(
      gatts_if,
      param->write.conn_id,
      param->write.trans_id,
      ESP_GATT_OK,
      NULL);

    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
      // All data has been transfered. Execute on data.
      if ((_write_cb) && (0 != _write_buf_len)) {
        _write_cb(_write_buf, _write_buf_len);
      }
    }

    _write_buf_len = 0;
    break;

  case ESP_GATTS_MTU_EVT:
    ESP_LOGI(__func__, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
    break;

  case ESP_GATTS_UNREG_EVT:
    break;

  case ESP_GATTS_ADD_INCL_SRVC_EVT:
    break;

  case ESP_GATTS_ADD_CHAR_EVT: {
    esp_attr_control_t control = { .auto_rsp = ESP_GATT_RSP_BY_APP };

    ESP_LOGI(
      __func__,
      "ESP_GATTS_ADD_CHAR_EVT, status %d, attr_handle %d, service_handle %d\n",
      param->add_char.status,
      param->add_char.attr_handle,
      param->add_char.service_handle);

    _profile.char_handle = param->add_char.attr_handle;
    _profile.descr_uuid.len = ESP_UUID_LEN_16;
    _profile.descr_uuid.uuid.uuid16 = _descriptor_uuid16;

    err = esp_ble_gatts_add_char_descr(
      _profile.service_handle,
       &_profile.descr_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      NULL,
      &control);
    if (err) {
      ESP_LOGE(__func__, "add char descr failed, error code =%x", err);
    }
    break;
  }

  case ESP_GATTS_ADD_CHAR_DESCR_EVT:
    _profile.descr_handle = param->add_char_descr.attr_handle;
    ESP_LOGI(
      __func__,
      "ESP_GATTS_ADD_CHAR_DESCR_EVT, status %d, attr_handle %d, "
      "service_handle %d\n",
      param->add_char_descr.status,
      param->add_char_descr.attr_handle,
      param->add_char_descr.service_handle);
    break;

  case ESP_GATTS_DELETE_EVT:
    break;

  case ESP_GATTS_START_EVT:
    ESP_LOGI(
      __func__,
      "ESP_GATTS_START_EVT, status %d, service_handle %d\n",
      param->start.status,
      param->start.service_handle);
    break;

  case ESP_GATTS_STOP_EVT:
    break;

  case ESP_GATTS_CONNECT_EVT: {
    esp_ble_conn_update_params_t conn_params = {0};
    memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    /* For the IOS system, please reference the apple official documents
    about the BLE connection parameters restrictions. */
    conn_params.latency = 0;
    conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
    conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
    conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms

    ESP_LOGI(
      __func__,
      "ESP_GATTS_CONNECT_EVT, conn_id %d, "
      "remote %02x:%02x:%02x:%02x:%02x:%02x:",
      param->connect.conn_id,
      param->connect.remote_bda[0],
      param->connect.remote_bda[1],
      param->connect.remote_bda[2],
      param->connect.remote_bda[3],
      param->connect.remote_bda[4],
      param->connect.remote_bda[5]);

    _profile.conn_id = param->connect.conn_id;
    _profile.descr_value = 0x0000;
    //start sent the update connection parameters to the peer device.
    esp_ble_gap_update_conn_params(&conn_params);
    break;
  }

  case ESP_GATTS_DISCONNECT_EVT:
    ESP_LOGI(__func__, "ESP_GATTS_DISCONNECT_EVT");
    esp_ble_gap_start_advertising(&adv_params);
    break;

  case ESP_GATTS_CONF_EVT:
    ESP_LOGI(__func__, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
    
    if (param->conf.status != ESP_GATT_OK) {
      esp_log_buffer_hex(__func__, param->conf.value, param->conf.len);
    }
    break;

  case ESP_GATTS_OPEN_EVT:
  case ESP_GATTS_CANCEL_OPEN_EVT:
  case ESP_GATTS_CLOSE_EVT:
  case ESP_GATTS_LISTEN_EVT:
  case ESP_GATTS_CONGEST_EVT:
  default:
    break;
  }
}

static void
gatts_callback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
  esp_ble_gatts_cb_param_t *param)
{
  /* If event is register event, store the gatts_if for each profile */
  if (event == ESP_GATTS_REG_EVT) {
    if ((param->reg.status == ESP_GATT_OK) &&
        (param->reg.app_id == _profile.app_id)) {
      _profile.gatts_if = gatts_if;
    }
    else {
      ESP_LOGE(
        __func__,
        "Reg app failed, app_id %04x, status %d\n",
        param->reg.app_id,
        param->reg.status);
      return;
    }
  }

  // NOTE: If gatt_if equals ESP_GATT_IF_NONE, call profile handler.
  if ((gatts_if == _profile.gatts_if) || (gatts_if == ESP_GATT_IF_NONE)) {
    gatts_event_handler(event, gatts_if, param);
  }
}

/***** Global Functions *****/

bool
ble_init(void)
{
  static bool run_once = true;
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_err_t err = ESP_OK;

  if (run_once) {
    run_once = false;

    _profile.app_id = 0;
    _profile.gatts_if = ESP_GATT_IF_NONE;

    _profile.char_permissions = ESP_GATT_PERM_WRITE | ESP_GATT_PERM_READ;
    _profile.char_property = 
      ESP_GATT_CHAR_PROP_BIT_WRITE |
      ESP_GATT_CHAR_PROP_BIT_READ |
      #if defined(BLE_SERVER_CONFIG_INDICATE_ENABLE)
      ESP_GATT_CHAR_PROP_BIT_INDICATE;
      #elif defined(BLE_SERVER_CONFIG_NOTIFY_ENABLE)
      ESP_GATT_CHAR_PROP_BIT_NOTIFY;
      #endif

    _profile.desc_permissions = ESP_GATT_PERM_WRITE | ESP_GATT_PERM_READ;

    err = esp_bt_controller_init(&bt_cfg);
    if (err) {
      printf("%s:%d\n", __func__, __LINE__);
      return false;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err) {
      printf("%s:%d\n", __func__, __LINE__);
      return false;
    }

    err = esp_bluedroid_init();
    if (err) {
      printf("%s:%d\n", __func__, __LINE__);
      return false;
    }

    err = esp_bluedroid_enable();
    if (err) {
      printf("%s:%d\n", __func__, __LINE__);
      return false;
    }

    err = esp_ble_gatts_register_callback(gatts_callback);
    if (err) {
      printf("%s:%d\n", __func__, __LINE__);
      return false;
    }

    err = esp_ble_gap_register_callback(gap_event_handler);
    if (err) {
      printf("%s:%d\n", __func__, __LINE__);
      return false;
    }

    err = esp_ble_gatts_app_register(_profile.app_id);
    if (err) {
      printf("%s:%d\n", __func__, __LINE__);
      return false;
    }

    err = esp_ble_gatt_set_local_mtu(500);
    if (err) {
      printf("%s:%d\n", __func__, __LINE__);
      return false;
    }
  }

  return true;
}

void
ble_register_write_callback(ble_write_cb cb)
{
  _write_cb = cb;
}

void
ble_register_read_callback(ble_read_cb cb)
{
  _read_cb = cb;
}

void
ble_register_notify_indicate_callback(ble_notify_indicate_cb cb)
{
  _notify_indicate_cb = cb;
}
