#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_cus.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "time_sync.h"
#include "timers.h"

static void on_connect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
  p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
  ble_cus_evt_t evt;
  evt.evt_type = BLE_CUS_EVT_CONNECTED;
  p_cus->evt_handler(p_cus, &evt);
}

static void on_disconnect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
  UNUSED_PARAMETER(p_ble_evt);
  p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt)
{
  const ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

  // Check if the handle passed with the event matches the Mic Value Characteristic handle.
  if (p_evt_write->handle == p_cus->mic_value_handles.value_handle) {
    NRF_LOG_INFO("Today's Menu: Shio Ramen");
  }

  // Check if the Mic value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
  if ((p_evt_write->handle == p_cus->mic_value_handles.cccd_handle) && (p_evt_write->len == 2)) {
    // CCCD written, call application event handler
    if (p_cus->evt_handler != NULL) {
      ble_cus_evt_t evt;

      if (ble_srv_is_notification_enabled(p_evt_write->data)) {
        evt.evt_type = BLE_CUS_EVT_NOTIFICATION_ENABLED;
      } else {
        evt.evt_type = BLE_CUS_EVT_NOTIFICATION_DISABLED;
      }

      // Call the application event handler.
      p_cus->evt_handler(p_cus, &evt);
    }
  }

  if (p_evt_write->handle == p_cus->time_sync_master_handles.value_handle) {
    if (p_evt_write->data[0] == 0x6D) {
      NRF_LOG_RAW_INFO("%08d [sync] made time sync master\n", systemTimeGetMs());
      ts_tx_start(200);
    } else if (p_evt_write->data[0] == 0x73) {
      NRF_LOG_RAW_INFO("%08d [sync] made time sync slave\n", systemTimeGetMs());
      ts_tx_stop();
    }
  }
}

static uint32_t mic_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init) {
  uint32_t            err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.read   = 0;
  char_md.char_props.write  = 0;
  char_md.char_props.notify = 1; 
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = NULL; 
  char_md.p_sccd_md         = NULL;
  
  memset(&attr_md, 0, sizeof(attr_md));

  attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
  attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 1;
  attr_md.wr_auth    = 1;
  attr_md.vlen       = 1;

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = MIC_CHAR_UUID;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;

  err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                              &attr_char_value,
                                              &p_cus->mic_value_handles);
  if (err_code != NRF_SUCCESS)
  {
      return err_code;
  }

  return NRF_SUCCESS;
}

static uint32_t time_sync_master_char_add(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init) {
  uint32_t            err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.read   = 0;
  char_md.char_props.write_wo_resp  = 1;
  char_md.char_props.notify = 0; 
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = NULL; 
  char_md.p_sccd_md         = NULL;
  
  memset(&attr_md, 0, sizeof(attr_md));

  attr_md.read_perm  = p_cus_init->custom_value_char_attr_md.read_perm;
  attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 1;
  attr_md.wr_auth    = 1;
  attr_md.vlen       = 0;

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = TIME_SYNC_MASTER_CHAR_UUID;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = sizeof(uint8_t);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = sizeof(uint8_t);

  err_code = sd_ble_gatts_characteristic_add(p_cus->service_handle, &char_md,
                                              &attr_char_value,
                                              &p_cus->time_sync_master_handles);
  if (err_code != NRF_SUCCESS)
  {
      return err_code;
  }

  return NRF_SUCCESS;
}

void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
  ble_cus_t * p_cus = (ble_cus_t *) p_context;

  if (p_cus == NULL || p_ble_evt == NULL) { return; }

  switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
      on_connect(p_cus, p_ble_evt);
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      on_disconnect(p_cus, p_ble_evt);
      break;

    case BLE_GATTS_EVT_WRITE:
      on_write(p_cus, p_ble_evt);
      break;

    default:
      // No implementation needed.
      break;
  }
}

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2
void ble_cus_on_gatt_evt(ble_cus_t * p_cus, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
  if (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED) {
    p_cus->max_payload_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
  }
}

bool ble_cus_transmit(ble_cus_t * p_cus, uint8_t * data, uint16_t length)
{
  uint16_t payload_len = p_cus->max_payload_len;
  ble_cus_evt_t evt;

  if (p_cus == NULL) { return NRF_ERROR_NULL; }
  if (length > payload_len) { NRF_LOG_INFO("length > max(%d), truncating", p_cus->max_payload_len); }
  else { payload_len = length; }

  ble_gatts_hvx_params_t const hvx_param =
  {
    .handle = p_cus->mic_value_handles.value_handle,
    .p_data = data,
    .p_len  = &payload_len,
    .type   = BLE_GATT_HVX_NOTIFICATION,
  };

  uint32_t err_code = NRF_SUCCESS;

  err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_param);

  if (err_code == NRF_ERROR_RESOURCES) {
    return false; // return busy flag
  } else if (err_code != NRF_SUCCESS) {
    NRF_LOG_ERROR("sd_ble_gatts_hvx() failed: 0x%x", err_code);
    return false;
  }

  p_cus->bytes_sent += payload_len;

  if (p_cus->kbytes_sent != (p_cus->bytes_sent / 1024)) {
    p_cus->kbytes_sent = (p_cus->bytes_sent / 1024);
    evt.evt_type             = BLE_CUS_EVT_TRANSFER_1KB;
    evt.bytes_transfered_cnt = p_cus->bytes_sent;
    p_cus->evt_handler(p_cus, &evt); // call the ble_manager event handler
  }

  return true;
}

uint32_t ble_cus_init(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init)
{
  if (p_cus == NULL || p_cus_init == NULL) { return NRF_ERROR_NULL; }

  uint32_t   err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE}; // Add Custom Service UUID

  // Initialize service structure
  p_cus->evt_handler               = p_cus_init->evt_handler;
  p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

  err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = CUSTOM_SERVICE_UUID;

  // Add the Custom Service
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
  VERIFY_SUCCESS(err_code);

  // Add Mic Characteristic
  mic_char_add(p_cus, p_cus_init);

  // Add Time Sync Master Characteristic
  time_sync_master_char_add(p_cus, p_cus_init);

  return NRF_SUCCESS;
}
