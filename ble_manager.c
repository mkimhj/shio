#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"

#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "main.h"
#include "ble_manager.h"

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
  uint32_t data_length;
  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
  {
    data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    //m_ble_params_info.mtu = m_ble_mic_service_max_data_len;

    NRF_LOG_INFO("gatt_event: ATT MTU is set to 0x%X (%d)", data_length, data_length);
  } else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED)) {
    data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
    // m_ble_mic_service_max_data_len = data_length;
    // m_ble_params_info.mtu = m_ble_mic_service_max_data_len;
    // ble_mic_service_ble_params_info_send(&m_mic_service, &m_ble_params_info);
    NRF_LOG_INFO("gatt_event: Data len is set to 0x%X (%d)", data_length, data_length);
  }
  //  NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
  //  p_gatt->att_mtu_desired_central,
  //  p_gatt->att_mtu_desired_periph);
}

void gatt_init(void)
{
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


#define BLE_UUID_MIC_TX_CHARACTERISTIC 0x0003                      /**< The UUID of the TX Characteristic. */
#define BLE_UUID_MIC_RX_CHARACTERISTIC 0x0002                      /**< The UUID of the RX Characteristic. */
#define BLE_MIC_MAX_RX_CHAR_LEN        BLE_MIC_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_MIC_MAX_TX_CHAR_LEN        BLE_MIC_MAX_DATA_LEN        /**< Maximum length of the TX Characteristic (in bytes). */


typedef enum {
  APP_CMD_NOCOMMAND = 0,
  APP_CMD_SINGLE_CAPTURE,
  APP_CMD_START_STREAM,
  APP_CMD_STOP_STREAM,
  APP_CMD_CHANGE_PHY,
  APP_CMD_SEND_BLE_PARAMS
} ble_mic_service_command_t;

static bool m_stream_mode_active = false;
static uint8_t m_new_command_received = 0;
static uint8_t m_new_phy;

static void mic_service_data_handler(ble_mic_service_t * p_mic_service, uint8_t const * p_data, uint16_t length)
{
  switch(p_data[0]) {
    case APP_CMD_SINGLE_CAPTURE:
    case APP_CMD_SEND_BLE_PARAMS:
      m_new_command_received = p_data[0];
      break;

    case APP_CMD_START_STREAM:
      m_stream_mode_active = true;
      m_new_command_received = p_data[0];
      break;

    case APP_CMD_STOP_STREAM:
      m_stream_mode_active = false;
      m_new_command_received = p_data[0];
      break;

    case APP_CMD_CHANGE_PHY:
      m_new_phy = p_data[1];
      m_new_command_received = p_data[0];
      break;

    default:
      NRF_LOG_ERROR("Unknown command!");
      break;
  }
}

void ble_mic_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
  if ((p_context == NULL) || (p_ble_evt == NULL)) {
    return;
  }

  ble_mic_service_t * p_mic_service = (ble_mic_service_t*)p_context;

  switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
      NRF_LOG_INFO("BLE_GAP_EVT_CONNECTED");
      // on_connect(p_mic_service, p_ble_evt);
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("BLE_GAP_EVT_DISCONNECTED");
      // on_disconnect(p_mic_service, p_ble_evt);
      break;

    case BLE_GATTS_EVT_WRITE:
      NRF_LOG_INFO("BLE_GATTS_EVT_WRITE");
      // on_write(p_mic_service, p_ble_evt);
      break;

    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
    {
      NRF_LOG_INFO("BLE_GATTS_EVT_HVN_TX_COMPLETE");
      //uint32_t count = p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count;
      // if(file_size > 0) {
      //   push_data_packets();
      // }
      // nrf_error_resources = false;
      break;
    }


    default:
      // No implementation needed.
      NRF_LOG_ERROR("ble_mic_service_on_ble_evt: not implemented, %d", p_ble_evt->header.evt_id);
      break;
  }
}

static uint32_t tx_char_add(ble_mic_service_t * p_mic_service, const ble_mic_service_init_t * p_mic_service_init)
{
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  memset(&cccd_md, 0, sizeof(cccd_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.notify = 1;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = &cccd_md;
  char_md.p_sccd_md         = NULL;

  ble_uuid.type = p_mic_service->uuid_type;
  ble_uuid.uuid = BLE_UUID_MIC_TX_CHARACTERISTIC;

  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  attr_md.vloc    = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen    = 1;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = sizeof(uint8_t);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_MIC_MAX_TX_CHAR_LEN;

  return sd_ble_gatts_characteristic_add(p_mic_service->service_handle,
                                          &char_md,
                                          &attr_char_value,
                                          &p_mic_service->tx_handles);
}

static uint32_t rx_char_add(ble_mic_service_t * p_mic_service, const ble_mic_service_init_t * p_mic_service_init)
{
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.write         = 1;
  char_md.char_props.write_wo_resp = 1;
  char_md.p_char_user_desc         = NULL;
  char_md.p_char_pf                = NULL;
  char_md.p_user_desc_md           = NULL;
  char_md.p_cccd_md                = NULL;
  char_md.p_sccd_md                = NULL;

  ble_uuid.type = p_mic_service->uuid_type;
  ble_uuid.uuid = BLE_UUID_MIC_RX_CHARACTERISTIC;

  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  attr_md.vloc    = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen    = 1;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = 1;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_MIC_MAX_RX_CHAR_LEN;

  return sd_ble_gatts_characteristic_add(p_mic_service->service_handle,
                                          &char_md,
                                          &attr_char_value,
                                          &p_mic_service->rx_handles);
}

/**< Used vendor specific UUID. */
#define MIC_SERVICE_BASE_UUID {{0x3E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}}
#define BLE_UUID_MIC_SERVICE 0x0001                      /**< The UUID of the Nordic UART Service. */

uint32_t ble_mic_service_init(ble_mic_service_t * p_mic_service, const ble_mic_service_init_t * p_mic_service_init)
{
  uint32_t      err_code;
  ble_uuid_t    ble_uuid;
  ble_uuid128_t mic_service_base_uuid = MIC_SERVICE_BASE_UUID;

  VERIFY_PARAM_NOT_NULL(p_mic_service);
  VERIFY_PARAM_NOT_NULL(p_mic_service_init);

  // Initialize the service structure.
  p_mic_service->conn_handle             = BLE_CONN_HANDLE_INVALID;
  p_mic_service->data_handler            = p_mic_service_init->data_handler;
  p_mic_service->is_notification_enabled = false;

  // Add a custom base UUID.
  err_code = sd_ble_uuid_vs_add(&mic_service_base_uuid, &p_mic_service->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_mic_service->uuid_type;
  ble_uuid.uuid = BLE_UUID_MIC_SERVICE;

  // Add the service.
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                      &ble_uuid,
                                      &p_mic_service->service_handle);
  VERIFY_SUCCESS(err_code);

  // Add the RX Characteristic.
  err_code = rx_char_add(p_mic_service, p_mic_service_init);
  VERIFY_SUCCESS(err_code);

  // Add the TX Characteristic.
  err_code = tx_char_add(p_mic_service, p_mic_service_init);
  VERIFY_SUCCESS(err_code);

  return NRF_SUCCESS;
}

void services_init(void)
{
  ret_code_t             err_code;
  nrf_ble_qwr_init_t     qwr_init = {0};
  ble_mic_service_init_t mic_service_init;

  // Initialize Queued Write Module.
  qwr_init.error_handler = nrf_qwr_error_handler;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);

  // Initialize mic service
  memset(&mic_service_init, 0, sizeof(mic_service_init));

  mic_service_init.data_handler = mic_service_data_handler;

  err_code = ble_mic_service_init(&m_mic_service, &mic_service_init);
  APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
  switch(p_evt->evt_type)
  {
    case BLE_CONN_PARAMS_EVT_SUCCEEDED:
      break;

    case BLE_CONN_PARAMS_EVT_FAILED:
      //err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
      //APP_ERROR_CHECK(err_code);
      NRF_LOG_ERROR("Conn params failed. Keep the connection anyway..");
      break;

    default:
      NRF_LOG_INFO("Unhandled conn params event %d", p_evt->evt_type);
      break;
  }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("BLE_ADV_EVT_IDLE...");
            sleep_mode_enter();
            // Option to restart advertising
            // err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            // APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            // ble_mic_service_ble_params_info_send(&m_mic_service, &m_ble_params_info);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {
          uint16_t max_con_int = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval;
          uint16_t min_con_int = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval;
          // m_ble_params_info.con_interval = max_con_int;
          // ble_mic_service_ble_params_info_send(&m_mic_service, &m_ble_params_info);
          NRF_LOG_INFO("Con params updated: CI %i, %i", (int)min_con_int, (int)max_con_int);
          break;
        }

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
          NRF_LOG_DEBUG("PHY update request.");
          ble_gap_phys_t const phys =
          {
              .rx_phys = BLE_GAP_PHY_AUTO,
              .tx_phys = BLE_GAP_PHY_AUTO,
          };
          err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
          APP_ERROR_CHECK(err_code);
          break;
        }

        case BLE_GAP_EVT_PHY_UPDATE:
        {
          int tx_phy = p_ble_evt->evt.gap_evt.params.phy_update.tx_phy;
          int rx_phy = p_ble_evt->evt.gap_evt.params.phy_update.rx_phy;
          // ble_mic_service_ble_params_info_send(&m_mic_service, &m_ble_params_info);
          NRF_LOG_INFO("Phy update: %i, %i", tx_phy, rx_phy);
          break;
        }

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
          NRF_LOG_INFO("Pairing not supported");
          err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
          APP_ERROR_CHECK(err_code);
          break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
          NRF_LOG_INFO("No system attributes have been stored.");
          err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
          APP_ERROR_CHECK(err_code);
          break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            NRF_LOG_INFO("BLE event not handled by app: %i", p_ble_evt->header.evt_id);
            break;
    }
}

void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = false;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

void advertising_start()
{
  ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);
}

void bleInit(void)
{
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
}