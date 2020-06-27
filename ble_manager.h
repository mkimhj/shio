#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_MIC_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#else
    #define BLE_MIC_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
    #warning NRF_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

// START OF CUSTOM MIC BLE SERVICE DEFINITIONS
void ble_mic_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

#define BLE_MIC_SERVICE_DEF(_name)               \
static ble_mic_service_t _name;                  \
NRF_SDH_BLE_OBSERVER(_name ## _obs,              \
                     BLE_NUS_BLE_OBSERVER_PRIO,  \
                     ble_mic_service_on_ble_evt, \
                     &_name);

typedef struct ble_mic_service_s ble_mic_service_t;

/**@brief Nordic UART Service event handler type. */
typedef void (*ble_mic_service_data_handler_t) (ble_mic_service_t * p_mic_service, uint8_t const * p_data, uint16_t length);

typedef struct ble_mic_service_s {
  uint8_t                  uuid_type;               /**< UUID type for Nordic UART Service Base UUID. */
  uint16_t                 service_handle;          /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
  ble_gatts_char_handles_t tx_handles;              /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
  ble_gatts_char_handles_t rx_handles;              /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
  uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
  bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
  bool                     is_info_char_notification_enabled;
  ble_mic_service_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
} ble_mic_service_t;

typedef struct {
  ble_mic_service_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_mic_service_init_t;

BLE_MIC_SERVICE_DEF(m_mic_service);
// END OF CUSTOM MIC BLE SERVICE DEFINITIONS

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

void advertising_start();
void bleInit(void);