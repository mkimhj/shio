#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_ble_lesc.h"

#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "nrf_cli.h"
#include "nrf_cli_uart.h"
#include "cli_m.h"
#include "ble_m.h"
#include "pm_m.h"
#include "nfc_m.h"
#include "nfc_central_m.h"
#include "nrf_cli_uart.h"

#include "time_sync.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"
#include "nrf_gpiote.h"

/**** PWM related ****/
#define PWM_PIN NRF_GPIO_PIN_MAP(1, 7)
#include "nrf_drv_pwm.h"
#include "nrf_drv_clock.h"
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
nrf_pwm_values_individual_t seq_values[] = {{100, 0, 0, 0}};
nrf_pwm_sequence_t const seq =
{
    .values.p_individual = seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats         = 0,
    .end_delay       = 0
};

void pwm_init(){
  nrf_drv_clock_lfclk_request(NULL);

  // init PWM for 50kHz clock
  // Start clock for accurate frequencies
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  // Wait for clock to start
  while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
  nrf_drv_pwm_config_t const config0 =
  {
      .output_pins =
      {
          PWM_PIN, // channel 0
          NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
          NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
          NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
      },
      .irq_priority = APP_IRQ_PRIORITY_LOWEST,
      .base_clock   = NRF_PWM_CLK_125kHz, //NRF_PWM_CLK_1MHz,
      .count_mode   = NRF_PWM_MODE_UP,
      .top_value    = 12500, //10Hz
      .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
      .step_mode    = NRF_PWM_STEP_AUTO
  };

  // Init PWM without error handler
  APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));

  nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
  // end
}


/**** end PWM related ****/

#define START_PIN NRF_GPIO_PIN_MAP(1, 6)
#define EN_IND_PIN 3
#define TS_IND_PIN 2


#define LED_BLE_NUS_CONN (BSP_BOARD_LED_0)
#define LED_BLE_NUS_RX   (BSP_BOARD_LED_1)
#define LED_CDC_ACM_CONN (BSP_BOARD_LED_2)
#define LED_CDC_ACM_RX   (BSP_BOARD_LED_3)
#define LED_BLINK_INTERVAL 800
#define ENDLINE_STRING "\r\n"
#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2
#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1
#define CLI_OVER_UART 1
#define CLI_EXAMPLE_LOG_QUEUE_SIZE  (4)
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

APP_TIMER_DEF(m_blink_ble);
APP_TIMER_DEF(m_blink_cdc);
NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 256, 256);
NRF_CLI_DEF(m_cli_uart, "udon] ", &m_cli_uart_transport.transport, '\r', CLI_LOG_QUEUE_SIZE);
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static volatile bool usbPortOpened = false;
static volatile bool m_usb_connected = false;
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);
static char m_cdc_data_array[BLE_NUS_MAX_DATA_LEN];

/** @brief CDC_ACM class instance */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

char testBytes[5] = { 'a', 'b', 'c', '\r', '\n' };
char testBytes2[244] = {0xF4,0x00,0xF5,0x00,0xF6,0x00,0xF7,0x00,0xF8,0x00,0xF9,0x00,0xFA,0x00,0xFB,0x00,0xFC,0x00,
                        0xFD,0x00,0xFE,0x00,0xFF,0x00,0x00,0x01,0x01,0x01,0x02,0x01,0x03,0x01,0x04,0x01,0x05,0x01,
                        0x06,0x01,0x07,0x01,0x08,0x01,0x09,0x01,0x0A,0x01,0x0B,0x01,0x0C,0x01,0x0D,0x01,0x0E,0x01,
                        0x0F,0x01,0x10,0x01,0x11,0x01,0x12,0x01,0x13,0x01,0x14,0x01,0x15,0x01,0x16,0x01,0x17,0x01,
                        0x18,0x01,0x19,0x01,0x1A,0x01,0x1B,0x01,0x1C,0x01,0x1D,0x01,0x1E,0x01,0x1F,0x01,0x20,0x01,
                        0x21,0x01,0x22,0x01,0x23,0x01,0x24,0x01,0x25,0x01,0x26,0x01,0x27,0x01,0x28,0x01,0x29,0x01,
                        0x2A,0x01,0x2B,0x01,0x2C,0x01,0x2D,0x01,0x2E,0x01,0x2F,0x01,0x30,0x01,0x31,0x01,0x32,0x01,
                        0x33,0x01,0x34,0x01,0x35,0x01,0x36,0x01,0x37,0x01,0x38,0x01,0x39,0x01,0x3A,0x01,0x3B,0x01,
                        0x3C,0x01,0x3D,0x01,0x3E,0x01,0x3F,0x01,0x40,0x01,0x41,0x01,0x42,0x01,0x43,0x01,0x44,0x01,
                        0x45,0x01,0x46,0x01,0x47,0x01,0x48,0x01,0x49,0x01,0x4A,0x01,0x4B,0x01,0x4C,0x01,0x4D,0x01,
                        0x4E,0x01,0x4F,0x01,0x50,0x01,0x51,0x01,0x52,0x01,0x53,0x01,0x54,0x01,0x55,0x01,0x56,0x01,
                        0x57,0x01,0x58,0x01,0x59,0x01,0x5A,0x01,0x5B,0x01,0x5C,0x01,0x5D,0x01,0x5E,0x01,0x5F,0x01,
                        0x60,0x01,0x61,0x01,0x62,0x01,0x63,0x01,0x64,0x01,0x65,0x01,0x66,0x01,0x67,0x01,0x68,0x01,
                        0x69,0x01,0x6A,0x01,0x6B,0x01,0x6C,0x01,0x6D,0x01};
uint8_t usbDataBuffer[512] = {0};

void blink_handler(void * p_context)
{
    bsp_board_led_invert((uint32_t) p_context);
}

void mainScratchpad()
{
  if (m_usb_connected && usbPortOpened) {
    NRF_LOG_INFO("writing");
    ret_code_t err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm, testBytes2, sizeof(testBytes2));
    APP_ERROR_CHECK(err_code);
  }
}

void usbSendData(uint8_t * data, uint16_t length)
{
  if (m_usb_connected && usbPortOpened) {
    memcpy(usbDataBuffer, data, length);
    ret_code_t err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm, usbDataBuffer, length);
    APP_ERROR_CHECK(err_code);
  }
}

/** @brief Function for initializing the timer module. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_blink_ble, APP_TIMER_MODE_REPEATED, blink_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_blink_cdc, APP_TIMER_MODE_REPEATED, blink_handler);
    APP_ERROR_CHECK(err_code);
}

static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);
    APP_ERROR_CHECK(err_code);
}

void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    static bool m_start_clock=false;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BSP_EVENT_KEY_3:

            if (m_start_clock)
            {
                m_start_clock = false;

                bsp_board_led_off(EN_IND_PIN);

                nrf_gpio_pin_clear(START_PIN);

                NRF_LOG_INFO("en stop\n");
            }
            else
            {
                m_start_clock = true;

                bsp_board_led_on(EN_IND_PIN);

                nrf_gpio_pin_set(START_PIN);

                NRF_LOG_INFO("en start\n");
            }
            break;
        default:
            break;
    }
}

static void buttons_leds_init(void)
{
    uint32_t err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    // NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());

    ret_code_t err_code;
    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);

    nrf_cli_process(&m_cli_uart);
    power_manage();
}

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            /*Set up the first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_cdc_data_array,
                                                   1);
            UNUSED_VARIABLE(ret);
            ret = app_timer_stop(m_blink_cdc);
            APP_ERROR_CHECK(ret);
            bsp_board_led_on(LED_CDC_ACM_CONN);
            NRF_LOG_INFO("CDC ACM port opened");

            usbPortOpened = true;


            break;
        }

        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            NRF_LOG_INFO("CDC ACM port closed");
            if (m_usb_connected)
            {
                ret_code_t ret = app_timer_start(m_blink_cdc,
                                                 APP_TIMER_TICKS(LED_BLINK_INTERVAL),
                                                 (void *) LED_CDC_ACM_CONN);
                APP_ERROR_CHECK(ret);
            }
            usbPortOpened = false;
            break;

        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;

        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            static uint8_t index = 0;
            index++;

            do
            {
                if ((m_cdc_data_array[index - 1] == '\n') ||
                    (m_cdc_data_array[index - 1] == '\r') ||
                    (index >= (m_ble_nus_max_data_len)))
                {
                    if (index > 1)
                    {
                        bsp_board_led_invert(LED_CDC_ACM_RX);
                        NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                        NRF_LOG_HEXDUMP_DEBUG(m_cdc_data_array, index);

                        do
                        {
                            uint16_t length = (uint16_t)index;
                            if (length + sizeof(ENDLINE_STRING) < BLE_NUS_MAX_DATA_LEN)
                            {
                                memcpy(m_cdc_data_array + length, ENDLINE_STRING, sizeof(ENDLINE_STRING));
                                length += sizeof(ENDLINE_STRING);
                            }

                            ret = ble_nus_data_send(&m_nus,
                                                    (uint8_t *) m_cdc_data_array,
                                                    &length,
                                                    m_conn_handle);

                            if (ret == NRF_ERROR_NOT_FOUND)
                            {
                                NRF_LOG_INFO("BLE NUS unavailable, data received: %s", m_cdc_data_array);
                                break;
                            }

                            if (ret == NRF_ERROR_RESOURCES)
                            {
                                NRF_LOG_ERROR("BLE NUS Too many notifications queued.");
                                break;
                            }

                            if ((ret != NRF_ERROR_INVALID_STATE) && (ret != NRF_ERROR_BUSY))
                            {
                                APP_ERROR_CHECK(ret);
                            }
                        }
                        while (ret == NRF_ERROR_BUSY);
                    }

                    index = 0;
                }

                /*Get amount of data transferred*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                NRF_LOG_DEBUG("RX: size: %lu char: %c", size, m_cdc_data_array[index - 1]);

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            &m_cdc_data_array[index],
                                            1);
                if (ret == NRF_SUCCESS)
                {
                    index++;
                }
            }
            while (ret == NRF_SUCCESS);

            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;

        case APP_USBD_EVT_DRV_RESUME:
            break;

        case APP_USBD_EVT_STARTED:
            break;

        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;

        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;

        case APP_USBD_EVT_POWER_REMOVED:
        {
            NRF_LOG_INFO("USB power removed");
            ret_code_t err_code = app_timer_stop(m_blink_cdc);
            APP_ERROR_CHECK(err_code);
            bsp_board_led_off(LED_CDC_ACM_CONN);
            m_usb_connected = false;
            app_usbd_stop();
        }
            break;

        case APP_USBD_EVT_POWER_READY:
        {
            NRF_LOG_INFO("USB ready");
            ret_code_t err_code = app_timer_start(m_blink_cdc,
                                                  APP_TIMER_TICKS(LED_BLINK_INTERVAL),
                                                  (void *) LED_CDC_ACM_CONN);
            APP_ERROR_CHECK(err_code);
            m_usb_connected = true;
            app_usbd_start();
        }
            break;

        default:
            break;
    }
}

void cli_init(void)
{
    ret_code_t ret;

    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
}

void cli_start(void)
{
    ret_code_t ret;

    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
}

void timeSyncInit(void)
{
  uint32_t       err_code;
  uint8_t        rf_address[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x19};
  ts_params_t    ts_params;

  nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(1, 14), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
  nrf_gpiote_task_enable(3);

  nrf_ppi_channel_endpoint_setup(
      NRF_PPI_CHANNEL0,
      (uint32_t) nrf_timer_event_address_get(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE4),
      nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
  nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);

  ts_params.high_freq_timer[0] = NRF_TIMER3;
  ts_params.high_freq_timer[1] = NRF_TIMER2;
  ts_params.rtc             = NRF_RTC1;
  ts_params.egu             = NRF_EGU3;
  ts_params.egu_irq_type    = SWI3_EGU3_IRQn;
  ts_params.ppi_chg         = 0;
  ts_params.ppi_chns[0]     = 1;
  ts_params.ppi_chns[1]     = 2;
  ts_params.ppi_chns[2]     = 3;
  ts_params.ppi_chns[3]     = 4;
  ts_params.rf_chn          = 125; /* For testing purposes */
  memcpy(ts_params.rf_addr, rf_address, sizeof(rf_address));

  err_code = ts_init(&ts_params);
  APP_ERROR_CHECK(err_code);

  err_code = ts_enable();
  APP_ERROR_CHECK(err_code);
}

int main(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    // Initialize.
    log_init();
    timers_init();

    buttons_leds_init();

    app_usbd_serial_num_generate();

    ret = nrf_drv_clock_init();
    pwm_init();
    APP_ERROR_CHECK(ret);

    NRF_LOG_INFO("udon booted");

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    // Console start
    cli_init();
    cli_start();
    ble_m_init();
    peer_manager_init();
    nfc_pairing_init();
    bond_get();

    // Start execution.
    // advertising_start();

    // time sync start
    timeSyncInit();

    ret = app_usbd_power_events_enable();
    APP_ERROR_CHECK(ret);

    // Enter main loop.
    for (;;)
    {
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
        idle_state_handle();
    }
}
