/******************************************************************************
 * @file    m_uwb_range.c
 * @author  Insight SiP
 * @brief   Range module implementation file.
 *
 * @attention
 *	THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *	OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *	OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *	OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include "app_timer.h"
#include "app_util_platform.h"
#include <string.h>

#include "boards.h"
#include "drv_uwb_range.h"
#include "m_uwb_range.h"
#if defined(USE_LEGACY_BLE_RANGE_SERVICE)
#include "legacy_ble_range.h"
#else
#include "ble_range.h"
#endif

#define NRF_LOG_MODULE_NAME m_uwb_range
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define TAG_LEDS_ENABLED
#define TAG_LEDS_ON_DELAY_MS 20

BLE_RANGE_DEF(m_ble_range);               /**< Structure to identify the range Service. */
APP_TIMER_DEF(range_timer_id);            /**< Timer for range update. */
APP_TIMER_DEF(led_timer_id);              /**< Timer for LED blink. */
static uint16_t range_interval = 300;     /**< Interval between range updates */
static m_uwb_range_role_t operating_role; /**< Initiator or responder */
static bool is_started = false;
static uint8_t log1[50];
static uint8_t log2[50];

static const uint8_t *operating_role_name[] = {
    "Initiator",
    "Responder"};

/**@brief Function for handling range timer timout event.
 */
static void range_timeout_handler(void *p_context) {
    uint32_t err_code;

    err_code = drv_uwb_range_request();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling led timer timout event.
 */
static void led_timeout_handler(void *p_context) {
    // Shutdown both LED
    nrf_gpio_cfg_default(PIN_GREEN_LED);
    nrf_gpio_cfg_default(PIN_RED_LED);
}

/**@brief Function for blinking LED
 */
static uint32_t blink_led(uint8_t pin) {
#if defined(TAG_LEDS_ENABLED)
    uint32_t err_code;

    if (pin == 255)
        return NRF_ERROR_INVALID_PARAM;

    if ((pin != PIN_GREEN_LED) && (pin != PIN_RED_LED))
        return NRF_ERROR_INVALID_PARAM;

    // LED ON
    nrf_gpio_pin_set(pin);
    nrf_gpio_cfg_output(pin);

    // Start timer to shutdown down LED after a while
    err_code = app_timer_start(led_timer_id, APP_TIMER_TICKS(TAG_LEDS_ON_DELAY_MS), NULL);
    VERIFY_SUCCESS(err_code);
#else
    return NRF_SUCCESS;
#endif
}

/**@brief Function for starting range sampling.
 */
static uint32_t range_start(void) {
    uint32_t err_code;

    switch (operating_role) {
    case TWR_RESPONDER:
        err_code = drv_uwb_range_scan_start();
        VERIFY_SUCCESS(err_code);
        is_started = true;
        break;

    case TWR_INITIATOR:
        err_code = app_timer_start(range_timer_id, APP_TIMER_TICKS(range_interval), NULL);
        VERIFY_SUCCESS(err_code);
        is_started = true;
        break;

    default:
        break;
    }

    return NRF_SUCCESS;
}

/**@brief Function for stopping range sampling.
 */
static uint32_t range_stop(void) {
    uint32_t err_code;

    switch (operating_role) {
    case TWR_RESPONDER:
        err_code = drv_uwb_range_scan_stop();
        VERIFY_SUCCESS(err_code);
        is_started = false;
        break;

    case TWR_INITIATOR:
        err_code = app_timer_stop(range_timer_id);
        VERIFY_SUCCESS(err_code);
        is_started = false;
        break;

    default:
        break;
    }

    return NRF_SUCCESS;
}
#if !defined(USE_LEGACY_BLE_RANGE_SERVICE)
/**@brief Function for verifying the configuration.
 *
 */
static uint32_t config_verify(ble_range_config_t *p_config) {
    uint32_t err_code;

    if (p_config->range_interval_ms < BLE_RANGE_CONFIG_RANGE_INT_MIN) {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (p_config->destination_address == NULL) {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (p_config->source_address == NULL) {
        return NRF_ERROR_INVALID_PARAM;
    }

    return NRF_SUCCESS;
}

/**@brief Function for applying the configuration.
 *
 */
static uint32_t config_apply(ble_range_config_t *p_config) {
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_config);

    range_stop();

    drv_uwb_range_destination_address_set(p_config->destination_address);
    drv_uwb_range_source_address_set(p_config->source_address);

    if ((p_config->range_interval_ms > 0) && (m_ble_range.is_range_notif_enabled)) {
        err_code = range_start();
        VERIFY_SUCCESS(err_code);
    }

    return NRF_SUCCESS;
}
#endif // !USE_LEGACY_BLE_RANGE_SERVICE

/**@brief Event handler, handles events in the uwb range driver.
 *
 */
static void drv_uwb_range_evt_handler(drv_uwb_range_evt_t const *p_evt) {
    switch (p_evt->type) {
    case DRV_RANGE_EVT_DATA: {
        ble_range_data_t data;
        data = p_evt->range;

        // If notif enabled send Raw data on BLE
#if defined(USE_LEGACY_BLE_RANGE_SERVICE)
        if (m_ble_range.is_inst_data_notif_enabled) {
            ble_range_inst_data_set(&m_ble_range, &data);
        }
#else
        if (m_ble_range.is_range_notif_enabled) {
            ble_range_data_set(&m_ble_range, &data);
        }
#endif // USE_LEGACY_BLE_RANGE_SERVICE
        NRF_LOG_INFO("Range = " NRF_LOG_FLOAT_MARKER " m", NRF_LOG_FLOAT(data));
        // Blink green LED
        blink_led(PIN_GREEN_LED);
        break;
    }

    case DRV_RANGE_EVT_TIMEOUT:
        NRF_LOG_WARNING("Range timeout");
        // Blink red LED
        blink_led(PIN_RED_LED);
        break;

    case DRV_RANGE_EVT_ERROR:
        NRF_LOG_WARNING("Range error");
        // Blink red LED
        blink_led(PIN_RED_LED);
        break;

    default:
        break;
    }
}

/**@brief Event handler, handles events in the Range Service.
 *
 * @details This callback function is often used to enable a service when requested over BLE,
 * and disable when not requested to save power. 
 */
static void ble_range_evt_handler(ble_range_t *p_range, ble_range_evt_t *p_evt) {
    uint32_t err_code;

    switch (p_evt->evt_type) {
#if defined(USE_LEGACY_BLE_RANGE_SERVICE)
    case BLE_RANGE_EVT_INST_NOTIFICATION_ENABLED:
        NRF_LOG_INFO("BLE_RANGE_EVT_INST_NOTIFICATION_ENABLED");
        break;

    case BLE_RANGE_EVT_INST_NOTIFICATION_DISABLED:
        NRF_LOG_INFO("BLE_RANGE_EVT_INST_NOTIFICATION_DISABLED");
        break;

    case BLE_RANGE_EVT_AVG_NOTIFICATION_ENABLED:
        NRF_LOG_INFO("BLE_RANGE_EVT_AVG_NOTIFICATION_ENABLED");
        break;

    case BLE_RANGE_EVT_AVG_NOTIFICATION_DISABLED:
        NRF_LOG_INFO("BLE_RANGE_EVT_AVG_NOTIFICATION_DISABLED");
        break;

    case BLE_RANGE_EVT_PERIOD_CHANGED:
        NRF_LOG_INFO("BLE_RANGE_EVT_PERIOD_CHANGED");
        uint16_t tmp_interval = (uint16_t)(p_evt->p_data[0]) | (uint16_t)(p_evt->p_data[1] << 8);
        if (tmp_interval >= 300) {
            range_interval = tmp_interval;
            if (is_started) {
                range_stop();
                range_start();
            }
        }
        break;

    case BLE_RANGE_EVT_UWB_CHANGED:
        NRF_LOG_INFO("BLE_RANGE_EVT_UWB_CHANGED");
        break;

    case BLE_RANGE_EVT_MODE_CHANGED:
        NRF_LOG_INFO("BLE_RANGE_EVT_MODE_CHANGED");
        break;

    case BLE_RANGE_EVT_CONTROLS_CHANGED:
        NRF_LOG_INFO("BLE_RANGE_EVT_CONTROLS_CHANGED");
        break;
#else
    case BLE_RANGE_EVT_NOTIFICATION_ENABLED:
        NRF_LOG_INFO("BLE_RANGE_EVT_NOTIFICATION_ENABLED");
        break;

    case BLE_RANGE_EVT_NOTIFICATION_DISABLED:
        NRF_LOG_INFO("BLE_RANGE_EVT_NOTIFICATION_DISABLED");
        break;

    case BLE_RANGE_EVT_CONFIG_RECEIVED:
        NRF_LOG_INFO("BLE_RANGE_EVT_CONFIG_RECEIVED: %d\r\n", p_evt->length);
        APP_ERROR_CHECK_BOOL(p_evt->length == sizeof(ble_range_config_t));

        if (config_verify((ble_range_config_t *)(p_evt->p_data)) == NRF_SUCCESS) {
            config_apply((ble_range_config_t *)(p_evt->p_data));
        }
        break;
#endif // USE_LEGACY_BLE_RANGE_SERVICE

    default:
        break;
    }
}

/**@brief Function for initializing the  range Service.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
static uint32_t range_service_init() {
#if defined(USE_LEGACY_BLE_RANGE_SERVICE)
    uint32_t err_code;
    ble_range_mode_t mode;
    ble_range_status_t status;
    ble_range_controls_t controls;
    ble_range_init_t range_init;
    ble_range_period_config_t period_config = 300;
    ble_range_uwb_config_t uwb_config = 0;

    if (operating_role == TWR_INITIATOR) {
        mode = 1;
        controls = 1;
        status = 1;
    } else {
        mode = 2;
        controls = 0;
        status = 0;
    }
    ble_range_avg_config_t avg_config = 8;
    ble_range_threshold_config_t threshold_config = 0x780064;

    // Initialize service
    memset(&range_init, 0, sizeof(range_init));
    range_init.p_init_period_config = &period_config;
    range_init.p_init_uwb_config = &uwb_config;
    range_init.p_init_mode = &mode;
    range_init.p_init_avg_config = &avg_config;
    range_init.p_init_threshold_config = &threshold_config;
    range_init.p_init_status = &status;
    range_init.p_init_controls = &controls;
    range_init.evt_handler = ble_range_evt_handler;

    err_code = ble_range_init(&m_ble_range, &range_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
#else
    uint32_t err_code;
    ble_range_init_t range_init;
    ble_range_config_t config_init;

    config_init.range_interval_ms = 500;
    drv_uwb_range_destination_address_get(config_init.destination_address);
    drv_uwb_range_source_address_get(config_init.source_address);

    // Initialize service
    memset(&range_init, 0, sizeof(range_init));
    memcpy(range_init.p_init_config, &config_init, sizeof(ble_range_config_t));
    range_init.evt_handler = ble_range_evt_handler;

    err_code = ble_range_init(&m_ble_range, &range_init);
    VERIFY_SUCCESS(err_code);

    config_apply(&config_init);

    return NRF_SUCCESS;
#endif // USE_LEGACY_BLE_RANGE_SERVICE
}

uint32_t m_range_init(m_uwb_range_init_t *p_params, m_ble_service_handle_t *p_handle) {
    uint32_t err_code;
    drv_uwb_range_init_t range_init;

    operating_role = p_params->role;
    p_handle->init_cb = range_service_init;

    if (operating_role >= NUM_ROLES) {
        NRF_LOG_ERROR("Unsuported UWB role");
        return NRF_ERROR_INTERNAL;
    }

    // the "own address" using for uwb ranging will be the nRF52 device ID
    uint8_t own_address[8];
    uint32_encode(NRF_FICR->DEVICEID[0], own_address);
    uint32_encode(NRF_FICR->DEVICEID[1], own_address + 4);

    // the "reply address" using for uwb ranging will be fetch from UICR 2 & 3
    uint8_t reply_address[8];
    uint32_encode(NRF_UICR->CUSTOMER[2], reply_address);
    uint32_encode(NRF_UICR->CUSTOMER[3], reply_address + 4);

    range_init.evt_handler = drv_uwb_range_evt_handler;
    range_init.role = operating_role;
    range_init.own_address = own_address;
    range_init.reply_address = reply_address;

    // enable filter if a reply_address is set
    if ((reply_address[0] != 0xFF) || (reply_address[1] != 0xFF) || (reply_address[2] != 0xFF) || (reply_address[3] != 0xFF) ||
        (reply_address[4] != 0xFF) || (reply_address[5] != 0xFF) || (reply_address[6] != 0xFF) || (reply_address[7] != 0xFF)) {
        range_init.enable_filter = true;
    }

    // Tag / Anchor specific configuration
    if (operating_role == TWR_RESPONDER) {
        range_init.enable_tx_rx_leds = true;
        range_init.enable_sleep = false;
    } else {
        range_init.enable_tx_rx_leds = false;
        range_init.enable_sleep = true;
    }

    // Initialize uwb range drivers
    err_code = drv_uwb_range_init(&range_init);
    VERIFY_SUCCESS(err_code);

    // range app timer
    err_code = app_timer_create(&range_timer_id, APP_TIMER_MODE_REPEATED, range_timeout_handler);
    VERIFY_SUCCESS(err_code);

    err_code = app_timer_create(&led_timer_id, APP_TIMER_MODE_SINGLE_SHOT, led_timeout_handler);
    VERIFY_SUCCESS(err_code);

    NRF_LOG_INFO("UWB role: %s", operating_role_name[operating_role]);
    sprintf(log1, "Src address: %x:%x:%x:%x:%x:%x:%x:%x",
        own_address[0], own_address[1], own_address[2], own_address[3],
        own_address[4], own_address[5], own_address[6], own_address[7]);
    NRF_LOG_INFO("%s", (uint32_t)log1);
    sprintf(log2, "Dst address: %x:%x:%x:%x:%x:%x:%x:%x",
        reply_address[0], reply_address[1], reply_address[2], reply_address[3],
        reply_address[4], reply_address[5], reply_address[6], reply_address[7]);
    NRF_LOG_INFO("%s", (uint32_t)log2);

    return NRF_SUCCESS;
}

uint32_t m_range_start(void) {
    uint32_t err_code;

    err_code = range_start();
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t m_range_stop(void) {
    uint32_t err_code;

    err_code = range_stop();
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t m_range_ble_uuid_get(uint16_t *uuid) {
    uint32_t err_code;

    *uuid = BLE_UUID_RANGE_SERVICE;

    return NRF_SUCCESS;
}