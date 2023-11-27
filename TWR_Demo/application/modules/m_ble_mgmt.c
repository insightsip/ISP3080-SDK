/******************************************************************************
 * @file    m_ble_mgmt.c
 * @author  Insight SiP
 * @brief   BLE management implementation file.
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

#include <stdint.h>
#include <string.h>

#include "app_scheduler.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dis.h"
#include "boards.h"
#include "m_ble_mgmt.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "sdk_config.h"

#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#endif // BLE_DFU_APP_SUPPORT

#define NRF_LOG_MODULE_NAME m_ble_mgmt
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define APP_ADV_INTERVAL 1000 /**< The advertising interval (in units of 0.625 ms). */
#define APP_ADV_DURATION 0    /**< The advertising duration in units of 10 milliseconds. */

#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG 1  /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(2000, UNIT_10_MS)   /**< Connection supervisory timeout (2 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

NRF_BLE_GATT_DEF(m_gatt);                                /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                      /**< Advertising module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                  /**< Context for the Queued Write module.*/
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static ble_uuid_t adv_uuids[1];                          /**< Universally unique service identifiers. */
static m_ble_mgmt_event_handler_t ble_mgmt_evt_handler;  /**< BLE mgmt event handler function pointer. */
static m_ble_mgmt_param_t ble_mgmt_param;                /**< BLE mgmt parameter. */
static m_ble_service_handle_t *m_service_handles = 0;
static uint32_t m_service_num = 0;

/** @brief Checks validity of supplied parameters.
 */
static uint32_t param_check(ble_mgmt_init_t const *const p_ble_mgmt_init) {
    VERIFY_PARAM_NOT_NULL(p_ble_mgmt_init);
    VERIFY_PARAM_NOT_NULL(p_ble_mgmt_init->evt_handler);
    VERIFY_PARAM_NOT_NULL(p_ble_mgmt_init->ble_mgmt_param.fw_version);
    VERIFY_PARAM_NOT_NULL(p_ble_mgmt_init->ble_mgmt_param.mfg_name);

    return NRF_SUCCESS;
}

#ifdef BLE_DFU_APP_SUPPORT
static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void *p_context) {
    if (state == NRF_SDH_EVT_STATE_DISABLED) {
        //TODO Check if needed
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        //nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        // nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
    {
        .handler = buttonless_dfu_sdh_state_observer,
};
#endif // BLE_DFU_APP_SUPPORT

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
    ret_code_t err_code = NRF_SUCCESS;
    m_ble_mgmt_event_t evt;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected.");
        evt.type = M_BLE_MGMT_EVENT_DISCONNECTED;
        ble_mgmt_evt_handler(&evt);
        break;

    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected.");
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        evt.type = M_BLE_MGMT_EVENT_CONNECTED;
        ble_mgmt_evt_handler(&evt);
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        // No implementation needed.
        break;
    }
}

static void disconnect(uint16_t m_conn_handle, void *p_context) {
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", m_conn_handle, err_code);
    } else {
        NRF_LOG_DEBUG("Disconnected connection handle %d", m_conn_handle);
    }
}

static void advertising_config_get(ble_adv_modes_config_t *p_config) {
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout = APP_ADV_DURATION;
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static uint32_t ble_stack_init(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    VERIFY_SUCCESS(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    VERIFY_SUCCESS(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    VERIFY_SUCCESS(err_code);

    // Enable DCDC
    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    VERIFY_SUCCESS(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    return NRF_SUCCESS;
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static uint32_t gap_params_init(uint8_t const *p_dev_name) {
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)p_dev_name, strlen(p_dev_name));
    VERIFY_SUCCESS(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for initializing the GATT module.
 */
static uint32_t gatt_init(void) {
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static uint32_t conn_params_init(void) {
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for initializing the Queued Write Module.
 */
static uint32_t qwr_init(void) {
    uint32_t err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event) {
    switch (event) {
    case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE: {
        NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

        // Prevent device from advertising on disconnect.
        ble_adv_modes_config_t config;
        advertising_config_get(&config);
        config.ble_adv_on_disconnect_disabled = true;
        ble_advertising_modes_config_set(&m_advertising, &config);

        // Disconnect all other bonded devices that currently are connected.
        // This is required to receive a service changed indication
        // on bootup after a successful (or aborted) Device Firmware Update.
        uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
        NRF_LOG_INFO("Disconnected %d links.", conn_count);
        break;
    }

    case BLE_DFU_EVT_BOOTLOADER_ENTER:
        // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
        //           by delaying reset by reporting false in app_shutdown_handler
        NRF_LOG_INFO("Device will enter bootloader mode.");
        break;

    case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
        NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
        // YOUR_JOB: Take corrective measures to resolve the issue
        //           like calling APP_ERROR_CHECK to reset the device.
        break;

    case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
        NRF_LOG_ERROR("Request to send a response to client failed.");
        // YOUR_JOB: Take corrective measures to resolve the issue
        //           like calling APP_ERROR_CHECK to reset the device.
        APP_ERROR_CHECK(false);
        break;

    default:
        NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
        break;
    }
}

/**@brief Function for initializing the DFU Service.
 */
static uint32_t dfu_init(void) {
    uint32_t err_code;
    ble_dfu_buttonless_init_t dfus_init = {0};

    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}
#endif // BLE_DFU_APP_SUPPORT

static uint32_t services_init(m_ble_service_handle_t *p_service_handles, uint32_t num_services) {
    uint32_t err_code;
    ble_dis_init_t dis_init_obj;

    // Initialize DIS
    memset(&dis_init_obj, 0, sizeof(ble_dis_init_t));
    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, (char *)ble_mgmt_param.mfg_name);
    ble_srv_ascii_to_utf8(&dis_init_obj.fw_rev_str, (char *)ble_mgmt_param.fw_version);
    dis_init_obj.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init_obj);
    VERIFY_SUCCESS(err_code);

    // Initialize other modules services
    for (uint32_t i = 0; i < num_services; i++) {
        if (p_service_handles[i].init_cb != NULL) {
            err_code = p_service_handles[i].init_cb();
            if (err_code != NRF_SUCCESS) {
                return err_code;
            }
        }
    }

#ifdef BLE_DFU_APP_SUPPORT
    // Initialize DFU service
    err_code = dfu_init();
    VERIFY_SUCCESS(err_code);
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
    ret_code_t err_code;

    switch (ble_adv_evt) {
    case BLE_ADV_EVT_FAST:
        NRF_LOG_INFO("Fast advertising.");
        break;

    case BLE_ADV_EVT_IDLE:

        break;

    default:
        break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static uint32_t advertising_init(void) {
    ret_code_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = 0;
    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    VERIFY_SUCCESS(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    return NRF_SUCCESS;
}

uint32_t m_ble_mgmt_init(ble_mgmt_init_t const *const p_params) {
    uint32_t err_code;

    err_code = param_check(p_params);
    VERIFY_SUCCESS(err_code);

    ble_mgmt_evt_handler = p_params->evt_handler;
    strncpy(ble_mgmt_param.fw_version, p_params->ble_mgmt_param.fw_version, MAX_FW_VERSION_SIZE);
    m_service_handles = p_params->p_service_handles;
    m_service_num = p_params->service_num;

#ifdef BLE_DFU_APP_SUPPORT
    // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    err_code = ble_dfu_buttonless_async_svci_init();
    VERIFY_SUCCESS(err_code);
#endif // BLE_DFU_APP_SUPPORT

    err_code = ble_stack_init();
    VERIFY_SUCCESS(err_code);

    err_code = gap_params_init(p_params->ble_mgmt_param.dev_name);
    VERIFY_SUCCESS(err_code);

    err_code = gatt_init();
    VERIFY_SUCCESS(err_code);

    err_code = conn_params_init();
    VERIFY_SUCCESS(err_code);

    err_code = qwr_init();
    VERIFY_SUCCESS(err_code);

    err_code = services_init(m_service_handles, m_service_num);
    VERIFY_SUCCESS(err_code);

    err_code = advertising_init();
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t m_ble_mgmt_start(uint16_t uuid) {
    ret_code_t err_code;
    ble_advdata_t adv_data, sr_data;

    // TODO Properly manage uuid parameter
    ble_uuid_t ble_range_uuid;
    ble_range_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN; // Assuming it is the first custom to be initialized
    ble_range_uuid.uuid = uuid;
    adv_uuids[0] = ble_range_uuid;

    memset(&adv_data, 0, sizeof(adv_data));
    adv_data.name_type = BLE_ADVDATA_FULL_NAME;
    adv_data.include_appearance = false;
    adv_data.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&sr_data, 0, sizeof(sr_data));
    sr_data.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    sr_data.uuids_complete.p_uuids = adv_uuids;

    err_code = ble_advertising_advdata_update(&m_advertising, &adv_data, &sr_data);
    VERIFY_SUCCESS(err_code);

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t m_ble_mgmt_stop(void) {

    // Disable Advertising
    (void)sd_ble_gap_adv_stop(m_advertising.adv_handle);

    // Disconnect from remote device
    if (m_conn_handle != NULL)
        sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

    return NRF_SUCCESS;
}