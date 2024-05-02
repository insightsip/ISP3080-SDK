/******************************************************************************
 * @file    main.c
 * @author  Insight SiP
 * @brief   TWR Demo
 *
 * @attention
 *    THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *    OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *    DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *    GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "boards.h"
#include "m_batt_meas.h"
#include "m_ble_mgmt.h"
#include "m_uwb_range.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdm.h"
#include <stdint.h>
#include <string.h>

APP_TIMER_DEF(m_repeated_timer_id); 

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, NRF_SDH_BLE_GATT_MAX_MTU_SIZE) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                                                                           /**< Maximum number of events in the scheduler queue. */

#define MAX_BLE_SERVICES 2
#define SERVICE_UWB_RANGE 0
#define SERVICE_BATT_MEAS 1
#define BATTERY_MEAS_INTERVAL_MS 10000
#define FW_REVISION "1.0.2"             /**< Version. */
#define MANUFACTURER_NAME "Insight SiP" /**< Manufacturer. */
#if defined(BOARD_ISP3080_UX_TG)
#define DEVICE_NAME "ISP3080-UX-TG" /**< Name of device. Will be included in the advertising data. */
#elif defined(BOARD_ISP3080_UX_AN)
#define DEVICE_NAME "ISP3080-UX-AN" /**< Name of device. Will be included in the advertising data. */
#endif

static bool m_ready_for_reset = true;
static uint16_t adv_uuid;
static m_ble_service_handle_t m_ble_service_handles[MAX_BLE_SERVICES];

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
    error_info_t *err_info = (error_info_t *)info;
    NRF_LOG_ERROR(" id = %d, pc = %d, file = %s, line number: %d, error code = %d = %s",
        id, pc, nrf_log_push((char *)err_info->p_file_name), err_info->line_num, err_info->err_code, nrf_log_push((char *)nrf_strerror_find(err_info->err_code)));
    NRF_LOG_FINAL_FLUSH();
    nrf_delay_ms(5);

    // On assert, the system can only recover with a reset.
#ifndef DEBUG
    NVIC_SystemReset();
#endif

    app_error_save_and_stop(id, pc, info);
}

#ifdef BLE_DFU_APP_SUPPORT
/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event) {
    switch (event) {
    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
        NRF_LOG_INFO("Power management wants to reset to DFU mode.");
        // Get ready to reset into DFU mode
        if (!m_ready_for_reset) {
            return false;
        } else {
            // Device ready to enter DFU
            uint32_t err_code;

            err_code = m_range_stop();
            APP_ERROR_CHECK(err_code);
#if defined(BOARD_ISP3080_UX_TG)
            err_code = m_batt_meas_stop();
            APP_ERROR_CHECK(err_code);
#endif
            err_code = m_ble_mgmt_stop();
            APP_ERROR_CHECK(err_code);
            err_code = app_timer_stop_all();
            APP_ERROR_CHECK(err_code);
        }

        break;

    default:
        // Implement any of the other events available from the power management module:
        //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
        //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
        //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
        return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);
#endif // BLE_DFU_APP_SUPPORT

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void) {
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE - 1 - (SCHED_MAX_EVENT_DATA_SIZE - 1) % 4 + 4, SCHED_QUEUE_SIZE);

    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
    if (NRF_LOG_PROCESS() == false) {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for handling battery measurement events.

 * @param[in] p_event  batt_meas event.
 */
void batt_meas_event_handler(m_batt_meas_event_t const *p_event) {
    if (p_event->type == M_BATT_MEAS_EVENT_DATA) {
        NRF_LOG_INFO("batt_meas_event_handler : U = %d mV", p_event->voltage_mv);
    }
}

/**@brief Function for initializing the battery measurement module
 */
void batt_meas_module_init(void) {
    uint32_t err_code;

    batt_meas_param_t batt_meas_param;
    batt_meas_param.pin_batt = PIN_BATT_MEAS;
    batt_meas_param.r1_ohm = BATT_VOLTAGE_DIVIDER_R1;
    batt_meas_param.r2_ohm = BATT_VOLTAGE_DIVIDER_R2;

    batt_meas_init_t batt_meas_init;
    batt_meas_init.evt_handler = batt_meas_event_handler;
    batt_meas_init.batt_meas_param = batt_meas_param;

    err_code = m_batt_meas_init(&batt_meas_init, &m_ble_service_handles[SERVICE_BATT_MEAS]);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the uwb range module
 */
void uwb_range_module_init(void) {
    uint32_t err_code;
    m_uwb_range_init_t uwb_range_init;

#if defined(BOARD_ISP3080_UX_TG)
    uwb_range_init.role = TWR_INITIATOR;
#elif defined(BOARD_ISP3080_UX_AN)
    uwb_range_init.role = TWR_RESPONDER;
#endif

    err_code = m_range_init(&uwb_range_init, &m_ble_service_handles[SERVICE_UWB_RANGE]);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling ble mgmt events.

 * @param[in] p_event  ble_mgmt event.
 */
void ble_mgmt_event_handler(m_ble_mgmt_event_t const *p_event) {
    switch (p_event->type) {
    case M_BLE_MGMT_EVENT_CONNECTED:
#if defined(BOARD_ISP3080_UX_TG)
        m_batt_meas_start(BATTERY_MEAS_INTERVAL_MS);
#endif
        break;

    case M_BLE_MGMT_EVENT_DISCONNECTED:
#if defined(BOARD_ISP3080_UX_TG)
        m_batt_meas_stop();
#endif
        m_ble_mgmt_start(adv_uuid);
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for initializing the ble management module
 */
void ble_mgmt_module_init(void) {
    uint32_t err_code;
    ble_mgmt_init_t ble_init;

    ble_init.evt_handler = ble_mgmt_event_handler;
    strncpy(ble_init.ble_mgmt_param.fw_version, FW_REVISION, MAX_FW_VERSION_SIZE);
    strncpy(ble_init.ble_mgmt_param.mfg_name, MANUFACTURER_NAME, MAX_MANUFACTURER_NAME_SIZE);
    strncpy(ble_init.ble_mgmt_param.dev_name, DEVICE_NAME, MAX_DEVICE_NAME_SIZE);
    ble_init.p_service_handles = m_ble_service_handles;
    ble_init.service_num = MAX_BLE_SERVICES;

    err_code = m_ble_mgmt_init(&ble_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */

static void repeated_timer_handler(void * p_context)
{
    static float accel_data[3] = {0};
    //nrf_drv_gpiote_out_toggle(LED_1);
    // drv_acc_get(data_l);
    //NRF_LOG_INFO("timer expired: %d", count);
    NRF_LOG_INFO("x = " NRF_LOG_FLOAT_MARKER " m/s*s", NRF_LOG_FLOAT(accel_data[0]));
    NRF_LOG_INFO("y = " NRF_LOG_FLOAT_MARKER " m/s*s", NRF_LOG_FLOAT(accel_data[1]));
    NRF_LOG_INFO("z = " NRF_LOG_FLOAT_MARKER " m/s*s", NRF_LOG_FLOAT(accel_data[2]));
    //count+= 0.1;
}

int main(void) {

    uint32_t err_code;
    

    // Initialize Logs, Power & Timer.
    log_init();
    timers_init();
    power_management_init();

    err_code = app_timer_create(&m_repeated_timer_id,
                        APP_TIMER_MODE_REPEATED,
                        repeated_timer_handler);

    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_repeated_timer_id, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);

/*
    // Initialize modules
#if defined(BOARD_ISP3080_UX_TG)
    batt_meas_module_init();
#endif
    uwb_range_module_init();
    ble_mgmt_module_init(); // To be called last

    // Start modules.
    m_range_ble_uuid_get(&adv_uuid);
    m_ble_mgmt_start(adv_uuid);

    // Set current mode pin before starting ranging operation (only for TAGs).
    if (PIN_CURR_MODE != 255) {
        nrf_gpio_pin_set(PIN_CURR_MODE);
        nrf_gpio_cfg_output(PIN_CURR_MODE);
    }
    m_range_start();
*/




    // Enter main loop.
    for (;;) {
        app_sched_execute();
        idle_state_handle();
    }
}