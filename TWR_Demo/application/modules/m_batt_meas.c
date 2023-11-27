/******************************************************************************
 * @file    m_batt_meas.c
 * @author  Insight SiP
 * @brief   Battery measurement module implementation file.
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
#include "ble_bas.h"
#include "boards.h"
#include "m_batt_meas.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrfx_saadc.h"
#include "sdk_config.h"

#define NRF_LOG_MODULE_NAME m_batt_meas
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define SAMPLES_IN_BUFFER 5
#define INVALID_BATTERY_LEVEL (0xFF) /**<  Invalid/default battery level. */

BLE_BAS_DEF(m_bas);                    /**< Structure used to identify the battery service. */
APP_TIMER_DEF(batt_meas_app_timer_id); /**< Timer for periodic battery measurement. */
static nrf_saadc_value_t m_buffer_pool[2][SAMPLES_IN_BUFFER];
static m_batt_meas_event_handler_t m_evt_handler;                    /**< Battery event handler function pointer. */
static batt_meas_param_t m_batt_meas_param;                          /**< Battery parameters. */
static uint8_t m_initial_batt_level_percent = INVALID_BATTERY_LEVEL; /**< Initial battery level in percent. */
static float battery_divider_factor;

/** @brief Function for converting digital pin number to analog pin number
 *
 * @param[in]  digital_pin     Digital pin number
 *
 * @return Analog pin number
 */
static nrf_saadc_input_t digital_to_analog_pin(uint32_t digital_pin) {
    nrf_saadc_input_t analog_pin;

    switch (digital_pin) {
    case 2:
        analog_pin = NRF_SAADC_INPUT_AIN0;
        break;
    case 3:
        analog_pin = NRF_SAADC_INPUT_AIN1;
        break;
    case 4:
        analog_pin = NRF_SAADC_INPUT_AIN2;
        break;
    case 5:
        analog_pin = NRF_SAADC_INPUT_AIN3;
        break;
    case 28:
        analog_pin = NRF_SAADC_INPUT_AIN4;
        break;
    case 29:
        analog_pin = NRF_SAADC_INPUT_AIN5;
        break;
    case 30:
        analog_pin = NRF_SAADC_INPUT_AIN6;
        break;
    case 31:
        analog_pin = NRF_SAADC_INPUT_AIN7;
        break;
    default:
        analog_pin = NRF_SAADC_INPUT_VDD;
    }

    return analog_pin;
}

void saadc_callback(nrfx_saadc_evt_t const *p_event) {
    if (p_event->type == NRFX_SAADC_EVT_DONE) {
        ret_code_t err_code;
        uint16_t soc;
        uint16_t voltage_mv;

        nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);

        // Compute mean value
        int32_t saadc_result = 0;
        for (int i = 0; i < SAMPLES_IN_BUFFER; i++) {
            saadc_result += p_event->data.done.p_buffer[i];
        }
        saadc_result /= SAMPLES_IN_BUFFER;
        if (saadc_result < 0)
            saadc_result = 0;

        // convert to voltage
        voltage_mv = (saadc_result * 3600 / 1024); // Voltage read by saadc, Input range = (0.6 V)/(1/6) = 3.6 V, resulution 10bits
        voltage_mv /= battery_divider_factor;      // Compensate the voltage divider

        // Convert to SoC
        soc = battery_level_in_percent(voltage_mv);

        // Update BLE characteristic
        err_code = ble_bas_battery_level_update(&m_bas, soc, BLE_CONN_HANDLE_ALL);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
            // APP_ERROR_HANDLER(err_code);
        } else {
            m_initial_batt_level_percent = soc;
        }

        // Call Battery event handler
        m_batt_meas_event_t batt_meas_evt;
        batt_meas_evt.voltage_mv = voltage_mv;
        batt_meas_evt.level_percent = soc;
        batt_meas_evt.type = M_BATT_MEAS_EVENT_DATA;
        m_evt_handler(&batt_meas_evt);
    }
}

/**@brief Initalizes SAADC for battery measurement
 *
 * @return NRF_SUCCESS
 * @return Other codes from the underlying driver.
 */
static uint32_t saadc_init(void) {
    uint32_t err_code;

    static const nrfx_saadc_config_t default_config = NRFX_SAADC_DEFAULT_CONFIG;
    err_code = nrfx_saadc_init(&default_config, saadc_callback);
    VERIFY_SUCCESS(err_code);

    nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(digital_to_analog_pin(m_batt_meas_param.pin_batt));
    channel_config.acq_time = NRF_SAADC_ACQTIME_15US;
    err_code = nrfx_saadc_channel_init(0, &channel_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    VERIFY_SUCCESS(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/** @brief Periodic timer handler.
 */
static void app_timer_periodic_handler(void *unused) {
    nrfx_saadc_sample();
}

/** @brief Checks validity of supplied parameters.
 */
static uint32_t param_check(batt_meas_init_t const *const p_batt_meas_init) {
    VERIFY_PARAM_NOT_NULL(p_batt_meas_init);
    VERIFY_PARAM_NOT_NULL(p_batt_meas_init->evt_handler);

    if ((p_batt_meas_init->batt_meas_param.r1_ohm == 0) &&
        (p_batt_meas_init->batt_meas_param.r2_ohm == 0)) {
        battery_divider_factor = 1;
    } else if ((p_batt_meas_init->batt_meas_param.r1_ohm == 0) ||
               (p_batt_meas_init->batt_meas_param.r2_ohm == 0)) {
        return NRF_ERROR_INVALID_PARAM;
    } else {
        battery_divider_factor = p_batt_meas_init->batt_meas_param.r2_ohm /
                                 (float)(p_batt_meas_init->batt_meas_param.r1_ohm +
                                         p_batt_meas_init->batt_meas_param.r2_ohm);
    }

    return NRF_SUCCESS;
}

/**@brief Event handler, handles events in the Battery Service.
 *
 * @details This callback function is often used to enable a service when requested over BLE,
 * and disable when not requested to save power. 
 */
static void ble_bas_evt_handler(ble_bas_t *p_bas, ble_bas_evt_t *p_evt) {
    switch (p_evt->evt_type) {
    case BLE_BAS_EVT_NOTIFICATION_ENABLED:
        NRF_LOG_DEBUG("BLE_BAS_EVT_NOTIFICATION_ENABLED");
        break;

    case BLE_BAS_EVT_NOTIFICATION_DISABLED:
        NRF_LOG_DEBUG("BLE_BAS_EVT_NOTIFICATION_DISABLED");
        break;

    default:
        break;
    }
}

/**@brief Function for initializing the Battery Service.
 *
 * @details This callback function will be called from the ble handling module to initialize the Battery service.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
static uint32_t battery_service_init(void) {
    uint32_t err_code;
    ble_bas_init_t bas_init;

    memset(&bas_init, 0, sizeof(bas_init));

    // Security level for the Battery Service
    bas_init.bl_report_rd_sec = SEC_OPEN;
    bas_init.bl_cccd_wr_sec = SEC_OPEN;
    bas_init.bl_rd_sec = SEC_OPEN;

    bas_init.evt_handler = ble_bas_evt_handler;
    bas_init.support_notification = true;
    bas_init.p_report_ref = NULL;
    bas_init.initial_batt_level = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t m_batt_meas_start(uint32_t meas_interval_ms) {
    uint32_t err_code;

    if (meas_interval_ms < MEAS_INTERVAL_LOW_LIMIT_MS) {
        return NRF_ERROR_INVALID_PARAM;
    }
    nrfx_saadc_sample();

    err_code = app_timer_start(batt_meas_app_timer_id, APP_TIMER_TICKS(meas_interval_ms), NULL);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t m_batt_meas_stop(void) {
    uint32_t err_code;

    err_code = app_timer_stop(batt_meas_app_timer_id);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t m_batt_meas_init(batt_meas_init_t const *const p_batt_meas_init, m_ble_service_handle_t *p_handle) {
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_batt_meas_init);

    err_code = param_check(p_batt_meas_init);
    VERIFY_SUCCESS(err_code);

    m_evt_handler = p_batt_meas_init->evt_handler;
    m_batt_meas_param = p_batt_meas_init->batt_meas_param;
    p_handle->init_cb = battery_service_init;

    // Initialize SAADC for battery measurement
    err_code = saadc_init();
    VERIFY_SUCCESS(err_code);

    // Initialize timer
    err_code = app_timer_create(&batt_meas_app_timer_id, APP_TIMER_MODE_REPEATED, app_timer_periodic_handler);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}