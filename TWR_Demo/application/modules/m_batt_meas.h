/******************************************************************************
 * @file    m_batt_meas.h
 * @author  Insight SiP
 * @brief   Battery measurement module header file.
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

#ifndef __M_BATT_MEAS_H__
#define __M_BATT_MEAS_H__

#include "m_ble_mgmt.h"
#include <stdbool.h>
#include <stdint.h>

#define MEAS_INTERVAL_LOW_LIMIT_MS 100

/** @brief Battery and charge event codes.
 */
typedef enum {
    M_BATT_MEAS_EVENT_DATA,  /**< New battery SoC available.  */
    M_BATT_MEAS_EVENT_ERROR, /**< Error state detected signalled by the charger (not implemeted, CHG and CHG finished will toggle in case of error).  */
} m_batt_meas_event_type_t;

/** @brief The struct passed to the handler with relevant battery information.
 */
typedef struct
{
    m_batt_meas_event_type_t type; /**< Given event type.  */
    uint16_t voltage_mv;           /**< Battery voltage given in millivolts.  */
    uint8_t level_percent;         /**< Remaining battery capacity percent.  */
} m_batt_meas_event_t;

/** @brief m_batt sensor event handler type. Should be implemented by user e.g. in main()
 */
typedef void (*m_batt_meas_event_handler_t)(m_batt_meas_event_t const *p_event);

/** @brief Input parameters for m_batt_meas_init.
 */
typedef struct
{
    uint32_t pin_batt; /**< Pin connected to SAADC */
    uint32_t r1_ohm;   /**< Voltage divider resistor r1 (situated between the battery and adc_pin) */
    uint32_t r2_ohm;   /**< Voltage divider resistor r2 (connects adc_pin and GND) */
} batt_meas_param_t;

/** @brief Init parameters for m_batt_meas_init.
 */
typedef struct
{
    m_batt_meas_event_handler_t evt_handler; ///< Function pointer to the event handler
    batt_meas_param_t batt_meas_param;       ///< Input parameters.
} batt_meas_init_t;

/**@brief Initalizes the battery driver.
 *
 * @param[in]  p_batt_meas_init     Struct containing the configuration parameters.
 * @param[out] p_handle    Pointer to the service handle.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
uint32_t m_batt_meas_init(batt_meas_init_t const *const p_batt_meas_init, m_ble_service_handle_t *p_handle);

/**@brief Enables battery measurement at the given interval.
 *
 * @param meas_interval_ms  Sampling interval given in milliseconds.
 *
 * @note This will call the handler supplied in p_batt_meas_init at the given interval
 * which is supplied with a m_batt_meas_event_t struct containing the information.
 *
 * @retval NRF_SUCCESS If enabling was successful.
 */
uint32_t m_batt_meas_start(uint32_t meas_interval_ms);

/**@brief Stops the battery measurement.
 *
 * @retval NRF_SUCCESS If disabling was successful.
 */
uint32_t m_batt_meas_stop(void);

#endif