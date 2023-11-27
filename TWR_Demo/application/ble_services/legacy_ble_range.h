/******************************************************************************
 * @file    legacy_ble_range.h
 * @author  Insight SiP
 * @brief   Range ble service header
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

#if defined(USE_LEGACY_BLE_RANGE_SERVICE)

#ifndef LEGACY_BLE_RANGE_H__
#define LEGACY_BLE_RANGE_H__

#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include <stdbool.h>
#include <stdint.h>

/**@brief Macro for defining a ble_range instance.
 *
 * @param   _name  Name of the instance.
 * @hideinitializer
 */
#define BLE_RANGE_DEF(_name)          \
    static ble_range_t _name;         \
    NRF_SDH_BLE_OBSERVER(_name##_obs, \
        2,                            \
        ble_range_on_ble_evt,         \
        &_name)

// 0011xxxx-4455-6677-8899-AABBCCDDEEFF
#define RANGE_BASE_UUID \
    { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF } /**< Used vendor specific UUID. */
#define BLE_UUID_RANGE_SERVICE 0x1100                                                                  /**< The UUID of the Range Service. */
#define BLE_RANGE_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3)                                          /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Range service module. */

#ifdef __GNUC__
#ifdef PACKED
#undef PACKED
#endif

#define PACKED(TYPE) TYPE __attribute__((packed))
#endif

typedef double ble_range_data_t;
typedef uint16_t ble_range_period_config_t;
typedef uint16_t ble_range_uwb_config_t;
typedef uint8_t ble_range_mode_t;
typedef uint8_t ble_range_avg_config_t;
typedef uint32_t ble_range_threshold_config_t;
typedef uint32_t ble_range_controls_t;
typedef uint8_t ble_range_status_t;

#define BLE_RANGE_CONFIG_RANGE_INT_MIN 50
#define BLE_RANGE_CONFIG_RANGE_INT_MAX 5000

/**@brief Range Service event type. */
typedef enum {
    BLE_RANGE_EVT_INST_NOTIFICATION_ENABLED,  /**< Range value notification enabled event. */
    BLE_RANGE_EVT_INST_NOTIFICATION_DISABLED, /**< Range value notification enabled event. */
    BLE_RANGE_EVT_AVG_NOTIFICATION_ENABLED,   /**< Range value notification enabled event. */
    BLE_RANGE_EVT_AVG_NOTIFICATION_DISABLED,  /**< Range value notification disabled event. */
    BLE_RANGE_EVT_PERIOD_CHANGED,             /**< Period changed event. */
    BLE_RANGE_EVT_UWB_CHANGED,                /**< Channel changed event. */
    BLE_RANGE_EVT_MODE_CHANGED,               /**< Mode changed event. */
    BLE_RANGE_EVT_AVG_CHANGED,                /**< Average changed event. */
    BLE_RANGE_EVT_THRESHOLD_CHANGED,          /**< Thresholds changed event. */
    BLE_RANGE_EVT_CONTROLS_CHANGED            /**< Controls changed event. */
} ble_range_evt_type_t;

/**@brief Range Service event. */
typedef struct
{
    ble_range_evt_type_t evt_type; /**< Type of event. */
    uint8_t const *p_data;         /**< A pointer to the buffer with received data. */
    uint16_t length;               /**< Length of received data. */
} ble_range_evt_t;

/* Forward declaration of the ble_range_t type. */
typedef struct ble_range_s ble_range_t;

/**@brief Range Service event handler type. */
typedef void (*ble_range_evt_handler_t)(ble_range_t *p_range, ble_range_evt_t *p_evt);

/**@brief Range Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_range_init function.
 */
typedef struct
{
    ble_range_data_t *p_init_inst_data;
    ble_range_data_t *p_init_avg_data;
    ble_range_period_config_t *p_init_period_config;
    ble_range_uwb_config_t *p_init_uwb_config;
    ble_range_mode_t *p_init_mode;
    ble_range_avg_config_t *p_init_avg_config;
    ble_range_threshold_config_t *p_init_threshold_config;
    ble_range_controls_t *p_init_controls;
    ble_range_status_t *p_init_status;
    ble_range_evt_handler_t evt_handler; /**< Event handler to be called for handling received data. */
} ble_range_init_t;

/**@brief Range Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_range_s {
    uint8_t uuid_type;                                 /**< UUID type for Range Service Base UUID. */
    uint16_t service_handle;                           /**< Handle of Range Service (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t inst_data_handles;        /**< Handles related to the instant data characteristic. */
    ble_gatts_char_handles_t avg_data_handles;         /**< Handles related to the average data characteristic. */
    ble_gatts_char_handles_t period_config_handles;    /**< Handles related to the period configuration characteristic. */
    ble_gatts_char_handles_t uwb_config_handles;       /**< Handles related to the uwb configuration characteristic. */
    ble_gatts_char_handles_t mode_handles;             /**< Handles related to the mode characteristic. */
    ble_gatts_char_handles_t avg_config_handles;       /**< Handles related to the avg configuration characteristic. */
    ble_gatts_char_handles_t threshold_config_handles; /**< Handles related to the threshold configuration characteristic. */
    ble_gatts_char_handles_t status_handles;           /**< Handles related to the status characteristic. */
    ble_gatts_char_handles_t controls_handles;         /**< Handles related to the controls characteristic. */
    uint16_t conn_handle;                              /**< Handle of the current connection (as provided by the S110 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool is_inst_data_notif_enabled;                   /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_avg_data_notif_enabled;                    /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_status_notif_enabled;                      /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    ble_range_evt_handler_t evt_handler;               /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the Range Service.
 *
 * @param[out] p_range      Range Service structure. This structure must be supplied
 *                          by the application. It is initialized by this function and will
 *                          later be used to identify this particular service instance.
 * @param[in] p_range_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_tes or p_tes_init is NULL.
 */
uint32_t ble_range_init(ble_range_t *p_range, const ble_range_init_t *p_range_init);

/**@brief Function for handling the Range Service's BLE events.
 *
 * @details The Range Service expects the application to call this function each time an
 * event is received from the S132 SoftDevice. This function processes the event if it
 * is relevant and calls the Range Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     Range Service structure.
 */
void ble_range_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

/**@brief Function for setting the instant range.
 *
 * @details This function sends the input range as a range characteristic notification to the peer.
 *
 * @param[in] p_range       Pointer to the Range Service structure.
 * @param[in] p_data        Pointer to the range data.
 * @param[in] conn_handle   Connection handle.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_range_inst_data_set(ble_range_t *p_range, ble_range_data_t *p_data);

/**@brief Function for setting the average range.
 *
 * @details This function sends the input range as a range characteristic notification to the peer.
 *
 * @param[in] p_range       Pointer to the Range Service structure.
 * @param[in] p_data        Pointer to the range data.
 * @param[in] conn_handle   Connection handle.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_range_avg_data_set(ble_range_t *p_range, ble_range_data_t *p_data);

/**@brief Function for setting the status.
 *
 * @details This function sends the input status as a status characteristic notification to the peer.
 *
 * @param[in] p_range           Pointer to the Range Service structure.
 * @param[in] p_data            Pointer to the status data.
 * @param[in] conn_handle       Connection handle.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_range_status_set(ble_range_t *p_range, ble_range_status_t *p_data);

#endif // LEGACY_BLE_RANGE_H__

#endif // USE_LEGACY_BLE_RANGE_SERVICE