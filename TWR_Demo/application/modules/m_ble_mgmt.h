/******************************************************************************
 * @file    m_ble_mgmt.h
 * @author  Insight SiP
 * @brief   BLE management module header file.
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

#ifndef __M_BLE_MGMT_H__
#define __M_BLE_MGMT_H__

#include <stdbool.h>
#include <stdint.h>

#define MAX_FW_VERSION_SIZE 10
#define MAX_MANUFACTURER_NAME_SIZE 15
#define MAX_DEVICE_NAME_SIZE 15

/** @brief Battery and charge event codes.
 */
typedef enum {
    M_BLE_MGMT_EVENT_CONNECTED,    /**< BLE device connected. */
    M_BLE_MGMT_EVENT_DISCONNECTED, /**< BLE device disconnected. */
    M_BLE_MGMT_EVENT_ERROR,        /**< Error state detected.  */
} m_ble_mgmt_event_type_t;

/** @brief The struct passed to the handler with relevant battery information.
 */
typedef struct
{
    m_ble_mgmt_event_type_t type; /**< Given event type.  */

} m_ble_mgmt_event_t;

/** @brief m_ble_mgmt  event handler type. Should be implemented by user e.g. in main()
 */
typedef void (*m_ble_mgmt_event_handler_t)(m_ble_mgmt_event_t const *p_event);

/**@brief  BLE service callback definitions.
*/
typedef uint32_t (*m_ble_service_init_cb_t)(void);

/** @brief Input parameters for m_batt_meas_init.
 */
typedef struct
{
    uint8_t fw_version[MAX_FW_VERSION_SIZE];      /**< Firmware version */
    uint8_t mfg_name[MAX_MANUFACTURER_NAME_SIZE]; /**< Manufacturer name */
    uint8_t dev_name[MAX_DEVICE_NAME_SIZE];       /**< Device name */
} m_ble_mgmt_param_t;

/**@brief BLE service handle structure.
*/
typedef struct
{
    m_ble_service_init_cb_t init_cb;
} m_ble_service_handle_t;

/** @brief Init parameters for m_ble_mgmt_init.
 */
typedef struct
{
    m_ble_mgmt_event_handler_t evt_handler; ///< Function pointer to the event handler
    m_ble_service_handle_t *p_service_handles; 
    uint32_t service_num;
    m_ble_mgmt_param_t ble_mgmt_param; ///< Input parameters.
} ble_mgmt_init_t;

/**@brief Initalizes BLE.
 *
 * @param[out] p_handle             Pointer to the location to store the service handle.
 * @param[in]  p_ble_mgmt_init      Struct containing the configuration parameters.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t m_ble_mgmt_init(ble_mgmt_init_t const *const p_ble_mgmt_init);

/**@brief Enable BLE.
 *
 * @param[in]  uuid uuid to display in the scan response.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t m_ble_mgmt_start(uint16_t uuid);

/**@brief Disable BLE.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t m_ble_mgmt_stop(void);

#endif