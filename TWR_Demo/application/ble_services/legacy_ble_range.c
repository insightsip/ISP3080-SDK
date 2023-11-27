/******************************************************************************
 * @file    legacy_ble_range.c
 * @author  Insight SiP
 * @brief   Range ble service implementation
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

#include "legacy_ble_range.h"
#include "app_error.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

#define BLE_UUID_RANGE_INST_DATA_CHAR 0x1101               /**< UWB Range instant range characteristic UUID. */
#define BLE_UUID_RANGE_AVG_DATA_CHAR 0x1102                /**< UWB Range average range characteristic UUID. */
#define BLE_UUID_RANGE_PERIOD_CONFIGURATION_CHAR 0x1103    /**< UWB Range period configuration characteristic UUID. */
#define BLE_UUID_RANGE_UWB_CONFIGURATION_CHAR 0x1104       /**< UWB Range signal configuration characteristic UUID. */
#define BLE_UUID_RANGE_MODE_CONFIGURATION_CHAR 0x1105      /**< UWB Range role configuration characteristic UUID. */
#define BLE_UUID_RANGE_AVG_CONFIGURATION_CHAR 0x1106       /**< UWB Range average configuration characteristic UUID. */
#define BLE_UUID_RANGE_THRESHOLD_CONFIGURATION_CHAR 0x1107 /**< UWB Range threshold configuration characteristic UUID. */
#define BLE_UUID_RANGE_STATUS_CHAR 0x1108                  /**< UWB Range status characteristic UUID. */
#define BLE_UUID_RANGE_CONTROLS_CHAR 0x1109                /**< UWB Range controls characteristic UUID. */

#define BLE_RANGE_MAX_RX_CHAR_LEN BLE_RANGE_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_RANGE_MAX_TX_CHAR_LEN BLE_RANGE_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_range       Range Service structure.
 * @param[in] p_ble_evt     Pointer to the event received from BLE stack.
 */
static void on_connect(ble_range_t *p_range, ble_evt_t const *p_ble_evt) {
    p_range->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_range     	Range Service structure.
 * @param[in] p_ble_evt 	Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_range_t *p_range, ble_evt_t const *p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);
    p_range->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S132 SoftDevice.
 *
 * @param[in] p_range       Range Service structure.
 * @param[in] p_ble_evt     Pointer to the event received from BLE stack.
 */
static void on_write(ble_range_t *p_range, ble_evt_t const *p_ble_evt) {
    ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    ble_range_evt_t evt;

    if (p_evt_write->handle == p_range->period_config_handles.value_handle) {
        evt.evt_type = BLE_RANGE_EVT_PERIOD_CHANGED;
        evt.p_data = p_evt_write->data;
        evt.length = p_evt_write->len;
        p_range->evt_handler(p_range, &evt);
    } else if (p_evt_write->handle == p_range->uwb_config_handles.value_handle) {
        evt.evt_type = BLE_RANGE_EVT_UWB_CHANGED;
        evt.p_data = p_evt_write->data;
        evt.length = p_evt_write->len;
        p_range->evt_handler(p_range, &evt);
    } else if (p_evt_write->handle == p_range->mode_handles.value_handle) {
        evt.evt_type = BLE_RANGE_EVT_MODE_CHANGED;
        evt.p_data = p_evt_write->data;
        evt.length = p_evt_write->len;
        p_range->evt_handler(p_range, &evt);
    } else if (p_evt_write->handle == p_range->avg_config_handles.value_handle) {
        evt.evt_type = BLE_RANGE_EVT_AVG_CHANGED;
        evt.p_data = p_evt_write->data;
        evt.length = p_evt_write->len;
        p_range->evt_handler(p_range, &evt);
    } else if (p_evt_write->handle == p_range->threshold_config_handles.value_handle) {
        evt.evt_type = BLE_RANGE_EVT_THRESHOLD_CHANGED;
        evt.p_data = p_evt_write->data;
        evt.length = p_evt_write->len;
        p_range->evt_handler(p_range, &evt);
    } else if (p_evt_write->handle == p_range->controls_handles.value_handle) {
        evt.evt_type = BLE_RANGE_EVT_CONTROLS_CHANGED;
        evt.p_data = p_evt_write->data;
        evt.length = p_evt_write->len;
        p_range->evt_handler(p_range, &evt);
    } else if ((p_evt_write->handle == p_range->inst_data_handles.cccd_handle) && (p_evt_write->len == 2)) {
        p_range->is_inst_data_notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
    } else if ((p_evt_write->handle == p_range->avg_data_handles.cccd_handle) && (p_evt_write->len == 2)) {
        p_range->is_avg_data_notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
    } else if ((p_evt_write->handle == p_range->status_handles.cccd_handle) && (p_evt_write->len == 2)) {
        p_range->is_status_notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
    }
}

/**@brief Function for adding inst data characteristic.
 *
 * @param[in] p_range       Range Service structure.
 * @param[in] p_range_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t inst_data_char_add(ble_range_t *p_range, const ble_range_init_t *p_range_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_range->uuid_type;
    ble_uuid.uuid = BLE_UUID_RANGE_INST_DATA_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_range_data_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_range_init->p_init_inst_data;
    attr_char_value.max_len = sizeof(ble_range_data_t);

    return sd_ble_gatts_characteristic_add(p_range->service_handle,
        &char_md,
        &attr_char_value,
        &p_range->inst_data_handles);
}

/**@brief Function for adding avg data characteristic.
 *
 * @param[in] p_range       Range Service structure.
 * @param[in] p_range_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t avg_data_char_add(ble_range_t *p_range, const ble_range_init_t *p_range_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_range->uuid_type;
    ble_uuid.uuid = BLE_UUID_RANGE_AVG_DATA_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_range_data_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_range_init->p_init_avg_data;
    attr_char_value.max_len = sizeof(ble_range_data_t);

    return sd_ble_gatts_characteristic_add(p_range->service_handle,
        &char_md,
        &attr_char_value,
        &p_range->avg_data_handles);
}

/**@brief Function for adding period configuration characteristic.
 *
 * @param[in] p_range       Range Service structure.
 * @param[in] p_range_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t period_config_char_add(ble_range_t *p_range, const ble_range_init_t *p_range_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_range->uuid_type;
    ble_uuid.uuid = BLE_UUID_RANGE_PERIOD_CONFIGURATION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_range_period_config_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_range_init->p_init_period_config;
    attr_char_value.max_len = sizeof(ble_range_period_config_t);

    return sd_ble_gatts_characteristic_add(p_range->service_handle,
        &char_md,
        &attr_char_value,
        &p_range->period_config_handles);
}

/**@brief Function for adding uwb configuration characteristic.
 *
 * @param[in] p_range       Range Service structure.
 * @param[in] p_range_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t uwb_config_char_add(ble_range_t *p_range, const ble_range_init_t *p_range_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_range->uuid_type;
    ble_uuid.uuid = BLE_UUID_RANGE_UWB_CONFIGURATION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_range_uwb_config_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_range_init->p_init_uwb_config;
    attr_char_value.max_len = sizeof(ble_range_uwb_config_t);

    return sd_ble_gatts_characteristic_add(p_range->service_handle,
        &char_md,
        &attr_char_value,
        &p_range->uwb_config_handles);
}

/**@brief Function for adding mode configuration characteristic.
 *
 * @param[in] p_range       Range Service structure.
 * @param[in] p_range_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t mode_config_char_add(ble_range_t *p_range, const ble_range_init_t *p_range_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_range->uuid_type;
    ble_uuid.uuid = BLE_UUID_RANGE_MODE_CONFIGURATION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_range_mode_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_range_init->p_init_mode;
    attr_char_value.max_len = sizeof(ble_range_mode_t);

    return sd_ble_gatts_characteristic_add(p_range->service_handle,
        &char_md,
        &attr_char_value,
        &p_range->mode_handles);
}

/**@brief Function for adding avg configuration characteristic.
 *
 * @param[in] p_range       Range Service structure.
 * @param[in] p_range_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t avg_config_char_add(ble_range_t *p_range, const ble_range_init_t *p_range_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_range->uuid_type;
    ble_uuid.uuid = BLE_UUID_RANGE_AVG_CONFIGURATION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_range_avg_config_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_range_init->p_init_avg_config;
    attr_char_value.max_len = sizeof(ble_range_avg_config_t);

    return sd_ble_gatts_characteristic_add(p_range->service_handle,
        &char_md,
        &attr_char_value,
        &p_range->avg_config_handles);
}

/**@brief Function for adding threshold configuration characteristic.
 *
 * @param[in] p_range       Range Service structure.
 * @param[in] p_range_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t threshold_config_char_add(ble_range_t *p_range, const ble_range_init_t *p_range_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_range->uuid_type;
    ble_uuid.uuid = BLE_UUID_RANGE_THRESHOLD_CONFIGURATION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_range_threshold_config_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_range_init->p_init_threshold_config;
    attr_char_value.max_len = sizeof(ble_range_threshold_config_t);

    return sd_ble_gatts_characteristic_add(p_range->service_handle,
        &char_md,
        &attr_char_value,
        &p_range->threshold_config_handles);
}

/**@brief Function for adding controls characteristic.
 *
 * @param[in] p_range       Range Service structure.
 * @param[in] p_range_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t controls_char_add(ble_range_t *p_range, const ble_range_init_t *p_range_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_range->uuid_type;
    ble_uuid.uuid = BLE_UUID_RANGE_CONTROLS_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_range_controls_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_range_init->p_init_controls;
    attr_char_value.max_len = sizeof(ble_range_controls_t);

    return sd_ble_gatts_characteristic_add(p_range->service_handle,
        &char_md,
        &attr_char_value,
        &p_range->controls_handles);
}

/**@brief Function for adding status characteristic.
 *
 * @param[in] p_range       Range Service structure.
 * @param[in] p_range_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t status_char_add(ble_range_t *p_range, const ble_range_init_t *p_range_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_range->uuid_type;
    ble_uuid.uuid = BLE_UUID_RANGE_STATUS_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_range_status_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_range_init->p_init_status;
    attr_char_value.max_len = sizeof(ble_range_status_t);

    return sd_ble_gatts_characteristic_add(p_range->service_handle,
        &char_md,
        &attr_char_value,
        &p_range->status_handles);
}

void ble_range_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
    ble_range_t *p_range = (ble_range_t *)p_context;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_range, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_range, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_range, p_ble_evt);
        break;

    default:
        // No implementation needed.
        break;
    }
}

uint32_t ble_range_init(ble_range_t *p_range, const ble_range_init_t *p_range_init) {
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t base_uuid = RANGE_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_range);
    VERIFY_PARAM_NOT_NULL(p_range_init);

    // Initialize the service structure.
    p_range->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_range->evt_handler = p_range_init->evt_handler;
    p_range->is_inst_data_notif_enabled = false;
    p_range->is_avg_data_notif_enabled = false;
    p_range->is_status_notif_enabled = false;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_range->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_range->uuid_type;
    ble_uuid.uuid = BLE_UUID_RANGE_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_range->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add the inst data Characteristic.
    err_code = inst_data_char_add(p_range, p_range_init);
    VERIFY_SUCCESS(err_code);

    // Add the avg data Characteristic.
    err_code = avg_data_char_add(p_range, p_range_init);
    VERIFY_SUCCESS(err_code);

    // Add the period config Characteristic.
    err_code = period_config_char_add(p_range, p_range_init);
    VERIFY_SUCCESS(err_code);

    // Add the uwb config Characteristic.
    err_code = uwb_config_char_add(p_range, p_range_init);
    VERIFY_SUCCESS(err_code);

    // Add the mode config Characteristic.
    err_code = mode_config_char_add(p_range, p_range_init);
    VERIFY_SUCCESS(err_code);

    // Add the avg config Characteristic.
    err_code = avg_config_char_add(p_range, p_range_init);
    VERIFY_SUCCESS(err_code);

    // Add the ths config Characteristic.
    err_code = threshold_config_char_add(p_range, p_range_init);
    VERIFY_SUCCESS(err_code);

    // Add the controls Characteristic.
    err_code = controls_char_add(p_range, p_range_init);
    VERIFY_SUCCESS(err_code);

    // Add the status Characteristic.
    err_code = status_char_add(p_range, p_range_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t ble_range_inst_data_set(ble_range_t *p_range, ble_range_data_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_range_data_t);

    VERIFY_PARAM_NOT_NULL(p_range);

    if ((p_range->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_range->is_inst_data_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_RANGE_MAX_DATA_LEN) {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_range->inst_data_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_range->conn_handle, &hvx_params);
}

uint32_t ble_range_avg_data_set(ble_range_t *p_range, ble_range_data_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_range_data_t);

    VERIFY_PARAM_NOT_NULL(p_range);

    if ((p_range->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_range->is_avg_data_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_RANGE_MAX_DATA_LEN) {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_range->avg_data_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_range->conn_handle, &hvx_params);
}

uint32_t ble_range_status_set(ble_range_t *p_range, ble_range_status_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_range_status_t);

    VERIFY_PARAM_NOT_NULL(p_range);

    if ((p_range->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_range->is_status_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_RANGE_MAX_DATA_LEN) {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_range->status_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_range->conn_handle, &hvx_params);
}

#endif // USE_LEGACY_BLE_RANGE_SERVICE