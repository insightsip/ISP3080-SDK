/******************************************************************************
 * @file    m_uwb_range.h
 * @author  Insight SiP
 * @brief   Range module header file.
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

#ifndef __M_UWB_RANGE_H__
#define __M_UWB_RANGE_H__

#include "drv_uwb_range.h"
#include "m_ble_mgmt.h"
#include <stdint.h>

typedef drv_uwb_range_role_t m_uwb_range_role_t;

/** @brief Init parameters for m_uwb_range_init.
 */
typedef struct
{
    m_uwb_range_role_t role; /**< Role of the device. */
} m_uwb_range_init_t;

/**@brief Function for starting the uwb module.
 *
 * @details This function should be called after m_uwb_init to start the uwb module.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
uint32_t m_range_start(void);

/**@brief Function for stopping the uwb module.
 *
 * @details This function should be called after m_uwb_start to stop the uwb module.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
uint32_t m_range_stop(void);

/**@brief Function for initializing the uwb module.
 *
 * @param[in] p_params    Pointer to the init parameters.
 * @param[out] p_handle    Pointer to the service handle.
 */
uint32_t m_range_init(m_uwb_range_init_t *p_params, m_ble_service_handle_t *p_handle);

/**@brief Function for getting uuid.
 *
 * @param[in] p_params    Pointer to the uuid.
 */
uint32_t m_range_ble_uuid_get(uint16_t *uuid);

#endif