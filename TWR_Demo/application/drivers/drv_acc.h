/******************************************************************************
 * @file    drv_acc.h
 * @author  Insight SiP
 * @brief   accelerometer driver header file.
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

#ifndef __DRV_ACC_H__
#define __DRV_ACC_H__

#include "drv_lis2de12.h"
#include "nrf_drv_twi.h"
#include <stddef.h>
#include <stdint.h>

#define DRV_ACC_PIN_NOT_USED 0xFF

/**@brief uwb event types.
 */
typedef enum {
    DRV_ACC_EVT_ERROR /**< Error */
} drv_acc_evt_type_t;

/**@brief uwb event struct.
 */
typedef struct
{
    drv_acc_evt_type_t type;
} drv_acc_evt_t;

/**@brief Motion driver event handler callback type.
 */
typedef void (*drv_acc_evt_handler_t)(drv_acc_evt_t const *p_evt);

/**@brief Initialization struct for motion driver.
 */
typedef struct
{
    uint8_t twi_addr;                      ///< TWI address.
    uint32_t pin_int1;                     ///< Interrupt pin 1.
    uint32_t pin_int2;                     ///< Interrupt pin 2.
    nrf_drv_twi_t const *p_twi_instance;   ///< The instance of TWI master to be used for transactions.
    nrf_drv_twi_config_t const *p_twi_cfg; ///< The TWI configuration to use while the driver is enabled.
    drv_acc_evt_handler_t evt_handler;     ///< Event handler - called after a pin interrupt has been detected.
} drv_acc_init_t;

/**@brief Function for initializing the Accelerometer driver.
 *
 * @param[in] evt_handler       acc param.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_acc_init(drv_acc_init_t *p_params);

/**@brief Function for enabling features in the Accelerometer driver.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_acc_enable(void);

/**@brief Function to disable features in the Accelerometer driver.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_acc_disable(void);

/**@brief Function for getting the acceleration raw data.
 *
 * @param[in] acc_value   acceleration in g
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_acc_get(float *acc_value);

#endif