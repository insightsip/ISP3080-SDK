/******************************************************************************
 * @file    m_accelerometer.h
 * @author  Insight SiP
 * @brief   Accelerometer module header file.
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

#ifndef __M_ACCELEROMETER_H__
#define __M_ACCELEROMETER_H__

#include <stdbool.h>
#include <stdint.h>


/**@brief Initalizes the accelerometer driver.
 *
 * @param[in]  p_batt_meas_init     Struct containing the configuration parameters.
 * @param[out] p_handle    Pointer to the service handle.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
uint32_t m_accelerometer_init(void);

/**@brief Start accelerometer activity detection
 *
 * @retval NRF_SUCCESS If successful.
 */
uint32_t m_accelerometer_activity_detection_start(void);

/**@brief Stop accelerometer measurements
 *
 * @retval NRF_SUCCESS If successful.
 */
uint32_t m_accelerometer_stop(void);

/**@brief Get accelerometer sample
 *
 * @param[out] acceleration_mg  accelerometer result in mg
 * @param[out] data_available   indicate if new data is avalaible (only for FIFO mode)
 *
 * @retval NRF_SUCCESS If successful.
 */
uint32_t m_accelerometer_acc_get(float *acceleration_mg, uint8_t *data_available);

/**@brief Get temperature sample
 *
 * @param[out] acceleration_mg  temperature result in °C
 * @param[out] data_available   indicate if new data is avalaible (only for FIFO mode)
 *
 * @retval NRF_SUCCESS If successful.
 */
uint32_t m_accelerometer_temp_get(float *temperature_degC , uint8_t *data_available);

#endif