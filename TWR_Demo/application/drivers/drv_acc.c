/******************************************************************************
 * @file    drv_acc.c
 * @author  Insight SiP
 * @brief   accelerometer driver implementation file.
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

#include "drv_acc.h"
#include "app_scheduler.h"
#include "drv_lis2de12.h"
#include "nrf_delay.h"
#include "sdk_macros.h"
#include <nrf_drv_gpiote.h>

#include "nrf_log.h"

#define RETURN_IF_INV_ERROR(PARAM) \
    if ((PARAM) != INV_SUCCESS) {  \
        return NRF_ERROR_INTERNAL; \
    }

#define ACC_SCALE_2G 0.015625f
#define ACC_SCALE_4G 0.031250f
#define ACC_SCALE_8G 0.062500f
#define ACC_SCALE_16G 0.125000f

/**@brief Motion configuration struct.
 */
typedef struct
{
    bool enabled;               ///< Driver enabled.
    drv_lis2de12_twi_cfg_t cfg; ///< TWI configuraion.
    drv_acc_evt_handler_t evt_handler;
} drv_acc_t;

/**@brief configuration.
 */
static drv_acc_t m_drv_acc;

/**@brief GPIOTE sceduled handler, executed in main-context.
 */
static void gpiote_evt_sceduled(void *p_event_data, uint16_t event_size) {
    drv_acc_evt_t evt;
//TODO manage interupts
    m_drv_acc.evt_handler(&evt);
}

/**@brief GPIOTE event handler, executed in interrupt-context.
 */
static void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    uint32_t err_code;

    if (pin == m_drv_acc.cfg.pin_int1) {
        err_code = app_sched_event_put(0, 0, gpiote_evt_sceduled);
        APP_ERROR_CHECK(err_code);
    }
    if (pin == m_drv_acc.cfg.pin_int2) {
        err_code = app_sched_event_put(0, 0, gpiote_evt_sceduled);
        APP_ERROR_CHECK(err_code);
    }
}

uint32_t drv_acc_init(drv_acc_init_t *p_params) {
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_params);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_instance);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_cfg);
    VERIFY_PARAM_NOT_NULL(p_params->evt_handler);

    m_drv_acc.evt_handler = p_params->evt_handler;
    m_drv_acc.cfg.twi_addr = p_params->twi_addr;
    m_drv_acc.cfg.pin_int1 = p_params->pin_int1;
    m_drv_acc.cfg.pin_int2 = p_params->pin_int2;
    m_drv_acc.cfg.p_twi_instance = p_params->p_twi_instance;
    m_drv_acc.cfg.p_twi_cfg = p_params->p_twi_cfg;
    m_drv_acc.enabled = false;

    // Configure interrupts
    if ((m_drv_acc.cfg.pin_int1 != DRV_ACC_PIN_NOT_USED) || (m_drv_acc.cfg.pin_int2 != DRV_ACC_PIN_NOT_USED)) {
        if (!nrfx_gpiote_is_init()) {
            err_code = nrfx_gpiote_init();
            VERIFY_SUCCESS(err_code);
        }
    }
    if (m_drv_acc.cfg.pin_int1 != DRV_ACC_PIN_NOT_USED) {
        nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
        in_config.pull = NRF_GPIO_PIN_PULLDOWN;
        err_code = nrf_drv_gpiote_in_init(m_drv_acc.cfg.pin_int1, &in_config, gpiote_evt_handler);
        VERIFY_SUCCESS(err_code);
    }
    if (m_drv_acc.cfg.pin_int2 != DRV_ACC_PIN_NOT_USED) {
        nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
        in_config.pull = NRF_GPIO_PIN_PULLDOWN;
        err_code = nrf_drv_gpiote_in_init(m_drv_acc.cfg.pin_int2, &in_config, gpiote_evt_handler);
        VERIFY_SUCCESS(err_code);
    }

    // Request twi bus
    err_code = drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_verify();
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_reboot();
    VERIFY_SUCCESS(err_code);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return err_code;
}

uint32_t drv_acc_enable(void) {
    uint32_t err_code;

    if (m_drv_acc.enabled) {
        return NRF_SUCCESS;
    }
    m_drv_acc.enabled = true;

    // Enable interrupt pins
    if (m_drv_acc.cfg.pin_int1 != DRV_ACC_PIN_NOT_USED)
        nrf_drv_gpiote_in_event_enable(m_drv_acc.cfg.pin_int1, true);
    if (m_drv_acc.cfg.pin_int2 != DRV_ACC_PIN_NOT_USED)
        nrf_drv_gpiote_in_event_enable(m_drv_acc.cfg.pin_int2, true);

    const drv_lis2de12_cfg_t lis2de12_cfg =
        {
            .reg_vals =
                {
                    .ctrl_reg1 = BITS_XEN | BITS_YEN | BITS_ZEN | BITS_LPEN | BITS_ODR_10HZ},
            .reg_selects =
                {
                    .ctrl_reg1 = true,
                    .ctrl_reg2 = false,
                    .ctrl_reg3 = false,
                    .ctrl_reg4 = false,
                    .ctrl_reg5 = false,
                    .ctrl_reg6 = false,
                    .temp_cfg_reg = false,
                    .fifo_ctrl_reg = false,
                    .int1_cfg = false,
                    .int2_cfg = false,
                    .click_cfg = false}};

    // Request twi bus
    err_code = drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_cfg_set(&lis2de12_cfg);
    VERIFY_SUCCESS(err_code);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_acc_disable(void) {
    uint32_t err_code = NRF_SUCCESS;

    m_drv_acc.enabled = false;

    // Disable interrupt pins
    if (m_drv_acc.cfg.pin_int1 != DRV_ACC_PIN_NOT_USED)
        nrf_drv_gpiote_in_event_disable(m_drv_acc.cfg.pin_int1);
    if (m_drv_acc.cfg.pin_int2 != DRV_ACC_PIN_NOT_USED)
        nrf_drv_gpiote_in_event_disable(m_drv_acc.cfg.pin_int2);

    const drv_lis2de12_cfg_t lis2de12_cfg =
        {
            .reg_vals =
                {
                    .ctrl_reg1 = CTRL_REG1_DEFAULT},
            .reg_selects =
                {
                    .ctrl_reg1 = true,
                    .ctrl_reg2 = false,
                    .ctrl_reg3 = false,
                    .ctrl_reg4 = false,
                    .ctrl_reg5 = false,
                    .ctrl_reg6 = false,
                    .temp_cfg_reg = false,
                    .fifo_ctrl_reg = false,
                    .int1_cfg = false,
                    .int2_cfg = false,
                    .click_cfg = false}};

    // Request twi bus
    err_code = drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_cfg_set(&lis2de12_cfg);
    VERIFY_SUCCESS(err_code);

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return err_code;
}

uint32_t drv_acc_get(float *p_acc) {
    uint32_t err_code = NRF_SUCCESS;
    int8_t raw_val[3];

    VERIFY_PARAM_NOT_NULL(p_acc);

    // Request twi bus
    drv_lis2de12_open(&m_drv_acc.cfg);
    VERIFY_SUCCESS(err_code);

    err_code = drv_lis2de12_acceleration_get(raw_val);
    VERIFY_SUCCESS(err_code);

    // Currently we use the accelerometer with +/-2g
    // TODO manage automatically full scale selection
    p_acc[0] = (float)(raw_val[0]) * ACC_SCALE_2G;
    p_acc[1] = (float)(raw_val[1]) * ACC_SCALE_2G;
    p_acc[2] = (float)(raw_val[2]) * ACC_SCALE_2G;

    // Release twi bus
    err_code = drv_lis2de12_close();
    VERIFY_SUCCESS(err_code);

    return err_code;
}