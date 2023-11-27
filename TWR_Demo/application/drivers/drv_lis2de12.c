/******************************************************************************
 * @file    drv_lis2de12.c
 * @author  Insight SiP
 * @brief   lis2de12 driver implementation file.
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

#include "drv_lis2de12.h"
#include "nrf_delay.h"
#include "sdk_macros.h"
#include "twi_manager.h"

/**@brief Check if the driver is open, if not return NRF_ERROR_INVALID_STATE.
 */
#define DRV_CFG_CHECK(PARAM)            \
    if ((PARAM) == NULL) {              \
        return NRF_ERROR_INVALID_STATE; \
    }

/**@brief TWI configuration.
 */
static struct
{
    drv_lis2de12_twi_cfg_t const *p_cfg;
} m_lis2de12;

/**@brief Open the TWI bus for communication.
 */
static __inline uint32_t twi_open(void) {
    uint32_t err_code;

    err_code = twi_manager_request(m_lis2de12.p_cfg->p_twi_instance,
        m_lis2de12.p_cfg->p_twi_cfg,
        NULL,
        NULL);
    VERIFY_SUCCESS(err_code);

    nrf_drv_twi_enable(m_lis2de12.p_cfg->p_twi_instance);

    return NRF_SUCCESS;
}

/**@brief Function to deinit the TWI module when this driver does not need to
 *        communicate on the TWI bus, so that other drivers can use the module.
 */
static __inline uint32_t twi_close(void) {
    nrf_drv_twi_disable(m_lis2de12.p_cfg->p_twi_instance);
    nrf_drv_twi_uninit(m_lis2de12.p_cfg->p_twi_instance);

    return NRF_SUCCESS;
}

/**@brief Function for reading a register.
 *
 * @param[in]  reg_addr            Address of the register to read.
 * @param[out] p_reg_val           Pointer to a buffer to receive the read value.
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t reg_read(uint8_t reg_addr, uint8_t *p_reg_val) {
    uint32_t err_code;

    err_code = nrf_drv_twi_tx(m_lis2de12.p_cfg->p_twi_instance,
        m_lis2de12.p_cfg->twi_addr,
        &reg_addr,
        1,
        true);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_twi_rx(m_lis2de12.p_cfg->p_twi_instance,
        m_lis2de12.p_cfg->twi_addr,
        p_reg_val,
        1);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for writing to a register.
 *
 * @param[in]  reg_addr            Address of the register to write to.
 * @param[in]  reg_val             Value to write to the register.
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t reg_write(uint8_t reg_addr, uint8_t reg_val) {
    uint32_t err_code;

    uint8_t buffer[2] = {reg_addr, reg_val};

    err_code = nrf_drv_twi_tx(m_lis2de12.p_cfg->p_twi_instance,
        m_lis2de12.p_cfg->twi_addr,
        buffer,
        2,
        false);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_lis2de12_init(void) {
    m_lis2de12.p_cfg = NULL;

    return NRF_SUCCESS;
}

uint32_t drv_lis2de12_open(drv_lis2de12_twi_cfg_t const *const p_cfg) {
    m_lis2de12.p_cfg = p_cfg;

    return twi_open();
}

uint32_t drv_lis2de12_close(void) {
    uint32_t err_code = twi_close();

    m_lis2de12.p_cfg = NULL;

    return err_code;
}

uint32_t drv_lis2de12_cfg_set(drv_lis2de12_cfg_t const *const p_cfg) {
    uint32_t err_code;

    if (p_cfg->reg_selects.ctrl_reg1) {
        err_code = reg_write(CTRL_REG1, p_cfg->reg_vals.ctrl_reg1);
        VERIFY_SUCCESS(err_code);
    }
    if (p_cfg->reg_selects.ctrl_reg2) {
        err_code = reg_write(CTRL_REG2, p_cfg->reg_vals.ctrl_reg2);
        VERIFY_SUCCESS(err_code);
    }
    if (p_cfg->reg_selects.ctrl_reg3) {
        err_code = reg_write(CTRL_REG3, p_cfg->reg_vals.ctrl_reg3);
        VERIFY_SUCCESS(err_code);
    }
    if (p_cfg->reg_selects.ctrl_reg4) {
        err_code = reg_write(CTRL_REG4, p_cfg->reg_vals.ctrl_reg4);
        VERIFY_SUCCESS(err_code);
    }
    if (p_cfg->reg_selects.ctrl_reg5) {
        err_code = reg_write(CTRL_REG5, p_cfg->reg_vals.ctrl_reg5);
        VERIFY_SUCCESS(err_code);
    }
    if (p_cfg->reg_selects.ctrl_reg6) {
        err_code = reg_write(CTRL_REG6, p_cfg->reg_vals.ctrl_reg6);
        VERIFY_SUCCESS(err_code);
    }
    if (p_cfg->reg_selects.click_cfg) {
        err_code = reg_write(CLICK_CFG, p_cfg->reg_vals.click_cfg);
        VERIFY_SUCCESS(err_code);
    }
    if (p_cfg->reg_selects.int1_cfg) {
        err_code = reg_write(INT1_CFG, p_cfg->reg_vals.int1_cfg);
        VERIFY_SUCCESS(err_code);
    }
    if (p_cfg->reg_selects.int2_cfg) {
        err_code = reg_write(INT2_CFG, p_cfg->reg_vals.int2_cfg);
        VERIFY_SUCCESS(err_code);
    }
    if (p_cfg->reg_selects.temp_cfg_reg) {
        err_code = reg_write(TEMP_CFG_REG, p_cfg->reg_vals.temp_cfg_reg);
        VERIFY_SUCCESS(err_code);
    }

    return NRF_SUCCESS;
}

uint32_t drv_lis2de12_cfg_get(drv_lis2de12_cfg_t *p_cfg) {
    //TO BE IMPLEMENTED

    return NRF_SUCCESS;
}

uint32_t drv_lis2de12_verify(void) {
    uint32_t err_code;
    uint8_t reg_val;
    bool found = false;

    DRV_CFG_CHECK(m_lis2de12.p_cfg);

    // Verify device
    err_code = reg_read(WHO_AM_I, &reg_val);
    VERIFY_SUCCESS(err_code);

    found = (reg_val == WHO_AM_I_DEFAULT) ? NRF_SUCCESS : NRF_ERROR_NOT_FOUND;

    return found;
}

uint32_t drv_lis2de12_temperature_meas_enable(void) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_lis2de12.p_cfg);

    err_code = reg_write(TEMP_CFG_REG, BITS_TEMP_EN);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_lis2de12_temperature_meas_disable(void) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_lis2de12.p_cfg);

    err_code = reg_write(TEMP_CFG_REG, BITS_TEMP_DIS);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


uint32_t drv_lis2de12_temperature_get(int16_t *p_temperature) {
    uint32_t err_code;
    uint8_t buff[2];

    DRV_CFG_CHECK(m_lis2de12.p_cfg);
    VERIFY_PARAM_NOT_NULL(p_temperature);

    err_code = reg_read(OUT_TEMP_L, buff);
    VERIFY_SUCCESS(err_code);

    err_code = reg_read(OUT_TEMP_H, buff+1);
    VERIFY_SUCCESS(err_code);

    *p_temperature = (int16_t)buff[1];
    *p_temperature = (*p_temperature * 256) + (int16_t)buff[0];

    return NRF_SUCCESS;
}

uint32_t drv_lis2de12_acceleration_get(int8_t *p_acc) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_lis2de12.p_cfg);
    VERIFY_PARAM_NOT_NULL(p_acc);

    // Fetch X
    err_code = reg_read(OUT_X_H, p_acc);
    VERIFY_SUCCESS(err_code);

    // Fetch Y
    err_code = reg_read(OUT_Y_H, p_acc + 1);
    VERIFY_SUCCESS(err_code);

    // Fetch Z
    err_code = reg_read(OUT_Z_H, p_acc + 2);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_lis2de12_reboot(void) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_lis2de12.p_cfg);

    err_code = reg_write(CTRL_REG5, BITS_BOOT);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}