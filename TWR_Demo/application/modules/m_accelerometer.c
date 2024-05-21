/******************************************************************************
 * @file    m_accelerometer.c
 * @author  Insight SiP
 * @brief   Accelerometer module implementation file.
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
#include "boards.h"
#include "lis2de12_reg.h"
#include "m_accelerometer.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "sdk_config.h"

#define NRF_LOG_MODULE_NAME m_accelerometer
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

// TWI instance ID.
#define TWI_INSTANCE_ID 1

// TWI instance.
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

// Forward declarations
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);

static const nrf_drv_twi_config_t twi_config = {
    .scl = PIN_LIS2DE12_SCL,
    .sda = PIN_LIS2DE12_SDA,
    .frequency = NRF_DRV_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    .clear_bus_init = false};
static stmdev_ctx_t dev_ctx = {
    .write_reg = platform_write,
    .read_reg = platform_read,
    .mdelay = platform_delay,
    .handle = NULL};

/**@brief GPIOTE event handler, executed in interrupt-context.
 */
static void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    //TODO
}

uint32_t m_accelerometer_init(void) {
    uint32_t err_code;
    uint8_t who_am_i;

    // Power up lis2de12
    nrf_gpio_pin_set(PIN_LIS2DE12_EN);
    nrf_gpio_cfg_output(PIN_LIS2DE12_EN);
    nrf_delay_ms(5);

    // Configure interrupts
    if (!nrfx_gpiote_is_init()) {
        err_code = nrfx_gpiote_init();
        VERIFY_SUCCESS(err_code);
    }

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;
    err_code = nrf_drv_gpiote_in_init(PIN_LIS2DE12_INT1, &in_config, gpiote_evt_handler);
    VERIFY_SUCCESS(err_code);

    in_config.pull = NRF_GPIO_PIN_PULLDOWN;
    err_code = nrf_drv_gpiote_in_init(PIN_LIS2DE12_INT2, &in_config, gpiote_evt_handler);
    VERIFY_SUCCESS(err_code);

    // Initialize TWI
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    VERIFY_SUCCESS(err_code);

    // Enable TWI
    nrf_drv_twi_enable(&m_twi);

    // Check device ID
    lis2de12_device_id_get(&dev_ctx, &who_am_i);

    // Disable TWI
    nrf_drv_twi_disable(&m_twi);

    if (who_am_i != LIS2DE12_ID) {
        return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
}

uint32_t m_accelerometer_start(void) {
    uint32_t err_code;

    // Enable TWI
    nrf_drv_twi_enable(&m_twi);

    /* Enable Block Data Update. */
    lis2de12_block_data_update_set(&dev_ctx, PROPERTY_DISABLE);
    // Set Output Data Rate to 1Hz
    lis2de12_data_rate_set(&dev_ctx, LIS2DE12_ODR_25Hz);
    // Set full scale to 2g
    lis2de12_full_scale_set(&dev_ctx, LIS2DE12_2g);

    return NRF_SUCCESS;
}

uint32_t m_accelerometer_stop(void) {
    uint32_t err_code;

    // Set Output Data Rate to PD
    lis2de12_data_rate_set(&dev_ctx, LIS2DE12_POWER_DOWN);

    // disable TWI
    nrf_drv_twi_disable(&m_twi);

    return NRF_SUCCESS;
}

uint32_t m_accelerometer_acc_get(float *acceleration_mg, uint8_t *data_available) {
#if defined(FIFO_MODE)
    lis2de12_reg_t reg;
    int16_t data_raw_acceleration[3];
    // Read output only if new value available
    lis2de12_xl_data_ready_get(&dev_ctx, &reg.byte);

    if (reg.byte) {
        // Read accelerometer data
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        lis2de12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
        acceleration_mg[0] = lis2de12_from_fs2_to_mg(data_raw_acceleration[0]);
        acceleration_mg[1] = lis2de12_from_fs2_to_mg(data_raw_acceleration[1]);
        acceleration_mg[2] = lis2de12_from_fs2_to_mg(data_raw_acceleration[2]);
        *data_available = 1;
    } else {
        *data_available = 0;
    }
#else // BYPASS
    // Here we read OUT_X_H, OUT_Y_H and OUT_Z_H registers, there are only 1 Byte (contrary to values in the FIFO) 
    int8_t data_raw_acceleration[3];
    platform_read(NULL, LIS2DE12_OUT_X_H, (int8_t*) &data_raw_acceleration[0], 1);
    platform_read(NULL, LIS2DE12_OUT_Y_H, (int8_t*) &data_raw_acceleration[1], 1);
    platform_read(NULL, LIS2DE12_OUT_Z_H, (int8_t*) &data_raw_acceleration[2], 1);

    // Assuming sensor is set with a scale of +/-2g, we need to multiply raw value by:
    // 4/(2^8) = 4/256 = 0.015625
    acceleration_mg[0] = data_raw_acceleration[0]*0.015625f;
    acceleration_mg[1] = data_raw_acceleration[1]*0.015625f;
    acceleration_mg[2] = data_raw_acceleration[2]*0.015625f;

    *data_available = 1;
#endif

    return NRF_SUCCESS;
}

uint32_t m_accelerometer_temp_get(float *temperature_degC, uint8_t *data_available) {
    lis2de12_reg_t reg;
    int16_t data_raw_temperature;

    lis2de12_temp_data_ready_get(&dev_ctx, &reg.byte);

    if (reg.byte) {
        // Read temperature data
        memset(&data_raw_temperature, 0x00, sizeof(int16_t));
        lis2de12_temperature_raw_get(&dev_ctx, &data_raw_temperature);
        *temperature_degC = lis2de12_from_lsb_to_celsius(data_raw_temperature);
    } else {
        *data_available = 0;
    }

    return NRF_SUCCESS;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    // Assuming data length will never exceed  128 Bytes
    uint8_t buffer[128] = {0};
    buffer[0] = reg;
    for (int i = 0; i < len; i++)
        buffer[i + 1] = bufp[i];

    nrf_drv_twi_tx(&m_twi, LIS2DE12_I2C_ADD_L >> 1, buffer, len+1, false);

    return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    nrf_drv_twi_tx(&m_twi, LIS2DE12_I2C_ADD_L >> 1, &reg, 1, true);
    nrf_drv_twi_rx(&m_twi, LIS2DE12_I2C_ADD_L >> 1, bufp, len);

    return 0;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms) {
    nrf_delay_ms(ms);
}