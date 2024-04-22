/******************************************************************************
 * @file    drv_lis2de12.h
 * @author  Insight SiP
 * @brief   lis2de12 driver header file.
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

#ifndef __DRV_LISDDE12_H
#define __DRV_LISDDE12_H

#include "nrf_drv_twi.h"
#include <stdbool.h>
#include <stdint.h>

// I2C Address
#define LISDDE12_ACC_I2C_ADDR 0x18

#define WHO_AM_I_DEFAULT 0x33

// Accelerometer Register map
#define STATUS_REG_AUX 0x07  // Default:  Type: r
#define OUT_TEMP_L 0x0C      // Default:  Type: r
#define OUT_TEMP_H 0x0D      // Default:  Type: r
#define WHO_AM_I 0x0F        // Default:  Type: r
#define CTRL_REG0 0x1E       // Default:  Type: r/w
#define TEMP_CFG_REG 0x1F    // Default:  Type: r/w
#define CTRL_REG1 0x20       // Default:  Type: r/w
#define CTRL_REG2 0x21       // Default:  Type: r/w
#define CTRL_REG3 0x22       // Default:  Type: r/w
#define CTRL_REG4 0x23       // Default:  Type: r/w
#define CTRL_REG5 0x24       // Default:  Type: r/w
#define CTRL_REG6 0x25       // Default:  Type: r/w
#define REFERENCE 0x26       // Default:  Type: r/w
#define STATUS_REG 0x27      // Default:  Type: r
#define FIFO_READ_START 0x28 // Default:  Type: r
#define OUT_X_H 0x29         // Default:  Type: r
#define OUT_Y_H 0x2B         // Default:  Type: r
#define OUT_Z_H 0x2D         // Default:  Type: r
#define FIFO_CTRL_REG 0x2E   // Default:  Type: r/w
#define FIFO_SRC_REG 0x2F    // Default:  Type: r
#define INT1_CFG 0x30        // Default:  Type: r/w
#define INT1_SRC 0x31        // Default:  Type: r
#define INT1_THS 0x32        // Default:  Type: r/w
#define INT1_DURATION 0x33   // Default:  Type: r/w
#define INT2_CFG 0x34        // Default:  Type: r/w
#define INT2_SRC 0x35        // Default:  Type: r
#define INT2_THS 0x36        // Default:  Type: r/w
#define INT2_DURATION 0x37   // Default:  Type: r/w
#define CLICK_CFG 0x38       // Default:  Type: r/w
#define CLICK_SRC 0x39       // Default:  Type: r
#define CLICK_THS 0x3A       // Default:  Type: r/w
#define TIME_LIMIT 0x3B      // Default:  Type: r/w
#define TIME_LATENCY 0x3C    // Default:  Type: r/w
#define TIME_WINDOW 0x3D     // Default:  Type: r/w
#define ACT_THS 0x3E         // Default:  Type: r/w
#define ACT_DUR 0x3F         // Default:  Type: r/w

// Configuration Values

/* CTRL_REG0 Values */
#define CTRL_REG0_DEFAULT 0x10
#define BITS_SDO_PU_DISC (0x1 << 7) // Disconnect SDO/SA0 pull-up.

/* TEMP_CFG_REG Values */
#define TEMP_CFG_REG_DEFAULT 0x00
#define BITS_TEMP_EN (0x3 << 6) // Temperature sensor (T) enable.
#define BITS_TEMP_DIS (0)       // Temperature sensor (T) disable.

/* CTRL_REG1 Values */
#define CTRL_REG1_DEFAULT 0x07
#define BITS_XEN (0x1 << 0) // X-axis enable.
#define BITS_YEN (0x1 << 1) // Y-axis enable.
#define BITS_ZEN (0x1 << 2) // Z-axis enable.
#define BITS_LPEN (0x1 << 3)
#define BITS_ODR_PD (0x0 << 4) // Output Data rate bits [7:4]
#define BITS_ODR_1HZ (0x1 << 4)
#define BITS_ODR_10HZ (0x2 << 4)
#define BITS_ODR_25HZ (0x3 << 4)
#define BITS_ODR_50HZ (0x4 << 4)
#define BITS_ODR_100HZ (0x5 << 4)
#define BITS_ODR_200HZ (0x6 << 4)
#define BITS_ODR_400HZ (0x7 << 4)
#define BITS_ODR_1620HZ (0x8 << 4)
#define BITS_ODR_5376HZ (0x9 << 4)

/* CTRL_REG2 Values */
#define CTRL_REG2_DEFAULT 0x00
#define BITS_HP_IA1 (0x1 << 0)  // High-pass filter enabled for AOI function on Interrupt 1.
#define BITS_HP_IA2 (0x1 << 1)  // High-pass filter enabled for AOI function on Interrupt 2.
#define BITS_HPCLICK (0x1 << 2) // High-pass filter enabled for CLICK function.
#define BITS_FDS (0x1 << 3)     // Filtered data selection: data from internal filter sent to output register and FIFO
#define BITS_HPCF_0 (0x0 << 4)  // High-pass filter cutoff frequency selection.
#define BITS_HPCF_1 (0x1 << 4)
#define BITS_HPCF_2 (0x2 << 4)
#define BITS_HPCF_3 (0x3 << 4)
#define BITS_HPM_NORM_0 (0x0 << 5) // High-pass filter mode selection.
#define BITS_HPM_REF (0x1 << 5)
#define BITS_HPM_NORM_1 (0x2 << 5)
#define BITS_HPM_AUTTORESET (0x3 << 5)

/* CTRL_REG3 Values */
#define CTRL_REG3_DEFAULT 0x00
#define BITS_I1_OVERRUN (0x1 << 1) // FIFO overrun interrupt on INT1 pin.
#define BITS_I1_WTM (0x1 << 2)     // FIFO watermark interrupt on INT1 pin.
#define BITS_I1_ZYXDA (0x1 << 4)   // ZYXDA interrupt on INT1 pin.
#define BITS_I1_IA2 (0x1 << 5)     // IA2 interrupt on INT1 pin.
#define BITS_I1_IA1 (0x1 << 6)     // IA1 interrupt on INT1 pin.
#define BITS_I1_CLICK (0x1 << 7)   // CLICK interrupt on INT1 pin.

/* CTRL_REG4 Values */
#define CTRL_REG4_DEFAULT 0x00
#define BITS_SIM (0x1 << 0)     // SPI serial interface mode selection.
#define BITS_ST_NORM (0x0 << 1) // Self-test enable.
#define BITS_ST_0 (0x1 << 1)
#define BITS_ST_1 (0x2 << 1)
#define BITS_FS_2G (0x0 << 4) // Full-scale selection.
#define BITS_FS_4G (0x1 << 4)
#define BITS_FS_8G (0x2 << 4)
#define BITS_FS_16G (0x3 << 4)
#define BITS_BDU (0x1 << 7) // Block data update.

/* CTRL_REG5 Values */
#define CTRL_REG5_DEFAULT 0x00
#define BITS_D4D_INT2 (0x1 << 0) // 4D enable: 4D detection is enabled on INT2 pin.
#define BITS_LIR_INT2 (0x1 << 1) // Latch interrupt request on INT2_SRC register.
#define BITS_D4D_INT1 (0x1 << 2) // 4D enable: 4D detection is enabled on INT1 pin.
#define BITS_LIR_INT1 (0x1 << 3) // Latch interrupt request on INT1_SRC register.
#define BITS_FIFO_EN (0x1 << 6)  // FIFO enable.
#define BITS_BOOT (0x1 << 7)     // Reboot memory content.

/* REFERENCE Values */
#define REFERENCE_DEFAULT 0x00

/* FIFO_CTRL_REG Values */
#define FIFO_CTRL_REG_DEFAULT 0x00
#define BITS_TR (0x1 << 5)        // Trigger selection. 0=INT1, 1=INT2
#define BITS_FM_BYPASS (0x0 << 6) // FIFO mode selection.
#define BITS_FM_FIFO (0x1 << 6)
#define BITS_FM_STREAM (0x2 << 6)
#define BITS_FM_STREAM_TO_FIFO (0x3 << 6)

/* INT1_CFG Values */
/* INT2_CFG Values */
#define INT1_CFG_REG_DEFAULT 0x00
#define INT2_CFG_REG_DEFAULT 0x00
#define BITS_XLIE (0x1 << 0) // Enable interrupt generation on X low event or on direction recognition.
#define BITS_XHIE (0x1 << 1) // Enable interrupt generation on X high event or on direction recognition.
#define BITS_YLIE (0x1 << 2) // Enable interrupt generation on Y low event or on direction recognition.
#define BITS_YHIE (0x1 << 3) // Enable interrupt generation on Y high event or on direction recognition.
#define BITS_ZLIE (0x1 << 4) // Enable interrupt generation on Z low event or on direction recognition.
#define BITS_ZHIE (0x1 << 5) // Enable interrupt generation on Z high event or on direction recognition.
#define BITS_6D (0x1 << 6)   // 6-direction detection function enabled.
#define BITS_AOI (0x1 << 7)  // And/Or combination of interrupt events.

/* CLICK_CFG Values */
#define CLICK_REG_DEFAULT 0x00
#define BITS_CLICK_XS (0x1 << 0) // Enable interrupt single-click on X-axis.
#define BITS_CLICK_XD (0x1 << 1) // Enable interrupt double-click on X-axis.
#define BITS_CLICK_YS (0x1 << 2) // Enable interrupt single-click on Y-axis.
#define BITS_CLICK_YD (0x1 << 3) // Enable interrupt double-click on Y-axis.
#define BITS_CLICK_ZS (0x1 << 4) // Enable interrupt single-click on X-axis.
#define BITS_CLICK_ZD (0x1 << 5) // Enable interrupt double-click on X-axis.

typedef struct
{
    uint8_t ctrl_reg1;
    uint8_t ctrl_reg2;
    uint8_t ctrl_reg3;
    uint8_t ctrl_reg4;
    uint8_t ctrl_reg5;
    uint8_t ctrl_reg6;
    uint8_t temp_cfg_reg;
    uint8_t fifo_ctrl_reg;
    uint8_t int1_cfg;
    uint8_t int2_cfg;
    uint8_t click_cfg;
} drv_lis2de12_regval_cfg_t;

typedef struct
{
    bool ctrl_reg1;
    bool ctrl_reg2;
    bool ctrl_reg3;
    bool ctrl_reg4;
    bool ctrl_reg5;
    bool ctrl_reg6;
    bool temp_cfg_reg;
    bool fifo_ctrl_reg;
    bool int1_cfg;
    bool int2_cfg;
    bool click_cfg;
} drv_lis2de12_regselect_cfg_t;

/**@brief Configuration structure for lis2de12 driver.
 */
typedef struct
{
    drv_lis2de12_regval_cfg_t reg_vals;
    drv_lis2de12_regselect_cfg_t reg_selects;
} drv_lis2de12_cfg_t;

/**@brief Initialization struct for lis2de12 driver.
 */
typedef struct
{
    uint8_t twi_addr;                      ///< Accelerometer TWI address.
    uint32_t pin_int1;                     ///< Accelerometer Interrupt pin number.
    uint32_t pin_int2;                     ///< Magnetometer Interrupt pin number.
    nrf_drv_twi_t const *p_twi_instance;   ///< The instance of TWI master to be used for transactions.
    nrf_drv_twi_config_t const *p_twi_cfg; ///< The TWI configuration to use while the driver is enabled.
} drv_lis2de12_twi_cfg_t;

typedef struct drv_lis2de12_acc_data {
    int8_t x;
    int8_t y;
    int8_t z;
} drv_lis2de12_acc_data_t;

/**@brief Initialize the lis2de12 driver.
 */
uint32_t drv_lis2de12_init(void);

/**@brief Opens the lis2de12 driver according to the specified configuration.
 *
 * @param[in]   p_twi_cfg Pointer to the driver configuration for the session to be opened.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lis2de12_open(drv_lis2de12_twi_cfg_t const *const p_twi_cfg);

/**@brief Close the lis2de12 driver.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lis2de12_close(void);

/**@brief Read and check the IDs register of the lis2de12 sensor.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lis2de12_verify(void);

/**@brief Configures the lis2de12 sensor according to the specified configuration.
 *
 * @param[in]   p_cfg Pointer to the sensor configuration.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lis2de12_cfg_set(drv_lis2de12_cfg_t const *const p_cfg);

/**@brief Reads the configuration of the lis2de12 sensor.
 *
 * @param[in]   p_cfg Pointer to the driver configuration for the session to be opened.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lis2de12_cfg_get(drv_lis2de12_cfg_t *p_cfg);

/**@brief Enable temperature measurement
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lis2de12_temperature_meas_enable(void);

/**@brief Disable temperature measurement
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lis2de12_temperature_meas_disable(void);

/**@brief Get temperature. 
 *
 * @param[out] p_temperature Pointer to the temprature value.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lis2de12_temperature_get(int16_t *p_temperature);

/**@brief Get acceleration. 
 *
 * @param[out] p_acc Pointer to the acceleration value.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lis2de12_acceleration_get(int8_t *p_acc);

/**@brief Function to reboot memory content. 
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lis2de12_reboot(void);

#endif